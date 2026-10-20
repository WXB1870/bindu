// Runs the patched driver's actual PDO cycle against two in-memory fake motors.
// No SOEM master/socket initialization or hardware thread is called or linked.
#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>
#include <cassert>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <iostream>
#define private public
#include "servo_control/ethercat_driver.h"
#undef private

static int fake_wkc = 40;
extern "C" int ec_send_processdata() { return 1; }
extern "C" int ec_receive_processdata(int) { return fake_wkc; }

static int64_t now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

int main() {
    BinduWatchdog watch;
    assert(!watch.blocked(1, false)); // Disabled legacy mode remains unchanged.
    watch.enable(); assert(watch.blocked(1, true));
    assert(watch.heartbeat(123, 1, 100)); assert(!watch.blocked(101, true));
    assert(!watch.heartbeat(456, 2, 102)); // A new process cannot borrow an old session.
    assert(!watch.heartbeat(123, 1, 102)); // A replay cannot renew time.
    assert(!watch.heartbeat(123, 2, 100+BinduWatchdog::timeout_ns));
    assert(watch.latched()); assert(!watch.heartbeat(123, 3, 1000000000));
    BinduWatchdog fault; fault.enable(); assert(fault.heartbeat(123, 1, 1));
    assert(fault.blocked(2, false)); assert(fault.latched());

    BinduGripperWatchdog grip;
    assert(grip.blocked(1, true));
    assert(grip.accept(1, 100, 200, false)); assert(!grip.blocked(101, true));
    assert(!grip.accept(1, 102, 200, false)); // no replay renewal
    assert(grip.blocked(300, true)); assert(grip.latched());
    assert(!grip.accept(2, 301, 200, true)); // late hold cannot erase timeout
    BinduGripperWatchdog held;
    assert(held.accept(1, 100, 200, true)); assert(held.blocked(10000, true));
    assert(!held.latched()); assert(held.accept(2, 10001, 200, false));
    assert(!held.blocked(10002, true));

    // Leak this tiny fixture until process exit to exclude real stop()/ec_close().
    auto* driver = new EtherCATDriver();
    std::memset(driver->csp_rx_, 0, sizeof(driver->csp_rx_));
    std::memset(driver->csp_tx_, 0, sizeof(driver->csp_tx_));
    motor_rx_pdo_t arm_rx{}, gripper_rx{};
    motor_tx_pdo_t arm_tx{}, gripper_tx{};
    csp_rx_t head_rx{}; csp_tx_t head_tx{};
    driver->m_rx_[0] = &arm_rx; driver->m_tx_[0] = &arm_tx;
    driver->m_rx_[7] = &gripper_rx; driver->m_tx_[7] = &gripper_tx;
    driver->csp_rx_[16] = &head_rx; driver->csp_tx_[16] = &head_tx;
    driver->is_csp_motor_[16] = true;
    driver->inOP_ = true; driver->expectedWKC_ = 40; driver->wkc_ = 40;
    driver->setSingleMotorEnable(0, true); driver->setSingleMotorEnable(7, true);
    driver->setSingleMotorEnable(16, true);
    arm_tx.actual_position = 1.25f; arm_tx.statusword = 0x27;
    gripper_tx.actual_position = .5f;
    head_tx.position = 5000; head_tx.statusword = 0x27;
    driver->setMotorCmd(0, 9.f, 2.f, 4.f, 8.f, 3.f);
    driver->setMotorCmd(7, .8f, .1f, 0.f, 2.f, 1.f);
    for (int i = 0; i < 40; ++i) driver->cycle();
    assert(arm_rx.target_position == 9.f); // Guard default does not change legacy.
    driver->enableBinduGuard();
    driver->cycle();
    assert(arm_rx.target_position == 1.25f && arm_rx.target_speed == 0.f);
    assert(gripper_rx.target_position == .8f); // J8 is never captured or driven.
    assert(head_rx.position == 5000 && head_rx.velocity == 0);
    assert(driver->binduHeartbeat(123, 1, now_ns()));
    driver->cycle(); assert(arm_rx.target_position == 9.f);
    // Simulate a frozen ROS thread while the independent RT thread still runs.
    driver->bindu_guard_.last_ns_ = now_ns()-BinduWatchdog::timeout_ns-1;
    driver->cycle();
    assert(driver->bindu_guard_.latched());
    assert(arm_rx.target_position == 1.25f && arm_rx.target_speed == 0.f);
    assert(arm_rx.KP == 8.f && arm_rx.KD == 3.f && arm_rx.target_torque == 4.f);
    arm_tx.actual_position = 2.f;
    driver->setMotorCmd(0, -8.f, 4.f, 9.f, 99.f, 10.f);
    driver->cycle();
    assert(arm_rx.target_position == 1.25f && arm_rx.KP == 8.f && arm_rx.target_torque == 4.f);
    assert(!driver->binduHeartbeat(123, 2, now_ns())); // No automatic resume.
    const auto sample = driver->completed_pdo_cycles_.load();
    const auto good = driver->last_good_pdo_ns_.load();
    fake_wkc = 39; arm_tx.actual_position = 12.f; driver->cycle();
    assert(driver->completed_pdo_cycles_.load() == sample);
    assert(driver->last_good_pdo_ns_.load() == good);
    assert(driver->motor_fb_pos_[0].load() == 2.f);
    driver->motor_fb_status_[0] = 8;
    driver->cycle();
    assert(arm_rx.controlword == 0x0006); // A latched fault cannot auto-reset.
    const auto health = driver->binduHealthJson(10., true);
    assert(health.find("\"guard_latched\":true") != std::string::npos);
    fake_wkc = 40;
    arm_tx.statusword = 0x27;
    std::thread pdo([&]() { for (int i = 0; i < 200000; ++i) driver->cycle(); });
    const std::string key = "\"sample_age\":";
    for (int i = 0; i < 50000; ++i) {
        const auto report = driver->binduHealthJson(10., true);
        const auto offset = report.find(key);
        assert(offset != std::string::npos);
        assert(std::strtod(report.data()+offset+key.size(), nullptr) >= 0.);
    }
    pdo.join();
    // An opt-in gripper has its own deadline even with a live arm heartbeat.
    auto* gripper_driver = new EtherCATDriver();
    std::memset(gripper_driver->csp_rx_, 0, sizeof(gripper_driver->csp_rx_));
    std::memset(gripper_driver->csp_tx_, 0, sizeof(gripper_driver->csp_tx_));
    motor_rx_pdo_t protected_rx{}; motor_tx_pdo_t protected_tx{};
    protected_tx.actual_position = -.25f; protected_tx.statusword = 0x27;
    gripper_driver->m_rx_[7] = &protected_rx; gripper_driver->m_tx_[7] = &protected_tx;
    gripper_driver->inOP_ = true; gripper_driver->expectedWKC_ = 40; gripper_driver->wkc_ = 40;
    gripper_driver->setSingleMotorEnable(7, true);
    for (int i = 0; i < 40; ++i) gripper_driver->cycle();
    gripper_driver->enableBinduGuard(); gripper_driver->enableBinduGrippers();
    assert(gripper_driver->binduHeartbeat(456, 1, now_ns()));
    assert(!gripper_driver->binduGripperCommand(0, 123, 1, now_ns(), 200000000, false));
    assert(gripper_driver->binduGripperCommand(0, 456, 1, now_ns(), 200000000, false));
    gripper_driver->setMotorCmd(7, -.5f, 0.f, 0.f, 100.f, 30.f);
    gripper_driver->cycle(); assert(protected_rx.target_position == -.5f);
    gripper_driver->bindu_gripper_guard_[0].expires_ = now_ns()-1;
    gripper_driver->cycle();
    assert(gripper_driver->bindu_gripper_guard_[0].latched());
    assert(protected_rx.target_position == -.25f && protected_rx.target_speed == 0.f);
    protected_tx.actual_position = -.3f;
    gripper_driver->setMotorCmd(7, .4f, 1.f, 2.f, 999.f, 20.f);
    gripper_driver->cycle();
    assert(protected_rx.target_position == -.25f && protected_rx.KP == 100.f);
    assert(!gripper_driver->binduGripperCommand(0, 456, 2, now_ns(), 200000000, true));
    assert(gripper_driver->binduHealthJson(10., true).find("\"gripper_latched_mask\":1") != std::string::npos);
    std::cout << "PASS: replay, exact timeout, bus fault, independent RT hold, frozen hold, J8 isolation, valid PDO progress\n";
}
