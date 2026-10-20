#include "../../tools/deployment/snowbot_vendor/position_contract.h"
#include <iostream>
#include <limits>
#include <stdexcept>
#ifdef BINDU_VENDOR_RUNIN
#include "JntController/runinController.h"
#include "controller/TopicJointState.h"
#endif

static int checks = 0;
static void check(bool value, const char* name) {
    if (!value) throw std::runtime_error(name);
    ++checks;
}

static int run_checks() {
    using namespace bindu_position;
    check(interpolation_steps(.02, .001, 500) == 20, "20 ms must not become 40 steps");
    check(interpolation_steps(.01, .001, 500) == 10, "10 ms window");
    check(interpolation_steps(.04, .001, 500) == 40, "40 ms window");
    check(interpolation_steps(0., .001, 500) == 20, "first callback fallback");
    check(interpolation_steps(-1., .001, 500) == 20, "clock regression fallback");
    check(interpolation_steps(1e100, .001, 500) == 500, "idle clamp before integer conversion");
    check(interpolation_steps(.0006, .001, 500) == 2, "minimum window");
    check(interpolation_steps(std::numeric_limits<double>::quiet_NaN(), .001, 500) == 20,
          "nonfinite interval fallback");

    TopicCapture gate;
    check(!gate.fresh(100., 10.), "no fabricated initial measurements");
    gate.observe(100., 100.01, 10., true);
    check(gate.fresh(100.01, 10.), "fresh source and receipt");
    check(gate.needs_seed(10., false), "first takeover needs seed");
    gate.accept(10.);
    check(!gate.needs_seed(10.02, false), "normal update retains follower state");
    check(gate.needs_seed(10.02, true), "publisher restart needs seed without a long gap");
    check(gate.needs_seed(10.3, false), "stream recovery needs seed");
    check(!gate.fresh(100.31, 10.01), "source-age budget");
    check(!gate.fresh(100.01, 10.3), "frozen ROS clock cannot extend receipt budget");
    gate.observe(100., 100.02, 10.02, true);
    check(!gate.fresh(100.02, 10.02), "duplicate samples cannot renew capture");
    gate.observe(100.03, 100.02, 10.03, true);
    check(!gate.fresh(100.02, 10.03), "future sample rejected");
    gate.observe(100.04, 100.04, 10.04, false);
    check(!gate.fresh(100.04, 10.04), "nonfinite joints rejected");
    gate.observe(100.05, 100.05, 10.05, true);
    check(gate.fresh(100.05, 10.05), "new valid measurement recovers capture");
    gate.reset();
    check(gate.needs_seed(10.05, false), "mode change needs new capture");

#ifdef BINDU_VENDOR_RUNIN
    // Execute the actual reviewed vendor kernel, rather than duplicating it.
    runin_control servo;
    const std::vector<double> zero(7, 0.), gain(7, 10.);
    auto hold = [&](double left, double right) {
        const std::vector<double> l(7, left), r(7, right);
        servo.init(l, r, 20);
        servo.ModelRunin(l, zero, zero, zero, zero, r, zero, zero, zero, zero,
                         gain, gain, gain, gain, 1, 20);
        for (int tick = 0; tick < 60; ++tick) {
            const auto result = servo.dataOutp(true);
            for (int axis = 0; axis < 7; ++axis) {
                check(std::abs(result.runinData.lArm_posDot[axis]-left) < 1e-10,
                      "nonzero left capture must not dip toward zero");
                check(std::abs(result.runinData.rArm_posDot[axis]-right) < 1e-10,
                      "nonzero right capture must not dip toward zero");
            }
        }
    };
    hold(.6, -.7);
    hold(-.4, .3);  // Re-capture clears the previous motion/filter history.
    TopicJointState middle;
    middle.position = middle.target = .6;
    middle.step(.02);
    check(std::abs(middle.position-.6) < 1e-12, "middle nonzero hold");
    middle.target = .7;
    middle.step(.02);
    const double previous = middle.position;
    middle.target = .8;
    middle.step(.02);
    check(middle.position > previous && middle.position < .8,
          "new target continues from emitted state");
#endif
    std::cout << checks << " position contract checks passed\n";
    return 0;
}

int main() {
    try { return run_checks(); }
    catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
