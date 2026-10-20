#include "feedback_snapshot.h"
#include <cassert>
#include <iostream>
#include <thread>

struct Message {
    std::array<double, 8> l_position{}, l_speed{}, l_iq{}, r_position{}, r_speed{}, r_iq{};
    std::array<double, 2> head_position{}, head_speed{}, torso_position{}, torso_speed{};
    std::array<uint16_t, 20> feedback_status{};
    uint64_t pdo_sample = 0;
    double pdo_age = -1;
    uint32_t feedback_valid_mask = 0;
    int pdo_wkc = 0, pdo_expected_wkc = 0;
};

int main() {
    using namespace bindu_feedback;
    Cache cache;
    Snapshot input, output;
    Message msg;
    assert(cache.read(output));
    fill_message(output, 100, msg);
    assert(msg.pdo_sample == 0 && msg.feedback_valid_mask == 0);
    assert(msg.pdo_age == -1 && std::isnan(msg.head_position[0]));
    input.wkc = input.expected_wkc = 42;
    input.valid_mask = 0xfffff;
    for (unsigned i = 0; i < axes; ++i) {
        input.position[i] = float(i) + .25f;
        input.speed[i] = -float(i) - .5f;
        input.current[i] = float(i);
        input.status[i] = uint16_t(100+i);
    }
    cache.update(input, 1000000000);
    assert(cache.read(output));
    fill_message(output, 1001000000, msg);
    assert(msg.pdo_sample == 1 && msg.feedback_valid_mask == 0xfffff);
    assert(std::abs(msg.pdo_age-.001) < 1e-12);
    for (unsigned i = 0; i < 8; ++i) {
        assert(msg.l_position[i] == input.position[i]);
        assert(msg.r_position[i] == input.position[8+i]);
        assert(msg.l_speed[i] == input.speed[i]);
        assert(msg.r_speed[i] == input.speed[8+i]);
    }
    for (unsigned i = 0; i < 2; ++i) {
        assert(msg.head_position[i] == input.position[16+i]);
        assert(msg.torso_position[i] == input.position[18+i]);
        assert(msg.head_speed[i] == input.speed[16+i]);
        assert(msg.torso_speed[i] == input.speed[18+i]);
    }
    assert(msg.feedback_status[19] == 119);
    assert(std::abs(csp_radians(65536) - 6.283185307179586) < 1e-6);
    assert(std::abs(csp_radians(-32768) + 3.141592653589793) < 1e-6);
    assert(csp_radians(0) == 0);
    input.wkc = 41;
    input.position.fill(1234); // Partial/failed receive must never become a new sample.
    cache.update(input, 1002000000);
    assert(cache.read(output));
    fill_message(output, 1003000000, msg);
    assert(msg.pdo_sample == 1 && output.sampled_ns == 1000000000);
    assert(output.position[0] == .25f && msg.feedback_valid_mask == 0);
    assert(std::isnan(msg.l_position[0]) && std::isnan(msg.torso_position[0]));
    input.wkc = 42;
    input.valid_mask &= ~(1u << 16);
    input.position[18] = std::numeric_limits<float>::infinity();
    cache.update(input, 1004000000);
    assert(cache.read(output));
    fill_message(output, 1005000000, msg);
    assert(msg.pdo_sample == 2);
    assert(!(msg.feedback_valid_mask & (1u << 16)) && std::isnan(msg.head_position[0]));
    assert(!(msg.feedback_valid_mask & (1u << 18)) && std::isnan(msg.torso_position[0]));
    fill_message(output, 1025000000, msg);
    assert(msg.feedback_valid_mask == 0 && std::isnan(msg.r_position[7]));
    fill_message(output, 1003000000, msg);
    assert(msg.pdo_age == -1 && msg.feedback_valid_mask == 0);
    // Unchanged values with advancing PDO receives are valid stationary measurements.
    cache.update(input, 1006000000);
    assert(cache.read(output) && output.sample == 3);

    Cache concurrent;
    std::atomic<bool> done{false};
    std::thread writer([&] {
        Snapshot sample;
        sample.wkc = sample.expected_wkc = 42;
        sample.valid_mask = 0xfffff;
        for (int i = 1; i <= 20000; ++i) {
            sample.position.fill(float(i)); sample.speed.fill(float(-i));
            sample.current.fill(float(i));
            concurrent.update(sample, i);
        }
        done.store(true);
    });
    unsigned coherent = 0;
    do {
        Snapshot sample;
        if (!concurrent.read(sample) || sample.sample == 0) continue;
        ++coherent;
        assert(sample.sampled_ns == int64_t(sample.sample));
        for (unsigned i = 0; i < axes; ++i) {
            assert(sample.position[i] == float(sample.sample));
            assert(sample.speed[i] == -float(sample.sample));
        }
    } while (!done.load() || coherent == 0);
    writer.join();
    assert(concurrent.read(output) && output.sample == 20000);
    assert(coherent > 0);
    std::cout << "PDO mapping, validity, age, progress and concurrent snapshot checks passed\n";
}
