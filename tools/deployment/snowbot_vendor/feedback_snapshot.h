#pragma once

#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>

// Measured PDO data only. Slot order is L1..L8, R1..R8, head1..2, torso1..2.
// Single RT writer, bounded ROS reader; no allocation, locks or device access.
namespace bindu_feedback {
constexpr unsigned axes = 20;
constexpr double csp_scale = 6.28318530717958647692 / 65536.0;
inline float csp_radians(int32_t counts) { return float(counts * csp_scale); }

struct Snapshot {
    std::array<float, axes> position{}, speed{}, current{};
    std::array<uint16_t, axes> status{};
    uint32_t valid_mask = 0;
    uint64_t sample = 0;
    int64_t sampled_ns = 0;
    int wkc = 0, expected_wkc = 0;
};

class Cache {
    std::atomic<uint64_t> version_{0}, sample_{0};
    std::atomic<int64_t> sampled_ns_{0};
    std::atomic<uint32_t> valid_mask_{0};
    std::atomic<int> wkc_{0}, expected_wkc_{0};
    std::array<std::atomic<float>, axes> position_{}, speed_{}, current_{};
    std::array<std::atomic<uint16_t>, axes> status_{};
public:
    Cache() {
        for (unsigned i = 0; i < axes; ++i) {
            position_[i].store(0); speed_[i].store(0); current_[i].store(0);
            status_[i].store(0);
        }
    }
    void update(const Snapshot& input, int64_t now_ns) {
        // All atomics use sequential consistency, including the version fence.
        // Unlike a seqlock over plain data this is free of C++ data races.
        ++version_;
        wkc_.store(input.wkc);
        expected_wkc_.store(input.expected_wkc);
        uint32_t valid = 0;
        if (input.expected_wkc > 0 && input.wkc == input.expected_wkc && now_ns > 0) {
            for (unsigned i = 0; i < axes; ++i) {
                if ((input.valid_mask & (1u << i)) &&
                        std::isfinite(input.position[i]) && std::isfinite(input.speed[i]) &&
                        std::isfinite(input.current[i])) {
                    valid |= 1u << i;
                    position_[i].store(input.position[i]);
                    speed_[i].store(input.speed[i]);
                    current_[i].store(input.current[i]);
                    status_[i].store(input.status[i]);
                }
            }
            sampled_ns_.store(now_ns);
            ++sample_;
        }
        valid_mask_.store(valid); // Failed receive invalidates, without advancing sample.
        ++version_;
    }

    bool read(Snapshot& output) const {
        for (int attempt = 0; attempt < 4; ++attempt) {
            const auto before = version_.load();
            if (before & 1u) continue;
            Snapshot result;
            result.sample = sample_.load();
            result.sampled_ns = sampled_ns_.load();
            result.valid_mask = valid_mask_.load();
            result.wkc = wkc_.load();
            result.expected_wkc = expected_wkc_.load();
            for (unsigned i = 0; i < axes; ++i) {
                result.position[i] = position_[i].load();
                result.speed[i] = speed_[i].load();
                result.current[i] = current_[i].load();
                result.status[i] = status_[i].load();
            }
            if (before == version_.load()) { output = result; return true; }
        }
        return false; // Skip publication if a coherent read could not be obtained.
    }
};

template<class Message>
void fill_message(const Snapshot& sample, int64_t now_ns, Message& msg) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    msg.pdo_sample = sample.sample;
    msg.pdo_age = sample.sampled_ns > 0 && now_ns >= sample.sampled_ns
        ? (now_ns - sample.sampled_ns) * 1e-9 : -1.0;
    msg.pdo_wkc = sample.wkc;
    msg.pdo_expected_wkc = sample.expected_wkc;
    msg.feedback_valid_mask = msg.pdo_age >= 0.0 && msg.pdo_age <= 0.02
        ? sample.valid_mask : 0u;
    for (unsigned i = 0; i < axes; ++i) {
        const bool valid = (msg.feedback_valid_mask & (1u << i)) != 0;
        const double q = valid ? sample.position[i] : nan;
        const double v = valid ? sample.speed[i] : nan;
        msg.feedback_status[i] = valid ? sample.status[i] : 0;
        if (i < 8) {
            msg.l_position[i] = q; msg.l_speed[i] = v;
            msg.l_iq[i] = valid ? sample.current[i] : nan;
        } else if (i < 16) {
            msg.r_position[i-8] = q; msg.r_speed[i-8] = v;
            msg.r_iq[i-8] = valid ? sample.current[i] : nan;
        } else if (i < 18) {
            msg.head_position[i-16] = q; msg.head_speed[i-16] = v;
        } else {
            msg.torso_position[i-18] = q; msg.torso_speed[i-18] = v;
        }
    }
}
} // namespace bindu_feedback
