#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>

// Shared by the reviewed 118 middle/servo patches. No ROS or device access.
namespace bindu_position {
inline double steady_seconds() {
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

inline int interpolation_steps(double elapsed, double period, int maximum) {
    if (!std::isfinite(elapsed) || elapsed <= 0.0005) elapsed = 0.02;
    // Clamp before converting to int, including the first command after an idle.
    const double count = std::clamp(elapsed / period, 2.0, double(maximum));
    return int(std::round(count));
}

class TopicCapture {
    double source_stamp_ = 0.0, received_ = 0.0, last_target_ = 0.0;
    bool valid_ = false, started_ = false;
public:
    void observe(double stamp, double ros_now, double steady_now, bool finite) {
        valid_ = finite && std::isfinite(stamp) && std::isfinite(ros_now) &&
            std::isfinite(steady_now) && stamp > source_stamp_ &&
            ros_now >= stamp && ros_now - stamp <= 0.2;
        if (valid_) {
            source_stamp_ = stamp;
            received_ = steady_now;
        }
    }
    bool fresh(double ros_now, double steady_now) const {
        return valid_ && std::isfinite(ros_now) && std::isfinite(steady_now) &&
            ros_now >= source_stamp_ && ros_now - source_stamp_ <= 0.2 &&
            steady_now >= received_ && steady_now - received_ <= 0.2;
    }
    bool needs_seed(double now, bool restart) const {
        return !started_ || restart || now < last_target_ || now-last_target_ > 0.2;
    }
    void accept(double now) { started_ = true; last_target_ = now; }
    void reset() { started_ = false; }
};
}  // namespace bindu_position
