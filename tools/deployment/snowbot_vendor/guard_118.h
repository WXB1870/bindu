#pragma once

#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>

// Native 118 position protocol. No ROS, allocation, device access or motion.
// ROS is the sole writer of command deadlines; RT independently checks them.
namespace bindu_guard118 {
constexpr int64_t timeout_ns = 200000000;
constexpr unsigned channels = 5; // arms, left gripper, right gripper, head, waist
constexpr std::array<uint32_t, channels> masks{0x7f7f, 0x80, 0x8000, 0x30000, 0xc0000};
struct Command {
    int64_t stamp_ns = 0;
    uint64_t sequence = 0, session = 0;
    bool hold = false;
};
inline bool fresh(int64_t stamp, int64_t now) {
    return stamp > 0 && now >= stamp && now-stamp < timeout_ns;
}
inline bool valid(const Command& c, int64_t now) {
    return c.sequence && c.session && fresh(c.stamp_ns, now);
}
template<class Message> Command from_message(const Message& m) {
    return {m.bindu_stamp_ns, m.bindu_sequence, m.bindu_session, m.bindu_hold};
}
template<class Message> void to_message(const Command& c, Message& m) {
    m.bindu_stamp_ns = c.stamp_ns; m.bindu_sequence = c.sequence;
    m.bindu_session = c.session; m.bindu_hold = c.hold;
}
// Preserve native position/gain prefix; guarded arrays append:
// [ROS stamp seconds, sequence, session high32, session low32, hold].
template<class Values> bool from_array(const Values& v, unsigned prefix, Command& c) {
    if (v.size() != prefix+5) return false;
    for (double x : v) if (!std::isfinite(x)) return false;
    const auto integer = [](double v, double hi) {
        return v >= 0 && v <= hi && std::floor(v) == v;
    };
    if (v[prefix] <= 0 || v[prefix] >= 9e9 ||
        !integer(v[prefix+1], 9007199254740991.) ||
        !integer(v[prefix+2], 4294967295.) || !integer(v[prefix+3], 4294967295.) ||
        !integer(v[prefix+4], 1.)) return false;
    c = {static_cast<int64_t>(std::llround(v[prefix]*1e9)),
         static_cast<uint64_t>(v[prefix+1]),
         (static_cast<uint64_t>(v[prefix+2]) << 32) | static_cast<uint64_t>(v[prefix+3]),
         v[prefix+4] == 1.};
    return c.sequence && c.session;
}

class Guard {
    std::atomic<bool> enabled_{false}, latched_{false};
    std::atomic<uint64_t> session_{0};
    std::atomic<int64_t> heartbeat_deadline_{0};
    // Deadline 0: untouched; -1: explicit hold, still subject to heartbeat/bus.
    std::array<std::atomic<int64_t>, channels> deadline_{};
    uint64_t heartbeat_sequence_ = 0; // ROS writer only
    std::array<Command, channels> last_{}; // ROS writer only
public:
    Guard() { for (auto& d : deadline_) d.store(0); }
    void enable() { enabled_.store(true); }
    bool enabled() const { return enabled_.load(); }
    bool armed() const { return heartbeat_deadline_.load() > 0; }
    bool latched() const { return latched_.load(); }
    uint64_t session() const { return session_.load(); }
    void trip() { if (enabled()) latched_.store(true); }
    bool blocked(int64_t now, bool healthy) {
        if (!enabled()) return false;
        const auto hb = heartbeat_deadline_.load();
        if (hb > 0 && (!healthy || now <= 0 || now >= hb)) trip();
        for (const auto& d : deadline_) {
            const auto expiry = d.load();
            if (expiry > 0 && now >= expiry) trip();
        }
        return hb <= 0 || latched();
    }
    bool heartbeat(uint64_t session, uint64_t sequence, int64_t stamp,
                   int64_t ros_now, int64_t steady_now, bool healthy) {
        if (!enabled() || (blocked(steady_now, healthy) && armed()) ||
            !healthy || !session || !sequence || sequence <= heartbeat_sequence_ ||
            !fresh(stamp, ros_now) || steady_now <= 0 ||
            (this->session() && session != this->session())) return false;
        if (latched()) return false;
        session_.store(session);
        heartbeat_sequence_ = sequence;
        heartbeat_deadline_.store(steady_now + timeout_ns - (ros_now-stamp));
        return !latched();
    }
    // Validate without opening RT output. Commit only after every cached target
    // has been updated in the ROS loop, so first enable cannot release old zeros.
    bool accepts(unsigned channel, const Command& c, int64_t ros_now, int64_t steady_now,
                 bool healthy) {
        if (channel >= channels || !enabled() || blocked(steady_now, healthy) ||
            c.session != session() || !valid(c, ros_now)) return false;
        const auto& old = last_[channel];
        return (c.sequence > old.sequence && c.stamp_ns >= old.stamp_ns) ||
            (c.sequence == old.sequence && c.stamp_ns == old.stamp_ns &&
             c.session == old.session && c.hold == old.hold);
    }
    bool commit(unsigned channel, const Command& c, int64_t ros_now, int64_t steady_now,
                bool healthy) {
        if (!accepts(channel, c, ros_now, steady_now, healthy)) return false;
        // Middle-layer interpolation may emit more than once per source command.
        // Its publications do not renew the original command's deadline.
        if (c.sequence == last_[channel].sequence) return true;
        last_[channel] = c;
        deadline_[channel].store(c.hold ? -1 : steady_now + timeout_ns - (ros_now-c.stamp_ns));
        return !latched();
    }
    uint32_t touched_mask() const {
        uint32_t mask = 0;
        for (unsigned i=0; i<channels; ++i) if (deadline_[i].load()) mask |= masks[i];
        return mask;
    }
    uint32_t hold_mask() const {
        uint32_t mask = 0;
        for (unsigned i=0; i<channels; ++i) if (deadline_[i].load() == -1) mask |= masks[i];
        return mask;
    }
};

// Freeze once at measured position. Never follow a drifting encoder after trip.
struct Hold {
    bool captured = false;
    float position = 0, kp = 0, kd = 0, torque = 0;
    void capture(float measured, float last_position, float p, float d, float gravity) {
        if (captured) return;
        position = std::isfinite(measured) ? measured : last_position;
        kp = p; kd = d; torque = gravity;
        captured = true;
    }
};
} // namespace bindu_guard118
