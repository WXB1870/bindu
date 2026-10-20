#ifndef SNOWBOT_BINDU_WATCHDOG_H
#define SNOWBOT_BINDU_WATCHDOG_H

#include <atomic>
#include <cstdint>

// ROS thread accepts heartbeats; the EtherCAT thread checks time independently.
// Deliberately latched: a late packet or restarted producer cannot resume motion.
class BinduWatchdog {
public:
    static constexpr int64_t timeout_ns = 200000000;
    void enable() { enabled_.store(true); }
    bool enabled() const { return enabled_.load(); }
    bool armed() const { return last_ns_.load() > 0; }
    bool latched() const { return latched_.load(); }
    uint64_t session() const { return session_.load(); }
    bool heartbeat(uint64_t session, uint64_t sequence, int64_t now_ns) {
        if (!enabled() || latched() || session == 0 ||
            (session_.load() != 0 && session != session_.load()) ||
            sequence <= sequence_.load() || now_ns <= 0)
            return false;
        // Expiry is checked here too, so callback scheduling cannot erase a gap
        // just before the next RT check. Never refresh an expired heartbeat.
        const int64_t last = last_ns_.load();
        if (last > 0 && (now_ns < last || now_ns-last >= timeout_ns)) {
            latched_.store(true);
            return false;
        }
        session_.store(session);
        sequence_.store(sequence);
        last_ns_.store(now_ns);
        return !latched();
    }
    bool blocked(int64_t now_ns, bool bus_healthy) {
        if (!enabled()) return false;
        const int64_t last = last_ns_.load();
        if (last > 0 && (!bus_healthy || now_ns < last || now_ns-last >= timeout_ns))
            latched_.store(true);
        return last <= 0 || latched();
    }
private:
    std::atomic<bool> enabled_{false}, latched_{false};
    std::atomic<uint64_t> sequence_{0};
    std::atomic<uint64_t> session_{0};
    std::atomic<int64_t> last_ns_{0};
};

// Independent command deadlines: an arm heartbeat cannot keep a stale gripper
// target alive. Explicit hold persists; timeout/fault is latched until restart.
class BinduGripperWatchdog {
public:
    uint64_t sequence() const { return sequence_.load(); }
    bool latched() const { return latched_.load(); }
    bool accept(uint64_t sequence, int64_t now, int64_t ttl, bool hold) {
        if (!sequence || sequence <= sequence_.load() || now <= 0 || ttl <= 0 || latched()) return false;
        if (sequence_.load() && !hold_.load() && (now < last_ns_.load() || now >= expires_.load())) {
            latched_.store(true); return false;
        }
        hold_.store(hold); last_ns_.store(now); expires_.store(now+ttl);
        sequence_.store(sequence); return true;
    }
    bool blocked(int64_t now, bool healthy) {
        if (!sequence_.load()) return true;
        if (!healthy || now < last_ns_.load() || (!hold_.load() && now >= expires_.load())) latched_.store(true);
        return hold_.load() || latched();
    }
private:
    std::atomic<uint64_t> sequence_{0};
    std::atomic<int64_t> last_ns_{0}, expires_{0};
    std::atomic<bool> hold_{true}, latched_{false};
};

#endif
