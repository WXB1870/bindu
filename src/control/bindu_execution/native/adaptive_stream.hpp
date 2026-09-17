#pragma once
// Scalar position-stream primitive used by the shared native reference engine.
// The proposal math in adaptive_stream.cpp derives from the read-only historical
// legs_necks_control adaptive_interpolator; validation and integration are new.
#include <vector>

namespace bindu::execution {
struct State { double q=0., v=0., a=0., j=0.; };
struct Limits {
    double lower=-1., upper=1., velocity=1.5, acceleration=20., jerk=400.;
    double max_gap=.05, capture_distance=.02, capture_speed=.2;
    double capture_acceleration=5., capture_wait=.03, capture_horizon=.4;
};
struct Phase { State initial; double duration=0., jerk=0.; };
// Exact polynomial pieces of the last step, including capture and braking
// boundaries. The group sampler uses these, never refits sampled positions.
struct Piece {
    State initial, final;
    double duration=0.;
    // Empty controls: exact constant jerk stored in initial.j. Otherwise use
    // the original capture polynomial and time interval, without refitting.
    std::vector<double> controls;
    double source_duration=0.,source_elapsed=0.;
};
struct Brake {
    std::vector<Phase> phases;
    State final;
    double duration=0.;
    State sample(double elapsed) const;
};
class AdaptiveStream {
public:
    explicit AdaptiveStream(Limits limits={});
    // Radians and seconds in one monotonic clock domain. A moving seed without
    // a target starts braking on the next step; it must never freeze in place.
    bool reset(double now, State state={});
    // Source timestamps must be ordered and no later than the current clock.
    // TTL is mandatory. Duplicate positions refresh TTL without restarting capture.
    bool target(double source_time, double position, double valid_until);
    void stop();
    // Failure latches a clock/state fault. Caller must stop hardware and reset;
    // no speculative late reference is returned after an overrun.
    bool step(double now);
    const State& state() const { return state_; }
    bool stopped() const {
        return initialized_ && !fault_ && !tracking_ && !stopping_ && state_.v==0. && state_.a==0.;
    }
    bool faulted() const { return fault_; }
    unsigned guarded_steps() const { return guarded_; }
    const std::vector<Piece>& pieces() const { return pieces_; }
    static bool brake(const Limits&, State, Brake&);
    static bool valid_phase(const Limits&, const Phase&);
private:
    bool advance(double dt);
    bool capture();
    void record_brake(const Brake&, double elapsed, double duration);
    void record_cubic(State initial, State final, double duration, double jerk);
    std::vector<Piece> pieces_;
    Limits limits_;
    State state_;
    Brake safe_brake_;
    bool initialized_=false, fault_=false, tracking_=false, stopping_=false, capturing_=false;
    double clock_=0., source_=-1., goal_=0., until_=0., changed_=0.;
    double brake_elapsed_=0., capture_elapsed_=0., capture_duration_=0.;
    std::vector<double> controls_;
    unsigned guarded_=0;
};
}  // namespace bindu::execution
