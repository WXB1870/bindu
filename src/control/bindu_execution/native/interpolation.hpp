#pragma once
#include "adaptive_stream.hpp"
#include <array>
#include <cstddef>
#include <memory>

namespace bindu::execution {
struct JointState { std::vector<double> q,v,a,j; };
using JointLimits=std::vector<Limits>;
// Position Bernstein controls, exactly as planned (4 for cubic, 6 for quintic).
// Each axis shares this piece's clock mapping. source_duration==0 means the
// polynomial covers duration; otherwise evaluate (elapsed+source_elapsed)/
// source_duration. q/v/a endpoints alone are not used to replan a wire segment.
struct CurveDescriptor {
    double start=0.,duration=0.,source_duration=0.,source_elapsed=0.;
    std::vector<std::vector<double>> controls;
};
struct Curve {
    double start,duration;
    std::vector<std::array<std::vector<double>,4>> axes;
    Curve(double start,double duration,const JointState& initial,const JointState& final);
    Curve(double start,const Piece& piece);
    explicit Curve(const CurveDescriptor&);
    CurveDescriptor describe() const;
    double end() const { return start+duration; }
    JointState sample(double now) const;
    // Bounded de Casteljau evaluation; no allocation, planning, or exceptions.
    // On false, none of the caller's buffers are changed. Buffers must not alias.
    bool sample_into(double now,double* q,double* v,double* a,double* j,
                     size_t capacity) const noexcept;
    bool valid(const JointLimits&) const;
    void append_axis(const Curve& axis);
private:
    double source_duration_=0.,source_elapsed_=0.;
    std::vector<State> cubic_initial_;
};
using Curves=std::vector<std::shared_ptr<Curve>>;
JointState sample(const Curves&,double now);
Curves fit_target(const JointLimits&,const JointState&,const std::vector<double>& goal,
                 double start,double horizon,double stop_timeout);
Curves fit_stop(const JointLimits&,const JointState&,double start,double timeout);
Curves fit_timed(const JointLimits&,const JointState&,const std::vector<JointState>& points,
                 const std::vector<double>& times,double start,bool chunk,double stop_timeout);
// Prepare on a non-real-time thread, then publish an immutable segment at its
// agreed start time. For replacement, seed q/v/a from the old segment sampled
// at that same time, not from a new measured pose. Only takeover/recovery seeds
// from measurements. Ownership/TTL, clock guards and measured stop confirmation
// stay with the caller; sampling never renews a source command's deadline.
class PreparedSegment {
public:
    static constexpr size_t max_axes=256,max_pieces=1024;
    PreparedSegment()=default; // Empty mailbox/slot; sampling returns false.
    static PreparedSegment target(const JointLimits&,const JointState&,
        const std::vector<double>& goal,double start,double horizon,double stop_timeout);
    static PreparedSegment stop(const JointLimits&,const JointState&,double start,double timeout);
    // Deep-copy already certified curves; no mutable Curve aliases escape.
    static PreparedSegment from_curves(const Curves&,const JointLimits&);
    static PreparedSegment from_description(const std::vector<CurveDescriptor>&,const JointLimits&);
    std::vector<CurveDescriptor> describe() const;
    size_t axes() const noexcept { return axes_; }
    size_t pieces() const noexcept { return curves_.size(); }
    double start() const noexcept { return curves_.empty()?0.:curves_.front()->start; }
    double end() const noexcept { return curves_.empty()?0.:curves_.back()->end(); }
    bool sample_into(double now,double* q,double* v,double* a,double* j,
                     size_t capacity) const noexcept;
    // Convenience wrapper for non-real-time clients, including Python.
    JointState sample(double now) const;
private:
    Curves curves_;
    size_t axes_=0;
};
// Immutable reference semantics with a lazy cache of exact polynomial pieces.
// Queries may be repeated or out of order (e.g. future replacement preview).
// Filling the cache does not change the trajectory or claim execution authority.
class Online {
public:
    Online(const JointLimits&,const JointState&,const std::vector<double>& goal,
           double start,double period,double horizon);
    JointState sample(double now);
private:
    double start_,period_,horizon_;
    std::vector<double> goals_;
    std::vector<AdaptiveStream> streams_;
    std::vector<Curves> paths_;
    std::vector<double> planned_;
    std::vector<bool> finished_;
    JointState initial_;
};
}  // namespace bindu::execution
