#pragma once
#include "adaptive_stream.hpp"
#include <array>
#include <memory>

namespace bindu::execution {
struct JointState { std::vector<double> q,v,a,j; };
using JointLimits=std::vector<Limits>;
struct Curve {
    double start,duration;
    std::vector<std::array<std::vector<double>,4>> axes;
    Curve(double start,double duration,const JointState& initial,const JointState& final);
    Curve(double start,const Piece& piece);
    double end() const { return start+duration; }
    JointState sample(double now) const;
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
