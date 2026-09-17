#include "interpolation.hpp"
#include "curve_math.hpp"
#include <limits>
#include <stdexcept>

namespace bindu::execution {
namespace {
void check(bool ok,const char* message) { if(!ok) throw std::invalid_argument(message); }
bool finite(const std::vector<double>& v) {
    return std::all_of(v.begin(),v.end(),[](double x){return std::isfinite(x);});
}
void validate(const JointState& s,size_t n) {
    check(n>0&&n<=256&&s.q.size()==n&&s.v.size()==n&&s.a.size()==n&&
          finite(s.q)&&finite(s.v)&&finite(s.a),"INVALID_REFERENCE_STATE");
}
void validate(const JointLimits& limits,const JointState& s) {
    validate(s,limits.size());
    for(size_t i=0;i<limits.size();++i) {
        const auto& l=limits[i];AdaptiveStream checked(l);
        check(s.q[i]>=l.lower-1e-9&&s.q[i]<=l.upper+1e-9&&std::abs(s.v[i])<=l.velocity+1e-9&&
              std::abs(s.a[i])<=l.acceleration+1e-9,"INITIAL_DYNAMIC_LIMIT");
    }
}
JointState rest(const std::vector<double>& q) {
    return {q,std::vector<double>(q.size()),std::vector<double>(q.size()),std::vector<double>(q.size())};
}
bool moving(const JointState& s) {
    for(double v:s.v) if(std::abs(v)>1e-8) return true;
    for(double a:s.a) if(std::abs(a)>1e-8) return true;
    return false;
}
}
Curve::Curve(double begin,double dt,const JointState& initial,const JointState& final):start(begin),duration(dt) {
    check(std::isfinite(start)&&std::isfinite(dt)&&dt>0.&&std::isfinite(start+dt),"INVALID_CURVE_TIME");
    validate(initial,initial.q.size());validate(final,initial.q.size());
    for(size_t i=0;i<initial.q.size();++i) {
        const double q=initial.q[i],v=initial.v[i],a=initial.a[i],p=final.q[i],w=final.v[i],b=final.a[i];
        std::array<std::vector<double>,4> orders;
        orders[0]={q,q+v*dt/5,q+2*v*dt/5+a*dt*dt/20,p-2*w*dt/5+b*dt*dt/20,p-w*dt/5,p};
        for(int k=1;k<4;++k) orders[k]=detail::deriv(orders[k-1],dt);
        axes.push_back(std::move(orders));
    }
}
JointState Curve::sample(double now) const {
    check(std::isfinite(now),"INVALID_SAMPLE_TIME");
    const double elapsed=std::clamp(now-start,0.,duration);
    const double u=source_duration_>0.?(elapsed+source_elapsed_)/source_duration_:elapsed/duration;
    JointState s;
    for(const auto& orders:axes) {
        s.q.push_back(detail::eval(orders[0],u));s.v.push_back(detail::eval(orders[1],u));
        s.a.push_back(detail::eval(orders[2],u));s.j.push_back(detail::eval(orders[3],u));
    }
    return s;
}
Curve::Curve(double begin,const Piece& piece):start(begin),duration(piece.duration) {
    check(std::isfinite(duration)&&duration>0.,"INVALID_PIECE_TIME");
    std::array<std::vector<double>,4> orders;
    if(!piece.controls.empty()) {
        source_duration_=piece.source_duration;source_elapsed_=piece.source_elapsed;
        orders[0]=piece.controls;
        for(int k=1;k<4;++k) orders[k]=detail::deriv(orders[k-1],source_duration_);
    } else {
        const auto s=piece.initial;const double t=duration,j=s.j;
        cubic_initial_.push_back(s);
        orders[0]={s.q,s.q+s.v*t/3,s.q+2*s.v*t/3+s.a*t*t/6,
                   s.q+s.v*t+s.a*t*t/2+j*t*t*t/6};
        orders[1]={s.v,s.v+s.a*t/2,s.v+s.a*t+j*t*t/2};
        orders[2]={s.a,s.a+j*t};orders[3]={j};
    }
    axes.push_back(std::move(orders));
}
void Curve::append_axis(const Curve& axis) {
    check(axis.start==start&&axis.duration==duration&&axis.axes.size()==1&&
          axis.cubic_initial_.size()==1&&cubic_initial_.size()==axes.size(),"INVALID_BRAKE_AXIS");
    axes.push_back(axis.axes.front());cubic_initial_.push_back(axis.cubic_initial_.front());
}
bool Curve::valid(const JointLimits& limits) const {
    check(limits.size()==axes.size(),"DIMENSION_MISMATCH");
    for(size_t i=0;i<axes.size();++i) {
        const auto& l=limits[i];AdaptiveStream checked(l);
        if(cubic_initial_.size()==axes.size()) {
            const auto initial=cubic_initial_[i];
            if(!AdaptiveStream::valid_phase(l,{initial,duration,initial.j})) return false;
            continue;
        }
        const double low[]={l.lower,-l.velocity,-l.acceleration,-l.jerk};
        const double high[]={l.upper,l.velocity,l.acceleration,l.jerk};
        for(int k=0;k<4;++k) if(!detail::bounded(axes[i][k],low[k],high[k])) return false;
    }
    return true;
}
JointState sample(const Curves& curves,double now) {
    check(std::isfinite(now),"INVALID_SAMPLE_TIME");
    check(!curves.empty(),"EMPTY_TRAJECTORY");
    for(const auto& c:curves) if(now<c->end()) return c->sample(now);
    auto result=curves.back()->sample(curves.back()->end());
    result.j.assign(result.q.size(),0.);return result;
}
Curves fit_stop(const JointLimits& limits,const JointState& state,double start,double timeout) {
    validate(limits,state);check(std::isfinite(timeout)&&timeout>0.,"INVALID_STOP_TIMEOUT");
    double duration=.02;
    for(int attempt=0;attempt<70&&duration<=timeout;++attempt,duration*=1.12) {
        auto goal=state.q;
        for(size_t i=0;i<goal.size();++i) goal[i]+=state.v[i]*duration/2+state.a[i]*duration*duration/12;
        auto c=std::make_shared<Curve>(start,duration,state,rest(goal));
        if(c->valid(limits)) return {c};
    }
    // Adaptive online states carry a certified jerk-limited brake even when a
    // single quintic stop cannot fit. Split at every axis's phase boundary.
    std::vector<Brake> brakes(limits.size());std::vector<double> knots{0.};
    for(size_t i=0;i<limits.size();++i) {
        check(AdaptiveStream::brake(limits[i],{state.q[i],state.v[i],state.a[i],0.},brakes[i]),"NO_FEASIBLE_STOP");
        double elapsed=0.;for(const auto& phase:brakes[i].phases) {elapsed+=phase.duration;knots.push_back(elapsed);}
        check(elapsed<=timeout,"NO_FEASIBLE_STOP");
    }
    std::sort(knots.begin(),knots.end());knots.erase(std::unique(knots.begin(),knots.end()),knots.end());
    Curves curves;
    for(size_t i=1;i<knots.size();++i) {
        if(knots[i]-knots[i-1]<1e-12) continue;
        std::shared_ptr<Curve> c;
        for(const auto& brake:brakes) {
            auto initial=brake.sample(knots[i-1]);
            initial.j=brake.sample((knots[i]+knots[i-1])/2).j;
            const Piece piece{initial,brake.sample(knots[i]),knots[i]-knots[i-1],{},0.,0.};
            auto axis=std::make_shared<Curve>(start+knots[i-1],piece);
            if(!c) c=axis;else c->append_axis(*axis);
        }
        check(c->valid(limits),"NO_FEASIBLE_STOP");curves.push_back(c);
    }
    check(!curves.empty(),"NO_FEASIBLE_STOP");return curves;
}
Curves fit_target(const JointLimits& limits,const JointState& state,const std::vector<double>& goal,
                 double start,double horizon,double stop_timeout) {
    validate(limits,state);validate(rest(goal),limits.size());
    check(std::isfinite(horizon)&&horizon>0.,"INVALID_TRANSITION_HORIZON");
    bool reverse=false;double distance=0.;
    for(size_t i=0;i<goal.size();++i) {
        check(goal[i]>=limits[i].lower&&goal[i]<=limits[i].upper,"JOINT_LIMIT");
        reverse|=state.v[i]*(goal[i]-state.q[i])<-1e-8;
        distance=std::max(distance,std::abs(goal[i]-state.q[i])/limits[i].velocity);
    }
    if(!reverse) {
        double duration=std::max(.02,distance);
        for(int attempt=0;attempt<70&&duration<=horizon;++attempt,duration*=1.12) {
            auto c=std::make_shared<Curve>(start,duration,state,rest(goal));
            if(c->valid(limits)) return {c};
        }
    }
    if(reverse||moving(state)) {
        auto prefix=fit_stop(limits,state,start,stop_timeout);
        const double end=prefix.back()->end();auto stopped=sample(prefix,end);
        // Normalize certified numerical endpoint residuals before recursion.
        stopped.v.assign(goal.size(),0.);stopped.a.assign(goal.size(),0.);
        auto suffix=fit_target(limits,stopped,goal,end,horizon,stop_timeout);
        prefix.insert(prefix.end(),suffix.begin(),suffix.end());return prefix;
    }
    throw std::invalid_argument("NO_FEASIBLE_CONTINUATION");
}
Curves fit_timed(const JointLimits& limits,const JointState& initial,const std::vector<JointState>& points,
                 const std::vector<double>& times,double start,bool chunk,double stop_timeout) {
    validate(limits,initial);
    check(std::isfinite(start),"INVALID_TRAJECTORY_TIME");
    check(!points.empty()&&points.size()<=1024&&points.size()==times.size()&&finite(times),"INVALID_TRAJECTORY_TIME");
    for(size_t i=0;i<points.size();++i) {
        validate(points[i],limits.size());
        if(i) check(times[i]>times[i-1],"INVALID_TRAJECTORY_TIME");
    }
    if(!chunk) {
        for(double x:points.back().v) check(std::abs(x)<=1e-9,"FINITE_ENDPOINT_NOT_AT_REST");
        for(double x:points.back().a) check(std::abs(x)<=1e-9,"FINITE_ENDPOINT_NOT_AT_REST");
    }
    Curves curves;auto previous=initial;const double switch_time=start;
    for(size_t i=0;i<points.size();++i) {
        if(times[i]<=switch_time+1e-5) {
            if(chunk) continue;
            throw std::invalid_argument("EXPIRED_TRAJECTORY_PREFIX");
        }
        check(times[i]-start>=1e-5,"TRAJECTORY_INTERVAL_TOO_SHORT");
        auto c=std::make_shared<Curve>(start,times[i]-start,previous,points[i]);
        if(!c->valid(limits)) {
            // Only the new connection to the first future knot may be shaped.
            // Preserve every supplied knot's absolute time and derivatives.
            // Saturated adaptive states may need a braking prefix before a
            // feasible quintic connection; never retime the caller's knots.
            check(curves.empty()&&moving(previous),"TRAJECTORY_DYNAMIC_LIMIT");
            Curves brake;
            try {brake=fit_stop(limits,previous,start,stop_timeout);}
            catch(const std::invalid_argument&){throw std::invalid_argument("TRAJECTORY_DYNAMIC_LIMIT");}
            const double end=brake.back()->end();
            check(times[i]-end>=1e-5,"TRAJECTORY_DYNAMIC_LIMIT");
            c=std::make_shared<Curve>(end,times[i]-end,sample(brake,end),points[i]);
            check(c->valid(limits),"TRAJECTORY_DYNAMIC_LIMIT");
            curves=std::move(brake);
        }
        curves.push_back(c);
        previous=points[i];start=times[i];
    }
    check(!curves.empty(),"EXPIRED_TRAJECTORY_PREFIX");
    if(chunk) {auto tail=fit_stop(limits,previous,start,stop_timeout);curves.insert(curves.end(),tail.begin(),tail.end());}
    return curves;
}
Online::Online(const JointLimits& limits,const JointState& initial,const std::vector<double>& goal,
               double start,double period,double horizon):start_(start),period_(period),horizon_(horizon),goals_(goal),initial_(initial) {
    validate(limits,initial);validate(rest(goal),limits.size());
    check(std::isfinite(start)&&std::isfinite(period)&&period>=1e-5&&std::isfinite(horizon)&&
          horizon>=period&&horizon/period<=1000000.,"INVALID_ONLINE_TIME");
    for(size_t i=0;i<limits.size();++i) {
        auto l=limits[i];l.max_gap=std::max(l.max_gap,period);
        streams_.emplace_back(l);
        check(streams_.back().reset(start,{initial.q[i],initial.v[i],initial.a[i],0.}),"NO_FEASIBLE_INITIAL_BRAKE");
        check(streams_.back().target(start,goal[i],std::numeric_limits<double>::max()),"INVALID_ONLINE_TARGET");
    }
    paths_.resize(goal.size());planned_.assign(goal.size(),start);finished_.assign(goal.size(),false);
}
JointState Online::sample(double now) {
    check(std::isfinite(now),"INVALID_SAMPLE_TIME");
    if(now<=start_) return initial_;
    JointState result;
    for(size_t axis=0;axis<streams_.size();++axis) {
        auto& stream=streams_[axis];auto& path=paths_[axis];auto& planned=planned_[axis];
        while(planned<now&&!finished_[axis]) {
            check(planned-start_<horizon_+1e-9,"ONLINE_TRANSITION_TIMEOUT");
            const double next=planned+period_;
            check(next>planned&&stream.step(next),"ONLINE_REFERENCE_FAULT");
            double begin=planned;
            for(const auto& p:stream.pieces()) {
                if(p.duration>0.) path.push_back(std::make_shared<Curve>(begin,p));
                begin+=p.duration;
            }
            planned=next;
            const auto s=stream.state();finished_[axis]=s.q==goals_[axis]&&s.v==0.&&s.a==0.;
        }
        // At a cached boundary use the left derivative consistently, whether
        // or not a future preview has already generated the next segment.
        const auto it=std::lower_bound(path.begin(),path.end(),now,
            [](const auto& curve,double t){return curve->end()<t;});
        auto s=(it==path.end()?path.back():*it)->sample(now);
        if(finished_[axis]&&now>=path.back()->end()) s.j[0]=0.;
        result.q.push_back(s.q[0]);result.v.push_back(s.v[0]);result.a.push_back(s.a[0]);result.j.push_back(s.j[0]);
    }
    return result;
}
}  // namespace bindu::execution
