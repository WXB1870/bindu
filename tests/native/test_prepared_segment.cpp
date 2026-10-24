// Standalone joint-segment tests. No ROS, device SDK, or hardware process.
#include "interpolation.hpp"
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <new>
#include <stdexcept>

namespace {
bool count_allocations=false;
size_t allocations=0,deallocations=0;
}
void* operator new(std::size_t n) {
    if(count_allocations) ++allocations;
    if(auto p=std::malloc(n?n:1)) return p;
    throw std::bad_alloc();
}
void operator delete(void* p) noexcept {if(count_allocations&&p) ++deallocations;std::free(p);}
void* operator new[](std::size_t n) {return ::operator new(n);}
void operator delete[](void* p) noexcept {::operator delete(p);}
void operator delete(void* p,std::size_t) noexcept {::operator delete(p);}
void operator delete[](void* p,std::size_t) noexcept {::operator delete(p);}

using namespace bindu::execution;
namespace {
void require(bool ok,const char* message) {if(!ok) throw std::runtime_error(message);}
void close(double a,double b,double eps=1e-8) {require(std::abs(a-b)<=eps,"numeric mismatch");}
void continuous(const JointState& lhs,const JointState& rhs) {
    for(size_t i=0;i<lhs.q.size();++i) {
        close(lhs.q[i],rhs.q[i]);close(lhs.v[i],rhs.v[i]);close(lhs.a[i],rhs.a[i]);
    }
}
void bounded(const JointLimits& limits,const JointState& s) {
    for(size_t i=0;i<limits.size();++i) {
        const auto& l=limits[i];
        require(s.q[i]>=l.lower-1e-8&&s.q[i]<=l.upper+1e-8,"position limit");
        require(std::abs(s.v[i])<=l.velocity+1e-8,"velocity limit");
        require(std::abs(s.a[i])<=l.acceleration+1e-8,"acceleration limit");
        require(std::abs(s.j[i])<=l.jerk+1e-8,"jerk limit");
    }
}
JointState rest(std::vector<double> q) {const auto n=q.size();return {q,std::vector<double>(n),std::vector<double>(n),{}};}
JointLimits limits(size_t n) {
    JointLimits out(n);
    for(size_t i=0;i<n;++i) {out[i].lower=-2.;out[i].upper=2.;out[i].velocity=.25+.01*i;out[i].acceleration=.5;out[i].jerk=4.;}
    return out;
}
void synchronized_and_no_allocations() {
    constexpr size_t n=16;auto ls=limits(n);
    auto initial=rest(std::vector<double>(n));std::vector<double> goal(n);
    for(size_t i=0;i<n;++i) {initial.q[i]=.01*i;goal[i]=initial.q[i]+(i%2?-.3:.2);}
    auto curves=fit_target(ls,initial,goal,10.,20.,5.);
    auto p=PreparedSegment::from_curves(curves,ls);
    require(p.axes()==n&&p.pieces()==1,"joint curve layout");
    for(int step=0;step<=1000;++step) {
        const auto t=p.start()+(p.end()-p.start())*step/1000;
        const auto s=p.sample(t);bounded(ls,s);
        for(size_t i=1;i<n;++i)
            close((s.q[i]-initial.q[i])/(goal[i]-initial.q[i]),
                  (s.q[0]-initial.q[0])/(goal[0]-initial.q[0]));
        continuous(s,sample(curves,t));
    }
    // Preparing snapshots must detach from mutable caller-owned curves.
    const auto before=p.sample(10.1);curves.front()->axes[0][0][0]=123.;
    continuous(before,p.sample(10.1));
    std::array<double,n> q{},v{},a{},j{};
    bool ok=true;allocations=deallocations=0;count_allocations=true;
    // 1 kHz timestamp sequence, plus unordered preview queries. This proves
    // bounded evaluation/no allocations, not real-time scheduling performance.
    for(int step=0;step<10000;++step) {
        ok=p.sample_into(10.+step*.001,q.data(),v.data(),a.data(),j.data(),n)&&ok;
        if(step%17==0) ok=p.sample_into(10.001,q.data(),v.data(),a.data(),j.data(),n)&&ok;
    }
    count_allocations=false;
    require(ok,"sampling failed");require(allocations==0&&deallocations==0,"sampler allocated or freed memory");
    const auto saved=q;
    require(!p.sample_into(10.,q.data(),v.data(),a.data(),j.data(),n-1),"short output accepted");
    require(!p.sample_into(std::nan(""),q.data(),v.data(),a.data(),j.data(),n),"NaN sample accepted");
    require(!p.sample_into(10.,nullptr,v.data(),a.data(),j.data(),n),"null output accepted");
    require(q==saved,"invalid sampling changed output");
    PreparedSegment empty;
    require(!empty.sample_into(0.,q.data(),v.data(),a.data(),j.data(),n),"empty segment sampled");
    const auto final=p.sample(p.end()+10.);
    for(size_t i=0;i<n;++i) {close(final.q[i],goal[i]);close(final.v[i],0.);close(final.a[i],0.);close(final.j[i],0.);}
}
void moving_retargets_and_stop() {
    constexpr size_t n=16;const auto ls=limits(n);
    auto initial=rest(std::vector<double>(n,.1));
    for(size_t i=0;i<n;++i) {initial.v[i]=(i%2?-.03:.04);initial.a[i]=.02;}
    auto p=PreparedSegment::target(ls,initial,std::vector<double>(n,.3),0.,20.,5.);
    continuous(initial,p.sample(0.));
    for(int update=1;update<=200;++update) {
        const double t=update*.01;const auto old=p.sample(t);
        std::vector<double> goal(n);
        // Axes turn at different phases: no straight-line ratio claim applies.
        for(size_t i=0;i<n;++i) goal[i]=.3*std::sin(update*.07+i*.4);
        auto next=PreparedSegment::target(ls,old,goal,t,20.,5.);
        continuous(old,next.sample(t));
        for(int k=0;k<=10;++k) bounded(ls,next.sample(t+k*.001));
        // The compiled sampler must agree with the exact shared planned path;
        // independent axis clipping or a second follower would break this.
        const auto reference=fit_target(ls,old,goal,t,20.,5.);
        for(int k=0;k<=20;++k) {
            const double at=t+(next.end()-t)*k/20.;
            continuous(next.sample(at),sample(reference,at));
        }
        p=std::move(next);
    }
    const auto at_stop=p.sample(2.005);
    const auto stop=PreparedSegment::stop(ls,at_stop,2.005,5.);
    continuous(at_stop,stop.sample(2.005));
    for(int k=0;k<=1000;++k) bounded(ls,stop.sample(2.005+(stop.end()-2.005)*k/1000.));
    const auto stopped=stop.sample(stop.end()+.1);
    for(size_t i=0;i<n;++i) {close(stopped.v[i],0.);close(stopped.a[i],0.);}
    // A fixed hold seed after an independently confirmed stop stays fixed.
    auto hold=PreparedSegment::target(ls,stopped,stopped.q,stop.end(),20.,5.);
    continuous(stopped,hold.sample(hold.end()+1.));
}
void stopped_and_rejected_layouts() {
    const auto ls=limits(2);const auto initial=rest({.4,-.2});
    const auto p=PreparedSegment::target(ls,initial,initial.q,10.,20.,5.);
    for(double t:{9.,10.,10.001,11.}) continuous(initial,p.sample(t));
    const auto gap=std::make_shared<Curve>(11.,1.,initial,initial);
    const auto first=std::make_shared<Curve>(9.,1.,initial,initial);
    bool rejected=false;
    try {PreparedSegment::from_curves({first,gap},ls);}catch(const std::invalid_argument&){rejected=true;}
    require(rejected,"time gap accepted");
    rejected=false;auto moving=initial;moving.v[0]=.1;
    try {PreparedSegment::from_curves({std::make_shared<Curve>(0.,1.,initial,moving)},ls);}
    catch(const std::invalid_argument&){rejected=true;}
    require(rejected,"moving endpoint without stop tail accepted");
    rejected=false;
    try {PreparedSegment::target(ls,initial,{3.,0.},0.,20.,5.);}
    catch(const std::invalid_argument&){rejected=true;}
    require(rejected,"out of range goal accepted");
}
void wire_roundtrip_and_source_pieces() {
    const auto ls=limits(2);const auto seed=rest({.1,-.1});
    auto p=PreparedSegment::target(ls,seed,{.3,.2},10.,20.,5.);
    const auto original=p.describe();require(original.size()==1,"test needs single quintic");
    auto first=original[0],second=first;
    first.source_duration=first.duration;first.duration*=.4;
    second.start=first.start+first.duration;second.source_duration=second.duration;
    second.source_elapsed=first.duration;second.duration-=first.duration;
    const auto split=PreparedSegment::from_description({first,second},ls);
    require(split.pieces()==2,"wire lost a piece");
    const auto roundtrip=PreparedSegment::from_description(split.describe(),ls);
    for(int k=0;k<=1000;++k) {
        const double t=p.start()+(p.end()-p.start())*k/1000.;
        const auto expected=p.sample(t);continuous(expected,split.sample(t));
        continuous(expected,roundtrip.sample(t));
        for(size_t i=0;i<2;++i) close(expected.j[i],roundtrip.sample(t).j[i]);
    }
    auto bad=first;bad.source_elapsed=bad.source_duration;
    bool rejected=false;
    try {PreparedSegment::from_description({bad},ls);}catch(const std::invalid_argument&){rejected=true;}
    require(rejected,"invalid source mapping accepted");

    // Saturated adaptive states can require the existing multi-phase cubic
    // brake fallback. Preserve all its pieces and actual polynomial controls.
    const JointLimits fast(1);
    const JointState saturated{{.02144333333333334},{.88},{20.},{}};
    const auto brake=PreparedSegment::stop(fast,saturated,0.,5.);
    require(brake.pieces()>1,"cubic fallback fixture lost");
    const auto restored=PreparedSegment::from_description(brake.describe(),fast);
    require(restored.pieces()==brake.pieces(),"cubic wire pieces lost");
    for(int k=0;k<=1000;++k) {
        const double t=brake.end()*k/1000.;
        const auto expected=brake.sample(t),actual=restored.sample(t);
        continuous(expected,actual);close(expected.j[0],actual.j[0],1e-6);
        bounded(fast,actual);
    }
}
void continuous_streams() {
    constexpr size_t n=16;auto ls=limits(n);
    for(auto& l:ls) {l.velocity=.25;l.acceleration=.5;l.jerk=162.;}
    auto initial=rest(std::vector<double>(n));
    auto old=PreparedSegment::target(ls,initial,initial.q,0.,20.,5.);
    auto fast=PreparedSegment::stream_target(ls,initial,initial.q,0.,20.,5.);
    double old_error=0.,fast_error=0.;std::vector<double> goal(n);
    for(int tick=0;tick<6000;++tick) {
        const double t=tick*.001;
        auto a=old.sample(t),b=fast.sample(t);bounded(ls,a);bounded(ls,b);
        if(tick%20==0) {
            // A ramp followed by reversal, then a fixed repeated endpoint.
            goal[0]=t<2.?.15*t:t<4.?.3-.15*(t-2.):0.;
            old=PreparedSegment::target(ls,a,goal,t,20.,5.);
            fast=PreparedSegment::stream_target(ls,b,goal,t,20.,5.);
            continuous(b,fast.sample(t));
            require(fast.pieces()<=PreparedSegment::max_pieces,"unbounded stream layout");
        }
        if(t>.3&&t<4.) {
            old_error+=std::pow(a.q[0]-goal[0],2);
            fast_error+=std::pow(b.q[0]-goal[0],2);
        }
        // Inactive axes do not acquire motion from another axis's target.
        for(size_t i=1;i<n;++i) {close(b.q[i],0.);close(b.v[i],0.);}
    }
    require(fast_error<old_error*.7,"stream reference lost response benefit");
    const auto settled=fast.sample(5.999);
    close(settled.q[0],goal[0],1e-5);
    const auto stopped=PreparedSegment::stop(ls,settled,6.,2.);
    continuous(settled,stopped.sample(6.));
    for(int k=0;k<=1000;++k) bounded(ls,stopped.sample(6.+(stopped.end()-6.)*k/1000.));

    // Exercise phase splitting and finite convergence on a
    // moving, non-collinear seed with different axes' brake boundaries.
    initial=rest(std::vector<double>(n,.1));
    for(size_t i=0;i<n;++i) {initial.v[i]=(i%2?-.08:.1);initial.a[i]=(i%3?-.2:.3);goal[i]=(i%2?-.1:.3);}
    fast=PreparedSegment::stream_target(ls,initial,goal,10.,20.,5.);
    require(fast.pieces()>2,"stream prefix missing");
    continuous(initial,fast.sample(10.));
    for(int k=0;k<=2000;++k) {
        const double t=10.+(fast.end()-10.)*k/2000.;
        bounded(ls,fast.sample(t));
    }
    const auto final=fast.sample(fast.end()+1.);
    for(size_t i=0;i<n;++i) {close(final.q[i],goal[i]);close(final.v[i],0.);close(final.a[i],0.);}
    std::array<double,n> q{},v{},a{},j{};
    bool ok=true;allocations=deallocations=0;count_allocations=true;
    for(int k=0;k<10000;++k) ok=fast.sample_into(10.+k*.001,q.data(),v.data(),a.data(),j.data(),n)&&ok;
    count_allocations=false;
    require(ok&&!allocations&&!deallocations,"stream sampler allocated/freed");

    const auto one=limits(1);const auto at_limit=rest({one[0].upper});
    const auto hold=PreparedSegment::stream_target(one,at_limit,at_limit.q,0.,20.,5.);
    continuous(at_limit,hold.sample(hold.end()+1.));
    for(const auto& bad:std::vector<std::vector<double>>{{},{3.},{std::nan("")}}) {
        bool rejected=false;
        try {PreparedSegment::stream_target(one,at_limit,bad,0.,20.,5.);}catch(const std::invalid_argument&){rejected=true;}
        require(rejected,"invalid stream target accepted");
    }
}
void velocity_aware_targets() {
    auto ls=limits(3);
    for(auto& l:ls) {l.velocity=.5;l.acceleration=1.5;l.jerk=162.;}
    const auto initial=rest({0.,0.,0.});
    auto old=PreparedSegment::target(ls,initial,initial.q,0.,20.,5.);
    auto track=PreparedSegment::tracking_target(ls,initial,initial.q,{0.,0.,0.},0.,20.,5.);
    double old_error=0.,track_error=0.;std::vector<double> goal(3),hint(3);
    for(int tick=0;tick<6000;++tick) {
        const double t=tick*.001;const auto a=old.sample(t),b=track.sample(t);
        bounded(ls,b);
        if(tick%20==0) {
            goal[0]=t<2.?.15*t:t<4.?.3-.15*(t-2.):0.;
            hint[0]=t<2.?.15:t<4.?-.15:0.;
            old=PreparedSegment::target(ls,a,goal,t,20.,5.);
            track=PreparedSegment::tracking_target(ls,b,goal,hint,t,20.,5.);
            continuous(b,track.sample(t));
        }
        if(t>.3&&t<4.) {old_error+=std::pow(a.q[0]-goal[0],2);track_error+=std::pow(b.q[0]-goal[0],2);}
        close(b.q[1],0.);close(b.q[2],0.);
    }
    require(track_error<old_error*.7,"velocity hint lost ramp response benefit");
    close(track.sample(5.999).q[0],0.,1e-5);

    // Losing the stream leaves a finite certified path to the last actual
    // goal, never a moving endpoint or a frozen reference with nonzero v/a.
    track=PreparedSegment::tracking_target(ls,initial,{.03,-.02,0.},{.1,-.08,0.},0.,20.,5.);
    for(int k=0;k<=2000;++k) bounded(ls,track.sample(track.end()*k/2000.));
    const auto final=track.sample(track.end()+1.);
    close(final.q[0],.03);close(final.q[1],-.02);
    for(size_t i=0;i<3;++i) {close(final.v[i],0.);close(final.a[i],0.);}
    std::array<double,3> q{},v{},a{},j{};bool ok=true;
    allocations=deallocations=0;count_allocations=true;
    for(int k=0;k<10000;++k) ok=track.sample_into(k*.001,q.data(),v.data(),a.data(),j.data(),3)&&ok;
    count_allocations=false;
    require(ok&&!allocations&&!deallocations,"tracking sampler allocated/freed");

    // An outward trend at the boundary must use the ordinary safe curve.
    const auto edge=rest({2.,0.,0.});
    track=PreparedSegment::tracking_target(ls,edge,edge.q,{.1,0.,0.},0.,20.,5.);
    continuous(edge,track.sample(track.end()+1.));
    for(const auto& bad:std::vector<std::vector<double>>{{},{.6,0.,0.},{NAN,0.,0.}}) {
        bool rejected=false;
        try {PreparedSegment::tracking_target(ls,initial,initial.q,bad,0.,20.,5.);}catch(const std::invalid_argument&){rejected=true;}
        require(rejected,"invalid velocity hint accepted");
    }
}
void small_streaming_motion_does_not_switch_braking_policy() {
    // A continuously advancing small target used to alternate between the
    // moving continuation and the rest-endpoint fallback. Position RMS hid
    // the resulting jerk pulses. Exercise both steady and irregular updates.
    for(int period:{10,20}) for(bool jitter:{false,true}) {
        auto ls=limits(3);
        for(auto& l:ls) {l.velocity=1.;l.acceleration=4.5;l.jerk=162.;}
        const auto initial=rest({0.,0.,0.});
        auto path=PreparedSegment::tracking_target(ls,initial,initial.q,{0.,0.,0.},0.,20.,5.);
        int next=0,updates=0,samples=0;double jerk_squared=0.;std::vector<double> last_goal;
        for(int tick=0;tick<6000;++tick) {
            const double t=tick*.001;const auto state=path.sample(t);bounded(ls,state);
            if(tick==next&&t<4.) {
                last_goal={.02*t+.005*std::sin(3*t),.001*std::sin(4*t),0.};
                const std::vector<double> hint{.02+.015*std::cos(3*t),.004*std::cos(4*t),0.};
                path=PreparedSegment::tracking_target(ls,state,last_goal,hint,t,20.,5.);
                continuous(state,path.sample(t));
                next+=period+(jitter?(++updates%2?3:-3):0);
            }
            close(state.q[2],0.);close(state.v[2],0.);
            if(t>.4&&t<3.8) {
                require(state.v[0]>.002,"advancing stream stopped/reversed");
                jerk_squared+=state.j[0]*state.j[0];++samples;
            }
        }
        require(std::sqrt(jerk_squared/samples)<5.,"small stream has braking-policy jerk pulses");
        const auto final=path.sample(path.end()+1.);
        for(size_t i=0;i<3;++i) {close(final.q[i],last_goal[i]);close(final.v[i],0.);close(final.a[i],0.);}
    }
}
}
int main() {
    try {
        synchronized_and_no_allocations();moving_retargets_and_stop();stopped_and_rejected_layouts();
        wire_roundtrip_and_source_pieces();
        continuous_streams();
        velocity_aware_targets();
        small_streaming_motion_does_not_switch_braking_policy();
        std::cout<<"prepared segment: synchronized/moving/retarget/stop/buffer tests passed; zero sampling allocations\n";
    }catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
