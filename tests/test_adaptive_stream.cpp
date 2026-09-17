// Standalone native-engine tests. No ROS, device SDK, or historical source.
#include "adaptive_stream.hpp"
#include "interpolation.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>
using namespace bindu::execution;
void require(bool ok,const std::string& message) { if(!ok) throw std::runtime_error(message); }
void close(double a,double b,double eps=1e-8) {require(std::abs(a-b)<=eps,"numeric mismatch");}
void valid(const Limits& l,State s) {
    require(std::isfinite(s.q)&&std::isfinite(s.v)&&std::isfinite(s.a)&&std::isfinite(s.j),"nonfinite state");
    require(s.q>=l.lower-1e-8&&s.q<=l.upper+1e-8,"position limit");
    require(std::abs(s.v)<=l.velocity+1e-8,"velocity limit");
    require(std::abs(s.a)<=l.acceleration+1e-8,"acceleration limit");
    require(std::abs(s.j)<=l.jerk+1e-8,"jerk limit");
}
struct Point {double t,q;};
void realized_bounds(const Limits& l,const std::vector<Point>& points) {
    std::vector<double> q;for(auto p:points) q.push_back(p.q);
    const double bounds[]={l.velocity,l.acceleration,l.jerk};
    double factorial=1.;
    for(size_t order=1;order<=3;++order) {
        factorial*=order;
        for(size_t i=0;i+1<q.size();++i) q[i]=(q[i+1]-q[i])/(points[i+order].t-points[i].t);
        q.pop_back();
        for(double x:q) require(std::abs(x)*factorial<=bounds[order-1]+2e-5,"realized derivative limit");
    }
}
void input_checks() {
    Limits l; l.jerk=std::numeric_limits<double>::quiet_NaN();
    bool rejected=false;try {AdaptiveStream bad(l);}catch(const std::invalid_argument&){rejected=true;}
    require(rejected,"NaN config accepted");
    l={};l.velocity=std::numeric_limits<double>::infinity();
    rejected=false;try {AdaptiveStream bad(l);}catch(const std::invalid_argument&){rejected=true;}
    require(rejected,"infinite config accepted");
    AdaptiveStream p;require(!p.stopped(),"uninitialized state reported stopped");
    require(!p.step(0.),"uninitialized step");require(p.reset(10.),"reset");
    require(!p.reset(10.,{std::nan(""),0.,0.,0.}),"NaN initial state accepted");
    require(!p.reset(10.,{1.,.5,0.,0.}),"nonviable initial state accepted");
    require(!p.reset(10.,{0.,0.,0.,401.}),"initial jerk limit");
    require(!p.target(10.,.1,std::numeric_limits<double>::infinity()),"infinite expiry accepted");
    require(!p.target(10.,std::nan(""),11.),"NaN target accepted");
    require(!p.target(10.,1.1,11.),"position target limit");
    require(!p.target(10.01,.2,11.),"future target accepted");
    require(!p.target(10.,.2,10.),"expired target accepted");
    require(p.target(10.,.2,11.),"valid target rejected");
    require(!p.target(9.9,.1,11.),"unordered target accepted");
    require(p.step(10.),"duplicate clock rejected");close(p.state().q,0.);
    require(!p.step(10.2)&&p.faulted(),"overrun not latched");
    require(!p.target(10.,.1,11.)&&!p.step(10.21),"fault resumed without reset");
    require(p.reset(20.),"fault reset");
    require(!p.step(19.99)&&p.faulted(),"backward clock accepted");
    require(p.reset(-10.)&&p.target(-10.,.1,-9.),"negative clock domain rejected");
    require(!p.step(std::nan(""))&&p.faulted(),"NaN clock accepted");
}
void settle_and_boundaries() {
    for(double goal:{-1.,-.8,-.001,0.,.001,.8,1.}) {
        AdaptiveStream p;require(p.reset(0.),"reset");
        require(p.target(0.,goal,10.),"target");
        std::vector<Point> trace{{0.,0.}};
        for(int i=1;i<=600;++i) {
            const double t=i*.01;
            require(p.step(t),"settling step");valid({},p.state());trace.push_back({t,p.state().q});
            // Identical fresh targets must not restart capture.
            require(p.target(t,goal,t+1.),"renew constant target");
            if(i>=500) {close(p.state().q,goal);close(p.state().v,0.);close(p.state().a,0.);}
        }
        realized_bounds({},trace);
        // The settled fast path must still expire and accept a later target.
        require(p.target(6.,goal,6.025),"settled TTL update");
        require(p.step(6.01)&&p.step(6.03),"settled expiry");
        require(p.stopped(),"settled reference ignored expiry");
        const double next=goal>0.?-.1:.1;
        require(p.target(6.03,next,8.),"settled retarget");
        for(int i=1;i<=150;++i) {
            require(p.step(6.03+i*.01),"settled resume step");valid({},p.state());
        }
        close(p.state().q,next);close(p.state().v,0.);close(p.state().a,0.);
    }
}
void stop_and_expiry() {
    // Expiry inside a tick must match advancing exactly to the deadline and
    // explicitly stopping there, rather than extending authority to the tick.
    AdaptiveStream ttl,manual;
    require(ttl.reset(0.)&&manual.reset(0.),"expiry comparison reset");
    require(ttl.target(0.,.8,.235)&&manual.target(0.,.8,2.),"expiry comparison target");
    for(int i=1;i<=23;++i) require(ttl.step(i*.01)&&manual.step(i*.01),"expiry comparison step");
    require(manual.step(.235),"deadline step");manual.stop();
    require(manual.step(.24)&&ttl.step(.24),"split expiry step");
    close(ttl.state().q,manual.state().q);close(ttl.state().v,manual.state().v);close(ttl.state().a,manual.state().a);
    AdaptiveStream seeded;require(seeded.reset(0.,{0.,.5,2.,0.}),"moving seed");
    require(!seeded.stopped(),"moving seed reported stopped");
    require(seeded.step(.01),"untargeted moving seed step");
    require(seeded.state().q>0.,"moving seed froze without braking");
    for(int i=2;i<=100;++i) {require(seeded.step(i*.01),"seeded braking");valid({},seeded.state());}
    require(seeded.stopped(),"untargeted moving seed did not stop");
    for(bool expire:{false,true}) {
        AdaptiveStream p;require(p.reset(0.),"reset");
        require(p.target(0.,.8,expire?.235:2.),"target");
        std::vector<Point> trace{{0.,0.}};
        for(int i=1;i<=150;++i) {
            const double t=i*.01;
            require(p.step(t),"stop step");valid({},p.state());trace.push_back({t,p.state().q});
            if(!expire&&i==23) {p.stop();p.stop();require(!p.target(t,.7,t+1.),"restart during stop");}
        }
        require(p.stopped(),"stop did not finish");close(p.state().v,0.);close(p.state().a,0.);
        require(p.state().q<.8,"expired target reached");realized_bounds({},trace);
        const auto q=p.state().q;require(p.step(1.51),"hold step");close(p.state().q,q);
        p.stop();require(p.step(1.52),"idempotent stop after completion");close(p.state().q,q);
        require(p.target(1.52,.1,3.),"restart after stop");
        for(int i=153;i<=200;++i) {require(p.step(i*.01),"resumed step");trace.push_back({i*.01,p.state().q});}
        realized_bounds({},trace);
    }
}
void jerk_integration() {
    AdaptiveStream p;require(p.reset(0.),"reset");require(p.target(0.,.9,1.),"target");
    State previous=p.state();require(p.step(.001),"tiny step");auto next=p.state();
    close(next.a,previous.a+next.j*.001);
    close(next.v,previous.v+previous.a*.001+next.j*.001*.001/2);
    close(next.q,previous.q+previous.v*.001+previous.a*.001*.001/2+next.j*.001*.001*.001/6);
}
void braking_states() {
    int viable=0,rejected=0;
    for(double q:{-.99,-.5,0.,.5,.99}) for(double v:{-1.5,-.5,0.,.5,1.5})
    for(double a:{-20.,-5.,0.,5.,20.}) {
        Brake b;
        if(!AdaptiveStream::brake({}, {q,v,a,0.},b)) {++rejected;continue;}
        ++viable;valid({},b.final);close(b.final.v,0.);close(b.final.a,0.);
        for(int i=0;i<=100;++i) valid({},b.sample(b.duration*i/100));
        close(b.sample(0.).q,q);close(b.sample(0.).v,v);close(b.sample(0.).a,a);
    }
    require(viable>50&&rejected>0,"brake viability coverage");
    std::cout<<"brake states: "<<viable<<" viable, "<<rejected<<" rejected\n";
}
void randomized_streams() {
    std::mt19937 rng(1870);std::uniform_real_distribution<double> goal(-1.,1.);
    unsigned total=0;
    for(int profile=0;profile<8;++profile) {
        Limits l;l.velocity=.3+.2*profile;l.acceleration=2.+3*profile;l.jerk=20.+60*profile;
        AdaptiveStream p(l);require(p.reset(0.),"reset");double t=0.;bool stopping=false;
        std::vector<Point> trace{{0.,0.}};
        for(int i=0;i<4000;++i) {
            t+=i%2?.014:.006;require(p.step(t),"randomized stream step at "+std::to_string(t));
            valid(l,p.state());trace.push_back({t,p.state().q});
            if(stopping&&p.stopped()) stopping=false;
            if(i%211==200) {p.stop();stopping=true;}
            if(i%7==0&&!stopping) require(p.target(t,goal(rng),t+1.),"random target");
        }
        p.stop();for(int i=0;i<300;++i) {t+=.01;require(p.step(t),"random stop");valid(l,p.state());trace.push_back({t,p.state().q});}
        require(p.stopped(),"random stop unfinished");realized_bounds(l,trace);total+=trace.size();
    }
    std::cout<<"randomized samples: "<<total<<"\n";
}

void input_rates() {
    for(int hz:{5,10,30,60,100,200}) {
        AdaptiveStream p;require(p.reset(0.),"rate reset");
        double next_update=0.,error=0.;int updates=0;
        std::vector<Point> trace{{0.,0.}};
        // Input arrival frequency is independent of the 1 kHz test sampler.
        // This tests timestamp semantics, not OS or device real-time timing.
        for(int i=1;i<=6000;++i) {
            const double t=i*.001,q=.3*std::sin(2*t);
            require(p.step(t),"rate step "+std::to_string(hz)+" at "+std::to_string(t));
            valid({},p.state());trace.push_back({t,p.state().q});
            if(t+1e-12>=next_update) {
                const auto before=p.state();
                require(p.target(t,q,t+2./hz+.05),"rate target");
                close(before.q,p.state().q);close(before.v,p.state().v);close(before.a,p.state().a);
                ++updates;next_update+=1./hz;
            }
            error+=(p.state().q-q)*(p.state().q-q);
        }
        realized_bounds({},trace);
        p.stop();
        for(int i=1;i<=200;++i) require(p.step(6.+i*.01),"rate stop");
        require(p.stopped(),"rate stop did not finish");
        std::cout<<"stream_hz="<<hz<<" updates="<<updates<<" rms="<<std::sqrt(error/6000)<<'\n';
    }
}

void p2p_matrix() {
    int cases=0;
    for(double dt:{.001,.01,.02}) for(double speed:{.3,1.5})
    for(double goal:{-1.,-.001,0.,.001,1.}) {
        Limits l;l.velocity=speed;AdaptiveStream p(l);
        require(p.reset(0.)&&p.target(0.,goal,12.),"P2P admission");
        std::vector<Point> trace{{0.,0.}};
        for(int i=1;i<=static_cast<int>(10./dt);++i) {
            const double t=i*dt;
            require(p.step(t),"P2P step dt="+std::to_string(dt)+" speed="+std::to_string(speed)+" goal="+std::to_string(goal));
            valid(l,p.state());trace.push_back({t,p.state().q});
            if(t>=9.) {close(p.state().q,goal);close(p.state().v,0.);close(p.state().a,0.);}
        }
        realized_bounds(l,trace);++cases;
        // Reaching a reference is not task completion or feedback confirmation.
        require(!p.stopped(),"active goal unexpectedly released on arrival");
    }
    std::cout<<"untimed_p2p_cases="<<cases<<'\n';
}

void stream_p2p_transitions() {
    AdaptiveStream p;require(p.reset(0.)&&p.target(0.,.8,6.),"transition initial P2P");
    std::vector<Point> trace{{0.,0.}};
    for(int i=1;i<=500;++i) {
        const double t=i*.01;
        require(p.step(t),"transition step");valid({},p.state());trace.push_back({t,p.state().q});
        if(i>=40&&i<200) {
            const auto before=p.state();
            require(p.target(t,.25*std::sin(3*t),t+.2),"transition stream");
            close(before.q,p.state().q);close(before.v,p.state().v);close(before.a,p.state().a);
        }
        if(i==200) require(p.target(t,-.2,6.),"transition final P2P");
        if(i>=400) {close(p.state().q,-.2);close(p.state().v,0.);close(p.state().a,0.);}
    }
    realized_bounds({},trace);
    std::cout<<"untimed_p2p_to_stream_to_p2p=PASS\n";
}

void capability_boundaries() {
    // A deadline is not represented in this API. TTL must not be advertised
    // as a requested arrival time, or a future stamp as a trajectory queue.
    AdaptiveStream p;require(p.reset(0.),"capability reset");
    require(!p.target(1.,.2,2.),"unexpected future target support");
    require(p.target(0.,.2,2.),"TTL target");double arrived=-1.;
    for(int i=1;i<=150;++i) {
        const double t=i*.01;require(p.step(t),"TTL witness step");
        if(arrived<0.&&std::abs(p.state().q-.2)<1e-8&&p.state().v==0.&&p.state().a==0.) arrived=t;
    }
    require(arrived>0.&&arrived<1.5,"TTL versus arrival witness");
    std::cout<<"ttl=2 reference_arrival="<<arrived<<" future_target=rejected\n";

    // Seven independent scalar instances are not a synchronized group planner.
    std::vector<AdaptiveStream> axes;std::vector<double> arrivals(7,-1.);
    for(int axis=0;axis<7;++axis) {
        Limits l;l.velocity=.2+.2*axis;axes.emplace_back(l);
        require(axes.back().reset(0.)&&axes.back().target(0.,.4,7.),"scalar array target");
    }
    for(int i=1;i<=600;++i) for(int axis=0;axis<7;++axis) {
        const double t=i*.01;auto& a=axes[axis];require(a.step(t),"scalar array step");
        Limits l;l.velocity=.2+.2*axis;valid(l,a.state());
        if(arrivals[axis]<0.&&std::abs(a.state().q-.4)<1e-8&&a.state().v==0.&&a.state().a==0.) arrivals[axis]=t;
    }
    for(double t:arrivals) require(t>0.,"scalar axis did not arrive");
    const double spread=*std::max_element(arrivals.begin(),arrivals.end())-*std::min_element(arrivals.begin(),arrivals.end());
    require(spread>.1,"expected unsynchronized scalar arrival witness");
    std::cout<<"independent_scalar_arrival_span="<<spread<<" (no group synchronization)\n";
    std::cout<<"SCALAR_ONLY: no timed/group API; these belong to the shared native engine and execution shell\n";
}

JointState joint_rest(size_t n,double q=0.) {
    return {std::vector<double>(n,q),std::vector<double>(n),std::vector<double>(n),std::vector<double>(n)};
}
void valid_group(const JointLimits& limits,const JointState& s) {
    require(s.q.size()==limits.size(),"native group dimensions");
    for(size_t i=0;i<limits.size();++i) valid(limits[i],{s.q[i],s.v[i],s.a[i],s.j[i]});
}
template<class F> void rejected(F&& f,const char* message) {
    bool failed=false;try {f();}catch(const std::invalid_argument&){failed=true;}require(failed,message);
}
void native_timed_group() {
    JointLimits limits(7);for(size_t i=0;i<7;++i) limits[i].velocity=.3+.15*i;
    auto zero=joint_rest(7),goal=joint_rest(7,.2);
    auto p2p=fit_timed(limits,zero,{goal},{2.},0.,false,5.);
    close(p2p.back()->end(),2.);
    for(int k=0;k<=1000;++k) valid_group(limits,sample(p2p,k*.002));
    const auto end=sample(p2p,2.);
    for(size_t i=0;i<7;++i) {close(end.q[i],.2);close(end.v[i],0.);close(end.a[i],0.);}
    auto waypoint=joint_rest(7,.05);waypoint.v.assign(7,.1);
    auto trajectory=fit_timed(limits,zero,{waypoint,goal},{1.,2.},0.,false,5.);
    auto at=sample(trajectory,1.);
    for(size_t i=0;i<7;++i) {close(at.q[i],.05);close(at.v[i],.1);}
    rejected([&]{fit_timed(limits,zero,{goal},{.01},0.,false,5.);},"impossible timed P2P admitted");
    rejected([&]{fit_timed(limits,zero,{waypoint},{1.},0.,false,5.);},"nonrest finite endpoint admitted");
    rejected([&]{fit_timed(limits,zero,{goal,goal},{1.,1.},0.,false,5.);},"duplicate knot time admitted");
    rejected([&]{fit_timed(limits,zero,{goal,goal},{1.,1.000001},0.,true,5.);},"short future interval silently discarded");
    std::cout<<"native_seven_axis_timed_p2p_and_derivatives=PASS\n";
}
void native_chunks() {
    JointLimits limits(3);auto zero=joint_rest(3),one=joint_rest(3,.1),two=joint_rest(3,.2);
    two.v.assign(3,.15);
    auto path=fit_timed(limits,zero,{one,one,one,two},{-.2,-.1,.5,1.},0.,true,5.);
    close(path.front()->end(),.5);require(path.back()->end()>1.,"chunk has no brake tail");
    const auto knot=sample(path,1.);for(size_t i=0;i<3;++i) {close(knot.q[i],.2);close(knot.v[i],.15);}
    for(int k=0;k<=1000;++k) valid_group(limits,sample(path,path.back()->end()*k/1000));
    auto end=sample(path,path.back()->end());
    for(size_t i=0;i<3;++i) {close(end.v[i],0.);close(end.a[i],0.);}
    rejected([&]{fit_timed(limits,zero,{one},{-.1},0.,true,5.);},"expired chunk admitted");
    std::cout<<"native_chunk_timeline_and_stop_tail=PASS\n";
}
void native_online_sampling() {
    JointLimits limits(3);auto zero=joint_rest(3);
    const std::vector<double> goal{.8,-.4,.001};
    Online a(limits,zero,goal,0.,.01,30.),b(limits,zero,goal,0.,.01,30.);
    a.sample(.5); // A future preview must not change any earlier reference.
    const auto boundary=b.sample(.01),left=b.sample(.01-1e-7);
    close((boundary.a[0]-left.a[0])/1e-7,boundary.j[0],1e-5);
    std::vector<Point> trace{{0.,0.}};
    for(int i=1;i<=2000;++i) {
        const double t=i*.001;auto x=a.sample(t),y=b.sample(t);valid_group(limits,x);
        for(size_t j=0;j<3;++j) {close(x.q[j],y.q[j]);close(x.v[j],y.v[j]);close(x.a[j],y.a[j]);close(x.j[j],y.j[j]);}
        trace.push_back({t,x.q[0]});
    }
    realized_bounds(limits[0],trace);
    for(int i=1;i<=90;++i) {
        const double t=i*.01;auto current=a.sample(t);
        Curves stop;
        try {stop=fit_stop(limits,current,t,5.);}
        catch(const std::invalid_argument&) {
            std::cerr<<std::setprecision(17)<<"brake failure at "<<t<<'\n';
            for(size_t axis=0;axis<limits.size();++axis) {
                Brake brake;
                std::cerr<<"q/v/a="<<current.q[axis]<<'/'<<current.v[axis]<<'/'<<current.a[axis]
                         <<" scalar_brake="<<AdaptiveStream::brake(limits[axis],{current.q[axis],current.v[axis],current.a[axis],0.},brake)<<'\n';
            }
            throw;
        }
        for(const auto& c:stop) require(c->valid(limits),"native online brake certification");
        const auto stopped=sample(stop,stop.back()->end());
        for(size_t j=0;j<3;++j) {close(stopped.v[j],0.);close(stopped.a[j],0.);}
    }
    for(int i=1;i<35;++i) {
        const double t=i*.013+.000123,h=1e-6;
        const auto left=a.sample(t-h),middle=a.sample(t),right=a.sample(t+h);
        for(size_t j=0;j<3;++j) {
            close((right.q[j]-left.q[j])/(2*h),middle.v[j],1e-5);
            close((right.v[j]-left.v[j])/(2*h),middle.a[j],1e-5);
        }
    }
    const auto end=a.sample(2.);for(size_t j=0;j<3;++j) {close(end.q[j],goal[j]);close(end.v[j],0.);}
    std::cout<<"native_online_multiaxis_preview_and_derivatives=PASS\n";
}
void native_mode_connection() {
    JointLimits limits(1);Online online(limits,joint_rest(1),{.6},0.,.01,30.);
    const auto initial=online.sample(.1);
    auto knot=joint_rest(1,.2);knot.v[0]=.2;
    auto timed=fit_timed(limits,initial,{knot,joint_rest(1,.3)},{.6,1.1},.1,false,5.);
    auto first=sample(timed,.1);close(first.q[0],initial.q[0]);close(first.v[0],initial.v[0]);close(first.a[0],initial.a[0]);
    close(sample(timed,.6).q[0],.2);close(sample(timed,.6).v[0],.2);close(timed.back()->end(),1.1);
    for(const auto& curve:timed) require(curve->valid(limits),"mode connection limit");
    const auto next_initial=sample(timed,.3);Online next(limits,next_initial,{-.2},.3,.01,30.);
    close(next.sample(.3).q[0],next_initial.q[0]);close(next.sample(2.).q[0],-.2);
    std::cout<<"native_online_timed_online_connection=PASS\n";
}
int main() {
    try {
        input_checks();settle_and_boundaries();stop_and_expiry();jerk_integration();braking_states();randomized_streams();
        input_rates();p2p_matrix();stream_p2p_transitions();capability_boundaries();
        native_timed_group();native_chunks();native_online_sampling();native_mode_connection();
        std::cout<<"PASS: 14 native reference test groups\n";
    } catch(const std::exception& e) {std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}
}
