// Standalone experimental-kernel tests. No ROS, device SDK, or historical source.
#include "adaptive_stream.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>
using namespace bindu::experimental;
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
    std::cout<<"UNSUPPORTED: timed P2P, derivative waypoints, action chunks/revisions, group synchronization, feedback completion\n";
}
int main() {
    try {
        input_checks();settle_and_boundaries();stop_and_expiry();jerk_integration();braking_states();randomized_streams();
        input_rates();p2p_matrix();stream_p2p_transitions();capability_boundaries();
        std::cout<<"PASS: 10 experimental adaptive stream test groups; capability gaps remain\n";
    } catch(const std::exception& e) {std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}
}
