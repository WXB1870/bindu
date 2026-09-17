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
int main() {
    try {
        input_checks();settle_and_boundaries();stop_and_expiry();jerk_integration();braking_states();randomized_streams();
        std::cout<<"PASS: 6 experimental adaptive stream test groups\n";
    } catch(const std::exception& e) {std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}
}
