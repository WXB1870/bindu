#include "guard_118.h"
#include <cassert>
#include <iostream>
#include <vector>
using namespace bindu_guard118;
constexpr int64_t ros = 1700000000000000000LL, t = 1000000000;
void arm(Guard& g) {
    g.enable();
    assert(!g.heartbeat(0, 1, ros, ros, t, true));
    assert(!g.heartbeat(99, 1, ros, ros, t, false));
    assert(g.heartbeat(99, 1, ros, ros, t, true));
}
int main() {
    {
        Guard g;
        assert(!g.blocked(t, false));
        g.enable();
        assert(g.blocked(t, true) && !g.latched() && !g.armed());
        assert(!g.accepts(0, {ros,1,99,false}, ros,t,true));
        arm(g);
        assert(!g.heartbeat(98, 2, ros, ros,t,true));
        assert(!g.heartbeat(99, 1, ros, ros,t,true));
        assert(!g.heartbeat(99, 2, ros+1, ros,t,true));
        assert(!g.heartbeat(99, 2, ros-timeout_ns, ros,t,true));
        assert(!g.blocked(t+timeout_ns-1,true));
        assert(g.blocked(t+timeout_ns,true));
        assert(!g.heartbeat(99, 2, ros+timeout_ns, ros+timeout_ns,t+timeout_ns,true));
        assert(g.latched());
    }
    for (unsigned c=0; c<channels; ++c) {
        Guard g; arm(g);
        const Command cmd{ros, 1, 99, false};
        assert(g.accepts(c, cmd, ros, t, true));
        assert(g.touched_mask() == 0); // checking is not committing an enable
        assert(g.commit(c, cmd, ros, t, true));
        assert(g.touched_mask() == masks[c]);
        assert(g.commit(c, cmd, ros+100000000, t+100000000, true));
        assert(!g.accepts(c, {ros+1,1,99,true},ros+1,t+1,true)); // modified replay
        assert(!g.accepts(c, {ros,2,98,true},ros,t,true)); // wrong session
        assert(!g.accepts(channels, cmd,ros,t,true));
        assert(g.heartbeat(99,2,ros+150000000,ros+150000000,t+150000000,true));
        assert(!g.blocked(t+timeout_ns-1,true));
        assert(g.blocked(t+timeout_ns,true)); // repeated middle frames did not renew
        assert(!g.commit(c,{ros+timeout_ns,2,99,true},ros+timeout_ns,t+timeout_ns,true));
    }
    {
        Guard g; arm(g);
        assert(g.commit(0,{ros,1,99,false},ros,t,true));
        assert(g.commit(0,{ros+10000000,2,99,true},ros+10000000,t+10000000,true));
        assert(g.hold_mask() == masks[0]);
        for (int i=1;i<=30;++i) {
            const int64_t dt=int64_t(i)*50000000;
            assert(g.heartbeat(99,i+1,ros+dt,ros+dt,t+dt,true));
            assert(!g.blocked(t+dt,true)); // explicit hold permits ordinary idle
        }
        assert(g.commit(0,{ros+1500000000,3,99,false},ros+1500000000,t+1500000000,true));
        assert(g.hold_mask() == 0);
        assert(g.blocked(t+1500000001,false)); // bad PDO trips even during a hold
        assert(!g.heartbeat(99,32,ros+1500000002,ros+1500000002,t+1500000002,true));
    }
    {
        Guard g; arm(g);
        assert(g.commit(0,{ros-190000000,1,99,false},ros,t,true));
        assert(!g.blocked(t+9999999,true));
        assert(g.blocked(t+10000000,true)); // queue time consumes source TTL
    }
    {
        Command c;
        std::vector<double> v{-.2,0.,0.,100.,30.,1700000000.,123.,4294967295.,4294967294.,1.};
        assert(from_array(v,5,c));
        assert(c.session == UINT64_MAX-1 && c.sequence == 123 && c.hold);
        assert(c.stamp_ns == ros);
        for (size_t i=0;i<v.size();++i) {
            auto bad=v;bad[i]=std::numeric_limits<double>::quiet_NaN();
            assert(!from_array(bad,5,c));
        }
        for (size_t i=6;i<v.size();++i) {
            auto bad=v;bad[i]=-.1;assert(!from_array(bad,5,c));
        }
        auto bad=v;bad[8]=4294967296.;assert(!from_array(bad,5,c));
        bad=v;bad[9]=2;assert(!from_array(bad,5,c));
        v.resize(5);assert(!from_array(v,5,c)); // legacy prefix cannot bypass guard
    }
    {
        Hold h;
        h.capture(.4f,.2f,100.f,30.f,5.f);
        h.capture(.8f,.9f,0.f,0.f,0.f);
        assert(h.position==.4f && h.kp==100.f && h.kd==30.f && h.torque==5.f);
        Hold fallback;
        fallback.capture(std::numeric_limits<float>::quiet_NaN(),.25f,1,2,3);
        assert(fallback.position==.25f);
    }
    std::cout << "118 guard deadline, replay, hold and bus-fault checks passed\n";
}
