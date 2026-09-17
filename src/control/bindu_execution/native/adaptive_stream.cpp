// Proposal equations adapted from historical legs_necks_control, preserved there.
// Shared scalar primitive for the native execution module.
#include "adaptive_stream.hpp"
#include "curve_math.hpp"
#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <limits>

namespace bindu::execution {
namespace {

constexpr double kEpsilon = 1e-6;

double limitedBy(double value, double max_abs_value)
{
    return std::clamp(value, -max_abs_value, max_abs_value);
}

double polynomialValue(double t, const std::vector<double>& coefficients)
{
    double result = 0.0;
    double t_power = 1.0;
    for (double coefficient : coefficients)
    {
        result += coefficient * t_power;
        t_power *= t;
    }
    return result;
}

std::vector<double> polynomialDerivative(const std::vector<double>& coefficients)
{
    if (coefficients.size() <= 1)
    {
        return {0.0};
    }

    std::vector<double> derivative;
    derivative.reserve(coefficients.size() - 1);
    for (std::size_t index = 1; index < coefficients.size(); ++index)
    {
        derivative.push_back(coefficients[index] * static_cast<double>(index));
    }
    return derivative;
}

double newtonRaphson(
    const std::function<double(double)>& function,
    const std::function<double(double)>& derivative,
    double initial,
    int max_iterations = 50,
    double tolerance = 1e-6)
{
    double x = initial;
    for (int iteration = 0; iteration < max_iterations; ++iteration)
    {
        const double fx = function(x);
        const double dfx = derivative(x);
        if (std::abs(dfx) < tolerance)
        {
            break;
        }

        const double next_x = x - fx / dfx;
        if (std::abs(next_x - x) < tolerance)
        {
            x = next_x;
            break;
        }
        x = next_x;
    }
    return x;
}

std::pair<double, double> solveSuggestedVelocityAndAcceleration(
    double distance,
    double max_velocity,
    double max_acceleration,
    double max_jerk)
{
    if (max_acceleration * max_acceleration >= max_velocity * max_jerk)
    {
        const double d1 = max_jerk * std::pow(max_velocity / max_jerk, 1.5) / 3.0;
        const double d2 = max_velocity * std::sqrt(max_velocity / max_jerk);
        if (distance >= d2)
        {
            return {max_velocity, 0.0};
        }

        if (distance >= d1)
        {
            const std::vector<double> coefficients {
                max_jerk * std::pow(max_velocity / max_jerk, 1.5) / 3.0,
                -max_velocity,
                max_jerk * std::sqrt(max_velocity / max_jerk),
                -max_jerk / 6.0,
            };
            const auto derivative = polynomialDerivative(coefficients);
            const auto function = [&](double t) { return polynomialValue(t, coefficients) - distance; };
            const auto function_derivative = [&](double t) {
                return polynomialValue(t, derivative);
            };

            const double t_suggested =
                newtonRaphson(function, function_derivative, max_acceleration / max_jerk);
            const double velocity_suggested =
                -max_jerk * t_suggested * t_suggested / 2.0 +
                2.0 * max_jerk * t_suggested * std::sqrt(max_velocity / max_jerk) -
                2.0 * max_jerk * max_velocity / max_jerk +
                max_velocity;
            const double acceleration_suggested =
                max_jerk * std::sqrt(max_velocity / max_jerk) -
                max_jerk * (t_suggested - std::sqrt(max_velocity / max_jerk));
            return {velocity_suggested, acceleration_suggested};
        }

        const double t_suggested = std::pow(6.0, 1.0 / 3.0) * std::pow(distance / max_jerk, 1.0 / 3.0);
        return {
            max_jerk * t_suggested * t_suggested / 2.0,
            max_jerk * t_suggested,
        };
    }

    const double t2 = max_velocity / max_acceleration;
    const double d1 = std::pow(max_acceleration, 3) / (6.0 * max_jerk * max_jerk);
    const double d2 =
        std::pow(max_acceleration, 3) / (6.0 * max_jerk * max_jerk) -
        max_acceleration * max_velocity / (2.0 * max_jerk) +
        (max_velocity * max_velocity) / (2.0 * max_acceleration);
    const double d3 =
        max_velocity * (max_acceleration * max_acceleration + max_jerk * max_velocity) /
        (2.0 * max_acceleration * max_jerk);

    if (distance >= d3)
    {
        return {max_velocity, 0.0};
    }

    if (distance >= d2)
    {
        const std::vector<double> coefficients {
            (std::pow(max_acceleration, 6) + std::pow(max_jerk, 3) * std::pow(max_velocity, 3)) /
                (6.0 * std::pow(max_acceleration, 3) * max_jerk * max_jerk),
            (-(std::pow(max_acceleration, 4)) - max_jerk * max_jerk * max_velocity * max_velocity) /
                (2.0 * max_acceleration * max_acceleration * max_jerk),
            (max_acceleration * max_acceleration + max_jerk * max_velocity) /
                (2.0 * max_acceleration),
            -max_jerk / 6.0,
        };
        const auto derivative = polynomialDerivative(coefficients);
        const auto function = [&](double t) { return polynomialValue(t, coefficients) - distance; };
        const auto function_derivative = [&](double t) {
            return polynomialValue(t, derivative);
        };

        const double t_suggested = newtonRaphson(function, function_derivative, t2);
        const double velocity_suggested =
            -(max_acceleration * max_acceleration) / (2.0 * max_jerk) +
            max_acceleration * t_suggested -
            max_jerk * t_suggested * t_suggested / 2.0 +
            max_jerk * t_suggested * max_velocity / max_acceleration -
            max_jerk * max_velocity * max_velocity / (2.0 * max_acceleration * max_acceleration);
        const double acceleration_suggested =
            max_acceleration - max_jerk * (t_suggested - max_velocity / max_acceleration);
        return {velocity_suggested, acceleration_suggested};
    }

    if (distance >= d1)
    {
        const double discriminant =
            3.0 * max_acceleration * max_acceleration +
            std::sqrt(3.0) *
                std::sqrt(max_acceleration * (-std::pow(max_acceleration, 3) + 24.0 * distance * max_jerk * max_jerk));
        const double t_suggested = discriminant / (6.0 * max_acceleration * max_jerk);
        return {
            -(max_acceleration * max_acceleration) / (2.0 * max_jerk) + max_acceleration * t_suggested,
            max_acceleration,
        };
    }

    const double t_suggested = std::pow(6.0, 1.0 / 3.0) * std::pow(distance / max_jerk, 1.0 / 3.0);
    return {
        max_jerk * t_suggested * t_suggested / 2.0,
        max_jerk * t_suggested,
    };
}

double computeBestAcceleration(
    double position,
    double velocity,
    double acceleration,
    double target_position,
    double target_velocity,
    double frequency_hz,
    double max_velocity,
    double max_acceleration,
    double adjust_factor)
{
    double distance_half =
        target_position - (position + position + velocity / frequency_hz + 0.5 * acceleration / (frequency_hz * frequency_hz)) / 2.0;
    const int direction = distance_half < 0.0 ? -1 : 1;
    distance_half *= static_cast<double>(direction);
    velocity *= static_cast<double>(direction);
    target_velocity *= static_cast<double>(direction);

    velocity -= target_velocity;
    const double adjusted_max_velocity = std::max(max_velocity - target_velocity, 0.0);
    const double velocity_suggested =
        limitedBy(std::sqrt(std::max(0.0, 2.0 * max_acceleration * distance_half)), adjusted_max_velocity);
    const double limited_acceleration =
        limitedBy((velocity_suggested - velocity) * frequency_hz, max_acceleration * adjust_factor);
    return limited_acceleration * static_cast<double>(direction);
}

double computeBestJerk(
    double position,
    double velocity,
    double acceleration,
    double target_position,
    double frequency_hz,
    double max_velocity,
    double max_acceleration,
    double max_jerk)
{
    double distance_half =
        target_position - (position + position + velocity / frequency_hz + 0.5 * acceleration / (frequency_hz * frequency_hz)) / 2.0;
    const int direction = distance_half < 0.0 ? -1 : 1;
    distance_half *= static_cast<double>(direction);
    velocity *= static_cast<double>(direction);
    acceleration *= static_cast<double>(direction);
    auto [velocity_suggested, acceleration_suggested] = solveSuggestedVelocityAndAcceleration(
        distance_half,
        max_velocity,
        max_acceleration,
        max_jerk);
    acceleration_suggested = -acceleration_suggested;

    const double jerk = computeBestAcceleration(
        velocity,
        acceleration,
        0.0,
        velocity_suggested,
        acceleration_suggested * (std::abs(velocity_suggested) < kEpsilon
                ? 1.0
                : std::clamp(velocity / velocity_suggested, 2.0 / 3.0, 1.0)),
        frequency_hz,
        max_acceleration,
        max_jerk,
        1.1);
    return jerk * static_cast<double>(direction);
}

}  // namespace

namespace {
State integrate(State s, double j, double dt) {
    return {s.q+s.v*dt+s.a*dt*dt/2+j*dt*dt*dt/6,
            s.v+s.a*dt+j*dt*dt/2, s.a+j*dt, j};
}
bool finite(State s) {
    return std::isfinite(s.q)&&std::isfinite(s.v)&&std::isfinite(s.a)&&std::isfinite(s.j);
}
bool valid_state(const Limits& l, State s) {
    return finite(s)&&s.q>=l.lower-1e-10&&s.q<=l.upper+1e-10&&
           std::abs(s.v)<=l.velocity+1e-10&&std::abs(s.a)<=l.acceleration+1e-10&&
           std::abs(s.j)<=l.jerk+1e-9;
}
using detail::deriv;
using detail::eval;
using detail::bounded;
bool curve_valid(const Limits& l, std::vector<double> p, double t) {
    const double lo[]={l.lower,-l.velocity,-l.acceleration,-l.jerk};
    const double hi[]={l.upper,l.velocity,l.acceleration,l.jerk};
    for (int i=0;i<4;++i) {
        if (!bounded(p,lo[i],hi[i])) return false;
        if(i<3) p=deriv(p,t);
    }
    return true;
}
bool phase_valid(const Limits& l, const Phase& p) {
    const auto s=p.initial;const double t=p.duration,j=p.jerk;
    constexpr double eps=1e-11;
    if(!finite(s)||!std::isfinite(t)||t<=0.||!std::isfinite(j)||std::abs(j)>l.jerk) return false;
    auto position_ok=[&](double u) {const double q=integrate(s,j,u).q;
        return std::isfinite(q)&&q>=l.lower-eps&&q<=l.upper+eps;};
    auto velocity_ok=[&](double u) {const double v=integrate(s,j,u).v;
        return std::isfinite(v)&&std::abs(v)<=l.velocity+eps;};
    const auto end=integrate(s,j,t);
    if(!position_ok(0.)||!position_ok(t)||!velocity_ok(0.)||!velocity_ok(t)||
       std::abs(s.a)>l.acceleration+eps||std::abs(end.a)>l.acceleration+eps) return false;
    // Exact extrema of the cubic position/quadratic velocity. A finite-depth
    // hull test can spuriously reject near a saturated velocity plateau.
    if(j!=0.) {
        const double extremum=-s.a/j;
        if(extremum>0.&&extremum<t&&!velocity_ok(extremum)) return false;
        const double disc=s.a*s.a-2*j*s.v;
        if(disc>=0.) {
            const double q=-.5*(s.a+std::copysign(std::sqrt(disc),s.a));
            const double roots[]={2*q/j,q!=0.?s.v/q:0.};
            for(double u:roots) if(u>0.&&u<t&&!position_ok(u)) return false;
        }
    } else if(s.a!=0.) {
        const double u=-s.v/s.a;
        if(u>0.&&u<t&&!position_ok(u)) return false;
    }
    return true;
}
State sample_curve(std::vector<double> p,double t,double elapsed) {
    const double u=std::clamp(elapsed/t,0.,1.);
    double value[4];
    for(int i=0;i<4;++i) {
        value[i]=eval(p,u);
        if(i<3) p=deriv(p,t);
    }
    return {value[0],value[1],value[2],value[3]};
}
}

State Brake::sample(double elapsed) const {
    for (const auto& p:phases) {
        if (elapsed<p.duration) return integrate(p.initial,p.jerk,std::max(0.,elapsed));
        elapsed-=p.duration;
    }
    return final;
}

bool AdaptiveStream::brake(const Limits& l, State s, Brake& out) {
    if(!std::isfinite(l.lower)||!std::isfinite(l.upper)||l.lower>=l.upper||
       !std::isfinite(l.velocity)||l.velocity<=0.||!std::isfinite(l.acceleration)||l.acceleration<=0.||
       !std::isfinite(l.jerk)||l.jerk<=0.||!valid_state(l,s)) return false;
    out={};
    // Solve the velocity-to-zero problem with bounded acceleration and jerk.
    // The switch sign includes velocity accumulated while removing acceleration.
    const double equivalent=s.v+s.a*std::abs(s.a)/(2*l.jerk);
    const double direction=equivalent<0?-1.:1.;
    const double v=direction*s.v, a=direction*s.a;
    const double peak=std::sqrt(std::max(0.,l.jerk*v+a*a/2));
    const double am=std::min(peak,l.acceleration);
    const double t1=std::max(0.,(a+am)/l.jerk);
    const double t2=peak>l.acceleration?std::max(0.,(v+a*a/(2*l.jerk)-am*am/l.jerk)/am):0.;
    const double t3=am/l.jerk;
    const double times[]={t1,t2,t3}, jerks[]={-direction*l.jerk,0.,direction*l.jerk};
    for (int i=0;i<3;++i) {
        if(times[i]<1e-12) continue;
        Phase p{s,times[i],jerks[i]};
        if(!phase_valid(l,p)) return false;
        out.phases.push_back(p); out.duration+=times[i]; s=integrate(s,jerks[i],times[i]);
    }
    if(std::abs(s.v)>1e-8||std::abs(s.a)>1e-8||!valid_state(l,s)) return false;
    out.final={s.q,0.,0.,0.};
    return true;
}

bool AdaptiveStream::valid_phase(const Limits& limits,const Phase& phase) {
    return phase_valid(limits,phase);
}

AdaptiveStream::AdaptiveStream(Limits limits):limits_(limits) {
    const auto& l=limits_;
    const double positives[]={l.velocity,l.acceleration,l.jerk,l.max_gap,l.capture_distance,
                              l.capture_speed,l.capture_acceleration,l.capture_wait,l.capture_horizon};
    if(!std::isfinite(l.lower)||!std::isfinite(l.upper)||l.lower>=l.upper||
       !std::all_of(std::begin(positives),std::end(positives),[](double v){return std::isfinite(v)&&v>0.;}))
        throw std::invalid_argument("invalid stream limits");
}
bool AdaptiveStream::reset(double now, State state) {
    Brake candidate;
    if(!std::isfinite(now)||!brake(limits_,state,candidate)) return false;
    state_=state; clock_=now; safe_brake_=candidate; source_=-std::numeric_limits<double>::infinity(); guarded_=0;
    initialized_=true; fault_=tracking_=stopping_=capturing_=false;
    pieces_.clear();
    return true;
}
bool AdaptiveStream::target(double source, double position, double until) {
    if(!initialized_||fault_||stopping_||!std::isfinite(source)||!std::isfinite(position)||!std::isfinite(until)||
       source<source_||source>clock_+1e-9||until<=clock_||until<=source||
       position<limits_.lower||position>limits_.upper) return false;
    if(!tracking_||position!=goal_) {capturing_=false;changed_=clock_;}
    source_=source; goal_=position; until_=until; tracking_=true;
    return true;
}
void AdaptiveStream::stop() {
    if(!initialized_||fault_||stopping_) return;
    tracking_=capturing_=false; stopping_=true; brake_elapsed_=0.;
}
bool AdaptiveStream::capture() {
    const auto& s=state_; const auto& l=limits_;
    if(clock_-changed_<l.capture_wait||std::abs(goal_-s.q)>l.capture_distance||
       std::abs(s.v)>l.capture_speed||std::abs(s.a)>l.capture_acceleration) return false;
    for(double t=.02;t<=l.capture_horizon;t*=1.15) {
        std::vector<double> p{s.q,s.q+s.v*t/5,s.q+2*s.v*t/5+s.a*t*t/20,goal_,goal_,goal_};
        if(curve_valid(l,p,t)) {controls_=p;capture_duration_=t;capture_elapsed_=0.;capturing_=true;return true;}
    }
    return false;
}
bool AdaptiveStream::advance(double dt) {
    if(stopping_) {
        record_brake(safe_brake_,brake_elapsed_,dt);
        brake_elapsed_+=dt; state_=safe_brake_.sample(brake_elapsed_);
        if(brake_elapsed_>=safe_brake_.duration) {
            stopping_=false;safe_brake_={};safe_brake_.final=state_;
        }
        return true;
    }
    if(!tracking_) {
        if(state_.v!=0.||state_.a!=0.) {stop();return advance(dt);}
        return true;
    }
    Brake next_brake; State candidate;
    if(capturing_||capture()) {
        // The endpoint and its resting brake were certified when capture
        // completed. New targets clear capturing_; step() still checks TTL.
        if(capture_elapsed_>=capture_duration_) {
            record_cubic(state_,state_,dt,0.);return true;
        }
        const double elapsed=capture_elapsed_+dt;
        candidate=elapsed>=capture_duration_?State{goal_,0.,0.,0.}:sample_curve(controls_,capture_duration_,elapsed);
        if(brake(limits_,candidate,next_brake)) {
            const double moving=std::min(dt,capture_duration_-capture_elapsed_);
            pieces_.push_back({state_,candidate,moving,controls_,capture_duration_,capture_elapsed_});
            if(dt>moving) record_cubic(candidate,candidate,dt-moving,0.);
            state_=candidate; safe_brake_=next_brake; capture_elapsed_=elapsed;
            return true;
        }
        capturing_=false;
    }
    const auto& l=limits_;
    const double j=computeBestJerk(state_.q,state_.v,state_.a,goal_,1./dt,l.velocity,l.acceleration,l.jerk);
    if(!std::isfinite(j)) return false;
    const double low=std::max({-l.jerk,(-l.acceleration-state_.a)/dt,2*(-l.velocity-state_.v-state_.a*dt)/(dt*dt)});
    const double high=std::min({l.jerk,(l.acceleration-state_.a)/dt,2*(l.velocity-state_.v-state_.a*dt)/(dt*dt)});
    auto acceptable = [&](double jerk, State& next, Brake& braking) {
        const Phase p{state_,dt,jerk};next=integrate(state_,jerk,dt);
        if(!phase_valid(l,p)||!brake(l,next,braking)) return false;
        // For stationary targets, reserve enough distance to stop at the goal.
        return !(clock_-changed_>=l.capture_wait&&
                 (goal_-state_.q)*(braking.final.q-goal_)>1e-12);
    };
    if(low<=high) {
        const double wanted=std::clamp(j,low,high);
        if(acceptable(wanted,candidate,next_brake)) {
            record_cubic(state_,candidate,dt,wanted);
            state_=candidate; safe_brake_=next_brake;return true;
        }
        // Find a less disruptive admissible jerk between the heuristic proposal
        // and a known safe constant-jerk prefix. Every candidate is independently
        // certified; no convexity assumption is needed for safety.
        if(!safe_brake_.phases.empty()&&safe_brake_.phases.front().duration>=dt) {
            double safe=safe_brake_.phases.front().jerk, unsafe=wanted;
            State best;Brake best_brake;
            if(acceptable(safe,best,best_brake)) {
                for(int i=0;i<10;++i) {
                    const double middle=(safe+unsafe)/2;
                    if(acceptable(middle,candidate,next_brake)) {
                        safe=middle;best=candidate;best_brake=next_brake;
                    } else unsafe=middle;
                }
                record_cubic(state_,best,dt,safe);
                ++guarded_;state_=best;safe_brake_=best_brake;return true;
            }
        }
    }
    ++guarded_;
    candidate=safe_brake_.sample(dt);
    if(!brake(l,candidate,next_brake)) return false;
    record_brake(safe_brake_,0.,dt);
    state_=candidate; safe_brake_=next_brake; return true;
}
bool AdaptiveStream::step(double now) {
    if(!initialized_||fault_) return false;
    const double dt=now-clock_;
    if(!std::isfinite(now)||dt<0.||dt>limits_.max_gap+1e-9) {fault_=true;return false;}
    if(dt==0.) return true;
    pieces_.clear();
    // Split at expiry, so a delayed tick cannot extend target authority.
    if(tracking_&&now>=until_) {
        if(until_>clock_&&!advance(until_-clock_)) {fault_=true;return false;}
        const double remaining=now-std::max(until_,clock_);
        stop();
        if(!advance(remaining)) {fault_=true;return false;}
    } else if(!advance(dt)) {fault_=true;return false;}
    if(pieces_.empty()) record_cubic(state_,state_,dt,0.);
    clock_=now; return true;
}

void AdaptiveStream::record_brake(const Brake& brake, double elapsed, double dt) {
    double at=elapsed, end=elapsed+dt, boundary=0.;
    for(const auto& phase:brake.phases) {
        boundary+=phase.duration;
        if(boundary<=at) continue;
        const double next=std::min(end,boundary);
        if(next>at) record_cubic(brake.sample(at),brake.sample(next),next-at,brake.sample((at+next)/2).j);
        at=next;if(at>=end) return;
    }
    if(end>at) record_cubic(brake.final,brake.final,end-at,0.);
}

void AdaptiveStream::record_cubic(State initial,State final,double duration,double jerk) {
    initial.j=jerk;
    pieces_.push_back({initial,final,duration,{},0.,0.});
}
}  // namespace bindu::execution
