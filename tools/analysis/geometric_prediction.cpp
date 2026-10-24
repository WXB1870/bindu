// Offline rigid-grasp prediction experiment. No ROS or device interfaces.
#include "rigid_prediction.hpp"
#include "prediction_model.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
using namespace bindu::execution;
using V=Eigen::VectorXd;
using M=Eigen::MatrixXd;
using Pose=Eigen::Isometry3d;

namespace {
using namespace bindu::execution::rigid;
struct ReplayModel:bindu::execution::rigid::Model {
  ReplayModel():bindu::execution::rigid::Model(prediction_model::nodes,prediction_model::left,prediction_model::right,prediction_model::dofs){}
};
void selfTest(ReplayModel& model,const V& q,const JointLimits& limits){
  const Pose target=model.evaluate(q);const M j=model.jacobian(q,target);double difference=0.;
  for(int i=0;i<q.size();++i){V plus=q,minus=q;plus[i]+=1e-6;minus[i]-=1e-6;const V numeric=(error(model.evaluate(plus),target)-error(model.evaluate(minus),target))/(2e-6);difference=std::max(difference,(numeric-j.col(i)).lpNorm<Eigen::Infinity>());}
  if(difference>1e-6)throw std::runtime_error("ANALYTIC_JACOBIAN_MISMATCH");
  V desired(q.size()),bounds(q.size());for(int i=0;i<q.size();++i){desired[i]=.25*std::sin(i+.3);bounds[i]=.8*limits[i].velocity;}
  V projected;if(!project(j,V::Zero(6),desired,bounds,projected)||(j*projected).norm()>1e-6)throw std::runtime_error("TANGENT_PROJECTION");
  V predicted=q+.1*projected;if(!retract(model,predicted,target,limits)||error(model.evaluate(predicted),target).norm()>1e-7)throw std::runtime_error("NONLINEAR_RETRACTION");
  V impossible;if(project(M::Identity(2,2),V::Ones(2)*10,V::Zero(2),V::Ones(2),impossible))throw std::runtime_error("INFEASIBLE_BOX_ACCEPTED");
  std::cout<<"CHECKS_OK jacobian_max="<<difference<<" projection_residual="<<(j*projected).norm()<<" retraction_residual="<<error(model.evaluate(predicted),target).norm()<<'\n';
}
}
int main(int argc,char** argv){try{
  if(argc!=3)return 2;const std::string mode=argv[1];const bool geometry=std::stoi(argv[2]);
  if(mode!="A"&&mode!="project"&&mode!="geometry"&&mode!="adaptive"&&mode!="check")throw std::invalid_argument("UNKNOWN_MODE");
  unsigned n,m;std::cin>>n>>m;if(n!=prediction_model::dofs||!m)return 3;JointLimits limits(n);
  for(auto& l:limits)std::cin>>l.lower>>l.upper>>l.velocity>>l.acceleration>>l.jerk;
  JointState state;state.q.resize(n);state.v.assign(n,0.);state.a.assign(n,0.);state.j.assign(n,0.);for(auto& x:state.q)std::cin>>x;
  std::vector<double> times(m),source(m);std::vector<std::vector<double>> goals(m,std::vector<double>(n));
  for(unsigned k=0;k<m;++k){std::cin>>times[k]>>source[k];for(auto& x:goals[k])std::cin>>x;
    if(!std::isfinite(times[k])||!std::isfinite(source[k])||times[k]<0||(k&&times[k]<times[k-1]))throw std::invalid_argument("INPUT_TIME_ORDER");}
  V capture(n);for(auto& x:capture)std::cin>>x;if(!std::cin)return 3;
  if(!capture.allFinite())throw std::invalid_argument("NONFINITE_GRASP_CAPTURE");
  ReplayModel model;const Pose grasp=model.evaluate(capture);if(mode=="check"){selfTest(model,vec(state.q),limits);return 0;}
  std::unique_ptr<PreparedSegment> path;unsigned next=0;bool stopped=false,previous_valid=false;double previous_source=0.;std::vector<double> previous_goal(n),trend(n,0.);
  const double stop_at=times.back()+.14;const int last_tick=int(std::ceil((stop_at+3.)*1000));std::cout<<std::setprecision(16);
  for(int tick=0;tick<=last_tick;++tick){const double now=tick*.001;if(path)state=path->sample(now);
    while(next<m&&times[next]<=now+1e-9){std::vector<double> hint(n,0.),raw(n,0.);const double dt=source[next]-previous_source;
      if(previous_valid&&dt>=.005&&dt<=.1)for(unsigned i=0;i<n;++i){raw[i]=std::clamp((goals[next][i]-previous_goal[i])/dt,-.8*limits[i].velocity,.8*limits[i].velocity);hint[i]=goals[next][i]==previous_goal[i]?0.:trend[i]+dt/(dt+.04)*(raw[i]-trend[i]);}
      double confidence=1.;if(previous_valid&&vec(raw).norm()>1e-8&&vec(trend).norm()>1e-8){const double cosine=vec(raw).dot(vec(trend))/(vec(raw).norm()*vec(trend).norm());confidence=cosine<0?.25:(cosine<.5?.5:1.);if(vec(raw).norm()<.25*vec(trend).norm())confidence=std::min(confidence,.25);}
      Decision d;const auto began=std::chrono::steady_clock::now();
      auto candidate=geometry&&mode!="A"?constrained(model,limits,state,goals[next],hint,grasp,now,mode,confidence,d):balanced(limits,state,goals[next],hint,now);
      const auto first=candidate.sample(now);double c2=0.;for(unsigned i=0;i<n;++i)c2=std::max({c2,std::abs(first.q[i]-state.q[i]),std::abs(first.v[i]-state.v[i]),std::abs(first.a[i]-state.a[i])});if(c2>1e-8)throw std::runtime_error("C2_REPLACEMENT");
      const double us=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-began).count();
      std::cerr<<std::setprecision(16)<<"E "<<now<<' '<<us<<' '<<c2<<' '<<d.accepted<<' '<<d.gain<<' '<<d.projection_failures<<' '<<d.guard_rejections<<' '<<d.attempts<<' '<<d.end_translation<<' '<<d.end_rotation<<'\n';
      path=std::make_unique<PreparedSegment>(std::move(candidate));trend=hint;previous_goal=goals[next];previous_source=source[next];previous_valid=true;++next;
    }
    if(!stopped&&now>=stop_at){path=std::make_unique<PreparedSegment>(PreparedSegment::stop(limits,state,now,2.));stopped=true;}
    for(unsigned i=0;i<n;++i)if(!std::isfinite(state.q[i])||state.q[i]<limits[i].lower-1e-7||state.q[i]>limits[i].upper+1e-7||std::abs(state.v[i])>limits[i].velocity+1e-7||std::abs(state.a[i])>limits[i].acceleration+1e-7||std::abs(state.j[i])>limits[i].jerk+1e-6)throw std::runtime_error("SAMPLED_DYNAMIC_LIMIT");
    std::cout<<now;for(const auto* x:{&state.q,&state.v,&state.a,&state.j})for(double v:*x)std::cout<<' '<<v;std::cout<<'\n';
  }
  for(unsigned i=0;i<n;++i)if(std::abs(state.v[i])>1e-8||std::abs(state.a[i])>1e-8)throw std::runtime_error("NONFINITE_STOP_TAIL");
  return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
