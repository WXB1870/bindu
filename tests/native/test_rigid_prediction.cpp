// Offline coupled-curve regression. Requires Eigen; no ROS or device backend.
#include "rigid_prediction.hpp"
#include <iostream>
#include <memory>

using namespace bindu::execution;
using namespace bindu::execution::rigid;
namespace {
void require(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
Model paired_slides(){
  const std::array<double,16> identity{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
  // Equal motion of two independent slides preserves their relative pose.
  const std::array<Node,2> nodes{{{0,0,true,identity,{1,0,0}},
                                 {0,1,true,identity,{1,0,0}}}};
  return Model(nodes,1,2,2);
}
void bounded(const JointState& s,const JointLimits& limits){
  for(size_t i=0;i<s.q.size();++i){
    require(s.q[i]>=limits[i].lower-1e-8&&s.q[i]<=limits[i].upper+1e-8,"position limit");
    require(std::abs(s.v[i])<=limits[i].velocity+1e-8,"velocity limit");
    require(std::abs(s.a[i])<=limits[i].acceleration+1e-8,"acceleration limit");
    require(std::abs(s.j[i])<=limits[i].jerk+1e-7,"jerk limit");
  }
}
void continuous(const JointState& a,const JointState& b){
  for(size_t i=0;i<a.q.size();++i)
    require(std::max({std::abs(a.q[i]-b.q[i]),std::abs(a.v[i]-b.v[i]),
                      std::abs(a.a[i]-b.a[i])})<1e-8,"C2 replacement");
}
void advancing_stream_and_stop(){
  auto model=paired_slides();const JointLimits limits(2,{-2.,2.,1.,4.5,162.});
  JointState state{{0.,0.},{0.,0.},{0.,0.},{0.,0.}};
  const Pose grasp=model.evaluate(vec(state.q));
  std::unique_ptr<PreparedSegment> path;double previous=0.,trend=0.;
  int accepted=0;
  for(int k=0;k<=60;++k){
    const double now=k*.02;
    if(path)state=path->sample(now);
    // The operator slows down, then speeds up, without reversing direction.
    const double goal=now<.3?.3*now:(now<.7?.09+.03*(now-.3):.102+.3*(now-.7));
    const double raw=(goal-previous)/.02;trend+=.02/(.02+.04)*(raw-trend);
    Decision decision;
    auto next=constrained(model,limits,state,{goal,goal},{trend,trend},grasp,now,"geometry",1.,decision);
    accepted+=decision.accepted;continuous(state,next.sample(now));
    for(int step=0;step<20;++step){
      const auto s=next.sample(now+step*.001);bounded(s,limits);
      require(s.v[0]>=-1e-7&&s.v[1]>=-1e-7,"advancing coupled target reversed reference");
      require(error(model.evaluate(vec(s.q)),grasp).norm()<1e-8,"relative pose changed");
    }
    path=std::make_unique<PreparedSegment>(std::move(next));previous=goal;
  }
  require(accepted>50,"coupled continuation unexpectedly fell back");
  for(int k=0;k<=1000;++k)bounded(path->sample(1.2+(path->end()-1.2)*k/1000.),limits);
  const auto end=path->sample(path->end()+1.);
  for(int i=0;i<2;++i){
    require(std::abs(end.q[i]-previous)<1e-8,"stream loss did not return to actual target");
    require(std::abs(end.v[i])+std::abs(end.a[i])<1e-8,"finite tail did not stop");
  }
  const auto seed=path->sample(1.22);Decision decision;
  const auto reversed=constrained(model,limits,seed,{previous-.15,previous-.15},{-.2,-.2},
                                grasp,1.22,"geometry",1.,decision);
  continuous(seed,reversed.sample(1.22));
  require(reversed.sample(1.22+reversed.describe().front().duration).v[0]<0.,"true reversal suppressed");
  const auto stop=PreparedSegment::stop(limits,seed,1.22,5.);continuous(seed,stop.sample(1.22));
  for(int k=0;k<=1000;++k)bounded(stop.sample(1.22+(stop.end()-1.22)*k/1000.),limits);
  const auto stopped=stop.sample(stop.end()+1.);
  require(std::abs(stopped.v[0])+std::abs(stopped.a[0])<1e-8,"stop did not settle");
}
void joint_boundary_keeps_rigid_alternative(){
  auto model=paired_slides();const JointLimits limits(2,{-2.,.2,1.,4.5,162.});
  const JointState seed{{.195,.195},{.1,.1},{0.,0.},{0.,0.}};
  const Pose grasp=model.evaluate(vec(seed.q));Decision decision;
  const auto path=constrained(model,limits,seed,{.197,.197},{.1,.1},grasp,0.,"geometry",1.,decision);
  require(decision.accepted&&std::string(decision.reason).find("accepted_absolute")==0,
          "joint boundary lost certified rigid fallback");
  continuous(seed,path.sample(0.));
  for(int k=0;k<=1000;++k){
    const auto s=path.sample(path.end()*k/1000.);bounded(s,limits);
    require(error(model.evaluate(vec(s.q)),grasp).norm()<1e-8,"fallback relative pose changed");
  }
  const auto end=path.sample(path.end()+1.);
  require(std::abs(end.q[0]-.197)+std::abs(end.v[0])+std::abs(end.a[0])<1e-8,
          "fallback lost finite actual-target tail");
}
}
int main(){try{
  advancing_stream_and_stop();joint_boundary_keeps_rigid_alternative();
  std::cout<<"rigid prediction: advancing stream, C2, geometry, limits, reversal and stop passed\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
