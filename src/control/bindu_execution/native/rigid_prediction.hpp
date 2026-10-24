#pragma once
// Non-real-time rigid relative-pose prediction; sampling stays in PreparedSegment.
// Optional Eigen-based helper. No ROS, device, clock, ownership or TTL effects.
#include "interpolation.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace bindu::execution::rigid {
using V=Eigen::VectorXd;using M=Eigen::MatrixXd;using Pose=Eigen::Isometry3d;
struct Node {int parent,index;bool prismatic;std::array<double,16> fixed;std::array<double,3> axis;};
inline V vec(const std::vector<double>& v){return Eigen::Map<const V>(v.data(),v.size());}
inline std::vector<double> values(const V& v){return {v.data(),v.data()+v.size()};}
inline Eigen::Matrix3d skew(const Eigen::Vector3d& x){Eigen::Matrix3d s;s<<0,-x.z(),x.y(),x.z(),0,-x.x(),-x.y(),x.x(),0;return s;}
inline Eigen::Vector3d logRotation(const Eigen::Matrix3d& r){Eigen::AngleAxisd a(r);return a.angle()*a.axis();}
inline V error(const Pose& p,const Pose& target){V e(6);e.head<3>()=p.translation()-target.translation();e.tail<3>()=logRotation(target.rotation().transpose()*p.rotation());return e;}
struct Model {
  std::vector<Node> nodes;int left,right,dofs;
  std::vector<Pose> world,origin;
  template<class Nodes> Model(const Nodes& input,int l,int r,int n):nodes(input.begin(),input.end()),left(l),right(r),dofs(n),world(nodes.size()+1),origin(nodes.size()){
    if(n<=0||n>int(PreparedSegment::max_axes)||l<=0||r<=0||l>int(nodes.size())||r>int(nodes.size()))throw std::invalid_argument("RIGID_MODEL_LAYOUT");
    for(size_t k=0;k<nodes.size();++k){const auto& node=nodes[k];
      if(node.parent<0||node.parent>int(k)||node.index< -1||node.index>=n)throw std::invalid_argument("RIGID_MODEL_NODE");
      for(double x:node.fixed)if(!std::isfinite(x))throw std::invalid_argument("RIGID_MODEL_TRANSFORM");
      const Eigen::Vector3d axis(node.axis[0],node.axis[1],node.axis[2]);
      if(!axis.allFinite()||std::abs(axis.squaredNorm()-1.)>1e-8)throw std::invalid_argument("RIGID_MODEL_AXIS");
    }
  }
  Pose evaluate(const V& q){
    if(q.size()!=dofs||!q.allFinite())throw std::invalid_argument("RIGID_MODEL_POSITIONS");
    world[0]=Pose::Identity();
    for(size_t k=0;k<nodes.size();++k){const auto& n=nodes[k];
      Pose fixed=Pose::Identity();for(int r=0;r<4;++r)for(int c=0;c<4;++c)fixed.matrix()(r,c)=n.fixed[4*r+c];
      origin[k]=world[n.parent]*fixed;Pose motion=Pose::Identity();
      if(n.index>=0){Eigen::Vector3d axis(n.axis[0],n.axis[1],n.axis[2]);
        if(n.prismatic)motion.translation()=axis*q[n.index];else motion.linear()=Eigen::AngleAxisd(q[n.index],axis).toRotationMatrix();}
      world[k+1]=origin[k]*motion;
    }
    return world[left].inverse()*world[right];
  }
  bool ancestor(int node,int end)const{while(end>0){if(end==node)return true;end=nodes[end-1].parent;}return false;}
  M jacobian(const V& q,const Pose& target){
    const Pose relative=evaluate(q);const auto& l=world[left];const auto& r=world[right];
    M j=M::Zero(6,q.size());const Eigen::Vector3d phi=logRotation(target.rotation().transpose()*relative.rotation());
    const double angle=phi.norm();const auto s=skew(phi);
    const Eigen::Matrix3d inverse=Eigen::Matrix3d::Identity()-.5*s+(angle<1e-5?1./12.:1./(angle*angle)-(1.+std::cos(angle))/(2.*angle*std::sin(angle)))*s*s;
    for(size_t k=0;k<nodes.size();++k){const auto& n=nodes[k];if(n.index<0)continue;
      const Eigen::Vector3d axis=origin[k].rotation()*Eigen::Vector3d(n.axis[0],n.axis[1],n.axis[2]);
      Eigen::Vector3d vl=Eigen::Vector3d::Zero(),vr=vl,wl=vl,wr=vl;
      if(ancestor(k+1,left)){if(n.prismatic)vl=axis;else{wl=axis;vl=axis.cross(l.translation()-origin[k].translation());}}
      if(ancestor(k+1,right)){if(n.prismatic)vr=axis;else{wr=axis;vr=axis.cross(r.translation()-origin[k].translation());}}
      j.col(n.index).head<3>()=l.rotation().transpose()*(vr-vl-wl.cross(r.translation()-l.translation()));
      j.col(n.index).tail<3>()=inverse*target.rotation().transpose()*l.rotation().transpose()*(wr-wl);
    }
    return j;
  }
};
inline bool project(const M& j,const V& rhs,const V& desired,const V& bounds,V& out){
  out=desired;std::vector<int> fixed(desired.size(),0);
  for(int step=0;step<=desired.size();++step){std::vector<int> free;V residual=rhs;
    for(int i=0;i<desired.size();++i){if(fixed[i])residual-=j.col(i)*out[i];else free.push_back(i);}
    if(!free.empty()){M a(j.rows(),free.size());V want(free.size());for(size_t k=0;k<free.size();++k){a.col(k)=j.col(free[k]);want[k]=desired[free[k]];}
      const V correction=a.completeOrthogonalDecomposition().solve(residual-a*want);
      for(size_t k=0;k<free.size();++k)out[free[k]]=want[k]+correction[k];}
    if(!out.allFinite())return false;
    int worst=-1;double violation=1e-10;
    for(int i:free)if(std::abs(out[i])-bounds[i]>violation){violation=std::abs(out[i])-bounds[i];worst=i;}
    if(worst<0)return (j*out-rhs).lpNorm<Eigen::Infinity>()<2e-7;
    fixed[worst]=1;out[worst]=std::copysign(bounds[worst],out[worst]);
  }
  return false;
}
inline bool retract(Model& model,V& q,const Pose& target,const JointLimits& limits){
  for(int k=0;k<10;++k){const V e=error(model.evaluate(q),target);if(e.head<3>().norm()<1e-8&&e.tail<3>().norm()<1e-8)return true;
    const M j=model.jacobian(q,target);V d=j.completeOrthogonalDecomposition().solve(-e);if(!d.allFinite())return false;
    double scale=std::min(1.,.03/std::max(1e-12,d.lpNorm<Eigen::Infinity>()));bool applied=false;
    for(int back=0;back<10;++back){const V proposed=q+scale*d;bool inside=true;
      for(int i=0;i<q.size();++i)inside=inside&&proposed[i]>=limits[i].lower&&proposed[i]<=limits[i].upper;
      if(inside&&error(model.evaluate(proposed),target).norm()<e.norm()){q=proposed;applied=true;break;}scale*=.5;}
    if(!applied)return false;
  }
  return false;
}
struct Decision {const char* reason="no_feasible_prediction";int projection_failures=0,guard_rejections=0,attempts=0,accepted=0;double gain=0.,end_translation=0.,end_rotation=0.;};

inline bool predictionGate(const PreparedSegment& candidate,const PreparedSegment& fallback,const std::vector<double>& goal,const std::vector<double>& hint,double start){
  V next=V::Zero(goal.size()),old=next;
  for(int k=1;k<=4;++k){const double dt=k*.01;const V predicted=vec(goal)+dt*vec(hint);
    next+=(vec(candidate.sample(start+dt).q)-predicted).array().square().matrix();
    old+=(vec(fallback.sample(start+dt).q)-predicted).array().square().matrix();}
  for(int i=0;i<next.size();++i)if(next[i]>4.*std::pow(1.1*std::sqrt(old[i]/4.)+.001,2))return false;
  return std::sqrt(next.sum()/(4.*goal.size()))<=std::sqrt(old.sum()/(4.*goal.size()))+1e-6;
}
inline PreparedSegment balanced(const JointLimits& limits,const JointState& seed,const std::vector<double>& goal,const std::vector<double>& hint,double start){
  auto fallback=PreparedSegment::target(limits,seed,goal,start,30.,5.);double duration=.02;
  bool moving=false;for(size_t i=0;i<goal.size();++i){duration=std::max(duration,std::abs(goal[i]-seed.q[i])/limits[i].velocity);moving=moving||std::abs(hint[i])>1e-8;}
  if(moving)for(int k=0;k<70&&duration<30.;++k,duration*=1.12){JointState end;end.q=goal;end.v=hint;end.a.assign(goal.size(),0.);end.j=end.a;bool inside=true;
    for(size_t i=0;i<goal.size();++i){end.v[i]*=std::min(1.,.2/duration);end.q[i]+=end.v[i]*duration;inside=inside&&end.q[i]>=limits[i].lower&&end.q[i]<=limits[i].upper;}
    if(!inside)break;
    auto curve=std::make_shared<Curve>(start,duration,seed,end);
    if(!curve->valid(limits))continue;
    try{auto paths=fit_target(limits,end,goal,curve->end(),30.-duration,5.);paths.insert(paths.begin(),curve);auto candidate=PreparedSegment::from_curves(paths,limits);
      return predictionGate(candidate,fallback,goal,hint,start)?candidate:fallback;
    }catch(const std::invalid_argument&){break;}}
  return fallback;
}
inline bool geometryGate(Model& model,const PreparedSegment& candidate,const PreparedSegment& baseline,const Pose& grasp,double start){
  std::vector<double> p,r,bp,br;double pmax=0,rmax=0,bpmax=0,brmax=0;
  const double duration=std::max(candidate.end(),baseline.end())-start;
  const int count=std::clamp(static_cast<int>(std::ceil(duration/.004)),2,128);
  for(int k=0;k<=count;++k){const double t=start+duration*k/count;
    const V e=error(model.evaluate(vec(candidate.sample(t).q)),grasp),b=error(model.evaluate(vec(baseline.sample(t).q)),grasp);
    p.push_back(e.head<3>().norm());r.push_back(e.tail<3>().norm());bp.push_back(b.head<3>().norm());br.push_back(b.tail<3>().norm());
    pmax=std::max(pmax,p.back());rmax=std::max(rmax,r.back());bpmax=std::max(bpmax,bp.back());brmax=std::max(brmax,br.back());}
  if(pmax>.03||rmax>.04||pmax>bpmax+.001||rmax>brmax+.002)return false;
  auto p95=[](std::vector<double>& x){std::sort(x.begin(),x.end());return x[static_cast<size_t>(.95*(x.size()-1))];};
  // Absolute slack avoids a singular ratio when the baseline error is zero.
  return p95(p)<=1.05*p95(bp)+1e-5&&p95(r)<=1.05*p95(br)+1e-5;
}
inline PreparedSegment constrained(Model& model,const JointLimits& limits,const JointState& seed,const std::vector<double>& goal,const std::vector<double>& raw_hint,const Pose& grasp,double start,const std::string& mode,double confidence,Decision& decision){
  auto baseline=balanced(limits,seed,goal,raw_hint,start);const V qgoal=vec(goal),desired=vec(raw_hint);const Pose target=model.evaluate(qgoal);
  // A failed geometric prediction must not reintroduce the unconstrained hint.
  // The unchanged accepted target still has a certified finite joint-space tail.
  auto fallback=PreparedSegment::target(limits,seed,goal,start,30.,5.);
  V vlimits(goal.size()),alimits(goal.size());for(size_t i=0;i<goal.size();++i){vlimits[i]=.8*limits[i].velocity;alimits[i]=limits[i].acceleration;}
  V projected;const M jac=model.jacobian(qgoal,target);
  if(!project(jac,V::Zero(6),desired,vlimits,projected)){++decision.projection_failures;decision.reason="velocity_projection";return fallback;}
  if(mode=="project")return balanced(limits,seed,goal,values(projected),start);
  const double max_gain=mode=="adaptive"?confidence:1.;
  for(double multiplier:{1.,.5,.25}){const double gain=max_gain*multiplier;double duration=.02;
    for(size_t i=0;i<goal.size();++i)duration=std::max(duration,std::abs(goal[i]-seed.q[i])/limits[i].velocity);
    for(int attempt=0;attempt<70&&duration<30.;++attempt,duration*=1.12){++decision.attempts;
      const double predict=std::min(.2,duration);V endq=qgoal+gain*predict*projected;
      if(!retract(model,endq,target,limits)){++decision.projection_failures;decision.reason="endpoint_retraction";break;}
      bool bounded=true;for(size_t i=0;i<goal.size();++i)bounded=bounded&&std::abs(endq[i]-qgoal[i])<=predict*vlimits[i]+1e-5;
      if(!bounded){++decision.projection_failures;decision.reason="prediction_displacement";break;}
      V endv;const M jend=model.jacobian(endq,target);
      if(!project(jend,V::Zero(6),gain*std::min(1.,.2/duration)*desired,vlimits,endv)){++decision.projection_failures;decision.reason="endpoint_velocity";break;}
      const double dt=1e-4;const M jp=model.jacobian(endq+dt*endv,target),jm=model.jacobian(endq-dt*endv,target);
      V enda;if(!project(jend,-((jp-jm)/(2*dt))*endv,V::Zero(goal.size()),alimits,enda)){++decision.projection_failures;decision.reason="endpoint_acceleration";break;}
      JointState end{values(endq),values(endv),values(enda),std::vector<double>(goal.size(),0.)};
      auto curve=std::make_shared<Curve>(start,duration,seed,end);if(!curve->valid(limits)){decision.reason="dynamic_limits";continue;}
      try{auto paths=fit_target(limits,end,goal,curve->end(),30.-duration,5.);paths.insert(paths.begin(),curve);auto candidate=PreparedSegment::from_curves(paths,limits);
        if(!geometryGate(model,candidate,baseline,grasp,start)){++decision.guard_rejections;decision.reason="geometry_guard";break;}
        // Compare against the unprojected input prediction to avoid choosing
        // a geometrically easy curve solely by changing its own scoring target.
        if(!predictionGate(candidate,fallback,goal,raw_hint,start)){decision.reason="prediction_error";break;}
        const V e=error(model.evaluate(endq),target);decision.reason="accepted";decision.accepted=1;decision.gain=gain;decision.end_translation=e.head<3>().norm();decision.end_rotation=e.tail<3>().norm();return candidate;
      }catch(const std::invalid_argument&){decision.reason="finite_tail";break;}
    }
  }
  return fallback;
}
} // namespace bindu::execution::rigid
