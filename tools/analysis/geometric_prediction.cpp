// Offline rigid-grasp prediction experiment. No ROS or device interfaces.
#include "interpolation.hpp"
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
V vec(const std::vector<double>& v){return Eigen::Map<const V>(v.data(),v.size());}
std::vector<double> values(const V& v){return {v.data(),v.data()+v.size()};}
Eigen::Matrix3d skew(const Eigen::Vector3d& x){Eigen::Matrix3d s;s<<0,-x.z(),x.y(),x.z(),0,-x.x(),-x.y(),x.x(),0;return s;}
Eigen::Vector3d logRotation(const Eigen::Matrix3d& r){Eigen::AngleAxisd a(r);return a.angle()*a.axis();}
V error(const Pose& p,const Pose& target){V e(6);e.head<3>()=p.translation()-target.translation();e.tail<3>()=logRotation(target.rotation().transpose()*p.rotation());return e;}
struct Model {
  std::vector<Pose> world,origin;
  explicit Model():world(prediction_model::nodes.size()+1),origin(prediction_model::nodes.size()){}
  Pose evaluate(const V& q){
    world[0]=Pose::Identity();
    for(size_t k=0;k<prediction_model::nodes.size();++k){const auto& n=prediction_model::nodes[k];
      Pose fixed=Pose::Identity();for(int r=0;r<4;++r)for(int c=0;c<4;++c)fixed.matrix()(r,c)=n.fixed[4*r+c];
      origin[k]=world[n.parent]*fixed;Pose motion=Pose::Identity();
      if(n.index>=0){Eigen::Vector3d axis(n.axis[0],n.axis[1],n.axis[2]);
        if(n.prismatic)motion.translation()=axis*q[n.index];else motion.linear()=Eigen::AngleAxisd(q[n.index],axis).toRotationMatrix();}
      world[k+1]=origin[k]*motion;
    }
    return world[prediction_model::left].inverse()*world[prediction_model::right];
  }
  bool ancestor(int node,int end)const{while(end>0){if(end==node)return true;end=prediction_model::nodes[end-1].parent;}return false;}
  M jacobian(const V& q,const Pose& target){
    const Pose relative=evaluate(q);const auto& l=world[prediction_model::left];const auto& r=world[prediction_model::right];
    M j=M::Zero(6,q.size());const Eigen::Vector3d phi=logRotation(target.rotation().transpose()*relative.rotation());
    const double angle=phi.norm();const auto s=skew(phi);
    const Eigen::Matrix3d inverse=Eigen::Matrix3d::Identity()-.5*s+(angle<1e-5?1./12.:1./(angle*angle)-(1.+std::cos(angle))/(2.*angle*std::sin(angle)))*s*s;
    for(size_t k=0;k<prediction_model::nodes.size();++k){const auto& n=prediction_model::nodes[k];if(n.index<0)continue;
      const Eigen::Vector3d axis=origin[k].rotation()*Eigen::Vector3d(n.axis[0],n.axis[1],n.axis[2]);
      Eigen::Vector3d vl=Eigen::Vector3d::Zero(),vr=vl,wl=vl,wr=vl;
      if(ancestor(k+1,prediction_model::left)){if(n.prismatic)vl=axis;else{wl=axis;vl=axis.cross(l.translation()-origin[k].translation());}}
      if(ancestor(k+1,prediction_model::right)){if(n.prismatic)vr=axis;else{wr=axis;vr=axis.cross(r.translation()-origin[k].translation());}}
      j.col(n.index).head<3>()=l.rotation().transpose()*(vr-vl-wl.cross(r.translation()-l.translation()));
      j.col(n.index).tail<3>()=inverse*target.rotation().transpose()*l.rotation().transpose()*(wr-wl);
    }
    return j;
  }
};
bool project(const M& j,const V& rhs,const V& desired,const V& bounds,V& out){
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
bool retract(Model& model,V& q,const Pose& target,const JointLimits& limits){
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
struct Decision {int projection_failures=0,guard_rejections=0,attempts=0,accepted=0;double gain=0.,end_translation=0.,end_rotation=0.;};

bool predictionGate(const PreparedSegment& candidate,const PreparedSegment& fallback,const std::vector<double>& goal,const std::vector<double>& hint,double start){
  V next=V::Zero(goal.size()),old=next;
  for(int k=1;k<=4;++k){const double dt=k*.01;const V predicted=vec(goal)+dt*vec(hint);
    next+=(vec(candidate.sample(start+dt).q)-predicted).array().square().matrix();
    old+=(vec(fallback.sample(start+dt).q)-predicted).array().square().matrix();}
  for(int i=0;i<next.size();++i)if(next[i]>4.*std::pow(1.1*std::sqrt(old[i]/4.)+.001,2))return false;
  return std::sqrt(next.sum()/(4.*goal.size()))<=std::sqrt(old.sum()/(4.*goal.size()))+1e-6;
}
PreparedSegment balanced(const JointLimits& limits,const JointState& seed,const std::vector<double>& goal,const std::vector<double>& hint,double start){
  auto fallback=PreparedSegment::target(limits,seed,goal,start,30.,5.);double duration=.02;
  bool moving=false;for(size_t i=0;i<goal.size();++i){duration=std::max(duration,std::abs(goal[i]-seed.q[i])/limits[i].velocity);moving=moving||std::abs(hint[i])>1e-8;}
  if(moving)for(int k=0;k<70&&duration<30.;++k,duration*=1.12){JointState end;end.q=goal;end.v=hint;end.a.assign(goal.size(),0.);end.j=end.a;bool inside=true;
    for(size_t i=0;i<goal.size();++i){end.v[i]*=std::min(1.,.2/duration);end.q[i]+=end.v[i]*duration;inside=inside&&end.q[i]>=limits[i].lower&&end.q[i]<=limits[i].upper;}
    if(!inside)break;auto curve=std::make_shared<Curve>(start,duration,seed,end);if(!curve->valid(limits))continue;
    try{auto paths=fit_target(limits,end,goal,curve->end(),30.-duration,5.);paths.insert(paths.begin(),curve);auto candidate=PreparedSegment::from_curves(paths,limits);
      return predictionGate(candidate,fallback,goal,hint,start)?candidate:fallback;
    }catch(const std::invalid_argument&){break;}}
  return fallback;
}
bool geometryGate(Model& model,const PreparedSegment& candidate,const PreparedSegment& baseline,const Pose& grasp,double start){
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
PreparedSegment constrained(Model& model,const JointLimits& limits,const JointState& seed,const std::vector<double>& goal,const std::vector<double>& raw_hint,const Pose& grasp,double start,const std::string& mode,double confidence,Decision& decision){
  auto baseline=balanced(limits,seed,goal,raw_hint,start);const V qgoal=vec(goal),desired=vec(raw_hint);const Pose target=model.evaluate(qgoal);
  // A failed geometric prediction must not reintroduce the unconstrained hint.
  // The unchanged accepted target still has a certified finite joint-space tail.
  auto fallback=PreparedSegment::target(limits,seed,goal,start,30.,5.);
  V vlimits(goal.size()),alimits(goal.size());for(size_t i=0;i<goal.size();++i){vlimits[i]=.8*limits[i].velocity;alimits[i]=limits[i].acceleration;}
  V projected;const M jac=model.jacobian(qgoal,target);
  if(!project(jac,V::Zero(6),desired,vlimits,projected)){++decision.projection_failures;return fallback;}
  if(mode=="project")return balanced(limits,seed,goal,values(projected),start);
  const double max_gain=mode=="adaptive"?confidence:1.;
  for(double multiplier:{1.,.5,.25}){const double gain=max_gain*multiplier;double duration=.02;
    for(size_t i=0;i<goal.size();++i)duration=std::max(duration,std::abs(goal[i]-seed.q[i])/limits[i].velocity);
    for(int attempt=0;attempt<70&&duration<30.;++attempt,duration*=1.12){++decision.attempts;
      const double predict=std::min(.2,duration);V endq=qgoal+gain*predict*projected;
      if(!retract(model,endq,target,limits)){++decision.projection_failures;break;}
      bool bounded=true;for(size_t i=0;i<goal.size();++i)bounded=bounded&&std::abs(endq[i]-qgoal[i])<=predict*vlimits[i]+1e-5;
      if(!bounded){++decision.projection_failures;break;}
      V endv;const M jend=model.jacobian(endq,target);
      if(!project(jend,V::Zero(6),gain*std::min(1.,.2/duration)*desired,vlimits,endv)){++decision.projection_failures;break;}
      const double dt=1e-4;const M jp=model.jacobian(endq+dt*endv,target),jm=model.jacobian(endq-dt*endv,target);
      V enda;if(!project(jend,-((jp-jm)/(2*dt))*endv,V::Zero(goal.size()),alimits,enda)){++decision.projection_failures;break;}
      JointState end{values(endq),values(endv),values(enda),std::vector<double>(goal.size(),0.)};
      auto curve=std::make_shared<Curve>(start,duration,seed,end);if(!curve->valid(limits))continue;
      try{auto paths=fit_target(limits,end,goal,curve->end(),30.-duration,5.);paths.insert(paths.begin(),curve);auto candidate=PreparedSegment::from_curves(paths,limits);
        if(!geometryGate(model,candidate,baseline,grasp,start)){++decision.guard_rejections;break;}
        // Compare against the unprojected input prediction to avoid choosing
        // a geometrically easy curve solely by changing its own scoring target.
        if(!predictionGate(candidate,fallback,goal,raw_hint,start))break;
        const V e=error(model.evaluate(endq),target);decision.accepted=1;decision.gain=gain;decision.end_translation=e.head<3>().norm();decision.end_rotation=e.tail<3>().norm();return candidate;
      }catch(const std::invalid_argument&){break;}
    }
  }
  return fallback;
}
void selfTest(Model& model,const V& q,const JointLimits& limits){
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
  Model model;const Pose grasp=model.evaluate(capture);if(mode=="check"){selfTest(model,vec(state.q),limits);return 0;}
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
