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
// Frozen nonlinear Snowbot chain and one adopted-target state from
// vr_caae146508fc, input 55, t=1.398s. No ROS/model files required.
// Joint order: left 1..7, right 1..7, waist 1..2; radians and seconds.
namespace nonlinear_fixture {
const std::array<Node,19> nodes{{
  {0,-1,false,{1,0,0,0.29599999999999999,0,1,0,0,0,0,1,0.32600000000000001,0,0,0,1},{1,0,0}},
  {1,14,false,{1,0,0,0,0,1,0,0,0,0,1,0.16450000000000001,0,0,0,1},{0,0,1}},
  {2,15,false,{1,0,0,0.089997985235361497,0,1,0,0,0,0,1,0.20000000000000001,0,0,0,1},{0,1,0}},
  {3,7,false,{1,0,0,0,0,1,0,-0.23899999999999999,0,0,1,0.29999999999999999,0,0,0,1},{0,-1,0}},
  {4,8,false,{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1},{1,0,0}},
  {5,9,false,{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1},{0,0,-1}},
  {6,10,false,{1,0,0,0.049998485961653003,0,1,0,0,0,0,1,-0.39999118520421001,0,0,0,1},{0,-1,0}},
  {7,11,false,{1,0,0,0.32500000000000001,0,1,0,0,0,0,1,-0.049999999999996297,0,0,0,1},{1,0,0}},
  {8,12,false,{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1},{0,1,0}},
  {9,13,false,{1,0,0,0.089999999999999997,0,1,0,0,0,0,1,0,0,0,0,1},{0,0,1}},
  {10,-1,false,{1,0,0,0.075999999999999998,0,1,0,0,0,0,1,0,0,0,0,1},{1,0,0}},
  {3,0,false,{1,0,0,0,0,1,0,0.23899999999999999,0,0,1,0.29999999999999999,0,0,0,1},{0,1,0}},
  {12,1,false,{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1},{1,0,0}},
  {13,2,false,{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1},{0,0,-1}},
  {14,3,false,{1,0,0,0.049999969498808597,0,1,0,0,0,0,1,-0.39999637072594402,0,0,0,1},{0,1,0}},
  {15,4,false,{1,0,0,0.32499999999999901,0,1,0,0,0,0,1,-0.049999999999999899,0,0,0,1},{1,0,0}},
  {16,5,false,{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1},{0,-1,0}},
  {17,6,false,{1,0,0,0.089999999999999705,0,1,0,0,0,0,1,0,0,0,0,1},{0,0,1}},
  {18,-1,false,{1,0,0,0.075999999999999998,0,1,0,0,0,0,1,0,0,0,0,1},{1,0,0}},
}};
constexpr int left=19,right=11,dofs=16;
const JointLimits limits{
  {-3.1417453292519943,3.1417453292519943,1,4.5,162},
  {-0.0017453292519943296,3.1417453292519943,1,4.5,162},
  {-2.6117453292519941,0.58174532925199429,1,4.5,162},
  {-1.5717453292519943,1.5717453292519943,1,4.5,162},
  {-3.1417453292519943,3.1417453292519943,1,4.5,162},
  {-1.3107453292519942,1.2757453292519942,1,4.5,162},
  {-1.2757453292519942,1.4507453292519943,1,4.5,162},
  {-3.1417453292519943,3.1417453292519943,1,4.5,162},
  {-3.1417453292519943,0.0017453292519943296,1,4.5,162},
  {-0.59745329251994328,2.6117453292519941,1,4.5,162},
  {-1.5717453292519943,1.5717453292519943,1,4.5,162},
  {-3.1417453292519943,3.1417453292519943,1,4.5,162},
  {-1.3107453292519942,1.2757453292519942,1,4.5,162},
  {-1.2757453292519942,1.4507453292519943,1,4.5,162},
  {-1.1717453292519941,1.1717453292519941,0.375,2.25,162},
  {-0.79174532925199437,0.55174532925199438,0.375,2.25,162},
};
const std::vector<double> q{
  0.020567721755338001,0.38296979115271118,0.41893127501774569,-0.41921430546366573,
  -0.95060551162810214,0.069316515969199854,-1.1294328850919031,-0.092187819687654132,
  -0.88998609154747788,-0.41059917745487229,0.51728227755629419,0.23135651483519121,
  0.42538686340047338,1.103805375730275,0.12287446253407459,0.076105555963954755,
};
const std::vector<double> v{
  -0.21582876968990561,-0.012868415663086051,-0.022762023566289379,-0.06497082766714804,
  -0.037130762215098548,-0.1652791990521961,0.034296631936810699,0.055336933174033487,
  -0.024463633668134109,0.12780351599211601,0.13261199911175359,0.067234124055966515,
  0.11389499518883429,-0.033865398009281913,-0.0035015921970648189,-0.00283698306334507,
};
const std::vector<double> a{
  -1.246531742281332,0.25935191941754882,-0.12574146487451099,-0.56247410818564958,
  -0.56383784351042321,-1.149799632700661,0.068191425278906731,-0.22068720054687371,
  -0.1889471610610417,0.91911339472676368,1.3798674417346251,0.54353023058764383,
  1.080390992170698,-0.7989267674066507,-0.039019668436366881,-0.02404466239514301,
};
const std::vector<double> goal{
  0.011038647920495914,0.38659381370813067,0.41799611153881261,-0.42491124462689012,
  -0.95502129060879359,0.058901709728342772,-1.1268885447648,-0.094030076599632292,
  -0.8901278285941906,-0.40420918928161098,0.52669191137176141,0.23617806709510064,
  0.43260495808692506,1.0980179288688137,0.12259456772678475,0.075923402023288372,
};
const std::vector<double> hint{
  -0.20349028235624431,-0.0054689245417313334,-0.034513622413987005,-0.078197656869571155,
  -0.068679347858874959,-0.16664390904238346,0.025516203641151487,0.043720722518438393,
  -0.0422832590586458,0.13696352953035751,0.1939472946836204,0.072303754263775694,
  0.18052519992182764,-0.05082441228367067,-0.0036437882560257069,-0.0034250470675150461,
};
const std::vector<double> capture{
  0.06282130628824234,0.42251583933830261,0.43586626648902888,-0.41832134127616882,
  -0.97402983903884888,0.10749849677085881,-1.1205968856811519,-0.1136104464530945,
  -0.88481926918029785,-0.44154676795005798,0.51445877552032471,0.22913837432861331,
  0.42977827787399292,1.093464612960815,0.1229102090001106,0.076507292687892914,
};
}
void nonlinear_prefix_rejection_falls_back(){
  namespace f=nonlinear_fixture;
  Model model(f::nodes,f::left,f::right,f::dofs);
  const JointState seed{f::q,f::v,f::a,{}};const Pose grasp=model.evaluate(vec(f::capture));
  Decision decision;const double start=0.;
  const auto path=constrained(model,f::limits,seed,f::goal,f::hint,grasp,start,"geometry",1.,decision);
  require(decision.accepted&&std::string(decision.reason)=="accepted_absolute_fallback",
          "nonlinear prefix rejection did not use certified rigid fallback");
  require(decision.guard_rejections>0,"new continuation was not geometrically rejected");
  continuous(seed,path.sample(start));
  for(int k=0;k<=100;++k){
    const auto s=path.sample(start+k*.001);bounded(s,f::limits);
    const V e=error(model.evaluate(vec(s.q)),grasp);
    require(e.head<3>().norm()<=.03&&e.tail<3>().norm()<=.04,"fallback exceeds original geometry guard");
  }
  for(int k=0;k<=1000;++k)bounded(path.sample(path.end()*k/1000.),f::limits);
  const auto end=path.sample(path.end()+1.);
  for(int i=0;i<f::dofs;++i)
    require(std::abs(end.q[i]-f::goal[i])+std::abs(end.v[i])+std::abs(end.a[i])<1e-8,
            "nonlinear fallback lost finite actual-target tail");
}
void joint_boundary_keeps_rigid_alternative(){
  auto model=paired_slides();const JointLimits limits(2,{-2.,.2,1.,4.5,162.});
  const JointState seed{{.195,.195},{.1,.1},{0.,0.},{0.,0.}};
  const Pose grasp=model.evaluate(vec(seed.q));Decision decision;
  const auto path=constrained(model,limits,seed,{.197,.197},{.1,.1},grasp,0.,"geometry",1.,decision);
  require(decision.accepted&&std::string(decision.reason).find("accepted_absolute")==0,
          "joint boundary lost certified rigid alternative");
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
  nonlinear_prefix_rejection_falls_back();
  std::cout<<"rigid prediction: advancing stream, C2, nonlinear fallback, limits, reversal and stop passed\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
