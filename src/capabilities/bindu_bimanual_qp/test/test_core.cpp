#include "dual_arm_tool_qp/core.hpp"
#include "bindu_bimanual_qp/c_api.h"

#include <gtest/gtest.h>
#include <Eigen/SVD>
#include <pinocchio/spatial/explog.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <limits>
#include <vector>

namespace datq = dual_arm_tool_qp;

TEST(CAbi, NullHandlesAndConfigLayoutFailWithoutCrashing) {
  EXPECT_EQ(bq_abi_version(), BQ_ABI_VERSION);
  bq_config config{};
  EXPECT_EQ(bq_config_defaults(&config, sizeof(config)-1), 0);
  EXPECT_EQ(bq_config_defaults(nullptr, sizeof(config)), 0);
  ASSERT_EQ(bq_config_defaults(&config, sizeof(config)), 1);
  EXPECT_EQ(config.struct_size, sizeof(config));
  EXPECT_EQ(config.abi_version, BQ_ABI_VERSION);
  EXPECT_DOUBLE_EQ(config.tool_gain, datq::Config{}.tool_gain);
  EXPECT_EQ(bq_nq(nullptr), 0);
  EXPECT_EQ(bq_nactive(nullptr), 0);
  EXPECT_EQ(bq_nmodel(nullptr), 0);
  EXPECT_EQ(bq_model_name(nullptr, 0), nullptr);
  EXPECT_EQ(bq_active_name(nullptr, 0), nullptr);
  EXPECT_STREQ(bq_error(nullptr), "null QP handle");
  EXPECT_EQ(bq_limits(nullptr, nullptr, nullptr, nullptr, 0), 0);
  EXPECT_EQ(bq_set_limits(nullptr, nullptr, nullptr, nullptr, 0), 0);
  EXPECT_EQ(bq_capture(nullptr, nullptr, 0, nullptr, &config), 0);
  EXPECT_EQ(bq_shared_targets(nullptr, nullptr, 0), 0);
  EXPECT_EQ(bq_shared_motion_weight(nullptr, 0.1), 0);
  EXPECT_EQ(bq_coupled_locked_joints(nullptr, nullptr), 0);
  EXPECT_EQ(bq_tracking_point(nullptr, nullptr), 0);
  EXPECT_EQ(bq_solve(nullptr, nullptr, 0, nullptr, 0, nullptr, .01, nullptr, nullptr), 0);
  EXPECT_EQ(bq_tool_pose(nullptr, nullptr, 0, nullptr), 0);
  EXPECT_EQ(bq_relative_pose(nullptr, nullptr, 0, nullptr), 0);
  EXPECT_EQ(bq_create(nullptr, nullptr, nullptr, nullptr, nullptr, nullptr), nullptr);
  bq_destroy(nullptr);
}

class CoreTest : public ::testing::Test {
protected:
  void SetUp() override {
    for (int i = 1; i <= 6; ++i) {
      left_names_.push_back("left_joint_" + std::to_string(i));
      right_names_.push_back("right_joint_" + std::to_string(i));
    }
    robot_ = std::make_unique<datq::RobotModel>(TEST_URDF_PATH, "left_ee", "right_ee",
                                                "tool_frame", left_names_, right_names_);
    auto q = robot_->positions();
    for (int i = 0; i < robot_->variables(); ++i)
      q[robot_->qIndex(robot_->activeJointNames()[i])] = 0.12 * std::sin(i + 1.0);
    robot_->update(q);
    kin_ = std::make_unique<datq::RigidToolKinematics>(*robot_);
    kin_->capture();
  }

  void checkJacobian(const Eigen::MatrixXd &analytic,
                     const std::function<pinocchio::SE3()> &pose) {
    const double h = 1e-7;
    const auto q0 = robot_->positions();
    const auto t0 = pose();
    for (int i = 0; i < robot_->variables(); ++i) {
      auto perturbed = q0;
      perturbed[robot_->qIndex(robot_->activeJointNames()[i])] += h;
      robot_->update(perturbed);
      const Eigen::Matrix<double, 6, 1> numerical =
          pinocchio::log6(t0.actInv(pose())).toVector() / h;
      EXPECT_LT((numerical - analytic.col(i)).norm(), 1e-5) << "column " << i;
    }
    robot_->update(q0);
  }

  std::vector<std::string> left_names_, right_names_;
  std::unique_ptr<datq::RobotModel> robot_;
  std::unique_ptr<datq::RigidToolKinematics> kin_;
};

TEST_F(CoreTest, LeftAndRightFrameJacobiansMatchFiniteDifferences) {
  checkJacobian(robot_->leftJacobian(), [this]() { return robot_->leftPose(); });
  checkJacobian(robot_->rightJacobian(), [this]() { return robot_->rightPose(); });
}

TEST_F(CoreTest, TorsoRelativeElbowDirectionJacobianMatchesFiniteDifferences) {
  const auto q0=robot_->positions();
  const auto [u,j]=robot_->relativeDirection("waist_link","left_shoulder","left_link_4");
  for (int i=0;i<robot_->variables();++i) {
    auto q=q0;q[robot_->qIndex(robot_->activeJointNames()[i])]+=1e-7;robot_->update(q);
    const auto perturbed=robot_->relativeDirection("waist_link","left_shoulder","left_link_4").first;
    EXPECT_LT(((perturbed-u)/1e-7-j.col(i)).norm(),1e-5);
  }
  robot_->update(q0);
  EXPECT_THROW(robot_->relativeDirection("waist_link","left_shoulder","left_shoulder"),std::invalid_argument);
}

TEST_F(CoreTest, ComfortImprovesRedundantJointWithoutChangingPrimaryTaskOrGrasp) {
  datq::RobotModel robot(TEST_URDF_PATH,"left_ee","right_ee","tool_frame",left_names_,right_names_,{"waist_joint"});
  auto q=robot.positions();
  for (int i=1;i<robot.variables();++i) q[robot.qIndex(robot.activeJointNames()[i])]=.2*std::sin(i);
  robot.update(q);
  datq::RigidToolKinematics kin(robot);kin.capture();
  datq::Config config;config.posture_weight=0.;
  datq::QpController baseline(robot,kin,config,robot.activePositions(),1);
  datq::QpController comfort(robot,kin,config,robot.activePositions(),1);
  auto lower=robot.lowerLimits(),upper=robot.upperLimits();
  Eigen::VectorXd weights=Eigen::VectorXd::Zero(robot.variables());
  lower[0]=.1;upper[0]=.2;weights[0]=.1;
  comfort.setJointComfort(lower,upper,weights,.3);
  const auto a=baseline.solve(kin.toolPose(),.01,robot.activePositions());
  const auto b=comfort.solve(kin.toolPose(),.01,robot.activePositions());
  ASSERT_TRUE(a.ok) << a.reason;ASSERT_TRUE(b.ok) << b.reason;
  // Independent closed form for one band objective on the exact task nullspace.
  // This fixture can have very little useful waist redundancy near a singularity.
  Eigen::MatrixXd task(12,robot.variables());
  task.topRows(6)=kin.toolJacobian();task.bottomRows(6)=kin.relativeErrorJacobian();
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(task,Eigen::ComputeFullV);
  svd.setThreshold(1e-9);
  const Eigen::MatrixXd basis=svd.matrixV().rightCols(robot.variables()-svd.rank());
  const Eigen::VectorXd direction=basis*basis.row(0).transpose();
  const Eigen::VectorXd expected=.1*.03/(config.regularization+.1*direction[0])*direction;
  EXPECT_GT(expected[0],0.);EXPECT_GT(b.velocity[0],0.);
  EXPECT_LT((b.velocity-expected).norm(),1e-5);
  EXPECT_LT((kin.toolJacobian()*(b.velocity-a.velocity)).norm(),2e-6);
  EXPECT_LT((kin.relativeErrorJacobian()*b.velocity).norm(),2e-6);
  EXPECT_LE(b.velocity.cwiseAbs().maxCoeff(),robot.velocityLimits().maxCoeff()+1e-9);
  // Ordinary arms retain the zero-velocity waist constraint, even outside a comfort band.
  comfort.independentArms();auto right=robot.rightPose();
  const auto held=comfort.solve(robot.leftPose(),.01,robot.activePositions(),
      Eigen::Matrix<double,6,1>::Zero(),nullptr,&right,1);
  ASSERT_TRUE(held.ok) << held.reason;EXPECT_NEAR(held.velocity[0],0.,1e-9);
  EXPECT_LT(held.velocity.tail(6).norm(),1e-8);
}

TEST_F(CoreTest, ArmComfortPreservesPrimaryTrackingAndValidatesGeometry) {
  datq::Config config;config.posture_weight=0.;
  datq::QpController baseline(*robot_,*kin_,config,robot_->activePositions());
  datq::QpController comfort(*robot_,*kin_,config,robot_->activePositions());
  comfort.setArmComfort(0,"waist_link","left_shoulder","left_link_4",-.25,.75,.3,.01);
  auto target=kin_->toolPose();target.translation().x()+=.005;
  const auto a=baseline.solve(target,.01,robot_->activePositions());
  const auto b=comfort.solve(target,.01,robot_->activePositions());
  ASSERT_TRUE(a.ok) << a.reason;ASSERT_TRUE(b.ok) << b.reason;
  EXPECT_LT((kin_->toolJacobian()*(b.velocity-a.velocity)).norm(),2e-6);
  EXPECT_LT((kin_->relativeErrorJacobian()*(b.velocity-a.velocity)).norm(),2e-6);
  EXPECT_THROW(comfort.setArmComfort(2,"waist_link","left_shoulder","left_link_4",0.,1.,.3,.01),std::invalid_argument);
  EXPECT_THROW(comfort.setArmComfort(0,"missing","left_shoulder","left_link_4",0.,1.,.3,.01),std::invalid_argument);
  EXPECT_THROW(comfort.setJointComfort(robot_->lowerLimits(),robot_->upperLimits(),
      Eigen::VectorXd::Constant(robot_->variables(),-1.),.3),std::invalid_argument);
}

TEST_F(CoreTest, IndependentTargetsHoldWaistAndInactiveArmWithoutClosure) {
  datq::RobotModel robot(TEST_URDF_PATH,"left_ee","right_ee","tool_frame",
                         left_names_,right_names_,{"waist_joint"});
  datq::RigidToolKinematics kin(robot);
  kin.capture(pinocchio::SE3::Identity());
  datq::QpController controller(robot,kin,{},robot.activePositions(),1);
  controller.independentArms();
  auto left=robot.leftPose(),right=robot.rightPose();
  left.translation()[2]+=.02;right.translation()[1]-=.02;
  const auto result=controller.solve(left,.01,robot.activePositions(),
      Eigen::Matrix<double,6,1>::Zero(),nullptr,&right,1);
  ASSERT_TRUE(result.ok) << result.reason;
  EXPECT_DOUBLE_EQ(result.velocity[0],0.);
  EXPECT_LT(result.velocity.tail(6).norm(),1e-12);
  EXPECT_GT(result.velocity.segment(1,6).norm(),1e-5);
  EXPECT_GT((kin.relativeErrorJacobian()*result.velocity).norm(),1e-5);
}

TEST_F(CoreTest, IndependentCapabilityCannotUseToolSolveOrInvalidMask) {
  datq::QpController controller(*robot_,*kin_,{},robot_->activePositions());
  controller.independentArms();
  auto right=robot_->rightPose();
  EXPECT_FALSE(controller.solve(robot_->leftPose(),.01,robot_->activePositions()).ok);
  for (int mask : {0,4,-1}) {
    const auto result=controller.solve(robot_->leftPose(),.01,robot_->activePositions(),
        Eigen::Matrix<double,6,1>::Zero(),nullptr,&right,mask);
    EXPECT_FALSE(result.ok);
    EXPECT_DOUBLE_EQ(result.velocity.norm(),0.);
  }
}

TEST_F(CoreTest, ToolJacobianMatchesFiniteDifferences) {
  checkJacobian(kin_->toolJacobian(), [this]() { return kin_->toolPose(); });
  const auto captured = robot_->leftPose() * kin_->leftToTool();
  EXPECT_LT((captured.translation() - robot_->urdfToolPose().translation()).norm(), 1e-12);
}

TEST_F(CoreTest, TrackingPointMatchesShiftedTaskWithoutChangingToolOrClosure) {
  const auto original=kin_->toolPose();
  const Eigen::Vector3d point(.3,-.08,.12);
  const pinocchio::SE3 offset(Eigen::Matrix3d::Identity(),point);
  datq::QpController at_point(*robot_,*kin_,{},robot_->activePositions());
  at_point.setTrackingPoint(point);
  datq::RigidToolKinematics shifted(*robot_);
  shifted.capture(kin_->leftToTool()*offset);
  datq::QpController explicit_task(*robot_,shifted,{},robot_->activePositions());
  auto target=original;
  target.translation()+=Eigen::Vector3d(.01,-.005,.008);
  target.rotation()=target.rotation()*pinocchio::exp3(Eigen::Vector3d(.01,-.015,.005));
  const auto a=at_point.solve(target,.02,robot_->activePositions());
  const auto b=explicit_task.solve(target*offset,.02,robot_->activePositions());
  ASSERT_TRUE(a.ok) << a.reason;ASSERT_TRUE(b.ok) << b.reason;
  EXPECT_GT(a.velocity.norm(),1e-5);
  EXPECT_LT((a.velocity-b.velocity).norm(),1e-7);
  EXPECT_LT((kin_->toolPose().toHomogeneousMatrix()-original.toHomogeneousMatrix()).norm(),1e-12);
  EXPECT_LT((kin_->relativeErrorJacobian()-shifted.relativeErrorJacobian()).norm(),1e-12);
  EXPECT_THROW(at_point.setTrackingPoint(Eigen::Vector3d(0.,0.,4.)),std::invalid_argument);
  EXPECT_THROW(at_point.setTrackingPoint(Eigen::Vector3d(0.,std::numeric_limits<double>::quiet_NaN(),0.)),std::invalid_argument);
}

TEST_F(CoreTest, RelativeErrorJacobianMatchesFiniteDifferences) {
  auto displaced = robot_->positions();
  displaced[robot_->qIndex("left_joint_2")] += 0.08;
  displaced[robot_->qIndex("right_joint_4")] -= 0.11;
  robot_->update(displaced);
  ASSERT_GT(kin_->relativeError().norm(), 0.01);
  const double h = 1e-7;
  const auto q0 = robot_->positions();
  const auto e0 = kin_->relativeError();
  const auto jac = kin_->relativeErrorJacobian();
  for (int i = 0; i < robot_->variables(); ++i) {
    auto perturbed = q0;
    perturbed[robot_->qIndex(robot_->activeJointNames()[i])] += h;
    robot_->update(perturbed);
    const Eigen::Matrix<double, 6, 1> numerical = (kin_->relativeError() - e0) / h;
    EXPECT_LT((numerical - jac.col(i)).norm(), 1e-5) << "column " << i;
  }
  robot_->update(q0);
}

TEST_F(CoreTest, QpTracksToolAndKeepsRigidGrasp) {
  datq::Config config;
  config.max_relative_error = 0.3;
  datq::QpController controller(*robot_, *kin_, config, robot_->activePositions());
  Eigen::Matrix<double, 6, 1> twist;
  twist << 0.01, -0.01, 0.006, 0.02, -0.01, 0.015;
  const auto desired = kin_->toolPose() * pinocchio::exp6(twist);
  const auto result = controller.solve(desired, 0.005, robot_->activePositions());
  ASSERT_TRUE(result.ok) << result.reason;
  EXPECT_GT(result.velocity.norm(), 1e-5);
  EXPECT_LT((kin_->relativeErrorJacobian() * result.velocity).norm(), 5e-4);
  EXPECT_LE(result.velocity.cwiseAbs().maxCoeff(), 2.0001);
  const auto next = controller.solve(desired, 0.005, robot_->activePositions());
  EXPECT_TRUE(next.ok) << next.reason;
}

TEST_F(CoreTest, LargeGraspErrorStops) {
  datq::Config config;
  config.max_relative_error = 0.01;
  datq::QpController controller(*robot_, *kin_, config, robot_->activePositions());
  auto q = robot_->positions();
  q[robot_->qIndex("right_joint_1")] += 0.2;
  robot_->update(q);
  const auto result = controller.solve(kin_->toolPose(), 0.01, robot_->activePositions());
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.velocity.norm(), 0.0);
}

TEST_F(CoreTest, HoldingInBothMarginsWorksBelowOldForcedRecoverySpeed) {
  robot_->setVelocityLimits(Eigen::VectorXd::Constant(robot_->variables(), .02));
  for (const bool at_upper : {false, true}) {
    for (double inset : {-1e-12, 0., .02}) {
      auto q = robot_->positions();
      q[robot_->qIndex("left_joint_1")] = at_upper ? robot_->upperLimits()[0]-inset
                                                        : robot_->lowerLimits()[0]+inset;
      robot_->update(q);
      kin_->capture();
      datq::QpController controller(*robot_, *kin_, {}, robot_->activePositions());
      const auto result = controller.solve(kin_->toolPose(), .1, robot_->activePositions());
      ASSERT_TRUE(result.ok) << result.reason;
      EXPECT_LT(result.velocity.norm(), 1e-7);
    }
  }
}

TEST_F(CoreTest, ReferenceLimitsAndLargeTimestepPreserveBoundsAndClosure) {
  auto reference = robot_->activePositions();
  reference[0] = robot_->lowerLimits()[0];
  reference[1] = robot_->upperLimits()[1];
  datq::Config config;
  config.damper_gain = 100.;
  datq::QpController controller(*robot_, *kin_, config, robot_->activePositions());
  auto target = kin_->toolPose();
  target.translation()[0] += .1;
  const auto result = controller.solve(target, .1, reference);
  ASSERT_TRUE(result.ok) << result.reason;
  EXPECT_GE(result.velocity[0], 0.);
  EXPECT_LE(result.velocity[1], 0.);
  const Eigen::VectorXd integrated = reference + .1*result.velocity;
  EXPECT_TRUE((integrated.array() >= robot_->lowerLimits().array()).all());
  EXPECT_TRUE((integrated.array() <= robot_->upperLimits().array()).all());
  EXPECT_LT((kin_->relativeErrorJacobian()*result.velocity).norm(), 5e-4);
}

TEST_F(CoreTest, SoftMarginAllowsOutwardTrackingAndTapersAtHardLimit) {
  for (const double sign : {-1.,1.}) {
    double previous_speed=std::numeric_limits<double>::infinity();
    for (const double distance : {.02,.005,.0005,0.}) {
      auto q=robot_->positions();
      const double bound=sign>0 ? robot_->upperLimits()[0] : robot_->lowerLimits()[0];
      q[robot_->qIndex("left_joint_1")]=bound-sign*distance;
      robot_->update(q);kin_->capture();
      datq::Config config;
      config.posture_gain=0.;config.limit_margin=.05;
      datq::QpController controller(*robot_,*kin_,config,robot_->activePositions());
      controller.independentArms();
      auto target_q=q;target_q[robot_->qIndex("left_joint_1")]+=sign*.1;
      robot_->update(target_q);const auto desired=robot_->leftPose();
      robot_->update(q);const auto right=robot_->rightPose();
      const auto reference=robot_->activePositions();
      const auto result=controller.solve(desired,.1,reference,
          Eigen::Matrix<double,6,1>::Zero(),nullptr,&right,1);
      ASSERT_TRUE(result.ok) << result.reason;
      const double speed=sign*result.velocity[0];
      if (distance>0.) EXPECT_GT(speed,1e-5);
      else EXPECT_LE(speed,1e-8);
      EXPECT_LE(speed,config.damper_gain*distance+1e-7);
      EXPECT_LE(speed,previous_speed+1e-7);previous_speed=speed;
      const auto next=(reference+.1*result.velocity).eval();
      EXPECT_GE(next[0],robot_->lowerLimits()[0]-1e-9);
      EXPECT_LE(next[0],robot_->upperLimits()[0]+1e-9);
    }
  }
}

TEST_F(CoreTest, HardViolationAndInvalidTimestepFailClosed) {
  datq::QpController controller(*robot_, *kin_, {}, robot_->activePositions());
  auto reference = robot_->activePositions();
  reference[0] = robot_->lowerLimits()[0]-.001;
  auto result = controller.solve(kin_->toolPose(), .01, reference);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.reason, "joint position outside hard limits");
  EXPECT_EQ(result.velocity.norm(), 0.);
  for (double dt : {0., -.01, .101, std::numeric_limits<double>::quiet_NaN()}) {
    result = controller.solve(kin_->toolPose(), dt, robot_->activePositions());
    EXPECT_FALSE(result.ok);
    EXPECT_EQ(result.velocity.norm(), 0.);
  }
  auto q = robot_->positions();
  q[robot_->qIndex("left_joint_1")] = robot_->lowerLimits()[0]-.001;
  robot_->update(q);
  kin_->capture();
  result = controller.solve(kin_->toolPose(), .01, robot_->activePositions());
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.reason, "joint position outside hard limits");
}

TEST_F(CoreTest, SharedWaistMovesToolWithoutChangingRelativeGrasp) {
  datq::RobotModel robot(TEST_URDF_PATH, "left_ee", "right_ee", "tool_frame",
                         left_names_, right_names_, {"waist_joint"});
  auto q = robot.positions();
  q[robot.qIndex("waist_joint")] = 0.25;
  q[robot.qIndex("left_joint_2")] = 0.6;
  q[robot.qIndex("left_joint_4")] = -0.8;
  q[robot.qIndex("right_joint_2")] = 0.55;
  q[robot.qIndex("right_joint_4")] = -0.75;
  robot.update(q);
  datq::RigidToolKinematics kin(robot);
  kin.capture();
  ASSERT_EQ(robot.activeJointNames().front(), "waist_joint");
  ASSERT_EQ(robot.variables(), 13);
  EXPECT_GT(kin.toolJacobian().col(0).norm(), 0.5);
  EXPECT_LT(kin.relativeErrorJacobian().col(0).norm(), 1e-10);

  const auto before = kin.toolPose();
  auto perturbed = q;
  perturbed[robot.qIndex("waist_joint")] += 1e-6;
  robot.update(perturbed);
  const Eigen::Matrix<double, 6, 1> numerical =
      pinocchio::log6(before.actInv(kin.toolPose())).toVector() / 1e-6;
  EXPECT_LT((numerical - kin.toolJacobian().col(0)).norm(), 1e-5);
  EXPECT_LT(kin.relativeError().norm(), 1e-10);
  robot.update(q);

  datq::Config config;
  datq::QpController controller(robot, kin, config, robot.activePositions());
  Eigen::Matrix<double, 6, 1> offset = Eigen::Matrix<double, 6, 1>::Zero();
  offset[0] = 0.08;
  offset[4] = 0.08;
  const auto result = controller.solve(kin.toolPose() * pinocchio::exp6(offset), 0.01, robot.activePositions());
  ASSERT_TRUE(result.ok) << result.reason;
  EXPECT_GT(std::abs(result.velocity[0]), 1e-4);
  EXPECT_LT((kin.relativeErrorJacobian() * result.velocity).norm(), 5e-4);
}

TEST_F(CoreTest, MeasuredTaskDoesNotResetAcceptedPositionOrCommandModel) {
  const Eigen::VectorXd measured=robot_->positions();
  const auto left=robot_->leftPose(), right=robot_->rightPose();
  auto command=measured;
  command[robot_->qIndex("left_joint_1")]+=.08;
  robot_->update(command);
  datq::Config config; config.posture_gain=0.;
  datq::QpController controller(*robot_,*kin_,config,robot_->activePositions());
  controller.independentArms();
  const Eigen::VectorXd reference=robot_->activePositions();
  for (int k=0;k<20;++k) {
    const auto result=controller.solve(left,.01,reference,
        Eigen::Matrix<double,6,1>::Zero(),nullptr,&right,3,&measured,.12);
    ASSERT_TRUE(result.ok) << result.reason;
    EXPECT_LT(result.velocity.norm(),1e-8);
    EXPECT_LT((robot_->positions()-command).norm(),1e-12);
    // The command remains .08 ahead; it is not measured + a tiny new step.
    EXPECT_LT((reference+.01*result.velocity-reference).norm(),1e-10);
  }
}

TEST_F(CoreTest, MeasuredTaskJacobianMatchesIndependentMeasuredKinematics) {
  const Eigen::VectorXd measured=robot_->positions();
  auto target=robot_->leftPose(); target.translation().x()+=.01;
  const auto right=robot_->rightPose();
  auto command=measured; command[robot_->qIndex("left_joint_1")]+=.08;
  robot_->update(command);
  const Eigen::VectorXd reference=robot_->activePositions();
  datq::Config config; config.posture_gain=0.;
  datq::QpController controller(*robot_,*kin_,config,reference);
  controller.independentArms();
  const auto result=controller.solve(target,.01,reference,
      Eigen::Matrix<double,6,1>::Zero(),nullptr,&right,1,&measured,.12);
  ASSERT_TRUE(result.ok) << result.reason;
  robot_->update(measured);
  const auto expected=controller.solve(target,.01,reference,
      Eigen::Matrix<double,6,1>::Zero(),nullptr,&right,1);
  ASSERT_TRUE(expected.ok) << expected.reason;
  EXPECT_LT((result.velocity-expected.velocity).norm(),1e-6);
  robot_->update(command);
}

TEST_F(CoreTest, MeasuredTaskWaistLeadReallocatesToArmsWithoutWholeStepScaling) {
  datq::RobotModel robot(TEST_URDF_PATH,"left_ee","right_ee","tool_frame",
                        left_names_,right_names_,{"waist_joint"});
  auto command=robot.positions();
  command[robot.qIndex("waist_joint")]=.12;
  command[robot.qIndex("left_joint_2")]=.6;
  command[robot.qIndex("left_joint_4")]=-.8;
  command[robot.qIndex("right_joint_2")]=.55;
  command[robot.qIndex("right_joint_4")]=-.75;
  robot.update(command); datq::RigidToolKinematics kin(robot); kin.capture();
  const Eigen::VectorXd reference=robot.activePositions();
  auto measured=command; measured[robot.qIndex("waist_joint")]=0.;
  robot.update(measured);
  Eigen::Matrix<double,6,1> shift=Eigen::Matrix<double,6,1>::Zero(); shift[5]=.04;
  const auto target=kin.toolPose()*pinocchio::exp6(shift);
  robot.update(command);
  datq::Config config; config.posture_gain=0.;
  datq::QpController controller(robot,kin,config,reference,1);
  const auto result=controller.solve(target,.01,reference,
      Eigen::Matrix<double,6,1>::Zero(),nullptr,nullptr,3,&measured,.12);
  ASSERT_TRUE(result.ok) << result.reason;
  EXPECT_LE(result.velocity[0],1e-12);
  EXPECT_GT(result.velocity.tail(12).norm(),1e-4);
  EXPECT_LT((kin.relativeErrorJacobian()*result.velocity).norm(),5e-4);
  EXPECT_LT((robot.positions()-command).norm(),1e-12);
}

TEST_F(CoreTest, SharedFeedbackTracksWaistWithoutChasingStaticArmResidual) {
  datq::RobotModel robot(TEST_URDF_PATH,"left_ee","right_ee","tool_frame",
                        left_names_,right_names_,{"waist_joint"});
  auto command=robot.positions();
  command[robot.qIndex("waist_joint")]=.08;
  command[robot.qIndex("left_joint_2")]=.6;
  command[robot.qIndex("left_joint_4")]=-.8;
  command[robot.qIndex("right_joint_2")]=.55;
  command[robot.qIndex("right_joint_4")]=-.75;
  robot.update(command);datq::RigidToolKinematics kin(robot);kin.capture();
  const Eigen::VectorXd reference=robot.activePositions();
  auto measured=command;measured[robot.qIndex("waist_joint")]=0.;
  robot.update(measured);const auto target=kin.toolPose();
  measured[robot.qIndex("left_joint_1")]+=.02;
  robot.update(command);
  datq::Config config;config.posture_gain=0.;
  datq::QpController controller(robot,kin,config,reference,1);
  const auto held=controller.solve(target,.01,reference,
      Eigen::Matrix<double,6,1>::Zero(),nullptr,nullptr,3,&measured,.12,true);
  ASSERT_TRUE(held.ok) << held.reason;
  EXPECT_LT(held.velocity.norm(),1e-8);
  // Actual waist advances; arm references must react without resetting the
  // pending waist command or chasing the unchanged arm feedback offset.
  measured[robot.qIndex("waist_joint")]=.01;
  const auto moved=controller.solve(target,.01,reference,
      Eigen::Matrix<double,6,1>::Zero(),nullptr,nullptr,3,&measured,.12,true);
  ASSERT_TRUE(moved.ok) << moved.reason;
  EXPECT_GT(moved.velocity.norm(),1e-4);
  EXPECT_LT((robot.positions()-command).norm(),1e-12);
}

TEST_F(CoreTest, MeasuredTaskRejectsPhysicalHardViolationAndInvalidBudget) {
  const Eigen::VectorXd reference=robot_->activePositions();
  auto measured=robot_->positions();
  measured[robot_->qIndex("left_joint_1")]=robot_->upperLimits()[0]+.01;
  datq::Config config;
  datq::QpController controller(*robot_,*kin_,config,reference);
  const auto target=kin_->toolPose();
  const auto bad=controller.solve(target,.01,reference,
      Eigen::Matrix<double,6,1>::Zero(),nullptr,nullptr,3,&measured,.12);
  EXPECT_FALSE(bad.ok); EXPECT_EQ(bad.reason,"joint position outside hard limits");
  measured=robot_->positions();
  const auto invalid=controller.solve(target,.01,reference,
      Eigen::Matrix<double,6,1>::Zero(),nullptr,nullptr,3,&measured,0.);
  EXPECT_FALSE(invalid.ok);
}

TEST_F(CoreTest, SharedMotionCostPrefersArmsWithoutLockingWaist) {
  datq::RobotModel robot(TEST_URDF_PATH, "left_ee", "right_ee", "tool_frame",
                         left_names_, right_names_, {"waist_joint"});
  auto q = robot.positions();
  q[robot.qIndex("waist_joint")] = 0.25;
  for (const auto &side : {"left", "right"}) {
    q[robot.qIndex(std::string(side)+"_joint_2")] = 0.6;
    q[robot.qIndex(std::string(side)+"_joint_4")] = -0.8;
  }
  robot.update(q);
  datq::RigidToolKinematics kin(robot);
  kin.capture();
  Eigen::Matrix<double,6,1> offset=Eigen::Matrix<double,6,1>::Zero();
  offset[0]=0.02; offset[4]=0.02;
  const auto target=kin.toolPose()*pinocchio::exp6(offset);
  datq::QpController ordinary(robot,kin,{},robot.activePositions(),1);
  datq::QpController weighted(robot,kin,{},robot.activePositions(),1);
  weighted.setSharedMotionWeight(0.1);
  const auto before=ordinary.solve(target,.01,robot.activePositions());
  const auto after=weighted.solve(target,.01,robot.activePositions());
  ASSERT_TRUE(before.ok) << before.reason;
  ASSERT_TRUE(after.ok) << after.reason;
  ASSERT_GT(std::abs(before.velocity[0]),1e-4);
  EXPECT_LT(std::abs(after.velocity[0]),.9*std::abs(before.velocity[0]));
  EXPECT_LT((kin.relativeErrorJacobian()*after.velocity).norm(),5e-4);
  const auto original_twist=(kin.toolJacobian()*before.velocity).eval();
  const auto weighted_twist=(kin.toolJacobian()*after.velocity).eval();
  EXPECT_GT(weighted_twist.dot(original_twist),.9*original_twist.squaredNorm());

  // When the arms cannot contribute, the soft cost still permits waist use.
  auto limits=robot.velocityLimits();
  limits.tail(12).setConstant(1e-6);
  robot.setVelocityLimits(limits);
  datq::QpController needed(robot,kin,{},robot.activePositions(),1);
  needed.setSharedMotionWeight(0.1);
  const auto assisted=needed.solve(target,.01,robot.activePositions());
  ASSERT_TRUE(assisted.ok) << assisted.reason;
  EXPECT_GT(std::abs(assisted.velocity[0]),1e-3);
  for (double invalid : {-1.,std::numeric_limits<double>::infinity(),
                         std::numeric_limits<double>::quiet_NaN()})
    EXPECT_THROW(needed.setSharedMotionWeight(invalid),std::invalid_argument);
}

TEST_F(CoreTest, CoupledLockHoldsNonzeroWaistAndReallocatesBothDirections) {
  datq::RobotModel robot(TEST_URDF_PATH,"left_ee","right_ee","tool_frame",
                         left_names_,right_names_,{"waist_joint"});
  auto q=robot.positions();
  q[robot.qIndex("waist_joint")]=.25;
  for (const auto &side : {"left","right"}) {
    q[robot.qIndex(std::string(side)+"_joint_2")]=.6;
    q[robot.qIndex(std::string(side)+"_joint_4")]=-.8;
  }
  robot.update(q);
  datq::RigidToolKinematics kin(robot); kin.capture();
  datq::QpController controller(robot,kin,{},robot.activePositions(),1);
  controller.setCoupledLockedJoints({"waist_joint"});
  for (double sign : {-1.,1.}) {
    Eigen::Matrix<double,6,1> offset=Eigen::Matrix<double,6,1>::Zero();
    offset[0]=sign*.02; offset[4]=sign*.02;
    const auto result=controller.solve(kin.toolPose()*pinocchio::exp6(offset),.01,
                                       robot.activePositions());
    ASSERT_TRUE(result.ok) << result.reason;
    EXPECT_DOUBLE_EQ(result.velocity[0],0.);
    EXPECT_GT(result.velocity.tail(12).norm(),1e-5);
    EXPECT_GT((kin.toolJacobian()*result.velocity).dot(offset),0.);
    EXPECT_LT((kin.relativeErrorJacobian()*result.velocity).norm(),5e-4);
  }
  controller.setCoupledLockedJoints({});
  // This fixture's shared joint rotates about Y: excite X/pitch, as above.
  Eigen::Matrix<double,6,1> offset=Eigen::Matrix<double,6,1>::Zero();
  offset[0]=.02; offset[4]=.02;
  const auto goal=kin.toolPose()*pinocchio::exp6(offset);
  const auto unlocked=controller.solve(goal,.01,robot.activePositions());
  ASSERT_TRUE(unlocked.ok) << unlocked.reason;
  EXPECT_GT(std::abs(unlocked.velocity[0]),1e-5);
  controller.setCoupledLockedJoints({"waist_joint"});
  auto outside=q; outside[robot.qIndex("waist_joint")]=robot.upperLimits()[0]+.01;
  robot.update(outside);
  EXPECT_FALSE(controller.solve(goal,.01,robot.activePositions()).ok);
}

TEST_F(CoreTest, CoupledLockDoesNotApplyToIndependentArmTasks) {
  datq::QpController controller(*robot_,*kin_,{},robot_->activePositions());
  controller.setCoupledLockedJoints(robot_->activeJointNames());
  controller.independentArms();
  auto left=robot_->leftPose(),right=robot_->rightPose(); left.translation().z()+=.01;
  const auto result=controller.solve(left,.01,robot_->activePositions(),
      Eigen::Matrix<double,6,1>::Zero(),nullptr,&right,1);
  ASSERT_TRUE(result.ok) << result.reason;
  EXPECT_GT(result.velocity.head(6).norm(),1e-5);
  EXPECT_LT(result.velocity.tail(6).norm(),1e-12);
}

TEST_F(CoreTest, CoupledLockCAbiRejectsBadNamesAndInvalidatesCapture) {
  const auto csv=[](const std::vector<std::string> &names) {
    std::string value;
    for (const auto &name : names) { if (!value.empty()) value+=","; value+=name; }
    return value;
  };
  auto *handle=bq_create(TEST_URDF_PATH,"left_ee","right_ee","waist_joint",
                         csv(left_names_).c_str(),csv(right_names_).c_str());
  ASSERT_NE(handle,nullptr);
  const std::unique_ptr<void,decltype(&bq_destroy)> cleanup(handle,bq_destroy);
  std::vector<double> q(bq_nq(handle),0.);
  const double identity[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
  bq_config config{}; ASSERT_EQ(bq_config_defaults(&config,sizeof(config)),1);
  for (const char *names : {"unknown","waist_joint,waist_joint","waist_joint,"}) {
    ASSERT_EQ(bq_capture(handle,q.data(),q.size(),identity,&config),1);
    EXPECT_EQ(bq_coupled_locked_joints(handle,names),0);
    EXPECT_EQ(bq_coupled_locked_joints(handle,"waist_joint"),0);
  }
  ASSERT_EQ(bq_capture(handle,q.data(),q.size(),identity,&config),1);
  EXPECT_EQ(bq_coupled_locked_joints(handle,"waist_joint"),1);
  EXPECT_EQ(bq_coupled_locked_joints(handle,nullptr),1);
}

TEST_F(CoreTest, SharedWaistTargetMovesWaistWhileHoldingToolAndGrasp) {
  datq::RobotModel robot(TEST_URDF_PATH, "left_ee", "right_ee", "tool_frame",
                         left_names_, right_names_, {"waist_joint"});
  auto q = robot.positions();
  q[robot.qIndex("waist_joint")] = 0.25;
  q[robot.qIndex("left_joint_2")] = 0.6;
  q[robot.qIndex("left_joint_4")] = -0.8;
  q[robot.qIndex("right_joint_2")] = 0.55;
  q[robot.qIndex("right_joint_4")] = -0.75;
  robot.update(q);
  datq::RigidToolKinematics kin(robot);
  kin.capture();
  datq::Config config;
  config.shared_target_weight = 0.3;
  datq::QpController controller(robot, kin, config, robot.activePositions(), 1);
  const auto goal = kin.toolPose();
  Eigen::VectorXd target(1);
  target[0] = 0.35;
  controller.setSharedTargets(target);
  const auto result = controller.solve(goal, 0.01, robot.activePositions());
  ASSERT_TRUE(result.ok) << result.reason;
  EXPECT_GT(result.velocity[0], 0.05);
  EXPECT_LT((kin.relativeErrorJacobian() * result.velocity).norm(), 5e-4);
  controller.clearSharedTargets();
  const auto stopped = controller.solve(goal, 0.01, robot.activePositions());
  ASSERT_TRUE(stopped.ok) << stopped.reason;
  EXPECT_LT(std::abs(stopped.velocity[0]), std::abs(result.velocity[0]) * 0.5);
}

TEST_F(CoreTest, PostureEnvelopeBoundsAutomaticWaistWithoutSharedTargets) {
  datq::RobotModel robot(TEST_URDF_PATH, "left_ee", "right_ee", "tool_frame",
                         left_names_, right_names_, {"waist_joint"});
  auto q = robot.positions();
  q[robot.qIndex("waist_joint")] = 0.199;
  q[robot.qIndex("left_joint_2")] = 0.6;
  q[robot.qIndex("left_joint_4")] = -0.8;
  q[robot.qIndex("right_joint_2")] = 0.55;
  q[robot.qIndex("right_joint_4")] = -0.75;
  robot.update(q);
  datq::RigidToolKinematics kin(robot); kin.capture();
  datq::Config config;
  datq::QpController controller(robot,kin,config,robot.activePositions(),1);
  auto lower = robot.lowerLimits(), upper = robot.upperLimits();
  lower[0]=-0.2; upper[0]=0.2;
  controller.setPosturePolicy(robot.activePositions(),lower,upper);
  auto goal=kin.toolPose(); goal.translation().y()+=0.05;
  const auto result=controller.solve(goal,0.1,robot.activePositions());
  ASSERT_TRUE(result.ok) << result.reason;
  EXPECT_GT(result.velocity[0],0.);
  EXPECT_LE(result.velocity[0],0.002+1e-8);
  EXPECT_LT((kin.relativeErrorJacobian()*result.velocity).norm(),5e-4);
  controller.setPostureReferenceMargin(0.01);
  const auto reserved=controller.solve(goal,0.1,robot.activePositions());
  ASSERT_TRUE(reserved.ok) << reserved.reason;
  EXPECT_LE(reserved.velocity[0],1e-7);
  EXPECT_LT((kin.relativeErrorJacobian()*reserved.velocity).norm(),5e-4);
  EXPECT_THROW(controller.setPostureReferenceMargin(-0.01),std::invalid_argument);
  EXPECT_THROW(controller.setPostureReferenceMargin(0.2),std::invalid_argument);
  auto invalid=lower; invalid[0]=0.3;
  EXPECT_THROW(controller.setPosturePolicy(robot.activePositions(),invalid,upper),std::invalid_argument);
  q[robot.qIndex("waist_joint")]=0.201; robot.update(q);
  EXPECT_FALSE(controller.solve(goal,0.01,robot.activePositions()).ok);
}

TEST_F(CoreTest, ReenabledControllerSolvesNewToolTarget) {
  datq::Config config;
  datq::QpController controller(*robot_, *kin_, config, robot_->activePositions());
  const auto initial = kin_->toolPose();
  const auto small = initial * pinocchio::exp6(
      (Eigen::Matrix<double, 6, 1>() << 0.015, 0.0, 0.0, 0.0, 0.0, 0.01).finished());
  ASSERT_TRUE(controller.solve(small, 0.01, robot_->activePositions()).ok);
  controller.reset();
  const auto larger = initial * pinocchio::exp6(
      (Eigen::Matrix<double, 6, 1>() << 0.08, 0.0, 0.0, 0.0, 0.0, 0.02).finished());
  const auto result = controller.solve(larger, 0.01, robot_->activePositions());
  ASSERT_TRUE(result.ok) << result.reason;
  EXPECT_GT(result.velocity.norm(), 1e-5);
}

TEST_F(CoreTest, ClosureRecoverySlowsWhenFullCorrectionExceedsVelocityBounds) {
  auto q = robot_->positions();
  q[robot_->qIndex("left_joint_1")] += 0.05;
  robot_->update(q);
  datq::Config config;
  config.max_relative_error = 1.0;
  config.relative_gain = 100.0;
  datq::QpController controller(*robot_, *kin_, config, robot_->activePositions());
  const auto result = controller.solve(kin_->toolPose(), 0.01, robot_->activePositions());
  ASSERT_TRUE(result.ok) << result.reason;
  EXPECT_LT(result.relative_gain_scale, 1.0);
  EXPECT_LT((kin_->relativeErrorJacobian() * result.velocity +
             result.relative_gain_scale * config.relative_gain * kin_->relativeError()).norm(),
            1e-3);
}

TEST_F(CoreTest, BoundedRecoveryRemainsClosingAcrossTightVelocityLimits) {
  auto q = robot_->positions();
  q[robot_->qIndex("left_joint_1")] += 0.03;
  robot_->update(q);
  const auto error = kin_->relativeError();
  const auto jac = kin_->relativeErrorJacobian();
  datq::Config config;
  config.relative_gain = 100.;
  config.max_relative_error = 1.;
  double previous_scale = 2.;
  for (double limit : {0.2, 0.01, 1e-4}) {
    robot_->setVelocityLimits(Eigen::VectorXd::Constant(robot_->variables(), limit));
    datq::QpController controller(*robot_, *kin_, config, robot_->activePositions());
    const auto result = controller.solve(kin_->toolPose(), 0.01, robot_->activePositions());
    ASSERT_TRUE(result.ok) << result.reason;
    EXPECT_GE(result.relative_gain_scale, 0.);
    EXPECT_LE(result.relative_gain_scale, 1.);
    EXPECT_LE(result.velocity.cwiseAbs().maxCoeff(), limit+1e-8);
    EXPECT_LT((jac*result.velocity + config.relative_gain*result.relative_gain_scale*error)
                  .lpNorm<Eigen::Infinity>(), config.equality_tolerance);
    EXPECT_LE(error.dot(jac*result.velocity), 1e-8);
    EXPECT_LE(result.relative_gain_scale, previous_scale+1e-7);
    previous_scale = result.relative_gain_scale;
  }
}

TEST_F(CoreTest, RepeatedQpStepsMoveToolWithoutOpeningGrasp) {
  auto bent = robot_->positions();
  bent[robot_->qIndex("left_joint_2")] = 0.6;
  bent[robot_->qIndex("left_joint_4")] = -0.8;
  bent[robot_->qIndex("right_joint_2")] = 0.55;
  bent[robot_->qIndex("right_joint_4")] = -0.75;
  robot_->update(bent);
  kin_->capture();
  datq::Config config;
  config.max_relative_error = 0.3;
  datq::QpController controller(*robot_, *kin_, config, robot_->activePositions());
  Eigen::Matrix<double, 6, 1> displacement = Eigen::Matrix<double, 6, 1>::Zero();
  displacement.head<3>() << 0.0, -0.015, 0.01;
  displacement.tail<3>() << 0.015, -0.01, 0.02;
  const auto desired = kin_->toolPose() * pinocchio::exp6(displacement);
  const double initial_error = pinocchio::log6(kin_->toolPose().actInv(desired)).toVector().norm();
  double max_grasp_error = 0.0;
  for (int step = 0; step < 100; ++step) {
    const auto result = controller.solve(desired, 0.005, robot_->activePositions());
    ASSERT_TRUE(result.ok) << "step " << step << ": " << result.reason;
    auto q = robot_->positions();
    for (int i = 0; i < robot_->variables(); ++i)
      q[robot_->qIndex(robot_->activeJointNames()[i])] += 0.005 * result.velocity[i];
    robot_->update(q);
    max_grasp_error = std::max(max_grasp_error, kin_->relativeError().norm());
  }
  const double final_error = pinocchio::log6(kin_->toolPose().actInv(desired)).toVector().norm();
  EXPECT_LT(final_error, initial_error * 0.75);
  EXPECT_LT(max_grasp_error, 0.002);
}

TEST_F(CoreTest, AxialGripJacobianMatchesFiniteDifferencesAtNonzeroError) {
  const auto tool = kin_->leftToTool();
  const pinocchio::SE3 right_grip(pinocchio::exp3(Eigen::Vector3d(.3,-.2,.4)),
                                  Eigen::Vector3d(.075,.02,-.03));
  const auto tool_grip = kin_->toolPose().actInv(robot_->rightPose() * right_grip);
  kin_->captureAxial(tool,right_grip,tool_grip);
  auto q = robot_->positions();
  q[robot_->qIndex("left_joint_2")] += .06;
  q[robot_->qIndex("right_joint_4")] -= .08;
  robot_->update(q);
  const auto error = kin_->relativeError();
  const auto jac = kin_->relativeErrorJacobian();
  ASSERT_EQ(error.size(),5);
  ASSERT_GT(error.norm(),.01);
  for (int i=0;i<robot_->variables();++i) {
    auto perturbed=q;
    perturbed[robot_->qIndex(robot_->activeJointNames()[i])] += 1e-7;
    robot_->update(perturbed);
    EXPECT_LT(((kin_->relativeError()-error)/1e-7-jac.col(i)).norm(),1e-5);
  }
  robot_->update(q);
  datq::Config cfg; cfg.max_relative_error=.3;
  datq::QpController controller(*robot_,*kin_,cfg,robot_->activePositions());
  auto result=controller.solve(kin_->toolPose(),.01,robot_->activePositions());
  ASSERT_TRUE(result.ok) << result.reason;
  EXPECT_LT((jac*result.velocity+result.relative_gain_scale*cfg.relative_gain*error).norm(),1e-3);
}

TEST_F(CoreTest, AxialRollIsFreeButSlipTiltAndReversedAxisAreNot) {
  const auto tool=kin_->leftToTool();
  const pinocchio::SE3 right_grip(pinocchio::exp3(Eigen::Vector3d(.3,.2,-.4)),
                                  Eigen::Vector3d(.075,.03,.02));
  const auto tool_grip=kin_->toolPose().actInv(robot_->rightPose()*right_grip);
  auto rolled=tool_grip;
  rolled.rotation()=rolled.rotation()*pinocchio::exp3(Eigen::Vector3d(0,0,2.1));
  kin_->captureAxial(tool,right_grip,rolled);
  EXPECT_LT(kin_->relativeError().norm(),1e-12);
  auto slid=rolled; slid.translation()+=rolled.rotation().col(2)*.02;
  kin_->captureAxial(tool,right_grip,slid);
  EXPECT_NEAR(kin_->relativeError().head<3>().norm(),.02,1e-12);
  auto tilted=tool_grip;
  tilted.rotation()=tilted.rotation()*pinocchio::exp3(Eigen::Vector3d(.1,0,0));
  kin_->captureAxial(tool,right_grip,tilted);
  EXPECT_NEAR(kin_->relativeError().tail<2>().norm(),std::sin(.1),1e-12);
  tilted.rotation()=tool_grip.rotation()*pinocchio::exp3(Eigen::Vector3d(3.141592653589793,0,0));
  kin_->captureAxial(tool,right_grip,tilted);
  EXPECT_THROW(kin_->relativeError(),std::runtime_error);
  kin_->capture(tool);
  EXPECT_EQ(kin_->constraintRows(),6);
  EXPECT_LT(kin_->relativeError().norm(),1e-12);
}
