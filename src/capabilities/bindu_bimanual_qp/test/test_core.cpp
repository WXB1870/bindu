#include "dual_arm_tool_qp/core.hpp"
#include "bindu_bimanual_qp/c_api.h"

#include <gtest/gtest.h>
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

TEST_F(CoreTest, ToolJacobianMatchesFiniteDifferences) {
  checkJacobian(kin_->toolJacobian(), [this]() { return kin_->toolPose(); });
  const auto captured = robot_->leftPose() * kin_->leftToTool();
  EXPECT_LT((captured.translation() - robot_->urdfToolPose().translation()).norm(), 1e-12);
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
