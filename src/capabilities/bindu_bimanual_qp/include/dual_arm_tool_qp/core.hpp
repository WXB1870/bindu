#pragma once

#include <Eigen/Core>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <proxsuite/proxqp/dense/dense.hpp>

#include <string>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

namespace dual_arm_tool_qp {

// Matches the shared executor's numerical position tolerance; not a physical
// tracking allowance. Measured kinematics retain their original values.
constexpr double position_roundoff = 1e-9;

struct Config {
  double tool_gain{2.0};
  double relative_gain{2.0};
  double posture_gain{0.3};
  double posture_weight{0.001};
  double shared_target_weight{0.0};
  double shared_target_gain{3.0};
  double shared_motion_weight{0.0};
  double regularization{0.0001};
  double max_linear_speed{0.15};
  double max_angular_speed{0.5};
  double max_relative_error{0.15};
  double limit_margin{0.05};
  double damper_gain{2.0};
  double equality_tolerance{0.0005};
};

struct SolveResult {
  Eigen::VectorXd velocity;
  bool ok{false};
  std::string reason;
  double solve_time_us{0.0};
  double relative_error{0.0};
  double relative_gain_scale{1.0};
  int qp_status{-1};
};

// Fixed-base URDF; all nonfixed joints must be scalar. Selected shared joints
// precede the left and right arm joints in the optimization and command vector.
// Other joints remain at measured positions.
class RobotModel {
public:
  RobotModel(const std::string &urdf_path, const std::string &left_frame,
             const std::string &right_frame, const std::string &tool_frame,
             const std::vector<std::string> &left_joints,
             const std::vector<std::string> &right_joints,
             const std::vector<std::string> &shared_joints = {});

  void update(const Eigen::VectorXd &q);
  const pinocchio::SE3 &leftPose() const;
  const pinocchio::SE3 &rightPose() const;
  const pinocchio::SE3 &urdfToolPose() const;
  const pinocchio::SE3 &getFramePose(const std::string &name) const;
  Eigen::MatrixXd jacobian(pinocchio::FrameIndex frame) const;
  Eigen::MatrixXd getFrameJacobian(const std::string &name) const;
  Eigen::MatrixXd leftJacobian() const { return jacobian(left_frame_); }
  Eigen::MatrixXd rightJacobian() const { return jacobian(right_frame_); }
  Eigen::VectorXd activePositions() const;
  const Eigen::VectorXd &positions() const { return q_; }
  const Eigen::VectorXd &lowerLimits() const { return lower_; }
  const Eigen::VectorXd &upperLimits() const { return upper_; }
  const Eigen::VectorXd &velocityLimits() const { return velocity_; }
  void setVelocityLimits(const Eigen::VectorXd &limits);
  void setPositionLimits(const Eigen::VectorXd &lower, const Eigen::VectorXd &upper);
  std::pair<Eigen::VectorXd, Eigen::VectorXd> getJointPositionLimits() const {
    return {lower_, upper_};
  }
  const Eigen::VectorXd &getJointVelocityLimits() const { return velocity_; }
  const std::vector<std::string> &activeJointNames() const { return active_names_; }
  const std::vector<std::string> &modelJointNames() const { return all_names_; }
  int qIndex(const std::string &name) const;
  int variables() const { return static_cast<int>(active_names_.size()); }
  int leftVariables() const { return left_count_; }
  const pinocchio::Model &model() const { return model_; }

private:
  pinocchio::Model model_;
  mutable pinocchio::Data data_;
  pinocchio::FrameIndex left_frame_, right_frame_, tool_frame_;
  Eigen::VectorXd q_, lower_, upper_, velocity_;
  std::vector<std::string> active_names_, all_names_;
  std::vector<int> active_q_indices_, active_v_indices_;
  std::unordered_map<std::string, int> q_indices_;
  int left_count_{0};
};

class RigidToolKinematics {
public:
  explicit RigidToolKinematics(RobotModel &robot) : robot_(robot) {}
  void capture();
  void capture(const pinocchio::SE3 &left_to_tool);
  // Right grip is a revolute support: translation and shaft direction remain
  // constrained, only rotation about grip-frame +Z is free. Both offsets are
  // physical grasp frames (not flange origins); caller switches while stopped.
  void captureAxial(const pinocchio::SE3 &left_to_tool,
                    const pinocchio::SE3 &right_to_grip,
                    const pinocchio::SE3 &tool_to_grip);
  int constraintRows() const { return axial_ ? 5 : 6; }
  void reset() { captured_ = false; axial_ = false; }
  bool captured() const { return captured_; }
  pinocchio::SE3 toolPose() const;
  Eigen::MatrixXd toolJacobian() const;
  pinocchio::SE3 relativePose() const;
  Eigen::VectorXd relativeError() const;
  Eigen::MatrixXd relativeErrorJacobian() const;
  const pinocchio::SE3 &referenceRelativePose() const { return relative_ref_; }
  const pinocchio::SE3 &leftToTool() const { return left_to_tool_; }

private:
  RobotModel &robot_;
  bool captured_{false};
  bool axial_{false};
  pinocchio::SE3 right_to_grip_{pinocchio::SE3::Identity()};
  pinocchio::SE3 tool_to_grip_{pinocchio::SE3::Identity()};
  pinocchio::SE3 left_to_tool_{pinocchio::SE3::Identity()};
  pinocchio::SE3 relative_ref_{pinocchio::SE3::Identity()};
};

class QpController {
public:
  QpController(RobotModel &robot, RigidToolKinematics &kin, Config config,
               const Eigen::VectorXd &nominal, int shared_count = 0);
  void reset();
  void independentArms() { independent_ = true; reset(); }
  void setSharedTargets(const Eigen::VectorXd &targets);
  void clearSharedTargets();
  void setSharedMotionWeight(double weight);
  void setCoupledLockedJoints(const std::vector<std::string> &names);
  void setLimitAvoidance(double margin, double weight);
  void setMeasuredLimitTolerance(double tolerance);
  void setPosturePolicy(const Eigen::VectorXd &reference,
                        const Eigen::VectorXd &lower, const Eigen::VectorXd &upper);
  void setPostureReferenceMargin(double margin);
  void setTrackingPoint(const Eigen::Vector3d &point);
  SolveResult solve(const pinocchio::SE3 &desired_tool, double dt,
                    const Eigen::VectorXd &reference,
                    const Eigen::Matrix<double, 6, 1> &world_velocity = Eigen::Matrix<double, 6, 1>::Zero(),
                    const pinocchio::SE3 *measured_tool = nullptr,
                    const pinocchio::SE3 *desired_right = nullptr, int active_mask = 3,
                    const Eigen::VectorXd *measured_q = nullptr, double max_lead = 0.,
                    bool shared_task_feedback = false);
  const Config &config() const { return config_; }

private:
  RobotModel &robot_;
  RigidToolKinematics &kin_;
  Config config_;
  Eigen::VectorXd nominal_;
  Eigen::VectorXd posture_lower_, posture_upper_;
  bool have_posture_policy_{false};
  double posture_reference_margin_{0.};
  Eigen::VectorXd shared_targets_;
  int shared_count_{0};
  bool have_shared_targets_{false};
  std::unique_ptr<proxsuite::proxqp::dense::QP<double>> qp_;
  bool initialized_{false};
  bool fresh_start_{true};
  bool independent_{false};
  std::vector<bool> coupled_locked_;
  double avoidance_margin_{0.}, avoidance_weight_{0.};
  double measured_limit_tolerance_{0.};
  Eigen::Vector3d tracking_point_{Eigen::Vector3d::Zero()};
};

}  // namespace dual_arm_tool_qp
