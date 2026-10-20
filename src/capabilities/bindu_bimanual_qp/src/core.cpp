#include "dual_arm_tool_qp/core.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/explog.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_set>

namespace dual_arm_tool_qp {
namespace {
pinocchio::Model loadModel(const std::string &path) {
  pinocchio::Model model;
  pinocchio::urdf::buildModel(path, model);
  return model;
}

void requirePositive(double x, const char *name) {
  if (!std::isfinite(x) || x <= 0.0) {
    throw std::invalid_argument(std::string(name) + " must be positive and finite");
  }
}

void clampTwist(Eigen::Matrix<double, 6, 1> &twist, double linear, double angular) {
  for (int part = 0; part < 2; ++part) {
    auto v = twist.segment<3>(part * 3);
    const double limit = part == 0 ? linear : angular;
    const double norm = v.norm();
    if (norm > limit) v *= limit / norm;
  }
}
}  // namespace

RobotModel::RobotModel(const std::string &urdf_path, const std::string &left_frame,
                       const std::string &right_frame, const std::string &tool_frame,
                       const std::vector<std::string> &left_joints,
                       const std::vector<std::string> &right_joints,
                       const std::vector<std::string> &shared_joints)
    : model_(loadModel(urdf_path)), data_(model_),
      left_frame_(model_.getFrameId(left_frame)), right_frame_(model_.getFrameId(right_frame)),
      tool_frame_(model_.getFrameId(tool_frame)), q_(pinocchio::neutral(model_)) {
  if (left_frame_ == static_cast<pinocchio::FrameIndex>(model_.nframes) ||
      right_frame_ == static_cast<pinocchio::FrameIndex>(model_.nframes) ||
      tool_frame_ == static_cast<pinocchio::FrameIndex>(model_.nframes)) {
    throw std::invalid_argument("left, right, or tool frame not found in URDF");
  }
  if (left_joints.empty() || right_joints.empty()) {
    throw std::invalid_argument("both arm joint lists must be nonempty");
  }
  for (pinocchio::JointIndex j = 1; j < static_cast<pinocchio::JointIndex>(model_.njoints); ++j) {
    if (model_.nqs[j] != 1 || model_.nvs[j] != 1) {
      throw std::invalid_argument("only fixed-base scalar movable joints are supported: " + model_.names[j]);
    }
    all_names_.push_back(model_.names[j]);
    q_indices_.emplace(model_.names[j], model_.idx_qs[j]);
  }
  std::unordered_set<std::string> seen;
  auto add = [&](const std::vector<std::string> &names) {
    for (const auto &name : names) {
      if (!seen.insert(name).second || !model_.existJointName(name)) {
        throw std::invalid_argument("duplicate or missing selected joint: " + name);
      }
      const auto j = model_.getJointId(name);
      active_names_.push_back(name);
      active_q_indices_.push_back(model_.idx_qs[j]);
      active_v_indices_.push_back(model_.idx_vs[j]);
    }
  };
  add(shared_joints);
  add(left_joints);
  add(right_joints);
  left_count_ = static_cast<int>(left_joints.size());
  const int n = variables();
  lower_.resize(n);
  upper_.resize(n);
  velocity_.resize(n);
  for (int i = 0; i < n; ++i) {
    const int qi = active_q_indices_[i], vi = active_v_indices_[i];
    lower_[i] = model_.lowerPositionLimit[qi];
    upper_[i] = model_.upperPositionLimit[qi];
    velocity_[i] = model_.velocityLimit[vi];
    if (!std::isfinite(lower_[i]) || !std::isfinite(upper_[i]) ||
        !std::isfinite(velocity_[i]) || lower_[i] >= upper_[i] || velocity_[i] <= 0) {
      throw std::invalid_argument("finite joint position and velocity limits required: " + active_names_[i]);
    }
  }
  update(q_);
}

void RobotModel::update(const Eigen::VectorXd &q) {
  if (q.size() != model_.nq || !q.allFinite()) throw std::invalid_argument("invalid joint positions");
  q_ = q;
  pinocchio::forwardKinematics(model_, data_, q_);
  pinocchio::updateFramePlacements(model_, data_);
  pinocchio::computeJointJacobians(model_, data_, q_);
}

void RobotModel::setVelocityLimits(const Eigen::VectorXd &limits) {
  if (limits.size() != velocity_.size() || !limits.allFinite() ||
      (limits.array() <= 0.).any() || (limits.array() > velocity_.array()+1e-9).any())
    throw std::invalid_argument("invalid configured velocity limits");
  velocity_ = limits;
}

void RobotModel::setPositionLimits(const Eigen::VectorXd &lower, const Eigen::VectorXd &upper) {
  if (lower.size() != variables() || upper.size() != variables() ||
      !lower.allFinite() || !upper.allFinite() || (lower.array() >= upper.array()).any() ||
      (lower.array() < lower_.array()).any() || (upper.array() > upper_.array()).any())
    throw std::invalid_argument("invalid configured position limits");
  lower_ = lower;
  upper_ = upper;
}

const pinocchio::SE3 &RobotModel::leftPose() const { return data_.oMf[left_frame_]; }
const pinocchio::SE3 &RobotModel::rightPose() const { return data_.oMf[right_frame_]; }
const pinocchio::SE3 &RobotModel::urdfToolPose() const { return data_.oMf[tool_frame_]; }

const pinocchio::SE3 &RobotModel::getFramePose(const std::string &name) const {
  const auto frame = model_.getFrameId(name);
  if (frame == static_cast<pinocchio::FrameIndex>(model_.nframes))
    throw std::invalid_argument("unknown frame: " + name);
  return data_.oMf[frame];
}

Eigen::MatrixXd RobotModel::getFrameJacobian(const std::string &name) const {
  const auto frame = model_.getFrameId(name);
  if (frame == static_cast<pinocchio::FrameIndex>(model_.nframes))
    throw std::invalid_argument("unknown frame: " + name);
  return jacobian(frame);
}

Eigen::MatrixXd RobotModel::jacobian(pinocchio::FrameIndex frame) const {
  Eigen::Matrix<double, 6, Eigen::Dynamic> full(6, model_.nv);
  full.setZero();
  pinocchio::getFrameJacobian(model_, data_, frame, pinocchio::LOCAL, full);
  Eigen::MatrixXd selected(6, variables());
  for (int i = 0; i < variables(); ++i) selected.col(i) = full.col(active_v_indices_[i]);
  return selected;
}

Eigen::VectorXd RobotModel::activePositions() const {
  Eigen::VectorXd result(variables());
  for (int i = 0; i < variables(); ++i) result[i] = q_[active_q_indices_[i]];
  return result;
}

int RobotModel::qIndex(const std::string &name) const {
  auto it = q_indices_.find(name);
  return it == q_indices_.end() ? -1 : it->second;
}

void RigidToolKinematics::capture() {
  capture(robot_.leftPose().actInv(robot_.urdfToolPose()));
}

void RigidToolKinematics::capture(const pinocchio::SE3 &left_to_tool) {
  reset();
  if (!left_to_tool.translation().allFinite() || !left_to_tool.rotation().allFinite() ||
      !left_to_tool.rotation().isUnitary(1e-5) || left_to_tool.rotation().determinant() < 0.999) {
    throw std::invalid_argument("invalid fixed tool transform");
  }
  axial_ = false;
  left_to_tool_ = left_to_tool;
  relative_ref_ = robot_.leftPose().actInv(robot_.rightPose());
  captured_ = true;
}

void RigidToolKinematics::captureAxial(const pinocchio::SE3 &left_to_tool,
                                      const pinocchio::SE3 &right_to_grip,
                                      const pinocchio::SE3 &tool_to_grip) {
  reset();
  for (const auto &p : {right_to_grip, tool_to_grip}) {
    if (!p.translation().allFinite() || !p.rotation().allFinite() ||
        !p.rotation().isUnitary(1e-5) || p.rotation().determinant() < .999)
      throw std::invalid_argument("invalid support grip transform");
  }
  capture(left_to_tool);
  right_to_grip_ = right_to_grip;
  tool_to_grip_ = tool_to_grip;
  axial_ = true;
}

pinocchio::SE3 RigidToolKinematics::toolPose() const {
  if (!captured_) throw std::logic_error("grasp not captured");
  return robot_.leftPose() * left_to_tool_;
}

Eigen::MatrixXd RigidToolKinematics::toolJacobian() const {
  if (!captured_) throw std::logic_error("grasp not captured");
  return left_to_tool_.inverse().toActionMatrix() * robot_.leftJacobian();
}

pinocchio::SE3 RigidToolKinematics::relativePose() const {
  return robot_.leftPose().actInv(robot_.rightPose());
}

Eigen::VectorXd RigidToolKinematics::relativeError() const {
  if (!captured_) throw std::logic_error("grasp not captured");
  if (!axial_) return pinocchio::log6(relative_ref_.actInv(relativePose())).toVector();
  const auto grip = toolPose().actInv(robot_.rightPose() * right_to_grip_);
  const Eigen::Vector3d axis = grip.rotation().col(2);
  // Two tangent components alone would also accept an antiparallel shaft.
  if (axis.dot(tool_to_grip_.rotation().col(2)) <= 0.)
    throw std::runtime_error("support shaft direction reversed");
  Eigen::VectorXd error(5);
  error.head<3>() = grip.translation() - tool_to_grip_.translation();
  error.tail<2>() = tool_to_grip_.rotation().leftCols<2>().transpose() * axis;
  return error;
}

Eigen::MatrixXd RigidToolKinematics::relativeErrorJacobian() const {
  if (!captured_) throw std::logic_error("grasp not captured");
  if (axial_) {
    const auto relative = toolPose().actInv(robot_.rightPose());
    const Eigen::Vector3d point = relative.act(right_to_grip_.translation());
    const Eigen::Vector3d axis = relative.rotation() * right_to_grip_.rotation().col(2);
    const auto jt = toolJacobian();
    const auto jr = robot_.rightJacobian();
    Eigen::MatrixXd jac(5, robot_.variables());
    jac.topRows<3>() = relative.rotation() * (jr.topRows<3>() -
        pinocchio::skew(right_to_grip_.translation()) * jr.bottomRows<3>()) -
        jt.topRows<3>() + pinocchio::skew(point) * jt.bottomRows<3>();
    jac.bottomRows<2>() = -tool_to_grip_.rotation().leftCols<2>().transpose() *
        pinocchio::skew(axis) * (relative.rotation() * jr.bottomRows<3>() - jt.bottomRows<3>());
    return jac;
  }
  const auto relative = relativePose();
  const auto error = relative_ref_.actInv(relative);
  const Eigen::MatrixXd relative_twist = robot_.rightJacobian() -
      relative.inverse().toActionMatrix() * robot_.leftJacobian();
  return pinocchio::Jlog6(error) * relative_twist;
}

QpController::QpController(RobotModel &robot, RigidToolKinematics &kin, Config config,
                           const Eigen::VectorXd &nominal, int shared_count)
    : robot_(robot), kin_(kin), config_(config), nominal_(nominal),
      shared_count_(shared_count) {
  if (nominal.size() != robot.variables() || !nominal.allFinite())
    throw std::invalid_argument("invalid nominal arm configuration");
  requirePositive(config_.tool_gain, "tool_gain");
  requirePositive(config_.relative_gain, "relative_gain");
  requirePositive(config_.max_linear_speed, "max_linear_speed");
  requirePositive(config_.max_angular_speed, "max_angular_speed");
  requirePositive(config_.max_relative_error, "max_relative_error");
  requirePositive(config_.damper_gain, "damper_gain");
  requirePositive(config_.equality_tolerance, "equality_tolerance");
  requirePositive(config_.shared_target_gain, "shared_target_gain");
  requirePositive(config_.regularization, "regularization");
  for (double value : {config_.posture_gain, config_.posture_weight,
                       config_.shared_target_weight, config_.shared_motion_weight,
                       config_.limit_margin}) {
    if (!std::isfinite(value) || value < 0.)
      throw std::invalid_argument("invalid QP weights or limit margin");
  }
  if (shared_count_ < 0 || shared_count_ > robot.variables() ||
      (2. * config_.limit_margin >= (robot.upperLimits()-robot.lowerLimits()).array()).any())
    throw std::invalid_argument("invalid shared joint count or limit margin");
  shared_targets_ = nominal_.head(shared_count_);
  posture_lower_ = robot_.lowerLimits();
  posture_upper_ = robot_.upperLimits();
  reset();
}

void QpController::setSharedTargets(const Eigen::VectorXd &targets) {
  if (targets.size() != shared_count_ || !targets.allFinite() ||
      (targets.array() < robot_.lowerLimits().head(shared_count_).array()).any() ||
      (targets.array() > robot_.upperLimits().head(shared_count_).array()).any())
    throw std::invalid_argument("invalid shared joint targets");
  shared_targets_ = targets;
  have_shared_targets_ = true;
}

void QpController::clearSharedTargets() { have_shared_targets_ = false; }

void QpController::setSharedMotionWeight(double weight) {
  if (!std::isfinite(weight) || weight < 0.)
    throw std::invalid_argument("invalid shared motion weight");
  config_.shared_motion_weight = weight;
}

void QpController::setPosturePolicy(const Eigen::VectorXd &reference,
                                    const Eigen::VectorXd &lower,
                                    const Eigen::VectorXd &upper) {
  const int n = robot_.variables();
  if (reference.size()!=n || lower.size()!=n || upper.size()!=n ||
      !reference.allFinite() || !lower.allFinite() || !upper.allFinite() ||
      (lower.array()>=upper.array()).any() ||
      (lower.array()<robot_.lowerLimits().array()).any() ||
      (upper.array()>robot_.upperLimits().array()).any() ||
      (reference.array()<lower.array()).any() || (reference.array()>upper.array()).any())
    throw std::invalid_argument("invalid posture policy");
  const auto q = robot_.activePositions();
  if ((q.array()<lower.array()-position_roundoff).any() ||
      (q.array()>upper.array()+position_roundoff).any())
    throw std::invalid_argument("capture outside posture envelope");
  nominal_=reference; posture_lower_=lower; posture_upper_=upper;
  have_posture_policy_=true;
}

void QpController::setPostureReferenceMargin(double margin) {
  if (!have_posture_policy_ || !std::isfinite(margin) || margin < 0. ||
      (2.*margin >= (posture_upper_-posture_lower_).array()).any())
    throw std::invalid_argument("invalid posture reference margin");
  posture_reference_margin_=margin;
}

void QpController::reset() {
  qp_ = std::make_unique<proxsuite::proxqp::dense::QP<double>>(
      robot_.variables(), independent_ ? 0 : kin_.constraintRows(), 0, true);
  qp_->settings.verbose = false;
  qp_->settings.compute_timings = true;
  qp_->settings.eps_abs = 1e-7;
  // Bound unsuccessful recovery attempts; the caller still enforces its wall
  // deadline and every accepted solution retains the same residual gates.
  qp_->settings.max_iter = 100;
  initialized_ = false;
  fresh_start_ = true;
}

void QpController::setTrackingPoint(const Eigen::Vector3d &point) {
  if (!point.allFinite() || point.norm()>3.)
    throw std::invalid_argument("invalid tool tracking point");
  tracking_point_=point;
}

SolveResult QpController::solve(const pinocchio::SE3 &desired_tool, double dt,
                                const Eigen::VectorXd &reference,
                                const Eigen::Matrix<double, 6, 1> &world_velocity,
                                const pinocchio::SE3 *measured_tool,
                                const pinocchio::SE3 *desired_right, int active_mask) {
  SolveResult out;
  out.velocity = Eigen::VectorXd::Zero(robot_.variables());
  if (!kin_.captured()) { out.reason = "grasp not captured"; return out; }
  if (independent_ != (desired_right != nullptr) || active_mask < 1 || active_mask > 3) {
    out.reason = "task mode or active arm mask mismatch"; return out;
  }
  if (!std::isfinite(dt) || dt <= 0. || dt > .1 ||
      reference.size() != robot_.variables() || !reference.allFinite()) {
    out.reason = "invalid reference or timestep"; return out;
  }
  if (!desired_tool.translation().allFinite() || !desired_tool.rotation().allFinite() || !world_velocity.allFinite()) {
    out.reason = "invalid tool target"; return out;
  }
  const Eigen::VectorXd e_rel = independent_ ? Eigen::VectorXd(0) : kin_.relativeError();
  out.relative_error = e_rel.norm();
  if (!e_rel.allFinite() || out.relative_error > config_.max_relative_error) {
    out.reason = "relative pose safety threshold"; return out;
  }
  // Integrate closure/Jacobians at the accepted reference, but track the
  // measured tool: reference error alone cannot correct position-servo lag.
  const pinocchio::SE3 offset(Eigen::Matrix3d::Identity(),tracking_point_);
  const auto actual = (measured_tool ? *measured_tool : kin_.toolPose())*offset;
  const auto desired_point=desired_tool*offset;
  const auto reference_point=kin_.toolPose()*offset;
  auto tool_error = pinocchio::log6(actual.actInv(desired_point)).toVector().eval();
  const auto error_rotation = (reference_point.rotation().transpose()*actual.rotation()).eval();
  tool_error.head<3>() = (error_rotation*tool_error.head<3>()).eval();
  tool_error.tail<3>() = (error_rotation*tool_error.tail<3>()).eval();
  if (!tool_error.allFinite()) { out.reason = "invalid tool pose error"; return out; }
  Eigen::Matrix<double, 6, 1> target_twist = config_.tool_gain * tool_error;
  // World velocity is at the tool origin, not a spatial twist at world zero.
  // Feedforward is inferred from past input only; the same task limits apply.
  const auto local = reference_point.rotation().transpose().eval();
  target_twist.head<3>() += local * world_velocity.head<3>();
  target_twist.tail<3>() += local * world_velocity.tail<3>();
  clampTwist(target_twist, config_.max_linear_speed, config_.max_angular_speed);

  const int n = robot_.variables();
  Eigen::MatrixXd jt = offset.inverse().toActionMatrix()*kin_.toolJacobian();
  Eigen::VectorXd task_twist = target_twist;
  Eigen::MatrixXd a = independent_ ? Eigen::MatrixXd(0,n) : kin_.relativeErrorJacobian();
  if (independent_) {
    jt.resize(12,n); task_twist.resize(12);
    for (int side=0; side<2; ++side) {
      const auto &actual_hand = side==0 ? robot_.leftPose() : robot_.rightPose();
      const auto &desired = side==0 ? desired_tool : *desired_right;
      Eigen::Matrix<double,6,1> twist = config_.tool_gain * pinocchio::log6(actual_hand.actInv(desired)).toVector();
      if (!twist.allFinite()) { out.reason="invalid independent arm target"; return out; }
      clampTwist(twist,config_.max_linear_speed,config_.max_angular_speed);
      jt.middleRows(6*side,6) = side==0 ? robot_.leftJacobian() : robot_.rightJacobian();
      task_twist.segment(6*side,6) = twist;
    }
  }
  Eigen::VectorXd b = -config_.relative_gain * e_rel;
  const Eigen::VectorXd q = robot_.activePositions();
  const Eigen::VectorXd nominal_velocity = config_.posture_gain * (nominal_ - q);
  Eigen::MatrixXd h = jt.transpose() * jt +
      (config_.posture_weight + config_.regularization) * Eigen::MatrixXd::Identity(n, n);
  Eigen::VectorXd g = -jt.transpose() * task_twist -
      config_.posture_weight * nominal_velocity;
  // Penalize motion of the model-selected shared group, without a fixed pose
  // target or a speed reduction. Tool/grasp tasks may still use these joints.
  for (int i = 0; i < shared_count_; ++i)
    h(i, i) += config_.shared_motion_weight;
  if (have_shared_targets_) {
    for (int i = 0; i < shared_count_; ++i) {
      const double desired_velocity = std::clamp(
          config_.shared_target_gain * (shared_targets_[i] - q[i]),
          -robot_.velocityLimits()[i], robot_.velocityLimits()[i]);
      h(i, i) += config_.shared_target_weight;
      g[i] -= config_.shared_target_weight * desired_velocity;
    }
  }
  Eigen::VectorXd lower(n), upper(n);
  for (int i = 0; i < n; ++i) {
    const double vmax = robot_.velocityLimits()[i];
    const double lo = robot_.lowerLimits()[i], hi = robot_.upperLimits()[i];
    if (q[i] < lo-position_roundoff || q[i] > hi+position_roundoff ||
        reference[i] < lo-position_roundoff || reference[i] > hi+position_roundoff) {
      out.reason = "joint position outside hard limits"; return out;
    }
    const double ref = std::clamp(reference[i], lo, hi);
    // Taper outward speed toward the physical limit, rather than stopping at
    // the start of the soft margin. Both measured and reference distances
    // constrain speed; zero is always feasible when holding at a boundary.
    const double slope = config_.limit_margin > 0.
        ? std::min(config_.damper_gain, vmax/config_.limit_margin)
        : config_.damper_gain;
    lower[i] = std::max({-vmax, (lo-ref)/dt,
        std::min(0., slope * (lo-q[i])),
        std::min(0., slope * (lo-ref))});
    upper[i] = std::min({vmax, (hi-ref)/dt,
        std::max(0., slope * (hi-q[i])),
        std::max(0., slope * (hi-ref))});
    if (have_posture_policy_) {
      // Additional posture envelope is a hard task bound. Physical limit_margin
      // remains relative to the model/profile bounds, not this comfort envelope.
      const double pl = posture_lower_[i], pu = posture_upper_[i];
      if (q[i]<pl-position_roundoff || q[i]>pu+position_roundoff ||
          reference[i]<pl-position_roundoff || reference[i]>pu+position_roundoff) {
        out.reason = "joint position outside posture envelope"; return out;
      }
      lower[i] = std::max({lower[i], (pl-ref)/dt,
                           config_.damper_gain*(pl-q[i]), config_.damper_gain*(pl-ref)});
      upper[i] = std::min({upper[i], (pu-ref)/dt,
                           config_.damper_gain*(pu-q[i]), config_.damper_gain*(pu-ref)});
      // Reserve measured servo error inside the unchanged physical envelope.
      // A recapture can lie in the reserve: allow inward recovery and prohibit
      // further outward velocity, without a position jump at mode transitions.
      const double il=pl+posture_reference_margin_, iu=pu-posture_reference_margin_;
      lower[i] = std::max({lower[i], std::min(0.,config_.damper_gain*(il-q[i])),
                                   std::min(0.,config_.damper_gain*(il-ref))});
      upper[i] = std::min({upper[i], std::max(0.,config_.damper_gain*(iu-q[i])),
                                   std::max(0.,config_.damper_gain*(iu-ref))});
    }
    if (independent_ && (i<shared_count_ ||
        (i<shared_count_+robot_.leftVariables() ? !(active_mask&1) : !(active_mask&2)))) {
      // Ordinary arms do not command the waist. A released arm retains the
      // accepted hold reference; it is never pulled by the other arm's task.
      if (lower[i]>0. || upper[i]<0.) { out.reason="inactive joint cannot hold"; return out; }
      lower[i]=upper[i]=0.;
    }
    if (lower[i] > upper[i]) { out.reason = "joint bounds infeasible"; return out; }
  }
  if (!h.allFinite() || !g.allFinite() || !a.allFinite() || !b.allFinite() ||
      !lower.allFinite() || !upper.allFinite()) {
    out.reason = "nonfinite QP input"; return out;
  }
  // A physical position servo can briefly violate the rigid grasp. The
  // requested recovery velocity may then be outside the joint velocity box.
  // Preserve the selected grasp equality, but lower its recovery rate until feasible.
  for (const double scale : {1.0, 0.5, 0.25, 0.1, 0.0}) {
    b = -scale * config_.relative_gain * e_rel;
    try {
      qp_->settings.initial_guess = fresh_start_
          ? proxsuite::proxqp::InitialGuessStatus::NO_INITIAL_GUESS
          : proxsuite::proxqp::InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT;
      if (!initialized_) {
        qp_->init(h, g, a, b, proxsuite::nullopt, proxsuite::nullopt,
                 proxsuite::nullopt, lower, upper);
        initialized_ = true;
      } else {
        qp_->update(h, g, a, b, proxsuite::nullopt, proxsuite::nullopt,
                   proxsuite::nullopt, lower, upper, true);
      }
      qp_->solve();
    } catch (const std::exception &ex) {
      out.reason = std::string("QP exception: ") + ex.what(); return out;
    }
    out.solve_time_us += qp_->results.info.solve_time;
    out.qp_status = static_cast<int>(qp_->results.info.status);
    if (qp_->results.info.status != proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED ||
        !qp_->results.x.allFinite() ||
        (qp_->results.x.array() < lower.array()-config_.equality_tolerance).any() ||
        (qp_->results.x.array() > upper.array()+config_.equality_tolerance).any()) {
      reset();
      continue;
    }
    // Remove solver roundoff at box boundaries, then recheck the closure
    // equality. Never independently clip the final position after validation.
    const Eigen::VectorXd candidate = qp_->results.x.cwiseMax(lower).cwiseMin(upper);
    if ((a.rows() && (a * candidate - b).lpNorm<Eigen::Infinity>() > config_.equality_tolerance) ||
        (candidate.array() < lower.array() - config_.equality_tolerance).any() ||
        (candidate.array() > upper.array() + config_.equality_tolerance).any()) {
      reset();
      continue;
    }
    fresh_start_ = false;
    out.velocity = candidate;
    out.relative_gain_scale = scale;
    out.ok = true;
    out.reason = "ok";
    return out;
  }
  const bool zero_feasible = (lower.array() <= 0.0).all() && (upper.array() >= 0.0).all();
  out.reason = "QP infeasible after closure recovery scaling (status " +
               std::to_string(out.qp_status) + ", zero feasible " +
               (zero_feasible ? "yes" : "no") + ")";
  return out;
}

}  // namespace dual_arm_tool_qp
