// C ABI authority: memory layouts, finite SE3/numeric inputs and model bounds.
// Python validates user configuration; the executor independently gates commands.
#include "bindu_bimanual_qp/c_api.h"
#include "dual_arm_tool_qp/core.hpp"
#include <pinocchio/spatial/se3.hpp>
#include <algorithm>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace {
using namespace dual_arm_tool_qp;
std::vector<std::string> split(const char *s) {
  std::vector<std::string> out;
  if (!s) return out;
  std::string value(s);
  for (size_t start = 0; start < value.size();) {
    const size_t end = value.find(',', start);
    auto name = value.substr(start, end == std::string::npos ? end : end-start);
    if (name.empty()) throw std::invalid_argument("empty joint name");
    out.push_back(name);
    if (end == std::string::npos) break;
    start = end+1;
    if (start == value.size()) throw std::invalid_argument("empty joint name");
  }
  return out;
}
pinocchio::SE3 pose(const double *a) {
  if (!a) throw std::invalid_argument("null SE3 matrix");
  for (int i = 0; i < 16; ++i)
    if (!std::isfinite(a[i])) throw std::invalid_argument("invalid SE3 matrix");
  Eigen::Matrix3d r;
  Eigen::Vector3d t;
  for (int i = 0; i < 3; ++i) {
    t[i] = a[4*i+3];
    for (int j = 0; j < 3; ++j) r(i,j) = a[4*i+j];
  }
  if (!r.isUnitary(1e-5) || r.determinant() < .999 ||
      std::abs(a[12]) > 1e-8 || std::abs(a[13]) > 1e-8 ||
      std::abs(a[14]) > 1e-8 || std::abs(a[15]-1.) > 1e-8)
    throw std::invalid_argument("invalid SE3 matrix");
  return pinocchio::SE3(r,t);
}
void matrix(const pinocchio::SE3 &p, double *a) {
  if (!a) throw std::invalid_argument("null pose output");
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) a[4*i+j] = p.rotation()(i,j);
    a[4*i+3] = p.translation()[i];
  }
  a[12] = a[13] = a[14] = 0.; a[15] = 1.;
}
template<class To, class From> void copyConfig(To &to, const From &from) {
#define COPY(name) to.name = from.name
  COPY(tool_gain); COPY(relative_gain); COPY(posture_gain); COPY(posture_weight);
  COPY(shared_target_weight); COPY(shared_target_gain); COPY(regularization);
  COPY(max_linear_speed); COPY(max_angular_speed); COPY(max_relative_error);
  COPY(limit_margin); COPY(damper_gain); COPY(equality_tolerance);
#undef COPY
}
struct Handle {
  std::unique_ptr<RobotModel> robot;
  std::unique_ptr<RigidToolKinematics> kin;
  std::unique_ptr<QpController> qp;
  int shared_count{0};
  std::string error;
};
template<class Operation> int checked(void *ptr, Operation operation) {
  auto *h = static_cast<Handle *>(ptr);
  if (!h) return 0;
  try {
    operation(*h);
    h->error.clear();
    return 1;
  } catch (const std::exception &e) { h->error = e.what(); }
  catch (...) { h->error = "unknown native error"; }
  return 0;
}
void update(Handle &h, const double *q, int n) {
  if (!q || n != h.robot->model().nq) throw std::invalid_argument("joint layout mismatch");
  h.robot->update(Eigen::Map<const Eigen::VectorXd>(q,n));
  const auto active = h.robot->activePositions();
  for (int i = 0; i < active.size(); ++i) {
    const double lo = h.robot->lowerLimits()[i], hi = h.robot->upperLimits()[i];
    if (active[i] < lo-position_roundoff || active[i] > hi+position_roundoff) {
      std::ostringstream message;
      message.precision(17);
      message << "joint position outside hard limits: " << h.robot->activeJointNames()[i]
              << " q=" << active[i] << " bounds=[" << lo << "," << hi << "]";
      throw std::invalid_argument(message.str());
    }
  }
}
}  // namespace

extern "C" {
uint32_t bq_abi_version(void) { return BQ_ABI_VERSION; }
int bq_config_defaults(bq_config *config, uint32_t size) {
  if (!config || size != sizeof(bq_config)) return 0;
  config->abi_version = BQ_ABI_VERSION;
  config->struct_size = sizeof(bq_config);
  copyConfig(*config, Config{});
  return 1;
}
void *bq_create(const char *urdf, const char *left_frame, const char *right_frame,
                const char *shared, const char *left, const char *right) {
  if (!urdf || !left_frame || !right_frame || !left || !right) return nullptr;
  try {
    auto h = std::make_unique<Handle>();
    auto shared_names = split(shared);
    h->shared_count = static_cast<int>(shared_names.size());
    // The explicit capture transform defines the virtual tool. This placeholder
    // only satisfies the legacy no-argument URDF capture path, unused by this ABI.
    h->robot = std::make_unique<RobotModel>(urdf,left_frame,right_frame,left_frame,
                                            split(left),split(right),shared_names);
    h->kin = std::make_unique<RigidToolKinematics>(*h->robot);
    return h.release();
  } catch (...) { return nullptr; }
}
void bq_destroy(void *ptr) { delete static_cast<Handle *>(ptr); }
int bq_nq(void *ptr) { return ptr ? static_cast<Handle *>(ptr)->robot->model().nq : 0; }
int bq_nactive(void *ptr) { return ptr ? static_cast<Handle *>(ptr)->robot->variables() : 0; }
int bq_nmodel(void *ptr) {
  return ptr ? static_cast<int>(static_cast<Handle *>(ptr)->robot->modelJointNames().size()) : 0;
}
int bq_limits(void *ptr, double *lower, double *upper, double *velocity, int n) {
  return checked(ptr, [&](Handle &h) {
    if (!lower || !upper || !velocity || n != h.robot->variables())
      throw std::invalid_argument("limit output layout mismatch");
    for (int i = 0; i < n; ++i) {
      lower[i] = h.robot->lowerLimits()[i];
      upper[i] = h.robot->upperLimits()[i];
      velocity[i] = h.robot->velocityLimits()[i];
    }
  });
}
int bq_set_limits(void *ptr, const double *lower, const double *upper,
                  const double *velocity, int n) {
  return checked(ptr, [&](Handle &h) {
    if (h.qp || !lower || !upper || !velocity || n != h.robot->variables())
      throw std::invalid_argument("invalid limit layout or solver already captured");
    const Eigen::Map<const Eigen::VectorXd> lo(lower,n), hi(upper,n), speed(velocity,n);
    if (!speed.allFinite() || (speed.array() <= 0.).any() ||
        (speed.array() > h.robot->velocityLimits().array()).any())
      throw std::invalid_argument("invalid configured velocity limits");
    h.robot->setPositionLimits(lo,hi);
    h.robot->setVelocityLimits(speed);
  });
}
const char *bq_model_name(void *ptr, int i) {
  if (!ptr) return nullptr;
  auto &names = static_cast<Handle *>(ptr)->robot->modelJointNames();
  return i >= 0 && i < static_cast<int>(names.size()) ? names[i].c_str() : nullptr;
}
const char *bq_active_name(void *ptr, int i) {
  if (!ptr) return nullptr;
  auto &names = static_cast<Handle *>(ptr)->robot->activeJointNames();
  return i >= 0 && i < static_cast<int>(names.size()) ? names[i].c_str() : nullptr;
}
const char *bq_error(void *ptr) {
  return ptr ? static_cast<Handle *>(ptr)->error.c_str() : "null QP handle";
}
int bq_capture(void *ptr, const double *q, int n, const double *tool, const bq_config *config) {
  return checked(ptr, [&](Handle &h) {
    // Failed recapture invalidates every old objective and grasp reference.
    h.qp.reset(); h.kin->reset();
    if (!config || config->abi_version != BQ_ABI_VERSION || config->struct_size != sizeof(bq_config))
      throw std::invalid_argument("QP config ABI mismatch");
    Config native;
    copyConfig(native,*config);
    update(h,q,n);
    try {
      h.kin->capture(pose(tool));
      h.qp = std::make_unique<QpController>(*h.robot,*h.kin,native,
                                           h.robot->activePositions(),h.shared_count);
    } catch (...) { h.kin->reset(); throw; }
  });
}
int bq_capture_axial(void *ptr, const double *q, int n, const double *tool,
                     const double *right_grip, const double *tool_grip,
                     const bq_config *config) {
  return checked(ptr, [&](Handle &h) {
    h.qp.reset(); h.kin->reset();
    if (!config || config->abi_version != BQ_ABI_VERSION || config->struct_size != sizeof(bq_config))
      throw std::invalid_argument("QP config ABI mismatch");
    Config native;
    copyConfig(native,*config);
    update(h,q,n);
    try {
      h.kin->captureAxial(pose(tool),pose(right_grip),pose(tool_grip));
      if (h.kin->relativeError().norm() > native.max_relative_error)
        throw std::invalid_argument("support grip mismatch");
      h.qp = std::make_unique<QpController>(*h.robot,*h.kin,native,
                                           h.robot->activePositions(),h.shared_count);
    } catch (...) { h.kin->reset(); throw; }
  });
}
int bq_shared_targets(void *ptr, const double *targets, int n) {
  return checked(ptr, [&](Handle &h) {
    if (!h.qp) throw std::invalid_argument("solver not captured");
    if (!targets && n == 0) { h.qp->clearSharedTargets(); return; }
    if (!targets || n != h.shared_count || n == 0)
      throw std::invalid_argument("shared target layout mismatch");
    if (h.qp->config().shared_target_weight <= 0.)
      throw std::invalid_argument("shared target weight must be positive");
    h.qp->setSharedTargets(Eigen::Map<const Eigen::VectorXd>(targets,n));
  });
}
int bq_posture_policy(void *ptr, const double *reference, const double *lower,
                      const double *upper, int n) {
  return checked(ptr, [&](Handle &h) {
    try {
      if (!h.qp || !reference || !lower || !upper || n!=h.robot->variables())
        throw std::invalid_argument("posture policy layout or solver not captured");
      h.qp->setPosturePolicy(Eigen::Map<const Eigen::VectorXd>(reference,n),
                            Eigen::Map<const Eigen::VectorXd>(lower,n),
                            Eigen::Map<const Eigen::VectorXd>(upper,n));
    } catch (...) { h.qp.reset(); h.kin->reset(); throw; }
  });
}
int bq_solve(void *ptr, const double *q, int nq, const double *reference, int n,
             const double *target, double dt, double *out, double *metrics) {
  return checked(ptr, [&](Handle &h) {
    if (!h.qp || !reference || !out || !metrics || n != h.robot->variables())
      throw std::invalid_argument("solver not captured or invalid output/reference layout");
    update(h,q,nq);
    const Eigen::Map<const Eigen::VectorXd> ref(reference,n);
    auto solved = h.qp->solve(pose(target),dt,ref);
    if (!solved.ok) throw std::runtime_error(solved.reason);
    // Canonicalize only sub-nanoradian/metre roundoff of captured/accepted
    // positions. Core validation has already rejected actual hard violations.
    const Eigen::VectorXd positions = ref.cwiseMax(h.robot->lowerLimits()).cwiseMin(
        h.robot->upperLimits()) + dt * solved.velocity;
    if (!positions.allFinite() || (positions.array() < h.robot->lowerLimits().array()).any() ||
        (positions.array() > h.robot->upperLimits().array()).any())
      throw std::runtime_error("integrated position exceeds joint limit");
    std::copy(positions.data(),positions.data()+n,out);
    metrics[0] = solved.solve_time_us;
    metrics[1] = solved.relative_error;
    metrics[2] = solved.relative_gain_scale;
  });
}
int bq_tool_pose(void *ptr, const double *q, int n, double *out) {
  return checked(ptr, [&](Handle &h) { update(h,q,n); matrix(h.kin->toolPose(),out); });
}
int bq_relative_pose(void *ptr, const double *q, int n, double *out) {
  return checked(ptr, [&](Handle &h) {
    update(h,q,n); matrix(h.robot->leftPose().actInv(h.robot->rightPose()),out);
  });
}
}  // extern "C"
