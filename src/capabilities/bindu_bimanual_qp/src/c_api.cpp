// C ABI for the optional Bindu worker. No ROS or robot-specific joint names.
#include "dual_arm_tool_qp/core.hpp"
#include <pinocchio/spatial/se3.hpp>
#include <algorithm>
#include <cstring>
#include <memory>
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
    auto name = value.substr(start, end == std::string::npos ? end : end - start);
    if (name.empty()) throw std::invalid_argument("empty joint name");
    out.push_back(name);
    if (end == std::string::npos) break;
    start = end + 1;
  }
  return out;
}
pinocchio::SE3 pose(const double *a) {
  Eigen::Matrix3d r;
  Eigen::Vector3d t;
  for (int i = 0; i < 3; ++i) {
    t[i] = a[4*i+3];
    for (int j = 0; j < 3; ++j) r(i,j) = a[4*i+j];
  }
  if (!r.allFinite() || !t.allFinite() || !r.isUnitary(1e-5) || r.determinant() < .999 ||
      std::abs(a[12]) > 1e-8 || std::abs(a[13]) > 1e-8 ||
      std::abs(a[14]) > 1e-8 || std::abs(a[15]-1.) > 1e-8)
    throw std::invalid_argument("invalid SE3 matrix");
  return pinocchio::SE3(r,t);
}
void matrix(const pinocchio::SE3 &p, double *a) {
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) a[4*i+j] = p.rotation()(i,j);
    a[4*i+3] = p.translation()[i];
  }
  a[12] = a[13] = a[14] = 0.; a[15] = 1.;
}
struct Handle {
  std::unique_ptr<RobotModel> robot;
  std::unique_ptr<RigidToolKinematics> kin;
  std::unique_ptr<QpController> qp;
  Config config;
  int shared_count{0};
  std::string error;
};
void update(Handle *h, const double *q, int n) {
  if (!q || n != h->robot->model().nq) throw std::invalid_argument("joint layout mismatch");
  h->robot->update(Eigen::Map<const Eigen::VectorXd>(q,n));
}
}  // namespace

extern "C" {
void *bq_create(const char *urdf, const char *left_frame, const char *right_frame,
                const char *shared, const char *left, const char *right) {
  try {
    auto h = std::make_unique<Handle>();
    auto shared_names = split(shared);
    h->shared_count = static_cast<int>(shared_names.size());
    // The tool is a fixed transform supplied at capture, so the left frame
    // satisfies the source core's URDF tool-frame lookup.
    h->robot = std::make_unique<RobotModel>(urdf,left_frame,right_frame,left_frame,
                                            split(left),split(right),shared_names);
    h->kin = std::make_unique<RigidToolKinematics>(*h->robot);
    return h.release();
  } catch (...) { return nullptr; }
}
void bq_destroy(void *ptr) { delete static_cast<Handle *>(ptr); }
int bq_nq(void *ptr) { return static_cast<Handle *>(ptr)->robot->model().nq; }
int bq_nactive(void *ptr) { return static_cast<Handle *>(ptr)->robot->variables(); }
int bq_nmodel(void *ptr) { return static_cast<int>(static_cast<Handle *>(ptr)->robot->modelJointNames().size()); }
void bq_limits(void *ptr, double *lower, double *upper, double *velocity) {
  auto *h = static_cast<Handle *>(ptr);
  for (int i = 0; i < h->robot->variables(); ++i) {
    lower[i] = h->robot->lowerLimits()[i];
    upper[i] = h->robot->upperLimits()[i];
    velocity[i] = h->robot->velocityLimits()[i];
  }
}
int bq_set_speed_limits(void *ptr, const double *velocity, int n) {
  auto *h = static_cast<Handle *>(ptr);
  try {
    if (!velocity || n != h->robot->variables())
      throw std::invalid_argument("velocity layout mismatch");
    h->robot->setVelocityLimits(Eigen::Map<const Eigen::VectorXd>(velocity,n));
    h->error.clear();
    return 1;
  } catch (const std::exception &e) { h->error = e.what(); return 0; }
}
const char *bq_model_name(void *ptr, int i) {
  auto &names = static_cast<Handle *>(ptr)->robot->modelJointNames();
  return i >= 0 && i < static_cast<int>(names.size()) ? names[i].c_str() : nullptr;
}
const char *bq_active_name(void *ptr, int i) {
  auto &names = static_cast<Handle *>(ptr)->robot->activeJointNames();
  return i >= 0 && i < static_cast<int>(names.size()) ? names[i].c_str() : nullptr;
}
const char *bq_error(void *ptr) { return static_cast<Handle *>(ptr)->error.c_str(); }
int bq_capture(void *ptr, const double *q, int n, const double *tool, double relative_gain,
               double max_relative_error, double max_linear_speed, double max_angular_speed) {
  auto *h = static_cast<Handle *>(ptr);
  try {
    update(h,q,n);
    h->kin->capture(pose(tool));
    h->config.relative_gain = relative_gain;
    h->config.max_relative_error = max_relative_error;
    h->config.max_linear_speed = max_linear_speed;
    h->config.max_angular_speed = max_angular_speed;
    h->qp = std::make_unique<QpController>(*h->robot,*h->kin,h->config,
                                           h->robot->activePositions(),h->shared_count);
    h->error.clear();
    return 1;
  } catch (const std::exception &e) { h->error = e.what(); h->qp.reset(); return 0; }
}
int bq_solve(void *ptr, const double *q, int n, const double *target, double dt,
             double *out, double *metrics) {
  auto *h = static_cast<Handle *>(ptr);
  try {
    if (!h->qp || !std::isfinite(dt) || dt <= 0. || dt > .1)
      throw std::invalid_argument("solver not captured or invalid timestep");
    update(h,q,n);
    auto solved = h->qp->solve(pose(target));
    metrics[0] = solved.solve_time_us;
    metrics[1] = solved.relative_error;
    metrics[2] = solved.relative_gain_scale;
    if (!solved.ok) { h->error = solved.reason; return 0; }
    const Eigen::VectorXd positions = h->robot->activePositions() + dt * solved.velocity;
    for (int i = 0; i < positions.size(); ++i) {
      if (positions[i] < h->robot->lowerLimits()[i] || positions[i] > h->robot->upperLimits()[i])
        throw std::runtime_error("integrated position exceeds joint limit: " +
                                 h->robot->activeJointNames()[i] + " q=" +
                                 std::to_string(h->robot->activePositions()[i]) + " target=" +
                                 std::to_string(positions[i]) + " bounds=[" +
                                 std::to_string(h->robot->lowerLimits()[i]) + "," +
                                 std::to_string(h->robot->upperLimits()[i]) + "]");
      out[i] = positions[i];
    }
    h->error.clear();
    return 1;
  } catch (const std::exception &e) { h->error = e.what(); return 0; }
}
int bq_tool_pose(void *ptr, const double *q, int n, double *out) {
  auto *h = static_cast<Handle *>(ptr);
  try { update(h,q,n); matrix(h->kin->toolPose(),out); h->error.clear(); return 1; }
  catch (const std::exception &e) { h->error = e.what(); return 0; }
}
double bq_flange_distance(void *ptr, const double *q, int n) {
  auto *h = static_cast<Handle *>(ptr);
  try {
    update(h,q,n);
    return (h->robot->leftPose().translation()-h->robot->rightPose().translation()).norm();
  } catch (const std::exception &e) { h->error = e.what(); return -1.; }
}
}  // extern "C"
