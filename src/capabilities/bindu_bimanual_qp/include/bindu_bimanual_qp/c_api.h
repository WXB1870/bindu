#ifndef BINDU_BIMANUAL_QP_C_API_H
#define BINDU_BIMANUAL_QP_C_API_H

#include <stdint.h>

#define BQ_ABI_VERSION 1u

#ifdef __cplusplus
extern "C" {
#endif

/* Stable C layout, independent of the internal C++ Config. SI units; rotations
 * in radians. Initialize with bq_config_defaults before overriding fields. */
typedef struct bq_config {
  uint32_t abi_version;
  uint32_t struct_size;
  double tool_gain;
  double relative_gain;
  double posture_gain;
  double posture_weight;
  double shared_target_weight;
  double shared_target_gain;
  double regularization;
  double max_linear_speed;
  double max_angular_speed;
  double max_relative_error;
  double limit_margin;
  double damper_gain;
  double equality_tolerance;
} bq_config;

uint32_t bq_abi_version(void);
int bq_config_defaults(bq_config *config, uint32_t size);
void *bq_create(const char *urdf, const char *left_frame, const char *right_frame,
                const char *shared, const char *left, const char *right);
void bq_destroy(void *handle);
/* Counts return 0, names return NULL for invalid inputs. Status functions return
 * 1 on success, 0 on failure. Error/name strings remain owned by the handle.
 * Handles must be live or NULL; arbitrary/destroyed pointers are not supported. */
int bq_nq(void *handle);
int bq_nactive(void *handle);
int bq_nmodel(void *handle);
const char *bq_model_name(void *handle, int index);
const char *bq_active_name(void *handle, int index);
const char *bq_error(void *handle);
int bq_limits(void *handle, double *lower, double *upper, double *velocity, int n);
/* Configuration may only tighten model limits and must precede capture. */
int bq_set_limits(void *handle, const double *lower, const double *upper,
                  const double *velocity, int n);
/* q follows model-name order; all poses are row-major 4x4 SE(3) matrices. */
int bq_capture(void *handle, const double *q, int nq, const double *left_to_tool,
               const bq_config *config);
/* Additive ABI v1 capability; old rigid callers/layouts are unchanged. The
 * support grip frame +Z is the shaft axis, with translation at the grasp point.
 * This is a kinematic constraint only, not a gripper force command. Call after
 * confirmed stop. Failed capture invalidates all previous grasp references. */
int bq_capture_axial(void *handle, const double *q, int nq, const double *left_to_tool,
                     const double *right_to_grip, const double *tool_to_grip,
                     const bq_config *config);
/* NULL,0 clears the optional shared-joint posture objective. */
int bq_shared_targets(void *handle, const double *targets, int n);
/* Additive ABI v1: nonnegative extra squared-velocity cost for the shared
 * group. Default zero, no pose target or hard lock. Set after each capture;
 * invalid configuration clears capture. The bq_config layout is unchanged. */
int bq_shared_motion_weight(void *handle, double weight);
/* Optional additive ABI v1 policy. All arrays follow active-name order and
 * model joint units. Bounds only tighten model/profile limits. Reference is a
 * soft posture preference, independent of tool/grasp capture. Failure clears
 * the capture so a partially configured policy cannot be used. */
int bq_posture_policy(void *handle, const double *reference,
                      const double *lower, const double *upper, int n);
/* Optional ABI v1 extension: inset the reference velocity damper; physical
 * posture bounds and measured capture checks remain unchanged. */
int bq_posture_reference_margin(void *handle, double margin);
/* Additive ABI v1: local tool tracking point [x,y,z] metres. Default zero.
 * Changes only the tracking task/Jacobian; tool FK and grasp/CAD stay fixed.
 * Set after each capture. Supplied world linear feedforward is at this point. */
int bq_tracking_point(void *handle, const double *point);
/* reference/out follow active-name order. reference is the last accepted
 * command (capture feedback for the first step), never an unaccepted result.
 * dt is the elapsed reference sampling interval, 0 < dt <= 0.1 s. The caller
 * owns acceptance/time/reset state. Kinematics use the supplied q; tracking
 * callers may supply accepted-reference q and a separate measured tool pose.
 * metrics has 3 doubles: QP solve microseconds, relative error, recovery scale.
 * Outputs are only valid on success; no C function sends device commands. */
int bq_solve(void *handle, const double *q, int nq, const double *reference, int n,
             const double *target, double dt, double *out, double *metrics);
/* Additive v1 entry: optional 6-vector [world linear m/s, world angular rad/s]
 * at the configured tracking point (default tool origin), and optional measured
 * tool-origin pose for feedback error.
 * Both NULL preserve the old pose-only solve exactly. */
int bq_solve_tracking(void *handle, const double *q, int nq, const double *reference, int n,
                        const double *target, const double *world_velocity, const double *measured_tool, double dt,
                        double *out, double *metrics);
int bq_tool_pose(void *handle, const double *q, int nq, double *out);
int bq_relative_pose(void *handle, const double *q, int nq, double *out);
/* Additive ABI v1 ordinary-arms capability. No rigid-tool closure equality;
 * shared joints and inactive arms are held. active_mask: left=1, right=2. */
int bq_capture_independent(void *handle, const double *q, int nq, const bq_config *config);
int bq_flange_poses(void *handle, const double *q, int nq, double *left, double *right);
int bq_solve_independent(void *handle, const double *q, int nq, const double *reference, int n,
                         const double *left, const double *right, int active_mask,
                         double dt, double *out, double *metrics);

#ifdef __cplusplus
}
#endif
#endif
