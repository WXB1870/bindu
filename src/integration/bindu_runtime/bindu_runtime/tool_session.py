"""Composable teleoperation session, worker and execution lifecycle.

The host supplies scheduling, events and execution transport. ROS subscriptions
and Action servers belong to bimanual_tool_node or the application's node.
"""

import json
import math
import time
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from rclpy.action import GoalResponse
from std_srvs.srv import Trigger
from rcl_interfaces.msg import ParameterDescriptor
from bindu_contracts.teleoperation import load_tool_config, VRFrame, ToolPoseFrame
from bindu_teleoperation.bimanual_session import ToolPause
from bindu_kinematics.bimanual_worker import BimanualWorker
from bindu_interfaces.srv import Lease, ControlExecution
from .common import MODE_STATE_MAX_AGE, seconds, joints_at_rest
from .execution_client import ExecutionClient, parameter, wait_for
from .tool_follow import run_session
from .stop_evidence import stop_feedback_matches

WORKER_POLL_PERIOD = 0.005


class ToolSession:
    def __init__(
        self,
        node,
        yield_control,
        *,
        gripper_config_loader=None,
        tool_config_loader=None,
        execution=None
    ):
        self.profile = node.profile
        self.runtime_mode = node.runtime_mode
        self.simulation = node.simulation
        self.run_id = node.run_id
        self.now = node.now
        self.event = node.event
        self.get_logger = node.get_logger
        self.matches_mode = node.matches_mode
        self.yield_control = yield_control
        parameter(node, "require_recording", True, descriptor=ParameterDescriptor(read_only=True))
        self.require_recording = node.get_parameter("require_recording").value
        self.detailed_diagnostics = parameter(node, "detailed_diagnostics", False).value
        self.next_solution_report = 0.0
        share = Path(get_package_share_directory("bindu_runtime"))
        parameter(node, "tool_config", str(share / "config/robots/g1/bimanual_g1_sim.json"))
        parameter(
            node, "model_root", str(Path(get_package_share_directory("bindu_description")) / "urdf")
        )
        self.cfg = load_tool_config(Path(node.get_parameter("tool_config").value))
        if tool_config_loader is not None:
            self.cfg = tool_config_loader(node, self.cfg)
        for key, default in (
            ("grip_press_threshold", 0.7),
            ("grip_release_threshold", 0.3),
            ("trigger_release_threshold", 0.3),
        ):
            self.cfg[key] = parameter(
                node,
                key,
                self.cfg.get(key, default),
                descriptor=ParameterDescriptor(read_only=True),
            ).value
        self.cfg["pair_max_skew_s"] = parameter(
            node,
            "pair_max_skew_s",
            self.cfg.get("pair_max_skew_s", 0.02),
            descriptor=ParameterDescriptor(read_only=True),
        ).value
        if not math.isfinite(self.cfg["pair_max_skew_s"]) or self.cfg["pair_max_skew_s"] <= 0:
            raise ValueError("VR_PAIR_MAX_SKEW")
        if (
            not 0 <= self.cfg["grip_release_threshold"] < self.cfg["grip_press_threshold"] <= 1
            or not 0 <= self.cfg["trigger_release_threshold"] < 1
        ):
            raise ValueError("VR_INPUT_THRESHOLDS")
        parameter(node, "position_contract", False)
        position_contract = node.get_parameter("position_contract").value
        parameter(
            node, "execution_mode", "position", descriptor=ParameterDescriptor(read_only=True)
        )
        parameter(
            node,
            "position_protocol",
            "legacy_guard",
            descriptor=ParameterDescriptor(read_only=True),
        )
        self.trajectory_reference = (
            not self.simulation
            and position_contract
            and node.get_parameter("position_protocol").value == "session_v3"
        )
        if type(self.cfg.get("rigid_reference_prediction", False)) is not bool:
            raise ValueError("RIGID_REFERENCE_CONFIG")
        from .teleop_deployment import apply_execution_tracking

        self.cfg = apply_execution_tracking(
            self.cfg,
            node.get_parameter("execution_mode").value,
            real=not self.simulation,
            position_contract=position_contract,
            trajectory_reference=self.trajectory_reference,
        )
        parameter(node, "gripper_config", "", descriptor=ParameterDescriptor(read_only=True))
        gripper_path = node.get_parameter("gripper_config").value
        self.gripper_cfg = None
        self.gripper_joints = {}
        if position_contract and gripper_path:
            if gripper_config_loader is None:
                raise ValueError("BIMANUAL_GRIPPER_ADAPTER_REQUIRED")
            self.gripper_cfg, self.gripper_joints = gripper_config_loader(
                json.loads(Path(gripper_path).read_text()), self.profile
            )
        self.independent_enabled = position_contract
        self.cfg["independent_arms_enabled"] = position_contract
        if self.cfg.get("simulation_only") is not (
            self.simulation and not position_contract
        ) or self.cfg.get("collision_scope") not in (
            "empty_scene_only",
            "configured_tool_geometry",
        ):
            raise ValueError("BIMANUAL_CONFIG_RUNTIME_MODE_MISMATCH")
        if not self.simulation and self.cfg.get("collision_scope") != "configured_tool_geometry":
            raise ValueError("BIMANUAL_REAL_REQUIRES_CONFIGURED_GEOMETRY")
        urdf = Path(self.cfg["urdf"])
        self.cfg["urdf"] = str(
            urdf if urdf.is_absolute() else Path(node.get_parameter("model_root").value) / urdf
        )
        parameter(node, "input_source", "vr")
        self.input_source = node.get_parameter("input_source").value
        if self.input_source not in ("vr", "tool_pose"):
            raise ValueError("BIMANUAL_INPUT_SOURCE")
        if self.input_source == "tool_pose" and not self.cfg.get("input_frame"):
            raise ValueError("BIMANUAL_INPUT_FRAME")
        parameter(node, "tool_ground_policy", "configured")
        ground_policy = node.get_parameter("tool_ground_policy").value
        if ground_policy not in ("configured", "report_only") or (
            ground_policy == "report_only" and (not self.simulation or self.input_source != "vr")
        ):
            raise ValueError("BIMANUAL_GROUND_POLICY_OVERRIDE")
        if ground_policy == "report_only":
            self.cfg["demo"]["acceptance"]["ground_policy"] = "report_only"
        parameter(node, "simulation_demo", False)
        demo = node.get_parameter("simulation_demo").value
        if type(demo) is not bool or (demo and (not self.simulation or self.input_source != "vr")):
            raise ValueError("PICO_DEMO_REQUIRES_KINEMATIC_SIMULATION")
        if demo:
            from bindu_teleoperation.simulation_fixture import short_rod_config

            self.cfg = short_rod_config(self.cfg)
        if (
            self.cfg.get(
                "integration_max_sample_gap", self.cfg.get("max_integration_interval", 0.1)
            )
            > self.cfg["input_max_age"]
        ):
            raise ValueError("BIMANUAL_SAMPLE_GAP_EXCEEDS_INPUT_TIMEOUT")
        self.tool_frame = None
        self.groups = tuple(self.cfg["resource_groups"])
        if (
            len(self.groups) not in (2, 3)
            or len(set(self.groups)) != len(self.groups)
            or any(g not in self.profile.groups for g in self.groups)
        ):
            raise ValueError("BIMANUAL_RESOURCE_LAYOUT")
        self.names = tuple(n for g in self.groups for n in self.profile.groups[g])
        self.worker = BimanualWorker(self.cfg, self.profile)
        self.worker_error = ""
        self.worker_result = None
        self.preflight_token = 0
        self.preflight_status = ""
        self.preflight_at = 0.0
        self.preflight_error = ""
        self.preflight_mode = ""
        self.preflight_grasp = ""
        self.frames = {}
        self.health = self.mode_state = None
        self.busy = False
        self.stop_unconfirmed = False
        self.diagnostic_error = ""
        self.execution_instance_id = ""
        self.grasp_continuation = None
        self.pause_control = ToolPause()
        execution = execution or ExecutionClient(node)
        self.execution = execution
        self.lease_client = execution.leases
        self.submit_client = execution.submit
        self.control_client = execution.control
        self.held_reference_client = execution.held_reference if self.trajectory_reference else None

    @property
    def state(self):
        return self.execution.state

    def close(self):
        self.worker.close()

    def request_pause(self, request, reply):
        if not self.busy:
            reply.success, reply.message = False, "TOOL_SESSION_INACTIVE"
            return reply
        try:
            self.pause_control.request(request.data, self.now())
        except ValueError as exc:
            reply.success, reply.message = False, str(exc)
            return reply
        reply.success = True
        reply.message = "PAUSE_REQUESTED" if request.data else "RESUME_REQUESTED"
        event = (
            "BIMANUAL_PAUSED"
            if self.pause_control.phase == "paused"
            else "BIMANUAL_PAUSE_REQUESTED" if request.data else "BIMANUAL_RESUME_REQUESTED"
        )
        self.event(
            event, "STOP_CONFIRMED" if event == "BIMANUAL_PAUSED" else self.pause_control.phase
        )
        return reply

    def on_health(self, msg):
        if msg.run_id == self.run_id and self.matches_mode(msg):
            self.health = msg

    def axial_position_ready(self):
        cfg = self.gripper_cfg
        s = self.state
        if (
            not cfg
            or cfg.get("independent_trigger_control", False)
            or not cfg["calibrated"]
            or "virtual" not in cfg["grippers"]["right"]
            or s is None
            or s.profile_hash != self.profile.digest
            or s.simulated != self.simulation
            or not 0 <= self.now() - seconds(s.feedback_stamp) <= self.cfg["feedback_max_age"]
        ):
            return False
        q = dict(zip(s.joints.name, s.joints.position))
        v = dict(zip(s.joints.name, s.joints.velocity))
        return all(
            n in q
            and n in v
            and math.isfinite(q[n])
            and math.isfinite(v[n])
            and abs(q[n] - cfg["grippers"][side]["virtual" if side == "right" else "closed"])
            <= cfg["position_tolerance"]
            and abs(v[n]) <= cfg["velocity_tolerance"]
            for side, n in self.gripper_joints.items()
        )

    def on_frame(self, msg):
        if self.matches_mode(msg) and msg.side in ("left", "right"):
            # Gripper motion is coordinated separately. Accept the projected
            # axial bit only after the configured opening has measured feedback.
            independent_grippers = bool(
                getattr(self, "gripper_cfg", None)
                and self.gripper_cfg.get("independent_trigger_control", False)
            )
            previous = self.frames.get(msg.side)
            edges = tuple(int(v) for v in msg.button_edges)
            if independent_grippers and len(edges) == 8:
                edges = edges[:6] + (0, 0)
            edge_changed = bool(
                previous
                and previous.source_id == msg.source_id
                and len(edges) == len(previous.button_edges) == 8
                and edges != previous.button_edges
            )
            blocked = not self.simulation and (
                msg.mapping != "pico_snowbot_v1"
                or msg.init
                or not independent_grippers
                and msg.trigger >= self.cfg.get("trigger_release_threshold", 0.3)
                or msg.virtual_grip
                and not self.axial_position_ready()
                or bool(msg.buttons)
                or len(edges) not in (0, 8)
                or edge_changed
                or bool(self.busy and previous and previous.stop)
            )
            self.frames[msg.side] = VRFrame(
                msg.source_id,
                msg.seq,
                seconds(msg.stamp),
                msg.side,
                tuple(msg.pose),
                msg.grip,
                0.0 if independent_grippers else msg.trigger,
                msg.init,
                msg.stop or blocked,
                msg.valid,
                msg.clutch_seq,
                msg.mapping or "v34",
                msg.buttons,
                edges,
                msg.virtual_grip,
                head_pose=tuple(getattr(msg, "head_pose", ())),
                head_stamp=seconds(msg.head_stamp) if getattr(msg, "head_pose", ()) else 0.0,
                head_age_s=getattr(msg, "head_age_s", 0.0),
                head_source_id=getattr(msg, "head_source_id", ""),
                head_seq=int(getattr(msg, "head_seq", 0)),
            )

    def on_tool_frame(self, msg):
        if self.input_source == "tool_pose" and self.matches_mode(msg):
            self.tool_frame = ToolPoseFrame(
                msg.source_id,
                msg.seq,
                seconds(msg.stamp),
                msg.frame_id,
                tuple(msg.pose),
                msg.tight,
                msg.valid,
                msg.stop
                or (
                    not self.simulation
                    and (
                        not msg.tight
                        or bool(self.busy and self.tool_frame and self.tool_frame.stop)
                    )
                ),
            )

    def poll_worker(self):
        # Reuse one timer: creating/destroying a ROS timer at every await
        # stalls callback scheduling during sustained input streams.
        if self.worker_error:
            return
        try:
            item = self.worker.poll()
            if item is not None:
                kind, token, value = item
                if token == self.preflight_token and token < 0:
                    self.preflight_status = "ready" if kind == "result" else "failed"
                    self.preflight_error = "" if kind == "result" else value
                else:
                    self.worker_result = item
        except RuntimeError as exc:
            self.worker_error = str(exc)
            self.event("BIMANUAL_WORKER_FAILED", self.worker_error)

    def startup_input_ready(self, after):
        # A new Action must never consume a STOP projection left by the previous
        # handoff. Wait for an actual subsequent paired input; do not restamp it.
        # Fresh STOP/invalid input still reaches the ordinary rejection guards.
        if self.input_source != "vr":
            return True
        left, right = self.frames.get("left"), self.frames.get("right")
        return bool(
            left
            and right
            and left.source_id == right.source_id
            and left.seq == right.seq
            and min(left.stamp, right.stamp) > after
        )

    def ready(self, request, reply):
        return self.preflight(reply, "bimanual_tool")

    def arms_ready(self, request, reply):
        return self.preflight(reply, "independent_arms")

    def preflight(self, reply, mode):
        reason = (
            "BIMANUAL_EXECUTION_STOP_UNCONFIRMED"
            if getattr(self, "stop_unconfirmed", False)
            else self.worker_error or ("" if self.worker.ready else "BIMANUAL_WORKER_NOT_READY")
        )
        s = self.state
        if not reason and (
            self.busy
            or s is None
            or s.profile_hash != self.profile.digest
            or s.simulated != self.simulation
            or not 0 <= self.now() - seconds(s.feedback_stamp) <= self.cfg["feedback_max_age"]
        ):
            reason = "BIMANUAL_PREFLIGHT_FEEDBACK"
        if not reason:
            grasp = (
                "right_axial"
                if mode == "bimanual_tool"
                and (
                    self.cfg.get("grasp_reference_source") == "measured_axial"
                    or self.axial_position_ready()
                )
                else "rigid"
            )
            if self.preflight_mode != mode or self.preflight_grasp != grasp:
                if self.worker.busy:
                    reply.success, reply.message = False, "BIMANUAL_PREFLIGHT_PENDING"
                    return reply
                self.preflight_status = ""
                self.preflight_mode = mode
                self.preflight_grasp = grasp
            if (
                self.preflight_status == "ready"
                and self.now() - self.preflight_at <= self.cfg["feedback_max_age"]
            ):
                pass
            elif (
                self.preflight_status == "failed"
                and self.now() - self.preflight_at <= self.cfg["feedback_max_age"]
            ):
                reason = self.preflight_error
            elif self.preflight_status == "pending":
                reason = "BIMANUAL_PREFLIGHT_PENDING"
            else:
                measured = dict(zip(s.joints.name, s.joints.position))
                self.preflight_token -= 1
                operation = (
                    "independent_capture" if mode == "independent_arms" else "preflight_capture"
                )
                if self.worker.submit(operation, self.preflight_token, measured, grasp_mode=grasp):
                    self.preflight_status = "pending"
                    self.preflight_at = self.now()
                reason = "BIMANUAL_PREFLIGHT_PENDING"
        reply.success, reply.message = not bool(reason), reason or "BIMANUAL_READY"
        return reply

    def check(self, require_input=True):
        if getattr(self, "stop_unconfirmed", False):
            raise RuntimeError("BIMANUAL_EXECUTION_STOP_UNCONFIRMED")
        now = self.now()
        m = self.mode_state
        if (
            m is None
            or m.profile_hash != self.profile.digest
            or m.phase != "READY"
            or m.mode
            not in (
                ("bimanual_tool", "independent_arms")
                if self.independent_enabled
                else ("bimanual_tool",)
            )
            or not 0 <= now - seconds(m.stamp) < MODE_STATE_MAX_AGE
        ):
            raise RuntimeError("BIMANUAL_MODE_NOT_SELECTED")
        if self.worker_error or not self.worker.ready:
            raise RuntimeError(self.worker_error or "BIMANUAL_WORKER_NOT_READY")
        s = self.state
        if (
            s is None
            or s.profile_hash != self.profile.digest
            or s.simulated != self.simulation
            or not 0 <= now - seconds(s.feedback_stamp) <= self.cfg["feedback_max_age"]
        ):
            raise RuntimeError("BIMANUAL_FEEDBACK_STALE")
        if s.stop_failures:
            raise RuntimeError("BIMANUAL_EXECUTION_STOP_UNCONFIRMED")
        if (
            getattr(s, "missing_feedback_groups", ())
            or len(s.joints.name) != len(s.joints.position)
            or len(s.joints.name) != len(s.joints.velocity)
            or len(set(s.joints.name)) != len(s.joints.name)
            or not all(math.isfinite(q) for q in (*s.joints.position, *s.joints.velocity))
            or not set(self.names) <= set(s.joints.name)
        ):
            raise RuntimeError("BIMANUAL_FEEDBACK_INVALID")
        measured = dict(zip(s.joints.name, s.joints.position))
        if any(
            not self.profile.limits[n][0] - 1e-9 <= measured[n] <= self.profile.limits[n][1] + 1e-9
            for n in self.names
        ):
            raise RuntimeError("BIMANUAL_MEASURED_POSITION_LIMIT")
        if getattr(self, "require_recording", True) and (
            self.health is None
            or not self.health.ready
            or not 0 <= now - seconds(self.health.stamp) < 0.75
        ):
            raise RuntimeError("RECORDER_UNAVAILABLE")
        if not require_input:
            if (
                self.tool_frame is not None
                and self.tool_frame.stop
                and self.input_source == "tool_pose"
            ):
                raise RuntimeError("TOOL_INPUT_STOP")
            if self.input_source == "vr" and any(f.stop for f in self.frames.values()):
                raise RuntimeError("VR_STOP")
            return
        if self.input_source == "tool_pose":
            f = self.tool_frame
            if (
                f is None
                or not f.valid
                or f.stop
                or f.stamp <= seconds(m.changed_at)
                or not 0 <= now - f.stamp <= self.cfg["input_max_age"]
            ):
                raise RuntimeError("TOOL_INPUT_TIMEOUT")
        elif any(
            side not in self.frames
            or self.frames[side].stamp <= seconds(m.changed_at)
            or not 0 <= now - self.frames[side].stamp <= self.cfg["input_max_age"]
            for side in ("left", "right")
        ):
            raise RuntimeError("VR_PAIR_TIMEOUT")

    def goal(self, request):
        try:
            self.check()
        except RuntimeError as exc:
            self.event("BIMANUAL_GOAL_REJECTED", str(exc), request.task_id)
            return GoalResponse.REJECT
        if (
            self.busy
            or not request.task_id
            or request.resource_group != self.mode_state.mode
            or not math.isfinite(request.duration)
            or not 0 < request.duration <= 3600
            or not all(
                c.service_is_ready()
                for c in (self.lease_client, self.submit_client, self.control_client)
            )
        ):
            return GoalResponse.REJECT
        self.pause_control = ToolPause()
        self.busy = True
        return GoalResponse.ACCEPT

    async def pause(self):
        await self.yield_control()

    async def wait(self, future, timeout=0.7):
        return await wait_for(
            future, self.pause, timeout, "BIMANUAL_SERVICE_TIMEOUT", clock=time.monotonic
        )

    def begin_stop(self, lease, release=False):
        # This identity is captured before acquire is dispatched. An idle
        # replacement executor cannot attest to the original executor's stop.
        instance_id = self.execution_instance_id
        requested = self.now()
        deadline = time.monotonic() + self.profile.stop_timeout + 0.5
        if release:
            future = self.lease_client.call_async(
                Lease.Request(operation="release", lease_id=lease.lease_id, epoch=lease.epoch)
            )
        else:
            future = self.control_client.call_async(
                ControlExecution.Request(
                    operation="hold", lease_id=lease.lease_id, epoch=lease.epoch
                )
            )
        return future, requested, deadline, instance_id

    async def execution_revision(self, lease, minimum=0, *, instance_id=None):
        """Read a fresh revision for this lease; late state cannot move it back."""
        if instance_id is None:
            instance_id = self.execution_instance_id
        deadline = time.monotonic() + 0.7
        while time.monotonic() < deadline:
            s = self.state
            if (
                s
                and instance_id
                and s.instance_id == instance_id
                and s.lease_active
                and s.lease_epoch == lease.epoch
                and s.profile_hash == self.profile.digest
                and s.simulated == self.simulation
                and s.revision >= minimum
                and 0 <= self.now() - seconds(s.feedback_stamp) <= self.cfg["feedback_max_age"]
            ):
                return s.revision
            await self.pause()
        raise RuntimeError("BIMANUAL_EXECUTION_REVISION_UNAVAILABLE")

    async def held_reference(self, lease, revision):
        reply = await self.wait(self.held_reference_client.call_async(Trigger.Request()))
        if not reply.success:
            raise RuntimeError(reply.message)
        data = json.loads(reply.message)
        if (
            data["instance_id"] != self.state.instance_id
            or data["profile_hash"] != self.profile.digest
            or data["lease_id"] != lease.lease_id
            or data["epoch"] != lease.epoch
            or data["revision"] != revision
            or not 0 <= self.now() - data["stamp"] <= self.cfg["feedback_max_age"]
            or set(data["positions"]) != set(self.names)
            or not all(math.isfinite(q) for q in data["positions"].values())
        ):
            raise RuntimeError("BIMANUAL_HELD_REFERENCE_CHANGED")
        return tuple(data["positions"][n] for n in self.names)

    async def stop_pending(self, lease, pending, revision):
        # Sending hold precedes every await. Stop increments Executor.revision;
        # an old SubmitMotion carries the previous revision and cannot restart.
        request = self.begin_stop(lease)
        accepted = 0
        rejection = ""
        if pending is not None:
            reply = await self.wait(pending)
            if reply.accepted:
                accepted = 1
                revision = max(revision, reply.revision)
            elif reply.code not in ("STALE_REVISION", "STOPPING", "STALE_OR_UNORDERED"):
                rejection = reply.code
        if not await self.stop(lease, request=request):
            return False, accepted, revision
        # Require the stop's newer state, never a delayed pre-stop snapshot.
        revision = await self.execution_revision(lease, revision + 1, instance_id=request[3])
        if rejection:
            raise RuntimeError(rejection)
        return True, accepted, revision

    def cleanup_event(self, name, detail, task_id=""):
        # Cleanup and its stop barrier must survive a failed diagnostic sink.
        try:
            self.event(name, detail, task_id)
        except Exception as exc:
            self.diagnostic_error = name + ":" + type(exc).__name__ + ":" + str(exc)
            try:
                self.get_logger().warning(self.diagnostic_error, throttle_duration_sec=5.0)
            except Exception:
                pass  # The retained field remains inspectable if logging also fails.

    async def stop(self, lease, release=False, *, request=None):
        instance_id = self.execution_instance_id
        requested = self.now()
        expected_epoch = lease.epoch + 1 if release else lease.epoch
        revoked_epoch = None
        reason = "DEADLINE_EXPIRED"
        try:
            future, requested, deadline, instance_id = request or self.begin_stop(lease, release)
            reply = await self.wait(future)
            if not reply.ok:
                # A timeout may already have revoked this lease and begun braking.
                # Releasing that old lease fails; the measured stop is still pending.
                if not (
                    release
                    and reply.code == "INVALID_LEASE"
                    and instance_id
                    and not reply.lease_id
                    and reply.epoch == lease.epoch + 1
                    and reply.profile_hash == self.profile.digest
                ):
                    raise RuntimeError("BIMANUAL_STOP_REPLY_REJECTED:" + reply.code)
                revoked_epoch = reply.epoch
            next_renewal = time.monotonic()
            while time.monotonic() < deadline:
                s = self.state
                if s and ("EMERGENCY_STOP" in s.code or s.code == "STOP_FEEDBACK_TIMEOUT"):
                    raise RuntimeError("BIMANUAL_EXECUTION_STOP_FAILED:" + s.code)
                if revoked_epoch is not None and not (
                    s
                    and s.instance_id == instance_id
                    and s.lease_epoch == revoked_epoch
                    and s.profile_hash == self.profile.digest
                    and s.simulated == self.simulation
                    and not s.lease_active
                ):
                    await self.pause()
                    continue
                if stop_feedback_matches(
                    s,
                    instance_id=instance_id,
                    epoch=expected_epoch,
                    lease_active=not release,
                    profile_hash=self.profile.digest,
                    simulated=self.simulation,
                    requested=requested,
                    now=self.now(),
                    max_age=self.cfg["feedback_max_age"],
                    at_rest=lambda joints: joints_at_rest(
                        joints,
                        self.names,
                        self.profile.feedback_tolerances.get("joint_velocity", 0.01),
                    ),
                ):
                    return True
                # Waiting for a hold suspends the session's normal renewal
                # loop. Keep only that still-owned lease alive while braking;
                # expiry/revocation is never accepted as same-lease rest.
                if (
                    not release
                    and time.monotonic() >= next_renewal
                    and s
                    and s.instance_id == instance_id
                    and s.lease_epoch == lease.epoch
                    and s.lease_active
                    and s.profile_hash == self.profile.digest
                    and s.simulated == self.simulation
                    and s.state != "FAILED"
                    and not s.stop_failures
                    and 0 <= self.now() - seconds(s.feedback_stamp) < self.cfg["feedback_max_age"]
                ):
                    renewal = self.lease_client.call_async(
                        Lease.Request(operation="renew", lease_id=lease.lease_id, epoch=lease.epoch)
                    )
                    renewed = await self.wait(
                        renewal, min(0.7, max(0.0, deadline - time.monotonic()))
                    )
                    if not (
                        renewed.ok
                        and renewed.lease_id == lease.lease_id
                        and renewed.epoch == lease.epoch
                        and renewed.profile_hash == self.profile.digest
                    ):
                        raise RuntimeError("BIMANUAL_STOP_RENEW_REJECTED:" + renewed.code)
                    next_renewal = time.monotonic() + 0.3
                await self.pause()
        except Exception as exc:
            reason = (
                type(exc).__name__ + ":" + str(exc)
                if not isinstance(exc, RuntimeError)
                else str(exc)
            )
        self.stop_unconfirmed = True
        s = self.state
        self.cleanup_event(
            "BIMANUAL_STOP_UNCONFIRMED",
            json.dumps(
                {
                    "reason": reason,
                    "state": s.state if s else "",
                    "code": s.code if s else "",
                    "requested_instance_id": instance_id,
                    "observed_instance_id": s.instance_id if s else "",
                    "expected_epoch": expected_epoch,
                    "observed_epoch": s.lease_epoch if s else None,
                    "lease_active": s.lease_active if s else None,
                    "reference_count": len(s.reference.name) if s else None,
                    "max_controlled_velocity": (
                        max(
                            (
                                abs(v)
                                for n, v in zip(s.joints.name, s.joints.velocity)
                                if n in self.names
                            ),
                            default=None,
                        )
                        if s
                        else None
                    ),
                }
            ),
        )
        return False

    def solution_report_due(self):
        now = self.now()
        if self.detailed_diagnostics or now >= self.next_solution_report:
            self.next_solution_report = now + 0.1
            return True
        return False

    async def execute(self, goal):
        return await run_session(self, goal)
