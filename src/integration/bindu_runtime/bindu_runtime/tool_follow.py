"""Following transaction from paired input to accepted joint references.

ToolSession owns shared worker/feedback state. Each invocation owns its
in-flight solve, submission, lease and bounded cleanup.
"""

import json
import math
import time
from bindu_contracts.teleoperation import reference_tracking_budget, joint_reference_limits
from bindu_teleoperation.bimanual_session import BimanualSession, ToolPoseSession, AcceptedReference
from bindu_teleoperation.independent_session import IndependentArmsSession
from bindu_interfaces.action import TeleopSession as TeleopAction
from bindu_interfaces.msg import MotionCommand
from bindu_interfaces.srv import Lease, SubmitMotion
from .common import seconds, stamp
from .session_operation import LeaseRejection, SessionResult

WORKER_POLL_PERIOD = 0.005


async def run_session(self, goal):
    mode = self.mode_state
    session_type = (
        IndependentArmsSession
        if mode.mode == "independent_arms"
        else ToolPoseSession if self.input_source == "tool_pose" else BimanualSession
    )
    continuation = self.grasp_continuation
    self.grasp_continuation = None
    session = session_type(
        self.cfg,
        mode.generation,
        seconds(mode.changed_at),
        **({"continuation": continuation} if self.input_source == "vr" else {})
    )
    reference = AcceptedReference(
        self.cfg.get("max_integration_interval", 0.1),
        max_sample_gap=self.cfg.get("integration_max_sample_gap"),
    )
    result = SessionResult(simulated=self.simulation)
    lease = acquisition = pending = renewal = None
    accepted = 0
    last_feedback_mode = None
    next_feedback_at = 0.0
    stopped = True
    canceled = False
    worker_token = 0
    awaiting = ""
    work_seq = -1
    work_stamp = 0.0
    work_sampled_at = 0.0
    work_sampled_monotonic = 0.0
    work_positions = None
    work_target = None
    work_dt = None
    work_gap = None
    work_feedback = work_reference = None
    work_execution_reference = None
    pending_reference = None
    trajectory_bootstrap = True
    trajectory_reference_deadline = None
    trajectory_sample_at = None
    last_mode = "idle"
    last_grasp_generation = 0
    deferral = None
    reference_deadline = None

    def end_deferral(outcome):
        nonlocal deferral
        if deferral is not None:
            self.event(
                "BIMANUAL_SOLVE_DEFERRED_END",
                json.dumps(
                    dict(
                        deferral,
                        outcome=outcome,
                        duration_s=time.monotonic() - deferral["started_at"],
                        session_code=getattr(result, "code", ""),
                    )
                ),
                goal.request.task_id,
            )
            deferral = None

    self.worker_result = None
    self.preflight_status = ""
    acquisition_epoch = None

    def record_acquisition_rejection(reply):
        if (
            not reply.lease_id
            and reply.epoch == acquisition_epoch
            and reply.profile_hash == self.profile.digest
            and self.state is not None
            and self.state.instance_id == self.execution_instance_id
        ):
            # Carry explicit no-grant evidence as data. The coordinator must
            # still observe fresh baseline-epoch rest from this executor.
            result.lease_rejection = LeaseRejection(
                instance_id=self.execution_instance_id,
                epoch=reply.epoch,
                profile_hash=reply.profile_hash,
                reason=reply.code,
            )
        return reply.code

    try:
        if getattr(self, "stop_unconfirmed", False):
            raise RuntimeError("BIMANUAL_EXECUTION_STOP_UNCONFIRMED")
        self.execution_instance_id = self.state.instance_id if self.state else ""
        if not self.execution_instance_id:
            raise RuntimeError("BIMANUAL_EXECUTION_REVISION_UNAVAILABLE")
        # Even a transport exception may follow request dispatch.
        stopped = False
        acquisition_epoch = self.state.lease_epoch
        acquisition = self.lease_client.call_async(
            Lease.Request(operation="acquire", owner=session.identity, resources=list(self.groups))
        )
        lease = await self.wait(acquisition)
        if not lease.ok:
            stopped = True  # Explicit rejection did not grant this session control.
            raise RuntimeError(record_acquisition_rejection(lease))
        if lease.profile_hash != self.profile.digest:
            raise RuntimeError("PROFILE_MISMATCH")
        # Service round-trips can span a DDS scheduling gap. Wait for
        # fresh mode, input, feedback and recorder samples before moving.
        startup_input_after = self.now()
        startup_deadline = time.monotonic() + 0.7
        while True:
            if self.mode_state and self.mode_state.generation != session.generation:
                raise RuntimeError("BIMANUAL_MODE_GENERATION_CHANGED")
            try:
                self.check()
                if not self.startup_input_ready(startup_input_after):
                    raise RuntimeError("BIMANUAL_START_INPUT_PENDING")
                break
            except RuntimeError:
                if time.monotonic() >= startup_deadline:
                    raise
                await self.pause()
        submit_revision = await self.execution_revision(lease)
        execution_instance_id = self.execution_instance_id
        self.event(
            "BIMANUAL_STARTED",
            json.dumps(
                {
                    "session_id": session.identity,
                    "teleop_mode": mode.mode,
                    "mode_generation": session.generation,
                    "resource_groups": self.groups,
                    "collision_scope": (
                        "robot_body_only"
                        if isinstance(session, IndependentArmsSession)
                        else self.cfg["collision_scope"]
                    ),
                    "tool_frame": self.cfg["left_frame"],
                    "tool_transform": (
                        None
                        if isinstance(session, IndependentArmsSession)
                        else self.cfg["left_to_tool"]
                    ),
                    "position_scale": self.cfg["position_scale"],
                    "rotation_scale": 1.0,
                    "tracking_feedback": self.cfg.get("tracking_feedback", "measured"),
                    "vr_tool_mapping": self.cfg.get("vr_tool_mapping", "legacy_wrist"),
                    "pose_filter": (
                        self.cfg.get("pose_filter") if self.input_source == "vr" else None
                    ),
                    "pose_filter_space": (
                        (
                            "per_hand_robot_pose"
                            if isinstance(session, IndependentArmsSession)
                            else "paired_controller_pose"
                        )
                        if self.input_source == "vr"
                        else None
                    ),
                    "mapping_frame": "robot_base",
                    "geometry_kind": self.cfg.get("demo", {}).get("geometry_kind", "shovel"),
                    "max_integration_interval": reference.max_interval,
                    "max_sample_gap": reference.max_sample_gap,
                    "right_axial_grasp": bool(session.axial),
                    "grip_actuation": "kinematic_constraint_only",
                }
            ),
            goal.request.task_id,
        )
        started = time.monotonic()
        next_renewal = started
        pending_at = renewal_at = 0.0
        while True:
            now = time.monotonic()
            if goal.is_cancel_requested:
                canceled = True
                break
            if now - started >= goal.request.duration:
                break
            self.check(require_input=self.pause_control.phase == "follow")
            if self.mode_state.generation != session.generation:
                raise RuntimeError("BIMANUAL_MODE_GENERATION_CHANGED")
            s = self.state
            if s.command_id.startswith(session.identity + "_") and s.state == "FAILED":
                raise RuntimeError(s.code)
            if (
                s.instance_id != execution_instance_id
                or not s.lease_active
                or s.lease_epoch != lease.epoch
            ):
                raise RuntimeError("BIMANUAL_EXECUTION_AUTHORITY_CHANGED")
            if (
                pending is not None
                and self.pause_control.phase != "pausing"
                and now - pending_at > 0.7
            ):
                raise RuntimeError("BIMANUAL_SUBMIT_TIMEOUT")
            if deferral is not None and now >= reference_deadline:
                raise RuntimeError("BIMANUAL_SOLVE_DEFERRED_TIMEOUT")
            if renewal is not None:
                if renewal.done():
                    if not renewal.result().ok:
                        raise RuntimeError("LEASE_RENEW_FAILED")
                    renewal = None
                elif now - renewal_at > 0.7:
                    raise RuntimeError("LEASE_RENEW_TIMEOUT")
            if now >= next_renewal and renewal is None:
                renewal = self.lease_client.call_async(
                    Lease.Request(operation="renew", lease_id=lease.lease_id, epoch=lease.epoch)
                )
                renewal_at, next_renewal = now, now + 0.3
            if self.pause_control.phase == "pausing":
                end_deferral("intent_interrupted")
                worker_token += 1
                self.worker_result = None
                awaiting = ""
                ok, settled, submit_revision = await self.stop_pending(
                    lease, pending, submit_revision
                )
                accepted += settled
                pending = None
                if not ok:
                    raise RuntimeError("BIMANUAL_PAUSE_STOP_UNCONFIRMED")
                self.worker_result = None
                reference.reset()
                pending_reference = None
                session.reanchor()
                self.pause_control.confirm_stopped()
                self.event("BIMANUAL_PAUSED", "STOP_CONFIRMED", goal.request.task_id)
            if self.pause_control.phase == "paused":
                goal.publish_feedback(
                    TeleopAction.Feedback(
                        session_id=session.identity,
                        mode="paused:" + session.grasp_mode,
                        accepted_commands=accepted,
                    )
                )
                await self.pause()
                continue
            if self.pause_control.phase == "waiting_input":
                frame_stamp = (
                    (self.tool_frame.stamp if self.tool_frame else float("-inf"))
                    if self.input_source == "tool_pose"
                    else min((f.stamp for f in self.frames.values()), default=float("-inf"))
                )
                frame_seq = (
                    (self.tool_frame.seq if self.tool_frame else -1)
                    if self.input_source == "tool_pose"
                    else min((f.seq for f in self.frames.values()), default=-1)
                )
                if frame_seq <= session.seq or not self.pause_control.accepts(frame_stamp):
                    if self.now() - self.pause_control.resume_after > self.cfg["input_max_age"]:
                        raise RuntimeError("TOOL_RESUME_INPUT_TIMEOUT")
                    await self.pause()
                    continue
                self.check()
                self.event("BIMANUAL_RESUMED", "FRESH_INPUT", goal.request.task_id)
            if self.input_source == "tool_pose":
                session.ingest(self.tool_frame, self.now())
            else:
                left, right = self.frames["left"], self.frames["right"]
                if left.source_id != right.source_id or left.seq != right.seq:
                    await self.pause()
                    continue
                session.ingest(left, right, self.now())
            if session.grasp_generation != last_grasp_generation:
                end_deferral("grasp_changed")
                worker_token += 1
                self.worker_result = None
                awaiting = ""
                ok, settled, submit_revision = await self.stop_pending(
                    lease, pending, submit_revision
                )
                accepted += settled
                pending = None
                if not ok:
                    raise RuntimeError("BIMANUAL_GRASP_STOP_UNCONFIRMED")
                self.worker_result = None
                reference.reset()
                pending_reference = None
                if isinstance(session, IndependentArmsSession):
                    session.reanchor(preserve_active=True)
                else:
                    session.reanchor()
                last_grasp_generation = session.grasp_generation
                self.event(
                    "BIMANUAL_GRASP_REQUESTED",
                    json.dumps(
                        {
                            "mode": session.grasp_mode,
                            "generation": last_grasp_generation,
                            "simulated": self.simulation,
                            "physical_gripper_command": False,
                        }
                    ),
                    goal.request.task_id,
                )
                continue
            if session.mode != last_mode:
                if session.mode == "idle":
                    end_deferral("clutch_released")
                    worker_token += 1
                    self.worker_result = None
                    awaiting = ""
                    ok, settled, submit_revision = await self.stop_pending(
                        lease, pending, submit_revision
                    )
                    accepted += settled
                    pending = None
                    if not ok:
                        raise RuntimeError("BIMANUAL_HOLD_UNCONFIRMED")
                    self.worker_result = None
                    reference.reset()
                    pending_reference = None
                last_mode = session.mode
                self.event("BIMANUAL_CLUTCH", session.mode, goal.request.task_id)
            # Validate source/clutch/grasp transitions before a receipt can
            # advance local integration. A submitted command may already
            # have executed; stop/settle it without reviving its reference.
            if pending is not None and self.pause_control.phase != "pausing":
                if pending.done():
                    reply = pending.result()
                    if not reply.accepted:
                        raise RuntimeError(reply.code)
                    if not work_stamp <= self.now() < work_stamp + self.cfg["command_max_age"]:
                        raise RuntimeError("BIMANUAL_SUBMIT_EXPIRED")
                    reference.accept(*pending_reference)
                    reference_deadline = work_sampled_monotonic + reference.max_sample_gap
                    end_deferral("software_acceptance")
                    submit_revision = max(submit_revision, reply.revision)
                    if self.trajectory_reference:
                        trajectory_bootstrap = False
                        trajectory_reference_deadline = time.monotonic() + reference.max_sample_gap
                    self.event(
                        "BIMANUAL_SUBMIT_TIMING",
                        json.dumps(
                            {
                                "input_seq": work_seq,
                                "reference_sampled_at": reference.sampled_at,
                                "acceptance_scope": "software_execution_service",
                                "executor_instance_id": execution_instance_id,
                                "lease_epoch": lease.epoch,
                                "source_stamp": work_stamp,
                                "source_valid_for_s": self.cfg["command_max_age"],
                                "acknowledged_at": self.now(),
                                "submit_round_trip_s": now - pending_at,
                                "server_monotonic_s": now,
                            }
                        ),
                        goal.request.task_id,
                        command.command_id,
                    )
                    pending_reference = None
                    accepted += 1
                    pending = None
            s = self.state
            if s.command_id.startswith(session.identity + "_") and s.state == "FAILED":
                raise RuntimeError(s.code)
            if self.worker_result is not None:
                kind, token, value = self.worker_result
                self.worker_result = None
                if token == worker_token and session.mode == "follow" and awaiting:
                    geometry = (
                        self.worker.diagnostic[1]
                        if self.worker.diagnostic and self.worker.diagnostic[0] == token
                        else None
                    )
                    if geometry is not None and geometry["scale"] < 1.0:
                        self.event(
                            "BIMANUAL_GEOMETRY_STEP",
                            json.dumps(
                                dict(
                                    geometry, input_seq=work_seq, control_sampled_at=work_sampled_at
                                )
                            ),
                            goal.request.task_id,
                        )
                    admission = None
                    if kind == "result" and awaiting == "solve":
                        # A solver result is bounded against its sampled
                        # feedback. Recheck the complete candidate against
                        # the current observation before any submission.
                        self.check(require_input=self.pause_control.phase == "follow")
                        s = self.state
                        if (
                            s.instance_id != execution_instance_id
                            or not s.lease_active
                            or s.lease_epoch != lease.epoch
                        ):
                            raise RuntimeError("BIMANUAL_EXECUTION_AUTHORITY_CHANGED")
                        positions, metrics = value
                        if len(positions) != len(self.names) or not all(
                            math.isfinite(q) for q in positions
                        ):
                            raise RuntimeError("BIMANUAL_NONFINITE_RESULT")
                        measured = dict(zip(s.joints.name, s.joints.position))
                        limit = float(self.cfg.get("max_reference_error", 0.15))
                        if not math.isfinite(limit) or limit <= 0:
                            raise RuntimeError("BIMANUAL_REFERENCE_LIMIT_CONFIG")
                        hard_limits = joint_reference_limits(self.names, limit, self.cfg.get("joint_reference_error_limits", {}))
                        leads = [abs(q - measured[n]) for n, q in zip(self.names, positions)]
                        executed = None
                        # No servo trajectory exists before the first accepted
                        # target; use the authenticated hold/goal guard once.
                        if self.trajectory_reference and not trajectory_bootstrap:
                            from bindu_runtime.stop_evidence import sampled_nominal_reference

                            executed, _ = sampled_nominal_reference(
                                s,
                                self.names,
                                lease.epoch,
                                self.now(),
                                min(0.1, self.cfg["feedback_max_age"]),
                            )
                        reference_lead, budgets = reference_tracking_budget(
                            work_reference,
                            tuple(measured[n] for n in self.names),
                            max(hard_limits),
                            executed=executed,
                            target_limit=self.cfg.get("max_target_lead_error", limit),
                        )
                        admission = dict(
                            feedback_stamp=seconds(s.feedback_stamp),
                            feedback={n: measured[n] for n in self.names},
                            candidate_lead_rad=max(leads),
                            reference_lead_rad=reference_lead,
                            reference_basis=(
                                "executed" if executed is not None else "accepted_target"
                            ),
                            target_lead_rad=max(
                                abs(q - measured[n]) for n, q in zip(self.names, work_reference)
                            ),
                            candidate_budget_rad=list(budgets),
                            worst_candidate_joint=self.names[
                                max(range(len(leads)), key=leads.__getitem__)
                            ],
                            reserve_rad=0.8 * self.cfg.get("max_target_lead_error", limit),
                            hard_limit_rad=max(hard_limits),
                            joint_hard_limits_rad=dict(zip(self.names, hard_limits)),
                        )
                        tracking = work_reference if executed is None else executed
                        if any(abs(q-measured[n]) > bound for n,q,bound in zip(self.names, tracking, hard_limits)):
                            kind, value = "error", "BIMANUAL_REFERENCE_TRACKING_ERROR"
                        elif any(lead > budget + 1e-9 for lead, budget in zip(leads, budgets)):
                            kind, value = "error", "BIMANUAL_RESULT_FEEDBACK_CHANGED"
                    if kind == "error":
                        if awaiting == "solve" and value in (
                            "BIMANUAL_QP_DEFERRED",
                            "BIMANUAL_RESULT_FEEDBACK_CHANGED",
                        ):
                            # A deferred candidate is not device stop evidence.
                            # The previously accepted motion may still run
                            # under its original interface/expiry. Nothing
                            # below resubmits it or authorizes a hold.
                            if (
                                pending is not None
                                or s.state == "STOPPING"
                                or s.state == "FAILED"
                                and s.command_id.startswith(session.identity + "_")
                            ):
                                raise RuntimeError("BIMANUAL_SOLVE_DEFERRED_UNCONFIRMED")
                            if (
                                not 0 <= self.now() - work_sampled_at <= reference.max_interval
                                or not 0 <= now - work_sampled_monotonic <= reference.max_interval
                            ):
                                raise RuntimeError("BIMANUAL_RESULT_STALE")
                            if reference_deadline is None or now >= reference_deadline:
                                raise RuntimeError("BIMANUAL_SOLVE_DEFERRED_TIMEOUT")
                            if deferral is None:
                                deferral = dict(
                                    count=0,
                                    started_at=now,
                                    first_input_seq=work_seq,
                                    deadline_monotonic=reference_deadline,
                                    count_limit=math.ceil(
                                        (reference_deadline - now) / WORKER_POLL_PERIOD
                                    ),
                                    count_limit_basis="remaining_reference_budget / worker_poll_period",
                                )
                                self.event(
                                    "BIMANUAL_SOLVE_DEFERRED",
                                    json.dumps(
                                        {
                                            "code": value,
                                            "input_seq": work_seq,
                                            "input_stamp": work_stamp,
                                            "control_sampled_at": work_sampled_at,
                                            "integration_dt": work_dt,
                                            "sample_gap_s": work_gap,
                                            "grasp_mode": session.grasp_mode,
                                            "feedback": work_feedback,
                                            "joint_names": self.names,
                                            "reference": work_reference,
                                            "target": work_target,
                                            "execution_state": s.state,
                                            "execution_command_id": s.command_id,
                                            "stop_confirmed": False,
                                            "waiting_policy": "no_new_command_original_contract_expires",
                                            "deadline_monotonic": reference_deadline,
                                            "count_limit": deferral["count_limit"],
                                            "admission": admission,
                                        }
                                    ),
                                    goal.request.task_id,
                                )
                            deferral["count"] += 1
                            deferral["last_input_seq"] = work_seq
                            if deferral["count"] >= deferral["count_limit"]:
                                raise RuntimeError("BIMANUAL_SOLVE_DEFERRED_LIMIT")
                            awaiting = ""
                            # Keep the accepted reference/time unchanged.
                            # work_seq requires a fresh frame; source TTL,
                            # execution failure and sample-gap budgets all
                            # continue to run. Never resubmit an old hold.
                            await self.pause()
                            continue
                        self.event(
                            "BIMANUAL_WORK_REJECTED",
                            json.dumps(
                                {
                                    "code": value,
                                    "operation": awaiting,
                                    "input_seq": work_seq,
                                    "input_stamp": work_stamp,
                                    "constraint": geometry.get("constraint") if geometry else None,
                                    "timing": geometry.get("timing") if geometry else None,
                                    "control_sampled_at": work_sampled_at,
                                    "integration_dt": work_dt,
                                    "sample_gap_s": work_gap,
                                    "grasp_mode": session.grasp_mode,
                                    "feedback": work_feedback,
                                    "joint_names": self.names,
                                    "reference": work_reference,
                                    "execution_reference": work_execution_reference,
                                    "target": work_target,
                                    "admission": admission,
                                }
                            ),
                            goal.request.task_id,
                        )
                        raise RuntimeError(value)
                    if awaiting == "capture":
                        if not 0 <= self.now() - work_sampled_at <= reference.max_sample_gap:
                            raise RuntimeError("BIMANUAL_CAPTURE_STALE")
                        session.capture(value)
                        # No motion was submitted while capturing. Start the
                        # integration clock here; capture latency is not dt.
                        reference.capture(work_positions, self.now())
                        reference_deadline = time.monotonic() + reference.max_sample_gap
                        trajectory_bootstrap = True
                        trajectory_reference_deadline = None
                        trajectory_sample_at = None
                        self.event(
                            "BIMANUAL_CAPTURED",
                            json.dumps(
                                {
                                    "tool_pose": value.tolist(),
                                    "measured_grasp_policy": self.cfg.get(
                                        "measured_grasp_policy", "enforce"
                                    ),
                                    "feedback": work_feedback,
                                    "capture_reference": dict(zip(self.names, work_positions)),
                                    "capture_basis": (
                                        "confirmed_hold"
                                        if work_reference is not None
                                        else "measured"
                                    ),
                                    "input_stamp": session.stamp,
                                    "input_seq": session.seq,
                                    "input_source_id": session.source,
                                    "grasp_mode": session.grasp_mode,
                                    "controller_pose": session.anchor_input.tolist(),
                                    "raw_controller_pose": (
                                        session.raw_input.tolist()
                                        if self.input_source == "vr"
                                        else None
                                    ),
                                    "pose_filter": (
                                        self.cfg.get("pose_filter")
                                        if self.input_source == "vr"
                                        else None
                                    ),
                                    "pose_filter_state": (
                                        [
                                            (
                                                hand.pose_filter.snapshot()
                                                if hand.pose_filter
                                                else None
                                            )
                                            for hand in session.hands
                                        ]
                                        if isinstance(session, IndependentArmsSession)
                                        else (
                                            session.pose_filter.snapshot()
                                            if isinstance(session, BimanualSession)
                                            and session.pose_filter
                                            else None
                                        )
                                    ),
                                    "controller_anchors": (
                                        [
                                            value.tolist() if value is not None else None
                                            for value in session.anchors
                                        ]
                                        if isinstance(session, IndependentArmsSession)
                                        else None
                                    ),
                                    "robot_anchors": (
                                        [
                                            value.tolist() if value is not None else None
                                            for value in session.robot_anchors
                                        ]
                                        if isinstance(session, IndependentArmsSession)
                                        else None
                                    ),
                                    "control_point_in_tool_m": (
                                        session.control_point_in_tool.tolist()
                                        if isinstance(session, BimanualSession)
                                        else None
                                    ),
                                    "feedback_sampled_at": work_sampled_at,
                                    "capture_completed_at": reference.sampled_at,
                                    "input_after_capture": reference.input_after_capture,
                                    "grasp_generation": session.grasp_generation,
                                    "direction_reference": self.cfg.get(
                                        "direction_reference", "fixed"
                                    ),
                                    "latched_operator_yaw_rad": (
                                        [hand.mapping_yaw for hand in session.hands]
                                        if isinstance(session, IndependentArmsSession)
                                        else getattr(session, "mapping_yaw", None)
                                    ),
                                    "heading_input": (
                                        [hand.heading_input for hand in session.hands]
                                        if isinstance(session, IndependentArmsSession)
                                        else getattr(session, "heading_input", None)
                                    ),
                                    "grasp_continuation": getattr(session, "resumed_grasp", False),
                                }
                            ),
                            goal.request.task_id,
                        )
                    else:
                        # A newer VR frame does not invalidate a bounded,
                        # same-grasp solve. Clutch/mode changes clear awaiting.
                        # Discarded results never advance the accepted reference.
                        if (
                            work_stamp >= self.now() - self.cfg["command_max_age"]
                            and pending is None
                        ):
                            if not 0 <= self.now() - work_sampled_at <= reference.max_interval:
                                raise RuntimeError("BIMANUAL_RESULT_STALE")
                            positions, metrics = value
                            command = MotionCommand(
                                schema_version=2,
                                runtime_mode=self.runtime_mode,
                                command_id=session.identity + "_" + str(token),
                                task_id=goal.request.task_id,
                                observation_id=str(token),
                                profile_hash=self.profile.digest,
                                lease_id=lease.lease_id,
                                epoch=lease.epoch,
                                expected_revision=submit_revision,
                                mode="joint_target",
                                resource_group="",
                                resource_groups=list(self.groups),
                                stamp=stamp(work_stamp),
                                valid_for=self.cfg["command_max_age"],
                                joint_names=list(self.names),
                                positions=list(positions),
                                rigid_reference=(
                                    self.trajectory_reference
                                    and self.cfg.get("rigid_reference_prediction", False)
                                    and mode.mode == "bimanual_tool"
                                    and session.grasp_mode == "rigid"
                                ),
                            )
                            pending = self.submit_client.call_async(
                                SubmitMotion.Request(command=command)
                            )
                            pending_reference = (positions, work_sampled_at)
                            pending_at = now
                            if self.solution_report_due():
                                self.event(
                                    "BIMANUAL_QP_RESULT",
                                    json.dumps(
                                        {
                                            "solve_us": metrics[0],
                                            "timing": geometry.get("timing") if geometry else None,
                                            "geometry_step": (
                                                geometry
                                                if geometry and geometry["scale"] < 1.0
                                                else None
                                            ),
                                            "relative_error": (
                                                None
                                                if isinstance(session, IndependentArmsSession)
                                                else metrics[1]
                                            ),
                                            "relative_error_source": (
                                                None
                                                if isinstance(session, IndependentArmsSession)
                                                else (
                                                    "accepted_reference"
                                                    if self.cfg.get("reference_kinematics")
                                                    else "measured"
                                                )
                                            ),
                                            "tracking_feedback": self.cfg.get(
                                                "tracking_feedback", "measured"
                                            ),
                                            "geometry_step_scale": (
                                                geometry["scale"] if geometry else 1.0
                                            ),
                                            "admission": admission,
                                            "teleop_mode": mode.mode,
                                            "recovery_scale": metrics[2],
                                            "input_stamp": work_stamp,
                                            "input_seq": work_seq,
                                            "integration_dt": work_dt,
                                            "sample_gap_s": work_gap,
                                            "control_sampled_at": work_sampled_at,
                                            "result_received_at": self.now(),
                                            "result_age_s": self.now() - work_sampled_at,
                                            "joint_names": self.names,
                                            "positions": positions,
                                            "reference": work_reference,
                                            "feedback": work_feedback,
                                            "execution_reference": work_execution_reference,
                                            "target": work_target,
                                        }
                                    ),
                                    goal.request.task_id,
                                    command.command_id,
                                )
                    awaiting = ""
            if (
                session.mode == "follow"
                and session.anchor_input is not None
                and not self.worker.busy
                and not awaiting
                and pending is None
                and (session.seq != work_seq or session.anchor_tool is None)
            ):
                measured = dict(zip(s.joints.name, s.joints.position))
                worker_token += 1
                operation = "capture" if session.anchor_tool is None else "solve"
                sampled_at = self.now()
                sampled_monotonic = time.monotonic()
                # A capture and this loop can share a clock tick. The next
                # fresh controller frame will provide a positive interval.
                if operation == "solve" and sampled_at <= reference.sampled_at:
                    await self.pause()
                    continue
                if operation == "solve" and not reference.permits_input(session.stamp):
                    # Stop raises the executor's timestamp fence. A frame
                    # already queued during FK capture must not cross it.
                    # Wait for real input; never relabel its timestamp.
                    await self.pause()
                    continue
                gap = sampled_at - reference.sampled_at if operation == "solve" else None
                if gap is not None and gap > 0.08:
                    self.event(
                        "BIMANUAL_SAMPLE_GAP",
                        json.dumps(
                            {
                                "sample_gap_s": gap,
                                "max_sample_gap_s": reference.max_sample_gap,
                                "max_integration_interval_s": reference.max_interval,
                                "reference_sampled_at": reference.sampled_at,
                                "control_sampled_at": sampled_at,
                                "server_monotonic_s": time.monotonic(),
                                "input_seq": session.seq,
                                "input_stamp": session.stamp,
                                "input_age_s": sampled_at - session.stamp,
                                "feedback_stamp": seconds(s.feedback_stamp),
                                "feedback_age_s": sampled_at - seconds(s.feedback_stamp),
                                "will_reject": gap > reference.max_sample_gap,
                            }
                        ),
                        goal.request.task_id,
                    )
                execution_sample = None
                if operation == "solve" and self.trajectory_reference and not trajectory_bootstrap:
                    from bindu_runtime.stop_evidence import sampled_nominal_reference

                    if trajectory_reference_deadline is None:
                        trajectory_reference_deadline = time.monotonic() + reference.max_sample_gap
                    sample = sampled_nominal_reference(
                        s,
                        self.names,
                        lease.epoch,
                        sampled_at,
                        min(0.1, self.cfg["feedback_max_age"]),
                        wait_until=trajectory_reference_deadline,
                        steady_now=time.monotonic(),
                    )
                    if sample is None:
                        await self.pause()
                        continue
                    actual, actual_stamp = sample
                    if (
                        trajectory_sample_at is not None
                        and actual_stamp <= trajectory_sample_at
                        or actual_stamp >= sampled_at
                    ):
                        if time.monotonic() >= trajectory_reference_deadline:
                            raise RuntimeError("SNOWBOT_TRAJECTORY_REFERENCE_NOT_ADVANCING")
                        await self.pause()
                        continue
                    # Keep integrating from accepted goals. Replacing them with
                    # the lagging servo sample each cycle erases planned progress
                    # and makes each tiny target a new rest-to-rest move.
                    # Still require fresh advancing, lease-bound servo feedback;
                    # QP measured tracking/geometry and device limits stay active.
                    trajectory_sample_at = actual_stamp
                    trajectory_reference_deadline = None
                    execution_sample = dict(positions=actual, stamp=actual_stamp, epoch=lease.epoch)
                baseline, dt = reference.step(sampled_at) if operation == "solve" else (None, None)
                if operation == "capture" and self.trajectory_reference:
                    baseline = await self.held_reference(lease, submit_revision)
                    # The request yields: use current measured feedback and
                    # retain all normal input/feedback checks before capture.
                    self.check()
                    measured = dict(zip(self.state.joints.name, self.state.joints.position))
                    sampled_at = self.now()
                    sampled_monotonic = time.monotonic()
                if self.worker.submit(
                    operation,
                    worker_token,
                    measured,
                    session.target,
                    reference=baseline,
                    dt=dt,
                    grasp_mode=session.grasp_mode,
                    target_stamp=session.stamp,
                    execution_reference=(
                        execution_sample["positions"] if execution_sample is not None else None
                    ),
                ):
                    awaiting = operation
                    work_seq, work_stamp = session.seq, session.stamp
                    work_sampled_at, work_dt = sampled_at, dt
                    work_sampled_monotonic = sampled_monotonic
                    work_gap = gap
                    work_positions = (
                        baseline
                        if operation == "capture" and baseline is not None
                        else tuple(measured[n] for n in self.names)
                    )
                    work_feedback, work_reference = measured, baseline
                    work_execution_reference = execution_sample
                    work_target = (
                        None if session.target is None else session.target.ravel().tolist()
                    )
            feedback_mode = (
                "capturing"
                if session.mode == "follow" and session.anchor_tool is None
                else session.mode
            ) + (
                ":" + session.grasp_mode
                if session.axial or isinstance(session, IndependentArmsSession)
                else ""
            )
            if feedback_mode != last_feedback_mode or now >= next_feedback_at:
                goal.publish_feedback(
                    TeleopAction.Feedback(
                        session_id=session.identity, mode=feedback_mode, accepted_commands=accepted
                    )
                )
                last_feedback_mode = feedback_mode
                next_feedback_at = now + 0.1
            await self.pause()
        result.code = "CANCELED" if canceled else "TELEOP_SESSION_COMPLETE"
    except Exception as exc:
        result.code = str(exc)
        if result.code == "BIMANUAL_MODE_NOT_SELECTED":
            mode = self.mode_state
            self.cleanup_event(
                "BIMANUAL_MODE_DIAGNOSTIC",
                json.dumps(
                    {
                        "mode": mode.mode if mode else "",
                        "phase": mode.phase if mode else "",
                        "generation": mode.generation if mode else None,
                        "age": self.now() - seconds(mode.stamp) if mode else None,
                    }
                ),
                goal.request.task_id,
            )
    finally:
        try:
            try:
                end_deferral("session_ended")
            except Exception as exc:
                if getattr(result, "code", "") in ("", "TELEOP_SESSION_COMPLETE", "CANCELED"):
                    result.code = "BIMANUAL_DIAGNOSTIC_FAILED:" + str(exc)
                self.cleanup_event("BIMANUAL_DIAGNOSTIC_FAILED", str(exc), goal.request.task_id)
        finally:
            try:
                if acquisition is not None and lease is None:
                    # Settle the exact unknown request without inferring its
                    # outcome from an idle/replacement executor snapshot.
                    stopped = False
                    try:
                        lease = await self.wait(acquisition)
                        stopped = not lease.ok
                        if stopped:
                            code = record_acquisition_rejection(lease)
                            if result.lease_rejection is not None:
                                result.code = code
                    except Exception as exc:
                        self.cleanup_event(
                            "BIMANUAL_LEASE_UNCONFIRMED",
                            session.identity + ":" + str(exc),
                            goal.request.task_id,
                        )
                if lease is not None and lease.ok:
                    stopped = await self.stop(lease, release=True)
                if pending is not None:
                    try:
                        if (await self.wait(pending)).accepted:
                            accepted += 1
                    except Exception as exc:
                        self.cleanup_event(
                            "BIMANUAL_SUBMIT_UNCONFIRMED",
                            session.identity + ":" + str(exc),
                            goal.request.task_id,
                        )
            except Exception as exc:
                stopped = False
                self.cleanup_event("BIMANUAL_CLEANUP_FAILED", str(exc), goal.request.task_id)
            finally:
                stopped = stopped and not getattr(self, "stop_unconfirmed", False)
                if not stopped:
                    self.stop_unconfirmed = True
                    result.code += ":STOP_UNCONFIRMED"
                self.busy = False
                result.accepted_commands = accepted
                result.success = result.code == "TELEOP_SESSION_COMPLETE"
            if (
                canceled
                and stopped
                and result.code == "CANCELED"
                and isinstance(session, BimanualSession)
            ):
                self.grasp_continuation = session.stopped_continuation(self.now())
            self.cleanup_event(
                "BIMANUAL_ENDED",
                json.dumps(
                    {"session_id": session.identity, "code": result.code, "accepted": accepted}
                ),
                goal.request.task_id,
            )
    if result.success:
        goal.succeed()
    elif canceled and stopped:
        goal.canceled()
    else:
        goal.abort()
    return result
