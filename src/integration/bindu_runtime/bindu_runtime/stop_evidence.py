"""One feedback predicate for coordinator and tool execution stop barriers."""
import math


def stop_feedback_matches(state, *, instance_id, epoch, lease_active, profile_hash,
                          simulated, requested, now, max_age, at_rest):
    if state is None or not instance_id:
        return False
    stamp=state.feedback_stamp
    sampled=stamp if isinstance(stamp,(float,int)) else stamp.sec+stamp.nanosec*1e-9
    return (state.instance_id==instance_id and state.lease_epoch==epoch and
            state.lease_active==lease_active and state.profile_hash==profile_hash and
            state.simulated==simulated and state.state not in ('RUNNING','STOPPING') and
            not state.stop_failures and 'EMERGENCY_STOP' not in state.code and
            state.code!='STOP_FEEDBACK_TIMEOUT' and sampled>=requested and
            0<=now-sampled<max_age and not state.reference.name and at_rest(state.joints))


def sampled_nominal_reference(state, names, epoch, now, max_age, *, wait_until=None, steady_now=None):
    """Read the servo sample published by execution; never substitute q_goal."""
    reference = state.reference
    sampled_at = reference.header.stamp.sec+reference.header.stamp.nanosec*1e-9
    if not state.lease_active or state.lease_epoch != epoch:
        raise RuntimeError('SNOWBOT_TRAJECTORY_REFERENCE_OWNER')
    if (not math.isfinite(sampled_at) or not 0 <= now-sampled_at <= max_age or
            len(reference.name) != len(names) or len(reference.position) != len(names) or
            set(reference.name) != set(names) or not all(math.isfinite(q) for q in reference.position)):
        if wait_until is not None and steady_now is not None and steady_now < wait_until:
            return None
        raise RuntimeError('SNOWBOT_TRAJECTORY_REFERENCE_UNAVAILABLE')
    positions = dict(zip(reference.name, reference.position))
    return tuple(positions[n] for n in names), sampled_at
