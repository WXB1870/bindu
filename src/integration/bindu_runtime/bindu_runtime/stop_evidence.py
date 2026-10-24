"""One feedback predicate for coordinator and tool execution stop barriers."""


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
