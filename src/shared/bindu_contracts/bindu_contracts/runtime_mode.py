"""Environment identity shared by deployment, execution and observation ports."""
SIMULATION = 'simulation'
REAL = 'real'
MODES = frozenset((SIMULATION, REAL))


def resolve_mode(mode='', simulation=None, *, default=SIMULATION):
    if not isinstance(mode, str) or (mode and mode not in MODES) or default not in MODES:
        raise ValueError('RUNTIME_MODE_INVALID')
    if simulation is not None and type(simulation) is not bool:
        raise ValueError('RUNTIME_SIMULATION_ALIAS_INVALID')
    legacy = None if simulation is None else (SIMULATION if simulation else REAL)
    if mode and legacy is not None and mode != legacy:
        raise ValueError('RUNTIME_MODE_CONFLICT')
    return mode or legacy or default


def command_mode(mode, expected):
    """Old untagged commands remain simulation-only; real commands are explicit."""
    if expected not in MODES or not isinstance(mode, str):
        raise ValueError('RUNTIME_MODE_INVALID')
    if not mode:
        if expected == REAL:
            raise ValueError('RUNTIME_MODE_REQUIRED_FOR_REAL_COMMAND')
        return SIMULATION
    if mode not in MODES or mode != expected:
        raise ValueError('RUNTIME_MODE_MISMATCH')
    return mode


def message_matches(message, expected):
    mode = getattr(message, 'runtime_mode', '')
    if expected not in MODES or mode not in ('', *MODES):
        return False
    if mode != expected and not (mode == '' and expected == SIMULATION):
        return False
    simulated = getattr(message, 'simulated', None)
    return simulated is None or (type(simulated) is bool and simulated == (expected == SIMULATION))
