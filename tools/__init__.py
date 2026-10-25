"""Repository tools; importing this package never starts a runtime."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def package_share(name):
    """Use the shared resolver before this checkout has been ROS-installed."""
    try:
        from bindu_contracts.resources import package_share as resolve
    except ModuleNotFoundError as exc:
        if exc.name != 'bindu_contracts':
            raise
        import importlib.util
        source = ROOT/'src/shared/bindu_contracts/bindu_contracts/resources.py'
        spec = importlib.util.spec_from_file_location('_bindu_source_resources', source)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        resolve = module.package_share
    return resolve(name)
