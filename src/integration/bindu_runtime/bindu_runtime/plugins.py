"""Resolve explicitly configured, trusted Python providers at startup."""
from importlib import import_module


def load_provider(path, *args):
    module, name = path.split(":", 1)
    return getattr(import_module(module), name)(*args)
