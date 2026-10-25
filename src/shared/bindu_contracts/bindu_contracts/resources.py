"""Resolve installed package resources or explicitly selected source packages.

Development tools may set BINDU_PACKAGE_PATH to package roots or directories
containing packages. No sibling checkout or historical artifact is inferred.
"""
import os
import re
from pathlib import Path
import xml.etree.ElementTree as ET


def package_share(name, packages=None):
    if not isinstance(name, str) or not re.fullmatch(r'[a-z][a-z0-9_]*', name):
        raise ValueError('RESOURCE_PACKAGE_NAME')
    if packages is not None and name in packages:
        return Path(packages[name]).resolve()
    matches = set()
    for entry in os.environ.get('BINDU_PACKAGE_PATH', '').split(os.pathsep):
        if not entry:
            continue
        root = Path(entry).expanduser().resolve()
        candidates = [root] if (root/'package.xml').is_file() else [root/name]
        for candidate in candidates:
            manifest = candidate/'package.xml'
            if manifest.is_file() and ET.parse(manifest).findtext('name') == name:
                matches.add(candidate)
    if len(matches) > 1:
        raise ValueError('PACKAGE_SOURCE_AMBIGUOUS:'+name)
    if matches:
        return matches.pop()
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory(name)).resolve()
    except (ImportError, LookupError) as exc:
        raise ValueError('PACKAGE_NOT_FOUND:'+name+
                         '; install the package or select its source with BINDU_PACKAGE_PATH') from exc


def resource_file(value, owner=None, packages=None):
    if not isinstance(value, str) or not value:
        raise ValueError('RESOURCE_PATH')
    if value.startswith('package://'):
        name, sep, relative = value[10:].partition('/')
        if (not name or not sep or not relative or Path(relative).is_absolute()
                or '..' in Path(relative).parts):
            raise ValueError('RESOURCE_PACKAGE_PATH')
        base = package_share(name, packages)
        # colcon --symlink-install deliberately points installed resources back
        # at their source files. Reject lexical traversal, while allowing those
        # exports from the explicitly selected package.
        result = (base/relative).resolve()
    else:
        result = Path(value)
        if not result.is_absolute():
            if owner is None:
                raise ValueError('RESOURCE_OWNER_REQUIRED')
            result = Path(owner).parent/result
        result = result.resolve()
    if not result.is_file():
        raise ValueError('RESOURCE_MISSING:'+str(result))
    return result
