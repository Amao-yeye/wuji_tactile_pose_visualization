"""Load the verified local Wuji SDK without replacing Kilted's Python runtime."""

from __future__ import annotations

import importlib
import sys
from pathlib import Path


DEFAULT_SDK_SITE_PACKAGES = "/home/oem/Workspaces/wuji/.venv/lib/python3.12/site-packages"


def import_wuji_sdk(site_packages: str):
    """Import ``wuji_sdk`` from the configured venv after ROS is initialized.

    The ROS node intentionally runs with Ubuntu/Kilted's Python so that rclpy
    keeps its apt-provided dependencies.  The official SDK is loaded from the
    already validated local virtual environment instead of activating that
    venv, which would otherwise hide Kilted's system Python packages.
    """

    candidate = Path(site_packages).expanduser()
    if not candidate.is_dir():
        raise RuntimeError(
            f"Wuji SDK site-packages directory does not exist: {candidate}. "
            "Set the sdk_site_packages ROS parameter to the venv that contains wuji-sdk."
        )
    candidate_text = str(candidate)
    if candidate_text not in sys.path:
        # Append instead of prepend: preserve Kilted's system modules such as
        # PyYAML and Matplotlib's matching NumPy ABI.
        sys.path.append(candidate_text)
    try:
        return importlib.import_module("wuji_sdk")
    except ModuleNotFoundError as exc:
        raise RuntimeError(f"Could not import wuji_sdk from {candidate}") from exc
