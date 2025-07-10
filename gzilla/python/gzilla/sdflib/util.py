"""Utility functions."""

import os
from pathlib import Path


def find_gz_resource(path_string: str) -> str:
    """Lookup Gazebo resource by name/path string.

    Returns absolute path to resource as string if found.
    Raises `FileNotFoundError` if resource name does not resolve in search paths.
    """
    model_paths = [
        Path(res_path) / path_string
        for res_path in os.environ.get("GZ_SIM_RESOURCE_PATH", "").split(os.pathsep)
    ]

    # Iterate over potential model paths, return first path that resolves
    for model_path in model_paths:
        if model_path.exists():
            return model_path.as_posix()

    msg = f"Failed to locate gz resource '{path_string}' in GZ_SIM_RESOURCE_PATH paths"
    raise FileNotFoundError(msg)
