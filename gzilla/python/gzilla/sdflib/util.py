import os
from pathlib import Path, PurePath

def find_gz_resource(path_string: str) -> str:
    model_paths = [
        Path(res_path) / path_string
        for res_path in os.environ.get("GZ_SIM_RESOURCE_PATH").split( os.pathsep )
    ]

    # Iterate over potential model paths, return first path that resolves
    for model_path in model_paths:
        if model_path.exists():
            return model_path.as_posix()
