from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from gzilla.mappings.sensor_mapping import GzSensorMap
from gzilla.mappings.plugin_mapping import GzPluginMap

def sensors() -> GzSensorMap:
    gzilla_share_dir = Path(
        get_package_share_directory("gzilla")
    )
    sensor_map_yaml = gzilla_share_dir / 'config' / 'gz_sensor_mappings.yaml'

    with sensor_map_yaml.open() as f:
        return GzSensorMap.parse_obj(
            yaml.safe_load(f)
        )

    
def plugins() -> GzSensorMap:
    gzilla_share_dir = Path(
        get_package_share_directory("gzilla")
    )
    plugin_map_yaml = gzilla_share_dir / 'config' / 'gz_plugin_mappings.yaml'

    with plugin_map_yaml.open() as f:
        return GzPluginMap.parse_obj(
            yaml.safe_load(f)
        )


def vrx_plugins() -> GzSensorMap:
    gzilla_share_dir = Path(
        get_package_share_directory("gzilla")
    )
    plugin_map_yaml = gzilla_share_dir / 'config' / 'gz_vrx_plugin_mappings.yaml'

    with plugin_map_yaml.open() as f:
        return GzPluginMap.parse_obj(
            yaml.safe_load(f)
        )
