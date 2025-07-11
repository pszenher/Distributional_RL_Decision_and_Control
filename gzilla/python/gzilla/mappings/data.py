"""Data sources for Gazebo sensor and plugin topic mappings."""
from pathlib import Path

import yaml  # type: ignore [import-untyped]
from ament_index_python.packages import (  # type: ignore [import-untyped]
    get_package_share_directory,
)

from gzilla.mappings.plugin_mapping import GzPluginMap
from gzilla.mappings.sensor_mapping import GzSensorMap


def sensors() -> GzSensorMap:
    gzilla_share_dir = Path(get_package_share_directory("gzilla"))
    sensor_map_yaml = gzilla_share_dir / "config" / "gz_sensor_mappings.yaml"

    with sensor_map_yaml.open() as f:
        return GzSensorMap.parse_obj(yaml.safe_load(f))


def plugins() -> GzPluginMap:
    gzilla_share_dir = Path(get_package_share_directory("gzilla"))
    plugin_map_yaml = gzilla_share_dir / "config" / "gz_plugin_mappings.yaml"

    with plugin_map_yaml.open() as f:
        return GzPluginMap.parse_obj(yaml.safe_load(f))


def vrx_plugins() -> GzPluginMap:
    gzilla_share_dir = Path(get_package_share_directory("gzilla"))
    plugin_map_yaml = gzilla_share_dir / "config" / "gz_vrx_plugin_mappings.yaml"

    with plugin_map_yaml.open() as f:
        return GzPluginMap.parse_obj(yaml.safe_load(f))
