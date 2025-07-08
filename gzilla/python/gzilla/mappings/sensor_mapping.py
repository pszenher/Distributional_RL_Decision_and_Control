from typing import Dict, List, Optional, Self, Tuple

from pydantic import PrivateAttr

from gzilla.mappings.specs import GzTopicSpec, GzSensorSpec

import gzilla.codegen.gz_sensor_map as gz_sensor_map

class GzSensorMap(gz_sensor_map.GzSensorMap):

    sensors: list[GzSensorSpec]

    _sensor_dict: dict[str,GzSensorSpec] = PrivateAttr(default_factory=dict)
    _alias_dict:  dict[str,GzSensorSpec] = PrivateAttr(default_factory=dict)

    def __init__(self, **data):
        super().__init__(**data)
        self._sensor_dict = self._generate_sensor_dict(self.sensors)
        self._alias_dict =  self._generate_alias_dict(self.sensors)
        self._check_sensors_aliases_disjoint()
    
    @staticmethod
    def _generate_sensor_dict(sensors: list[GzSensorSpec]) -> dict[str,GzSensorSpec]:
        sensor_dict = dict()
        dups = {
            s.sensor for s in sensors
            if s.sensor in sensor_dict
            or sensor_dict.update({s.sensor: s})
        }
        if dups:
            raise ValueError(
                f"Duplicate sensor names found in sensor map: '{dups}'"
            )
        return sensor_dict

    @staticmethod
    def _generate_alias_dict(sensors: list[GzSensorSpec]) -> dict[str,GzSensorSpec]:
        alias_dict = dict()
        dups = {
            alias
            for s in sensors
            for alias in s.aliases
            if alias in alias_dict
            or alias_dict.update({alias: s})
        }
        if dups:
            raise ValueError(
                f"Duplicate alias names found in sensor map: '{dups}'"
            )
        return alias_dict

    def _check_sensors_aliases_disjoint(self) -> None:
        if self._sensor_dict.keys() & self._alias_dict.keys():
            raise ValueError(
                f"Sensor type cannot have both definition and alias: '{dups}'"
            )

    def lookup_sensor(self, gz_sensor: str) -> Optional[GzSensorSpec]:
        return self._sensor_dict.get(gz_sensor)

    def lookup_alias(self, gz_sensor: str) -> Optional[GzSensorSpec]:
        return self._alias_dict.get(gz_sensor)
            
    def lookup(self, name: str) -> Optional[GzSensorSpec]:
        return self.lookup_sensor(name) \
            or self.lookup_alias(name)
