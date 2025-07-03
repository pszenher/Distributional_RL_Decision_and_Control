from typing import Dict, List, Optional, Self, Tuple

from gzilla.mappings.specs import GzTopicSpec, GzSensorSpec, GzSensorRuntimeConfig

from pydantic import model_validator

import gzilla.codegen.gz_sensor_map_schema  as gz_sensor_map_schema

class GzSensorMap(gz_sensor_map_schema.GzSensorMap):

    sensors: List[GzSensorSpec]

    _sensor_dict: Dict[str,GzSensorSpec]
    _alias_dict: Dict[str,GzSensorSpec]

    @model_validator(mode='after')
    def check_sensors_unique(self) -> Self:
        sensor_dict = dict()
        dups = {
            s.sensor for s in self.sensors
            if s.sensor in sensor_dict
            or sensor_dict.update({s.sensor: s})
        }
        self._sensor_dict = sensor_dict
        if dups:
            raise ValueError(
                f"Duplicate sensor names found in sensor map: '{dups}'"
            )
        return self

    @model_validator(mode='after')
    def check_aliases_unique(self) -> Self:
        alias_dict = dict()
        dups = {
            alias
            for s in self.sensors
            for alias in s.aliases
            if alias in alias_dict
            or alias_dict.update({alias: s})
        }
        self._alias_dict = alias_dict
        if dups:
            raise ValueError(
                f"Duplicate alias names found in sensor map: '{dups}'"
            )
        return self

    @model_validator(mode='after')
    def check_sensors_aliases_disjoint(self) -> Self:
        if self._sensor_dict.keys() & self._alias_dict.keys():
            raise ValueError(
                f"Sensor type cannot have both definition and alias: '{dups}'"
            )
        return self

    def lookup_sensor(self, gz_sensor: str) -> Optional[GzSensorSpec]:
        return self._sensor_dict.get(gz_sensor)

    def lookup_alias(self, gz_sensor: str) -> Optional[GzSensorSpec]:
        return self._alias_dict.get(gz_sensor)
            
    def lookup(self, name: str) -> Optional[GzSensorSpec]:
        return self.lookup_sensor(name) \
            or self.lookup_alias(name)
