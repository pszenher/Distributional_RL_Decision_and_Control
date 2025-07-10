from pydantic import PrivateAttr

from gzilla.codegen import gz_sensor_map
from gzilla.mappings.specs import GzSensorSpec


class GzSensorMap(gz_sensor_map.GzSensorMap):
    sensors: list[GzSensorSpec]

    _sensor_dict: dict[str, GzSensorSpec] = PrivateAttr(default_factory=dict)
    _alias_dict: dict[str, GzSensorSpec] = PrivateAttr(default_factory=dict)

    def __init__(self, **data):
        super().__init__(**data)
        self._sensor_dict = self._generate_sensor_dict(self.sensors)
        self._alias_dict = self._generate_alias_dict(self.sensors)
        self._check_sensors_aliases_disjoint()

    @staticmethod
    def _generate_sensor_dict(sensors: list[GzSensorSpec]) -> dict[str, GzSensorSpec]:
        sensor_dict = {}
        dups = {
            s.sensor
            for s in sensors
            if s.sensor in sensor_dict or sensor_dict.update({s.sensor: s})
        }
        if dups:
            msg = f"Duplicate sensor names found in sensor map: '{dups}'"
            raise ValueError(msg)
        return sensor_dict

    @staticmethod
    def _generate_alias_dict(sensors: list[GzSensorSpec]) -> dict[str, GzSensorSpec]:
        alias_dict = {}
        dups = {
            alias
            for s in sensors
            if s.aliases
            for alias in s.aliases
            if alias in alias_dict or alias_dict.update({alias: s})
        }
        if dups:
            msg = f"Duplicate alias names found in sensor map: '{dups}'"
            raise ValueError(msg)
        return alias_dict

    def _check_sensors_aliases_disjoint(self) -> None:
        if (dups := self._sensor_dict.keys() & self._alias_dict.keys()):
            msg = f"Sensor type cannot have both definition and alias: '{dups}'"
            raise ValueError(msg)

    def lookup_sensor(self, gz_sensor: str) -> GzSensorSpec | None:
        return self._sensor_dict.get(gz_sensor)

    def lookup_alias(self, gz_sensor: str) -> GzSensorSpec | None:
        return self._alias_dict.get(gz_sensor)

    def lookup(self, name: str) -> GzSensorSpec | None:
        return self.lookup_sensor(name) or self.lookup_alias(name)
