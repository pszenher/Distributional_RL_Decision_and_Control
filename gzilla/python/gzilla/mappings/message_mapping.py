"""Mapping data source for Gazebo/ROS message translation."""

from collections import defaultdict
from collections.abc import Mapping, Sequence

import ros_gz_bridge  # type: ignore [import-untyped]

# FIXME(pszenher): why can't mypy see this?
from pydantic import BaseModel, Field, PrivateAttr  # type: ignore [import-not-found]


class GzRosMessageMap(BaseModel):
    """Handler class for `ros_gz_bridge` default `MessageMapping` objects."""

    mapping: Sequence[ros_gz_bridge.MessageMapping] = Field(
        ros_gz_bridge.mappings(None),
        description=(
            "Sequence of objects storing known equivalences between "
            "GZ and ROS2 message types"
        ),
    )

    _gz_type_dict: Mapping[str, list[ros_gz_bridge.MessageMapping]] = PrivateAttr()
    _ros_type_dict: Mapping[tuple[str, str], ros_gz_bridge.MessageMapping] = (
        PrivateAttr()
    )

    def __init__(self, **data) -> None:
        """Initialize GzRosMessageMap object.

        Calls into Pydantic BaseModel constructor via `super()`, then
        creates private lookup tables mapping ROS and GZ messages to
        their respective mappings.
        """
        super().__init__(**data)
        self._gz_type_dict = self._generate_gz_type_dict(self.mapping)
        self._ros_type_dict = self._generate_ros_type_dict(self.mapping)

    @staticmethod
    def _generate_gz_type_dict(
        mapping: Sequence[ros_gz_bridge.MessageMapping],
    ) -> Mapping[str, ros_gz_bridge.MessageMapping]:
        gz_dict = defaultdict(list)
        for msgmap in mapping:
            gz_dict[msgmap.gz_message_name].append(msgmap)
        return gz_dict

    @staticmethod
    def _generate_ros_type_dict(
        mapping: Sequence[ros_gz_bridge.MessageMapping],
    ) -> Mapping[tuple[str, str], ros_gz_bridge.MessageMapping]:
        ros_dict = {}
        dups = {
            (msgmap.gz_message_name, msgmap.ros2_package_name, msgmap.ros2_message_name)
            for msgmap in mapping
            if (msgmap.gz_message_name in ros_dict)
            or ros_dict.update(
                {(msgmap.ros2_package_name, msgmap.ros2_message_name): msgmap}
            )
        }
        if dups:
            msg = f"Duplicate ROS message type found in type map:  '{dups}'"
            raise ValueError(msg)
        return ros_dict

    def query_gz(self, gz_type: str) -> ros_gz_bridge.MessageMapping | None:
        """Return the `MessageMapping` corresponding to Gazebo message type name."""
        match self._gz_type_dict.get(gz_type):
            case None:
                msg = f"Unknown GZ message type passed for conversion:  '{gz_type}'"
                raise ValueError(msg)
            case [ros_type, *rest]:
                if rest:
                    print(
                        f"WARN:  multiple ROS types map to GZ type '{gz_type}', "
                        f"using first ('{ros_type.ros2_string()}')"
                    )
                return ros_type
            case val:
                msg = f"Unexpected lookup output from gz_type_dict:  {val}"
                raise RuntimeError(msg)

    def query_ros(
        self, ros_pkg: str, ros_type: str
    ) -> ros_gz_bridge.MessageMapping | None:
        """Return the `MessageMapping` corresponding to ROS message type parameters."""
        return self._ros_type_dict.get((ros_pkg, ros_type))

    def query_gz_string(self, gz_string: str) -> ros_gz_bridge.MessageMapping | None:
        """Return the `MessageMapping` corresponding to Gazebo message type string."""
        match gz_string.split("."):
            case ["gz", "msgs", gz_type]:
                return self.query_gz(gz_type)
            case _:
                msg = (
                    f"Invalid gz type string '{gz_string}', expected string prefixed "
                    "with 'gz.msgs.'"
                )
                raise ValueError(msg)

    def query_ros_string(self, ros_string: str) -> ros_gz_bridge.MessageMapping | None:
        """Return the `MessageMapping` corresponding to ROS message type string."""
        match ros_string.split("/"):
            case [ros_pkg, "msg", ros_type]:
                return self.query_ros(ros_pkg, ros_type)
            case _:
                msg = (
                    f"Invalid ros type string '{ros_string}', expected string with "
                    "format '{ros_pkg}/msg/{ros_type}'"
                )
                raise ValueError(msg)
