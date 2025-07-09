"""Spawner of gz entities."""

from dataclasses import dataclass
from enum import Enum
from typing import Self

from collections.abc import Mapping, Sequence

import gz.math7 as gz_math  # type: ignore
import gz.transport13 as gz_transport  # type: ignore
import gz.msgs10 as gz_msgs  # type: ignore

import rclpy

import std_msgs.msg  # type: ignore
from rclpy.parameter import AllowableParameterValue

AllowableParameterValueWeak = (
    AllowableParameterValue
    | Sequence[bool]
    | Sequence[int]
    | Sequence[float]
    | Sequence[str]
)


class SdfSourceType(Enum):
    FILENAME = "file"
    XML_STRING = "string"
    ROS_TOPIC = "topic"

    @classmethod
    def ros_param_keys(cls) -> list[str]:
        return SdfSourceType._member_names_

    @classmethod
    def ros_param_spec(cls) -> list[tuple[str, str]]:
        return [(param, "") for param in cls.ros_param_keys()]

    @classmethod
    def prefixed_ros_param_spec(
        cls, prefix: str = "sdf"
    ) -> list[tuple[str, str, None]]:
        return [
            (f"{prefix}.{param}", default, None)
            for (param, default) in cls.ros_param_spec()
        ]

    @classmethod
    def from_ros_param(cls, param_name: str) -> Self:
        if param_name not in cls.ros_param_keys():
            raise ValueError(
                f"Invalid SDF source type param passed: '{param_name}';  "
                f"expected one of '{cls.ros_param_keys()}'"
            )
        return cls(param_name)


@dataclass(frozen=True)
class SdfDataSource:
    source_type: SdfSourceType
    data: str

    def __str__(self) -> str:
        return str(self.data)

    def is_resolved(self) -> bool:
        match self.source_type:
            case SdfSourceType.FILENAME | SdfSourceType.XML_STRING:
                return True
            case SdfSourceType.ROS_TOPIC:
                return False
            case _:
                raise ValueError(
                    f"Expected SdfSourceType-valued source type, received: "
                    f"{type(self.source_type)}"
                )

    def topic(self) -> str:
        if not self.source_type == SdfSourceType.ROS_TOPIC:
            raise ValueError(
                f"Attempted to read ROS topic from {self.source_type}-valued "
                f"SdfDataSource"
            )
        return self.data

    @classmethod
    def from_param_dict(
        cls, param_dict: Mapping[str, AllowableParameterValueWeak]
    ) -> Self:
        resolved: list[Self] = []
        for param, value in param_dict.items():
            if param not in SdfSourceType:
                raise ValueError(
                    f"Unexpected SDF data source parameter name encountered: {param}"
                )
            if value:
                resolved.append(
                    cls(
                        source_type=SdfSourceType(param),
                        data=str(value),
                    )
                )

        try:
            [data_source] = resolved
        except ValueError as err:
            raise ValueError(
                f"Expected a single value to be set as SDF data source, "
                f"received {len(resolved)}: ({resolved})"
            ) from err

        return data_source


class GzPose(gz_math.Pose3):
    @classmethod
    def ros_param_keys(cls) -> list[str]:
        return ["x", "y", "z", "roll", "pitch", "yaw"]

    @classmethod
    def ros_param_spec(cls) -> list[tuple[str, float]]:
        return [(param, 0.0) for param in cls.ros_param_keys()]

    @classmethod
    def prefixed_ros_param_spec(
        cls, prefix: str = "pose"
    ) -> list[tuple[str, float, None]]:
        return [
            (f"{prefix}.{param_name}", default, None)
            for (param_name, default) in cls.ros_param_spec()
        ]

    @classmethod
    def from_param_dict(
        cls, param_dict: Mapping[str, AllowableParameterValueWeak]
    ) -> Self:
        match param_dict:
            case {
                "x": x,
                "y": y,
                "z": z,
                "roll": roll,
                "pitch": pitch,
                "yaw": yaw,
            }:
                return cls(x, y, z, roll, pitch, yaw)
            case _:
                raise ValueError(
                    "Unexpected parameter dictionary received for gazebo pose data:  "
                    f"'{param_dict}'"
                )


class GzSpawnerNode(rclpy.Node):
    world_name: rclpy.Parameter
    entity_name: rclpy.Parameter
    sdf_data: SdfDataSource
    pose: GzPose

    gz_node: gz_transport.Node

    def __init__(self, node_name: str, **kwargs):
        super().__init__(node_name, **kwargs)

        self.declare_parameters(
            namespace="",
            parameters=[
                ("world_name", "", None),
                ("entity_name", "", None),
                ("allow_renaming", False, None),
                *SdfSourceType.prefixed_ros_param_spec(),
                *GzPose.prefixed_ros_param_spec(),
                # TODO: consider adding params for remaining spawner args?
                #   `relative_to`: string   -> frame relative to which pose is
                #                              interpreted
                #   `spherical_coordinates` -> Spherical coordinates where the entity
                #                              will be spawned in the world.
            ],
        )

        # Define guard condition for triggering spawn action when
        # requisite async callbacks finish firing
        self.spawner_ready = self.create_guard_condition(self.spawn_entity)

        self.world_name = self.get_parameter("world_name").value
        self.entity_name = self.get_parameter("entity_name")
        self.allow_renaming = self.get_parameter("allow_renaming")

        # Load the mutually-exclusive filename/string/topic SDF
        # params, use `SdfDataSource` method to handle invalid config
        # (i.e., more/less than one source type set)
        self.sdf_data = SdfDataSource.from_param_dict(
            self.get_parameters_by_prefix("sdf")
        )

        self.pose = GzPose.from_param_dict(self.get_parameters_by_prefix("pose"))

        # If the world name was not provided via ROS param, query the
        # Gazebo server for its list of loaded worlds.  If the request
        # fails (likely due to the server not being fully initialized
        # yet), attempt every 5 seconds until one succeeds.
        if not self.world_name:
            self.world_request_timer = self.create_timer(
                timer_period_sec=5, callback=self.request_gazebo_worlds
            )

        # If the SDF data format is not directly resolvable (in this
        # case, passed as a ROS topic), configure a subscriber to
        # receive the string at runtime.
        if not self.sdf_data.is_resolved():
            self.sdf_subscriber = self.create_subscription(
                std_msgs.msg.String,
                self.sdf_data.topic(),
                self.receive_sdf_string,
                rclpy.qos.QoSProfile(
                    depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
                ),  # latch-like behavior
            )

    def request_gazebo_worlds(self) -> None:
        if self.gz_node.request("/gazebo/worlds", self.receive_gazebo_worlds):
            self._logger.debug("Sent async request to gz server requesting world names")
            # Cancel subsequent invocations of this callback
            self.destroy_timer(self.world_request_timer)
        else:
            self._logger.debug(
                "Request for Gazebo server world list failed; spinning..."
            )

    def receive_gazebo_worlds(
        self,
        _reply: gz_msgs.StringMsg_V,
        _result: bool,
    ) -> None:
        if not _result:
            self._logger.error("Failed to fetch worlds loaded in Gazebo server")
            # FIXME: implement
            raise RuntimeError()
        self._logger.debug(
            f"Received current world list from Gabebo server: {_reply.data}"
        )
        match _reply:
            case []:
                self._logger.error("Gazebo server has no intialized worlds")
                # FIXME: implement
            case [world_name, *rest]:
                if rest:
                    self._logger.warning(
                        f"Gazebo server has %d worlds loaded; using only the first: "
                        f"'{(len(rest) + 1, world_name)}'"
                    )
                self.world_name = world_name
                self.spawner_ready.trigger()

    def receive_sdf_string(self, sdf_msg: std_msgs.msg.String) -> None:
        self._logger.debug(
            f"SDF message string received on ROS topic {self.sdf_data.topic()}"
        )
        # Replace the unresolved ROS_TOPIC-type SdfDataSource object
        # with its newly resolved XML_STRING equivalent
        self.sdf_data = SdfDataSource(
            source_type=SdfSourceType.XML_STRING, data=sdf_msg
        )
        # Schedule an attempt to spawn the entity
        self.spawner_ready.trigger()
        # Cancel subsequent invocations of this subscriber
        self.destroy_subscription(self.sdf_subscriber)

    def generate_factory_msg(self) -> gz_msgs.EntityFactory:
        req = gz_msgs.EntityFactory()

        req.set_name(self.entity_name)
        req.set_allow_renaming(self.allow_renaming)

        match self.sdf_data.source_type:
            case SdfSourceType.FILENAME:
                req.set_sdf_filename(self.sdf_data)
            case SdfSourceType.XML_STRING:
                req.set_sdf(self.sdf_data)
            case SdfSourceType.ROS_TOPIC:
                raise RuntimeError(
                    "Attempted to generate EntityFactory message with unresolved SDF "
                    "data: "
                    f"{self.sdf_data.source_type}"
                )

        req.set_pose(self.pose)

        return req

    def spawn_entity(self) -> None:
        if self.world_name and self.sdf_data.is_resolved():
            self.gz_node.request(
                f"/world/{self.world_name}/create",
                self.generate_factory_msg(),
                self.receive_spawner_response,
            )
        else:
            self._logger.debug(
                "Attempted to spawn entity, but world name/SDF data resolution "
                "is still in progress; spinning..."
            )

    def receive_spawner_response(
        self,
        _reply: bool,
        _result: bool,
    ) -> None:
        if _reply and _result:
            self._logger.debug("Gazebo server entity spawn request successful")
        else:
            self._logger.fatal("Gazebo server entity creation failed")

        # On success or failure, shut down this node
        if self.executor:
            self.executor.shutdown()
        else:
            raise RuntimeError("Node callback hit without valid executor")


def main(args=None):
    try:
        with rclpy.init(args):
            spawner = GzSpawnerNode()
            executor = rclpy.SingleThreadedExecutor()
            executor.add_node(spawner)

            executor.spin()

            # TODO: these shouldn't be necessary when using a context
            #       manager expression, but it's not clear how much of
            #       this functionality has been backported to Jazzy...
            spawner.destroy_node()
            rclpy.shutdown()
    except (KeyboardInterrupt, rclpy.ExternalShutdownException):
        pass
