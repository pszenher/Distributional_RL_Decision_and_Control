""""""

import dataclasses
from collections.abc import Mapping
from dataclasses import dataclass
from enum import StrEnum
from itertools import chain
from typing import TYPE_CHECKING, Any, Self

import launch  # type: ignore [import-untyped]
import launch_ros.actions  # type: ignore [import-untyped]

import gzilla.mappings.data
import gzilla.mappings.specs
from gzilla import sdflib
from gzilla.codegen.gz_topic_spec import Direction

if TYPE_CHECKING:
    from gzilla.mappings.message_mapping import GzRosMessageMap
    from gzilla.mappings.plugin_mapping import GzPluginMap
    from gzilla.mappings.sensor_mapping import GzSensorMap


# TODO(pszenher): just use enum.auto() calls for value-tagging these (consider StrEnum?)
class RosGzBridgeDirections(StrEnum):
    NONE = "NONE"
    GZ_TO_ROS = "GZ_TO_ROS"
    ROS_TO_GZ = "ROS_TO_GZ"
    BIDIRECTIONAL = "BIDIRECTIONAL"

    @classmethod
    def from_topic_direction(cls, direction: Direction) -> "RosGzBridgeDirections":
        match direction:
            case Direction.PUB:
                return cls.GZ_TO_ROS
            case Direction.SUB:
                return cls.ROS_TO_GZ


# TODO(pszenher): just use enum.auto() calls for value-tagging these (consider StrEnum?)
class RosGzBridgeQosProfiles(StrEnum):
    SENSOR_DATA = "SENSOR_DATA"
    PARAMETERS = "PARAMETERS"
    SERVICES = "SERVICES"
    PARAMETER_EVENTS = "PARAMETER_EVENTS"
    ROSOUT = "ROSOUT"
    SYSTEM_DEFAULT = "SYSTEM_DEFAULT"
    BEST_AVAILABLE = "BEST_AVAILABLE"


@dataclass
class TopicBridgeConfig:
    ros_type_name: str
    ros_topic_name: str
    gz_type_name: str
    gz_topic_name: str

    direction: str = "BIDIRECTIONAL"
    """String denoting direction of bridge conversion."""

    publisher_queue: int = 10
    """Message queue size for ROS bridge publisher."""

    subscriber_queue: int = 10
    """Message queue size for ROS bridge subscriber."""

    lazy: bool = False
    """If true, defer starting bridge until a subscriber is spawned to consume it."""

    # NOTE: see `bridge_config.cpp` for valid values, codified in
    #     `RosGzBridgeQosProfiles` above
    qos_profile: str = ""
    """Quality of Service profile to apply when creating pub/sub on ROS bridge topic."""

    @classmethod
    def from_topic_spec(
        cls, topic: gzilla.mappings.specs.GzTopicSpec, message_map
    ) -> Self:
        return cls(
            ros_type_name=message_map.query_gz_string(topic.msg_type).ros2_string(),
            # FIXME(pszenher): currently setting ROS topic directly to
            #     gazebo base name; this is sometimes illegal
            ros_topic_name=topic.spec.topic.base_name().removeprefix("/"),
            gz_type_name=topic.msg_type,
            gz_topic_name=topic.name,
            direction=str(RosGzBridgeDirections.from_topic_direction(topic.direction)),
            subscriber_queue=10,
            publisher_queue=10,
            lazy=False,
            qos_profile="",
        )


@dataclass
class ServiceBridgeConfig:
    ros_type_name: str
    gz_req_type_name: str
    gz_rep_type_name: str
    service_name: str


@dataclass
class RosGzBridgeParams:
    bridges: Mapping[str, TopicBridgeConfig | ServiceBridgeConfig]
    """Mapping from elements in `bridge_names` to per-bridge parameter structures."""

    bridge_names: list[str]
    """List of bridge names, to be used in dereferencing configuration parameters."""

    subscription_heartbeat: int = 1000
    """Time (in ms) to wait in between polling checks for new subscribers.
    Only meaningful on (unstarted) lazy bridges"""

    config_file: str = ""
    """Filename of YAML-formatted bridge configuration file."""

    expand_gz_topic_names: bool = False
    """If true, prepend the ROS namespace to Gazebo topic strings."""

    override_timestamps_with_wall_time: bool = False
    """If true, rewrite GZ_to_ROS message timestamps with system wall-time."""

    def merge(self, other: Self) -> Self:
        merged_bridge_dict = {
            f"payload_bridge_{idx}": bridge
            for idx, bridge in enumerate(
                chain(self.bridges.values(), other.bridges.values())
            )
        }
        return dataclasses.replace(
            self,
            bridges=merged_bridge_dict,
            bridge_names=merged_bridge_dict.keys(),
        )

    def to_flattened_params(self) -> dict[str, Any]:
        return {
            f"bridges.{bridge_name}.{key}": value
            for bridge_name, bridge_conf in self.bridges.items()
            for key, value in dataclasses.asdict(bridge_conf).items()
        } | {"bridge_names": list(self.bridge_names)}

    @classmethod
    def from_sdf_model(
        cls,
        model_sdf: sdflib.SdfModel,
        spawner_conf: gzilla.mappings.specs.GzEntitySpawnerConfig,
    ) -> Self:
        if not spawner_conf.world_name:
            msg = (
                "Expected string name of SDF world entity was not provided "
                "(runtime resolution from server not yet implemented)"
            )
            raise ValueError(msg)

        # Setup relevant sensor/plugin/message mappings
        # TODO(pszenher): we shouldn't need to init these; make them
        #     global exports of their respective modules
        sensor_map: GzSensorMap = gzilla.mappings.data.sensors()
        plugin_map: GzPluginMap = gzilla.mappings.data.plugins()
        message_map: GzRosMessageMap = gzilla.mappings.message_mapping.GzRosMessageMap()

        # TODO(pszenher): adding this here so we can resolve all names; should
        #       eventually allow for hot-loading these with a child
        #       element in config (sibling to current `sdf` elements)
        #
        vrx_plugin_map: GzPluginMap = gzilla.mappings.data.vrx_plugins()

        # Fetch sensor sdf configuration parameters via parser
        sensors = model_sdf.sensors()
        # Fetch plugin sdf configuration parameters via parser
        plugins = model_sdf.plugins()

        topic_bridges: list[TopicBridgeConfig] = []
        for sensor in sensors:
            # Look up sdf sensor type in sensor map; throw if it's not
            # recognized (which shouldn't be possible unless the SDF spec
            # and/or sdformat implementation changes)
            #
            sensor_type = sensor.type_str()

            if not (sensor_spec := sensor_map.lookup(sensor_type)):
                msg = f"Unknown SDF sensor type encountered in input:  '{sensor_type}'"
                raise ValueError(msg)
            runtime_conf = gzilla.mappings.specs.GzRuntimeConfig(
                world_name=spawner_conf.world_name,
                model_name=spawner_conf.entity_name,
                sdf=sensor,
            )

            topic_bridges.extend(
                [
                    TopicBridgeConfig.from_topic_spec(topic, message_map)
                    for topic in sensor_spec.final_msg_topics(runtime_conf)
                ]
            )

        for plugin in plugins:
            # Look up sdf plugin type in plugin map; throw if it's not
            # recognized (this is expected, TODO(pszenher): refactor)
            #
            if plugin_spec := plugin_map.lookup(
                plugin.filename
            ) or vrx_plugin_map.lookup(plugin.filename):
                pass
            else:
                msg = (
                    f"Unknown SDF plugin type (filename) encountered in input:  "
                    f"'{plugin.filename}'"
                )
                raise ValueError(msg)
            runtime_conf = gzilla.mappings.specs.GzRuntimeConfig(
                world_name=spawner_conf.world_name,
                model_name=spawner_conf.entity_name,
                sdf=plugin,
            )

            topic_bridges.extend(
                [
                    TopicBridgeConfig.from_topic_spec(topic, message_map)
                    for topic in plugin_spec.final_msg_topics(runtime_conf)
                ]
            )

        bridge_dict = {
            f"payload_bridge_{idx}": bridge for idx, bridge in enumerate(topic_bridges)
        }

        return cls(
            bridges=bridge_dict,
            bridge_names=bridge_dict.keys(),
        )

    @classmethod
    def from_sdf_file(
        cls,
        model_sdf_filename: str,
        spawner_conf: gzilla.mappings.specs.GzEntitySpawnerConfig,
    ) -> Self:
        model_sdf = sdflib.SdfModel.from_sdf_file(model_sdf_filename)
        return cls.from_sdf_model(model_sdf, spawner_conf)

    @classmethod
    def from_sdf_string(
        cls,
        model_sdf_string: str,
        spawner_conf: gzilla.mappings.specs.GzEntitySpawnerConfig,
    ) -> Self:
        model_sdf = sdflib.SdfModel.from_sdf_string(model_sdf_string)
        return cls.from_sdf_model(model_sdf, spawner_conf)


@launch.frontend.expose_action("ros_gz_payload_bridge")
class RosGzPayloadBridge(launch.Action):
    """Launch action executing a ros_gz bridge ROS node, with topics from SDF file."""

    def __init__(
        self,
        *,
        world_name: str,
        sdf_inputs: list[dict[str, Any]],
        **kwargs,
    ):
        super().__init__(**kwargs)
        self._logger = launch.logging.get_logger(__name__)

        self._world_name = world_name
        self._sdf_inputs = sdf_inputs

    @classmethod
    def parse(
        cls, entity: launch.frontend.Entity, parser: launch.frontend.Parser
    ) -> tuple[type, dict[str, Any]]:
        _, kwargs = super().parse(entity, parser)

        world_name = entity.get_attr(
            name="world_name",
            data_type=str,
            optional=False,
        )

        sdf_inputs = []
        for child in entity.get_attr(
            name="sdf", data_type=list[launch.frontend.Entity], optional=True
        ):
            sdf_filename_param = child.get_attr(
                name="filename",
                data_type=str,
                optional=True,
            )
            sdf_string_param = child.get_attr(
                name="string",
                data_type=str,
                optional=True,
            )

            entity_name_param = child.get_attr(
                name="entity_name",
                data_type=str,
                optional=True,
            )

            match [
                (sdf_type, val)
                for (sdf_type, val) in (
                    ("filename", sdf_filename_param),
                    ("string", sdf_string_param),
                )
                if val
            ]:
                case []:
                    msg = (
                        f"Cannot find at least one of the required attributes "
                        f"'filename', 'string' in Entity '{child.type_name}'"
                    )
                    raise AttributeError(msg)
                case [(param, val), *rest]:
                    if rest:
                        msg = (
                            f"multiple mutual-exclusive values set: "
                            f"{[param, *[x[0] for x in rest]]}"
                            # TODO(pszenher): this iterator is hideous, refactor
                        )
                        raise AttributeError(msg)
                    sdf_inputs.append(
                        {
                            "type": param,
                            "value": parser.parse_substitution(val),
                            "entity_name": parser.parse_substitution(entity_name_param),
                        }
                    )

        kwargs["world_name"] = parser.parse_substitution(world_name)
        kwargs["sdf_inputs"] = sdf_inputs

        return cls, kwargs

    def execute(self, context: launch.LaunchContext) -> list[launch.Action] | None:
        # FIXME(pszenher): this is atrocious, refactor to hell and back
        #
        #        we really shouldn't be working with
        #        `RosGzBridgeParams` objects at all here, as the added
        #        structure is a hinderance.  TODO(pszenher): work directly on
        #        lists of `TopicBridgeConfig`s
        #
        params = RosGzBridgeParams(bridges={}, bridge_names=[])

        # FIXME(pszenher): manually destructing lists of substitutions for now,
        #        should handle this properly, and check for ill-formed
        world_name = self._world_name[0].perform(context)

        for sdf_input in self._sdf_inputs:
            match sdf_input:
                case {"type": "filename", "value": [filename]}:
                    sdf_model = sdflib.SdfModel.from_sdf_file(filename.perform(context))
                case {"type": "string", "value": [xml_string]}:
                    sdf_model = sdflib.SdfModel.from_sdf_string(
                        xml_string.perform(context)
                    )
                case _:
                    msg = f"Unrecognized sdf_input value received:  '{sdf_input}'"
                    raise ValueError(msg)

            if entity_params := sdf_input.get("entity_name"):
                match entity_params:
                    case [entity_param]:
                        entity_name = entity_param.perform(context)
                    case _:
                        msg = (
                            f"Unrecognized entity_name param received:  "
                            f"'{entity_params}'"
                        )
                        raise ValueError(msg)

            spawner_conf = gzilla.mappings.specs.GzEntitySpawnerConfig(
                world_name=world_name,
                entity_name=entity_name,
            )
            params = params.merge(
                RosGzBridgeParams.from_sdf_model(sdf_model, spawner_conf)
            )

        return [
            launch_ros.actions.Node(
                package="ros_gz_bridge",
                executable="bridge_node",
                # FIXME(pszenher): customize
                name="payload_bridge_node",
                output="screen",
                parameters=[params.to_flattened_params()],
            )
        ]
