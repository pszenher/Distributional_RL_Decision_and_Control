from pprint import pp

from typing import List, Optional, Tuple, Mapping, Self, Any

from itertools import chain
from enum import Enum, StrEnum
import dataclasses
from dataclasses import dataclass
from pathlib import Path

import launch
import launch_ros.actions
import ros_gz_bridge

# FIXME clean up imports
import gzilla.mappings.specs
import gzilla.mappings.data
# from gzilla.mappings.specs           import GzTopicSpec
from gzilla.mappings.sensor_mapping  import GzSensorMap
from gzilla.mappings.message_mapping import GzRosMessageMap

import gzilla.sdflib as sdflib

# FIXME: wrap this
from gzilla.codegen.gz_topic_spec import Direction

# TODO: just use enum.auto() calls for value-tagging these (consider StrEnum?)
class RosGzBridgeDirections(StrEnum):
    NONE          = 'NONE'
    GZ_TO_ROS     = 'GZ_TO_ROS'
    ROS_TO_GZ     = 'ROS_TO_GZ'
    BIDIRECTIONAL = 'BIDIRECTIONAL'

    @classmethod
    def from_topic_direction(cls, direction: Direction ) -> Self:
        match direction:
            case Direction.PUB:
                return cls.GZ_TO_ROS
            case Direction.SUB:
                return cls.ROS_TO_GZ


# TODO: just use enum.auto() calls for value-tagging these (consider StrEnum?)
class RosGzBridgeQosProfiles(StrEnum):
    SENSOR_DATA      = "SENSOR_DATA"
    PARAMETERS       = "PARAMETERS"
    SERVICES         = "SERVICES"
    PARAMETER_EVENTS = "PARAMETER_EVENTS"
    ROSOUT           = "ROSOUT"
    SYSTEM_DEFAULT   = "SYSTEM_DEFAULT"
    BEST_AVAILABLE   = "BEST_AVAILABLE"

@dataclass
class TopicBridgeConfig:

    ros_type_name  : str
    ros_topic_name : str
    gz_type_name   : str
    gz_topic_name  : str

    direction: str = "BIDIRECTIONAL"
    """String denoting direction of bridge conversion"""
    
    publisher_queue: int = 10
    """Message queue size for ROS bridge publisher"""
    
    subscriber_queue: int = 10
    """Message queue size for ROS bridge subscriber"""
    
    lazy: bool = False
    """If true, defer starting bridge until a subscriber is spawned to consume it"""

    # NOTE: see `bridge_config.cpp` for valid values, codified in `RosGzBridgeQosProfiles` above
    qos_profile: str = ""
    """Quality of Service profile to apply when creating publisher/subscriber on ROS bridge topic"""

    @classmethod
    def from_topic_spec(cls, topic: gzilla.mappings.specs.GzTopicSpec, message_map ) -> Self:
        return cls(
            ros_type_name    = message_map.query_gz_string( topic.msg_type ).ros2_string(),
            # FIXME: currently setting ROS topic directly to gazebo base name; this is sometimes illegal
            ros_topic_name   = topic.spec.topic.base_name().removeprefix('/'),
            gz_type_name     = topic.msg_type,
            gz_topic_name    = topic.name,
            direction        = str( RosGzBridgeDirections.from_topic_direction( topic.direction ) ),
            subscriber_queue = 10,
            publisher_queue  = 10,
            lazy             = False,
            qos_profile      = "",
        )
    
@dataclass
class ServiceBridgeConfig:
    ros_type_name : str
    gz_req_type_name: str
    gz_rep_type_name: str
    service_name: str

@dataclass
class RosGzBridgeParams:

    bridges : Mapping[
        str, TopicBridgeConfig | ServiceBridgeConfig
    ]
    """Mapping from elements in `bridge_names` to per-bridge parameter structures."""

    bridge_names : List[ str ]
    """List of bridge names, to be used in dereferencing their respective configuration parameters."""

    subscription_heartbeat : int = 1000
    """Time (in ms) to wait in between polling checks for new subscribers, for (unstarted) lazy bridges"""
    
    config_file : str = ''
    """Filename of YAML-formatted bridge configuration file."""

    expand_gz_topic_names : bool = False
    """If true, prepend the ROS namespace to Gazebo topic strings (by default, this is only done for ROS topics)"""

    override_timestamps_with_wall_time : bool = False
    """If true, rewrite GZ_to_ROS message timestamps with system wall-time before republication as ROS message"""

    def merge(self, other: Self) -> Self:
        merged_bridge_dict = {
            f'payload_bridge_{idx}': bridge
            for idx,bridge in enumerate( chain(self.bridges.values(), other.bridges.values()) )
        }
        return dataclasses.replace(
            self,
            bridges      = merged_bridge_dict,
            bridge_names = merged_bridge_dict.keys(),
        )

    def to_flattened_params( self ) -> dict[ str, Any ]:
        return {
            f'bridges.{bridge_name}.{key}': value
            for bridge_name, bridge_conf in self.bridges.items()
            for key,value in dataclasses.asdict(bridge_conf).items()
        } | { 'bridge_names': list( self.bridge_names ) }

    @classmethod
    def from_sdf_model(
            cls,
            model_sdf: sdflib.SdfModel,
            spawner_conf: gzilla.mappings.specs.GzEntitySpawnerConfig,
    ) -> Self:
        if not spawner_conf.world_name:
            raise ValueError(
                'Expected string name of SDF world entity was not provided '
                '(runtime resolution from server not yet implemented)'
            )

        # Fetch sensor sdf configuration parameters via parser
        sensor_configs = model_sdf.sensor_configs()

        sensor_map  : GzSensorMap     = gzilla.mappings.data.sensors()
        message_map : GzRosMessageMap = gzilla.mappings.message_mapping.GzRosMessageMap()


        topic_bridges: list[TopicBridgeConfig] = []
        for sensor_conf in sensor_configs:
            
            # Look up sdf sensor type in sensor map; throw if it's not
            # recognized (which shouldn't be possible unless the SDF spec
            # and/or sdformat implementation changes)
            # 
            if not (sensor_spec := sensor_map.lookup( sensor_conf.sensor_type )):
                raise ValueError(
                    f"Unknown SDF sensor type encountered in input:  '{conf.sensor_type}'"
                )
            runtime_conf = gzilla.mappings.specs.GzSensorRuntimeConfig.from_sdf_and_spawner_configs(
                sensor_conf,
                spawner_conf
            )

            topic_bridges.extend([
                TopicBridgeConfig.from_topic_spec( topic, message_map )
                for topic in sensor_spec.final_msg_topics(
                        runtime_conf,
                        {'topic': sensor_conf.topic} )
            ])

        bridge_dict = {
            f'payload_bridge_{idx}': bridge
            for idx,bridge in enumerate(topic_bridges)
        }
        
        return cls(
            bridges = bridge_dict,
            bridge_names = bridge_dict.keys() ,
        )
    
    @classmethod
    def from_sdf_file(
            cls,
            model_sdf_filename: str,
            spawner_conf: gzilla.mappings.specs.GzEntitySpawnerConfig,
    ) -> Self:
        model_sdf = sdflib.SdfModel.from_sdf_file( model_sdf_filename )
        return cls.from_sdf_model( model_sdf, spawner_conf )
        
    @classmethod
    def from_sdf_string(
            cls,
            model_sdf_string: str,
            spawner_conf: gzilla.mappings.specs.GzEntitySpawnerConfig,
    ) -> Self:
        model_sdf = sdflib.SdfModel.from_sdf_string( model_sdf_string )
        return cls.from_sdf_model( model_sdf, spawner_conf )


@launch.frontend.expose_action('ros_gz_payload_bridge')
class RosGzPayloadBridge(launch.Action):
    """Launch action executing a ros_gz bridge ROS node, with topics determined from SDF file."""

    def __init__(
            self,
            *,
            world_name: str,
            model_name: str,
            sdf_inputs: list[ dict[str,Any] ],
            **kwargs,
    ):
        super().__init__(**kwargs)
        self._logger = launch.logging.get_logger(__name__)

        self._world_name = world_name
        self._model_name = model_name
        self._sdf_inputs = sdf_inputs

    @classmethod
    def parse(
            cls,
            entity: launch.frontend.Entity,
            parser: launch.frontend.Parser
    ) -> tuple[type, dict[str,Any]]:
        _, kwargs = super().parse(entity, parser)

        world_name = entity.get_attr(
            name      = 'world_name',
            data_type = str,
            optional  = False,
        )

        model_name = entity.get_attr(
            name      = 'model_name',
            data_type = str,
            optional  = False,
        )

        one_of_params = {
            'filename': str,
            'string': str,
        }
        sdf_inputs = []
        for child in entity.get_attr(
                name      = 'sdf',
                data_type = List[launch.frontend.Entity],
                optional  = True
        ):
            sdf_filename_param = child.get_attr(
                name      = 'filename',
                data_type = str,
                optional  = True,
            )
            sdf_string_param = child.get_attr(
                name      = 'string',
                data_type = str,
                optional  = True,
            )

            # entity_name_param = child.get_attr(
            #     name      = 'entity_name',
            #     data_type = str,
            #     optional  = False,
            # )

            match [
                param for param in (
                ('filename', sdf_filename_param),
                ('string', sdf_string_param)
            ) if param[1]
            ]:
                case []:
                    raise AttributeError(
                        f"Cannot find at least one of the required attributes "
                        f"'filename', 'string' in Entity '{child.type_name}'"
                    )
                case [(param,val),*rest]:
                    if rest:
                        raise AttributeError(
                            f'multiple mutual-exclusive values set: '
                            f'{[param] + list(map(lambda x: x[0], rest))}'
                        )
                    # self._logger.debug( f'found element requesting sdf from {param}' )
                    sdf_inputs.append({
                        'type': param,
                        'value': parser.parse_substitution(val),
                        'entity_name': parser.parse_substitution(sdf_string_param)
                    })

        kwargs['world_name'] = parser.parse_substitution(world_name)
        kwargs['model_name'] = parser.parse_substitution(model_name)
        kwargs['sdf_inputs'] = sdf_inputs

        return cls, kwargs

    def execute(
            self,
            context: launch.LaunchContext
    ) -> Optional[list[launch.Action]]:
        
        # FIXME: this is atrocious, refactor to hell and back
        # 
        #        we really shouldn't be working with
        #        `RosGzBridgeParams` objects at all here, as the added
        #        structure is a hinderance.  TODO: work directly on
        #        lists of `TopicBridgeConfig`s
        # 
        params = RosGzBridgeParams(
            bridges={},
            bridge_names=[]
        )

        # FIXME: manually destructing lists of substitutions for now,
        #        should handle this properly, and check for ill-formed
        world_name = self._world_name[0].perform(context)
        model_name = self._model_name[0].perform(context)
        
        for sdf_input in self._sdf_inputs:
            match sdf_input:
                case {'type': 'filename', 'value': [filename]}:
                    sdf_model = sdflib.SdfModel.from_sdf_file( filename.perform(context) )
                case {'type': 'string', 'value': [xml_string]}:
                    sdf_model = sdflib.SdfModel.from_sdf_string( xml_string.perform(context) )
                case _:
                    raise ValueError(
                        f"Unrecognized sdf_input value received:  '{sdf_input}'"
                    )
            spawner_conf = gzilla.mappings.specs.GzEntitySpawnerConfig(
                world_name = world_name,
                entity_name = model_name,
            )
            params = params.merge( RosGzBridgeParams.from_sdf_model( sdf_model, spawner_conf ) )
            
        # FIXME: this is atrocious, refactor to hell and back
        #
        # Here we rewrite the kwargs key for `extra_bridge_params` on the fly
        # to include our additional bridges derived from the provided
        # SDF file input source.  We need to be careful to work around
        # behavior in `ros_gz_bridge.py` here, as we're essentially
        # patching this handling behavior in here around it.
        # 
        # for entry in self.__extra_bridge_params:
        #     match entry:
        #         case {
        #             'bridges': bridges,
        #             'bridge_names': bridge_names,
        #         }:
        #             kwargs_aux.append({
        #                 'bridges': bridges | params.bridges,
        #                 'bridge_names': bridge_names + params.bridge_names,
        #             })
        #         case _:
        #             kwargs_aux.append(entry)

        # parsed_bridge_params = {
        #     'bridges':      params.bridges,
        #     # 'bridges':      {name: asdict(spec) for name,spec in params.bridges.items()},
        #     'bridge_names': params.bridge_names
        # }


        # bridge_name = 'payload_bridge_node'
        
        # flattened_bridge_params = {
        #     f'bridges.{bridge_name}.{key}': value
        #     for bridge_name, bridge_conf in params.bridges.items()
        #     for key,value in bridge_conf.items()
        # }

        pp(params.to_flattened_params())
        
        
        # launch_descriptions = super().execute(context)
        return  [
            launch_ros.actions.Node(
                package='ros_gz_bridge',
                executable='bridge_node',
                # FIXME: customize
                name='payload_bridge_node',
                # namespace=self.__namespace,
                output='screen',
                # respawn=perform_typed_substitution(context, self.__use_respawn, bool),
                # respawn_delay=2.0,
                parameters=[ params.to_flattened_params() ]
                # parameters=[{'config_file': self.__config_file, **parsed_bridge_params}],
                # arguments=['--ros-args', '--log-level', self.__log_level],
            )

        ]

    # NOTE: we don't (shouldn't...) need to implement this, as failing
    #       to override it should cause the superclass' method to be
    #       executed when it's invoked; all of the additions we make
    #       above are appended to the data structures used internally,
    #       so it should execute our bridges there as well.
    # 
    # def execute(self, context: launch.LaunchContext) -> Optional[ List[launch.Action] ]:
    #     raise NotImplementedError()
