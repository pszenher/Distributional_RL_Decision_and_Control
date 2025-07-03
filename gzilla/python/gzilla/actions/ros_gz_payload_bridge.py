from typing import List, Optional, Tuple, Mapping

from enum import Enum
from dataclasses import asdict, dataclass
from pathlib import Path

import launch
import launch_ros.actions
import ros_gz_bridge

import gzilla.mappings.specs
from gzilla.mappings.sensor_mapping import GzSensorMap
from gzilla.mappings.message_mapping import GzRosMessageMap

# TODO: just use enum.auto() calls for value-tagging these (consider StrEnum?)
class RosGzBridgeDirections(Enum):
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
class RosGzBridgeQosProfiles(Enum):
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
    def from_topic_spec(cls, topic: GzTopicSpec ) -> Self:
        return cls(
            ros_type_name    = message_map.ros_type_from_gz_string( topic.msg_type ),
            # FIXME: currently setting ROS topic directly to gazebo topic;  this is sometimes illegal
            ros_topic_name   = topic.name,
            gz_type_name     = topic.msg_type,
            gz_topic_name    = topic.name,
            direction        = RosGzBridgeDirections.from_topic_direction( topic.direction ),
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
            f'payload_bridge_{idx}': asdict(bridge)
            for idx,bridge in enumerate( self.bridges.values() + other.bridges.values() )
        }
        return self.replace(
            bridges      = merged_bridge_dict,
            bridge_names = merged_bridge_dict.keys(),
        )
            
    @classmethod
    def from_sdf_model(
            cls,
            model_sdf: sdflib.SdfModel,
            spawner_conf: mappings.specs.GzEntitySpawnerConfig,
    ) -> Self:
        if not spawner_conf.world_name:
            raise ValueError(
                'Expected string name of SDF world entity was not provided '
                '(runtime resolution from server not yet implemented)'
            )

        # Fetch sensor sdf configuration parameters via parser
        sensor_configs = model_sdf.sensor_configs()

        sensor_map  : GzSensorMap     = mappings.data.sensors()
        message_map : GzRosMessageMap = mappings.message_mapping.GzRosMessageMap()


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
            runtime_conf = mappings.specs.GzSensorRuntimeConfig.from_sdf_and_spawner_configs(
                sensor_conf,
                spawner_conf
            )

            topic_bridges.extend([
                TopicBridgeConfig.from_topic_spec( topic )
                for topic in sensor_spec.topics(
                        runtime_conf,
                        {'topic': sensor_conf.topic} )
            ])

        bridge_dict = {
            f'payload_bridge_{idx}': asdict(bridge)
            for idx,bridge in enumerate(topic_bridges)
        }
        
        return cls(
            bridges = bridge_dict,
            bridge_names = bridge_dict.keys(),
        )
    
    @classmethod
    def from_sdf_file(
            cls,
            model_sdf_filename: str,
            spawner_conf: mappings.specs.GzEntitySpawnerConfig,
    ) -> Self:
        model_sdf = sdflib.SdfModel.from_sdf_file( model_sdf_filename )
        return cls.from_sdf_model( model_sdf, spawner_conf )
        
    @classmethod
    def from_sdf_string(
            cls,
            model_sdf_string: str,
            spawner_conf: mappings.specs.GzEntitySpawnerConfig,
    ) -> Self:
        model_sdf = sdflib.SdfModel.from_sdf_string( model_sdf_string )
        return cls.from_sdf_model( model_sdf, spawner_conf )


@expose_action('ros_gz_payload_bridge')
class RosGzPayloadBridge(ros_gz_bridge.action.RosGzBridge):
    """Launch action executing a ros_gz bridge ROS node, with topics determined from SDF file."""

    def __init__( self, **kwargs, ):
        super().__init__(**kwargs)
        self._logger = launch.logging.get_logger(__name__)

    @classmethod
    def parse(cls, entity: Entity, parser: Parser):
        _, kwargs = super().parse(entity, parser)
        one_of_params = {
            'filename': str,
            'string': str,
        }
        sdf_inputs = []
        for child in entity.get_attr('sdf', data_type=List[Entity], optional=True):
            params = {
                param: parser.parse_substitution(
                    child.get_attr( name=param,
                                    data_type=dtype,
                                    optional=True )
                )
                for (param,dtype) in one_of_params.items()
            }
            params_filtered = filter(
                lambda _k,v: v is not None,
                params.items()
            )
            match list(params_filtered):
                case []:
                    raise AttributeError(
                        f"Cannot find at least one of the required attributes "
                        f"'{one_of_params.keys()}' in Entity '{child.type_name}'"
                    )
                case [(param,val),*rest]:
                    if rest:
                        self._logger.warning(
                            f'multiple mutual-exclusive values set: '
                            f'{[param] + map(lambda x: x[0], rest)}, '
                            f"using '{param}'"
                        )
                    self._logger.debug( f'found element requesting sdf from {param}' )
                    sdf_inputs.append({
                        'type': param,
                        'value': val,
                    })
                    # sdf_param = param
                    # sdf_val = val

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
        
        for sdf_input in sdf_inputs:
            match sdf_input:
                case {'type': 'filename', 'content': filename}:
                    params.merge(
                        RosGzBridgeParams.from_sdf_file( filename, spawner_conf )
                    )
                case {'type': 'string', 'content': xml_string}:
                    params.merge(
                        RosGzBridgeParams.from_sdf_string( xml_string, spawner_conf )
                    )
                case _:
                    raise ValueError(
                        f"Unrecognized sdf_input value received:  '{sdf_input}'"
                    )
            
        # FIXME: this is atrocious, refactor to hell and back
        #
        # Here we rewrite the kwargs key for `extra_bridge_params` on the fly
        # to include our additional bridges derived from the provided
        # SDF file input source.  We need to be careful to work around
        # behavior in `ros_gz_bridge.py` here, as we're essentially
        # patching this handling behavior in here around it.
        # 
        kwargs_aux = []
        for entry in kwargs['extra_bridge_params']:
            match entry:
                case {
                    'bridges': bridges,
                    'bridge_names': bridge_names,
                }:
                    kwargs_aux.append({
                        'bridges': bridges | params.bridges,
                        'bridge_names': bridge_names + params.bridge_names,
                    })
                case _:
                    kwargs_aux.append(entry)
                    
        return cls, kwargs

    # NOTE: we don't (shouldn't...) need to implement this, as failing
    #       to override it should cause the superclass' method to be
    #       executed when it's invoked; all of the additions we make
    #       above are appended to the data structures used internally,
    #       so it should execute our bridges there as well.
    # 
    # def execute(self, context: launch.LaunchContext) -> Optional[ List[launch.Action] ]:
    #     raise NotImplementedError()
