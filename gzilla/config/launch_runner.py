import sdflib

import mappings.data
import mappings.specs

import mappings.message_mapping

from pydantic.dataclasses import dataclass
from pydantic.types import NonNegativeInt
from enum import Enum

# XXX: see note on `BridgeConfig` below;  same applies here
# 
class BridgeDirection(Enum):
    NONE = 0
    BIDIRECTIONAL = 1
    GZ_TO_ROS = 2
    ROS_TO_GZ = 3

# XXX: this could (should?) be handled as a pybind11 binding to the
#      `BridgeConfig` struct in `ros_gz_bridge/bridge_config.hpp`; for
#      now though, just recreate it so we know the structure of ros
#      params to send it
# 
@dataclass
class BridgeConfig:
    ros_type_name: str
    ros_topic_name: str
    gz_type_name: str
    gz_topic_name: str
    direction: BridgeDirection = BridgeDirection.BIDIRECTIONAL
    subscriber_queue: NonNegativeInt = 10
    publisher_queue: NonNegativeInt = 10
    lazy: bool = False

# Generate a set of `BridgeConfig` objects from the information
# available to us when invoked from a ROS launch file.
# 
def from_launch(
        model_sdf_string: str,
        spawner_conf: GzEntitySpawnerConfig,
) -> Iterator[BridgeConfig]:

    # Might be better wrapped in the EntitySpawner config object, as
    # it's essentially coming from there in all cases (except the ROS
    # topic one, which gets more complicated)
    #
    # There's a separate note here about URDF being passed directly to
    # the Gazebo parser, but we won't touch that just yet...
    # 
    model_sdf = sdflib.SdfModel.from_sdf_string( model_sdf_string )

    # TODO: Get world name straight from the source; we can't misalign
    #       on this if we just read it ourselves.  This saves us from
    #       the weird mismatch between world file name and world
    #       `name` attr, especially in cases where a world file is
    #       copied in order to make custom changes.  There's no good
    #       reason to require a world pre-extension filename exactly
    #       match the string name in its file contents.
    #
    if not spawner_conf.world_name:
        raise ValueError(
            'Expected string name of SDF world entity was not provided '
            '(runtime resolution from server not yet implemented)'
        )

    # Fetch sensor sdf configuration parameters via parser
    sensor_configs = model_sdf.sensor_configs()

    # Initialize the known mapping associations
    sensor_map  : GzSensorMap = mappings.data.sensors()
    message_map : GzRosMessageMap = mappings.message_mapping.GzRosMessageMap()
    
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

        # Perform realization of sensor specification topic names,
        # using a `GzSensorRuntimeConfig` object and a dict of
        # relevant SDF element keys relevant to this action
        # 
        # TODO: the `sdf_values` field is a janky attempt to make the
        #       sensor topic realization interface identical to the
        #       one for plugins.  We can do better, refactor...
        # 
        topics = sensor_spec.topics(
            runtime_conf,
            {'topic': sensor_conf.topic}
        )

        # FIXME: move this literally anywhere else; not yet clear
        #        where though (utility method?)
        #
        # Convert from GzTopic message `Direction` to a ros_gz_bridge `BridgeDirection`
        # 
        def direction_to_bridge_direction( direction: Direction ) -> BridgeDirection:
            match direction:
                case Direction.PUB:
                    return BridgeDirection.GZ_TO_ROS
                case Direction.SUB:
                    return BridgeDirection.ROS_TO_GZ

        # FIXME: currently have `ros_topic_name` set to
        #        `NotImplemented`, as we don't have a good way to
        #        guess a good mapping yet.  Doing so will probably
        #        rely on having an additional mechanism for decrees
        #        on-high (i.e., the launch file) to pass additional
        #        parameters which reach this depth, scoped to a
        #        specific sensor/topic pair.
        # 
        #        This is a bit difficult, as we don't have a
        #        well-defined system of unique identity for sensor
        #        topics from the SDF spec (as topic name isn't
        #        realized there).  
        #
        # TODO: we need a way of providing values to some of the
        #       optional parameters below, in addition to the
        #       `ros_topic_name` issue noted above.
        # 
        #       We can play a bit more fast-and-loose with unique
        #       identity for this, as it might be reasonable to say
        #       "make all topics of a given message type lazy" or
        #       "make all topics of a given sensor lazy", or even
        #       "make all topics of a given sensor with a given
        #       message type lazy", none of which rely on a strong
        #       definition of topic identity.
        # 
        # Iterate over realized topics, yielding a `BridgeConfig` object for each one
        # 
        for topic in topics:
            yield BridgeConfig(
                ros_type_name    = message_map.ros_type_from_gz_string(
                    topic.msg_type
                ),
                ros_topic_name   = NotImplemented,
                gz_type_name     = topic.msg_type,
                gz_topic_name    = topic.name,
                direction        = direction_to_bridge_direction( topic.direction ),
                subscriber_queue = 10,
                publisher_queue  = 10,
                lazy             = False
            ) 
