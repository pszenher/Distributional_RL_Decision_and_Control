from typing import Mapping, Optional, Self, List, Tuple

from pydantic.dataclasses import dataclass

import gzilla.codegen.gz_topic_spec  as gz_topic_spec_schema
import gzilla.codegen.gz_sensor_spec as gz_sensor_spec_schema
import gzilla.codegen.gz_plugin_spec as gz_plugin_spec_schema
from   gzilla.codegen.gz_topic_spec  import PrefixType, Direction, GzTopicSpec

@dataclass
class SdfSensorConfig:
    model_name: str
    link_name: str
    sensor_name: str
    sensor_type: str
    topic: str

@dataclass
class GzEntitySpawnerConfig:
    # TODO: world name is known as a function of the world_sdf string,
    #       which we can drag into scope here with a proper launch action
    #       interface
    world_name: Optional[str] = None
    entity_name: Optional[str] = None
    # TODO: this can be specified on the ROS side by any one of the following
    # 
    #     1. SDF xml-encoded string
    #     2. Filename of SDF file
    #     3. ROS topic name of type `std_msgs/msg/String` which periodically publishes the literal SDF/URDF string
    #
    #   The problem is primarily with 3, which cannot be analyzed
    #   statically (as it's a runtime feature).  Not clear yet how to
    #   handle this case without just insisting that people don't use
    #   it...
    # 
    # model_sdf: ???
    # allow_renaming: bool

@dataclass
class GzSensorRuntimeConfig:
    world_name: str
    model_name: str
    link_name: str
    sensor_name: str

    def to_scoped_prefix(self) -> str:
        return (
            f'/world/{self.world_name}'
            f'/model/{self.model_name}'
            f'/link/{self.link_name}'
            f'/sensor/{self.sensor_name}'
        )

    @staticmethod
    def from_sdf_and_spawner_configs(
            sdf_config: SdfSensorConfig,
            spawner_config: GzEntitySpawnerConfig,
    ) -> Self:
        return GzSensorRuntimeConfig(
            world_name  = spawner_config.world_name,
            model_name  = spawner_config.entity_name or sdf_config.model_name,
            link_name   = sdf_config.link_name,
            sensor_name = sdf_config.sensor_name,
        )

@dataclass
class GzTopic:
    name: str
    spec: GzTopicSpec

    @property
    def msg_type(self) -> str:
        return self.spec.msg_type

    @property
    def direction(self) -> Direction:
        return self.spec.direction


class GzTopicNameSpec(gz_topic_spec_schema.GzTopicNameSpec):

    def final_msg_topic_name(
            self,
            runtime_config: GzSensorRuntimeConfig,
            sdf_values: Mapping[str, str],
    ) -> str:
        return self.final_name(runtime_config, sdf_values)
    
    def final_name(
            self,
            runtime_config: GzSensorRuntimeConfig,
            sdf_values: Mapping[str, str],
    ) -> str:
        if (explicit_topic_name := sdf_values.get(self.override)):
            return explicit_topic_name + self.suffix
        return self.default_name(runtime_config)
        
    def default_name(
            self,
            runtime_config: Optional[GzSensorRuntimeConfig] = None
    ) -> str:
        match self.prefix_type:
            case PrefixType.SCOPED:
                if not runtime_config:
                    raise ValueError(
                        'Expected runtime configuration parameters for scoped-prefix '
                        'topic determination; received:  '
                        f'runtime_config={runtime_config}'
                    )
                return self.scoped_name(runtime_config)
            case PrefixType.ABSOLUTE:
                return self.base_name()
    
    def scoped_name(self, conf: GzSensorRuntimeConfig) -> str :
        return conf.to_scoped_prefix() + self.base_name()

    def base_name(self) -> str:
        return self.base + self.suffix


class GzTopicSpec(gz_topic_spec_schema.GzTopicSpec):

    topic: GzTopicNameSpec

    def final_msg_topic(
            self,
            runtime_config: GzSensorRuntimeConfig,
            sdf_values: Mapping[str, str],
    ) -> GzTopic:
        return GzTopic(
            name = self.topic.final_msg_topic_name(runtime_config, sdf_values),
            spec = self,
        )
    
    def final_name(
            self,
            runtime_config: GzSensorRuntimeConfig,
            sdf_values: Mapping[str, str],
    ) -> str:
        return self.topic.final_name(runtime_config, sdf_values)
    
    def default_name(
            self,
            runtime_config: Optional[GzSensorRuntimeConfig] = None
    ) -> str:
        return self.topic.default_name(runtime_config)

class GzSensorSpec(gz_sensor_spec_schema.GzSensorSpec):
    topics: List[GzTopicSpec]

    def final_msg_topics(
            self,
            runtime_config,
            sdf_values
    ) -> List[ GzTopic ]:
        return [
            topic_spec.final_msg_topic(runtime_config, sdf_values)
            for topic_spec in self.topics
        ]

    def default_topics(
            self,
            runtime_config: Optional[GzSensorRuntimeConfig] = None
    ) -> Tuple[str]:
        return tuple(
            topic_spec.default_name(runtime_config)
            for topic_spec in self.topics
        )

class GzPluginSpec(gz_plugin_spec_schema.GzPluginSpec):
    topics: List[GzTopicSpec]

    def default_topics(
            self,
            # FIXME: need new type for plugin runtime config
            #
            #  - should account for differences in different plugin instantiation points in SDF
            #  - some might be under visual, world, model tags;  how to disambiguate?
            # 
            runtime_config: Optional[GzSensorRuntimeConfig] = None
    ) -> Tuple[str]:
        return tuple(
            topic_spec.default_name(runtime_config)
            for topic_spec in self.topics
        )
