from typing import Mapping, Optional, Self, List, Tuple, Iterator
import xml.etree.ElementTree as ElementTree
from dataclasses import dataclass
# from pydantic.dataclasses import dataclass

import gzilla.codegen.gz_topic_spec  as gz_topic_spec_schema
import gzilla.codegen.gz_sensor_spec as gz_sensor_spec_schema
import gzilla.codegen.gz_plugin_spec as gz_plugin_spec_schema
from   gzilla.codegen.gz_topic_spec  import PrefixType, Direction, GzTopicSpec

from gzilla.sdflib import SdfSensor, SdfPlugin, SdfModel, SdfWorld

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
class GzRuntimeConfig:
    world_name: str
    model_name: str
    sdf: SdfSensor | SdfPlugin
    
    def model_scoped_prefix(self) -> str:
        return f'/model/{self.model_name}'

    def scoped_prefix(self) -> str:
        # FIXME: this might be a bit too much mustard for a one-line
        #        return statement; refactor for clarity
        return '/'.join((
            '/world', self.world_name, 'model', self.model_name,
            *reversed(
                [
                    f'{sdf_elt.xml_tag}/{sdf_elt.name()}'
                    for sdf_elt in self.sdf.ancestors_up_to( SdfModel | SdfWorld )
                ]
            ),
            # FIXME: this should probably be wrapped into the reversed
            #        iterator above; maybe define new method that iterates to
            #        root but includes current element?
            # 
            self.sdf.xml_tag,
            # TODO: this technically typechecks, but might be better
            #       to have a `Protocol` class definied that captures
            #       expected behavior of SdfSensor and SdfPlugin
            self.sdf.name,
        ))
    
#     link_or_joint_name: str
#     sensor_name: str

#     # FIXME: refactor this to operate on the raw `sdflib` parsed DOM
#     #        handles; the SDF spec is too flexible, we're better off just
#     #        passing the (parent-linked) form of the objects around
#     #
#     #        There is a chance in doing this that we'll hit a wall
#     #        where SDFormat's parsing disagrees with the ECM live
#     #        datamodel, but if we get to that point it is what it is
#     # 
#     def to_scoped_prefix(self) -> str:
#         return (
#             f'/world/{self.world_name}'
#             f'/model/{self.model_name}'
#             # FIXME: we're really pushing it here with this one; the refactor to rectify this will be painful...
#             f'/link/{self.link_or_joint_name}'
#             f'/sensor/{self.sensor_name}'
#         )


#     @staticmethod
#     def from_sdf_and_spawner_configs(
#             sdf_config: SdfSensorConfig,
#             spawner_config: GzEntitySpawnerConfig,
#     ) -> Self:
#         return GzSensorRuntimeConfig(
#             world_name  = spawner_config.world_name,
#             model_name  = spawner_config.entity_name or sdf_config.model_name,
#             link_or_joint_name   = sdf_config.link_or_joint_name,
#             sensor_name = sdf_config.sensor_name,
#         )

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

    def final_name( self, conf: GzRuntimeConfig ) -> str:
        match conf.sdf:
            case SdfSensor():
                if (explicit_topic_name := conf.sdf.topic()):
                    return explicit_topic_name + self.suffix
            case SdfPlugin():
                if self.override:
                    print(self.override)
                    sdf_plugin_tree = ElementTree.fromstring( conf.sdf.xml )
                    overrides = sdf_plugin_tree.findall( f'{self.override}' )
                    # FIXME: refactor, this is too deeply nested,
                    #        shuould use destructuring bind instead of
                    #        list index, warn on multiples, etc.
                    if overrides:
                        return overrides[0].text.strip() + self.suffix
        return self.default_name(conf)
        
    def default_name(
            self,
            conf: Optional[GzRuntimeConfig] = None
    ) -> str:
        match self.prefix_type:
            case PrefixType.SCOPED:
                if not conf:
                    raise ValueError(
                        'Expected runtime configuration parameters for scoped prefix '
                        'topic determination; received:  '
                        f'runtime_config={conf}'
                    )
                return self.scoped_name(conf)
            case PrefixType.MODEL_SCOPED:
                if not conf:
                    raise ValueError(
                        'Expected runtime configuration parameters for model-scoped prefix '
                        'topic determination; received:  '
                        f'runtime_config={conf}'
                    )
                return self.model_scoped_name(conf)
            case PrefixType.ABSOLUTE:
                return self.base_name()
            case PrefixType.CUSTOM:
                raise NotImplementedError(
                    'Received topic name request for topic-generating SDF element '
                    f"with prefix type 'custom', which is not yet implemented: {self}"
                )

    def scoped_name(self, conf: GzRuntimeConfig) -> str :
        return conf.scoped_prefix() + self.base_name()

    def model_scoped_name(self, conf: GzRuntimeConfig) -> str :
        return conf.model_scoped_prefix() + self.base_name()

    def base_name(self) -> str:
        return self.base + self.suffix


class GzTopicSpec(gz_topic_spec_schema.GzTopicSpec):

    topic: GzTopicNameSpec

    def final_msg_topic(
            self,
            runtime_config: GzRuntimeConfig
    ) -> GzTopic:
        return GzTopic(
            name = self.final_name(runtime_config),
            spec = self,
        )
    
    def final_name(
            self,
            runtime_config: GzRuntimeConfig,
    ) -> str:
        return self.topic.final_name(runtime_config)
    
    def default_name(
            self,
            runtime_config: Optional[GzRuntimeConfig] = None
    ) -> str:
        return self.topic.default_name(runtime_config)

class HasTopicsMixin:
    def final_msg_topics(
            self,
            runtime_config,
    ) -> Iterator[ GzTopic ]:
        for topic_spec in self.topics:
            try:
                yield topic_spec.final_msg_topic(runtime_config)
            except NotImplementedError as err:
                # TODO: use real logging here
                print(f'WARN: found unimplemented message topic: {str(err)}')
                
    def default_topics(
            self,
            runtime_config: Optional[GzRuntimeConfig] = None
    ) -> Iterator[str]:
        for topic_spec in self.topics:
            yield topic_spec.default_name(runtime_config)

class GzSensorSpec(gz_sensor_spec_schema.GzSensorSpec, HasTopicsMixin):
    topics: List[GzTopicSpec]

class GzPluginSpec(gz_plugin_spec_schema.GzPluginSpec, HasTopicsMixin):
    topics: List[GzTopicSpec]
