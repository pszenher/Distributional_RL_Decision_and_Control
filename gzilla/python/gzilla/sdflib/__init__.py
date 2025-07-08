import os
from enum import Enum
from typing import Union, Iterator, Optional, Self, TypeAlias, ForwardRef, Any, TypeVar, Generic
from types import UnionType
from pathlib import Path, PurePath
from urllib.parse import urlparse
from itertools import repeat
import xml.etree.ElementTree as ElementTree

from pydantic import BaseModel, Field
from pydantic.dataclasses import dataclass
import sdformat14 as sdf

import gzilla.sdflib.util

# @dataclass
# class SdfSensorConfig:
#     model_name: str
#     link_or_joint_name: str
#     sensor_name: str
#     sensor_type: str
#     topic: str

SdfWorldCarrier  : TypeAlias = 'SdfRoot'
SdfModelCarrier  : TypeAlias = Union[ 'SdfRoot', 'SdfWorld', 'SdfModel' ]
SdfSensorCarrier : TypeAlias = Union[ 'SdfLink', 'SdfJoint' ]

# TODO: implement wrappers for the rest of these
SdfPluginCarrier : TypeAlias = Union[
    'SdfWorld',
    'SdfModel',
    'SdfSensor',
    sdf.Visual,
    sdf.Projector,
    sdf.Gui
]

# SdfT = TypeVar('SdfT')
SdfCarrierT = TypeVar('SdfCarrierT')

class SdfWrapper(Generic[SdfCarrierT]):

    _xml_tag: str
    
    def __init__(
            self,
            sdf_raw: Any | None = None,
            parent: SdfCarrierT | None = None
    ) -> Self:
        super().__init__(sdf_raw) if sdf_raw else super().__init__()
        self._parent = parent
    
    @property
    def parent(self) -> SdfCarrierT | None:
        return self._parent

    @property
    def xml_tag(self) -> str:
        return self._xml_tag

    # TODO: is there a way to type this that's valid?
    def ancestors(self) -> Iterator[ 'SdfWrapper[Any]' ]:
        cur_node = self
        while cur_node := cur_node.parent:
            yield cur_node

    def ancestors_up_to(self, sdf_type: type | UnionType) -> Iterator[ 'SdfWrapper[Any]' ]:
        cur_node = self
        while cur_node := cur_node.parent:
            if isinstance(cur_node, sdf_type):
                break
            yield cur_node

class SdfRoot(SdfWrapper[None], sdf.Root):

    _xml_tag: str = 'sdf'
    
    def entity(self) -> ForwardRef('SdfModel') | sdf.Light | None:
        if (model := self.model()):
            return SdfModel(model, self)
        if (light := self.light()):
            return light
        # XXX: this is where an actor check would go, if the pybind
        #      api actually had any methods for them bound...
        # 
        # if (actor := self.actor()):
        #     return actor
        # 
        return None

    def worlds(self) -> Iterator[ 'SdfWorld' ]:
        for world_idx in range(self.world_count()):
            yield SdfWorld(
                self.world_by_index( world_idx )
            )

    def world_models(self) -> Iterator[ tuple['SdfWorld','SdfModel'] ]:
        for world in self.worlds():
            for model in world.models():
                yield (world, model)

    def world_links(self) -> Iterator[ tuple['SdfWorld','SdfModel','SdfLink'] ]:
        for (world,model) in self.world_models():
            for link in model.links():
                yield (world,model,link)

    def world_sensors(
            self
    ) -> Iterator[
        tuple[
            'SdfWorld',
            'SdfModel',
            ForwardRef('SdfLink')|ForwardRef('SdfJoint'),
            'SdfSensor'
        ]
    ]:
        for (world,model) in self.world_models():
            for (link_or_joint,sensor) in model.sensors():
                yield (world,model,link_or_joint,sensor)

    # def link_configs(self):
    #     for (world,model,link) in self.world_links():
    #         # FIXME: this is not a data model;  add a real type or delete this
    #         yield {
    #             'world_name': world.name(),
    #             'model_name': model.name(),
    #             'link_name': link.name(),
    #             'model_plugins': model.plugins(),
    #             'world_plugins': list(map(sdf.Plugin.filename,world.plugins())),
    #         }

    @classmethod
    def from_sdf_file(
            cls,
            sdf_filename: str,
            parser_config: Optional[sdf.ParserConfig] = None,
    ) -> Self:
        with Path( sdf_filename ).open() as f:
            sdf_string = f.read()
        return cls.from_sdf_string(
            sdf_string,
            parser_config
        )

    @classmethod
    def from_sdf_string(
            cls,
            sdf_string: str,
            parser_config: Optional['SdfParserConfig'] = None,
    ) -> Self:
        if not parser_config:
            parser_config = SdfParserConfig()

        # TODO: check the initial contents of the `parser_config` object first, i.e.:
        #     uri_map : Mapping[str, List[str]] = parser_config.uri_path_map

        sdf_tree = ElementTree.fromstring( sdf_string )

        # Find all <uri> tags in SDF file, add mappings to parser config
        for uri_elt in sdf_tree.findall( './/uri' ):
            parser_config.handle_uri( uri_elt.text )

        # Load SDF string into SDFormat parser, using uri-aware parser config
        root = SdfRoot()
        root.load_sdf_string( sdf_string, parser_config )
        return root

class SdfPlugin(SdfWrapper[SdfPluginCarrier], sdf.Plugin):

    _xml_tag: str = 'plugin'
    
    @property
    def name(self) -> str:
        return super().name()

    @property
    def filename(self) -> str:
        return super().filename()

    @property
    def xml(self) -> str:
        return self.__str__()

class SdfSensor(SdfWrapper[SdfSensorCarrier], sdf.Sensor):
    _xml_tag: str = 'sensor'

    @property
    def name(self) -> str:
        return super().name()
    
    def plugins(self) -> Iterator[SdfPlugin]:
        return map(SdfPlugin, super().plugins(), repeat(self))
    
class SdfJoint(SdfWrapper['SdfModel'], sdf.Joint):
    _xml_tag: str = 'joint'
    # def __init__(
    #         self,
    #         raw_joint: sdf.Joint,
    #         parent: Optional['SdfModel'] = None,
    # ) -> Self:
    #     sdf.Joint.__init__(self, raw_joint)
    #     self._parent = parent

    @property
    def parent(self) -> Optional['SdfModel']:
        return self._parent
        
    def sensors(self) -> Iterator[SdfSensor]:
        for sensor_idx in range(self.sensor_count()):
            yield SdfSensor(self.sensor_by_index( sensor_idx ), self)

class SdfLink(SdfWrapper['SdfModel'], sdf.Link):
    _xml_tag: str = 'link'
    # def __init__(
    #         self,
    #         raw_link: sdf.Link,
    #         parent: Optional['SdfModel'] = None,
    # ) -> Self:
    #     sdf.Link.__init__(self, raw_link)
    #     self._parent = parent

    @property
    def parent(self) -> Optional['SdfModel']:
        return self._parent
    
    def sensors(self) -> Iterator[SdfSensor]:
        for sensor_idx in range(self.sensor_count()):
            yield SdfSensor(self.sensor_by_index( sensor_idx ), self)

class SdfModel(SdfWrapper[SdfModelCarrier], sdf.Model):
    _xml_tag: str = 'model'
    # def __init__(
    #         self,
    #         raw_model: sdf.Model,
    #         parent: SdfModelCarrier | None = None,
    # ) -> Self:
    #     sdf.Model.__init__(self, raw_model)
    #     self._parent = parent

    @property
    def parent(self) -> SdfModelCarrier | None:
        return self._parent

    def plugins(self) -> Iterator[SdfPlugin]:
        return map(SdfPlugin, super().plugins(), repeat(self))

    def links(self) -> Iterator[SdfLink]:
        for link_idx in range(self.link_count()):
            yield SdfLink(self.link_by_index( link_idx ))
    def joints(self) -> Iterator[SdfLink]:
        for joint_idx in range(self.joint_count()):
            yield SdfJoint(self.joint_by_index( joint_idx ))
    def sensors(self) -> Iterator[ SdfSensor ]:
        for link in self.links():
            for sensor in link.sensors():
                yield sensor
        for joint in self.joints():
            for sensor in joint.sensors():
                yield sensor

    # def sensor_configs(self) -> Iterator[SdfSensorConfig]:
    #     for (link_or_joint,sensor) in self.sensors():
    #         yield SdfSensorConfig(
    #             model_name  = self.name(),
    #             # TODO: this is trash, refactor
    #             link_or_joint_name = link_or_joint.name(),
    #             sensor_name = sensor.name(),
    #             sensor_type = sensor.type_str(),
    #             topic       = sensor.topic(),
    #         )

    @classmethod
    def from_sdf_root(
            cls,
            root: SdfRoot
    ) -> Self:
        match (entity := root.entity()):
            case gzilla.sdflib.SdfModel():
                return entity
            case _:
                raise ValueError(
                    f'Provided SDFRoot does not contain a root-anchored <model> entity;  found:  {entity}'
                )

    @classmethod
    def from_sdf_string(
            cls,
            sdf_string: str,
            parser_config: Optional[sdf.ParserConfig] = None,
    ) -> Self:
        root = SdfRoot.from_sdf_string( sdf_string, parser_config )
        return cls.from_sdf_root( root )

    @classmethod
    def from_sdf_file(
            cls,
            sdf_filename: str,
            parser_config: Optional[sdf.ParserConfig] = None,
    ) -> Self:
        root = SdfRoot.from_sdf_file( sdf_filename, parser_config )
        return cls.from_sdf_root( root )
        
class SdfWorld(SdfWrapper[SdfWorldCarrier], sdf.World):
    _xml_tag: str = 'world'
    # def __init__(
    #         self,
    #         raw_world: sdf.World,
    #         parent: Optional[SdfWorldCarrier] = None,
    # ) -> Self:
    #     sdf.World.__init__(self, raw_world)
    #     self._parent = parent

    @property
    def parent(self) -> Optional[SdfWorldCarrier]:
        return self._parent

    def plugins(self) -> Iterator[SdfPlugin]:
        return map(SdfPlugin, super().plugins(), repeat(self))

    def models(self) -> Iterator[SdfModel]:
        for model_idx in range(self.model_count()):
            yield SdfModel(self.model_by_index( model_idx ), self)
        
    @staticmethod
    def from_sdf_string(self) -> Self:
        root = SdfRoot.from_sdf_string(
            sdf_string
        )

        match list(root.worlds()):
            case [world]:
                return world
            case []:
                raise ValueError(
                    f'Provided SDF string does not contain any <world> entities, expected exactly 1'
                )
            case [*worlds]:
                raise ValueError(
                    f'Provided SDF string contains {len(worlds)} <world> entities, expected exactly 1'
                )
            
class SdfParserConfig(sdf.ParserConfig):
    def __init__(self) -> Self:
        sdf.ParserConfig.__init__(self)

    @staticmethod
    def local_uri_path(path_string: str) -> str:
        # TODO: should probably drop this method entirely, call the
        #       util func directly inline
        return gzilla.sdflib.util.find_gz_resource( path_string )
        
    @staticmethod
    def fuel_uri_path(netloc: str, path: str) -> str:
        # TODO: refactor this function into `util` module

        # NOTE: while it does very much look like this location would
        #       be one capable of override via env var:  it can't be
        #
        #       all of the references to this hierarchy scattered
        #       across the gazebo codebase leverage the `GZ_HOMEDIR`
        #       macro, which hold string name of the environment
        #       variable to be expanded and prepended to the `.gz`
        #       path
        # 
        #       It just expands to "HOME" (or "USERPROFILE" on Win32)
        #
        #       There doesn't seem to be a fallback here (except for
        #       `logPath`, which is handled in
        #       `gz-common/src/SystemPaths.cc`, so if `${HOME}` is
        #       unset this likely ends up writing to `/.gz` )
        #
        #       see:
        #         https://github.com/gazebosim/gz-common/blob/a31d9ed5be240a697cb42bc1662ad32a6dc7af63/src/SystemPaths.cc#L91C1-L114C2
        # 
        fuel_dir = Path.home() / '.gz' / 'fuel'

        # Unpack uri path parts, append relative path components to fuel cachedir
        [_, fuel_vers, *path_components] = PurePath( path ).parts
        model_path = fuel_dir / netloc / PurePath( *path_components )

        # Select the newest version of the model from fuel cachedir
        # TODO: not clear if fuel cachedir versions are always integer values
        try:
            [model_vers,*_] = sorted(
                [ int(vers := path.name) for path in model_path.iterdir() ]
            )
        except ValueError:
            raise RuntimeError(
                f'Unexpected model version directory encounted in fuel cachedir:  '
                f'{model_path / vers}'
            )

        # Append newest model version directory to cache path
        model_path_versioned = model_path / str(model_vers)
        return model_path_versioned.as_posix()
        
    # FIXME: while the below does handle uri resolution for all
    #        encountered uri's in the primary sdf string/file, it
    #        fails to account for nested uri-resolution which may be
    #        needed by models pointed to by these uri's.
    #
    #        Accounting for this will require that each encountered
    #        URI be aggregated in a list, for which each unique uri
    #        path should then recursively invoke the python sdf
    #        parser, until the closure of recursive uri deps have been
    #        mapped.
    # 
    #        This brings with it the additional complication that some
    #        remote (fuel) uri targets may not yet be cached on the
    #        local disk.  `gz-fuel-tools` doesn't expose a bound
    #        python api, so the easiest option here will likely be to
    #        shell out with a call to `gz fuel download --uri
    #        ${target}`, and hope that the target dir gets populated
    #        as expected (and that the subshell returns in a sane
    #        amount of time).
    # 
    def handle_uri(self, uri_string: str) -> None:
        uri = urlparse( uri_string )
        match uri.scheme:
            case 'https':
                # Insert uri path mapping into sdf parser config
                self.add_uri_path(
                    uri_string,
                    self.fuel_uri_path(uri.netloc, uri.path)
                )
            case "model" | "file" | "":
                # Re-append netloc to uri path if it is non-empty, handle as local path
                self.add_uri_path(
                    uri_string, 
                    self.local_uri_path( uri.netloc + uri.path )
                )
            case "package":
                raise NotImplementedError(
                    f'ROS `package` SDF include URIs are not currently supported: {uri_string}'
                )
            case _:
                raise ValueError(
                    f"Unrecognized URI scheme '{uri.scheme}' in SDF input: {uri_string}"
                )

# def parse_sdf_string(
#         sdf_string: str,
#         parser_config: Optional[SdfParserConfig] = None,
# ) -> SdfRoot:
#     if not parser_config:
#         parser_config = SdfParserConfig()

#     # TODO: check the initial contents of the `parser_config` object first, i.e.:
#     #     uri_map : Mapping[str, List[str]] = parser_config.uri_path_map

#     sdf_tree = ElementTree.fromstring( sdf_string )

#     # Find all <uri> tags in SDF file, add mappings to parser config
#     for uri_elt in sdf_tree.findall( './/uri' ):
#         parser_config.handle_uri( uri_elt.text )

#     # Load SDF string into SDFormat parser, using uri-aware parser config
#     root = SdfRoot()
#     root.load_sdf_string( sdf_string, parser_config )
#     return root

#     # Blindly try each top-level SDF child element getter, since
#     # the Python SDFormat api doesn't give us a better way to test this
#     if (model := root.model()):
#         return model
#     elif (world := root.world()):
#         raise NotImplementedError(
#             '<world> SDF file handling is not yet implemented...'
#         )
#     elif (light := root.light()):
#         raise NotImplementedError(
#             '<light> SDF file handling is not yet implemented...'
#         )
#     else:
#         raise ValueError(
#             f'Unknown child element of SDF;  not <model>, <world>, or <light>'
#         )

# def parse_sdf_file(
#         sdf_filename: str,
#         parser_config: Optional[sdf.ParserConfig] = None,
# ) -> SdfRoot:
#     with Path( sdf_filename ).open() as f:
#         sdf_string = f.read()
#     return parse_sdf_string(
#         sdf_string,
#         parser_config
#     )

# class SdfHandler(BaseModel):
#     _root : sdf.Root = Field( default_factory=sdf.Root )
#     _parser_config : sdf.ParserConfig = Field( default_factory=sdf.ParserConfig.global_config )
            
