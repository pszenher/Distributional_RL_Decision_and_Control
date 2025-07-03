import os
from enum import Enum
from typing import Iterator, Generator, Optional, Self, Tuple, Union
from pathlib import Path, PurePath
from urllib.parse import urlparse
import xml.etree.ElementTree as ElementTree

from pydantic import BaseModel, Field
from pydantic.dataclasses import dataclass
import sdformat14 as sdf

import sdflib.util

@dataclass
class SdfSensorConfig:
    model_name: str
    link_name: str
    sensor_name: str
    sensor_type: str
    topic: str

class SdfSensor(sdf.Sensor):
    pass
    
class SdfLink(sdf.Link):
    def sensors(self) -> Iterator[SdfSensor]:
        for sensor_idx in range(self.sensor_count()):
            yield SdfSensor(self.sensor_by_index( sensor_idx ))
    
class SdfModel(sdf.Model):
    def links(self) -> Iterator[SdfLink]:
        for link_idx in range(self.link_count()):
            yield SdfLink(self.link_by_index( link_idx ))
    def sensors(self) -> Iterator[ Tuple[SdfLink,SdfSensor] ]:
        for link in self.links():
            for sensor in link.sensors():
                yield (link, sensor)
    def sensor_configs(self) -> Iterator[SdfSensorConfig]:
        for (link,sensor) in self.sensors():
            yield SdfSensorConfig(
                model_name  = self.name(),
                link_name   = link.name(),
                sensor_name = sensor.name(),
                sensor_type = sensor.type_str(),
                topic       = sensor.topic(),
            )
    @staticmethod
    def from_sdf_string(sdf_string: str) -> Self:
        root = SdfRoot.from_sdf_string(
            sdf_string
        )

        match (entity := root.entity()):
            case sdflib.SdfModel():
                return entity
            case _:
                raise ValueError(
                    f'Provided SDF string does not contain a root-anchored <model> entity;  found:  {entity}'
                )


class SdfWorld(sdf.World):
    def models(self) -> Iterator[SdfModel]:
        for model_idx in range(self.model_count()):
            yield SdfModel(self.model_by_index( model_idx ))

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

class SdfRoot(sdf.Root):
    
    def __init__(self) -> Self:
        sdf.Root.__init__(self)

    def entity(self) -> Union[ SdfModel, sdf.Light, None ]:
        if (model := self.model()):
            return SdfModel(model)
        if (light := self.light()):
            return light
        # XXX: this is where an actor check would go, if the pybind
        #      api actually had any methods for them bound...
        # 
        # if (actor := self.actor()):
        #     return actor
        # 
        return None

    def worlds(self) -> Iterator[ SdfWorld ]:
        for world_idx in range(self.world_count()):
            yield SdfWorld(
                self.world_by_index( world_idx )
            )

    def world_models(self) -> Iterator[ Tuple[SdfWorld,SdfModel] ]:
        for world in self.worlds():
            for model in world.models():
                yield (world, model)

    def world_links(self) -> Iterator[ Tuple[SdfWorld,SdfModel,SdfLink] ]:
        for (world,model) in self.world_models():
            for link in model.links():
                yield (world,model,link)

    def world_sensors(self) -> Iterator[ Tuple[SdfWorld,SdfModel,SdfLink,SdfSensor] ]:
        for (world,model) in self.world_models():
            for (link,sensor) in model.sensors():
                yield (world,model,link,sensor)

    def link_configs(self):
        for (world,model,link) in self.world_links():
            # FIXME: this is not a data model;  add a real type or delete this
            yield {
                'world_name': world.name(),
                'model_name': model.name(),
                'link_name': link.name(),
                'model_plugins': model.plugins(),
                'world_plugins': list(map(sdf.Plugin.filename,world.plugins())),
            }

    @staticmethod
    def from_sdf_file(
            sdf_filename: str,
            parser_config: Optional[sdf.ParserConfig] = None,
    ) -> SdfRoot:
        with Path( sdf_filename ).open() as f:
            sdf_string = f.read()
        return SdfRoot.parse_sdf_string(
            sdf_string,
            parser_config
        )

    @staticmethod
    def from_sdf_string(
        sdf_string: str,
        parser_config: Optional[SdfParserConfig] = None,
    ) -> SdfRoot:
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

        # Blindly try each top-level SDF child element getter, since
        # the Python SDFormat api doesn't give us a better way to test this
        if (model := root.model()):
            return model
        elif (world := root.world()):
            raise NotImplementedError(
                '<world> SDF file handling is not yet implemented...'
            )
        elif (light := root.light()):
            raise NotImplementedError(
                '<light> SDF file handling is not yet implemented...'
            )
        else:
            raise ValueError(
                f'Unknown child element of SDF;  not <model>, <world>, or <light>'
            )



class SdfParserConfig(sdf.ParserConfig):
    def __init__(self) -> Self:
        sdf.ParserConfig.__init__(self)

    @staticmethod
    def local_uri_path(path_string: str) -> str:
        # TODO: should probably drop this method entirely, call the
        #       util func directly inline
        return sdflib.util.find_gz_resource()
        
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
            
