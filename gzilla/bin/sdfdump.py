#!/usr/bin/env python3

try:
    from pydantic.dataclasses import dataclass
except ImportError:
    from dataclasses import dataclass

from dataclasses import asdict

from typing import Dict, List, Optional
from enum import Enum
from collections import defaultdict

from pathlib import Path, PurePath
from urllib.parse import urlparse

import os
import json
import yaml
import sys
import xml.etree.ElementTree as ET

import sdformat14 as sdf

from ament_index_python.packages import get_package_share_directory
import ros_gz_bridge

class MessageMappingSet:
    """"""
    def __init__(self, message_mappings: ros_gz_bridge.MessageMapping = ros_gz_bridge.mappings(None)):
        self._mappings = message_mappings

        self._gz_type_dict = defaultdict(list)
        self._ros_type_dict = defaultdict(list)

        for mapping in self._mappings:
            self._gz_type_dict[ mapping.gz_message_name ].append(asdict(mapping))
            self._ros_type_dict[ mapping.ros2_message_name ].append(asdict(mapping))
            
    def find_gz_type(self, gz_type: str):
        return self._gz_type_dict[ gz_type ]

    def find_gz_string(self, gz_type_string: str):
        match gz_type_string.split('.'):
            case [ 'gz', 'msgs', gz_type ]:
                return self.find_gz_type( gz_type )
            case _:
                raise ValueError(
                    f"Invalid gz type string '{gz_type_string}', expected string of form gz.msgs.$name"
                )

@dataclass
class GzMessageType:

    gz_message_name: str

    def gz_string(self):
        return f'gz.msgs.{self.gz_message_name}'

    def gz_type(self):
        return f'gz.msgs.{self.gz_type}'

    @staticmethod
    def _from_full_string( gz_full_string: str, sep: str ):
        match gz_full_string.split( sep ):
            case [ 'gz', 'msgs', gz_type ]:
                return self.find_gz_type( gz_type )
            case _:
                raise ValueError(
                    f"Invalid gz type string '{gz_type_string}', expected string of form gz{sep}msgs{sep}$name"
                )

    @staticmethod
    def from_gz_string( gz_string: str ):
        _from_full_string( gz_string, '.' )

    @staticmethod
    def from_gz_type( gz_string: str ):
        _from_full_string( gz_string, '::' )

@dataclass
class GzSensorSpec:

    model_name: str
    link_name: str
    sensor_name: str

@dataclass
class GzTopicNameSpec:

    class TopicPrefixType(Enum):
        SCOPED = "scoped"
        ABSOLUTE = "absolute"
        
    prefix_type: TopicPrefixType
    base: str
    suffix: str

    def scoped_topic_name(
            self,
            world_name: str,
            spec: GzSensorSpec
    ):
        return self.scoped_prefix(world_name, spec) + self.absolute_topic_name

    # def scoped_no_world_topic_name(
    #         self,
    #         world_name: str,
    #         spec: GzSensorSpec
    # ):
    #     return self.scoped_prefix_no_world(world_name, spec) + self.absolute_topic_name

    def absolute_topic_name(self):
        return f'/{base}/{suffix}'
    
    def scoped_prefix(
            cls,
            world_name: str,
            spec: GzSensorSpec,
    ):
        return f'/world/{world_name}' + cls.topic_name_scoped_no_world(sensor_spec)

    @classmethod
    def scoped_prefix_no_world(
            cls,
            spec: GzSensorSpec
    ):
        return f'/model/{spec.model_name}/link/{spec.link_name}/sensor/{spec.sensor_name}'
        
@dataclass
class GzTopicSpec:
    topics: List[ GzTopicNameSpec ]
    msg_type: GzMessageType
    
@dataclass
class GzSensorType:

    type_name: str
    topic: GzTopicSpec

    # sdf_type: sdf.Sensortype
    
class SensorMappingSet:
    """"""
    def __init__(self, sensor_mappings: Optional[GzSensorType] = None):
        if sensor_mappings is None:
            gzilla_path = Path( get_package_share_directory('gzilla') )
            with ( gzilla_path / 'config' / 'gz_sensor_mappings.yaml' ).open() as f:
                sensor_mappings = yaml.safe_load(f)
            
        self._gz_sensor_dict = sensor_mappings
            
    def find_gz_sensor(self, gz_sensor: str):
        try:
            return self._gz_sensor_dict[ gz_sensor ]
        except KeyError:
            raise ValueError(
                f'Unrecognized gz sensor type: {gz_sensor}'
            )

def plugins_from_sdf(model_sdf: str, config = None) -> List[Dict]:
    root = sdf.Root()
    root.load_sdf_string(model_sdf, config)

    if root.world_count() == 1:
        model = root.world_by_index(0)
    elif root.world_count() > 1:
        raise ValueError(
            'SDF files with multiple worlds are not yet supported by this script.'
        )
    else:
        model = root.model()

    plugins = []
    for plugin in model.plugins():
        tree = ET.fromstring( plugin.__str__() )
        topics = tree.findall( './topic' )
        topic_strings = list( map( lambda t: t.text, topics ) )
        plugins.append({
            'model': model.name(),
            'name': plugin.name(),
            'filename': plugin.filename(),
            'topic': topic_strings,
            'xml': plugin.__str__(),
        })
    return plugins

# class SdfModel(sdf.Model):
#     def 
    
def sensors_from_sdf(model_sdf: str, config = None):
    msg_mapping = MessageMappingSet()
    sensor_mapping = SensorMappingSet()
    
    sensors = []
    root = sdf.Root()
    root.load_sdf_string(model_sdf, config)
    model = root.model()
    for link_index in range(model.link_count()):
        link = model.link_by_index(link_index)
        for sensor_index in range(link.sensor_count()):
            sensor = link.sensor_by_index(sensor_index)
            sensors.append({
                'model': model.name(),
                'link': link.name(),
                'name': sensor.name(),
                'type': sensor.type_str(),
                'topic': sensor.topic(),
                'sensor_map': sensor_mapping.find_gz_sensor(
                    sensor.type_str()
                ),
                # 'sensor_map': sensor_type_map[sensor.type_str()],
                'message_map': [
                    msg_mapping.find_gz_string( sensor_map['gz_type'] )
                    for sensor_map in sensor_mapping.find_gz_sensor(
                            sensor.type_str()
                    )
                ],
                'topic_full': f'/world/TODO/model/{model.name()}/link/{link.name()}/sensor/{sensor.name()}',
                'topic_full_stub': [
                    f'/world/TODO/model/{model.name()}/link/{link.name()}/sensor/{sensor.name()}/{topic_stub['gz_topic']}'
                    for topic_stub in sensor_mapping.find_gz_sensor( sensor.type_str() )
                ],
            })
    return sensors

if __name__ == '__main__':
    sdf_parser_config = sdf.ParserConfig.global_config()

    data = sys.stdin.read()
    tree = ET.fromstring( data )

    # TODO: this location can probably be overridden;  should handle that case
    fuel_dir = Path.home() / '.gz' / 'fuel'
    
    for uri_elt in tree.findall( './/uri' ):
        uri = urlparse( uri_elt.text )
        if uri.scheme == 'https':
            [_, fuel_vers, *path_components] = PurePath( uri.path ).parts
            model_path = fuel_dir / uri.netloc / PurePath( *path_components )
            [model_vers,*_] = sorted( model_path.iterdir() )
            model_path_versioned = model_path / model_vers
            sdf_parser_config.add_uri_path(
                uri_elt.text,
                model_path_versioned.as_posix()
            )
            # print(f'{uri_elt.text}    ->    {model_path_versioned}')
        else:
            for resource_path in os.environ.get("GZ_SIM_RESOURCE_PATH").split( os.pathsep ):
                model_path = Path( resource_path ) / uri.path
                if model_path.exists():
                    sdf_parser_config.add_uri_path(
                        uri_elt.text,
                        model_path.as_posix()
                    )
                    break
                
    # data = ET.tostring( tree )
    
    print( json.dumps( sensors_from_sdf(data, sdf_parser_config), indent=2) )
    # print( json.dumps( plugins_from_sdf(data, sdf_parser_config), indent=2) )
    # print( mappings )
