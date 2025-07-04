from typing import Sequence, Mapping, Tuple, Self, Optional
from collections import defaultdict

import ros_gz_bridge

from pydantic import BaseModel, Field, PrivateAttr

class GzRosMessageMap(BaseModel):
    mapping: Sequence[ros_gz_bridge.MessageMapping] = Field(
        ros_gz_bridge.mappings(None),
        description='Sequence of objects storing known equivalences between GZ and ROS2 message types',
    )

    _gz_type_dict  : Mapping[ str, list[ros_gz_bridge.MessageMapping] ] = PrivateAttr()
    _ros_type_dict : Mapping[ Tuple[str,str], ros_gz_bridge.MessageMapping ] = PrivateAttr()

    def __init__(self, **data) -> Self:
        super().__init__(**data)
        self._gz_type_dict  = self._generate_gz_type_dict( self.mapping )
        self._ros_type_dict = self._generate_ros_type_dict( self.mapping )

    @staticmethod
    def _generate_gz_type_dict(
            mapping: Sequence[ros_gz_bridge.MessageMapping],
    ) -> Mapping[ str, ros_gz_bridge.MessageMapping ]:
        gz_dict = defaultdict(list)
        for msgmap in mapping:
            gz_dict[msgmap.gz_message_name].append(msgmap)
        return gz_dict
        # dups = {
        #     (msgmap.gz_message_name, msgmap.ros2_package_name, msgmap.ros2_message_name)
        #     for msgmap in mapping
        #     if (msgmap.gz_message_name in gz_dict)
        #     or gz_dict.update({ msgmap.gz_message_name: msgmap })
        # }
        # if dups:
        #     raise ValueError(
        #         f"Duplicate GZ message type found in type map:  '{dups}'"
        #     )

    @staticmethod
    def _generate_ros_type_dict(
            mapping: Sequence[ros_gz_bridge.MessageMapping],
    ) -> Mapping[ str, ros_gz_bridge.MessageMapping ]:
        ros_dict = dict()
        dups = {
            (msgmap.gz_message_name, msgmap.ros2_package_name, msgmap.ros2_message_name)
            for msgmap in mapping
            if (msgmap.gz_message_name in ros_dict)
            or ros_dict.update(
                { ( msgmap.ros2_package_name, msgmap.ros2_message_name ): msgmap }
            )
        }
        if dups:
            raise ValueError(
                f"Duplicate ROS message type found in type map:  '{dups}'"
            )
        return ros_dict
    
    def query_gz (self, gz_type: str) -> Optional[ros_gz_bridge.MessageMapping]:
        match self._gz_type_dict.get(gz_type):
            case None:
                raise ValueError(
                    f"Unknown GZ message type passed for conversion:  '{gz_type}'"
                )
            case [ros_type, *rest]:
                if rest:
                    print(f"WARN:  multiple ROS types map to GZ type '{gz_type}', using first ('{ros_type.ros2_string()}')")
                return ros_type

    def query_ros (self, ros_pkg: str, ros_type: str) -> Optional[ros_gz_bridge.MessageMapping]:
        return self._ros_type_dict.get((ros_pkg, ros_type))
    
    def query_gz_string (self, gz_string: str) -> Optional[ros_gz_bridge.MessageMapping]:
        match gz_string.split('.'):
            case [ 'gz', 'msgs', gz_type ]:
                return self.query_gz( gz_type )
            case _:
                raise ValueError(
                    f"Invalid gz type string '{gz_string}', expected string prefixed with 'gz.msgs.'"
                )

    def query_ros_string(self, ros_string: str) -> Optional[ros_gz_bridge.MessageMapping]:
        match ros_string.split('/'):
            case [ ros_pkg, 'msg', ros_type ]:
                return self.query_ros( ros_pkg, ros_type )
            case _:
                raise ValueError(
                    f"Invalid ros type string '{ros_string}', expected string with format "
                    "'{ros_pkg}/msg/{ros_type}'"
                )
        
        
