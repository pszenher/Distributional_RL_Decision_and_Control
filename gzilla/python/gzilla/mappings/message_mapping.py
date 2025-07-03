from typing import Sequence, Mapping, Tuple

import ros_gz_bridge

from pydantic import BaseModel

class GzRosMessageMap(BaseModel):
    mapping: Sequence[ros_gz_bridge.MessageMapping] = Field(
        ros_gz_bridge.mappings(None),
        description='Sequence of objects storing known equivalences between GZ and ROS2 message types',
    )

    _gz_type_dict  : Mapping[ str,            ros_gz_bridge.MessageMapping ] = Field( default_factory=dict )
    _ros_type_dict : Mapping[ Tuple[str,str], ros_gz_bridge.MessageMapping ] = Field( default_factory=dict )

    @model_validator(mode='after')
    def check_mapping_unique(self) -> Self:
        
        dups = {
            msgmap
            for msgmap in self.mapping
            if (msgmap.gz_message_name in self._gz_type_dict)
            or (
                self._gz_type_dict.update({ msgmap.gz_message_name: msgmap })
                and
                self._ros_type_dict.update(
                    { ( msgmap.ros2_message_type, msgmap.ros2_message_name ): msgmap }
                )
            )
        }
        
        if dups:
            raise ValueError(
                f"Duplicate message specifications found in GzRos message type map: '{dups}'"
            )
        return self

    def ros_type_from_gz (self, gz_type: str) -> Optional[ros_gz_bridge.MessageMapping]:
        self._gz_type_dict.get(gz_type)

    def gz_type_from_ros (self, ros_pkg: str, ros_type: str) -> Optional[ros_gz_bridge.MessageMapping]:
        self._ros_type_dict.get((ros_pkg, ros_type))
    
    def ros_type_from_gz_string (self, gz_string: str) -> Optional[ros_gz_bridge.MessageMapping]:
        match gz_string.split('.'):
            case [ 'gz', 'msgs', gz_type ]:
                return self.ros_type_from_gz( gz_type )
            case _:
                raise ValueError(
                    f"Invalid gz type string '{gz_string}', expected string prefixed with 'gz.msgs.'"
                )

    def gz_type_from_ros_string(self, ros_string: str) -> Optional[ros_gz_bridge.MessageMapping]:
        match ros_string.split('/'):
            case [ ros_pkg, 'msg', ros_type ]:
                return self.gz_type_from_ros( ros_pkg, ros_type )
            case _:
                raise ValueError(
                    f"Invalid ros type string '{ros_string}', expected string with format "
                    "'{ros_pkg}/msg/{ros_type}'"
                )
        
        
