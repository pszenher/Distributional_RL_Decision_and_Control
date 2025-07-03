from pprint import pp

from typing import Dict, List, Optional, Self, Tuple

import json
import yaml

from pathlib import Path

from pydantic import ValidationError
from pydantic.json import pydantic_encoder

from ament_index_python.packages import get_package_share_directory
from mappings.sensor_mapping import GzSensorMap

gzilla_share_dir = Path( get_package_share_directory('gzilla') )

sensor_map_yaml = gzilla_share_dir / 'config' / 'gz_sensor_mappings.yaml'

if __name__ == '__main__':

    with sensor_map_yaml.open() as f:
        try:
            sensor_map = GzSensorMap.model_validate(
                yaml.safe_load(f)
            )
        except ValidationError as err:
            # FIXME: this shouldn't be a print; log the error properly
            print(err)
            exit(1)         

    # print( sensor_map._sensor_dict )

    # print(
    #     json.dumps(
    #         sensor_map._alias_dict,
    #         default=pydantic_encoder,
    #         indent=4
    #     )
    # )

    print(
        sensor_map.lookup(
            'gps'
        ).model_dump_json( indent=4 )
    )

    pp(
        sensor_map.lookup('gpu_lidar').default_topics(
            GzSensorRuntimeConfig.from_sdf_and_spawner_configs(
                spawner_config=GzEntitySpawnerConfig(
                    "sydney_regatta_minimal",
                    "wamv26"
                ),
                sdf_config=SdfSensorConfig(
                    "WAM-V",
                    "wamv26/base_link",
                    "wamv_imu_sensor",
                    "imu",
                    ""
                )
            )
        )
    )

    # print(
    #     sensor_map.model_dump_json(
    #         indent=4
    #     )
    # )

    # print(
    #     list( map( lambda x: x.aliases, sensor_map.sensors ) )
    # )

    # sensor_map_schema = GzSensorMap.model_json_schema()
    # print(
    #     json.dumps(
    #         sensor_map_schema,
    #         indent=2
    #     )
    # )
