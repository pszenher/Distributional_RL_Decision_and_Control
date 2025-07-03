from dataclasses import asdict
from pprint import pp

import sdflib

if __name__ == "__main__":
    root = sdflib.SdfRoot.from_sdf_file(
        # "/ws/src/virelex/worlds/sydney_regatta.sdf"
        # "/ws/src/virelex/worlds/sydney_regatta_minimal.sdf"
        "/tmp/wamv.sdf"
        # "/ws/src/virelex/models/robotx_light_buoy_unlit/model.sdf"
    ) 

    match (entity := root.entity()):
        case sdflib.SdfModel():
            pp( list(entity.sensor_configs()) )
            # print(list(map(lambda x: x.name(), entity.links())))
        case None:
            pp( "No entities in provided SDF file" )
        case _:
            print( "fuck" )

    pp( list(root.link_configs()) )

