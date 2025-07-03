from pprint import pp

from pathlib import Path

from xsdata_pydantic.bindings import XmlParser
from codegen_xsd.sdf import Sdf

if __name__ == "__main__":
    parser = XmlParser()

    sdf_path = "../worlds/sydney_regatta.sdf"
    
    with Path(sdf_path).open() as f:
        root = parser.parse(f, Sdf)

    print(
        root.model_dump_json( indent=2 )
    )
