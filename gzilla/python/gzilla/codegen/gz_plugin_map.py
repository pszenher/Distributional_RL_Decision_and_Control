# generated by datamodel-codegen:
#   filename:  gz_plugin_map.json
#   timestamp: 2025-07-03T15:41:37+00:00
#   version:   0.31.2

from __future__ import annotations

from typing import List, Optional

from pydantic import BaseModel

from .gz_plugin_spec import GzPluginSpec


class GzPluginMap(BaseModel):
    """
    List of gz plugin specifications
    """

    class Config:
        allow_mutation = False

    plugins: Optional[List[GzPluginSpec]] = None
