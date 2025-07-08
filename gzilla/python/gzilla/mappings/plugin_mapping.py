from pathlib import PurePath
from pydantic import PrivateAttr

from gzilla.mappings.specs import GzTopicSpec, GzPluginSpec
import gzilla.codegen.gz_plugin_map as gz_plugin_map

class GzPluginMap(gz_plugin_map.GzPluginMap):

    plugins: list[GzPluginSpec]

    _plugin_dict: dict[str,GzPluginSpec] = PrivateAttr()

    def __init__(self, **data):
        super().__init__(**data)
        self._plugin_dict = self._generate_plugin_dict(self.plugins)

    @staticmethod
    def _generate_plugin_dict(plugins: list[GzPluginSpec]) -> dict[str,GzPluginSpec]:
        plugin_dict = dict()
        dups = {
            p.plugin for p in plugins
            if p.plugin in plugin_dict
            or plugin_dict.update({p.plugin: p})
        }
        if dups:
            raise ValueError(
                f"Duplicate plugin names found in plugin map: '{dups}'"
            )
        return plugin_dict

    def lookup(self, name: str) -> GzPluginSpec | None:
        return self._plugin_dict.get(
            # Grab basename if this is an absolute file path, strip '.so' (or any other) extension
            PurePath(name).stem
        )
