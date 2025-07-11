""""""

import xml.etree.ElementTree as ET
from collections.abc import Iterator, Sequence
from dataclasses import dataclass

import gzilla.codegen.gz_plugin_spec as gz_plugin_spec_schema
import gzilla.codegen.gz_sensor_spec as gz_sensor_spec_schema
from gzilla.codegen import gz_topic_spec
from gzilla.codegen.gz_topic_spec import Direction, PrefixType
from gzilla.sdflib import SdfModel, SdfPlugin, SdfSensor, SdfWorld


@dataclass
class GzEntitySpawnerConfig:
    world_name: str | None = None
    entity_name: str | None = None


@dataclass
class GzRuntimeConfig:
    world_name: str
    model_name: str
    sdf: SdfSensor | SdfPlugin

    def model_scoped_prefix(self) -> str:
        return f"/model/{self.model_name}"

    def scoped_prefix(self) -> str:
        # FIXME(pszenher): this might be a bit too much mustard for a one-line
        #        return statement; refactor for clarity
        return "/".join(
            (
                "/world",
                self.world_name,
                "model",
                self.model_name,
                *reversed(
                    [
                        # TODO(pszenher): refine type spec of sdflib
                        #     to allow sdf_elt.name to typecheck here...
                        f"{sdf_elt.xml_tag}/{sdf_elt.name()}"  # type: ignore [attr-defined]
                        for sdf_elt in self.sdf.ancestors_up_to(SdfModel | SdfWorld)
                    ]
                ),
                # FIXME(pszenher): this should probably be wrapped into the reversed
                #        iterator above; maybe define new method that iterates to
                #        root but includes current element?
                #
                self.sdf.xml_tag,
                # TODO(pszenher): this technically typechecks, but might be better
                #       to have a `Protocol` class definied that captures
                #       expected behavior of SdfSensor and SdfPlugin
                self.sdf.name,
            )
        )


@dataclass
class GzTopic:
    name: str
    spec: "GzTopicSpec"

    @property
    def msg_type(self) -> str:
        return self.spec.msg_type

    @property
    def direction(self) -> Direction:
        return self.spec.direction


class GzTopicNameSpec(gz_topic_spec.GzTopicNameSpec):
    def final_name(self, conf: GzRuntimeConfig) -> str:
        match conf.sdf:
            case SdfSensor():
                if explicit_topic_name := conf.sdf.topic():
                    return explicit_topic_name + self.suffix
            case SdfPlugin():
                if self.override:
                    sdf_plugin_tree = ET.fromstring(conf.sdf.xml)  # noqa: S314
                    overrides = sdf_plugin_tree.findall(f"{self.override}")
                    # FIXME(pszenher): refactor, this is too deeply nested,
                    #        shuould use destructuring bind instead of
                    #        list index, warn on multiples, etc.
                    if overrides and overrides[0].text:
                        return overrides[0].text.strip() + (self.suffix or "")
        return self.default_name(conf)

    def default_name(self, conf: GzRuntimeConfig | None = None) -> str:
        match self.prefix_type:
            case PrefixType.SCOPED:
                if not conf:
                    msg = (
                        "Expected runtime configuration parameters for scoped prefix "
                        "topic determination; received:  "
                        f"runtime_config={conf}"
                    )
                    raise ValueError(msg)
                return self.scoped_name(conf)
            case PrefixType.MODEL_SCOPED:
                if not conf:
                    msg = (
                        f"Expected runtime configuration parameters for model-scoped "
                        f"prefix topic determination; received:  runtime_config={conf}"
                    )
                    raise ValueError(msg)
                return self.model_scoped_name(conf)
            case PrefixType.ABSOLUTE:
                return self.base_name()
            case PrefixType.CUSTOM:
                msg = (
                    "Received topic name request for topic-generating SDF element "
                    f"with prefix type 'custom', which is not yet implemented: {self}"
                )
                raise NotImplementedError(msg)

    def scoped_name(self, conf: GzRuntimeConfig) -> str:
        return conf.scoped_prefix() + self.base_name()

    def model_scoped_name(self, conf: GzRuntimeConfig) -> str:
        return conf.model_scoped_prefix() + self.base_name()

    def base_name(self) -> str:
        return self.base + (self.suffix or "")


class GzTopicSpec(gz_topic_spec.GzTopicSpec):
    topic: GzTopicNameSpec

    def final_msg_topic(self, runtime_config: GzRuntimeConfig) -> GzTopic:
        return GzTopic(
            name=self.final_name(runtime_config),
            spec=self,
        )

    def final_name(
        self,
        runtime_config: GzRuntimeConfig,
    ) -> str:
        return self.topic.final_name(runtime_config)

    def default_name(self, runtime_config: GzRuntimeConfig | None = None) -> str:
        return self.topic.default_name(runtime_config)


class HasTopicsMixin:
    topics: Sequence[GzTopicSpec]

    def final_msg_topics(
        self,
        runtime_config,
    ) -> Iterator[GzTopic]:
        for topic_spec in self.topics:
            try:
                yield topic_spec.final_msg_topic(runtime_config)
            except NotImplementedError as err:
                # TODO(pszenher): use real logging here
                print(f"WARN: found unimplemented message topic: {err!s}")

    def default_topics(
        self, runtime_config: GzRuntimeConfig | None = None
    ) -> Iterator[str]:
        for topic_spec in self.topics:
            yield topic_spec.default_name(runtime_config)


class GzSensorSpec(gz_sensor_spec_schema.GzSensorSpec, HasTopicsMixin):
    topics: Sequence[GzTopicSpec]


class GzPluginSpec(gz_plugin_spec_schema.GzPluginSpec, HasTopicsMixin):
    topics: Sequence[GzTopicSpec]
