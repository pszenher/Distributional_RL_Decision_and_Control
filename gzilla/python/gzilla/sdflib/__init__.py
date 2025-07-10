"""Wrapper class hierarchy for SDFormat pybind11 module."""

import xml.etree.ElementTree as ET
from collections.abc import Iterator, Sequence
from itertools import repeat
from pathlib import Path, PurePath
from types import UnionType
from typing import Any, Protocol, Self
from urllib.parse import urlparse

# This is a binary module, mypy can't see it without stubs
import sdformat14 as sdf  # type: ignore [import-not-found]

import gzilla.sdflib.util

type SdfWorldCarrier = "SdfRoot"
type SdfModelCarrier = "SdfRoot" | "SdfWorld" | "SdfModel"
type SdfSensorCarrier = "SdfLink" | "SdfJoint"

# TODO(pszenher): implement wrappers for the rest of these
type SdfPluginCarrier = (
    "SdfWorld" | "SdfModel" | "SdfSensor" | sdf.Visual | sdf.Projector | sdf.Gui
)


class HasParent(Protocol):
    """An object with some notion of having a single "parent".

    Enforces implementation of a `parent` property method by subclasses, which returns
    the parent object or `None`.
    """

    @property
    def parent(self) -> "HasParent | None":
        """The parent of the current object, or `None`."""
        ...


class SdfMixin[T: HasParent | None]:
    """Mixin class for SDF pybind object wrapper classes."""

    _xml_tag: str

    def __init__(self, *args: Sequence[Any], parent: T | None = None) -> None:
        """Initialize SDF mixin class.

        Expects to be used immediately before a binary-module `sdf.*`
        type in the class inheritance chain, such that `super()`
        resolves to an `sdf.*`-family type.

        """
        super().__init__(*args)
        self._parent = parent

    @property
    def parent(self) -> T | None:
        """The parent of the current object, or `None`."""
        return self._parent

    @property
    def xml_tag(self) -> str:
        """String content of the XML-tag which encloses this element type."""
        return self._xml_tag

    def ancestors(self) -> Iterator[HasParent]:
        """Iterate over node ancestry, starting at current node parent.

        Yields only non-None ancestors;  returns upon encountering None-parent.
        """
        cur_node: HasParent | None = self.parent
        while cur_node is not None:
            yield cur_node
            cur_node = cur_node.parent

    def ancestors_up_to(self, sdf_type: type | UnionType) -> Iterator[HasParent]:
        """Iterate over node ancestry, starting at current node parent.

        Yields only non-None ancestors; returns upon encountering
        None-parent, or a parent contained in type `sdf_type`.
        """
        cur_node: HasParent | None = self.parent
        while cur_node:
            if isinstance(cur_node, sdf_type):
                break
            yield cur_node
            cur_node = cur_node.parent


class SdfRoot(SdfMixin[None], sdf.Root):
    """An SDF <sdf> element."""

    _xml_tag: str = "sdf"

    def entity(self) -> "SdfModel | sdf.Light | None":
        """Return contained SDF entity if it exists, else `None`.

        The SDF spec allows for a single <model>, <light>, or <actor>
        tag in the root model.
        """
        if model := self.model():
            return SdfModel(model, parent=self)
        if light := self.light():
            return light
        # NOTE: this is where an actor check would go, if the pybind
        #       api actually had any methods for them bound, i.e.:
        #
        # > if (actor := self.actor()):
        # >    return actor
        #
        return None

    def worlds(self) -> Iterator["SdfWorld"]:
        """Return iterator over (immediate) child <world> entities."""
        for world_idx in range(self.world_count()):
            yield SdfWorld(self.world_by_index(world_idx))

    def world_models(self) -> Iterator["SdfModel"]:
        """Return iterator over <model> entities worlds."""
        for world in self.worlds():
            yield from world.models()

    def world_links(self) -> Iterator["SdfLink"]:
        """Return iterator over <link> entities attached to models in child worlds."""
        for model in self.models():
            yield from model.links()

    def world_joints(self) -> Iterator["SdfJoint"]:
        """Return iterator over <link> entities attached to models in child worlds."""
        for model in self.models():
            yield from model.joints()

    def world_sensors(self) -> Iterator["SdfSensor"]:
        """Return iterator over <sensor> entities attached to models in child worlds."""
        for model in self.world_models():
            yield from model.sensors()

    @classmethod
    def from_sdf_file(
        cls,
        sdf_filename: str,
        parser_config: sdf.ParserConfig | None = None,
    ) -> Self:
        """Resolve SDF file by filename and return SdfRoot object."""
        with Path(sdf_filename).open() as f:
            sdf_string = f.read()
        return cls.from_sdf_string(sdf_string, parser_config)

    @classmethod
    def from_sdf_string(
        cls,
        sdf_string: str,
        parser_config: "SdfParserConfig | None" = None,
    ) -> Self:
        """Parse SdfRoot object from xml-encoded SDF string."""
        if not parser_config:
            parser_config = SdfParserConfig()

        # TODO(pszenher): check the initial contents of the
        # `parser_config` object first, i.e.:
        #
        # > uri_map : Mapping[str, List[str]] = parser_config.uri_path_map

        sdf_tree = ET.fromstring(sdf_string)  # noqa: S314

        # Find all <uri> tags in SDF file, add mappings to parser config
        for uri_elt in sdf_tree.findall(".//uri"):
            if uri_elt.text:
                parser_config.handle_uri(uri_elt.text)

        # Load SDF string into SDFormat parser, using uri-aware parser config
        root = SdfRoot()
        root.load_sdf_string(sdf_string, parser_config)
        return root


class SdfPlugin(SdfMixin[SdfPluginCarrier], sdf.Plugin):
    """An SDF <plugin> element."""

    _xml_tag: str = "plugin"

    @property
    def name(self) -> str:
        """Return `name` attribute of plugin."""
        return super().name()

    @property
    def filename(self) -> str:
        """Return `filename` attribute of plugin."""
        return super().filename()

    @property
    def xml(self) -> str:
        """Return raw SDF <plugin> element tags and body as xml-formatted string."""
        return self.__str__()


class SdfSensor(SdfMixin[SdfSensorCarrier], sdf.Sensor):
    """An SDF <sensor> element."""

    _xml_tag: str = "sensor"

    @property
    def name(self) -> str:
        """Return `name` attribute of sensor."""
        return super().name()

    def plugins(self) -> Iterator[SdfPlugin]:
        """Return iterator over <plugin> entities directly attached to sensor."""
        return map(
            lambda plgn, parent: SdfPlugin(plgn, parent=parent),
            super().plugins(),
            repeat(self),
        )


class SdfJoint(SdfMixin["SdfModel"], sdf.Joint):
    """An SDF <joint> element."""

    _xml_tag: str = "joint"

    def sensors(self) -> Iterator[SdfSensor]:
        """Return iterator over <sensor> entities directly attached to joint."""
        for sensor_idx in range(self.sensor_count()):
            yield SdfSensor(self.sensor_by_index(sensor_idx), parent=self)


class SdfLink(SdfMixin["SdfModel"], sdf.Link):
    """An SDF <link> element."""

    _xml_tag: str = "link"

    def sensors(self) -> Iterator[SdfSensor]:
        """Return iterator over <sensor> entities directly attached to link."""
        for sensor_idx in range(self.sensor_count()):
            yield SdfSensor(self.sensor_by_index(sensor_idx), parent=self)


class SdfModel(SdfMixin[SdfModelCarrier], sdf.Model):
    """An SDF <model> element."""

    _xml_tag: str = "model"

    def plugins(self) -> Iterator[SdfPlugin]:
        """Return iterator over <plugin> entities directly attached to model."""
        return map(
            lambda plgn, parent: SdfPlugin(plgn, parent=parent),
            super().plugins(),
            repeat(self),
        )

    def links(self) -> Iterator[SdfLink]:
        """Return iterator over <link> entities attached to model."""
        for link_idx in range(self.link_count()):
            yield SdfLink(self.link_by_index(link_idx))

    def joints(self) -> Iterator[SdfLink]:
        """Return iterator over <joint> entities attached to model."""
        for joint_idx in range(self.joint_count()):
            yield SdfJoint(self.joint_by_index(joint_idx))

    def sensors(self) -> Iterator[SdfSensor]:
        """Return iterator over <sensor> entities attached to model."""
        for link in self.links():
            for sensor in link.sensors():
                yield sensor
        for joint in self.joints():
            for sensor in joint.sensors():
                yield sensor

    @classmethod
    def from_sdf_root(cls, root: SdfRoot) -> Self:
        """Return top-level SdfModel entity from SdfRoot object.

        Raises ValueError if SdfRoot entity does not contain a
        top-level <model>.
        """
        match entity := root.entity():
            case gzilla.sdflib.SdfModel():
                return entity
            case _:
                msg = (
                    f"Provided SDFRoot does not contain a root-anchored <model> "
                    f"entity;  found:  {entity}"
                )
                raise ValueError(msg)

    @classmethod
    def from_sdf_string(
        cls,
        sdf_string: str,
        parser_config: sdf.ParserConfig | None = None,
    ) -> Self:
        """Return top-level SdfModel entity from xml-encoded SDF string.

        Raises ValueError if SdfRoot entity does not contain a
        top-level <model>.
        """
        root = SdfRoot.from_sdf_string(sdf_string, parser_config)
        return cls.from_sdf_root(root)

    @classmethod
    def from_sdf_file(
        cls,
        sdf_filename: str,
        parser_config: sdf.ParserConfig | None = None,
    ) -> Self:
        """Return top-level SdfModel entity from SDF filename.

        Raises ValueError if SdfRoot entity does not contain a
        top-level <model>.
        """
        root = SdfRoot.from_sdf_file(sdf_filename, parser_config)
        return cls.from_sdf_root(root)


class SdfWorld(SdfMixin[SdfWorldCarrier], sdf.World):
    """An SDF <world> element."""

    _xml_tag: str = "world"

    def plugins(self) -> Iterator[SdfPlugin]:
        """Return iterator over <plugin> entities directly attached to the world."""
        return map(
            lambda plgn, parent: SdfPlugin(plgn, parent=parent),
            super().plugins(),
            repeat(self),
        )

    def models(self) -> Iterator[SdfModel]:
        """Return iterator over <model> entities directly attached to the world."""
        for model_idx in range(self.model_count()):
            yield SdfModel(self.model_by_index(model_idx), parent=self)

    @classmethod
    def from_sdf_string(cls, sdf_string: str) -> Self:
        """Return SdfWorld object from xml-encoded SDF string."""
        root = SdfRoot.from_sdf_string(sdf_string)

        match list(root.worlds()):
            case [world]:
                return world
            case []:
                msg = (
                    "Provided SDF string does not contain any <world> entities, "
                    "expected exactly 1"
                )
                raise ValueError(msg)
            case [*worlds]:
                msg = (
                    f"Provided SDF string contains {len(worlds)} <world> entities, "
                    f"expected exactly 1"
                )
                raise ValueError(msg)
            case val:
                raise TypeError(
                    "Expected list-type value for root SDF worlds, received: "
                    + str(val)
                )


class SdfParserConfig(sdf.ParserConfig):
    """Configuration for SDF parser.

    Used by parser for processing raw XML strings/files into SDF
    objects, such as `SdfRoot.load_sdf_string`
    """

    @staticmethod
    def local_uri_path(path_string: str) -> str:
        """Resolve relative model path to abspath.

        Raises `FileNotFoundError` if model not found in search paths.
        """
        # TODO(pszenher): should probably drop this method entirely,
        # call the util func directly inline
        return gzilla.sdflib.util.find_gz_resource(path_string)

    @staticmethod
    def fuel_uri_path(netloc: str, path: str) -> str:
        """Return expected local cache abspath for Gazebo Fuel URI path."""
        # TODO(pszenher): refactor this function into `util` module

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
        fuel_dir = Path.home() / ".gz" / "fuel"

        # Unpack uri path parts, append relative path components to fuel cachedir
        [_, fuel_vers, *path_components] = PurePath(path).parts
        model_path = fuel_dir / netloc / PurePath(*path_components)

        # Select the newest version of the model from fuel cachedir
        # TODO(pszenher): not clear if fuel cachedir versions are always integer values
        try:
            [model_vers, *_] = sorted(
                [int(vers := path.name) for path in model_path.iterdir()]
            )
        except ValueError as err:
            msg = (
                f"Unexpected model version directory encounted in fuel cachedir:  "
                f"{model_path / vers}"
            )
            raise RuntimeError(msg) from err

        # Append newest model version directory to cache path
        model_path_versioned = model_path / str(model_vers)
        return model_path_versioned.as_posix()

    # TODO(pszenher): while the below does handle uri resolution for
    #        all encountered uri's in the primary sdf string/file, it
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
        """Handle URI string to be dereferenced for SDF model instantiation.

        Selects between local and remote uri handler methods depending
        on format of provided `uri_string`.
        """
        uri = urlparse(uri_string)
        match uri.scheme:
            case "https":
                # Insert uri path mapping into sdf parser config
                self.add_uri_path(uri_string, self.fuel_uri_path(uri.netloc, uri.path))
            case "model" | "file" | "":
                # Re-append netloc to uri path if it is non-empty, handle as local path
                self.add_uri_path(
                    uri_string, self.local_uri_path(uri.netloc + uri.path)
                )
            case "package":
                msg = (
                    f"ROS `package` SDF include URIs are not currently supported: "
                    f"{uri_string}"
                )
                raise NotImplementedError(msg)
            case _:
                msg = (
                    f"Unrecognized URI scheme '{uri.scheme}' in SDF input: {uri_string}"
                )
                raise ValueError(msg)
