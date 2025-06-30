"""Module for the node_list action."""

from copy import deepcopy
from typing import List, Optional, Mapping, Any

from launch import LaunchDescriptionEntity
from launch.action import Action
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.frontend import Entity, expose_action, Parser
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitutions import PathJoinSubstitution
import launch.utilities

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import PushROSNamespace

@expose_action('node_list')
class NodeList(Action):
    """Action that executes a list of ROS nodes."""

    SEPARATOR = ' '
    
    def __init__(
        self,
        namespaces: SomeSubstitutionsType = None,
        *,
        function,
        **kwargs
    ) -> None:
        """
        Construct a node_list action.

        All arguments are forwarded to `virelex.launch.node_list.launch.py`.

        :param: namespaces List of node namespaces.
        """

        super().__init__(**kwargs)
        self._namespaces = launch.utilities.normalize_to_list_of_substitutions(namespaces)
        self._function = function
        self._logger = get_logger(__name__)

    @classmethod
    def parse(cls, entity: Entity, parser: Parser):
        """Parse node_list launchfile tag contents."""
        _, kwargs = super().parse(entity, parser)

        namespaces = entity.get_attr(
            name='namespaces',
            data_type=str,
            optional=False,
        )
        namespaces = parser.parse_substitution(namespaces)

        parsed_children = [parser.parse_action(e) for e in entity.children]
        def per_namespace(namespace: str) -> List[LaunchDescriptionEntity]:
            return [
                GroupAction(
                    actions = [
                        PushROSNamespace(namespace),
                        *deepcopy(parsed_children),
                    ]
                ),
            ]

        kwargs['function']   = per_namespace
        kwargs['namespaces'] = namespaces

        return cls, kwargs
    
    def execute(self, context: LaunchContext) -> List[Action]:
        """Execute the node_list action."""

        namespaces = launch.utilities.perform_substitutions(context, self._namespaces)
        self._logger.debug(f'namespaces={namespaces}')

        namespace_list = namespaces.strip().split(self.SEPARATOR)
        if not namespace_list:
            self._logger.warning('no input values: will not execute')

        entities = []
        for namespace in namespace_list:
            self._logger.debug(f'namespace: {namespace}')
            if not namespace:
                self._logger.warning('empty namespace found, skipping')
            else:
                ns_entities = self._function(namespace)
                entities.extend(ns_entities)

        return entities
        
        # node_list_description = IncludeLaunchDescription(
        #     launch_description_source=PythonLaunchDescriptionSource(
        #         [
        #             PathJoinSubstitution(
        #                 [FindPackageShare('virelex'),
        #                  'launch',
        #                  'node_list.launch.py']
        #             )
        #         ]
        #     ),
        #     launch_arguments=[
        #         ('namespaces', self._namespaces)
        #     ]
        # )

        # return [node_list_description]
        
