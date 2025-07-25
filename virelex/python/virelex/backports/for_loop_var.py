# Copyright 2025 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# This is a copy of the new ForEachVar substitution from the `kilted` release of ROS2:
#     https://github.com/ros2/launch/blob/72f3ac62635cf121af577081bcfa66ab81f6a975/launch/launch/substitutions/for_loop_var.py

"""Module for the ForLoopIndex substitution."""

from typing import List
from typing import Optional
from typing import Sequence
from typing import Text

import launch
from launch.frontend import expose_substitution
from launch import LaunchContext
from launch.logging import get_logger
from launch import SomeSubstitutionsType
from launch import Substitution
from launch.utilities import perform_substitutions
from launch.utilities import normalize_to_list_of_substitutions

@expose_substitution('for-var')
class ForEachVar(Substitution):
    """Substitution for a :class:`launch.actions.ForEach` iteration variable value."""

    def __init__(
        self,
        name: SomeSubstitutionsType,
        *,
        default_value: Optional[SomeSubstitutionsType] = None,
    ) -> None:
        """
        Create a ForEachVar.

        :param name: the name of the :class:`launch.actions.ForEach` iteration variable
        :param default_value: a default value for the variable if a value is not available for a
            given iteration
        """
        super().__init__()

        self._name = normalize_to_list_of_substitutions(name)
        self._default_value = (
            normalize_to_list_of_substitutions(default_value)
            if default_value is not None
            else None
        )
        self._logger = get_logger(__name__)

    @property
    def name(self) -> List[Substitution]:
        return self._name

    @property
    def default_value(self) -> Optional[List[Substitution]]:
        return self._default_value

    def describe(self) -> Text:
        return (
            self.__class__.__name__ +
            '(' +
            f"name={' + '.join([sub.describe() for sub in self._name])}" +
            ', ' +
            f"default_value={' + '.join([sub.describe() for sub in self._default_value or []])}" +
            ')'
        )

    @classmethod
    def parse(cls, data: Sequence[SomeSubstitutionsType]):
        if not any(len(data) == length for length in (1, 2)):
            raise ValueError(f'{cls.__name__} substitution expects 1 or 2 arguments')
        kwargs = {}
        kwargs['name'] = data[0]
        if len(data) == 2:
            kwargs['default_value'] = data[1]
        return cls, kwargs

    def perform(self, context: LaunchContext) -> Text:
        name = perform_substitutions(context, self._name)
        self._logger.debug(f'name={name}')
        local_arg_name = self.get_local_arg_name(name)
        if not hasattr(context.locals, local_arg_name):
            if self._default_value is None:
                raise RuntimeError(
                    f'No value available for {self.__class__.__name__} '
                    f"'{name}' and no default value provided"
                )
            value = perform_substitutions(context, self._default_value)
        else:
            value = getattr(context.locals, local_arg_name)
        self._logger.debug(f'{name}={value}')
        return value

    @classmethod
    def get_local_arg_name(cls, name: str) -> str:
        # Prevent local variable collisions
        return f'ForEachVar__{name}'


@expose_substitution('index')
class ForLoopIndex(ForEachVar):
    """Substitution for a :class:`launch.actions.ForLoop` iteration index value."""

    def __init__(
        self,
        name: SomeSubstitutionsType,
        **kwargs,
    ) -> None:
        """
        Create a ForLoopIndex.

        :param name: the name of the :class:`launch.actions.ForLoop` index which this substitution
            is part of
        """
        super().__init__(name)
