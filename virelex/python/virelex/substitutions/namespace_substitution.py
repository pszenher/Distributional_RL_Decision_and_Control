from typing import Optional, Sequence, Text

import launch

@launch.frontend.expose_substitution('ns')
class NamespaceSubstitution(launch.Substitution):
    """Substitution for the currently-scoped namespace stack."""

    def __init__(self, level: Optional[launch.SomeSubstitutionsType]) -> None:
        super().__init__()

        self._level = launch.utilities.normalize_to_list_of_substitutions(level) if level is not None else None
        self._logger = launch.logging.get_logger(__name__)

    @classmethod
    def parse(cls, data: Sequence[launch.SomeSubstitutionsType]):
        # kwargs = {'parent': data[0]
        if len(data) not in (0,1):
            raise ValueError(f'{cls.__name__} substitution expects 0 or 1 argument')
        kwargs = {}
        if data:
            kwargs['level'] = data[0]
            
        return cls, kwargs

    def perform(self, context: launch.LaunchContext) -> Text:

        namespace = context.launch_configurations['ros_namespace']

        if self._level is not None:
            level = launch.utilities.perform_substitutions(context, self._level)

            try:
                level_num = int(level)
                if level_num < 0:
                    raise ValueError()
            except ValueError:
                raise ValueError(f'{cls.__name__} expects integer-formatted argument, got {level}')
            
            self._logger.debug(f'level={level_num}')
            namespace_comp = list(filter(None, namespace.split('/')))
            try:
                # FIXME: this is a visual nightmare, refactor
                namespace = namespace_comp[-(level_num+1)]
            except IndexError:
                raise ValueError(
                    f'{cls.__name__} attempted namespace lookup at level {level_num}, '
                    f'but only {len(namespace_comp)} levels found'
                )

        return namespace
