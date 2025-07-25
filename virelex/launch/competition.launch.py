# Copyright 2021 Open Source Robotics Foundation, Inc.
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


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource,PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

import numpy as np
from copy import deepcopy

import json

import vrx_gz.launch
from vrx_gz.model import Model


def launch(context, *args, **kwargs):
    config_file = LaunchConfiguration('config_file').perform(context)
    world_name = LaunchConfiguration('world').perform(context)
    sim_mode = LaunchConfiguration('sim_mode').perform(context)
    bridge_competition_topics = LaunchConfiguration(
        'bridge_competition_topics').perform(context).lower() == 'true'
    robot = LaunchConfiguration('robot').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    robot_urdf = LaunchConfiguration('urdf').perform(context)
    gz_paused = LaunchConfiguration('paused').perform(context).lower() == 'true'
    competition_mode = LaunchConfiguration('competition_mode').perform(context).lower() == 'true'
    extra_gz_args = LaunchConfiguration('extra_gz_args').perform(context)
    init_poses_str = LaunchConfiguration('init_poses').perform(context)
    goals_str = LaunchConfiguration('goals').perform(context)
    buoy_poses_str = LaunchConfiguration('buoy_poses').perform(context)
    method = LaunchConfiguration('method').perform(context)
    agent_type = LaunchConfiguration('agent_type').perform(context)
    model_path = LaunchConfiguration('model_path').perform(context)

    init_poses = [[float(p.split(',')[0]), float(p.split(',')[1]), 0.0, 0.0, 0.0, float(p.split(',')[2])] 
                  for p in init_poses_str.split(';')]
    goals = [[float(g) for g in goal.split(',')] for goal in goals_str.split(';')]

    launch_processes = []


    models = []
    if config_file and config_file != '':
        with open(config_file, 'r') as stream:
            models = Model.FromConfig(stream)
    else:
        robot_names = []
        for i,pose in enumerate(init_poses):
            name = f'wamv{i+1}'
            robot_names.append(name)
            model = Model(name,'wam-v',pose)
            models.append(model)

        robot_names_arg = DeclareLaunchArgument(
            'robot_names',
            default_value=' '.join(robot_names),
            description='Space-separated list of robot names'
        )
        robot_goals = [','.join(str(p) for p in goal) for goal in goals]
        robot_goals_arg = DeclareLaunchArgument(
            'robot_goals',
            default_value=' '.join(robot_goals)
        )
        buoy_poses_arg = DeclareLaunchArgument(
            'buoy_poses',
            default_value=buoy_poses_str
        )
        method_arg = DeclareLaunchArgument(
            'method',
            default_value=method
        )
        model_path_arg = DeclareLaunchArgument(
            'model_path',
            default_value=model_path
        )
        agent_type_arg = DeclareLaunchArgument(
            'agent_type',
            default_value=agent_type
        )

        robots_dict = [
            { 'name': name, 'goal': goal }
            for (name,goal) in zip(robot_names, robot_goals)
        ]

        robots_struct_arg = DeclareLaunchArgument(
            'robots',
            default_value=';'.join(json.dumps(d) for d in robots_dict),
        )

        virelex_dir = get_package_share_directory('virelex')
        world_name, ext = os.path.splitext(world_name)

        launch_processes.append(robot_names_arg)
        launch_processes.append(robot_goals_arg)
        launch_processes.append(buoy_poses_arg)
        launch_processes.append(method_arg)
        launch_processes.append(model_path_arg)
        launch_processes.append(agent_type_arg)
        launch_processes.append(robots_struct_arg)

        launch_processes.append(IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(f'{virelex_dir}/launch/unified.launch.xml'),
            launch_arguments={
                'world_name': world_name,
                
                'robots': LaunchConfiguration('robots'),
                
                'obstacles': ';'.join(
                    str(params) for params in [
                        {
		            'sdf_file': 'robotx_light_buoy_unlit',
		            'x': -500.0,
		            'y': 225.0
		        },
                        *[
                            {
                                'sdf_file': 'robotx_light_buoy_unlit',
                                'x': -500.0 + offset,
                                'y':  225.0 + offset,
                            }
                            for offset in range(5,50,10)
                        ],
		        # {
		        #     'sdf_file': 'robotx_light_buoy_unlit',
		        #     'x': -485.0,
		        #     'y': 240.0
		        # },
		        # {
		        #     'sdf_file': 'robotx_light_buoy_unlit',
		        #     'x': -450.0,
		        #     'y': 275.0
		        # },
		    ]
                ),
                'method':     LaunchConfiguration('method'),
                'agent_type': LaunchConfiguration('agent_type'),
                'model_path': LaunchConfiguration('model_path'),
            }.items()
        ))

    # FIXME: this short-form doesn't register the event handler that the
    #        experiments rely on, refactor...
    launch_processes.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch'),
                                           '/gz_sim.launch.py']),
            launch_arguments={'gz_args': ' '.join(
                (['-r'] if not gz_paused else []) +
                (['-s'] if headless      else []) +
                # ['-v4'] +
                # ['--physics-engine', 'gz-physics-bullet-plugin'] +
                [extra_gz_args] +
                # Set bullet physics engine (must be done here, SDF `<physics type="...">` is ignored)
                [f'{world_name}.sdf']
            )}.items())
    )
    
    world_name_base = os.path.basename(world_name)
    launch_processes.extend(vrx_gz.launch.spawn(sim_mode, world_name_base, models, robot))

    if (sim_mode == 'bridge' or sim_mode == 'full') and bridge_competition_topics:
        launch_processes.extend(vrx_gz.launch.competition_bridges(world_name_base, competition_mode))

    return launch_processes


def generate_launch_description():
    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'world',
            default_value='sydney_regatta',
            description='Name of world'),
        DeclareLaunchArgument(
            'sim_mode',
            default_value='full',
            description='Simulation mode: "full", "sim", "bridge".'
                        'full: spawns robot and launch ros_gz bridges, '
                        'sim: spawns robot only, '
                        'bridge: launch ros_gz bridges only.'),
        DeclareLaunchArgument(
            'bridge_competition_topics',
            default_value='True',
            description='True to bridge competition topics, False to disable bridge.'),
        DeclareLaunchArgument(
            'config_file',
            default_value='',
            description='YAML configuration file to spawn'),
        DeclareLaunchArgument(
            'robot',
            default_value='',
            description='Name of robot to spawn if specified. '
                        'This must match one of the robots in the config_file'),
        DeclareLaunchArgument(
            'headless',
            default_value='False',
            description='True to run simulation headless (no GUI). '),
        DeclareLaunchArgument(
            'urdf',
            default_value='',
            description='URDF file of the wam-v model. '),
        DeclareLaunchArgument(
            'paused',
            default_value='False',
            description='True to start the simulation paused. '),
        DeclareLaunchArgument(
            'competition_mode',
            default_value='False',
            description='True to disable debug topics. '),
        DeclareLaunchArgument(
            'extra_gz_args',
            default_value='',
            description='Additional arguments to be passed to gz sim. '),
        DeclareLaunchArgument(
            'method',
            default_value='',
            description='method of action planner'
        ),
        DeclareLaunchArgument(
            'agent_type',
            default_value='',
            description='RL agent type of action planner'
        ),
        DeclareLaunchArgument(
            'model_path',
            default_value='',
            description='RL agent model of action planner'
        ),
        DeclareLaunchArgument(
            'init_poses',
            default_value='',
            description='Initial poses of robots in the format: "x_1,y_1,theta_1;..." '
        ),
        DeclareLaunchArgument(
            'goals',
            default_value='',
            description='Goals of robots in the format: "x_1,y_1;..."'
        ),
        DeclareLaunchArgument(
            'buoy_poses',
            default_value='',
            description='Positions and radii of bouys in the format: "x_1,y_1,r_1;..."'
        ),
        OpaqueFunction(function=launch),
    ])
