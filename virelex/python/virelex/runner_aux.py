from pprint import pp

from typing import Optional, Self
from pathlib import Path
from enum import StrEnum, auto
from copy import deepcopy
import json
import numpy as np
import os
import shutil
import xml.etree.ElementTree as ET

from datetime import datetime

import rclpy

import rfarl.envs.marinenav

from virelex.experiment import ExperimentManager 

from ament_index_python.packages import get_package_share_directory

def read_exp_setup(ep_data):

    test_env_center = [27.5,27.5]
    vrx_center = [-480.0,240.0]
    
    init_poses = ""
    goals = ""
    for i in range(len(ep_data["robots"]["start"])):
        start = ep_data["robots"]["start"][i]
        goal = ep_data["robots"]["goal"][i]
        init_theta = ep_data["robots"]["init_theta"][i]

        start_x = start[0] - test_env_center[0] + vrx_center[0]
        start_y = start[1] - test_env_center[1] + vrx_center[1]

        goal_x = goal[0] - test_env_center[0] + vrx_center[0]
        goal_y = goal[1] - test_env_center[1] + vrx_center[1]

        pose = str(start_x)+","+str(start_y)+","+str(init_theta)+";"
        goal = str(goal_x)+","+str(goal_y)+";"

        init_poses += pose
        goals += goal

    return init_poses[:-1], goals[:-1]

class MethodType(StrEnum):
    RL  = auto()
    MPC = auto()
    APF = auto()

class RLAgentType(StrEnum):
    AC_IQN  = auto()
    IQN     = auto()
    DQN     = auto()
    DDPG    = auto()
    SAC     = auto()
    Rainbow = auto()

class PlannerMethod:
    method: MethodType
    agent_type: Optional[RLAgentType] = None
    model_path: Optional[Path] = None

    def __init__(
            self,
            method: MethodType,
            agent_type: Optional[RLAgentType] = None,
            model_path: Optional[Path] = None,
    ) -> Self:
        self.method = MethodType(method)
        match self.method:
            case MethodType.RL:
                if not (agent_type and model_path):
                    raise ValueError(
                        f'Selected RL method, but agent_type ({agent_type}) '
                        f'and/or model_path ({model_path}) unset'
                    )
                self.agent_type = agent_type
                self.model_path = model_path
            case _:
                if agent_type or model_path:
                    raise ValueError(
                        f'Selected non-RL method ({method}), but agent_type ({agent_type}) '
                        f'and/or model_path ({model_path}) set'
                    )
                
if __name__ == '__main__':

    virelex_share_dir = Path( get_package_share_directory("virelex") )
    
    METHODS = {
        "RL_AC-IQN" : PlannerMethod(
            method     = MethodType.RL,
            agent_type = RLAgentType.AC_IQN,
            model_path = virelex_share_dir / "trained/traced_AC_IQN_model.pt",
        ),
        "RL_DDPG" : PlannerMethod(
            method     = MethodType.RL,
            agent_type = RLAgentType.DDPG,
            model_path = virelex_share_dir / "trained/traced_DDPG_model.pt",
        ),
        "RL_DQN" : PlannerMethod(
            method     = MethodType.RL,
            agent_type = RLAgentType.DQN,
            model_path = virelex_share_dir / "trained/traced_DQN_model.pt",
        ),
        "RL_IQN" : PlannerMethod(
            method     = MethodType.RL,
            agent_type = RLAgentType.IQN,
            model_path = virelex_share_dir / "trained/traced_IQN_model.pt",
        ),
        "RL_Rainbow" : PlannerMethod(
            method     = MethodType.RL,
            agent_type = RLAgentType.Rainbow,
            model_path = virelex_share_dir / "trained/traced_Rainbow_model.pt",
        ),
        "RL_SAC" : PlannerMethod(
            method     = MethodType.RL,
            agent_type = RLAgentType.SAC,
            model_path = virelex_share_dir / "trained/traced_SAC_model.pt",
        ),
        "MPC" : PlannerMethod( method = MethodType.MPC ),
        "APF" : PlannerMethod( method = MethodType.APF ),
    }
    
    planner = METHODS["RL_AC-IQN"]

    test_env = rfarl.envs.marinenav.MarineNavEnv3(seed = 0)
    test_env.num_robots = 5
    test_env.num_cores = 0
    test_env.num_obs = 0
    test_env.min_start_goal_dis = 40.0
    test_env.reset()

    ( init_poses, goal_poses ) = read_exp_setup( test_env.episode_data())

    # FIXME: these pprints are an abomination;  refactor
    print(f'[EXPERIMENT] vehicle poses:  ')
    pp(list(zip(enumerate(map(lambda x: x.split(','), init_poses.split(';'))))))
    print(f'[EXPERIMENT] goal positions:  ')
    pp(list(zip(enumerate(map(lambda x: x.split(','), goal_poses.split(';'))))))
    # print(f'[EXPERIMENT] obstacle poses: {buoy_poses}')
    
    print(f"[EXPERIMENT] Running {planner.agent_type} experiment with num_robots={test_env.num_robots}")

    rclpy.init()

    exp_manager = ExperimentManager(
        # test_env.num_robots,
        # successes_data=[],
        # travel_times_data=[],
    )


    print("\n\n\n====== Experiment Result ======")
    print("Number of robots: ",exp_manager.num_robots)
    print("Current episode success: ","True" if exp_manager.exp_success else "False")
    print("Current episode travel times:",exp_manager.robot_info)
    # print(
    #     "All episodes success rate: ",
    #     np.sum( exp_manager.successes_data[-1] ) / len( exp_manager.successes_data[-1] )
    # )
    # print(
    #     "All episodes avg travel time: ",
    #     np.mean( exp_manager.travel_times_data[-1] )
    #     if len( exp_manager.travel_times_data[-1] ) > 0
    #     else "NaN","\n\n\n"
    # )
