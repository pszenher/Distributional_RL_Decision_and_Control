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

    buoy_poses = ""
    for i in range(len(ep_data["env"]["obstacles"]["positions"])):
        pos = ep_data["env"]["obstacles"]["positions"][i]

        pos_x = pos[0] - test_env_center[0] + vrx_center[0]
        pos_y = pos[1] - test_env_center[1] + vrx_center[1]
        r = 0.75

        buoy_poses += str(pos_x)+","+str(pos_y)+","+str(r)+";"

    return init_poses[:-1], goals[:-1], buoy_poses[:-1]

if __name__ == '__main__':
    
    method = "RL"
    
    if method == "RL":
        agent_type = "AC-IQN"
        model_path = get_package_share_directory("virelex") + "/trained/traced_AC_IQN_model.pt"
    elif method == "APF":
        agent_type = "APF"
        model_path = " "
    elif method == "MPC":
        agent_type = "MPC"
        model_path = " "
    else:
        raise RuntimeError("Agent type not implemented!")


    # vrx envrionment configuration file
    world_sdf_dir = get_package_share_directory("virelex") + "/worlds"

    dt = datetime.now()
    timestamp = dt.strftime("%Y-%m-%d-%H-%M-%S")
    exp_result_file_dir = "vrx/experiments/results/save/directory"
    result_file_dir = os.path.join(exp_result_file_dir,f"{agent_type}/{timestamp}")
    os.makedirs(result_file_dir)

    ##### run multiple experiments with randomly generated configs #####

    # vrx experiment settings file 
    # note: running over 10 episodes in one trial is not recommended
    #       since Gazebo is more likely to crash after frequent killing
    #       and relaunching simulations in this program   
    eval_schedules = dict(
        num_episodes=[10],
        num_robots=[5],
        num_cores=[0],
        num_obstacles=[4],
        min_start_goal_dis=[40.0]
    )

    result_file = "vrx_exp_results.npz"

    test_env = rfarl.envs.marinenav.MarineNavEnv3(seed = 0)
    test_env.num_robots = 5
    test_env.num_cores = 0
    test_env.num_obs = 0
    test_env.min_start_goal_dis = 40.0
    test_env.reset()

    ( init_poses,
      goals,
      buoy_poses ) = read_exp_setup( test_env.episode_data())

    print(f'[EXPERIMENT] vehicle poses: {init_poses}')
    print(f'[EXPERIMENT] goal positions: {goals}')
    print(f'[EXPERIMENT] obstacle poses: {buoy_poses}')
    
    print(f"[EXPERIMENT] Running {agent_type} experiment with num_robots={test_env.num_robots}")

    rclpy.init()

    exp_manager = ExperimentManager(
        test_env.num_robots,
        successes_data=[],
        travel_times_data=[],
    )
    
    exp_manager.launch_simulation(
        init_poses=init_poses,
        goals=goals,
        buoy_poses=buoy_poses,
        method=method,
        agent_type=agent_type,
        model_path=model_path,
        world_name="sydney_regatta"
    )

    print("\n\n\n====== Experiment Result ======")
    print("Number of robots: ",exp_manager.num_robots)
    print("Current episode success: ","True" if exp_manager.exp_success else "False")
    print("Current episode travel times:",exp_manager.robot_info)
    print(
        "All episodes success rate: ",
        np.sum( exp_manager.successes_data[-1] ) / len( exp_manager.successes_data[-1] )
    )
    print(
        "All episodes avg travel time: ",
        np.mean( exp_manager.travel_times_data[-1] )
        if len( exp_manager.travel_times_data[-1] ) > 0
        else "NaN","\n\n\n"
    )
