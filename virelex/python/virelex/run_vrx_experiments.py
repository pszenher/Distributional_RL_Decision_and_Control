import copy
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

def add_buoy_to_sdf(input_file, output_file, buoy_poses):
    shutil.copyfile(input_file, output_file)
    tree = ET.parse(output_file)
    root = tree.getroot()

    # Find the <world name="sydney_regatta"> tag
    world_tag = root.find(".//world[@name='sydney_regatta']")
    if world_tag is None:
        print("Error: Could not find <world name='sydney_regatta'> tag in SDF file.")
        return

    # Set world `name` attribute to match new sdf filename
    #   note:  failing to keep these in sync breaks the gz->ros sensor bridge
    (new_world_name, _) = os.path.splitext(
        os.path.basename( output_file )
    )
    world_tag.set('name', new_world_name)
    
    poses = []
    if len(buoy_poses)>0: 
        poses = [[float(p) for p in pos.split(',')] for pos in buoy_poses.split(';')]
    
    assert len(poses) <= 4, "Error: Currently do not support more than 4 buoys"
    buoys = ["robotx_light_buoy_rgb","robotx_light_buoy_rgy","robotx_light_buoy_ybr",
             "robotx_light_buoy_yrg"]

    for i,pose in enumerate(poses):
        include_elem = ET.Element('include')

        name_elem = ET.Element('name')
        name_elem.text = buoys[i]
        include_elem.append(name_elem)

        # Add pose element
        pose_elem = ET.Element('pose')
        pose_elem.text = f"{pose[0]} {pose[1]} 0.32 0 0 0"  # Set the fixed orientation
        include_elem.append(pose_elem)

        # Add uri element
        uri_elem = ET.Element('uri')
        # append `_waves` to uri for use of custom wrapper models with builtin buoyancy
        uri_elem.text = buoys[i] + "_waves" 
        include_elem.append(uri_elem)

        # Append the include element to the world_tag
        world_tag.append(include_elem)

    # Write the modified tree back to the file
    tree.write(output_file)

def exp_setup(test_env,eval_schedule,i):
    test_env.num_robots = eval_schedule["num_robots"][i]
    test_env.num_cores = eval_schedule["num_cores"][i]
    test_env.num_obs = eval_schedule["num_obstacles"][i]
    test_env.min_start_goal_dis = eval_schedule["min_start_goal_dis"][i]
    test_env.reset()

    ep_data = test_env.episode_data()

    return read_exp_setup(ep_data)

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
    
    input_world = "sydney_regatta"
    output_world = "sydney_regatta_aux"
    
    input_sdf_file = f"{world_sdf_dir}/{input_world}.sdf"
    output_sdf_file = f"{world_sdf_dir}/{output_world}.sdf"

    init_poses_data = []
    goals_data = []
    buoy_poses_data = []
    robot_num_data = []
    buoy_num_data = []
    successes_data = []
    travel_times_data = []

    
    dt = datetime.now()
    timestamp = dt.strftime("%Y-%m-%d-%H-%M-%S")
    exp_result_file_dir = "vrx/experiments/results/save/directory"
    result_file_dir = os.path.join(exp_result_file_dir,f"{agent_type}/{timestamp}")
    os.makedirs(result_file_dir)


    run_with_exp_config = False
    if run_with_exp_config:
        ##### run an experiment with specified config ##### 
        
        print(f"\n\n\nRunning {agent_type} experiment with given config settings \n\n\n")
        
        exp_config_file = "path/to/vrx/experiment/episode/config/file"

        successes_data.append([])
        travel_times_data.append([])

        with open(exp_config_file,"r") as f:
            episode_setup = json.load(f)
        
        init_poses,goals,buoy_poses = read_exp_setup(episode_setup)

        add_buoy_to_sdf(input_sdf_file,output_sdf_file,buoy_poses)

        rclpy.init()

        exp_manager = ExperimentManager(len(episode_setup["robots"]["start"]),successes_data,travel_times_data,save_traj=True)
        exp_manager.launch_simulation(init_poses,goals,buoy_poses,method,agent_type,model_path,output_world)

        # save trajectory data
        timestamps = {}
        poses = {}
        velocities = {}
        for name in exp_manager.timestamp_data.keys():
            timestamps[name] = [round(t.stamp.sec+t.stamp.nanosec * 1e-9,2) for t in exp_manager.timestamp_data[name]]
            poses[name] = [[p.position.x,p.position.y,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w] 
                           for p in exp_manager.pose_data[name]]
            velocities[name] = [[v.linear.x,v.linear.y] for v in exp_manager.velocity_data[name]]

            print(f"\ntimes: {len(timestamps[name])}, poses: {len(poses[name])}, velocities: {len(velocities[name])}\n")
        
        traj_data = dict(timestamps=timestamps,
                         poses=poses,
                         velocities=velocities)

        result_file = "vrx_exp_traj.json"
        with open(os.path.join(result_file_dir,result_file),"w") as file:
            json.dump(traj_data,file)
        
        config_file = "vrx_exp_config.json"
        with open(os.path.join(result_file_dir,config_file),"w") as file:
            json.dump(episode_setup,file)
    else:
        ##### run multiple experiments with randomly generated configs #####

        # vrx experiment settings file 
        # note: running over 10 episodes in one trial is not recommended
        #       since Gazebo is more likely to crash after frequent killing
        #       and relaunching simulations in this program   
        eval_schedules = dict(num_episodes=[10],
                            num_robots=[5],
                            num_cores=[0],
                            num_obstacles=[4],
                            min_start_goal_dis=[40.0]
                            )

        seed = 0
        test_env = rfarl.envs.marinenav.MarineNavEnv3(seed = seed)
        
        result_file = "vrx_exp_results.npz"

        for idx,count in enumerate(eval_schedules["num_episodes"]):
            init_poses_data.append([])
            goals_data.append([])
            buoy_poses_data.append([])
            robot_num_data.append(eval_schedules["num_robots"][idx])
            buoy_num_data.append(eval_schedules["num_obstacles"][idx])
            successes_data.append([])
            travel_times_data.append([])
            for i in range(count):
                print(f"\n\n\nRunning {agent_type} experiment {i} of schedule {idx}\n\n\n")

                init_poses,goals,buoy_poses = exp_setup(test_env,eval_schedules,idx)
                
                init_poses_data[-1].append(copy.deepcopy(init_poses))
                goals_data[-1].append(copy.deepcopy(goals))
                buoy_poses_data[-1].append(copy.deepcopy(buoy_poses))

                add_buoy_to_sdf(input_sdf_file,output_sdf_file,buoy_poses)

                rclpy.init()

                exp_manager = ExperimentManager(eval_schedules["num_robots"][idx],successes_data,travel_times_data)
                exp_manager.launch_simulation(init_poses,goals,buoy_poses,method,agent_type,model_path,output_world)

                successes_data = copy.deepcopy(exp_manager.successes_data)
                travel_times_data = copy.deepcopy(exp_manager.travel_times_data)

                print("\n\n\n====== Experiment Result ======")
                print("Number of robots: ",exp_manager.num_robots)
                print("Current episode success: ","True" if exp_manager.exp_success else "False")
                print("Current episode travel times:",exp_manager.robot_info)
                print("All episodes success rate: ",np.sum(successes_data[-1])/len(successes_data[-1]))
                print("All episodes avg travel time: ",np.mean(travel_times_data[-1]) if len(travel_times_data[-1])>0 else "NaN","\n\n\n")

                np.savez(os.path.join(result_file_dir,result_file),
                        agent_type=agent_type,
                        seed=seed,
                        init_poses=init_poses_data,
                        goals=goals_data,
                        buoy_poses=buoy_poses_data,
                        robot_num=robot_num_data,
                        buoy_num=buoy_num_data,
                        successes=successes_data,
                        travel_times=travel_times_data)

