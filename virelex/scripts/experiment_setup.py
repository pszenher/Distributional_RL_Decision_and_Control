#!/usr/bin/env python3

import argparse
from dataclasses import dataclass
from typing import Any, Self

import rfarl.envs.marinenav

@dataclass
class EpisodeData:
    data: dict[ str, dict[str,Any] ]

    TEST_ENV_CENTER = (27.5, 27.5)

    def goal_poses(
            self, center_x: float, center_y: float,
    ) -> str:
        goal_poses = []
        for (goal_x_raw, goal_y_raw) in self.data["robots"]["goal"]:
            goal_x = goal_x_raw - self.TEST_ENV_CENTER[0] + center_x
            goal_y = goal_y_raw - self.TEST_ENV_CENTER[1] + center_y

            goal_poses.append(
                f'{goal_x},{goal_y}'
            )

        return ';'.join(goal_poses)

    def init_poses(
            self, center_x: float, center_y: float,
    ) -> str:
        init_poses = []
        for (init_x_raw, init_y_raw), init_theta in zip(self.data["robots"]["start"], self.data["robots"]["init_theta"]):
            init_x = init_x_raw - self.TEST_ENV_CENTER[0] + center_x
            init_y = init_y_raw - self.TEST_ENV_CENTER[1] + center_y

            init_poses.append(
                f'{init_x},{init_y},{init_theta}'
            )
                              
        return ';'.join(init_poses)

    @classmethod
    def from_params(
            cls,
            *,
            seed: int,
            num_robots: int,
            min_start_goal_distance: float
    ) -> Self:
        test_env = rfarl.envs.marinenav.MarineNavEnv3(
            seed = seed
        )
        test_env.num_robots = num_robots
        test_env.num_cores  = 0
        test_env.num_obs    = 0
        test_env.min_start_goal_dis = min_start_goal_distance
        test_env.reset()

        return cls( test_env.episode_data() )


if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description='Simple command-line utility for generating experiment configurations from rfarl env'
    )

    parser.add_argument(
        'request_type',
        choices=['init_poses','goal_poses'],
    )
        
    parser.add_argument(
        '--num-robots',
        type=int,
        required=True,
        help='Number of robots spawned and used in the experiment'
    )

    parser.add_argument(
        '--seed',
        type=int,
        default=0,
        help='Seed (integer) value for random number generator'
    )

    parser.add_argument(
        '--min-start-goal-distance',
        type=float,
        default=40.0,
        help='Minimum distance between randomly generated start and goal positions'
    )

    parser.add_argument(
        '--center-x',
        type=float,
        default=-480.0,
        help='x-component of center position for experiments in target environment'
    )

    parser.add_argument(
        '--center-y',
        type=float,
        default=240.0,
        help='y-component of center position for experiments in target environment'
    )
    
    args = parser.parse_args()

    episode_data = EpisodeData.from_params(
        seed = args.seed,
        num_robots = args.num_robots,
        min_start_goal_distance = args.min_start_goal_distance
    )

    match args.request_type:
        case 'init_poses':
            print(episode_data.init_poses(args.center_x, args.center_y))
        case 'goal_poses':
            print(episode_data.goal_poses(args.center_x, args.center_y))
