#!/usr/bin/env python3

import math

from typing import Self, Optional

import std_msgs.msg
import ros_gz_interfaces.msg
import ros_gz_interfaces.srv
import virelex_msgs.msg

import rclpy
import rclpy.node
import rclpy.executors

from rcl_interfaces.msg import ParameterDescriptor, IntegerRange, FloatingPointRange

class ExperimentManager(rclpy.node.Node):

    def __init__(
            self,
            node_name: Optional[str] = 'experiment_manager',
            **kwargs,
    ) -> Self:
        super().__init__(node_name, **kwargs)
        
        self.num_robots = self.declare_parameter(
            name       = 'num_robots',
            value      = rclpy.Parameter.Type.INTEGER,
            descriptor = ParameterDescriptor(
                description   = 'Number of robots spawned and used in the experiment',
                read_only     = True,
                integer_range = [IntegerRange(from_value=1,to_value=math.inf,step=1)],
            ),
        ).value
        
        self.world_name = self.declare_parameter(
            name       = 'world_name',
            value      = rclpy.Parameter.Type.STRING,
            descriptor = ParameterDescriptor(
                description = 'String name of Gazebo world, as present on the `<world>` attribute of the loaded SDF data',
                read_only   = True,
            ),
        ).value
        
        self.time_limit = self.declare_parameter(
            name       = 'time_limit',
            value      = 90.0,
            descriptor = ParameterDescriptor(
                description = 'Floating-point number of seconds after which the experiment will terminate automatically',
                read_only   = True,
                floating_point_range = [FloatingPointRange(from_value=0,to_value=math.inf,step=0)],
            )
        ).value
        
        if not self.num_robots:
            raise ValueError(
                'num_robots is a required parameter, received:  '
                f"'{self.num_robots}'"
            )
        
        if not self.world_name:
            raise ValueError(
                'world_name is a required parameter, received:  '
                f"'{self.world_name}'"
            )
        
        self.robot_info = {
            f'wamv{idx}': []
            for idx in range(1, self.num_robots+1)
        }
        
        self.create_subscription(
            msg_type    = ros_gz_interfaces.msg.Contacts,
            topic       = '/vrx/contacts',
            callback    = self.collision_detection_callback,
            qos_profile = 10,
        )

        for robot_name in self.robot_info:
            self.create_subscription(
                msg_type    = virelex_msgs.msg.RobotInfo,
                topic       = f'/{robot_name}/robot_info',
                callback    = self.robot_info_callback,
                qos_profile = 10
            )

        self.unpause_signal_publishers = [
            self.create_publisher(
                msg_type    = std_msgs.msg.Empty, 
                topic       = f'/{robot_name}/unpause_signal',
                qos_profile = 10
            )
            for robot_name in self.robot_info
        ]

        # FIXME: this should probably tear itself down after the first
        #        invocation (better-yet, we shouldn't need to time
        #        this wait at all; TODO: use gazebo comms to wait on
        #        simulator init...)
        self.create_timer(
            timer_period_sec = 20,
            callback = lambda: [
                pub.publish( std_msgs.msg.Empty() )
                for pub in self.unpause_signal_publishers
            ]
        )

        self.world_control = self.create_client(
            srv_type = ros_gz_interfaces.srv.ControlWorld,
            srv_name = f'/world/{self.world_name}/control',
        )

        self.get_logger().info("Starting experiment_manager with %d robots in world '%s'" % (
            self.num_robots,
            self.world_name,
        ))

    def collision_detection_callback(self, msg: ros_gz_interfaces.msg.Contacts):
        match msg.contacts:
            case []:
                self.get_logger().error(
                    'Empty contacts list in Contact message at time %d.%d' % (
                        msg.header.stamp.sec,
                        msg.header.stamp.nanosec
                    )
                )
            case [ *contacts ]:
                # TODO: should we extract more information from this
                #       (this construction discards joint/link info)
                contact_pairs = set([
                    tuple(
                        sorted([
                            contact.collision1.name.split('::')[0],
                            contact.collision2.name.split('::')[0],
                        ])
                    )
                    for contact in contacts
                ])

                for (entity_1, entity_2) in contact_pairs:
                    self.get_logger().warning(
                        'Contact detected between {} and {}'.format(
                            entity_1,
                            entity_2,
                        )
                    )
        self.pause_simulation()

    def robot_info_callback(self,msg):
        robot_name = msg.robot_name.data
        if robot_name not in self.robot_info:
            self.get_logger().error(
                f"Received RobotInfo data for robot '{robot_name}', which is "
                f'not in the list of known robot names ({self.robot_info.keys()})'
            )
        self.robot_info[robot_name].append(msg)

        if all(
            states and states[-1].reach_goal.data
            for _,states in self.robot_info.items()
        ):
            self.get_logger().info( 'SUCCESS: all goals reached' )
            self.pause_simulation()

        if any(
            states and states[-1].travel_time.data > self.time_limit
            for _,states in self.robot_info.items()
        ):
            self.get_logger().warning( 'FAILURE: experiment time limit exceeded' )
            self.pause_simulation()

    def pause_simulation(self):
        srv = ros_gz_interfaces.srv.ControlWorld
        msg = ros_gz_interfaces.msg.WorldControl()
        msg.pause = True
        # msg.reset.all = True
        req = srv.Request(world_control=msg)
        self.get_logger().info(
            f'Pausing simulation using srv %s' % self.world_control.service_name
        )

        if not self.world_control.call( req, 5.0 ):
            self.get_logger().error(
                f"Gazebo WorldControl pause request timed out on service '%s'" % self.world_control.service_name
            )


def main(args=None):
    try:
        rclpy.init(args=args)
        experiment_manager = ExperimentManager()
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(experiment_manager)

        executor.spin()

        experiment_manager.destroy_node()
        rclpy.shutdown()

    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
        
if __name__ == "__main__":
    main()
