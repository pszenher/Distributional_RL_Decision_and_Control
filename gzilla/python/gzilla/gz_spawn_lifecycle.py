from dataclasses import dataclass

import rclpy.lifecycle

@dataclass
class GzSpawnerConfig:
    world_name     : str
    entity_name    : str
    allow_renaming : bool

    sdf_data : SdfDataSource
    pose     : GzPose3

    def to_entity_factory(
            self
    ) -> gz_msgs.EntityFactory:

        req = gz_msgs.EntityFactory()

        req.set_name( self.entity_name )
        req.set_allow_renaming( self.allow_renaming )
        
        match self.sdf_data.source_type:
            case SdfSourceType.FILENAME:
                req.set_sdf_filename( self.sdf_data )
            case SdfSourceType.XML_STRING:
                req.set_sdf( self.sdf_data )
            case SdfSourceType.ROS_TOPIC:
                raise RuntimeError(
                    'Attempted to generate EntityFactory message with unresolved SDF data: '
                    f'{self.sdf_data.source_type}'
                )

        req.set_pose(self.pose)

        return req

    
class GzSpawnerNode(rclpy.lifecycle.LifecycleNode):

    def __init__(self, node_name, **kwargs) -> Self:
        super().__init__(node_name, **kwargs)

        # These are reparameterizable at runtime; we declare here, and
        # allow affecting reparameterizations to take place before the
        # call to `configure` is made
        self.declare_parameters(
            namespace='',
            parameters=[
                ('world_name', ''),
                ('entity_name', ''),
                ('allow_renaming', False),
                *SdfSourceType.ros_param_spec(),
                *GzPose.ros_param_spec()

                # TODO: consider adding params for remaining spawner args?
                #   `relative_to`: string   -> frame relative to which pose is interpreted
                #   `spherical_coordinates` -> Spherical coordinates where the entity will be spawned in the world.
              ]
        )

    def on_configure(
            self,
            state: rclpy.lifecycle.State
    ) -> rclpy.lifecycle.TransitionCallbackReturn:

        self.spawner_config = GzSpawnerConfig(
            world_name     = self.get_parameter('world_name'),
            entity_name    = self.get_parameter('entity_name'),
            allow_renaming = self.get_parameter('allow_renaming'),
            # Load the mutually-exclusive filename/string/topic SDF
            # params, use `SdfDataSource` method to handle invalid config
            sdf_data       = SdfDataSource.from_param_dict( self.get_parameters_by_prefix('sdf') ),
            # Load pose parameters directly into the Gazebo `Pose3` format
            # used in the spawn request body
            pose           = GzPose3.from_param_dict( self.get_parameters_by_prefix('pose') ),
        )

        # If the world name was not provided via ROS param, query the
        # Gazebo server for its list of loaded worlds.  If the request
        # fails (likely due to the server not being fully initialized
        # yet), attempt every 5 seconds until one succeeds.
        # 
        if not self.spawner_config.world_name:
            self.world_request_timer = self.create_timer(
                timer_period_sec = 5,
                callback = self.request_gazebo_worlds)

        # If the SDF data format is not directly resolvable (in this
        # case, passed as a ROS topic), configure a subscriber to
        # receive the string at runtime.
        # 
        if not self.spawner_config.sdf_data.is_resolved():
            self.sdf_subscriber = self.create_subscription(
                std_msgs.msg.String,
                self.sdf_data.topic(),
                self.receive_sdf_string,
                # TODO: do we add `rclpy.QoS(1).transient_local()` here for latch-like behavior?
            )

        return rclpy.lifecycle.TransitionCallbackReturn.SUCCESS

    def on_activate(
            self,
            state: rclpy.lifecycle.State
    ) -> rclpy.lifecycle.TransitionCallbackReturn:

        if not self.spawner_config.world_name or not self.spawner_config.sdf_data.is_resolved():
            # If world name or sdf aren't realized yet, reject activation
            rclpy.lifecycle.TransitionCallbackReturn.FAILURE

        
