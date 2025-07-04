#pragma once

namespace gzilla
{

  struct ServiceBridgeConfig
  {
    /// \brief ROS message type (eg std_msgs/msg/String)
    std::string ros_type_name;

    /// \brief ROS service name to bridge
    std::string ros_service_name;
    
    /// \brief Gazebo service request type
    std::string gz_req_type_name;

    /// \brief Gazebo service reply type
    std::string gz_rep_type_name;

    /// \brief Gazebo service name to bridge
    std::string ros_service_name;
  };

  class ServiceBridgeHandle
  {
  public:
    ServiceBridgeHandle()
  };
  
} // namespace gzilla
