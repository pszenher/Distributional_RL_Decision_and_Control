// Copyright 2022 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// NOTE: derived from `ros_gz_bridge.hpp` in the `ros_gz_bridge` package repo
// 
// see:
//   https://github.com/gazebosim/ros_gz/blob/bba6783a85955e2719a0d11468d9a8a79223b0a7/ros_gz_bridge/include/ros_gz_bridge/ros_gz_bridge.hpp


#pragma once

#include <memory>
#include <string>
#include <vector>

#include <gz/msgs/config.hh>
#include <gz/transport/Node.hh>
#include <rclcpp/node.hpp>
#include "gzilla/bridge_config.hpp"

namespace gzilla
{
  /// Forward declarations
  class TopicBridgeHandle;
  class ServiceBridgeHandle;

  /// \brief Component container for the ROS-GZ Bridge
  class GzBridge : public rclcpp::Node
{
public:
  /// \brief Constructor
  /// \param[in] options options control creation of the ROS 2 node
  explicit GzBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// \brief Create a new Gazebo topic bridge
  /// \param[in] config Parameters to control creation of a new bridge
  void add_topic_bridge(const BridgeConfig & config);

  /// \brief Create a new Gazebo service bridge
  /// \param[in] ros_type_name Name of the ROS service (eg ros_bz_interfaces/srv/ControlWorld)
  /// \param[in] gz_req_type_name Gazebo service request type
  /// \param[in] gz_req_type_name Gazebo service response type
  /// \param[in] service_name Address of the service to be bridged
  void add_service_bridge(const std::string & ros_type_name,
			  const std::string & gz_req_type_name,
			  const std::string & gz_rep_type_name,
			  const std::string & service_name);

protected:
  /// \brief Periodic callback to check connectivity and liveliness
  void spin();

protected:
  /// \brief Pointer to Gazebo node used to create publishers/subscribers
  std::shared_ptr<gz::transport::Node> gz_node_;

  /// \brief List of bridge handles
  std::vector<std::shared_ptr<gzilla::TopicBridgeHandle>> topic_handles_;

  /// \brief List of ROS -> Gazebo bridge handles
  std::vector<std::shared_ptr<gzilla::ServiceBridgeHandle>> 

  /// \brief List of bridged ROS services
  std::vector<rclcpp::ServiceBase::SharedPtr> services_;

  /// \brief Timer to control periodic callback
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};
}  // namespace gzilla
