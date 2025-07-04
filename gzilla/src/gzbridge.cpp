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

// NOTE: derived from `ros_gz_bridge.cpp` in the `ros_gz_bridge` package repo
// 
// see:
//   https://github.com/gazebosim/ros_gz/blob/bba6783a85955e2719a0d11468d9a8a79223b0a7/ros_gz_bridge/src/ros_gz_bridge.cpp


#include <gzilla/gzbridge.hpp>

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "bridge_handle_ros_to_gz.hpp"
#include "bridge_handle_gz_to_ros.hpp"

#include <rclcpp/expand_topic_or_service_name.hpp>

namespace gzilla
{

GzBridge::GzBridge(const rclcpp::NodeOptions & options)
: rclcpp::Node("ros_gz_bridge", options),
  gz_node_{ std::make_shared<gz::transport::Node>() },

{
  this->declare_parameters('', /*namespace*/
			   {
			     {"subscription_heartbeat",              1000},
			     {"config_file",                           ""},
			     {"expand_gz_topic_names",              false},
			     {"override_timestamps_with_wall_time", false},
			     {"bridge_names",  std::vector<std::string>()}
			   },
			   false /*ignore-overrides*/)

  const auto names =
    this->get_parameter("bridge_names").as_string_array();

  for (const auto & name : names) {
    this->declare_parameters(
			     // NOTE: originally just `bridges`, all under same namespace
			     "topic_bridge" + "." + name,
			     {
			       {"ros_type_name",    rclcpp::PARAMETER_STRING},
			       {"ros_topic_name",   rclcpp::PARAMETER_STRING},
			       {"gz_topic_name",    rclcpp::PARAMETER_STRING},
			       {"gz_type_name",     rclcpp::PARAMETER_STRING},

			       {"direction",        "BIDIRECTIONAL"},
			       {"publisher_queue",   -1},
			       {"subscriber_queue",  -1},
			       {"lazy",             false},
			       {"qos_profile",      ""},
			     }
			     );
    
    this->declare_parameters(
			     "service_bridge" + "." + name,
			     {
			       // NOTE: originally just `service-name`, applied at both ends
			       {"ros_service_name", rclcpp::PARAMETER_STRING},
			       {"gz_service_name",  rclcpp::PARAMETER_STRING},
			       
			       {"ros_type_name",    rclcpp::PARAMETER_STRING},
			       {"gz_req_type_name", rclcpp::PARAMETER_STRING},
			       {"gz_rep_type_name", rclcpp::PARAMETER_STRING},
			     }
			     );
    
    if (ros_type_name.get_type() == PARAMETER_NOT_SET) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Bridge %s does not set required parameter ros_type_name.", name.c_str());
      continue;
    }

    if (is_topic) {
      if (gz_topic_name.get_type() == PARAMETER_NOT_SET) {
        RCLCPP_ERROR( this->get_logger(),
		     "Bridge %s does not set required parameter gz_topic_name.",
		     name.c_str());
        continue;
      }
      if (gz_type_name.get_type() == PARAMETER_NOT_SET) {
        RCLCPP_ERROR( this->get_logger(),
		     "Bridge %s does not set required parameter gz_type_name.",
 		     name.c_str());
        continue;
      }
    } else {
      if (gz_req_type.get_type() == PARAMETER_NOT_SET) {
        RCLCPP_ERROR( this->get_logger(),
		     "Bridge %s does not set required parameter gz_req_type_name.",
		     name.c_str());
        continue;
      }
      if (gz_rep_type.get_type() == PARAMETER_NOT_SET) {
        RCLCPP_ERROR( this->get_logger(),
		     "Bridge %s does not set required parameter gz_rep_type_name.",
		     name.c_str());
        continue;
      }
    }
  }

  int heartbeat;
  this->get_parameter("subscription_heartbeat", heartbeat);
  
  heartbeat_timer_ = this->create_wall_timer(std::chrono::milliseconds(heartbeat),
					     std::bind(&GzBridge::spin, this));
}

void GzBridge::spin()
{
  if (handles_.empty()) {
    std::string config_file;
    this->get_parameter("config_file", config_file);
    bool expand_names;
    this->get_parameter("expand_gz_topic_names", expand_names);
    const std::string ros_ns = this->get_namespace();
    const std::string ros_node_name = this->get_name();

    // Add bridges from config file
    if (!config_file.empty()) {
      auto entries = readFromYamlFile(config_file);
      for (auto & entry : entries) {
        if (expand_names) {
          entry.gz_topic_name = rclcpp::expand_topic_or_service_name(
            entry.gz_topic_name, ros_node_name, ros_ns, false);
        }
        this->add_bridge(entry);
      }
    }

    // Add bridges from parameters
    const auto names = this->get_parameter("bridge_names").as_string_array();
    for (const auto & name : names) {
      const auto prefix = "bridges." + name + ".";
      if (!this->get_parameter(prefix + "ros_topic_name").as_string().empty()) {
        const auto directionStr = this->get_parameter(prefix + "direction").as_string();
        BridgeDirection direction {BridgeDirection::NONE};
        if (directionStr == "NONE") {
          direction = BridgeDirection::NONE;
        } else if (directionStr == "BIDIRECTIONAL") {
          direction = BridgeDirection::BIDIRECTIONAL;
        } else if (directionStr == "GZ_TO_ROS") {
          direction = BridgeDirection::GZ_TO_ROS;
        } else if (directionStr == "ROS_TO_GZ") {
          direction = BridgeDirection::ROS_TO_GZ;
        } else {
          RCLCPP_ERROR(
            this->get_logger(),
            "Bridge %s defines unknown direction %s.",
            name.c_str(), directionStr.c_str());
          continue;
        }

        const auto qos_profile_str = this->get_parameter(prefix + "qos_profile").as_string();
        std::optional<rclcpp::QoS> qos_profile;
        if (!qos_profile_str.empty()) {
          try {
            qos_profile = parseQoS(qos_profile_str);
          } catch (const std::invalid_argument & e) {
            RCLCPP_ERROR(
              this->get_logger(),
              "Bridge %s defines unknown QoS profile %s.",
              name.c_str(), qos_profile_str.c_str());
            continue;
          }
        }

        const auto pub_queue_size_int = this->get_parameter(prefix + "publisher_queue").as_int();
        std::optional<size_t> pub_queue_size;
        if (pub_queue_size_int >= 0) {
          pub_queue_size = pub_queue_size_int;
        } else if (!qos_profile.has_value()) {
          pub_queue_size = kDefaultPublisherQueue;
        }

        const auto sub_queue_size_int = this->get_parameter(prefix + "subscriber_queue").as_int();
        std::optional<size_t> sub_queue_size;
        if (sub_queue_size_int >= 0) {
          sub_queue_size = sub_queue_size_int;
        } else if (!qos_profile.has_value()) {
          sub_queue_size = kDefaultSubscriberQueue;
        }

        BridgeConfig config {
          this->get_parameter(prefix + "ros_type_name").as_string(),
          this->get_parameter(prefix + "ros_topic_name").as_string(),
          this->get_parameter(prefix + "gz_type_name").as_string(),
          this->get_parameter(prefix + "gz_topic_name").as_string(),
          direction,
          pub_queue_size,
          sub_queue_size,
          this->get_parameter(prefix + "lazy").as_bool(),
          qos_profile,
        };
        if (expand_names) {
          config.gz_topic_name = rclcpp::expand_topic_or_service_name( config.gz_topic_name, ros_node_name, ros_ns, false );
        }

        this->add_bridge(config);
      } else {
        this->add_service_bridge(
          this->get_parameter(prefix + "ros_type_name").as_string(),
          this->get_parameter(prefix + "gz_req_type_name").as_string(),
          this->get_parameter(prefix + "gz_rep_type_name").as_string(),
          this->get_parameter(prefix + "service_name").as_string());
      }
    }
  }
  for (auto & bridge : handles_) {
    bridge->Spin();
  }
}

void GzBridge::add_topic_bridge(const BridgeConfig & config)
{
  bool gz_to_ros = false;
  bool ros_to_gz = false;

  if (config.direction == BridgeDirection::GZ_TO_ROS) {
    gz_to_ros = true;
  }

  if (config.direction == BridgeDirection::ROS_TO_GZ) {
    ros_to_gz = true;
  }

  if (config.direction == BridgeDirection::BIDIRECTIONAL) {
    ros_to_gz = true;
    gz_to_ros = true;
  }

  try {
    if (gz_to_ros) {
      // RCLCPP_INFO(
      //   this->get_logger(),
      //   "Creating GZ->ROS Bridge: [%s (%s) -> %s (%s)] (Lazy %d)",
      //   config.gz_topic_name.c_str(), config.gz_type_name.c_str(),
      //   config.ros_topic_name.c_str(), config.ros_type_name.c_str(),
      //   config.is_lazy);
      handles_.push_back(
        std::make_unique<ros_gz_bridge::BridgeHandleGzToRos>( shared_from_this(), gz_node_, config ));

      handles_.back()->Start();
    }

    if (ros_to_gz) {
      // RCLCPP_INFO(
      //   this->get_logger(),
      //   "Creating ROS->GZ Bridge: [%s (%s) -> %s (%s)] (Lazy %d)",
      //   config.ros_topic_name.c_str(), config.ros_type_name.c_str(),
      //   config.gz_topic_name.c_str(), config.gz_type_name.c_str(),
      //   config.is_lazy);
      handles_.push_back(
        std::make_unique<ros_gz_bridge::BridgeHandleRosToGz>( shared_from_this(), gz_node_, config));

      handles_.back()->Start();
    }
  } catch (std::runtime_error & _e) {
    RCLCPP_WARN(
      this->get_logger(),
      "Failed to create a bridge for topic [%s] with ROS2 type [%s] "
      "to topic [%s] with Gazebo Transport type [%s]: %s",
      config.ros_topic_name.c_str(),
      config.ros_type_name.c_str(),
      config.gz_topic_name.c_str(),
      config.gz_type_name.c_str(),
      _e.what());
  }
}

void GzBridge::add_service_bridge(
  const std::string & ros_type_name,
  const std::string & gz_req_type_name,
  const std::string & gz_rep_type_name,
  const std::string & service_name)
{
  try {
    // RCLCPP_INFO(
    //   this->get_logger(),
    //   "Creating ROS->GZ service bridge [%s (%s -> %s/%s)]",
    //   service_name.c_str(), ros_type_name.c_str(),
    //   gz_req_type_name.c_str(), gz_rep_type_name.c_str());
    auto factory = get_service_factory(ros_type_name,
				       gz_req_type_name,
				       gz_rep_type_name);
    services_.push_back( factory->create_ros_service(shared_from_this(),
						     gz_node_,
						     service_name) );
  } catch (std::runtime_error & _e) {
    RCLCPP_WARN(
      this->get_logger(),
      "Failed to create a bridge for service [%s] with ROS2 type [%s] "
      " and Gazebo types [%s/%s]: %s",
      service_name.c_str(), ros_type_name.c_str(),
      gz_req_type_name.c_str(), gz_rep_type_name.c_str(),
      _e.what());
  }
}

}  // namespace gzilla

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(gzilla::GzBridge)
