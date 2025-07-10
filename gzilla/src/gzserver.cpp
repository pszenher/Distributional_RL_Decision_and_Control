// Copyright 2024 Open Source Robotics Foundation, Inc.
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

// NOTE: derived from `gzserver.cpp` in the `ros_gz_sim` package repo
//
// see:
//   https://github.com/gazebosim/ros_gz/blob/bba6783a85955e2719a0d11468d9a8a79223b0a7/ros_gz_sim/src/gzserver.cpp
//

#include "gzilla/gzcommon.hpp"
#include "gzilla/gzserver.hpp"

#include <algorithm>
#include <gz/common/Console.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/sim/SystemLoader.hh>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <thread>

namespace gzilla
{

class GzServer::Implementation
{
  /// \brief We don't want to block the ROS thread.

public:
  std::thread thread;
};

GzServer::GzServer(const rclcpp::NodeOptions & options)
: Node("gzserver", options), dataPtr(gz::utils::MakeUniqueImpl<Implementation>())
{
  this->dataPtr->thread = std::thread([this] { OnStart(); });
}

GzServer::~GzServer()
{
  // Make sure to join the thread on shutdown.
  if (this->dataPtr->thread.joinable()) {
    this->dataPtr->thread.join();
  }
}

void GzServer::OnStart()
{
  auto world_sdf_file = this->declare_parameter("world_sdf_file", "");
  auto world_sdf_string = this->declare_parameter("world_sdf_string", "");
  auto initial_sim_time = this->declare_parameter("initial_sim_time", 0.0);

  auto verbosity = this->declare_parameter("verbosity", 1);
  auto paused = this->declare_parameter("paused", true);

  if (verbosity < 1 || GZ_MAX_LOG_LEVEL < verbosity) {
    auto input_verbosity = verbosity;
    verbosity = std::clamp(verbosity, 1l, static_cast<long>(GZ_MAX_LOG_LEVEL));
    RCLCPP_WARN(
      this->get_logger(),
      "Out-of-range verbosity level provided (verbosity=%ld);  clamping to nearest:  %ld",
      input_verbosity, verbosity);
  }

  gz::common::Console::SetVerbosity(static_cast<int>(verbosity));
  gz::sim::ServerConfig server_config;

  if (!world_sdf_file.empty()) {
    server_config.SetSdfFile(world_sdf_file);
  } else if (!world_sdf_string.empty()) {
    server_config.SetSdfString(world_sdf_string);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Must specify either 'world_sdf_file' or 'world_sdf_string'");
    rclcpp::shutdown();
    return;
  }
  server_config.SetInitialSimTime(initial_sim_time);

  gz::sim::Server server(server_config);
  server.Run(true /* _blocking   */, 0 /* _iterations */, paused /* _paused     */);
  rclcpp::shutdown();
}

}  // namespace gzilla

RCLCPP_COMPONENTS_REGISTER_NODE(gzilla::GzServer)
