// Copyright 2025 Open Source Robotics Foundation
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

// NOTE: derived from `gzserver.hpp` in the `ros_gz_sim` package repo
// 
// see:
//   https://github.com/gazebosim/ros_gz/blob/bba6783a85955e2719a0d11468d9a8a79223b0a7/ros_gz_sim/include/ros_gz_sim/gzserver.hpp
// 

#ifndef GZILLA__GZSERVER_HPP_
#define GZILLA__GZSERVER_HPP_

#include <gz/utils/ImplPtr.hh>
#include <rclcpp/node.hpp>

// ROS node that executes a gz-sim Server given a world SDF file or string.
namespace gzilla
{
class GzServer : public rclcpp::Node
{
public:
  // Class constructor.
  explicit GzServer(const rclcpp::NodeOptions & options);

public:
  // Class destructor.
  ~GzServer();

public:
  /// \brief Run the gz sim server.
  void OnStart();

private:
  /// \internal
  /// \brief Private data pointer.
  GZ_UTILS_UNIQUE_IMPL_PTR(dataPtr)
};
}  // namespace gzilla

#endif  // GZILLA__GZSERVER_HPP_
