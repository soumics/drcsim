/*
 * Copyright 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef DRCSIM_GAZEBO_ROS_PLUGINS__ROSNODEOPTIONS_HPP_
#define DRCSIM_GAZEBO_ROS_PLUGINS__ROSNODEOPTIONS_HPP_

#include <cstdlib>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace drcsim_gazebo_ros_plugins
{
/// \brief Every ROS-coupled plugin in this package (`VRCPlugin`,
/// `AtlasPlugin`, `VRCScoringPlugin`, `DRCVehicleROSPlugin`) constructs
/// its own internal `rclcpp::Node`, but none of them are started via
/// `ros2 run` -- they're gz-sim System plugins loaded as `.so`s into
/// gzserver's single process, with no access to that process's real
/// `argv`, so `rclcpp::init(0, nullptr)` (what every one of them calls)
/// can never pick up `--ros-args --params-file` overrides the normal
/// way. Without this, none of them can receive parameters from a launch
/// file at all -- not `robot_description`, not `robot_initial_pose.*`,
/// not `atlas_controller.gains.<joint>.*` -- every one of those already-
/// existing `declare_parameter(name, default)` calls silently keeps
/// returning `default`.
///
/// This gives every such node a real way to receive parameters anyway:
/// if the `DRCSIM_ROS_PARAMS_FILE` environment variable is set (by a
/// launch file, in the environment `gz sim` itself is started in), every
/// plugin's node is constructed with that file as its ROS params file.
/// One shared YAML can hold a top-level section per node name
/// (`vrc_plugin`, `atlas_plugin`, `vrc_scoring_plugin`,
/// `drc_vehicle_ros_plugin`) -- each node only ever picks up its own
/// matching section, standard ROS 2 params-file behavior. Without the
/// env var set (e.g. under a gtest, or a bare `gz sim world.sdf` with no
/// launch file involved), this returns exactly the default-constructed
/// `NodeOptions` every plugin used before -- unchanged behavior.
inline rclcpp::NodeOptions RosNodeOptionsFromEnv()
{
  const char * paramsFile = std::getenv("DRCSIM_ROS_PARAMS_FILE");
  if (!paramsFile || std::string(paramsFile).empty()) {
    return rclcpp::NodeOptions();
  }
  return rclcpp::NodeOptions().arguments(
    {"--ros-args", "--params-file", std::string(paramsFile)});
}
}  // namespace drcsim_gazebo_ros_plugins

#endif  // DRCSIM_GAZEBO_ROS_PLUGINS__ROSNODEOPTIONS_HPP_
