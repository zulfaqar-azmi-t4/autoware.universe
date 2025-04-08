// Copyright 2025 TIER IV, Inc.
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

#ifndef TYPE_ALIAS_HPP_
#define TYPE_ALIAS_HPP_

#include <autoware/boundary_departure_checker/parameters.hpp>
#include <autoware/motion_utils/marker/virtual_wall_marker_creator.hpp>
#include <autoware/trajectory/trajectory_point.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/polling_subscriber.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <vector>

namespace autoware::motion_velocity_planner
{
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseWithCovariance;
using nav_msgs::msg::Odometry;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using autoware_adapi_v1_msgs::msg::OperationModeState;
using BoundaryThreshold = lane_departure_checker::Side<double>;
using lane_departure_checker::Output;                       // NOLINT(misc-unused-using-decls)
namespace trajectory = autoware::trajectory;                // NOLINT(misc-unused-alias-decls)
namespace polling_policy = autoware_utils::polling_policy;  // NOLINT(misc-unused-alias-decls)
using autoware_utils_geometry::LinearRing2d;
using autoware_utils_geometry::Point2d;  // NOLINT(misc-unused-using-decls)
using lane_departure_checker::EgoFootprintSide;
using lane_departure_checker::EgoFootprintsSides;
using visualization_msgs::msg::MarkerArray;
}  // namespace autoware::motion_velocity_planner

#endif  // TYPE_ALIAS_HPP_
