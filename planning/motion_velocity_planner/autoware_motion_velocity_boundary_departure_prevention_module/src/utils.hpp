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

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include "parameters.hpp"

#include <vector>

namespace autoware::motion_velocity_planner::utils
{
std::optional<std::vector<std::pair<LinearRing2d, Pose>>> create_vehicle_footprints(
  const geometry_msgs::msg::PoseWithCovariance & covariance, const TrajectoryPoints & trajectory,
  const VehicleInfo & vehicle_info, const double footprint_margin_scale);

EgoFootprintsSides get_ego_footprints_sides(
  const std::vector<std::pair<LinearRing2d, Pose>> & footprints_with_pose);

param::DepartureStatusesIdx check_departure_status(
  const SideToBoundary & side_near_boundary, const param::NodeParam & param);

double calc_braking_distance(
  const double abs_velocity, const double max_deceleration, const double delay_time,
  const double dist_error);
}  // namespace autoware::motion_velocity_planner::utils
#endif  // UTILS_HPP_
