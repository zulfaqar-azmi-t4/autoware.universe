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

#include "utils.hpp"

#include <vector>

namespace autoware::motion_velocity_planner::utils
{
param::FootprintMargin calc_footprint_margin(
  const geometry_msgs::msg::PoseWithCovariance & covariance, const double scale)
{
  const auto cov_in_map = covariance.covariance;
  Eigen::Matrix2d cov_xy_map;
  cov_xy_map << cov_in_map[0 * 6 + 0], cov_in_map[0 * 6 + 1], cov_in_map[1 * 6 + 0],
    cov_in_map[1 * 6 + 1];

  const double yaw_vehicle = tf2::getYaw(covariance.pose.orientation);

  // To get a position in a transformed coordinate, rotate the inverse direction
  Eigen::Matrix2d r_map2vehicle;
  r_map2vehicle << std::cos(-yaw_vehicle), -std::sin(-yaw_vehicle), std::sin(-yaw_vehicle),
    std::cos(-yaw_vehicle);
  // Rotate covariance E((X, Y)^t*(X, Y)) = E(R*(x,y)*(x,y)^t*R^t)
  // when Rotate point (X, Y)^t= R*(x, y)^t.
  const Eigen::Matrix2d cov_xy_vehicle = r_map2vehicle * cov_xy_map * r_map2vehicle.transpose();

  // The longitudinal/lateral length is represented
  // in cov_xy_vehicle(0,0), cov_xy_vehicle(1,1) respectively.
  return param::FootprintMargin{cov_xy_vehicle(0, 0) * scale, cov_xy_vehicle(1, 1) * scale};
}

std::optional<std::vector<LinearRing2d>> create_vehicle_footprints(
  const geometry_msgs::msg::PoseWithCovariance & covariance, const TrajectoryPoints & trajectory,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const double footprint_margin_scale)
{
  if (trajectory.empty()) {
    return std::nullopt;
  }
  // Calculate longitudinal and lateral margin based on covariance
  const auto margin = calc_footprint_margin(covariance, footprint_margin_scale);

  // Create vehicle footprint in base_link coordinate
  const auto local_vehicle_footprint = vehicle_info.createFootprint(margin.lat, margin.lon);

  // Create vehicle footprint on each TrajectoryPoint
  std::vector<LinearRing2d> vehicle_footprints;
  vehicle_footprints.reserve(trajectory.size());
  std::transform(
    trajectory.begin(), trajectory.end(), std::back_inserter(vehicle_footprints),
    [&](const auto & p) -> LinearRing2d {
      using autoware_utils::transform_vector;
      using autoware_utils::pose2transform;
      return transform_vector(local_vehicle_footprint, pose2transform(p.pose));
    });

  return vehicle_footprints;
}

EgoFootprintsSides get_ego_footprints_sides(const std::vector<LinearRing2d> & footprints_with_pose)
{
  EgoFootprintsSides footprints_sides;
  footprints_sides.reserve(footprints_with_pose.size());
  for (const auto & fp : footprints_with_pose) {
    EgoFootprintSide footprint_side;
    const Point2d right_front(fp.at(1).x(), fp.at(1).y());
    const Point2d right_rear(fp.at(3).x(), fp.at(3).y());
    footprint_side.right = {right_front, right_rear};

    const Point2d left_front(fp.at(6).x(), fp.at(6).y());
    const Point2d left_rear(fp.at(4).x(), fp.at(4).y());
    footprint_side.left = {left_front, left_rear};
    footprints_sides.push_back(footprint_side);
  }

  return footprints_sides;
}
}  // namespace autoware::motion_velocity_planner::utils
