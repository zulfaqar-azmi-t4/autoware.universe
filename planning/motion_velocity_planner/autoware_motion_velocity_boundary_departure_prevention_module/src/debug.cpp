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

#include "debug.hpp"

#include <autoware_utils/ros/marker_helper.hpp>

namespace autoware::motion_velocity_planner::debug
{
MarkerArray create_debug_marker_array(
  [[maybe_unused]]const lane_departure_checker::Output & output, const rclcpp::Clock::SharedPtr & clock_ptr,
  [[maybe_unused]]const double base_link_z)
{
  using autoware_utils::create_default_marker;
  using autoware_utils::create_marker_color;
  using autoware_utils::create_marker_scale;
  using autoware_utils::to_msg;

  const auto line_list = visualization_msgs::msg::Marker::LINE_LIST;

  const auto curr_time = clock_ptr->now();

  MarkerArray marker_array;
  const auto color = create_marker_color(0.0, 1.0, 0.0, 0.5);
  const auto m_scale = create_marker_scale(0.05, 0, 0);

  auto marker =
    create_default_marker("map", curr_time, "vehicle_sides", 0, line_list, m_scale, color);

  const auto to_geom = [base_link_z](const auto & pt) { return to_msg(pt.to_3d(base_link_z)); };

  for (const auto & [left, right] : output.ego_footprints_sides) {
    marker.points.push_back(to_geom(left.first));
    marker.points.push_back(to_geom(left.second));
    marker.points.push_back(to_geom(right.first));
    marker.points.push_back(to_geom(right.second));
  }

  marker_array.markers.push_back(marker);
  return marker_array;
}
}  // namespace autoware::motion_velocity_planner::debug
