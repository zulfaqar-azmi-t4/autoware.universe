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
EgoFootprintsSides get_ego_footprints_sides(
  const std::vector<std::pair<LinearRing2d, Pose>> & footprints_with_pose)
{
  EgoFootprintsSides footprints_sides;
  footprints_sides.reserve(footprints_with_pose.size());
  for (const auto & [fp, pose] : footprints_with_pose) {
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
