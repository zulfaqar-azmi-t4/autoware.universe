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

#include "type_alias.hpp"

#include <vector>

namespace autoware::motion_velocity_planner::utils
{
EgoFootprintsSides get_ego_footprints_sides(
  const std::vector<std::pair<LinearRing2d, Pose>> & footprints_with_pose);
}  // namespace autoware::motion_velocity_planner::utils
#endif  // UTILS_HPP_
