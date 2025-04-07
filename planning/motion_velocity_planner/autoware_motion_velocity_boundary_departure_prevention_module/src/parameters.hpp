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

#include "type_alias.hpp"

#include <limits>
#include <string>
#include <vector>

#ifndef PARAMETERS_HPP_
#define PARAMETERS_HPP_

namespace autoware::motion_velocity_planner::param
{
struct BoundaryBehaviorTrigger
{
  bool enable{true};
  BoundaryThreshold th_dist_to_boundary_m{
    std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
};

struct NodeParam
{
  double update_rate{10.0};
  std::vector<std::string> boundary_type_to_detect{};
  BoundaryBehaviorTrigger slow_down_near_boundary;
  BoundaryBehaviorTrigger slow_down_before_departure;
  BoundaryBehaviorTrigger stop_before_departure;
};
}  // namespace autoware::motion_velocity_planner::param

#endif  // PARAMETERS_HPP_
