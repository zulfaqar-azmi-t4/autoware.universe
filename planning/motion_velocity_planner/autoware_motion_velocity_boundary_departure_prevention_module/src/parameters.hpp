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

#include <autoware_utils/ros/parameter.hpp>

#include <limits>
#include <string>
#include <vector>

#ifndef PARAMETERS_HPP_
#define PARAMETERS_HPP_

namespace autoware::motion_velocity_planner::param
{
using autoware_utils::get_or_declare_parameter;
struct BoundaryBehaviorTrigger
{
  bool enable{true};
  BoundaryThreshold th_dist_to_boundary_m{
    std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
};

struct PredictedPathFootprint
{
  double scale{1.0};
  double extra_margin_m{0.0};
  double resample_interval_m{0.3};
};

struct FootprintMargin
{
  double lon;
  double lat;
};

enum class DepartureStatus { NORMAL = 0, NEAR_BOUNDARY, APPROACHING_DEPARTURE, CRITICAL_DEPARTURE };

struct NodeParam
{
  double th_data_timeout_s{1.0};
  std::vector<std::string> boundary_types_to_detect;

  BoundaryBehaviorTrigger slow_down_near_boundary;
  BoundaryBehaviorTrigger slow_down_before_departure;
  BoundaryBehaviorTrigger stop_before_departure;

  PredictedPathFootprint pred_path_footprint;

  NodeParam() = default;
  explicit NodeParam(rclcpp::Node & node)
  {
    const std::string module_name{"boundary_departure_prevention."};
    th_data_timeout_s = get_or_declare_parameter<double>(node, module_name + "th_data_timeout_s");
    boundary_types_to_detect = get_or_declare_parameter<std::vector<std::string>>(
      node, module_name + "boundary_types_to_detect");

    auto boundary_behaviour_trigger_param = [&node,
                                             &module_name](const std::string & trigger_type_str) {
      BoundaryBehaviorTrigger trigger;
      const std::string ns{module_name + trigger_type_str + "."};
      trigger.enable = get_or_declare_parameter<bool>(node, ns + "enable");
      trigger.th_dist_to_boundary_m.left =
        get_or_declare_parameter<double>(node, ns + "th_dist_to_boundary_m.left");
      trigger.th_dist_to_boundary_m.right =
        get_or_declare_parameter<double>(node, ns + "th_dist_to_boundary_m.right");
      return trigger;
    };

    slow_down_near_boundary = boundary_behaviour_trigger_param("slow_down_near_boundary");
    slow_down_before_departure = boundary_behaviour_trigger_param("slow_down_before_departure");
    stop_before_departure = boundary_behaviour_trigger_param("stop_before_departure");

    pred_path_footprint = std::invoke([&node, &module_name]() {
      PredictedPathFootprint param;
      const std::string ns{module_name + "predicted_path_footprint."};
      param.scale = get_or_declare_parameter<double>(node, ns + "scale");
      param.extra_margin_m = get_or_declare_parameter<double>(node, ns + "extra_margin_m");
      param.resample_interval_m =
        get_or_declare_parameter<double>(node, ns + "resample_interval_m");
      return param;
    });
  }
};
}  // namespace autoware::motion_velocity_planner::param

#endif  // PARAMETERS_HPP_
