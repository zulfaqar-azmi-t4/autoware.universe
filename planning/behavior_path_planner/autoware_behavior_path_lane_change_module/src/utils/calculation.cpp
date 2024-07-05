// Copyright 2024 TIER IV, Inc.
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

#include "autoware/behavior_path_lane_change_module/utils/calculation.hpp"

#include "autoware/behavior_path_planner_common/utils/utils.hpp"

namespace autoware::behavior_path_planner::utils::lane_change::calculation
{
static rclcpp::Logger logger{rclcpp::get_logger("lane_change.utils")};

double calc_min_lane_change_length(
  const LCParamPtr & lc_param_ptr, const std::vector<double> & shift_intervals)
{
  if (shift_intervals.empty()) {
    return 0.0;
  }

  const auto min_vel = lc_param_ptr->minimum_lane_changing_velocity;
  const auto lat_acc = lc_param_ptr->lane_change_lat_acc_map.find(min_vel);
  const auto max_lat_acc = lat_acc.second;
  const auto lat_jerk = lc_param_ptr->lane_changing_lateral_jerk;
  const auto finish_judge_buffer = lc_param_ptr->lane_change_finish_judge_buffer;

  const auto calc_sum = [&](double sum, double shift_interval) {
    const auto t = calc_lane_changing_duration(shift_interval, max_lat_acc, lat_jerk);
    return sum + (min_vel * t + finish_judge_buffer);
  };

  const auto total_length =
    std::accumulate(shift_intervals.begin(), shift_intervals.end(), 0.0, calc_sum);

  const auto backward_buffer = lc_param_ptr->backward_length_buffer_for_end_of_lane;
  return total_length + backward_buffer * (static_cast<double>(shift_intervals.size()) - 1.0);
}

double calc_min_lane_change_length(
  const CommonDataPtr & common_data_ptr, const lanelet::ConstLanelets & lanes, Direction direction)
{
  if (lanes.empty()) {
    RCLCPP_DEBUG(logger, "Because the lanes are empty, 0.0 meter is returned.");
    return 0.0;
  }

  const auto & route_handler_ptr = common_data_ptr->route_handler_ptr;
  if (!route_handler_ptr) {
    RCLCPP_DEBUG(logger, "Because route hander pointer is null, 0.0 is returned.");
    return 0.0;
  }

  const auto shift_intervals =
    route_handler_ptr->getLateralIntervalsToPreferredLane(lanes.back(), direction);

  const auto & lc_param_ptr = common_data_ptr->lc_param_ptr;
  return calc_min_lane_change_length(lc_param_ptr, shift_intervals);
}

double calc_ego_remaining_distance_in_current_lanes(const CommonDataPtr & common_data_ptr)
{
  return std::max(
    0.0, calc_ego_remaining_distance_in_lanes(common_data_ptr, common_data_ptr->lanes.current));
}

double calc_ego_remaining_distance_in_lanes(
  const CommonDataPtr & common_data_ptr, const lanelet::ConstLanelets & lanes)
{
  if (lanes.empty()) {
    RCLCPP_DEBUG(logger, "Because the lanes are empty, 0.0 is returned.");
    return 0.0;
  }

  const auto & route_handler_ptr = common_data_ptr->route_handler_ptr;
  if (route_handler_ptr->isInGoalRouteSection(lanes.back())) {
    const auto & goal_pose = route_handler_ptr->getGoalPose();
    return utils::getSignedDistance(common_data_ptr->get_ego_pose(), goal_pose, lanes);
  }

  return utils::getDistanceToEndOfLane(common_data_ptr->get_ego_pose(), lanes);
}

double calc_lane_changing_duration(
  const double shift_length, const double lat_acc, const double lat_jerk)
{
  return PathShifter::calcShiftTimeFromJerk(shift_length, lat_jerk, lat_acc);
}
}  // namespace autoware::behavior_path_planner::utils::lane_change::calculation
