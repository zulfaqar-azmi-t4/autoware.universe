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

double calc_length_with_acceleration(
  const double velocity, const double acceleration, const double duration)
{
  return velocity * duration + 0.5 * acceleration * std::pow(duration, 2);
}

double calc_prepare_segment_length(
  const double velocity, const double maximum_velocity, const double acceleration,
  const double duration)
{
  const auto length_with_acceleration =
    calc_length_with_acceleration(velocity, acceleration, duration);

  const auto length_with_max_velocity = maximum_velocity * duration;
  return std::clamp(length_with_acceleration, 0.0, length_with_max_velocity);
}

double calc_lane_changing_segment_length(
  const double velocity, const double minimum_velocity, const double maximum_velocity,
  const double acceleration, const double duration)
{
  if (minimum_velocity > maximum_velocity) {
    return 0.0;
  }
  const auto length_with_acceleration =
    calc_length_with_acceleration(velocity, acceleration, duration);

  const auto length_with_min_velocity = minimum_velocity * duration;
  const auto length_with_max_velocity = maximum_velocity * duration;
  return std::clamp(length_with_acceleration, length_with_min_velocity, length_with_max_velocity);
}

double calc_lane_changing_duration(
  const double shift_length, const double lat_acc, const double lat_jerk)
{
  return PathShifter::calcShiftTimeFromJerk(shift_length, lat_jerk, lat_acc);
}

double calc_shift_length(
  const CommonDataPtr & common_data_ptr, const lanelet::ConstLanelets & lanes)
{
  if (lanes.empty()) {
    return 0.0;
  }

  const auto shift_intervals =
    common_data_ptr->route_handler_ptr->getLateralIntervalsToPreferredLane(lanes.back());
  if (shift_intervals.empty()) {
    return 0.0;
  }
  return shift_intervals.front();
}

double calc_acceleration(
  const double current_velocity, const double target_velocity, const double duration)
{
  return (target_velocity - current_velocity) /
         (duration > 0.0 ? duration : std::numeric_limits<double>::max());
}

double calc_minimum_longitudinal_acceleration(const CommonDataPtr & common_data_ptr)
{
  const auto & bpp_param_ptr = common_data_ptr->bpp_param_ptr;
  const auto & lc_param_ptr = common_data_ptr->lc_param_ptr;

  const auto global_min_lon_acc = bpp_param_ptr->min_acc;
  const auto lc_min_lon_acc = lc_param_ptr->min_longitudinal_acc;
  const auto min_lon_acc = std::max(global_min_lon_acc, lc_min_lon_acc);

  const auto prepare_duration = lc_param_ptr->lane_change_prepare_duration;

  const auto min_lc_vel = lc_param_ptr->minimum_lane_changing_velocity;

  const auto current_vel = common_data_ptr->get_ego_speed();
  const auto acc = calc_acceleration(current_vel, min_lc_vel, prepare_duration);
  return std::clamp(acc, -std::abs(min_lon_acc), -std::numeric_limits<double>::epsilon());
}

double calc_maximum_longitudinal_acceleration(
  const CommonDataPtr & common_data_ptr, const double max_path_velocity)
{
  const auto & bpp_param_ptr = common_data_ptr->bpp_param_ptr;
  const auto & lc_param_ptr = common_data_ptr->lc_param_ptr;

  const auto global_max_lon_acc = bpp_param_ptr->max_acc;
  const auto lc_max_lon_acc = lc_param_ptr->max_longitudinal_acc;
  const auto max_lon_acc = std::min(global_max_lon_acc, lc_max_lon_acc);

  const auto prepare_duration = lc_param_ptr->lane_change_prepare_duration;

  const auto global_max_vel = bpp_param_ptr->max_vel;
  const auto max_vel = std::min(max_path_velocity, global_max_vel);

  const auto current_vel = common_data_ptr->get_ego_speed();
  const auto acc = calc_acceleration(current_vel, max_vel, prepare_duration);
  return std::clamp(acc, 0.0, max_lon_acc);
}

}  // namespace autoware::behavior_path_planner::utils::lane_change::calculation
