// Copyright 2022 TIER IV, Inc.
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
#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_CHANGE__LANE_CHANGE_MODULE_DATA_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_CHANGE__LANE_CHANGE_MODULE_DATA_HPP_

#include "behavior_path_planner/scene_module/lane_change/lane_change_path.hpp"
#include "lanelet2_core/geometry/Lanelet.h"

#include "autoware_auto_planning_msgs/msg/path_point_with_lane_id.hpp"

#include <vector>

namespace behavior_path_planner
{
struct LaneChangeParameters
{
  double lane_change_prepare_duration;
  double lane_changing_duration;
  double minimum_lane_change_prepare_distance;
  double lane_change_finish_judge_buffer;
  double minimum_lane_change_velocity;
  double prediction_time_resolution;
  double maximum_deceleration;
  int lane_change_sampling_num;
  double abort_lane_change_velocity_thresh;
  double abort_lane_change_angle_thresh;
  double abort_lane_change_distance_thresh;
  double prepare_phase_ignore_target_speed_thresh;
  bool enable_abort_lane_change;
  bool enable_collision_check_at_prepare_phase;
  bool use_predicted_path_outside_lanelet;
  bool use_all_predicted_path;
  bool publish_debug_marker;
  // drivable area expansion
  double drivable_area_right_bound_offset;
  double drivable_area_left_bound_offset;
};

struct LaneChangeStatus
{
  PathWithLaneId lane_follow_path;
  LaneChangePath lane_change_path;
  lanelet::ConstLanelets current_lanes;
  lanelet::ConstLanelets lane_change_lanes;
  std::vector<uint64_t> lane_follow_lane_ids;
  std::vector<uint64_t> lane_change_lane_ids;
  bool is_safe;
  double start_distance;
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_CHANGE__LANE_CHANGE_MODULE_DATA_HPP_
