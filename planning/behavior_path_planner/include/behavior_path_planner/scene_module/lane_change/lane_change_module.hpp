// Copyright 2021 Tier IV, Inc.
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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_CHANGE__LANE_CHANGE_MODULE_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_CHANGE__LANE_CHANGE_MODULE_HPP_

#include "behavior_path_planner/scene_module/lane_change/debug.hpp"
#include "behavior_path_planner/scene_module/lane_change/lane_change_module_data.hpp"
#include "behavior_path_planner/scene_module/lane_change/lane_change_path.hpp"
#include "behavior_path_planner/scene_module/scene_module_interface.hpp"
#include "behavior_path_planner/turn_signal_decider.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>

#include <tf2/utils.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace behavior_path_planner
{
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using marker_utils::CollisionCheckDebug;

inline std::string toStr(const LaneChangeStates state)
{
  if (state == LaneChangeStates::Trying) {
    return "Trying";
  } else if (state == LaneChangeStates::Success) {
    return "Success";
  } else if (state == LaneChangeStates::Revert) {
    return "Revert";
  } else if (state == LaneChangeStates::Abort) {
    return "Abort";
  }
  return "UNKNOWN";
}

class LaneChangeModule : public SceneModuleInterface
{
public:
  LaneChangeModule(
    const std::string & name, rclcpp::Node & node,
    std::shared_ptr<LaneChangeParameters> parameters);

  BehaviorModuleOutput run() override;

  bool isExecutionRequested() const override;
  bool isExecutionReady() const override;
  BT::NodeStatus updateState() override;
  BehaviorModuleOutput plan() override;
  BehaviorModuleOutput planWaitingApproval() override;
  CandidateOutput planCandidate() const override;
  void onEntry() override;
  void onExit() override;

  void publishRTCStatus() override
  {
    rtc_interface_left_.publishCooperateStatus(clock_->now());
    rtc_interface_right_.publishCooperateStatus(clock_->now());
  }

  bool isActivated() override
  {
    if (rtc_interface_left_.isRegistered(uuid_left_)) {
      return rtc_interface_left_.isActivated(uuid_left_);
    }
    if (rtc_interface_right_.isRegistered(uuid_right_)) {
      return rtc_interface_right_.isActivated(uuid_right_);
    }
    return false;
  }

  void lockRTCCommand() override
  {
    rtc_interface_left_.lockCommandUpdate();
    rtc_interface_right_.lockCommandUpdate();
  }

  void unlockRTCCommand() override
  {
    rtc_interface_left_.unlockCommandUpdate();
    rtc_interface_right_.unlockCommandUpdate();
  }

private:
  std::shared_ptr<LaneChangeParameters> parameters_;
  LaneChangeStatus status_;
  PathShifter path_shifter_;
  mutable LaneChangeStates current_lane_change_state_;
  mutable std::shared_ptr<LaneChangeAbortPath> abort_path_;
  mutable std::shared_ptr<LaneChangePath> force_lane_change_path_;

  double lane_change_lane_length_{200.0};
  double check_distance_{100.0};

  RTCInterface rtc_interface_left_;
  RTCInterface rtc_interface_right_;
  UUID uuid_left_;
  UUID uuid_right_;
  UUID candidate_uuid_;

  bool is_abort_path_approved_ = false;
  bool is_abort_approval_requested_ = false;

  bool is_activated_ = false;

  void resetParameters();

  void waitApprovalLeft(const double start_distance, const double finish_distance)
  {
    rtc_interface_left_.updateCooperateStatus(
      uuid_left_, isExecutionReady(), start_distance, finish_distance, clock_->now());
    is_waiting_approval_ = true;
  }

  void waitApprovalRight(const double start_distance, const double finish_distance)
  {
    rtc_interface_right_.updateCooperateStatus(
      uuid_right_, isExecutionReady(), start_distance, finish_distance, clock_->now());
    is_waiting_approval_ = true;
  }

  void updateRegisteredRTCStatus(const LaneChangePath & path)
  {
    const auto start_distance_to_path_change = motion_utils::calcSignedArcLength(
      path.path.points, planner_data_->self_pose->pose.position, path.shift_line.start.position);

    const auto finish_distance_to_path_change = motion_utils::calcSignedArcLength(
      path.path.points, planner_data_->self_pose->pose.position, path.shift_line.end.position);

    const auto & start_idx = path.shift_line.start_idx;
    const auto & end_idx = path.shift_line.end_idx;
    const auto lateral_shift =
      path.shifted_path.shift_length.at(end_idx) - path.shifted_path.shift_length.at(start_idx);

    if (lateral_shift > 0.0) {
      rtc_interface_left_.updateCooperateStatus(
        uuid_left_, isExecutionReady(), start_distance_to_path_change,
        finish_distance_to_path_change, clock_->now());
      candidate_uuid_ = uuid_left_;
    } else {
      rtc_interface_right_.updateCooperateStatus(
        uuid_right_, isExecutionReady(), start_distance_to_path_change,
        finish_distance_to_path_change, clock_->now());
      candidate_uuid_ = uuid_right_;
    }

    RCLCPP_WARN_STREAM(
      getLogger(), "Direction is UNKNOWN, start_distance = " << start_distance_to_path_change);
  }

  void updateRTCStatus(const CandidateOutput & candidate)
  {
    if (candidate.lateral_shift > 0.0) {
      rtc_interface_left_.updateCooperateStatus(
        uuid_left_, isExecutionReady(), candidate.start_distance_to_path_change,
        candidate.finish_distance_to_path_change, clock_->now());
      candidate_uuid_ = uuid_left_;
      return;
    }
    if (candidate.lateral_shift < 0.0) {
      rtc_interface_right_.updateCooperateStatus(
        uuid_right_, isExecutionReady(), candidate.start_distance_to_path_change,
        candidate.finish_distance_to_path_change, clock_->now());
      candidate_uuid_ = uuid_right_;
      return;
    }

    RCLCPP_WARN_STREAM(
      getLogger(),
      "Direction is UNKNOWN, start_distance = " << candidate.start_distance_to_path_change);
  }

  void removeRTCStatus() override
  {
    rtc_interface_left_.clearCooperateStatus();
    rtc_interface_right_.clearCooperateStatus();
  }

  void removeCandidateRTCStatus()
  {
    if (rtc_interface_left_.isRegistered(candidate_uuid_)) {
      rtc_interface_left_.removeCooperateStatus(candidate_uuid_);
    } else if (rtc_interface_right_.isRegistered(candidate_uuid_)) {
      rtc_interface_right_.removeCooperateStatus(candidate_uuid_);
    }
  }

  void removePreviousRTCStatusLeft()
  {
    if (rtc_interface_left_.isRegistered(uuid_left_)) {
      rtc_interface_left_.removeCooperateStatus(uuid_left_);
    }
  }

  void removePreviousRTCStatusRight()
  {
    if (rtc_interface_right_.isRegistered(uuid_right_)) {
      rtc_interface_right_.removeCooperateStatus(uuid_right_);
    }
  }

  lanelet::ConstLanelets get_original_lanes() const;
  PathWithLaneId getReferencePath() const;
  lanelet::ConstLanelets getLaneChangeLanes(
    const lanelet::ConstLanelets & current_lanes, const double lane_change_lane_length) const;
  std::pair<bool, bool> getSafePath(
    const lanelet::ConstLanelets & lane_change_lanes, const double check_distance,
    LaneChangePath & safe_path) const;

  void updateLaneChangeStatus();

  bool isSafe() const;
  bool isNearEndOfLane() const;
  bool isCurrentSpeedLow() const;
  bool isAbortConditionSatisfied();
  bool hasFinishedLaneChange() const;

  void append_marker_array(const MarkerArray & marker_array) const;
  void setObjectDebugVisualization() const;
  mutable std::unordered_map<std::string, CollisionCheckDebug> object_debug_;
  mutable std::vector<LaneChangePath> debug_valid_path_;
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_CHANGE__LANE_CHANGE_MODULE_HPP_
