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

#include "behavior_path_planner/scene_module/lane_change/lane_change_module.hpp"

#include "behavior_path_planner/path_utilities.hpp"
#include "behavior_path_planner/scene_module/lane_change/util.hpp"
#include "behavior_path_planner/scene_module/scene_module_interface.hpp"
#include "behavior_path_planner/turn_signal_decider.hpp"
#include "behavior_path_planner/utilities.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <autoware_auto_perception_msgs/msg/object_classification.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>

#include <fmt/format.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

std::string to_string(const unique_identifier_msgs::msg::UUID & uuid)
{
  std::stringstream ss;
  for (auto i = 0; i < 16; ++i) {
    ss << std::hex << std::setfill('0') << std::setw(2) << +uuid.uuid[i];
  }
  return ss.str();
}

namespace behavior_path_planner
{
using autoware_auto_perception_msgs::msg::ObjectClassification;

LaneChangeModule::LaneChangeModule(
  const std::string & name, rclcpp::Node & node, std::shared_ptr<LaneChangeParameters> parameters)
: SceneModuleInterface{name, node},
  parameters_{std::move(parameters)},
  rtc_interface_left_(&node, "lane_change_left"),
  rtc_interface_right_(&node, "lane_change_right"),
  uuid_left_{generateUUID()},
  uuid_right_{generateUUID()}
{
  steering_factor_interface_ptr_ = std::make_unique<SteeringFactorInterface>(&node, "lane_change");
}

BehaviorModuleOutput LaneChangeModule::run()
{
  const auto tmp_plan = [this]() {
    RCLCPP_DEBUG(getLogger(), "Was waiting approval, and now approved. Do plan().");
    current_state_ = BT::NodeStatus::RUNNING;
    is_activated_ = isActivated();
    auto output = plan();

    // const auto turn_signal_info = output.turn_signal_info;
    // const auto current_pose = planner_data_->self_pose->pose;
    // const double start_distance = motion_utils::calcSignedArcLength(
    //   output.path->points, current_pose.position,
    //   status_.lane_change_path.shift_line.start.position);
    // const double finish_distance = motion_utils::calcSignedArcLength(
    //   output.path->points, current_pose.position,
    //   status_.lane_change_path.shift_line.end.position);

    // const uint16_t steering_factor_direction =
    //   std::invoke([this, &start_distance, &finish_distance, &turn_signal_info]() {
    //     if (turn_signal_info.turn_signal.command == TurnIndicatorsCommand::ENABLE_LEFT) {
    //       waitApprovalLeft(start_distance, finish_distance);
    //       return SteeringFactor::LEFT;
    //     }
    //     if (turn_signal_info.turn_signal.command == TurnIndicatorsCommand::ENABLE_RIGHT) {
    //       waitApprovalRight(start_distance, finish_distance);
    //       return SteeringFactor::RIGHT;
    //     }
    //     return SteeringFactor::UNKNOWN;
    //   });
    // // TODO(tkhmy) add handle status TRYING
    // steering_factor_interface_ptr_->updateSteeringFactor(
    //   {status_.lane_change_path.shift_line.start, status_.lane_change_path.shift_line.end},
    //   {start_distance, finish_distance}, SteeringFactor::LANE_CHANGE, steering_factor_direction,
    //   SteeringFactor::TURNING, "");

    // std::cerr << "reset uuid: " << reinterpret_cast<char*>(&uuid_left_) << ", " <<
    // reinterpret_cast<char*>(&uuid_right_) << ", " << reinterpret_cast<char*>(&candidate_uuid_) <<
    // std::endl;

    return output;
  };

  RCLCPP_INFO(
    getLogger(), "current_lane_change_state_ = %s", toStr(current_lane_change_state_).c_str());

  current_state_ = BT::NodeStatus::RUNNING;

  updateData();

  std::cerr << "name: " << name() << ", isActivated: " << (isActivated() ? "True" : "False")
            << ", isWaitingApproval: " << (isWaitingApproval() ? "True" : "False") << std::endl;

  if (!isWaitingApproval()) {
    // if (isAbortConditionSatisfied()) {
    // }
    fmt::print("isActivated, not waiting approval and running tmp_plan()\n");
    return tmp_plan();
  }

  // module is waiting approval. Check it.
  if (isActivated()) {
    fmt::print("isActivated and running tmp_plan()\n");
    return tmp_plan();
  } else {
    return planWaitingApproval();
  }
}

void LaneChangeModule::resetParameters()
{
  is_abort_path_approved_ = false;
  is_abort_approval_requested_ = false;
  current_lane_change_state_ = LaneChangeStates::Trying;
  abort_path_.reset();
}

void LaneChangeModule::onEntry()
{
  RCLCPP_DEBUG(getLogger(), "LANE_CHANGE onEntry");
  current_state_ = BT::NodeStatus::SUCCESS;
  current_lane_change_state_ = LaneChangeStates::Trying;
  updateLaneChangeStatus();
  // Get arclength to start lane change
  const auto current_pose = planner_data_->self_pose->pose;
  const auto arclength_start =
    lanelet::utils::getArcCoordinates(status_.lane_change_lanes, current_pose);
  status_.start_distance = arclength_start.length;
  resetParameters();
}

void LaneChangeModule::onExit()
{
  clearWaitingApproval();
  removeRTCStatus();
  steering_factor_interface_ptr_->clearSteeringFactors();
  debug_marker_.markers.clear();
  current_state_ = BT::NodeStatus::IDLE;
  resetParameters();
  RCLCPP_DEBUG(getLogger(), "LANE_CHANGE onExit");
}

bool LaneChangeModule::isExecutionRequested() const
{
  if (current_state_ == BT::NodeStatus::RUNNING) {
    return true;
  }

  // Get lane change lanes
  const auto current_lanes = util::getCurrentLanes(planner_data_);
  const auto lane_change_lanes = getLaneChangeLanes(current_lanes, lane_change_lane_length_);

  // Find lane change path
  LaneChangePath selected_path;
  const auto [found_valid_path, found_safe_path] =
    getSafePath(lane_change_lanes, check_distance_, selected_path);

  return found_valid_path;
}

bool LaneChangeModule::isExecutionReady() const
{
  if (current_state_ == BT::NodeStatus::RUNNING) {
    return true;
  }

  // Get lane change lanes
  const auto current_lanes = util::getCurrentLanes(planner_data_);
  const auto lane_change_lanes = getLaneChangeLanes(current_lanes, lane_change_lane_length_);

  LaneChangePath selected_path;
  const auto [found_valid_path, found_safe_path] =
    getSafePath(lane_change_lanes, check_distance_, selected_path);

  return found_safe_path;
}

BT::NodeStatus LaneChangeModule::updateState()
{
  RCLCPP_DEBUG(getLogger(), "LANE_CHANGE updateState");
  if (isAbortConditionSatisfied()) {
    if (isNearEndOfLane() && isCurrentSpeedLow()) {
      current_state_ = BT::NodeStatus::RUNNING;
      return current_state_;
    }

    if (current_lane_change_state_ == LaneChangeStates::Abort && abort_path_) {
      current_state_ = BT::NodeStatus::RUNNING;
      return current_state_;
    }
    current_state_ = BT::NodeStatus::FAILURE;
    return current_state_;
  }

  if (hasFinishedLaneChange()) {
    abort_path_.reset();
    current_state_ = BT::NodeStatus::SUCCESS;
    return current_state_;
  }
  current_state_ = BT::NodeStatus::RUNNING;
  return current_state_;
}

BehaviorModuleOutput LaneChangeModule::plan()
{
  constexpr double RESAMPLE_INTERVAL{1.0};
  fmt::print(
    "[plan], resampling path, with path size = {}\n", status_.lane_change_path.path.points.size());

  auto selected_path = status_.lane_change_path.path;
  RCLCPP_INFO(
    getLogger(), "[plan] current_lane_change_state_ = %s",
    toStr(current_lane_change_state_).c_str());

  if (current_lane_change_state_ == LaneChangeStates::Abort) {
    if (!is_abort_approval_requested_) {
      uuid_left_ = generateUUID();
      uuid_right_ = generateUUID();
      candidate_uuid_ = generateUUID();
      removePreviousRTCStatusLeft();
      removePreviousRTCStatusRight();
      is_abort_approval_requested_ = true;
      RCLCPP_ERROR(getLogger(), "[plan] uuid is reset to request abort approval.");
    } else {
      fmt::print(stderr, "[plan] approval request is done(change uuid). check isActivated(). \n");
      if (isActivated()) {
        is_abort_path_approved_ = true;
        RCLCPP_INFO(
          getLogger(), "[plan] isActivated() is true. set is_abort_path_approved to true.");
        selected_path = abort_path_->path;
      } else {
        selected_path = abort_path_->prev_path;
        RCLCPP_INFO(getLogger(), "[plan] isActivated() is False.");
        waitApproval();
      }
    }
  }

  // path = abort_path_->path;

  fmt::print(stderr, "[plan] setting path to abort path\n");
  auto path = util::resamplePathWithSpline(selected_path, RESAMPLE_INTERVAL);
  generateDrivableArea(path);
  BehaviorModuleOutput output;
  output.path = std::make_shared<PathWithLaneId>(path);
  const auto turn_signal_info = util::getPathTurnSignal(
    status_.current_lanes, status_.lane_change_path.shifted_path,
    status_.lane_change_path.shift_line, planner_data_->self_pose->pose,
    planner_data_->self_odometry->twist.twist.linear.x, planner_data_->parameters);
  output.turn_signal_info.turn_signal.command = turn_signal_info.first.command;

  lane_change_utils::get_turn_signal_info(status_.lane_change_path, &output.turn_signal_info);
  // output.turn_signal_info.signal_distance = turn_signal_info.second;
  return output;
}

CandidateOutput LaneChangeModule::planCandidate() const
{
  CandidateOutput output;

  LaneChangePath selected_path;
  if (current_lane_change_state_ != LaneChangeStates::Abort) {
    // Get lane change lanes
    const auto current_lanes = util::getCurrentLanes(planner_data_);
    const auto lane_change_lanes = getLaneChangeLanes(current_lanes, lane_change_lane_length_);

    [[maybe_unused]] const auto [found_valid_path, found_safe_path] =
      getSafePath(lane_change_lanes, check_distance_, selected_path);
    selected_path.path.header = planner_data_->route_handler->getRouteHeader();
  }

  if (current_lane_change_state_ == LaneChangeStates::Abort) {
    std::cerr << "[planCandidate] abort\n";
    if (abort_path_) {
      if (is_abort_path_approved_) {
        selected_path = *abort_path_;
        std::cerr << "[planCandidate] abort: abort path is used as an output.\n";
      } else {
        fmt::print(
          stderr, "abort path {} force path {}\n", (abort_path_ ? "exist" : "not exist"),
          (force_lane_change_path_ ? "exist" : "non exist"));
        selected_path = status_.lane_change_path;
        std::cerr << "[planCandidate] in the abort status, but not approved. Use previous path. \n";
      }
    }
  }

  if (selected_path.path.points.empty()) {
    return output;
  }

  const auto & start_idx = selected_path.shift_line.start_idx;
  const auto & end_idx = selected_path.shift_line.end_idx;

  output.path_candidate = selected_path.path;
  output.lateral_shift = selected_path.shifted_path.shift_length.at(end_idx) -
                         selected_path.shifted_path.shift_length.at(start_idx);
  output.start_distance_to_path_change = motion_utils::calcSignedArcLength(
    selected_path.path.points, planner_data_->self_pose->pose.position,
    selected_path.shift_line.start.position);
  output.finish_distance_to_path_change = motion_utils::calcSignedArcLength(
    selected_path.path.points, planner_data_->self_pose->pose.position,
    selected_path.shift_line.end.position);

  const uint16_t steering_factor_direction = std::invoke([&output]() {
    if (output.lateral_shift > 0.0) {
      return SteeringFactor::LEFT;
    }
    return SteeringFactor::RIGHT;
  });

  steering_factor_interface_ptr_->updateSteeringFactor(
    {selected_path.shift_line.start, selected_path.shift_line.end},
    {output.start_distance_to_path_change, output.finish_distance_to_path_change},
    SteeringFactor::LANE_CHANGE, steering_factor_direction, SteeringFactor::APPROACHING, "");

  return output;
}

BehaviorModuleOutput LaneChangeModule::planWaitingApproval()
{
  BehaviorModuleOutput out;
  out.path = std::make_shared<PathWithLaneId>(getReferencePath());
  [[maybe_unused]] const auto abort_condition_satisfied = isAbortConditionSatisfied();
  if (current_lane_change_state_ == LaneChangeStates::Abort) {
    generateDrivableArea(abort_path_->prev_path);
    out.path = std::make_shared<PathWithLaneId>(abort_path_->prev_path);
  }
  const auto candidate = planCandidate();
  out.path_candidate = std::make_shared<PathWithLaneId>(candidate.path_candidate);

  fmt::print(stderr, "[planWaitingApproval] planned.\n");
  bool not_waiting_anymore = false;
  if (!not_waiting_anymore) {
    updateRTCStatus(candidate);
    waitApproval();
    fmt::print(stderr, "[planWaitingApproval] is waiting.\n");
  } else {
    clearWaitingApproval();
    removeCandidateRTCStatus();
    fmt::print(stderr, "[planWaitingApproval] not waiting anymore.\n");
  }
  return out;
}

void LaneChangeModule::updateLaneChangeStatus()
{
  const auto current_lanes = util::getCurrentLanes(planner_data_);
  status_.current_lanes = current_lanes;

  // Get lane change lanes
  const auto lane_change_lanes = getLaneChangeLanes(current_lanes, lane_change_lane_length_);
  status_.lane_change_lanes = lane_change_lanes;

  // Find lane change path
  LaneChangePath selected_path;
  const auto [found_valid_path, found_safe_path] =
    getSafePath(lane_change_lanes, check_distance_, selected_path);

  if (found_safe_path) {
    force_lane_change_path_.reset();
  } else {
    force_lane_change_path_ = std::make_shared<LaneChangePath>(selected_path);
  }

  // Update status
  status_.is_safe = found_safe_path;
  status_.lane_change_path = selected_path;
  status_.lane_follow_lane_ids = util::getIds(current_lanes);
  status_.lane_change_lane_ids = util::getIds(lane_change_lanes);

  const auto current_pose = planner_data_->self_pose->pose;
  const auto arclength_start =
    lanelet::utils::getArcCoordinates(status_.lane_change_lanes, current_pose);
  status_.start_distance = arclength_start.length;

  status_.lane_change_path.path.header = planner_data_->route_handler->getRouteHeader();
}

PathWithLaneId LaneChangeModule::getReferencePath() const
{
  PathWithLaneId reference_path;

  const auto & route_handler = planner_data_->route_handler;
  const auto current_pose = planner_data_->self_pose->pose;
  const auto & common_parameters = planner_data_->parameters;

  // Set header
  reference_path.header = route_handler->getRouteHeader();

  const auto current_lanes = util::getCurrentLanes(planner_data_);

  if (current_lanes.empty()) {
    return reference_path;
  }

  if (reference_path.points.empty()) {
    reference_path = util::getCenterLinePath(
      *route_handler, current_lanes, current_pose, common_parameters.backward_path_length,
      common_parameters.forward_path_length, common_parameters);
  }

  const int num_lane_change =
    std::abs(route_handler->getNumLaneToPreferredLane(current_lanes.back()));
  double optional_lengths{0.0};
  const auto isInIntersection = util::checkLaneIsInIntersection(
    *route_handler, reference_path, current_lanes, common_parameters, num_lane_change,
    optional_lengths);
  if (isInIntersection) {
    reference_path = util::getCenterLinePath(
      *route_handler, current_lanes, current_pose, common_parameters.backward_path_length,
      common_parameters.forward_path_length, common_parameters, optional_lengths);
  }
  const double & buffer =
    common_parameters.backward_length_buffer_for_end_of_lane;  // buffer for min_lane_change_length
  const double lane_change_buffer =
    num_lane_change * (common_parameters.minimum_lane_change_length + buffer) + optional_lengths;

  reference_path = util::setDecelerationVelocity(
    *route_handler, reference_path, current_lanes, parameters_->lane_change_prepare_duration,
    lane_change_buffer);

  const auto current_extended_lanes = util::expandLanelets(
    current_lanes, parameters_->drivable_area_left_bound_offset,
    parameters_->drivable_area_right_bound_offset);

  reference_path.drivable_area = util::generateDrivableArea(
    reference_path, current_extended_lanes, common_parameters.drivable_area_resolution,
    common_parameters.vehicle_length, planner_data_);

  return reference_path;
}

lanelet::ConstLanelets LaneChangeModule::getLaneChangeLanes(
  const lanelet::ConstLanelets & current_lanes, const double lane_change_lane_length) const
{
  lanelet::ConstLanelets lane_change_lanes;
  const auto & route_handler = planner_data_->route_handler;
  const auto & minimum_lane_change_length = planner_data_->parameters.minimum_lane_change_length;
  const auto & lane_change_prepare_duration = parameters_->lane_change_prepare_duration;
  const auto current_pose = planner_data_->self_pose->pose;
  const auto current_twist = planner_data_->self_odometry->twist.twist;

  if (current_lanes.empty()) {
    return lane_change_lanes;
  }

  // Get lane change lanes
  lanelet::ConstLanelet current_lane;
  lanelet::utils::query::getClosestLanelet(current_lanes, current_pose, &current_lane);
  const double lane_change_prepare_length =
    std::max(current_twist.linear.x * lane_change_prepare_duration, minimum_lane_change_length);
  lanelet::ConstLanelets current_check_lanes =
    route_handler->getLaneletSequence(current_lane, current_pose, 0.0, lane_change_prepare_length);
  lanelet::ConstLanelet lane_change_lane;
  if (route_handler->getLaneChangeTarget(current_check_lanes, &lane_change_lane)) {
    lane_change_lanes = route_handler->getLaneletSequence(
      lane_change_lane, current_pose, lane_change_lane_length, lane_change_lane_length);
  } else {
    lane_change_lanes.clear();
  }

  return lane_change_lanes;
}

std::pair<bool, bool> LaneChangeModule::getSafePath(
  const lanelet::ConstLanelets & lane_change_lanes, const double check_distance,
  LaneChangePath & safe_path) const
{
  const auto & route_handler = planner_data_->route_handler;
  const auto current_pose = planner_data_->self_pose->pose;
  const auto current_twist = planner_data_->self_odometry->twist.twist;
  const auto & common_parameters = planner_data_->parameters;

  const auto current_lanes = util::getCurrentLanes(planner_data_);

  if (!lane_change_lanes.empty()) {
    // find candidate paths
    const auto lane_change_paths = lane_change_utils::getLaneChangePaths(
      *route_handler, current_lanes, lane_change_lanes, current_pose, current_twist,
      common_parameters, *parameters_);

    // get lanes used for detection
    lanelet::ConstLanelets check_lanes;
    if (!lane_change_paths.empty()) {
      const auto & longest_path = lane_change_paths.front();
      // we want to see check_distance [m] behind vehicle so add lane changing length
      const double check_distance_with_path =
        check_distance + longest_path.preparation_length + longest_path.lane_change_length;
      check_lanes = route_handler->getCheckTargetLanesFromPath(
        longest_path.path, lane_change_lanes, check_distance_with_path);
    }

    // select valid path
    const LaneChangePaths valid_paths = lane_change_utils::selectValidPaths(
      lane_change_paths, current_lanes, check_lanes, *route_handler->getOverallGraphPtr(),
      current_pose, route_handler->isInGoalRouteSection(current_lanes.back()),
      route_handler->getGoalPose());

    if (valid_paths.empty()) {
      return std::make_pair(false, false);
    }
    debug_valid_path_ = valid_paths;

    // select safe path
    object_debug_.clear();
    const bool found_safe_path = lane_change_utils::selectSafePath(
      valid_paths, current_lanes, check_lanes, planner_data_->dynamic_object, current_pose,
      current_twist, common_parameters, *parameters_, &safe_path, object_debug_);

    if (parameters_->publish_debug_marker) {
      // setObjectDebugVisualization();
    } else {
      debug_marker_.markers.clear();
    }

    return std::make_pair(true, found_safe_path);
  }

  return std::make_pair(false, false);
}

bool LaneChangeModule::isSafe() const { return status_.is_safe; }

bool LaneChangeModule::isNearEndOfLane() const
{
  const auto & current_pose = planner_data_->self_pose->pose;
  const auto & common_parameters = planner_data_->parameters;
  const double threshold = 5 + common_parameters.minimum_lane_change_length;

  return std::max(0.0, util::getDistanceToEndOfLane(current_pose, status_.current_lanes)) <
         threshold;
}

bool LaneChangeModule::isCurrentSpeedLow() const
{
  const auto & current_twist = planner_data_->self_odometry->twist.twist;
  constexpr double threshold_kmph = 10.0;
  return util::l2Norm(current_twist.linear) < threshold_kmph * 1000 / 3600;
}

bool LaneChangeModule::isAbortConditionSatisfied()
{
  std::cerr << "isAbortConditionSatisfied() -> abort condition satisfied\n";
  const auto & route_handler = planner_data_->route_handler;
  const auto current_pose = planner_data_->self_pose->pose;
  const auto current_twist = planner_data_->self_odometry->twist.twist;
  const auto & dynamic_objects = planner_data_->dynamic_object;
  const auto & common_parameters = planner_data_->parameters;

  const auto & current_lanes = status_.current_lanes;

  // check abort enable flag
  if (!parameters_->enable_abort_lane_change) {
    current_lane_change_state_ = LaneChangeStates::Success;
    return false;
  }

  if (!is_activated_) {
    current_lane_change_state_ = LaneChangeStates::Success;
    std::cerr << "no activated\n";
    return false;
  }

  // find closest lanelet in original lane
  lanelet::ConstLanelet closest_lanelet{};
  auto clock{rclcpp::Clock{RCL_ROS_TIME}};
  if (!lanelet::utils::query::getClosestLanelet(current_lanes, current_pose, &closest_lanelet)) {
    RCLCPP_ERROR_THROTTLE(
      getLogger(), clock, 1000,
      "Failed to find closest lane! Lane change aborting function is not working!");
    return false;
  }

  const auto path = status_.lane_change_path;
  fmt::print(
    "isAbortConditionSatisfied() -> status_.lane_change_path.path.points.size() = {}\n",
    status_.lane_change_path.path.points.size());
  // check if lane change path is still safe
  Pose ego_pose_before_collision;
  const bool is_path_safe =
    std::invoke([this, &route_handler, &dynamic_objects, &path, &current_lanes, &current_pose,
                 &current_twist, &common_parameters, &ego_pose_before_collision]() {
      constexpr double check_distance = 100.0;
      // get lanes used for detection
      const double check_distance_with_path =
        check_distance + path.preparation_length + path.lane_change_length;
      const auto check_lanes = route_handler->getCheckTargetLanesFromPath(
        path.path, status_.lane_change_lanes, check_distance_with_path);

      std::unordered_map<std::string, CollisionCheckDebug> debug_data;

      const size_t current_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
        path.path.points, current_pose, common_parameters.ego_nearest_dist_threshold,
        common_parameters.ego_nearest_yaw_threshold);
      return lane_change_utils::isLaneChangePathSafe(
        path.path, current_lanes, check_lanes, dynamic_objects, current_pose, current_seg_idx,
        current_twist, common_parameters, *parameters_, ego_pose_before_collision, debug_data,
        false, status_.lane_change_path.acceleration);
    });

  // abort only if velocity is low or vehicle pose is close enough
  if (!is_path_safe) {
    // check vehicle velocity thresh
    const bool is_velocity_low =
      util::l2Norm(current_twist.linear) < parameters_->abort_lane_change_velocity_thresh;

    // check if vehicle is within lane
    const bool is_within_original_lane =
      std::invoke([&current_lanes, current_pose, common_parameters]() {
        const auto lane_length = lanelet::utils::getLaneletLength2d(current_lanes);
        const auto lane_poly =
          lanelet::utils::getPolygonFromArcLength(current_lanes, 0, lane_length);
        const auto vehicle_poly = util::getVehiclePolygon(
          current_pose, common_parameters.vehicle_width, common_parameters.base_link2front);
        return boost::geometry::within(
          lanelet::utils::to2D(vehicle_poly).basicPolygon(),
          lanelet::utils::to2D(lane_poly).basicPolygon());
      });

    if (is_velocity_low && is_within_original_lane) {
      current_lane_change_state_ = LaneChangeStates::Revert;
      return true;
    }

    // check distance from original lane's centerline
    const bool is_distance_small = std::invoke([this, &closest_lanelet, &current_pose]() {
      const auto centerline2d =
        lanelet::utils::to2D(closest_lanelet.centerline()).basicLineString();
      lanelet::BasicPoint2d vehicle_pose2d(current_pose.position.x, current_pose.position.y);
      const double distance = lanelet::geometry::distance2d(centerline2d, vehicle_pose2d);
      return distance < parameters_->abort_lane_change_distance_thresh;
    });

    // check angle thresh from original lane
    const bool is_angle_diff_small = std::invoke([this, closest_lanelet, &current_pose]() {
      const double lane_angle =
        lanelet::utils::getLaneletAngle(closest_lanelet, current_pose.position);
      const double vehicle_yaw = tf2::getYaw(current_pose.orientation);
      const double yaw_diff = tier4_autoware_utils::normalizeRadian(lane_angle - vehicle_yaw);
      return std::abs(yaw_diff) < parameters_->abort_lane_change_angle_thresh;
    });

    if (is_distance_small && is_angle_diff_small) {
      current_lane_change_state_ = LaneChangeStates::Revert;
      return true;
    }
    auto clock{rclcpp::Clock{RCL_ROS_TIME}};
    RCLCPP_WARN_STREAM_THROTTLE(
      getLogger(), clock, 1000,
      "DANGER!!! Path is not safe anymore, but it is too late to abort! Please be cautious");
    fmt::print(
      stderr,
      "[isAbortConditionSatisfied] Cancel not possible, change lane change state to abort\n");
    current_lane_change_state_ = LaneChangeStates::Abort;

    ShiftLine shift_line;
    const auto abort_path = lane_change_utils::get_abort_paths(
      planner_data_, path, ego_pose_before_collision, shift_line);

    append_marker_array(
      marker_utils::lane_change_markers::show_shift_line(shift_line, "abort_shift", 0L));
    if (abort_path) {
      if (!abort_path_) {
        abort_path_ = std::make_shared<LaneChangeAbortPath>(*abort_path);
      }
    }
    return true;
  }

  std::cerr << "[isAbortConditionSatisfied] No need cancel and abort, the shifted path is safe\n";
  return false;
}

bool LaneChangeModule::hasFinishedLaneChange() const
{
  const auto & current_pose = planner_data_->self_pose->pose;
  const auto arclength_current =
    lanelet::utils::getArcCoordinates(status_.lane_change_lanes, current_pose);
  const double travel_distance = arclength_current.length - status_.start_distance;
  const double finish_distance = status_.lane_change_path.preparation_length +
                                 status_.lane_change_path.lane_change_length +
                                 parameters_->lane_change_finish_judge_buffer;
  return travel_distance > finish_distance;
}

void LaneChangeModule::append_marker_array(const MarkerArray & marker_array) const
{
  tier4_autoware_utils::appendMarkerArray(marker_array, &debug_marker_);
}

void LaneChangeModule::setObjectDebugVisualization() const
{
  using marker_utils::lane_change_markers::showAllValidLaneChangePath;
  using marker_utils::lane_change_markers::showLerpedPose;
  using marker_utils::lane_change_markers::showObjectInfo;
  using marker_utils::lane_change_markers::showPolygon;
  using marker_utils::lane_change_markers::showPolygonPose;

  debug_marker_.markers.clear();
  append_marker_array(showObjectInfo(object_debug_, "object_debug_info"));
  append_marker_array(showLerpedPose(object_debug_, "lerp_pose_before_true"));
  append_marker_array(showPolygonPose(object_debug_, "expected_pose"));
  append_marker_array(showPolygon(object_debug_, "lerped_polygon"));
  append_marker_array(showAllValidLaneChangePath(debug_valid_path_, "lane_change_valid_paths"));
}

lanelet::ConstLanelets LaneChangeModule::get_original_lanes() const
{
  const auto & route_handler = planner_data_->route_handler;
  const auto current_pose = status_.lane_change_path.shifted_path.path.points.front().point.pose;
  const auto & common_parameters = planner_data_->parameters;

  lanelet::ConstLanelet current_lane;
  if (!route_handler->getClosestLaneletWithinRoute(current_pose, &current_lane)) {
    // RCLCPP_ERROR(getLogger(), "failed to find closest lanelet within route!!!");
    std::cerr << "failed to find closest lanelet within route!!!" << std::endl;
    return {};  // TODO(Horibe) what should be returned?
  }

  // For current_lanes with desired length
  return route_handler->getLaneletSequence(
    current_lane, current_pose, common_parameters.backward_path_length,
    common_parameters.forward_path_length);
}

void LaneChangeModule::generateDrivableArea(PathWithLaneId & path) const
{
  lanelet::ConstLanelets lanes;
  lanes.reserve(status_.current_lanes.size() + status_.lane_change_lanes.size());
  lanes.insert(lanes.end(), status_.current_lanes.begin(), status_.current_lanes.end());
  lanes.insert(lanes.end(), status_.lane_change_lanes.begin(), status_.lane_change_lanes.end());
  lanes = util::expandLanelets(
    lanes, parameters_->drivable_area_left_bound_offset,
    parameters_->drivable_area_right_bound_offset);
  const auto & common_parameters = planner_data_->parameters;
  const double & resolution = common_parameters.drivable_area_resolution;
  path.drivable_area = util::generateDrivableArea(
    path, lanes, resolution, common_parameters.vehicle_length, planner_data_);
}
}  // namespace behavior_path_planner
