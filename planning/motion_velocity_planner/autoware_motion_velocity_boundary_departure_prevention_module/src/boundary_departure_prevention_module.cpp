// Copyright 2025 TIER IV, Inc. All rights reserved.
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

#include "boundary_departure_prevention_module.hpp"

#include "debug.hpp"
#include "utils.hpp"

#include <autoware/boundary_departure_checker/utils.hpp>
#include <magic_enum.hpp>

#include <fmt/format.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::motion_velocity_planner
{

void BoundaryDeparturePreventionModule::init(
  [[maybe_unused]] rclcpp::Node & node, [[maybe_unused]] const std::string & module_name)
{
  fmt::print("Running Boundary Departure Prevention Module\n");
  module_name_ = module_name;
  clock_ptr_ = node.get_clock();
  logger_ = node.get_logger();

  node_param_ = param::NodeParam(node);
  subscribe_topics(node);
  publish_topics(node);
}

void BoundaryDeparturePreventionModule::update_parameters(
  [[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters)
{
}

void BoundaryDeparturePreventionModule::subscribe_topics(rclcpp::Node & node)
{
  fmt::print("Subscribing\n");
  sub_ego_pred_traj_ = node.create_subscription<Trajectory>(
    "/control/trajectory_follower/lateral/predicted_trajectory", rclcpp::QoS{1},
    [&](const Trajectory::ConstSharedPtr msg) { ego_pred_traj_ptr_ = msg; });

  sub_op_mode_state_ = node.create_subscription<OperationModeState>(
    "~/api/operation_mode/state", rclcpp::QoS{1},
    [this](const OperationModeState::ConstSharedPtr msg) { op_mode_state_ptr_ = msg; });
}

void BoundaryDeparturePreventionModule::publish_topics(rclcpp::Node & node)
{
  const std::string ns = "boundary_departure_prevention";
  debug_publisher_ =
    node.create_publisher<visualization_msgs::msg::MarkerArray>("~/" + ns + "/debug_markers", 1);
}

VelocityPlanningResult BoundaryDeparturePreventionModule::plan(
  [[maybe_unused]] const TrajectoryPoints & raw_trajectory_points,
  [[maybe_unused]] const TrajectoryPoints & smoothed_trajectory_points,
  const std::shared_ptr<const PlannerData> planner_data)
{
  fmt::print("Running Boundary Departure Prevention Module is running\n");
  VelocityPlanningResult result;
  [[maybe_unused]] const auto & curr_pose = planner_data->current_odometry.pose;
  [[maybe_unused]] const auto & curr_twist = planner_data->current_odometry.twist.twist;
  [[maybe_unused]] const auto & vehicle_info = planner_data->vehicle_info_;

  [[maybe_unused]] auto trajectory =
    trajectory::Trajectory<TrajectoryPoint>::Builder{}.build(raw_trajectory_points);

  if (!ego_pred_traj_ptr_) {
    fmt::print("Invalid ego pred path ptr\n");
    return {};
  }

  const auto output_opt = plan(
    planner_data->current_odometry.pose, ego_pred_traj_ptr_->points, vehicle_info,
    node_param_.pred_path_footprint.scale, *planner_data->route_handler->getLaneletMapPtr(),
    node_param_.boundary_types_to_detect, node_param_);
  if (!output_opt) {
    fmt::print("Invalid output\n");
    return {};
  }

  if (debug_publisher_) {
    debug_publisher_->publish(
      debug::create_debug_marker_array(*output_opt, clock_ptr_, curr_pose.pose.position.z));
  }
  return {};
}

std::optional<Output> BoundaryDeparturePreventionModule::plan(
  const PoseWithCovariance & pose_with_covariance, const TrajectoryPoints & ego_pred_traj,
  const VehicleInfo & vehicle_info, const double footprint_margin_scale,
  const lanelet::LaneletMap & lanelet_map,
  const std::vector<std::string> & boundary_types_to_detect, const param::NodeParam & param)
{
  Output output;

  autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;

  const auto footprints_with_pose = utils::create_vehicle_footprints(
    pose_with_covariance, ego_pred_traj, vehicle_info, footprint_margin_scale);
  output.processing_time_map["create_vehicle_footprint"] = stop_watch.toc(true);

  if (!footprints_with_pose) {
    fmt::print("Invalid footprints\n");
    return std::nullopt;
  }

  output.ego_footprints_sides = utils::get_ego_footprints_sides(*footprints_with_pose);
  output.processing_time_map["get_ego_footprints_sides"] = stop_watch.toc(true);

  output.side_near_boundary = lane_departure_checker::utils::get_closest_boundary_from_side(
    lanelet_map, output.ego_footprints_sides, boundary_types_to_detect);
  output.processing_time_map["get_closest_boundary_from_side"] = stop_watch.toc(true);
  fmt::print("{}\n", output.processing_time_map["get_closest_boundary_from_side"]);

  const auto departure_status = utils::check_departure_status(output.side_near_boundary, param);

  fmt::print("{}\n", magic_enum::enum_name(departure_status));

  return output;
}

bool BoundaryDeparturePreventionModule::is_data_ready(
  [[maybe_unused]] std::unordered_map<std::string, double> & processing_times)
{
  if (!ego_pred_traj_ptr_) {
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_ptr_, throttle_duration_ms, "waiting for predicted trajectory...");
    return false;
  }

  if (!op_mode_state_ptr_) {
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_ptr_, throttle_duration_ms, "waiting for operation mode state...");
    return false;
  }

  return true;
}

bool BoundaryDeparturePreventionModule::is_data_valid() const
{
  if (ego_pred_traj_ptr_->points.empty()) {
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_ptr_, throttle_duration_ms, "empty predicted trajectory...");
    return false;
  }
  return true;
}

bool BoundaryDeparturePreventionModule::is_data_timeout(const Odometry & odom) const
{
  const auto now = clock_ptr_->now();
  const auto time_diff_s = (rclcpp::Time(odom.header.stamp) - now).seconds();
  constexpr auto th_pose_timeout_s = 1.0;

  if (time_diff_s > th_pose_timeout_s) {
    RCLCPP_INFO_THROTTLE(logger_, *clock_ptr_, throttle_duration_ms, "pose timeout...");
    return true;
  }

  return false;
}
}  // namespace autoware::motion_velocity_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::motion_velocity_planner::BoundaryDeparturePreventionModule,
  autoware::motion_velocity_planner::PluginModuleInterface)
