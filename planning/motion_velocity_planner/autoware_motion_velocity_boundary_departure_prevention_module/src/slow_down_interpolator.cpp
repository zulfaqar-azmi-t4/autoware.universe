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

#include "slow_down_interpolator.hpp"

#include <fmt/format.h>

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

namespace autoware::motion_velocity_planner::experimental::utils
{
tl::expected<SlowDownInterpolator::SlowDownPlan, std::string>
SlowDownInterpolator::get_interp_to_point(
  const double curr_vel, const double lon_dist_to_bound_m, double lat_dist_to_bound_m,
  const SideKey side_key) const
{
  const auto interp_vel_mps = interp_velocity(curr_vel, lat_dist_to_bound_m, side_key);
  const auto exp_decel_mps2 =
    calc_expected_deceleration(curr_vel, interp_vel_mps, lon_dist_to_bound_m);

  fmt::print(
    "vel {}[m/s], lat dist {} [m]\n", autoware_utils_math::mps2kmph(interp_vel_mps),
    lat_dist_to_bound_m);

  const auto max_decel = th_trigger_.th_acc_mps2.max;
  if (std::abs(exp_decel_mps2) > std::abs(max_decel)) {
    return tl::unexpected<std::string>(
      "Expected deceleration for the interpolated velocity exceeded threshold.");
  }

  const auto exp_jerk = interp_jerk(lat_dist_to_bound_m, exp_decel_mps2, side_key);
  const auto exp_vel = calc_new_velocity(curr_vel, exp_decel_mps2, exp_jerk, lon_dist_to_bound_m);

  const auto rel_dist_m =
    std::max((interp_vel_mps * interp_vel_mps - curr_vel * curr_vel) / (2 * exp_decel_mps2), 0.0);

  fmt::print(
    "exp vel {}[m/s], rel dist {} [m]\n", autoware_utils_math::mps2kmph(exp_vel), rel_dist_m);
  const auto expected_stop = calc_expected_stop_distance(curr_vel);

  const auto expected_min_velocity = calc_expected_min_velocity();
  fmt::print("exp stop dist, {:.3f}, exp min vel, {:.3f}\n", *expected_stop, *expected_min_velocity);

  return SlowDownPlan{rel_dist_m, exp_vel, exp_decel_mps2};
}

double SlowDownInterpolator::calc_expected_deceleration(
  const double curr_vel, const double target_vel, const double arclength_to_point_m) const
{
  const auto & th_acc_mps2 = th_trigger_.th_acc_mps2;
  if (arclength_to_point_m < std::numeric_limits<double>::epsilon()) {
    return th_acc_mps2.max;
  }

  const auto raw_decel =
    (target_vel * target_vel - curr_vel * curr_vel) / (2.0 * arclength_to_point_m);
  return std::clamp(raw_decel, th_acc_mps2.max, th_acc_mps2.min);
}

double SlowDownInterpolator::calc_new_velocity(
  const double curr_vel, const double a_target, const double jerk,
  const double lon_dist_to_point_m) const
{
  const auto th_slow_down_vel = th_trigger_.th_vel_mps.min;

  if (curr_vel < th_slow_down_vel) {
    return th_slow_down_vel;
  }

  const double t = lon_dist_to_point_m / std::max(curr_vel, 1e-6);  // avoid div by zero
  const double t_j = std::abs(a_target / std::max(jerk, 1e-6));

  if (t <= t_j) {
    return std::clamp(
      curr_vel + 0.5 * jerk * t * t, th_slow_down_vel,
      std::max(th_slow_down_vel + std::numeric_limits<double>::epsilon(), curr_vel));
  }

  const double delta_v1 = 0.5 * a_target * a_target / jerk;
  const double delta_v2 = a_target * (t - t_j);

  return std::clamp(
    curr_vel + delta_v1 + delta_v2, th_slow_down_vel,
    std::max(th_slow_down_vel + std::numeric_limits<double>::epsilon(), curr_vel));
};

double SlowDownInterpolator::interp_velocity(
  const double curr_vel, const double lat_dist, const SideKey side_key) const
{
  if (curr_vel <= th_trigger_.th_vel_mps.min) {
    return th_trigger_.th_vel_mps.min;
  }

  const auto min_dist = th_trigger_.th_dist_to_boundary_m[side_key].min;
  const auto max_dist = th_trigger_.th_dist_to_boundary_m[side_key].max;
  const auto lat_dist_axis = get_lat_dist_axis(side_key);

  const auto vel_axis = get_vel_axis();
  if (lat_dist >= max_dist) {
    return vel_axis.back();
  }

  if (lat_dist <= min_dist) {
    return vel_axis.front();
  }

  return autoware::interpolation::lerp(lat_dist_axis, vel_axis, lat_dist);
}

std::vector<double> SlowDownInterpolator::get_lat_dist_axis(const SideKey side_key) const
{
  return {
    th_trigger_.th_dist_to_boundary_m[side_key].min,
    th_trigger_.th_dist_to_boundary_m[side_key].max};
}

std::vector<double> SlowDownInterpolator::get_vel_axis() const
{
  return {th_trigger_.th_vel_mps.min, th_trigger_.th_vel_mps.max};
}

double SlowDownInterpolator::interp_jerk(
  const double lat_dist_to_bound_m, const double expected_decel_mps2, const SideKey side_key) const
{
  const auto th_dist_to_boundary_m = th_trigger_.th_dist_to_boundary_m[side_key];
  const auto lat_ratio = (th_dist_to_boundary_m.max - lat_dist_to_bound_m) /
                         (th_dist_to_boundary_m.max - th_dist_to_boundary_m.min);

  const auto th_decel_mps2 = th_trigger_.th_acc_mps2;
  const auto max_decel = std::abs(th_decel_mps2.max);
  const auto min_decel = std::abs(th_decel_mps2.min);
  const auto lon_ratio = (std::abs(expected_decel_mps2) - min_decel) / (max_decel - min_decel);

  const auto combined_ratio = 0.5 * lat_ratio + 0.5 * lon_ratio;

  const auto min_jerk = th_trigger_.th_jerk_mps3.min;
  const auto max_jerk = th_trigger_.th_jerk_mps3.max;
  if (combined_ratio <= 0.0) return min_jerk;
  if (combined_ratio >= 1.0) return max_jerk;

  return autoware::interpolation::lerp(min_jerk, max_jerk, combined_ratio);
}

tl::expected<double, std::string> SlowDownInterpolator::calc_expected_min_velocity() const
{
  const auto t_e = calc_max_accel_jerk_time();
  const auto a_c = th_trigger_.th_acc_mps2.min;
  auto min_vel = std::min(-a_c * t_e, th_trigger_.th_vel_mps.min);

  if (min_vel < 0.0) {
    return tl::make_unexpected("Invalid, expected min vel less than zero.");
  }

  return min_vel;
}

tl::expected<double, std::string> SlowDownInterpolator::calc_expected_stop_distance(
  const double curr_vel) const
{
  /*
    Derivation of jerk-limited stopping distance (no constant accel phase):

    Assume:
      - Jerk j is applied to ramp acceleration from 0 to a_e
      - Time to reach a_e: t_e = a_e / j
      - Velocity during ramp: v(t) = ∫ a(t) dt = ∫ jt dt = 0.5 * j * t^2
      - Distance during ramp: s(t) = ∫ v(t) dt = ∫ (0.5 * j * t^2) dt = (1/6) * j * t^3

    Plug in t_e = a_e / j:
      s_half = (1/6) * j * (a_e / j)^3
             = (1/6) * a_e^3 / j^2

    Full stop has symmetric ramp-in and ramp-out:
      s_total = 2 * s_half
              = (1/3) * a_e^3 / j^2

    Let t_e = a_e / j  →  j = a_e / t_e
    Substitute into equation:
      s_total = (1/3) * a_e * t_e^2
  */

  const auto t_delay = th_trigger_.brake_delay_s;
  const auto t_e = calc_max_accel_jerk_time();
  const auto error_margin = th_trigger_.dist_error_m;
  const auto a_e = th_trigger_.th_acc_mps2.max;
  fmt::print("t_delay: {:.6f} [s]\n", t_delay);
  fmt::print("t_e (a_e / j_e): {:.6f} [s]\n", t_e);
  fmt::print("error_margin: {:.6f} [m]\n", error_margin);
  fmt::print("a_e (max decel): {:.6f} [m/s²]\n", a_e);

  const auto delay_dist = std::max(curr_vel, th_trigger_.th_vel_mps.min) * t_delay;
  const auto jerk_stop_dist = -a_e * t_e * t_e / 3.0;
  fmt::print(
    "curr_vel: {:.6f} [m/s], th_vel_mps.min: {:.6f} [m/s]\n", curr_vel, th_trigger_.th_vel_mps.min);

  fmt::print("delay_dist = max(curr_vel, v_min) * t_delay = {:.6f} [m]\n", delay_dist);
  fmt::print("jerk_stop_dist = 1/3 * a_e * t_e^2 = {:.6f} [m]\n", jerk_stop_dist);

  fmt::print(
    "total_stop_dist = delay_dist + jerk_stop_dist + error_margin = {:.6f} [m]\n",
    delay_dist + jerk_stop_dist + error_margin);

  if (delay_dist < 0.0 || jerk_stop_dist < 0.0 || error_margin < 0.0) {
    return tl::make_unexpected("Invalid, expected values less than zero.");
  }

  return delay_dist + jerk_stop_dist + error_margin;
}

double SlowDownInterpolator::calc_max_accel_jerk_time() const
{
  const auto j_e = th_trigger_.th_jerk_mps3.max;
  const auto a_e = th_trigger_.th_acc_mps2.max;
  return a_e / j_e;
}
}  // namespace autoware::motion_velocity_planner::experimental::utils
