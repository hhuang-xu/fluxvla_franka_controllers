// Copyright 2026 Limx Dynamics
// SPDX-License-Identifier: MIT

// See ruckig_joint_position_controller.h for the controller layout.

#include <fluxvla_franka_controllers/ruckig_joint_position_controller.h>

#include <algorithm>
#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace fluxvla_franka_controllers {

namespace {

constexpr std::array<double, 7> kPandaMaxVelocity = {
    2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100};
constexpr std::array<double, 7> kPandaMaxAcceleration = {
    15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0};
constexpr std::array<double, 7> kPandaMaxJerk = {
    7500.0, 3750.0, 5000.0, 6250.0, 7500.0, 10000.0, 10000.0};

constexpr double kDefaultLimitFraction = 0.8;

std::array<double, 7> defaultWithFraction(const std::array<double, 7>& spec,
                                          double fraction) {
  std::array<double, 7> out{};
  for (size_t i = 0; i < 7; ++i) {
    out[i] = spec[i] * fraction;
  }
  return out;
}

double readLimitFraction(ros::NodeHandle& nh,
                         const std::string& fraction_name,
                         const std::string& percent_name,
                         const std::string& controller_name) {
  double fraction = kDefaultLimitFraction;
  double raw_value = 0.0;
  if (nh.getParam(fraction_name, raw_value)) {
    fraction = raw_value;
  } else if (nh.getParam(percent_name, raw_value)) {
    fraction = raw_value / 100.0;
  } else if (nh.getParam("limit_fraction", raw_value)) {
    fraction = raw_value;
  } else if (nh.getParam("limit_percent", raw_value)) {
    fraction = raw_value / 100.0;
  }

  if (!std::isfinite(fraction) || fraction <= 0.0) {
    ROS_WARN_STREAM(controller_name << ": invalid limit fraction from '"
                                    << fraction_name << "' / '"
                                    << percent_name << "', using "
                                    << kDefaultLimitFraction);
    return kDefaultLimitFraction;
  }
  if (fraction > 1.0) {
    ROS_WARN_STREAM(controller_name << ": limit fraction " << fraction
                                    << " exceeds 100% of Panda spec.");
  }
  return fraction;
}

}  // namespace

bool RuckigJointPositionController::readSevenDoublesParam(
    ros::NodeHandle& nh, const std::string& name,
    std::array<double, 7>& out) {
  std::vector<double> v;
  if (!nh.getParam(name, v)) {
    return false;
  }
  if (v.size() != 7) {
    ROS_ERROR_STREAM("RuckigJointPositionController: parameter '"
                     << name << "' must be of size 7, got " << v.size());
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    if (!std::isfinite(v[i]) || v[i] <= 0.0) {
      ROS_ERROR_STREAM("RuckigJointPositionController: parameter '"
                       << name << "'[" << i
                       << "] must be finite and positive, got " << v[i]);
      return false;
    }
    out[i] = v[i];
  }
  return true;
}

bool RuckigJointPositionController::init(
    hardware_interface::RobotHW* robot_hw, ros::NodeHandle& nh) {
  std::string arm_id;
  if (!nh.getParam("arm_id", arm_id)) {
    ROS_ERROR(
        "RuckigJointPositionController: Could not read parameter arm_id");
    return false;
  }

  if (!nh.getParam("joint_names", joint_names_) || joint_names_.size() != 7) {
    ROS_ERROR(
        "RuckigJointPositionController: Invalid or missing "
        "'joint_names' parameter (need 7 names).");
    return false;
  }

  pos_iface_ = robot_hw->get<hardware_interface::PositionJointInterface>();
  if (pos_iface_ == nullptr) {
    ROS_ERROR(
        "RuckigJointPositionController: Error getting "
        "PositionJointInterface.");
    return false;
  }
  joint_handles_.clear();
  joint_handles_.reserve(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(pos_iface_->getHandle(joint_names_[i]));
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "RuckigJointPositionController: Exception getting joint "
          "handle '"
          << joint_names_[i] << "': " << e.what());
      return false;
    }
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR(
        "RuckigJointPositionController: Could not get "
        "FrankaStateInterface.");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "RuckigJointPositionController: Exception getting Franka state "
        "handle: "
        << e.what());
    return false;
  }

  if (!readSevenDoublesParam(nh, "max_velocity", max_velocity_)) {
    const double fraction = readLimitFraction(
        nh, "max_velocity_fraction", "max_velocity_percent",
        "RuckigJointPositionController");
    max_velocity_ = defaultWithFraction(kPandaMaxVelocity, fraction);
    ROS_INFO_STREAM(
        "RuckigJointPositionController: using max_velocity = "
        << fraction * 100.0 << "% of Panda spec");
  }
  if (!readSevenDoublesParam(nh, "max_acceleration", max_acceleration_)) {
    const double fraction = readLimitFraction(
        nh, "max_acceleration_fraction", "max_acceleration_percent",
        "RuckigJointPositionController");
    max_acceleration_ = defaultWithFraction(kPandaMaxAcceleration, fraction);
    ROS_INFO_STREAM(
        "RuckigJointPositionController: using max_acceleration = "
        << fraction * 100.0 << "% of Panda spec");
  }
  if (!readSevenDoublesParam(nh, "max_jerk", max_jerk_)) {
    const double fraction = readLimitFraction(
        nh, "max_jerk_fraction", "max_jerk_percent",
        "RuckigJointPositionController");
    max_jerk_ = defaultWithFraction(kPandaMaxJerk, fraction);
    ROS_INFO_STREAM("RuckigJointPositionController: using max_jerk = "
                    << fraction * 100.0 << "% of Panda spec");
  }
  for (size_t i = 0; i < 7; ++i) {
    if (max_velocity_[i] > kPandaMaxVelocity[i]) {
      ROS_WARN_STREAM(
          "RuckigJointPositionController: max_velocity["
          << i << "]=" << max_velocity_[i]
          << " exceeds Panda spec " << kPandaMaxVelocity[i]
          << "; libfranka rate_limiter will clip.");
    }
    if (max_acceleration_[i] > kPandaMaxAcceleration[i]) {
      ROS_WARN_STREAM(
          "RuckigJointPositionController: max_acceleration["
          << i << "]=" << max_acceleration_[i]
          << " exceeds Panda spec " << kPandaMaxAcceleration[i]);
    }
    if (max_jerk_[i] > kPandaMaxJerk[i]) {
      ROS_WARN_STREAM(
          "RuckigJointPositionController: max_jerk["
          << i << "]=" << max_jerk_[i]
          << " exceeds Panda spec " << kPandaMaxJerk[i]);
    }
  }

  nh.param<bool>("enable_lookahead", enable_lookahead_, true);
  nh.param<double>("lookahead_delay", lookahead_delay_, 0.08);
  nh.param<double>("buffer_duration", buffer_duration_, 0.5);
  nh.param<double>("target_filter_alpha", target_filter_alpha_, 0.02);
  nh.param<double>("max_interpolation_gap", max_interpolation_gap_, 0.25);

  lookahead_delay_ = std::max(0.0, lookahead_delay_);
  buffer_duration_ = std::max(buffer_duration_, lookahead_delay_ + 0.05);
  target_filter_alpha_ = std::clamp(target_filter_alpha_, 1e-4, 1.0);
  max_interpolation_gap_ = std::max(max_interpolation_gap_, 1e-3);

  otg_ = std::make_unique<ruckig::Ruckig<7>>(0.001);
  for (size_t i = 0; i < 7; ++i) {
    input_.max_velocity[i] = max_velocity_[i];
    input_.max_acceleration[i] = max_acceleration_[i];
    input_.max_jerk[i] = max_jerk_[i];
  }
  input_.synchronization = ruckig::Synchronization::None;

  sub_target_ = nh.subscribe(
      "target_joint_state", 20,
      &RuckigJointPositionController::targetCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  ROS_INFO_STREAM(
      "RuckigJointPositionController initialized for arm '"
      << arm_id << "' with lookahead="
      << (enable_lookahead_ ? lookahead_delay_ : 0.0)
      << " s, buffer=" << buffer_duration_
      << " s, target_alpha=" << target_filter_alpha_);
  return true;
}

void RuckigJointPositionController::starting(
    const ros::Time& /*time*/) {
  franka::RobotState robot_state = state_handle_->getRobotState();
  for (size_t i = 0; i < 7; ++i) {
    input_.current_position[i] = robot_state.q_d[i];
    input_.current_velocity[i] = robot_state.dq_d[i];
    input_.current_acceleration[i] = robot_state.ddq_d[i];

    input_.target_position[i] = robot_state.q_d[i];
    input_.target_velocity[i] = 0.0;
    input_.target_acceleration[i] = 0.0;

    q_filtered_target_[i] = robot_state.q_d[i];
  }
  has_filtered_target_ = true;

  std::lock_guard<std::mutex> lock(target_mutex_);
  target_buffer_.clear();
}

void RuckigJointPositionController::update(
    const ros::Time& time, const ros::Duration& /*period*/) {
  std::array<double, 7> q_raw_target{};
  const ros::Time sample_time = time.isZero() ? ros::Time::now() : time;
  const bool has_raw_target = sampleLookaheadTarget(sample_time, q_raw_target);

  if (has_raw_target) {
    if (!has_filtered_target_) {
      q_filtered_target_ = q_raw_target;
      has_filtered_target_ = true;
    } else {
      for (size_t i = 0; i < 7; ++i) {
        q_filtered_target_[i] =
            target_filter_alpha_ * q_raw_target[i] +
            (1.0 - target_filter_alpha_) * q_filtered_target_[i];
      }
    }
  }

  for (size_t i = 0; i < 7; ++i) {
    input_.target_position[i] = q_filtered_target_[i];
    input_.target_velocity[i] = 0.0;
    input_.target_acceleration[i] = 0.0;
  }

  const ruckig::Result result = otg_->update(input_, output_);
  if (result == ruckig::Result::Working ||
      result == ruckig::Result::Finished) {
    for (size_t i = 0; i < 7; ++i) {
      joint_handles_[i].setCommand(output_.new_position[i]);
    }
    output_.pass_to_input(input_);
  } else {
    ROS_WARN_THROTTLE(
        1.0,
        "RuckigJointPositionController: ruckig returned result %d.",
        static_cast<int>(result));
    for (size_t i = 0; i < 7; ++i) {
      joint_handles_[i].setCommand(input_.current_position[i]);
    }
  }
}

bool RuckigJointPositionController::parseTargetJointState(
    const sensor_msgs::JointState& msg, std::array<double, 7>& out) const {
  if (msg.position.size() < 7) {
    ROS_WARN_THROTTLE(
        2.0,
        "RuckigJointPositionController: target_joint_state has only "
        "%zu positions (need >= 7). Ignored.",
        msg.position.size());
    return false;
  }

  if (!msg.name.empty()) {
    for (size_t i = 0; i < 7; ++i) {
      auto it = std::find(msg.name.begin(), msg.name.end(), joint_names_[i]);
      if (it == msg.name.end()) {
        ROS_WARN_THROTTLE(
            2.0,
            "RuckigJointPositionController: target_joint_state missing "
            "joint '%s'. Ignored.",
            joint_names_[i].c_str());
        return false;
      }
      const size_t idx =
          static_cast<size_t>(std::distance(msg.name.begin(), it));
      if (idx >= msg.position.size()) {
        ROS_WARN_THROTTLE(
            2.0,
            "RuckigJointPositionController: target_joint_state has name "
            "without position for '%s'. Ignored.",
            joint_names_[i].c_str());
        return false;
      }
      out[i] = msg.position[idx];
      if (!std::isfinite(out[i])) {
        ROS_WARN_THROTTLE(
            2.0,
            "RuckigJointPositionController: target_joint_state has "
            "non-finite value for '%s'. Ignored.",
            joint_names_[i].c_str());
        return false;
      }
    }
  } else {
    for (size_t i = 0; i < 7; ++i) {
      out[i] = msg.position[i];
      if (!std::isfinite(out[i])) {
        ROS_WARN_THROTTLE(
            2.0,
            "RuckigJointPositionController: target_joint_state has "
            "non-finite value at index %zu. Ignored.",
            i);
        return false;
      }
    }
  }
  return true;
}

void RuckigJointPositionController::targetCallback(
    const sensor_msgs::JointStateConstPtr& msg) {
  std::array<double, 7> q_new{};
  if (!parseTargetJointState(*msg, q_new)) {
    return;
  }

  const ros::Time receive_time = ros::Time::now();
  const ros::Time stamp =
      msg->header.stamp.isZero() ? receive_time : msg->header.stamp;
  if (stamp.isZero()) {
    ROS_WARN_THROTTLE(
        2.0,
        "RuckigJointPositionController: target_joint_state has no valid "
        "timestamp yet. Ignored.");
    return;
  }

  if (receive_time > stamp &&
      (receive_time - stamp).toSec() >
          buffer_duration_ + lookahead_delay_ + max_interpolation_gap_) {
    ROS_WARN_THROTTLE(
        2.0,
        "RuckigJointPositionController: target_joint_state timestamp is "
        "too old for the lookahead buffer. Ignored.");
    return;
  }

  std::lock_guard<std::mutex> lock(target_mutex_);
  if (!target_buffer_.empty() && stamp <= target_buffer_.back().stamp) {
    ROS_WARN_THROTTLE(
        2.0,
        "RuckigJointPositionController: target_joint_state timestamp "
        "moved backwards. Ignored.");
    return;
  }

  target_buffer_.push_back(TargetSample{stamp, q_new});
  trimBufferLocked(stamp);
}

bool RuckigJointPositionController::sampleLookaheadTarget(
    const ros::Time& time, std::array<double, 7>& out) {
  std::lock_guard<std::mutex> lock(target_mutex_);
  if (target_buffer_.empty()) {
    return false;
  }

  if (!enable_lookahead_) {
    out = target_buffer_.back().position;
    return true;
  }

  const ros::Time play_time = time - ros::Duration(lookahead_delay_);
  if (play_time < target_buffer_.front().stamp) {
    return false;
  }
  if (play_time >= target_buffer_.back().stamp) {
    out = target_buffer_.back().position;
    return true;
  }

  for (size_t i = 1; i < target_buffer_.size(); ++i) {
    const TargetSample& next = target_buffer_[i];
    if (next.stamp < play_time) {
      continue;
    }

    const TargetSample& prev = target_buffer_[i - 1];
    const double gap = (next.stamp - prev.stamp).toSec();
    if (gap <= 0.0) {
      out = next.position;
      return true;
    }
    if (gap > max_interpolation_gap_) {
      out = prev.position;
      return true;
    }

    const double ratio =
        std::clamp((play_time - prev.stamp).toSec() / gap, 0.0, 1.0);
    for (size_t j = 0; j < 7; ++j) {
      out[j] = (1.0 - ratio) * prev.position[j] + ratio * next.position[j];
    }
    return true;
  }

  out = target_buffer_.back().position;
  return true;
}

void RuckigJointPositionController::trimBufferLocked(
    const ros::Time& newest_stamp) {
  const double keep_duration =
      std::max(buffer_duration_, lookahead_delay_ + max_interpolation_gap_);
  const ros::Time cutoff = newest_stamp - ros::Duration(keep_duration);
  while (target_buffer_.size() > 2 && target_buffer_.front().stamp < cutoff) {
    target_buffer_.pop_front();
  }
}

}  // namespace fluxvla_franka_controllers

PLUGINLIB_EXPORT_CLASS(
    fluxvla_franka_controllers::RuckigJointPositionController,
    controller_interface::ControllerBase)
