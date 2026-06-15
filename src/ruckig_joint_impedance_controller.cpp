// Copyright 2026 Limx Dynamics
// SPDX-License-Identifier: MIT

// See ruckig_joint_impedance_controller.h for the controller layout.

#include <fluxvla_franka_controllers/ruckig_joint_impedance_controller.h>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace fluxvla_franka_controllers {

namespace {

constexpr size_t kNumJoints = 7;

constexpr std::array<double, kNumJoints> kPandaMaxVelocity = {
    2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100};
constexpr std::array<double, kNumJoints> kPandaMaxAcceleration = {
    15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0};
constexpr std::array<double, kNumJoints> kPandaMaxJerk = {
    7500.0, 3750.0, 5000.0, 6250.0, 7500.0, 10000.0, 10000.0};

constexpr std::array<double, kNumJoints> kSyncMaxVelocity = {
    2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5};
constexpr std::array<double, kNumJoints> kSyncMaxAcceleration = {
    5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0};

constexpr double kDefaultLimitFraction = 0.8;
constexpr double kLookaheadDelay = 0.03;
constexpr double kBufferDuration = 0.5;
constexpr double kMaxInterpolationGap = 0.25;
constexpr double kTargetFilterAlpha = 0.70;
constexpr double kTargetTimeout = 0.5;
constexpr double kSyncMotionSpeedFactor = 0.2;
constexpr double kDeltaQMotionFinished = 1e-6;

std::array<double, kNumJoints> defaultWithFraction(
    const std::array<double, kNumJoints>& spec, double fraction) {
  std::array<double, kNumJoints> out{};
  for (size_t i = 0; i < kNumJoints; ++i) {
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

int sign(double value) {
  return (value > 0.0) - (value < 0.0);
}

}  // namespace

bool RuckigJointImpedanceController::readSevenDoublesParam(
    ros::NodeHandle& nh, const std::string& name,
    std::array<double, kJoints>& out) const {
  std::vector<double> values;
  if (!nh.getParam(name, values)) {
    return false;
  }
  if (values.size() != kJoints) {
    ROS_ERROR_STREAM("RuckigJointImpedanceController: parameter '"
                     << name << "' must be of size " << kJoints << ", got "
                     << values.size());
    return false;
  }
  for (size_t i = 0; i < kJoints; ++i) {
    if (!std::isfinite(values[i]) || values[i] <= 0.0) {
      ROS_ERROR_STREAM("RuckigJointImpedanceController: parameter '"
                       << name << "'[" << i
                       << "] must be finite and positive, got " << values[i]);
      return false;
    }
    out[i] = values[i];
  }
  return true;
}

bool RuckigJointImpedanceController::init(
    hardware_interface::RobotHW* robot_hw, ros::NodeHandle& nh) {
  std::string arm_id;
  if (!nh.getParam("arm_id", arm_id)) {
    ROS_ERROR("RuckigJointImpedanceController: Could not read parameter arm_id");
    return false;
  }

  if (!nh.getParam("joint_names", joint_names_) ||
      joint_names_.size() != kJoints) {
    ROS_ERROR(
        "RuckigJointImpedanceController: Invalid or missing 'joint_names' "
        "parameter (need 7 names).");
    return false;
  }

  if (!readSevenDoublesParam(nh, "k_gains", k_gains_) ||
      !readSevenDoublesParam(nh, "d_gains", d_gains_)) {
    return false;
  }

  nh.param<double>("k_alpha", k_alpha_, 0.99);
  if (!std::isfinite(k_alpha_) || k_alpha_ < 0.0 || k_alpha_ > 1.0) {
    ROS_ERROR(
        "RuckigJointImpedanceController: k_alpha must be in the range [0, 1].");
    return false;
  }
  nh.param<bool>("sync_after_activation", sync_after_activation_, true);
  nh.param<double>("coriolis_factor", coriolis_factor_, 1.0);

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR(
        "RuckigJointImpedanceController: Could not get FrankaModelInterface.");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "RuckigJointImpedanceController: Exception getting Franka model "
        "handle: "
        << e.what());
    return false;
  }

  effort_iface_ = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_iface_ == nullptr) {
    ROS_ERROR(
        "RuckigJointImpedanceController: Error getting EffortJointInterface.");
    return false;
  }
  joint_handles_.clear();
  joint_handles_.reserve(kJoints);
  for (size_t i = 0; i < kJoints; ++i) {
    try {
      joint_handles_.push_back(effort_iface_->getHandle(joint_names_[i]));
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "RuckigJointImpedanceController: Exception getting joint handle '"
          << joint_names_[i] << "': " << e.what());
      return false;
    }
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR(
        "RuckigJointImpedanceController: Could not get FrankaStateInterface.");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "RuckigJointImpedanceController: Exception getting Franka state "
        "handle: "
        << e.what());
    return false;
  }

  if (!readSevenDoublesParam(nh, "max_velocity", max_velocity_)) {
    const double fraction = readLimitFraction(
        nh, "max_velocity_fraction", "max_velocity_percent",
        "RuckigJointImpedanceController");
    max_velocity_ = defaultWithFraction(kPandaMaxVelocity, fraction);
  }
  if (!readSevenDoublesParam(nh, "max_acceleration", max_acceleration_)) {
    const double fraction = readLimitFraction(
        nh, "max_acceleration_fraction", "max_acceleration_percent",
        "RuckigJointImpedanceController");
    max_acceleration_ = defaultWithFraction(kPandaMaxAcceleration, fraction);
  }
  if (!readSevenDoublesParam(nh, "max_jerk", max_jerk_)) {
    const double fraction =
        readLimitFraction(nh, "max_jerk_fraction", "max_jerk_percent",
                          "RuckigJointImpedanceController");
    max_jerk_ = defaultWithFraction(kPandaMaxJerk, fraction);
  }

  otg_ = std::make_unique<ruckig::Ruckig<kJoints>>(0.001);
  for (size_t i = 0; i < kJoints; ++i) {
    input_.max_velocity[i] = max_velocity_[i];
    input_.max_acceleration[i] = max_acceleration_[i];
    input_.max_jerk[i] = max_jerk_[i];
  }
  input_.synchronization = ruckig::Synchronization::None;

  sub_target_ =
      nh.subscribe("target_joint_state", 20,
                   &RuckigJointImpedanceController::targetCallback, this,
                   ros::TransportHints().reliable().tcpNoDelay());
  state_pub_ = nh.advertise<std_msgs::String>("state", 1, true);
  publishState("INACTIVE", true);

  return true;
}

void RuckigJointImpedanceController::starting(const ros::Time& time) {
  const ros::Time start_time = time.isZero() ? ros::Time::now() : time;
  franka::RobotState robot_state = state_handle_->getRobotState();

  std::array<double, kJoints> q{};
  std::array<double, kJoints> dq{};
  for (size_t i = 0; i < kJoints; ++i) {
    q[i] = robot_state.q[i];
    dq[i] = robot_state.dq[i];
    dq_filtered_[i] = dq[i];
    q_filtered_target_[i] = q[i];
    q_goal_last_[i] = q[i];
    q_hold_[i] = q[i];
  }
  has_filtered_target_ = true;
  resetRuckigReference(q, dq);

  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    target_buffer_.clear();
    has_first_valid_target_ = false;
    last_target_receive_time_ = ros::Time(0);
  }

  motion_generator_initialized_ = false;
  move_to_start_position_finished_ = !sync_after_activation_;
  sync_start_time_ = start_time;

  publishState(sync_after_activation_ ? "SYNCING" : "FOLLOWING");
}

void RuckigJointImpedanceController::stopping(const ros::Time& /*time*/) {
  publishState("INACTIVE");
}

void RuckigJointImpedanceController::update(
    const ros::Time& time, const ros::Duration& /*period*/) {
  const ros::Time sample_time = time.isZero() ? ros::Time::now() : time;
  franka::RobotState robot_state = state_handle_->getRobotState();

  std::array<double, kJoints> q{};
  std::array<double, kJoints> dq{};
  std::array<double, kJoints> tau_j_d{};
  for (size_t i = 0; i < kJoints; ++i) {
    q[i] = robot_state.q[i];
    dq[i] = robot_state.dq[i];
    tau_j_d[i] = robot_state.tau_J_d[i];
  }

  std::array<double, kJoints> q_goal = q_goal_last_;

  if (sync_after_activation_ && !move_to_start_position_finished_) {
    std::array<double, kJoints> first_target{};
    bool has_first_target = false;
    {
      std::lock_guard<std::mutex> lock(target_mutex_);
      has_first_target = has_first_valid_target_;
      first_target = first_valid_target_;
    }

    if (!motion_generator_initialized_) {
      if (!has_first_target) {
        writePdTorque(q_hold_, q, dq, tau_j_d);
        return;
      }
      initializeSyncMotion(q, first_target, sample_time);
    }

    const double trajectory_time = (sample_time - sync_start_time_).toSec();
    move_to_start_position_finished_ =
        calculateSyncDesiredValues(trajectory_time, q_goal);

    if (move_to_start_position_finished_) {
      std::array<double, kJoints> zero_dq{};
      resetRuckigReference(q_goal, zero_dq);
      q_filtered_target_ = first_target;
      has_filtered_target_ = true;
      publishState("FOLLOWING");
    }
  } else {
    publishState("FOLLOWING");

    std::array<double, kJoints> q_raw_target{};
    if (sampleLookaheadTarget(sample_time, q_raw_target)) {
      if (!has_filtered_target_) {
        q_filtered_target_ = q_raw_target;
        has_filtered_target_ = true;
      } else {
        for (size_t i = 0; i < kJoints; ++i) {
          q_filtered_target_[i] =
              kTargetFilterAlpha * q_raw_target[i] +
              (1.0 - kTargetFilterAlpha) * q_filtered_target_[i];
        }
      }

      for (size_t i = 0; i < kJoints; ++i) {
        input_.target_position[i] = q_filtered_target_[i];
        input_.target_velocity[i] = 0.0;
        input_.target_acceleration[i] = 0.0;
      }

      const ruckig::Result result = otg_->update(input_, output_);
      if (result == ruckig::Result::Working ||
          result == ruckig::Result::Finished) {
        for (size_t i = 0; i < kJoints; ++i) {
          q_goal[i] = output_.new_position[i];
        }
        output_.pass_to_input(input_);
      } else {
        ROS_WARN_THROTTLE(
            1.0,
            "RuckigJointImpedanceController: ruckig returned result %d; "
            "holding the last reference.",
            static_cast<int>(result));
      }
    }
  }

  q_goal_last_ = q_goal;
  writePdTorque(q_goal, q, dq, tau_j_d);
}

bool RuckigJointImpedanceController::parseTargetJointState(
    const sensor_msgs::JointState& msg,
    std::array<double, kJoints>& out) const {
  if (msg.position.size() < kJoints) {
    ROS_WARN_THROTTLE(
        2.0,
        "RuckigJointImpedanceController: target_joint_state has only %zu "
        "positions (need >= 7). Ignored.",
        msg.position.size());
    return false;
  }

  if (!msg.name.empty()) {
    for (size_t i = 0; i < kJoints; ++i) {
      auto it = std::find(msg.name.begin(), msg.name.end(), joint_names_[i]);
      if (it == msg.name.end()) {
        ROS_WARN_THROTTLE(
            2.0,
            "RuckigJointImpedanceController: target_joint_state missing joint "
            "'%s'. Ignored.",
            joint_names_[i].c_str());
        return false;
      }
      const size_t idx =
          static_cast<size_t>(std::distance(msg.name.begin(), it));
      if (idx >= msg.position.size()) {
        ROS_WARN_THROTTLE(
            2.0,
            "RuckigJointImpedanceController: target_joint_state has name "
            "without position for '%s'. Ignored.",
            joint_names_[i].c_str());
        return false;
      }
      out[i] = msg.position[idx];
      if (!std::isfinite(out[i])) {
        ROS_WARN_THROTTLE(
            2.0,
            "RuckigJointImpedanceController: target_joint_state has non-finite "
            "value for '%s'. Ignored.",
            joint_names_[i].c_str());
        return false;
      }
    }
  } else {
    for (size_t i = 0; i < kJoints; ++i) {
      out[i] = msg.position[i];
      if (!std::isfinite(out[i])) {
        ROS_WARN_THROTTLE(
            2.0,
            "RuckigJointImpedanceController: target_joint_state has non-finite "
            "value at index %zu. Ignored.",
            i);
        return false;
      }
    }
  }
  return true;
}

void RuckigJointImpedanceController::targetCallback(
    const sensor_msgs::JointStateConstPtr& msg) {
  std::array<double, kJoints> q_new{};
  if (!parseTargetJointState(*msg, q_new)) {
    return;
  }

  const ros::Time receive_time = ros::Time::now();
  const ros::Time stamp =
      msg->header.stamp.isZero() ? receive_time : msg->header.stamp;
  if (stamp.isZero()) {
    ROS_WARN_THROTTLE(
        2.0,
        "RuckigJointImpedanceController: target_joint_state has no valid "
        "timestamp yet. Ignored.");
    return;
  }

  if (receive_time > stamp &&
      (receive_time - stamp).toSec() >
          kBufferDuration + kLookaheadDelay + kMaxInterpolationGap) {
    ROS_WARN_THROTTLE(
        2.0,
        "RuckigJointImpedanceController: target_joint_state timestamp is too "
        "old for the lookahead buffer. Ignored.");
    return;
  }

  std::lock_guard<std::mutex> lock(target_mutex_);
  if (!target_buffer_.empty() && stamp <= target_buffer_.back().stamp) {
    ROS_WARN_THROTTLE(
        2.0,
        "RuckigJointImpedanceController: target_joint_state timestamp moved "
        "backwards. Ignored.");
    return;
  }

  target_buffer_.push_back(TargetSample{stamp, q_new});
  if (!has_first_valid_target_) {
    first_valid_target_ = q_new;
    has_first_valid_target_ = true;
  }
  last_target_receive_time_ = receive_time;
  trimBufferLocked(stamp);
}

bool RuckigJointImpedanceController::sampleLookaheadTarget(
    const ros::Time& time, std::array<double, kJoints>& out) {
  std::lock_guard<std::mutex> lock(target_mutex_);
  if (target_buffer_.empty()) {
    return false;
  }

  const ros::Time play_time = time - ros::Duration(kLookaheadDelay);
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
    if (gap > kMaxInterpolationGap) {
      out = prev.position;
      return true;
    }

    const double ratio =
        std::clamp((play_time - prev.stamp).toSec() / gap, 0.0, 1.0);
    for (size_t j = 0; j < kJoints; ++j) {
      out[j] = (1.0 - ratio) * prev.position[j] + ratio * next.position[j];
    }
    return true;
  }

  out = target_buffer_.back().position;
  return true;
}

void RuckigJointImpedanceController::trimBufferLocked(
    const ros::Time& newest_stamp) {
  const double keep_duration =
      std::max(kBufferDuration, kLookaheadDelay + kMaxInterpolationGap);
  const ros::Time cutoff = newest_stamp - ros::Duration(keep_duration);
  while (target_buffer_.size() > 2 && target_buffer_.front().stamp < cutoff) {
    target_buffer_.pop_front();
  }
}

void RuckigJointImpedanceController::initializeSyncMotion(
    const std::array<double, kJoints>& q_start,
    const std::array<double, kJoints>& q_goal,
    const ros::Time& start_time) {
  sync_q_start_ = q_start;
  sync_start_time_ = start_time;
  for (size_t i = 0; i < kJoints; ++i) {
    sync_delta_q_[i] = q_goal[i] - q_start[i];
    sync_dq_max_[i] = kSyncMaxVelocity[i] * kSyncMotionSpeedFactor;
    sync_ddq_max_start_[i] =
        kSyncMaxAcceleration[i] * kSyncMotionSpeedFactor;
    sync_ddq_max_goal_[i] = kSyncMaxAcceleration[i] * kSyncMotionSpeedFactor;
  }
  calculateSyncSynchronizedValues();
  motion_generator_initialized_ = true;
}

void RuckigJointImpedanceController::calculateSyncSynchronizedValues() {
  std::array<double, kJoints> dq_max_reach = sync_dq_max_;
  std::array<double, kJoints> t_f{};
  std::array<double, kJoints> delta_t_2_sync{};

  sync_dq_max_sync_.fill(0.0);
  sync_t_1_sync_.fill(0.0);
  sync_t_2_sync_.fill(0.0);
  sync_t_f_sync_.fill(0.0);
  sync_q_1_.fill(0.0);

  for (size_t i = 0; i < kJoints; ++i) {
    if (std::abs(sync_delta_q_[i]) <= kDeltaQMotionFinished) {
      continue;
    }

    const double threshold =
        0.75 * (std::pow(sync_dq_max_[i], 2.0) / sync_ddq_max_start_[i]) +
        0.75 * (std::pow(sync_dq_max_[i], 2.0) / sync_ddq_max_goal_[i]);
    if (std::abs(sync_delta_q_[i]) < threshold) {
      dq_max_reach[i] =
          std::sqrt(4.0 / 3.0 * std::abs(sync_delta_q_[i]) *
                    (sync_ddq_max_start_[i] * sync_ddq_max_goal_[i]) /
                    (sync_ddq_max_start_[i] + sync_ddq_max_goal_[i]));
    }

    const double t_1 = 1.5 * dq_max_reach[i] / sync_ddq_max_start_[i];
    const double delta_t_2 =
        1.5 * dq_max_reach[i] / sync_ddq_max_goal_[i];
    t_f[i] =
        t_1 / 2.0 + delta_t_2 / 2.0 + std::abs(sync_delta_q_[i]) / dq_max_reach[i];
  }

  const double max_t_f = *std::max_element(t_f.begin(), t_f.end());
  for (size_t i = 0; i < kJoints; ++i) {
    if (std::abs(sync_delta_q_[i]) <= kDeltaQMotionFinished) {
      continue;
    }

    const double param_a =
        1.5 / 2.0 * (sync_ddq_max_goal_[i] + sync_ddq_max_start_[i]);
    const double param_b =
        -1.0 * max_t_f * sync_ddq_max_goal_[i] * sync_ddq_max_start_[i];
    const double param_c =
        std::abs(sync_delta_q_[i]) * sync_ddq_max_goal_[i] *
        sync_ddq_max_start_[i];
    double delta = param_b * param_b - 4.0 * param_a * param_c;
    if (delta < 0.0) {
      delta = 0.0;
    }
    sync_dq_max_sync_[i] =
        (-1.0 * param_b - std::sqrt(delta)) / (2.0 * param_a);
    sync_t_1_sync_[i] =
        1.5 * sync_dq_max_sync_[i] / sync_ddq_max_start_[i];
    delta_t_2_sync[i] =
        1.5 * sync_dq_max_sync_[i] / sync_ddq_max_goal_[i];
    sync_t_f_sync_[i] = sync_t_1_sync_[i] / 2.0 +
                        delta_t_2_sync[i] / 2.0 +
                        std::abs(sync_delta_q_[i] / sync_dq_max_sync_[i]);
    sync_t_2_sync_[i] = sync_t_f_sync_[i] - delta_t_2_sync[i];
    sync_q_1_[i] =
        sync_dq_max_sync_[i] * static_cast<double>(sign(sync_delta_q_[i])) *
        (0.5 * sync_t_1_sync_[i]);
  }
}

bool RuckigJointImpedanceController::calculateSyncDesiredValues(
    double time, std::array<double, kJoints>& q_goal) const {
  time = std::max(time, 0.0);
  bool all_finished = true;

  for (size_t i = 0; i < kJoints; ++i) {
    double delta_q_d = 0.0;
    if (std::abs(sync_delta_q_[i]) <= kDeltaQMotionFinished) {
      q_goal[i] = sync_q_start_[i];
      continue;
    }

    const double sign_delta_q = static_cast<double>(sign(sync_delta_q_[i]));
    const double t_d = sync_t_2_sync_[i] - sync_t_1_sync_[i];
    const double delta_t_2_sync = sync_t_f_sync_[i] - sync_t_2_sync_[i];

    if (time < sync_t_1_sync_[i]) {
      delta_q_d = -1.0 / std::pow(sync_t_1_sync_[i], 3.0) *
                  sync_dq_max_sync_[i] * sign_delta_q *
                  (0.5 * time - sync_t_1_sync_[i]) * std::pow(time, 3.0);
    } else if (time < sync_t_2_sync_[i]) {
      delta_q_d = sync_q_1_[i] +
                  (time - sync_t_1_sync_[i]) * sync_dq_max_sync_[i] *
                      sign_delta_q;
    } else if (time < sync_t_f_sync_[i]) {
      delta_q_d =
          sync_delta_q_[i] +
          0.5 *
              (1.0 / std::pow(delta_t_2_sync, 3.0) *
                   (time - sync_t_1_sync_[i] - 2.0 * delta_t_2_sync - t_d) *
                   std::pow((time - sync_t_1_sync_[i] - t_d), 3.0) +
               (2.0 * time - 2.0 * sync_t_1_sync_[i] - delta_t_2_sync -
                2.0 * t_d)) *
              sync_dq_max_sync_[i] * sign_delta_q;
    } else {
      delta_q_d = sync_delta_q_[i];
    }

    q_goal[i] = sync_q_start_[i] + delta_q_d;
    if (time < sync_t_f_sync_[i]) {
      all_finished = false;
    }
  }

  return all_finished;
}

void RuckigJointImpedanceController::resetRuckigReference(
    const std::array<double, kJoints>& q,
    const std::array<double, kJoints>& dq) {
  for (size_t i = 0; i < kJoints; ++i) {
    input_.current_position[i] = q[i];
    input_.current_velocity[i] = dq[i];
    input_.current_acceleration[i] = 0.0;
    input_.target_position[i] = q[i];
    input_.target_velocity[i] = 0.0;
    input_.target_acceleration[i] = 0.0;
    q_goal_last_[i] = q[i];
  }
}

void RuckigJointImpedanceController::publishState(const std::string& state,
                                                  bool force) {
  if (!force && state == current_state_) {
    return;
  }
  current_state_ = state;
  std_msgs::String msg;
  msg.data = state;
  state_pub_.publish(msg);
}

std::array<double, RuckigJointImpedanceController::kJoints>
RuckigJointImpedanceController::calculateModelFeedforward() {
  const std::array<double, kJoints> coriolis = model_handle_->getCoriolis();
  std::array<double, kJoints> tau_model{};
  for (size_t i = 0; i < kJoints; ++i) {
    tau_model[i] = coriolis_factor_ * coriolis[i];
  }
  return tau_model;
}

std::array<double, RuckigJointImpedanceController::kJoints>
RuckigJointImpedanceController::saturateTorqueRate(
    const std::array<double, kJoints>& tau_d_calculated,
    const std::array<double, kJoints>& tau_j_d) const {
  std::array<double, kJoints> tau_d_saturated{};
  for (size_t i = 0; i < kJoints; ++i) {
    const double difference = tau_d_calculated[i] - tau_j_d[i];
    tau_d_saturated[i] =
        tau_j_d[i] +
        std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

void RuckigJointImpedanceController::writePdTorque(
    const std::array<double, kJoints>& q_goal,
    const std::array<double, kJoints>& q,
    const std::array<double, kJoints>& dq,
    const std::array<double, kJoints>& tau_j_d) {
  std::array<double, kJoints> tau = calculateModelFeedforward();
  for (size_t i = 0; i < kJoints; ++i) {
    dq_filtered_[i] = (1.0 - k_alpha_) * dq_filtered_[i] + k_alpha_ * dq[i];
    tau[i] +=
        k_gains_[i] * (q_goal[i] - q[i]) + d_gains_[i] * (-dq_filtered_[i]);
  }

  const std::array<double, kJoints> tau_saturated =
      saturateTorqueRate(tau, tau_j_d);
  for (size_t i = 0; i < kJoints; ++i) {
    joint_handles_[i].setCommand(tau_saturated[i]);
  }
}

}  // namespace fluxvla_franka_controllers

PLUGINLIB_EXPORT_CLASS(fluxvla_franka_controllers::RuckigJointImpedanceController,
                       controller_interface::ControllerBase)
