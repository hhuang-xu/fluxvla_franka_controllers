// Copyright 2026 Limx Dynamics
// SPDX-License-Identifier: MIT

// Jerk-limited joint-position controller with input target smoothing.
//
// This controller subscribes to target_joint_state and filters the incoming
// target stream before handing it to Ruckig.

#pragma once

#include <array>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>

#include <franka_hw/franka_state_interface.h>
#include <ruckig/ruckig.hpp>

namespace fluxvla_franka_controllers {

class RuckigJointPositionController
    : public controller_interface::MultiInterfaceController<
          hardware_interface::PositionJointInterface,
          franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw,
            ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration&) override;

 private:
  struct TargetSample {
    ros::Time stamp;
    std::array<double, 7> position;
  };

  void targetCallback(const sensor_msgs::JointStateConstPtr& msg);

  bool readSevenDoublesParam(ros::NodeHandle& nh, const std::string& name,
                             std::array<double, 7>& out);
  bool parseTargetJointState(const sensor_msgs::JointState& msg,
                             std::array<double, 7>& out) const;
  bool sampleLookaheadTarget(const ros::Time& time,
                             std::array<double, 7>& out);
  void trimBufferLocked(const ros::Time& newest_stamp);

  hardware_interface::PositionJointInterface* pos_iface_{nullptr};
  std::vector<hardware_interface::JointHandle> joint_handles_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::vector<std::string> joint_names_;

  std::unique_ptr<ruckig::Ruckig<7>> otg_;
  ruckig::InputParameter<7> input_;
  ruckig::OutputParameter<7> output_;

  // Target samples are written by the ROS callback and sampled by update().
  std::deque<TargetSample> target_buffer_;
  std::mutex target_mutex_;

  std::array<double, 7> q_filtered_target_{};
  bool has_filtered_target_{false};

  // YAML-configurable Ruckig limits.
  std::array<double, 7> max_velocity_{};
  std::array<double, 7> max_acceleration_{};
  std::array<double, 7> max_jerk_{};

  // YAML-configurable input smoothing.
  bool enable_lookahead_{true};
  double lookahead_delay_{0.08};
  double buffer_duration_{0.5};
  double target_filter_alpha_{0.02};
  double max_interpolation_gap_{0.25};

  ros::Subscriber sub_target_;
};

}  // namespace fluxvla_franka_controllers
