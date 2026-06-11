// Ruckig-smoothed joint impedance controller for Franka.
//
// The controller accepts low-rate sensor_msgs/JointState targets, shapes them
// into a 1 kHz joint reference with a small lookahead buffer and Ruckig, then
// applies the same joint PD torque law used by Franka's follower controller.

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
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <ruckig/ruckig.hpp>

namespace fluxvla_franka_controllers {

class RuckigJointImpedanceController
    : public controller_interface::MultiInterfaceController<
          franka_hw::FrankaModelInterface,
          hardware_interface::EffortJointInterface,
          franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw,
            ros::NodeHandle& node_handle) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override;

 private:
  static constexpr size_t kJoints = 7;

  struct TargetSample {
    ros::Time stamp;
    std::array<double, kJoints> position;
  };

  bool readSevenDoublesParam(ros::NodeHandle& nh, const std::string& name,
                             std::array<double, kJoints>& out) const;
  bool parseTargetJointState(const sensor_msgs::JointState& msg,
                             std::array<double, kJoints>& out) const;
  void targetCallback(const sensor_msgs::JointStateConstPtr& msg);
  bool sampleLookaheadTarget(const ros::Time& time,
                             std::array<double, kJoints>& out);
  void trimBufferLocked(const ros::Time& newest_stamp);

  void initializeSyncMotion(const std::array<double, kJoints>& q_start,
                            const std::array<double, kJoints>& q_goal,
                            const ros::Time& start_time);
  void calculateSyncSynchronizedValues();
  bool calculateSyncDesiredValues(double time,
                                  std::array<double, kJoints>& q_goal) const;

  void resetRuckigReference(const std::array<double, kJoints>& q,
                            const std::array<double, kJoints>& dq);
  void publishState(const std::string& state, bool force = false);
  std::array<double, kJoints> calculateModelFeedforward();
  std::array<double, kJoints> saturateTorqueRate(
      const std::array<double, kJoints>& tau_d_calculated,
      const std::array<double, kJoints>& tau_j_d) const;
  void writePdTorque(const std::array<double, kJoints>& q_goal,
                     const std::array<double, kJoints>& q,
                     const std::array<double, kJoints>& dq,
                     const std::array<double, kJoints>& tau_j_d);

  hardware_interface::EffortJointInterface* effort_iface_{nullptr};
  std::vector<hardware_interface::JointHandle> joint_handles_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<std::string> joint_names_;

  std::unique_ptr<ruckig::Ruckig<kJoints>> otg_;
  ruckig::InputParameter<kJoints> input_;
  ruckig::OutputParameter<kJoints> output_;

  std::deque<TargetSample> target_buffer_;
  std::mutex target_mutex_;

  std::array<double, kJoints> q_filtered_target_{};
  std::array<double, kJoints> q_goal_last_{};
  std::array<double, kJoints> q_hold_{};
  std::array<double, kJoints> first_valid_target_{};
  std::array<double, kJoints> dq_filtered_{};
  bool has_filtered_target_{false};
  bool has_first_valid_target_{false};

  std::array<double, kJoints> k_gains_{};
  std::array<double, kJoints> d_gains_{};
  double k_alpha_{0.99};
  double coriolis_factor_{1.0};
  const double delta_tau_max_{1.0};

  std::array<double, kJoints> max_velocity_{};
  std::array<double, kJoints> max_acceleration_{};
  std::array<double, kJoints> max_jerk_{};

  bool sync_after_activation_{true};
  bool motion_generator_initialized_{false};
  bool move_to_start_position_finished_{false};
  ros::Time sync_start_time_;
  ros::Time last_target_receive_time_;

  std::array<double, kJoints> sync_q_start_{};
  std::array<double, kJoints> sync_delta_q_{};
  std::array<double, kJoints> sync_dq_max_{};
  std::array<double, kJoints> sync_ddq_max_start_{};
  std::array<double, kJoints> sync_ddq_max_goal_{};
  std::array<double, kJoints> sync_dq_max_sync_{};
  std::array<double, kJoints> sync_t_1_sync_{};
  std::array<double, kJoints> sync_t_2_sync_{};
  std::array<double, kJoints> sync_t_f_sync_{};
  std::array<double, kJoints> sync_q_1_{};

  ros::Subscriber sub_target_;
  ros::Publisher state_pub_;
  std::string current_state_{"INACTIVE"};
};

}  // namespace fluxvla_franka_controllers
