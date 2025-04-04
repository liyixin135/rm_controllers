//
// Created by guanlin on 25-4-3.
//

#pragma once

#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>
#include <dynamic_reconfigure/server.h>
#include <rm_common/ros_utilities.h>
#include <rm_msgs/BulletSolverState.h>
#include <rm_gimbal_controllers/BulletSolverConfig.h>
#include "rm_gimbal_controllers/bullet_solver/target_kinematics.h"
#include "rm_gimbal_controllers/bullet_solver/selector.h"

namespace rm_gimbal_controllers
{
struct Config
{
  double resistance_coff_qd_10, resistance_coff_qd_15, resistance_coff_qd_16, resistance_coff_qd_18,
      resistance_coff_qd_30, g, delay, wait_next_armor_delay, wait_diagonal_armor_delay, dt, timeout,
      ban_shoot_duration, gimbal_switch_duration, max_switch_angle, min_switch_angle, min_shoot_beforehand_vel,
      max_chassis_angular_vel, max_track_target_vel, track_rotate_target_delay, track_move_target_delay;
  int min_fit_switch_count;
};
class BulletSolver
{
public:
  explicit BulletSolver(ros::NodeHandle& controller_nh);
  void selectTarget(geometry_msgs::Point pos, geometry_msgs::Vector3 vel, double bullet_speed, double yaw, double v_yaw,
                    double r1, double r2, double dz);
  bool solve();
  double getYaw() const
  {
    return output_yaw_;
  }
  double getPitch() const
  {
    return -output_pitch_;
  }
  double getResistanceCoefficient(double bullet_speed) const;
  double getGimbalError(double yaw_real, double pitch_real);
  void getSelectedArmorPosAndVel(geometry_msgs::Point& armor_pos, geometry_msgs::Vector3& armor_vel);
  void publishState();
  void reconfigCB(rm_gimbal_controllers::BulletSolverConfig& config, uint32_t);
  ~BulletSolver() = default;

private:
  std::unique_ptr<TrackedTargetKinematic> tracked_target_kinematic_;
  std::unique_ptr<UntrackedTargetKinematic> untracked_target_kinematic_;
  std::unique_ptr<TargetSelector> target_selector_;
  std::unique_ptr<realtime_tools::RealtimePublisher<rm_msgs::BulletSolverState>> state_pub_;
  geometry_msgs::Point target_pos_{};
  realtime_tools::RealtimeBuffer<Config> config_rt_buffer_;
  dynamic_reconfigure::Server<rm_gimbal_controllers::BulletSolverConfig>* d_srv_{};
  Config config_{};
  bool dynamic_reconfig_initialized_{};
  bool track_target_{};
  double output_yaw_{}, output_pitch_{};
  double bullet_speed_{}, resistance_coff_{}, fly_time_{};
  int current_armor_{};
};
}  // namespace rm_gimbal_controllers
