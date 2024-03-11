/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

//
// Created by qiayuan on 8/14/20.
//

#include "rm_gimbal_controllers/bullet_solver.h"
#include <cmath>
#include <tf/transform_datatypes.h>
#include <rm_common/ori_tool.h>
#include <angles/angles.h>

namespace rm_gimbal_controllers
{
BulletSolver::BulletSolver(ros::NodeHandle& controller_nh)
{
  config_ = { .resistance_coff_qd_10 = getParam(controller_nh, "resistance_coff_qd_10", 0.),
              .resistance_coff_qd_15 = getParam(controller_nh, "resistance_coff_qd_15", 0.),
              .resistance_coff_qd_16 = getParam(controller_nh, "resistance_coff_qd_16", 0.),
              .resistance_coff_qd_18 = getParam(controller_nh, "resistance_coff_qd_18", 0.),
              .resistance_coff_qd_30 = getParam(controller_nh, "resistance_coff_qd_30", 0.),
              .g = getParam(controller_nh, "g", 0.),
              .delay = getParam(controller_nh, "delay", 0.),
              .dt = getParam(controller_nh, "dt", 0.),
              .timeout = getParam(controller_nh, "timeout", 0.) };
  max_track_target_vel_ = getParam(controller_nh, "max_track_target_vel", 5.0);
  config_rt_buffer_.initRT(config_);

  marker_desire_.header.frame_id = "odom";
  marker_desire_.ns = "model";
  marker_desire_.action = visualization_msgs::Marker::ADD;
  marker_desire_.type = visualization_msgs::Marker::POINTS;
  marker_desire_.scale.x = 0.02;
  marker_desire_.scale.y = 0.02;
  marker_desire_.color.r = 1.0;
  marker_desire_.color.g = 0.0;
  marker_desire_.color.b = 0.0;
  marker_desire_.color.a = 1.0;

  marker_real_ = marker_desire_;
  marker_real_.color.r = 0.0;
  marker_real_.color.g = 1.0;

  d_srv_ = new dynamic_reconfigure::Server<rm_gimbal_controllers::BulletSolverConfig>(controller_nh);
  dynamic_reconfigure::Server<rm_gimbal_controllers::BulletSolverConfig>::CallbackType cb =
      [this](auto&& PH1, auto&& PH2) { reconfigCB(PH1, PH2); };
  d_srv_->setCallback(cb);

  path_desire_pub_.reset(
      new realtime_tools::RealtimePublisher<visualization_msgs::Marker>(controller_nh, "model_desire", 10));
  path_real_pub_.reset(
      new realtime_tools::RealtimePublisher<visualization_msgs::Marker>(controller_nh, "model_real", 10));
}

double BulletSolver::getResistanceCoefficient(double bullet_speed) const
{
  // bullet_speed have 5 value:10,15,16,18,30
  double resistance_coff;
  if (bullet_speed < 12.5)
    resistance_coff = config_.resistance_coff_qd_10;
  else if (bullet_speed < 15.5)
    resistance_coff = config_.resistance_coff_qd_15;
  else if (bullet_speed < 17)
    resistance_coff = config_.resistance_coff_qd_16;
  else if (bullet_speed < 24)
    resistance_coff = config_.resistance_coff_qd_18;
  else
    resistance_coff = config_.resistance_coff_qd_30;
  return resistance_coff;
}

bool BulletSolver::solve(geometry_msgs::Point pos, geometry_msgs::Vector3 vel, double bullet_speed, double yaw,
                         double v_yaw, double r1, double r2, double dz, int armors_num)
{
  config_ = *config_rt_buffer_.readFromRT();
  bullet_speed_ = bullet_speed;
  resistance_coff_ = getResistanceCoefficient(bullet_speed_) != 0 ? getResistanceCoefficient(bullet_speed_) : 0.001;
  target_state_ = TargetState{ .current_target_center_pos = pos,
                               .current_target_center_vel = vel,
                               .yaw = yaw,
                               .v_yaw = v_yaw,
                               .armors_num = armors_num };

  double temp_z = pos.z;
  double target_rho = std::sqrt(std::pow(pos.x, 2) + std::pow(pos.y, 2));
  output_yaw_ = std::atan2(pos.y, pos.x);
  output_pitch_ = std::atan2(temp_z, std::sqrt(std::pow(pos.x, 2) + std::pow(pos.y, 2)));
  double rough_fly_time =
      (-std::log(1 - target_rho * resistance_coff_ / (bullet_speed_ * std::cos(output_pitch_)))) / resistance_coff_;
  double r = r1;
  double z = pos.z;
  track_target_ = std::abs(v_yaw) < max_track_target_vel_;
  double aim_range_front;
  double switch_armor_angle = track_target_ ?
                                  acos(r / target_rho) - M_PI / 12 +
                                      (-acos(r / target_rho) + M_PI / 6) * std::abs(v_yaw) / max_track_target_vel_ :
                                  M_PI / 12;
  aim_range_front = output_yaw_ + (v_yaw > 0 ? switch_armor_angle - 2 * M_PI / armors_num : -switch_armor_angle);
  double shortest_angular_distance = angles::shortest_angular_distance(yaw + v_yaw * rough_fly_time, aim_range_front);
  if (shortest_angular_distance < 0)
    shortest_angular_distance += 2 * M_PI;
  selected_armor_ =
      static_cast<SelectedArmor>(fmod((shortest_angular_distance / (2 * M_PI / armors_num) + 1), armors_num));
  if (armors_num == 4 && selected_armor_ != SelectedArmor::FRONT && selected_armor_ != SelectedArmor::BACK)
  {
    r = r2;
    z = pos.z + dz;
  }
  target_state_.r = r;
  target_state_.current_target_center_pos.z = z;
  int count{};
  double error = 999;
  if (track_target_)
  {
    target_pos_.x = pos.x - r * cos(yaw + static_cast<int>(selected_armor_) * 2 * M_PI / armors_num);
    target_pos_.y = pos.y - r * sin(yaw + static_cast<int>(selected_armor_) * 2 * M_PI / armors_num);
  }
  else
  {
    target_pos_.x = pos.x - r * cos(atan2(pos.y, pos.x));
    target_pos_.y = pos.y - r * sin(atan2(pos.y, pos.x));
  }
  target_pos_.z = z;
  while (error >= 0.001)
  {
    output_yaw_ = std::atan2(target_pos_.y, target_pos_.x);
    target_rho = std::sqrt(std::pow(target_pos_.x, 2) + std::pow(target_pos_.y, 2));
    output_pitch_ = std::atan2(temp_z, target_rho);
    fly_time_ =
        (-std::log(1 - target_rho * resistance_coff_ / (bullet_speed_ * std::cos(output_pitch_)))) / resistance_coff_;
    double real_z = (bullet_speed_ * std::sin(output_pitch_) + (config_.g / resistance_coff_)) *
                        (1 - std::exp(-resistance_coff_ * fly_time_)) / resistance_coff_ -
                    config_.g * fly_time_ / resistance_coff_;

    if (track_target_)
    {
      target_pos_.x = pos.x + vel.x * fly_time_ -
                      r * cos(yaw + v_yaw * fly_time_ + static_cast<int>(selected_armor_) * 2 * M_PI / armors_num);
      target_pos_.y = pos.y + vel.y * fly_time_ -
                      r * sin(yaw + v_yaw * fly_time_ + static_cast<int>(selected_armor_) * 2 * M_PI / armors_num);
      target_vel_.x =
          vel.x + v_yaw * r * sin(yaw + v_yaw * fly_time_ + static_cast<int>(selected_armor_) * 2 * M_PI / armors_num);
      target_vel_.y =
          vel.y - v_yaw * r * cos(yaw + v_yaw * fly_time_ + static_cast<int>(selected_armor_) * 2 * M_PI / armors_num);
      target_accel_.x =
          pow(v_yaw, 2) * r * cos(yaw + v_yaw * fly_time_ + static_cast<int>(selected_armor_) * 2 * M_PI / armors_num);
      target_accel_.y =
          pow(v_yaw, 2) * r * sin(yaw + v_yaw * fly_time_ + static_cast<int>(selected_armor_) * 2 * M_PI / armors_num);
    }
    else
    {
      double target_pos_after_fly_time[2];
      target_pos_after_fly_time[0] = pos.x + vel.x * fly_time_;
      target_pos_after_fly_time[1] = pos.y + vel.y * fly_time_;
      target_pos_.x =
          target_pos_after_fly_time[0] - r * cos(atan2(target_pos_after_fly_time[1], target_pos_after_fly_time[0]));
      target_pos_.y =
          target_pos_after_fly_time[1] - r * sin(atan2(target_pos_after_fly_time[1], target_pos_after_fly_time[0]));
    }
    target_pos_.z = z + vel.z * fly_time_;

    double target_yaw = std::atan2(target_pos_.y, target_pos_.x);
    double error_theta = target_yaw - output_yaw_;
    double error_z = target_pos_.z - real_z;
    temp_z += error_z;
    error = std::sqrt(std::pow(error_theta * target_rho, 2) + std::pow(error_z, 2));
    count++;

    if (count >= 20 || std::isnan(error))
      return false;
  }
  compute_target_position_.position.x = target_pos_.x;
  compute_target_position_.position.y = target_pos_.y;
  compute_target_position_.position.z = target_pos_.z;
  compute_target_position_pub.publish(compute_target_position_);
  return true;
}

void BulletSolver::getYawVelAndAccelDes(double& vel_des, double& accel_des)
{
  double yaw_vel_des =
      (target_pos_.x * target_vel_.y - target_pos_.y * target_vel_.x) / (pow(target_pos_.x, 2) + pow(target_pos_.y, 2));
  double yaw_accel_des = (pow(target_pos_.x, 3) * target_accel_.y - pow(target_pos_.y, 3) * target_accel_.x +
                          2 * target_pos_.x * target_pos_.y * pow(target_vel_.x, 2) -
                          2 * target_pos_.x * target_pos_.y * pow(target_vel_.y, 2) -
                          pow(target_pos_.x, 2) * target_pos_.y * target_accel_.x +
                          target_pos_.x * pow(target_pos_.y, 2) * target_accel_.y -
                          2 * pow(target_pos_.x, 2) * target_vel_.x * target_vel_.y +
                          2 * pow(target_pos_.y, 2) * target_vel_.x * target_vel_.y) /
                         pow((pow(target_pos_.x, 2) + pow(target_pos_.y, 2)), 2);
  vel_des = yaw_vel_des;
  accel_des = yaw_accel_des;
}

void BulletSolver::getPitchVelAndAccelDes(double& vel_des, double& accel_des)
{
  double dt = 0.01;
  double r = target_state_.r;
  double pos_x = target_state_.current_target_center_pos.x +
                 target_state_.current_target_center_vel.x * (fly_time_ + dt) -
                 r * cos(target_state_.yaw + target_state_.v_yaw * (fly_time_ + dt) +
                         static_cast<int>(selected_armor_) * 2 * M_PI / target_state_.armors_num);
  double pos_y = target_state_.current_target_center_pos.y +
                 target_state_.current_target_center_vel.y * (fly_time_ + dt) -
                 r * sin(target_state_.yaw + target_state_.v_yaw * (fly_time_ + dt) +
                         static_cast<int>(selected_armor_) * 2 * M_PI / target_state_.armors_num);
  double pos_z =
      target_state_.current_target_center_pos.z + (fly_time_ + dt) * target_state_.current_target_center_vel.z;
  double target_rho = std::sqrt(std::pow(pos_x, 2) + std::pow(pos_y, 2));
  double temp_z = target_rho * tan(output_pitch_);
  double output_pitch_next = output_pitch_;
  double error_z = 999;
  while (std::abs(error_z) >= 1e-9)
  {
    output_pitch_next = std::atan2(temp_z, target_rho);
    double fly_time = (-std::log(1 - target_rho * resistance_coff_ / (bullet_speed_ * std::cos(output_pitch_next)))) /
                      resistance_coff_;
    double real_z = (bullet_speed_ * std::sin(output_pitch_next) + (config_.g / resistance_coff_)) *
                        (1 - std::exp(-resistance_coff_ * fly_time)) / resistance_coff_ -
                    config_.g * fly_time / resistance_coff_;
    error_z = pos_z - real_z;
    temp_z += error_z;
  }
  double pitch_vel_des, pitch_accel_des;
  pitch_vel_des = (output_pitch_next - output_pitch_) / dt;
  ros::Time now = ros::Time::now();
  pitch_accel_des = (pitch_vel_des - last_pitch_vel_des_) / (now - last_pitch_vel_des_solve_time_).toSec();
  last_pitch_vel_des_ = pitch_vel_des;
  last_pitch_vel_des_solve_time_ = now;
  vel_des = pitch_vel_des;
  accel_des = pitch_accel_des;
}

void BulletSolver::bulletModelPub(const geometry_msgs::TransformStamped& odom2pitch, const ros::Time& time)
{
  marker_desire_.points.clear();
  marker_real_.points.clear();
  double roll{}, pitch{}, yaw{};
  quatToRPY(odom2pitch.transform.rotation, roll, pitch, yaw);
  geometry_msgs::Point point_desire{}, point_real{};
  double target_rho = std::sqrt(std::pow(target_pos_.x, 2) + std::pow(target_pos_.y, 2));
  int point_num = int(target_rho * 20);
  for (int i = 0; i <= point_num; i++)
  {
    double rt_bullet_rho = target_rho * i / point_num;
    double fly_time = (-std::log(1 - rt_bullet_rho * resistance_coff_ / (bullet_speed_ * std::cos(output_pitch_)))) /
                      resistance_coff_;
    double rt_bullet_z = (bullet_speed_ * std::sin(output_pitch_) + (config_.g / resistance_coff_)) *
                             (1 - std::exp(-resistance_coff_ * fly_time)) / resistance_coff_ -
                         config_.g * fly_time / resistance_coff_;
    point_desire.x = rt_bullet_rho * std::cos(output_yaw_) + odom2pitch.transform.translation.x;
    point_desire.y = rt_bullet_rho * std::sin(output_yaw_) + odom2pitch.transform.translation.y;
    point_desire.z = rt_bullet_z + odom2pitch.transform.translation.z;
    marker_desire_.points.push_back(point_desire);
  }
  for (int i = 0; i <= point_num; i++)
  {
    double rt_bullet_rho = target_rho * i / point_num;
    double fly_time =
        (-std::log(1 - rt_bullet_rho * resistance_coff_ / (bullet_speed_ * std::cos(-pitch)))) / resistance_coff_;
    double rt_bullet_z = (bullet_speed_ * std::sin(-pitch) + (config_.g / resistance_coff_)) *
                             (1 - std::exp(-resistance_coff_ * fly_time)) / resistance_coff_ -
                         config_.g * fly_time / resistance_coff_;
    point_real.x = rt_bullet_rho * std::cos(yaw) + odom2pitch.transform.translation.x;
    point_real.y = rt_bullet_rho * std::sin(yaw) + odom2pitch.transform.translation.y;
    point_real.z = rt_bullet_z + odom2pitch.transform.translation.z;
    marker_real_.points.push_back(point_real);
  }
  marker_desire_.header.stamp = time;
  if (path_desire_pub_->trylock())
  {
    path_desire_pub_->msg_ = marker_desire_;
    path_desire_pub_->unlockAndPublish();
  }
  marker_real_.header.stamp = time;
  if (path_real_pub_->trylock())
  {
    path_real_pub_->msg_ = marker_real_;
    path_real_pub_->unlockAndPublish();
  }
}

double BulletSolver::getGimbalError(double yaw_real, double pitch_real)
{
  double delay = track_target_ ? 0 : config_.delay;
  double error;
  if (track_target_)
  {
    double bullet_rho =
        bullet_speed_ * std::cos(pitch_real) * (1 - std::exp(-resistance_coff_ * fly_time_)) / resistance_coff_;
    double bullet_x = bullet_rho * std::cos(yaw_real);
    double bullet_y = bullet_rho * std::sin(yaw_real);
    double bullet_z = (bullet_speed_ * std::sin(pitch_real) + (config_.g / resistance_coff_)) *
                          (1 - std::exp(-resistance_coff_ * fly_time_)) / resistance_coff_ -
                      config_.g * fly_time_ / resistance_coff_;
    error = std::sqrt(std::pow(target_pos_.x - bullet_x, 2) + std::pow(target_pos_.y - bullet_y, 2) +
                      std::pow(target_pos_.z - bullet_z, 2));
  }
  else
  {
    geometry_msgs::Point target_pos_after_fly_time_and_delay{};
    target_pos_after_fly_time_and_delay.x =
        target_state_.current_target_center_pos.x + target_state_.current_target_center_vel.x * (fly_time_ + delay) -
        target_state_.r * cos(target_state_.yaw + target_state_.v_yaw * (fly_time_ + delay) +
                              static_cast<int>(selected_armor_) * 2 * M_PI / target_state_.armors_num);
    target_pos_after_fly_time_and_delay.y =
        target_state_.current_target_center_pos.y + target_state_.current_target_center_vel.y * (fly_time_ + delay) -
        target_state_.r * sin(target_state_.yaw + target_state_.v_yaw * (fly_time_ + delay) +
                              static_cast<int>(selected_armor_) * 2 * M_PI / target_state_.armors_num);
    target_pos_after_fly_time_and_delay.z =
        target_state_.current_target_center_pos.z + target_state_.current_target_center_vel.z * (fly_time_ + delay);
    error = std::sqrt(std::pow(target_pos_.x - target_pos_after_fly_time_and_delay.x, 2) +
                      std::pow(target_pos_.y - target_pos_after_fly_time_and_delay.y, 2) +
                      std::pow(target_pos_.z - target_pos_after_fly_time_and_delay.z, 2));
  }
  return error;
}

void BulletSolver::reconfigCB(rm_gimbal_controllers::BulletSolverConfig& config, uint32_t /*unused*/)
{
  ROS_INFO("[Bullet Solver] Dynamic params change");
  if (!dynamic_reconfig_initialized_)
  {
    Config init_config = *config_rt_buffer_.readFromNonRT();  // config init use yaml
    config.resistance_coff_qd_10 = init_config.resistance_coff_qd_10;
    config.resistance_coff_qd_15 = init_config.resistance_coff_qd_15;
    config.resistance_coff_qd_16 = init_config.resistance_coff_qd_16;
    config.resistance_coff_qd_18 = init_config.resistance_coff_qd_18;
    config.resistance_coff_qd_30 = init_config.resistance_coff_qd_30;
    config.g = init_config.g;
    config.delay = init_config.delay;
    config.dt = init_config.dt;
    config.timeout = init_config.timeout;
    dynamic_reconfig_initialized_ = true;
  }
  Config config_non_rt{ .resistance_coff_qd_10 = config.resistance_coff_qd_10,
                        .resistance_coff_qd_15 = config.resistance_coff_qd_15,
                        .resistance_coff_qd_16 = config.resistance_coff_qd_16,
                        .resistance_coff_qd_18 = config.resistance_coff_qd_18,
                        .resistance_coff_qd_30 = config.resistance_coff_qd_30,
                        .g = config.g,
                        .delay = config.delay,
                        .dt = config.dt,
                        .timeout = config.timeout };
  config_rt_buffer_.writeFromNonRT(config_non_rt);
}
}  // namespace rm_gimbal_controllers
