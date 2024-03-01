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

namespace rm_gimbal_controllers
{
BulletSolver::BulletSolver(ros::NodeHandle& controller_nh)
{
  // 整个config_都是在yaml参数中拿数据
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
  // 把数据放在缓存区
  config_rt_buffer_.initRT(config_);

  // visualization_msgs::Marker marker_desire_ 调用库内结构体，我们起名为marker_desire_
  marker_desire_.header.frame_id = "odom";                   // 坐标系
  marker_desire_.ns = "model";                               // 命名空间
  marker_desire_.action = visualization_msgs::Marker::ADD;   // 操作，是添加还是修改还是删除
  marker_desire_.type = visualization_msgs::Marker::POINTS;  // 类型
  // 靶子大小为0.02*0.02
  marker_desire_.scale.x = 0.02;
  marker_desire_.scale.y = 0.02;
  // marker_desire_靶子颜色为红色
  marker_desire_.color.r = 1.0;
  marker_desire_.color.g = 0.0;
  marker_desire_.color.b = 0.0;
  marker_desire_.color.a = 1.0;

  // visualization_msgs::Marker marker_real_，再次调用库内结构体，清明为marker_desire_
  marker_real_ = marker_desire_;
  marker_real_.color.r = 0.0;
  // marker_desire_ 靶子颜色为绿色
  marker_real_.color.g = 1.0;

  // 动态调参
  d_srv_ = new dynamic_reconfigure::Server<rm_gimbal_controllers::BulletSolverConfig>(controller_nh);
  dynamic_reconfigure::Server<rm_gimbal_controllers::BulletSolverConfig>::CallbackType cb =
      [this](auto&& PH1, auto&& PH2) { reconfigCB(PH1, PH2); };
  d_srv_->setCallback(cb);

  // std::shared_ptr<realtime_tools::RealtimePublisher<visualization_msgs::Marker>> path_desire_pub_
  // path_desire_pub_消息类型为visualization_msgs::Marker
  // path_desire_pub发布在“model_desire”下
  path_desire_pub_.reset(
      new realtime_tools::RealtimePublisher<visualization_msgs::Marker>(controller_nh, "model_desire", 10));
  // path_real_pub_发布在“model_real”下
  path_real_pub_.reset(
      new realtime_tools::RealtimePublisher<visualization_msgs::Marker>(controller_nh, "model_real", 10));
}

// 整个BulletSolver::getResistanceCoefficient在对应不同的子弹弹速在给resistance_coff的值
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

// 标志位solve_success拿的是BulletSolver::solve函数的return值
// solve(pos相对于我方机器人的目标中心位置, vel相对于我方机器人的目标速度, bullet_speed子弹速度, 装甲板的朝向yaw值相对我方机器人,
//        v_yaw装甲板旋转的角速度,r1半径1, r2半径2, dz相邻装甲板的高度差, armors_num装甲板数量);
bool BulletSolver::solve(geometry_msgs::Point pos, geometry_msgs::Vector3 vel, double bullet_speed, double yaw,
                         double v_yaw, double r1, double r2, double dz, int armors_num)
{
  // config_拿了一堆参数
  config_ = *config_rt_buffer_.readFromRT();
  bullet_speed_ = bullet_speed;
  // 对应弹速拿resistance_coff_值，目前车上均为0
  resistance_coff_ = getResistanceCoefficient(bullet_speed_) != 0 ? getResistanceCoefficient(bullet_speed_) : 0.001;

  // 默认temp_z是相对于我方机器人目标位置的z位置
  double temp_z = pos.z;
  // target_rho等于根号下x平方+y平方，即不考虑z轴，把平面压缩为xoy平面
  // target_rho是xoy平面我方和敌方车中心的距离
  double target_rho = std::sqrt(std::pow(pos.x, 2) + std::pow(pos.y, 2));
  // 输出yaw为y/x的反正切
  // output_yaw_为现在我方头yaw需要偏向敌方车的角度
  output_yaw_ = std::atan2(pos.y, pos.x);
  // output_pitch_为现在我方头pitch需要偏向敌方车的角度
  output_pitch_ = std::atan2(temp_z, std::sqrt(std::pow(pos.x, 2) + std::pow(pos.y, 2)));
  // 根据我看不懂的公式得出粗略的飞行时间rough_fly_time
  double rough_fly_time =
      (-std::log(1 - target_rho * resistance_coff_ / (bullet_speed_ * std::cos(output_pitch_)))) / resistance_coff_;
  selected_armor_ = 0;  // 选择装甲板先默认为0
  // 用r1初始化r
  double r = r1;
  // z取相对于我方机器人的目标位置的z
  double z = pos.z;
  // max_track_target_vel_是我们设置的参数max_track_target_vel
  // 如果装甲板旋转速度大于max_track_target_vel_，track_target给0
  // 如果装甲板旋转速度小于max_track_target_vel_，track_target给1
  track_target_ = std::abs(v_yaw) < max_track_target_vel_;
  // 装甲板旋转速度大于max_track_target_vel_的情况下，switch_armor_angle等于pai/12
  // 装甲板旋转速度小于max_track_target_vel_的情况下，switch_armor_angle等于什么什么
  double switch_armor_angle = track_target_ ?
                                  acos(r / target_rho) - M_PI / 12 +
                                      (-acos(r / target_rho) + M_PI / 6) * std::abs(v_yaw) / max_track_target_vel_ :
                                  M_PI / 12;
  // switch_armor_angle是计算车中心与装甲板相对于我方机器人的角度
  // 情况一，装甲板预期位置大于我们所能达到位置，
  // 情况二，装甲板预期位置小于我们所能达到位置，
  if ((((yaw + v_yaw * rough_fly_time) > output_yaw_ + switch_armor_angle) && v_yaw > 0.) ||
      (((yaw + v_yaw * rough_fly_time) < output_yaw_ - switch_armor_angle) && v_yaw < 0.))
  {
    // 装甲板角速度正向给-1，反向给1
    // 这里本质是需要换下一块或者上一块板子
    selected_armor_ = v_yaw > 0. ? -1 : 1;
    // 装甲板数目是4，r取r2，前面已经算完了r1，后面开始算r2
    // 如果不需要板子，r2根本不需要出现
    r = armors_num == 4 ? r2 : r1;
    // 如果装甲板数目是4，z等于中心位置加上偏差dz，即为下一块或者上一块装甲板的实际z位置
    // 如果装甲板数目不是4，那么z的位置就是目标位置z
    z = armors_num == 4 ? pos.z + dz : pos.z;
  }
  int count{};
  double error = 999;
  // 本质为了算目标位置的xy
  if (track_target_)  // 装甲板速度小于5进if
  {
    // target_pos_第一次出现
    target_pos_.x = pos.x - r * cos(yaw + selected_armor_ * 2 * M_PI / armors_num);
    target_pos_.y = pos.y - r * sin(yaw + selected_armor_ * 2 * M_PI / armors_num);
  }
  else  // 装甲板速度大于5进else
  {
    target_pos_.x = pos.x - r * cos(atan2(pos.y, pos.x));
    target_pos_.y = pos.y - r * sin(atan2(pos.y, pos.x));
  }
  // 情况具体为如果为4块装甲板的兵种，并且这块装甲板它打不到了，只能重新计算z
  // target_pos_.z在这里完成了真正的目标位置赋值
  target_pos_.z = z;
  // 第一次直接进循环，error默认
  while (error >= 0.001)
  {
    // 又重新计算相对yaw值，相对pitch值，我敌距离，粗略飞行时间
    output_yaw_ = std::atan2(target_pos_.y, target_pos_.x);
    output_pitch_ = std::atan2(temp_z, std::sqrt(std::pow(target_pos_.x, 2) + std::pow(target_pos_.y, 2)));
    target_rho = std::sqrt(std::pow(target_pos_.x, 2) + std::pow(target_pos_.y, 2));
    fly_time_ =
        (-std::log(1 - target_rho * resistance_coff_ / (bullet_speed_ * std::cos(output_pitch_)))) / resistance_coff_;
    //
    double real_z = (bullet_speed_ * std::sin(output_pitch_) + (config_.g / resistance_coff_)) *
                        (1 - std::exp(-resistance_coff_ * fly_time_)) / resistance_coff_ -
                    config_.g * fly_time_ / resistance_coff_;

    if (track_target_)  // 装甲板速度小于5进
    {
      // 重新计算目标位置的x和y
      target_pos_.x =
          pos.x + vel.x * fly_time_ - r * cos(yaw + v_yaw * fly_time_ + selected_armor_ * 2 * M_PI / armors_num);
      target_pos_.y =
          pos.y + vel.y * fly_time_ - r * sin(yaw + v_yaw * fly_time_ + selected_armor_ * 2 * M_PI / armors_num);
    }
    else  // 装甲板速度大于5进
    {
      // 写个数组，保存飞行后目标位置的x和y
      double target_pos_after_fly_time[2];
      // 计算出来飞行后目标位置的x和y
      target_pos_after_fly_time[0] = pos.x + vel.x * fly_time_;
      target_pos_after_fly_time[1] = pos.y + vel.y * fly_time_;
      // 利用上面，重新计算目标位置的x和y
      target_pos_.x =
          target_pos_after_fly_time[0] - r * cos(atan2(target_pos_after_fly_time[1], target_pos_after_fly_time[0]));
      target_pos_.y =
          target_pos_after_fly_time[1] - r * sin(atan2(target_pos_after_fly_time[1], target_pos_after_fly_time[0]));
    }
    // 重新计算z的位置
    target_pos_.z = z + vel.z * fly_time_;

    // 又重新定义一个target_yaw算目标位置所在的yaw的角
    double target_yaw = std::atan2(target_pos_.y, target_pos_.x);
    // 计算偏差角
    double error_theta = target_yaw - output_yaw_;
    // 计算偏差z
    double error_z = target_pos_.z - real_z;
    // 把偏差z进行累加到temp_z
    temp_z += error_z;
    // 计算出真正的error
    error = std::sqrt(std::pow(error_theta * target_rho, 2) + std::pow(error_z, 2));
    // 走一次count加1
    count++;

    if (count >= 20 || std::isnan(error))  // 如果迭代20次以上或者error不是数组，程序错误
      return false;
  }
  return true;
}

void BulletSolver::getSelectedArmorPosAndVel(geometry_msgs::Point& armor_pos, geometry_msgs::Vector3& armor_vel,
                                             geometry_msgs::Point pos, geometry_msgs::Vector3 vel, double yaw,
                                             double v_yaw, double r1, double r2, double dz, int armors_num)
{
  double r = r1, z = pos.z;
  if (armors_num == 4 && selected_armor_ != 0)
  {
    r = r2;
    z = pos.z + dz;
  }
  if (track_target_)
  {
    armor_pos.x = pos.x - r * cos(yaw + selected_armor_ * 2 * M_PI / armors_num);
    armor_pos.y = pos.y - r * sin(yaw + selected_armor_ * 2 * M_PI / armors_num);
    armor_pos.z = z;
    armor_vel.x = vel.x + v_yaw * r * sin(yaw + selected_armor_ * 2 * M_PI / armors_num);
    armor_vel.y = vel.y - v_yaw * r * cos(yaw + selected_armor_ * 2 * M_PI / armors_num);
    armor_vel.z = vel.z;
  }
  else
  {
    armor_pos = pos;
    armor_pos.z = z;
    armor_vel = vel;
  }
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

double BulletSolver::getGimbalError(geometry_msgs::Point pos, geometry_msgs::Vector3 vel, double yaw, double v_yaw,
                                    double r1, double r2, double dz, int armors_num, double yaw_real, double pitch_real,
                                    double bullet_speed)
{
  double delay = track_target_ ? 0 : config_.delay;
  double r = r1;
  double z = pos.z;
  if (selected_armor_ != 0)
  {
    r = armors_num == 4 ? r2 : r1;
    z = armors_num == 4 ? pos.z + dz : pos.z;
  }
  double error;
  if (track_target_)
  {
    double bullet_rho =
        bullet_speed * std::cos(pitch_real) * (1 - std::exp(-resistance_coff_ * fly_time_)) / resistance_coff_;
    double bullet_x = bullet_rho * std::cos(yaw_real);
    double bullet_y = bullet_rho * std::sin(yaw_real);
    double bullet_z = (bullet_speed * std::sin(pitch_real) + (config_.g / resistance_coff_)) *
                          (1 - std::exp(-resistance_coff_ * fly_time_)) / resistance_coff_ -
                      config_.g * fly_time_ / resistance_coff_;
    error = std::sqrt(std::pow(target_pos_.x - bullet_x, 2) + std::pow(target_pos_.y - bullet_y, 2) +
                      std::pow(target_pos_.z - bullet_z, 2));
  }
  else
  {
    geometry_msgs::Point target_pos_after_fly_time_and_delay{};
    target_pos_after_fly_time_and_delay.x =
        pos.x + vel.x * (fly_time_ + delay) -
        r * cos(yaw + v_yaw * (fly_time_ + delay) + selected_armor_ * 2 * M_PI / armors_num);
    target_pos_after_fly_time_and_delay.y =
        pos.y + vel.y * (fly_time_ + delay) -
        r * sin(yaw + v_yaw * (fly_time_ + delay) + selected_armor_ * 2 * M_PI / armors_num);
    target_pos_after_fly_time_and_delay.z = z + vel.z * (fly_time_ + delay);
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
