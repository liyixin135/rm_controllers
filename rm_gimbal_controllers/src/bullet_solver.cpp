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
  // max_track_target_vel_它是最大跟随敌方目标的速度
  max_track_target_vel_ = getParam(controller_nh, "max_track_target_vel", 5.0);
  // 把数据放在缓存区
  config_rt_buffer_.initRT(config_);

  // visualization_msgs::Marker marker_desire_ 调用库内结构体，我们起名为marker_desire_，用来显示敌方目标
  marker_desire_.header.frame_id = "odom";  // 显示均是在odom下
  marker_desire_.ns = "model";  // 设置标记的命名空间为 "model"。命名空间可以帮助将标记进行分组，使其更易于管理。
  marker_desire_.action = visualization_msgs::Marker::ADD;  // 设置标记的动作为添加。这表示标记将被添加到可视化中。
  marker_desire_.type =
      visualization_msgs::Marker::POINTS;  // 设置标记的类型为点。将表示为一系列点，通常用于表示离散的位置或目标点。
  marker_desire_.scale.x = 0.02;  // 每个标记点的 x 轴方向上的尺寸为 0.02。
  marker_desire_.scale.y = 0.02;  // 每个标记点的 y 轴方向上的尺寸为 0.02。

  marker_desire_.color.r = 1.0;  // 完全红色
  marker_desire_.color.g = 0.0;  // 没有绿色
  marker_desire_.color.b = 0.0;  // 没有蓝色
  marker_desire_.color.a = 1.0;  // 标记物不透明

  // visualization_msgs::Marker marker_real_，再次调用库内结构体，起名为marker_desire_
  marker_real_ = marker_desire_;
  marker_real_.color.r = 0.0;
  // marker_desire_ 敌方中心点颜色为绿色，也在odom坐标系下，命名空间也是model，也是点，大小也一样
  marker_real_.color.g = 1.0;

  // 动态调参
  d_srv_ = new dynamic_reconfigure::Server<rm_gimbal_controllers::BulletSolverConfig>(controller_nh);
  dynamic_reconfigure::Server<rm_gimbal_controllers::BulletSolverConfig>::CallbackType cb =
      [this](auto&& PH1, auto&& PH2) { reconfigCB(PH1, PH2); };
  d_srv_->setCallback(cb);

  // path_desire_pub_是一个智能指针，类型是realtime_tools::RealtimePublisher<visualization_msgs::Marker>
  // reset方法用于替换智能指针中当前管理的对象。这里，它创建了一个新的RealtimePublisher<visualization_msgs::Marker>实例，
  // 将其地址赋给path_desire_pub_智能指针。这个新的RealtimePublisher对象用于发布visualization_msgs::Marker类型的消息。
  // controller_nh用于ros节点的通信，如确定所在的命名空间
  // path_desire_pub发布在“model_desire”话题下
  // 10是队列大小
  path_desire_pub_.reset(
      new realtime_tools::RealtimePublisher<visualization_msgs::Marker>(controller_nh, "model_desire", 10));
  // path_real_pub_发布在“model_real”话题下，其他都跟上面一下
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
// solve(pos相对于我方机器人的目标中心位置, vel相对于我方机器人的目标速度, bullet_speed子弹速度cmd_gimbal_.bullet_speed,
// 装甲板的朝向yaw值相对我方机器人,
//        v_yaw装甲板旋转的角速度,r1半径1, r2半径2, dz相邻装甲板的高度差, armors_num装甲板数量);
bool BulletSolver::solve(geometry_msgs::Point pos, geometry_msgs::Vector3 vel, double bullet_speed, double yaw,
                         double v_yaw, double r1, double r2, double dz, int armors_num)
{
  // config_拿了一堆参数
  config_ = *config_rt_buffer_.readFromRT();
  // 子弹飞行速度
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
  // 设置为能打就算角度，设置为不能打直接给个角度
  double switch_armor_angle = track_target_ ?
                                  acos(r / target_rho) - M_PI / 12 +
                                      (-acos(r / target_rho) + M_PI / 6) * std::abs(v_yaw) / max_track_target_vel_ :
                                  M_PI / 12;
  // 当v_yaw > 0（顺时针旋转）且预期朝向超过了容忍区间的上限时，可能需要切换到一个更适合当前旋转方向的装甲板。
  // 当v_yaw < 0（逆时针旋转）且预期朝向低于了容忍区间的下限时，同样可能需要切换到另一个更适合的装甲板
  // 本质就是追不上了
  if ((((yaw + v_yaw * rough_fly_time) > output_yaw_ + switch_armor_angle) && v_yaw > 0.) ||
      (((yaw + v_yaw * rough_fly_time) < output_yaw_ - switch_armor_angle) && v_yaw < 0.))
  {
    // 装甲板角速度正向给-1，反向给1，就是反着敌方yaw旋转方向走，去找它转到中心的下一块
    selected_armor_ = v_yaw > 0. ? -1 : 1;
    // 装甲板数目是4，r取r2，前面已经算完了r1，后面开始算r2
    // 如果不需要板子，r2根本不需要出现
    r = armors_num == 4 ? r2 : r1;
    // 如果装甲板数目是4，z等于中心位置加上偏差dz，即为下一块或者上一块装甲板的实际z位置
    // 如果装甲板数目不是4，那么z的位置就是目标位置z，例如平衡步兵
    z = armors_num == 4 ? pos.z + dz : pos.z;
  }
  int count{};
  double error = 999;
  // 本质为了算目标位置的xy
  if (track_target_)  // 低速模式
  {
    // target_pos_第一次出现
    // target_pos_算得是装甲板的目标位置，算上了它正在转，selected_armor_是在决定打不打下一块
    target_pos_.x = pos.x - r * cos(yaw + selected_armor_ * 2 * M_PI / armors_num);
    target_pos_.y = pos.y - r * sin(yaw + selected_armor_ * 2 * M_PI / armors_num);
  }
  else  // 打中心模式，算车正对中点
  {
    target_pos_.x = pos.x - r * cos(atan2(pos.y, pos.x));
    target_pos_.y = pos.y - r * sin(atan2(pos.y, pos.x));
  }
  // 完成target_pos_的所有赋值，如果前面觉得打不到了，就有加dz操作，如果不打下一块，直接等于pos.z
  target_pos_.z = z;
  // 第一次直接进循环，error默认
  while (error >= 0.001)  // 如果迭代了20次，error还是大于0.001,即表示解算失败
  {
    // 前面算上敌方装甲板的运动，得到了新的target_pos_数据，接而又重新计算相对yaw值，相对pitch值，我敌距离，粗略飞行时间
    output_yaw_ = std::atan2(target_pos_.y, target_pos_.x);
    output_pitch_ = std::atan2(temp_z, std::sqrt(std::pow(target_pos_.x, 2) + std::pow(target_pos_.y, 2)));
    target_rho = std::sqrt(std::pow(target_pos_.x, 2) + std::pow(target_pos_.y, 2));
    fly_time_ =
        (-std::log(1 - target_rho * resistance_coff_ / (bullet_speed_ * std::cos(output_pitch_)))) / resistance_coff_;
    // 计算子弹在飞行一定时间后的实际垂直位置
    double real_z = (bullet_speed_ * std::sin(output_pitch_) + (config_.g / resistance_coff_)) *
                        (1 - std::exp(-resistance_coff_ * fly_time_)) / resistance_coff_ -
                    config_.g * fly_time_ / resistance_coff_;

    if (track_target_)  // 低速模式
    {
      // 重新计算目标位置的x和y，不仅算上了它在转，还算上了敌方车中心在运动，selected_armor_是为了打下一块而设置
      target_pos_.x =
          pos.x + vel.x * fly_time_ - r * cos(yaw + v_yaw * fly_time_ + selected_armor_ * 2 * M_PI / armors_num);
      target_pos_.y =
          pos.y + vel.y * fly_time_ - r * sin(yaw + v_yaw * fly_time_ + selected_armor_ * 2 * M_PI / armors_num);
    }
    else  // 打中心模式
    {
      // 写个数组，保存飞行后目标位置的x和y
      double target_pos_after_fly_time[2];
      // 计算出来飞行后目标位置的x和y（算上了敌方车中心的运动，不用算转了，角度被固定死了
      target_pos_after_fly_time[0] = pos.x + vel.x * fly_time_;
      target_pos_after_fly_time[1] = pos.y + vel.y * fly_time_;
      // 利用上面，重新计算目标位置的x和y
      target_pos_.x =
          target_pos_after_fly_time[0] - r * cos(atan2(target_pos_after_fly_time[1], target_pos_after_fly_time[0]));
      target_pos_.y =
          target_pos_after_fly_time[1] - r * sin(atan2(target_pos_after_fly_time[1], target_pos_after_fly_time[0]));
    }
    // 重新计算z的位置，在z方向上面不转
    target_pos_.z = z + vel.z * fly_time_;

    // 又重新定义一个target_yaw算目标位置所在的yaw的角
    double target_yaw = std::atan2(target_pos_.y, target_pos_.x);
    // 计算误差偏差角
    double error_theta = target_yaw - output_yaw_;
    // 计算误差偏差z
    double error_z = target_pos_.z - real_z;
    // 把偏差z进行累加到temp_z
    temp_z += error_z;
    // 计算出真正的error，即在xyz空间体系中的距离
    error = std::sqrt(std::pow(error_theta * target_rho, 2) + std::pow(error_z, 2));
    // 走一次count加1
    count++;

    if (count >= 20 || std::isnan(error))  // 如果迭代20次以上或者error不是数组，程序错误
      return false;
  }
  return true;
}

// armor_pos是在odom坐标下敌方车中心的位置
// armor_vel是在odom坐标下敌方车中心的速度
// pos是在相机坐标系下敌方车中心的位置
// vel是在相机坐标系下敌方车中心的速度
// yaw是在相机坐标系装甲板的朝向yaw
// v_yaw是在相机坐标系下装甲板旋转的角速度
// armors_num指看见的兵种是几块装甲板的
// 函数就是得到选中装甲板的位置和速度
void BulletSolver::getSelectedArmorPosAndVel(geometry_msgs::Point& armor_pos, geometry_msgs::Vector3& armor_vel,
                                             geometry_msgs::Point pos, geometry_msgs::Vector3 vel, double yaw,
                                             double v_yaw, double r1, double r2, double dz, int armors_num)
{
  double r = r1, z = pos.z;
  // 如果对方是4块装甲板并且敌方装甲板是在运动的
  if (armors_num == 4 && selected_armor_ != 0)
  {
    r = r2;
    z = pos.z + dz;
  }
  if (track_target_)  // 小于max_track_target_vel_，即不是打中间模式
  {
    // pos.x
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
  // 清除了point上面的所有点，以便添加最新的可视化信息，不会因为旧数据累计而变得混乱
  marker_desire_.points.clear();
  marker_real_.points.clear();
  double roll{}, pitch{}, yaw{};
  // 通过quatToRPY函数将odom2pitch转换为roll,pitch,yaw
  quatToRPY(odom2pitch.transform.rotation, roll, pitch, yaw);
  geometry_msgs::Point point_desire{}, point_real{};
  // 是在xoy平面我方机器人和敌方机器人的距离
  double target_rho = std::sqrt(std::pow(target_pos_.x, 2) + std::pow(target_pos_.y, 2));
  //
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

// pos相对于pitch敌方车中心的位置,vel相对于底盘敌方车中心速度,yaw装甲板的朝向yaw值相对我方机器人,v_yaw装甲板旋转的角速度
// r1,r2,dz,armors_num,yaw_real现在车yaw的位置，pitch_real现在车pitch的位置
// bullet_speed子弹速度
double BulletSolver::getGimbalError(geometry_msgs::Point pos, geometry_msgs::Vector3 vel, double yaw, double v_yaw,
                                    double r1, double r2, double dz, int armors_num, double yaw_real, double pitch_real,
                                    double bullet_speed)
{
  double delay = track_target_ ? 0 : config_.delay;  // 如果用中心模式，则用延迟模式
  // 先设置一波默认参数，r1以及目标中心位置的z
  double r = r1;
  double z = pos.z;
  if (selected_armor_ != 0)  // 如果是打下一块，r切r2，z加上dz
  {
    r = armors_num == 4 ? r2 : r1;
    z = armors_num == 4 ? pos.z + dz : pos.z;
  }
  double error;       // 定义error
  if (track_target_)  // 如果低速模式
  {
    // 忽略延迟对瞄准的影响，直接计算发射距离（在xyz空间
    double bullet_rho =
        bullet_speed * std::cos(pitch_real) * (1 - std::exp(-resistance_coff_ * fly_time_)) / resistance_coff_;
    // 分解到xyz三个方向
    double bullet_x = bullet_rho * std::cos(yaw_real);
    double bullet_y = bullet_rho * std::sin(yaw_real);
    double bullet_z = (bullet_speed * std::sin(pitch_real) + (config_.g / resistance_coff_)) *
                          (1 - std::exp(-resistance_coff_ * fly_time_)) / resistance_coff_ -
                      config_.g * fly_time_ / resistance_coff_;
    // 根据目标位置计算真正的gimbal_error
    error = std::sqrt(std::pow(target_pos_.x - bullet_x, 2) + std::pow(target_pos_.y - bullet_y, 2) +
                      std::pow(target_pos_.z - bullet_z, 2));
  }
  else  // 如果中心模式
  {
    // 算上延迟算真正的gimbal_error
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
  return error;  // 直接返回error
}

// 有关动态调参的
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
