//
// Created by guanlin on 25-4-3.
//

#pragma once

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace rm_gimbal_controllers
{
enum Armor
{
  LEFT = 0,
  FRONT,
  RIGHT,
  BACK,
};
enum SwitchArmorState
{
  NO_SWITCH = 0,
  READY_SWITCH,
  START_SWITCH,
};
class TargetSelector
{
public:
  TargetSelector() = default;
  void reset(geometry_msgs::Point pos, geometry_msgs::Vector3 vel, double yaw, double v_yaw, double r1, double r2,
             double bullet_speed, double resistance_coff, double max_track_target_vel, double delay, int armors_num)
  {
    pos_ = pos;
    vel_ = vel;
    yaw_ = yaw;
    v_yaw_ = v_yaw;
    r1_ = r1;
    r2_ = r2;
    armors_num_ = armors_num;
    delay_ = delay;
    bullet_speed_ = bullet_speed;
    resistance_coff_ = resistance_coff;
    max_track_target_vel_ = max_track_target_vel;
  }
  int getTargetArmor()
  {
    double target_rho = std::sqrt(std::pow(pos_.x, 2) + std::pow(pos_.y, 2));
    double output_yaw = std::atan2(pos_.y, pos_.x);
    double output_pitch = std::atan2(pos_.z, std::sqrt(std::pow(pos_.x, 2) + std::pow(pos_.y, 2)));
    double rough_fly_time =
        (-std::log(1 - target_rho * resistance_coff_ / (bullet_speed_ * std::cos(output_pitch)))) / resistance_coff_;
    double max_switch_angle = 50 / 180 * M_PI;
    double min_switch_angle = 0 / 180 * M_PI;
    // 给切换角
    double switch_armor_angle = v_yaw_ < max_track_target_vel_ ?
                                    (acos(r1_ / target_rho) - max_switch_angle +
                                     (-acos(r1_ / target_rho) + (max_switch_angle + min_switch_angle)) *
                                         std::abs(v_yaw_) / max_track_target_vel_) :
                                    min_switch_angle;
    if (v_yaw_ < max_track_target_vel_)
    {
      // 判断这一块是否超过切换角，超过计算下一块的切换角
      if (((((yaw_ + v_yaw_ * rough_fly_time) > output_yaw + switch_armor_angle) && v_yaw_ > 0.) ||
           (((yaw_ + v_yaw_ * rough_fly_time) < output_yaw - switch_armor_angle) && v_yaw_ < 0.)) &&
          std::abs(v_yaw_) >= 1.0)
      {
        double next_switch_armor_angle = v_yaw_ < max_track_target_vel_ ?
                                             (acos(r2_ / target_rho) - max_switch_angle +
                                              (-acos(r2_ / target_rho) + (max_switch_angle + min_switch_angle)) *
                                                  std::abs(v_yaw_) / max_track_target_vel_) :
                                             min_switch_angle;
        // 判断下一块是否超过切换，追踪back这一块，back是用来看曲线判断是否进入中心模式的，不追击下两块，追击曲线太丑陋
        if (((((yaw_ - (M_PI * 2 / armors_num_) + v_yaw_ * rough_fly_time) > output_yaw + next_switch_armor_angle) &&
              v_yaw_ > 0.) ||
             (((yaw_ + (M_PI * 2 / armors_num_) + v_yaw_ * rough_fly_time) < output_yaw - next_switch_armor_angle) &&
              v_yaw_ < 0.)) &&
            std::abs(v_yaw_) >= 1.0)
          target_armor_ = BACK;
        else
          target_armor_ = v_yaw_ > 0. ? LEFT : RIGHT;  // 不追击下两块，那就追击下一块的
      }
      else
        target_armor_ = FRONT;  // 不追击下一块，那就追击当前一块
                                // 左0,正对1,右边2,后面3
      // 如果这一块经过发弹延迟和飞行时间超过切换角，就开始切换，切换状态变为开始切换
      if (((((yaw_ + (M_PI * 2 / armors_num_) * (target_armor_ - 1) + v_yaw_ * (rough_fly_time + delay_)) >
             output_yaw + switch_armor_angle) &&
            v_yaw_ > 0.) ||
           (((yaw_ + (M_PI * 2 / armors_num_) * (target_armor_ - 1) + v_yaw_ * (rough_fly_time + delay_)) <
             output_yaw - switch_armor_angle) &&
            v_yaw_ < 0.)) &&
          std::abs(v_yaw_) >= 1.0)
        switch_armor_state_ = READY_SWITCH;  // 注意开始切换是加上了发弹延迟，实际是还没有切换
    }
    else  // 中心模式
    {
      // 如果这一块打不中
      if (((((yaw_ + v_yaw_ * (rough_fly_time + delay_)) > output_yaw + switch_armor_angle) && v_yaw_ > 0.) ||
           (((yaw_ + v_yaw_ * (rough_fly_time + delay_)) < output_yaw - switch_armor_angle) && v_yaw_ < 0.)) &&
          std::abs(v_yaw_) >= 1.0)
      {
        // 判断能不能打中下两块
        if (((((yaw_ - (M_PI * 2 / armors_num_) + v_yaw_ * (rough_fly_time + delay_)) >
               output_yaw + switch_armor_angle) &&
              v_yaw_ > 0.) ||
             (((yaw_ + (M_PI * 2 / armors_num_) + v_yaw_ * (rough_fly_time + delay_)) <
               output_yaw - switch_armor_angle) &&
              v_yaw_ < 0.)) &&
            std::abs(v_yaw_) >= 1.0)
          target_armor_ = BACK;
        else
          target_armor_ = v_yaw_ > 0. ? LEFT : RIGHT;  // 不用下两块就打下一块
      }
      else
        target_armor_ = FRONT;  // 不用打下一块就打这一块
    }

    // 之前打的是前面那块，现在打的不是前面那块
    // 之前打的是左右两块中的一块，现在打的是后面那块，感觉这个情况可以pass
    // 切换状态变为开始切换
    if ((current_armor_ == FRONT && target_armor_ != FRONT) ||
        ((current_armor_ == LEFT || current_armor_ == RIGHT) && target_armor_ == BACK))
      switch_armor_state_ = START_SWITCH;
    // 如果没有经历过这一块变为打下一块
    else if (switch_armor_state_ != READY_SWITCH)
      switch_armor_state_ = NO_SWITCH;  // 切换状态为没有切换
    current_armor_ = target_armor_;     // 更新上一次跟随目标，用于下一次比较
    return target_armor_;               // 将跟随目标返回
  }

  int getSwitchArmorState()
  {
    return switch_armor_state_;
  }

private:
  geometry_msgs::Point pos_;
  geometry_msgs::Vector3 vel_;
  double yaw_{}, v_yaw_{}, r1_{}, r2_{};
  double delay_{}, bullet_speed_{}, resistance_coff_{}, max_track_target_vel_{};
  int current_armor_{ 1 }, target_armor_{ 1 };
  int switch_armor_state_{ 0 };
  int armors_num_{};
};
}  // namespace rm_gimbal_controllers
