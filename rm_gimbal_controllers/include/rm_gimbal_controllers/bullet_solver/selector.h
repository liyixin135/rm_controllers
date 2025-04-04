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
class TargetSelector
{
public:
  TargetSelector() = default;
  void reset(geometry_msgs::Point pos, geometry_msgs::Vector3 vel, double yaw, double v_yaw, double r1, double r2,
             double bullet_speed, double resistance_coff, double max_track_target_vel, double delay)
  {
    pos_ = pos;
    vel_ = vel;
    yaw_ = yaw;
    v_yaw_ = v_yaw;
    r1_ = r1;
    r2_ = r2;
    bullet_speed_ = bullet_speed;
    resistance_coff_ = resistance_coff;
    max_track_target_vel_ = max_track_target_vel;
  }
  int getArmor()
  {
    double target_rho = std::sqrt(std::pow(pos_.x, 2) + std::pow(pos_.y, 2));
    double output_yaw = std::atan2(pos_.y, pos_.x);
    double output_pitch = std::atan2(pos_.z, std::sqrt(std::pow(pos_.x, 2) + std::pow(pos_.y, 2)));
    double rough_fly_time =
        (-std::log(1 - target_rho * resistance_coff_ / (bullet_speed_ * std::cos(output_pitch)))) / resistance_coff_;
    double max_switch_angle = 50 / 180 * M_PI;
    double min_switch_angle = 0 / 180 * M_PI;
    double switch_armor_angle = v_yaw_ < max_track_target_vel_ ?
                                    (acos(r1_ / target_rho) - max_switch_angle +
                                     (-acos(r1_ / target_rho) + (max_switch_angle + min_switch_angle)) *
                                         std::abs(v_yaw_) / max_track_target_vel_) :
                                    min_switch_angle;
    if (v_yaw_ < max_track_target_vel_)
    {
      if (((((yaw_ + v_yaw_ * rough_fly_time) > output_yaw + switch_armor_angle) && v_yaw_ > 0.) ||
           (((yaw_ + v_yaw_ * rough_fly_time) < output_yaw - switch_armor_angle) && v_yaw_ < 0.)) &&
          std::abs(v_yaw_) >= 1.0)
      {
        double next_switch_armor_angle = v_yaw_ < max_track_target_vel_ ?
                                             (acos(r2_ / target_rho) - max_switch_angle +
                                              (-acos(r2_ / target_rho) + (max_switch_angle + min_switch_angle)) *
                                                  std::abs(v_yaw_) / max_track_target_vel_) :
                                             min_switch_angle;
        if (((((yaw_ - M_PI / 2 + v_yaw_ * rough_fly_time) > output_yaw + next_switch_armor_angle) && v_yaw_ > 0.) ||
             (((yaw_ + M_PI / 2 + v_yaw_ * rough_fly_time) < output_yaw - next_switch_armor_angle) && v_yaw_ < 0.)) &&
            std::abs(v_yaw_) >= 1.0)
          return BACK;
        else
          return v_yaw_ > 0. ? LEFT : RIGHT;
      }
      else
        return FRONT;
    }
    else
    {
      if (((((yaw_ + v_yaw_ * (rough_fly_time + delay_)) > output_yaw + switch_armor_angle) && v_yaw_ > 0.) ||
           (((yaw_ + v_yaw_ * (rough_fly_time + delay_)) < output_yaw - switch_armor_angle) && v_yaw_ < 0.)) &&
          std::abs(v_yaw_) >= 1.0)
      {
        if (((((yaw_ - M_PI / 2 + v_yaw_ * (rough_fly_time + delay_)) > output_yaw + switch_armor_angle) &&
              v_yaw_ > 0.) ||
             (((yaw_ + M_PI / 2 + v_yaw_ * (rough_fly_time + delay_)) < output_yaw - switch_armor_angle) &&
              v_yaw_ < 0.)) &&
            std::abs(v_yaw_) >= 1.0)
          return BACK;
        else
          return v_yaw_ > 0. ? LEFT : RIGHT;
      }
      else
        return FRONT;
    }
  }

private:
  geometry_msgs::Point pos_;
  geometry_msgs::Vector3 vel_;
  double bullet_speed_{}, yaw_{}, v_yaw_{}, r1_{}, r2_{}, resistance_coff_{};
  double max_track_target_vel_{}, delay_{};
};
}  // namespace rm_gimbal_controllers
