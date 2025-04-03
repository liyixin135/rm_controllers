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
             double dz, double bullet_speed, double resistance_coff)
  {
    pos_ = pos;
    vel_ = vel;
    yaw_ = yaw;
    v_yaw_ = v_yaw;
    r1_ = r1;
    r2_ = r2;
    dz_ = dz;
    bullet_speed_ = bullet_speed;
    resistance_coff_ = resistance_coff;
  }
  int getArmor()
  {
    // some select logic
    return FRONT;
  }

private:
  geometry_msgs::Point pos_;
  geometry_msgs::Vector3 vel_;
  double bullet_speed_{}, yaw_{}, v_yaw_{}, r1_{}, r2_{}, dz_{}, resistance_coff_{};
};
}  // namespace rm_gimbal_controllers
