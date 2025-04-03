//
// Created by guanlin on 25-4-3.
//

#pragma once

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace rm_gimbal_controllers
{
class TargetKinematicsBase
{
public:
  TargetKinematicsBase() = default;
  virtual ~TargetKinematicsBase() = default;
  virtual geometry_msgs::Point position(double time) = 0;
  virtual geometry_msgs::Vector3 velocity(double time) = 0;
  void reset(geometry_msgs::Point pos, geometry_msgs::Vector3 vel, double yaw, double v_yaw, double r)
  {
    pos_ = pos;
    vel_ = vel;
    yaw_ = yaw;
    v_yaw_ = v_yaw;
    r_ = r;
  }

protected:
  geometry_msgs::Point pos_;
  geometry_msgs::Vector3 vel_;
  double yaw_;
  double v_yaw_;
  double r_;
};

class TrackedTargetKinematic final : public TargetKinematicsBase
{
public:
  TrackedTargetKinematic() : TargetKinematicsBase()
  {
  }
  geometry_msgs::Point position(double time) override
  {
    geometry_msgs::Point target_pos;
    target_pos.x = pos_.x + vel_.x * time - r_ * cos(yaw_ + v_yaw_ * time);
    target_pos.y = pos_.y + vel_.y * time - r_ * sin(yaw_ + v_yaw_ * time);
    target_pos.z = pos_.z + vel_.z * time;
    return target_pos;
  }
  geometry_msgs::Vector3 velocity(double time) override
  {
    geometry_msgs::Vector3 target_vel;
    target_vel.x = vel_.x + v_yaw_ * r_ * sin(yaw_ + v_yaw_ * time);
    target_vel.y = vel_.y - v_yaw_ * r_ * cos(yaw_ + v_yaw_ * time);
    return target_vel;
  }
};

class UntrackedTargetKinematic final : public TargetKinematicsBase
{
public:
  UntrackedTargetKinematic() : TargetKinematicsBase()
  {
  }
  geometry_msgs::Point position(double time) override
  {
    geometry_msgs::Point target_pos;
    double target_center_pos[2];
    target_center_pos[0] = pos_.x + vel_.x * time;
    target_center_pos[1] = pos_.y + vel_.y * time;
    target_pos.x = target_center_pos[0] - r_ * cos(atan2(target_center_pos[1], target_center_pos[0]));
    target_pos.y = target_center_pos[1] - r_ * sin(atan2(target_center_pos[1], target_center_pos[0]));
    target_pos.z = pos_.z + vel_.z * time;
    return target_pos;
  }
  geometry_msgs::Vector3 velocity(double /*time*/) override
  {
    geometry_msgs::Vector3 target_vel;
    return target_vel;
  }
};
}  // namespace rm_gimbal_controllers
