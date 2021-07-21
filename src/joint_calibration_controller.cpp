//
// Created by qiayuan on 5/16/21.
//

#include "rm_calibration_controllers/joint_calibration_controller.h"

#include <pluginlib/class_list_macros.hpp>

namespace rm_calibration_controllers {

bool JointCalibrationController::init(hardware_interface::RobotHW *robot_hw,
                                      ros::NodeHandle &root_nh,
                                      ros::NodeHandle &controller_nh) {

  velocity_ctrl_.init(robot_hw->get<hardware_interface::EffortJointInterface>(), controller_nh);
  XmlRpc::XmlRpcValue actuators;
  if (!controller_nh.getParam("actuators", actuators)) {
    ROS_ERROR("No actuators given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  for (int i = 0; i < actuators.size(); ++i)
    actuators_.push_back(robot_hw->get<hardware_interface::ActuatorExtraInterface>()->getHandle(actuators[i]));
  if (!controller_nh.getParam("search_velocity", vel_search_)) {
    ROS_ERROR("Velocity value was not specified (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("threshold", threshold_)) {
    ROS_ERROR("Velocity value was not specified (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (threshold_ < 0) {
    threshold_ *= -1.;
    ROS_ERROR("Negative velocity threshold is not supported for joint %s. Making the velocity threshold positive.",
              velocity_ctrl_.getJointName().c_str());
  }
  // advertise service to check calibration
  is_calibrated_srv_ = controller_nh.advertiseService("is_calibrated", &JointCalibrationController::isCalibrated, this);
  return true;
}

void JointCalibrationController::starting(const ros::Time &time) {
  for (auto &actuator:actuators_) {
    actuator.setOffset(0.0);
    actuator.setCalibrated(false);
    state_ = INITIALIZED;
    if (actuator.getCalibrated())
      ROS_INFO("Joint %s will be recalibrated, but was already calibrated at offset %f",
               velocity_ctrl_.getJointName().c_str(), actuator.getOffset());
  }
}

void JointCalibrationController::update(const ros::Time &time, const ros::Duration &period) {
  //TODO: Add GPIO switch support
  switch (state_) {
    case INITIALIZED: {
      velocity_ctrl_.setCommand(vel_search_);
      countdown_ = 100;
      state_ = MOVING;
      break;
    }
    case MOVING: {
      bool halted = false;
      for (const auto &actuator:actuators_) {
        halted |= actuator.getHalted();
      }
      if (std::abs(velocity_ctrl_.joint_.getVelocity()) < threshold_ && !halted) {
        countdown_--;
      } else
        countdown_ = 100;
      if (countdown_ < 0) {
        velocity_ctrl_.setCommand(0);
        for (auto &actuator:actuators_) {
          actuator.setOffset(-actuator.getPosition());
          actuator.setCalibrated(true);
        }
        state_ = CALIBRATED;
        ROS_INFO("Joint %s calibrated", velocity_ctrl_.getJointName().c_str());
      }
      break;
    }
    case CALIBRATED: {
      velocity_ctrl_.joint_.setCommand(0.);
      break;
    }
  }
  velocity_ctrl_.update(time, period);
}

bool JointCalibrationController::isCalibrated(control_msgs::QueryCalibrationState::Request &req,
                                              control_msgs::QueryCalibrationState::Response &resp) {
  ROS_DEBUG("Is calibrated service %d", state_ == CALIBRATED);
  resp.is_calibrated = (state_ == CALIBRATED);
  return true;
}

}

PLUGINLIB_EXPORT_CLASS(rm_calibration_controllers::JointCalibrationController, controller_interface::ControllerBase)
