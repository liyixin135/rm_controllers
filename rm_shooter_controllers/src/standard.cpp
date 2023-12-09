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
// Created by huakang on 2021/1/18.
//

#include <rm_common/ros_utilities.h>
#include <string>
#include <pluginlib/class_list_macros.hpp>
#include "rm_shooter_controllers/standard.h"

namespace rm_shooter_controllers
{
bool Controller::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  //给config_定义
  config_ = { .block_effort = getParam(controller_nh, "block_effort", 0.),
              .block_speed = getParam(controller_nh, "block_speed", 0.),
              .block_duration = getParam(controller_nh, "block_duration", 0.),
              .block_overtime = getParam(controller_nh, "block_overtime", 0.),
              .anti_block_angle = getParam(controller_nh, "anti_block_angle", 0.),
              .anti_block_threshold = getParam(controller_nh, "anti_block_threshold", 0.),
              .forward_push_threshold = getParam(controller_nh, "forward_push_threshold", 0.1),
              .exit_push_threshold = getParam(controller_nh, "exit_push_threshold", 0.1),
              .extra_wheel_speed = getParam(controller_nh, "extra_wheel_speed", 0.) };
  //缓存初始化config_
  config_rt_buffer.initRT(config_);
  //拿参拿参
  push_per_rotation_ = getParam(controller_nh, "push_per_rotation", 0);
  push_wheel_speed_threshold_ = getParam(controller_nh, "push_wheel_speed_threshold", 0.);

  //命令的订阅者，就是把话题拿出来实现
  cmd_subscriber_ = controller_nh.subscribe<rm_msgs::ShootCmd>("command", 1, &Controller::commandCB, this);
  //发布shoot状态的发布者
  shoot_state_pub_.reset(new realtime_tools::RealtimePublisher<rm_msgs::ShootState>(controller_nh, "state", 10));
  // Init dynamic reconfigure，动态调参用的
  d_srv_ = new dynamic_reconfigure::Server<rm_shooter_controllers::ShooterConfig>(controller_nh);
  dynamic_reconfigure::Server<rm_shooter_controllers::ShooterConfig>::CallbackType cb = [this](auto&& PH1, auto&& PH2) {
    reconfigCB(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2));
  };
  d_srv_->setCallback(cb);

  //给左右摩擦轮和拨盘定义句柄
  ros::NodeHandle nh_friction_l = ros::NodeHandle(controller_nh, "friction_left");
  ros::NodeHandle nh_friction_r = ros::NodeHandle(controller_nh, "friction_right");
  ros::NodeHandle nh_trigger = ros::NodeHandle(controller_nh, "trigger");
  //effort_joint_interface_可以命令基于力距控制的关节
  effort_joint_interface_ = robot_hw->get<hardware_interface::EffortJointInterface>();
  return ctrl_friction_l_.init(effort_joint_interface_, nh_friction_l) &&
         ctrl_friction_r_.init(effort_joint_interface_, nh_friction_r) &&
         ctrl_trigger_.init(effort_joint_interface_, nh_trigger);
}

//进入一次starting
void Controller::starting(const ros::Time& /*time*/)
{
  state_ = STOP;
  state_changed_ = true;
}

//starting之后来到update
void Controller::update(const ros::Time& time, const ros::Duration& period)
{
  //从缓存区拿命令和参数
  cmd_ = *cmd_rt_buffer_.readFromRT();
  config_ = *config_rt_buffer.readFromRT();

  //其实就是在换模式，当模式和命令不一样了就可以进第一道if
  if (state_ != cmd_.mode)
  {
    if (state_ != BLOCK)//不是block堵塞了就可以进第二道if
      if ((state_ != PUSH || cmd_.mode != READY) ||
          (cmd_.mode == READY &&
           std::fmod(std::abs(ctrl_trigger_.command_struct_.position_ - ctrl_trigger_.getPosition()), 2. * M_PI) <
               config_.exit_push_threshold))//情况一，如果是stop或者
      {
        state_ = cmd_.mode;//state变为命令状态
        state_changed_ = true;//改标志位，已成功改变状态
      }
  }

  if (state_ != STOP)
    setSpeed(cmd_);
  //选择模式进入
  switch (state_)
  {
    case READY:
      ready(period);
      break;
    case PUSH:
      push(time, period);
      break;
    case STOP:
      stop(time, period);
      break;
    case BLOCK:
      block(time, period);
      break;
  }
  // 如果能获得锁，就可以发布时间，状态
  if (shoot_state_pub_->trylock())
  {
    shoot_state_pub_->msg_.stamp = time;
    shoot_state_pub_->msg_.state = state_;
    shoot_state_pub_->unlockAndPublish();
  }
  //向关节发布命令
  ctrl_friction_l_.update(time, period);
  ctrl_friction_r_.update(time, period);
  ctrl_trigger_.update(time, period);
}

// 就是停住
void Controller::stop(const ros::Time& time, const ros::Duration& period)
{
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Shooter] Enter STOP");

    ctrl_friction_l_.setCommand(0.);
    ctrl_friction_r_.setCommand(0.);
    ctrl_trigger_.setCommand(ctrl_trigger_.joint_.getPosition());  // 是拿拨盘当前位置
  }
}

//ready去normalize去函数看
void Controller::ready(const ros::Duration& period)
{
  //打印信息
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Shooter] Enter READY");

    normalize();
  }
}

void Controller::push(const ros::Time& time, const ros::Duration& period)
{
  //打印信息
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Shooter] Enter PUSH");
  }
  // 对于摩擦轮，要么速度为0,要么转起来了，速度绝对值大于命令的百分之90,且大于2pai。然后再要求时间达到所设的赫兹就可以进去这个if
  if ((cmd_.wheel_speed == 0. ||
       (ctrl_friction_l_.joint_.getVelocity() >= push_wheel_speed_threshold_ * ctrl_friction_l_.command_ &&
        ctrl_friction_l_.joint_.getVelocity() > M_PI &&
        ctrl_friction_r_.joint_.getVelocity() <= push_wheel_speed_threshold_ * ctrl_friction_r_.command_ &&
        ctrl_friction_r_.joint_.getVelocity() < -M_PI)) &&
      (time - last_shoot_time_).toSec() >= 1. / cmd_.hz)
  {  // Time to shoot!!!
    if (std::fmod(std::abs(ctrl_trigger_.command_struct_.position_ - ctrl_trigger_.getPosition()), 2. * M_PI) <
        config_.forward_push_threshold)  // 防止命令的位置距离现有位置推出去太多
    {
      // 命令拨盘拨一个弹位
      ctrl_trigger_.setCommand(ctrl_trigger_.command_struct_.position_ -
                               2. * M_PI / static_cast<double>(push_per_rotation_));
      last_shoot_time_ = time;  // 更新时间
    }
    // Check block，情况一，力和速度比设定的block要大，情况二，上次发射的时间已经长于设定的频率并且速度比设定的block要大
    if ((ctrl_trigger_.joint_.getEffort() < -config_.block_effort &&
         std::abs(ctrl_trigger_.joint_.getVelocity()) < config_.block_speed) ||
        ((time - last_shoot_time_).toSec() > 1 / cmd_.hz &&
         std::abs(ctrl_trigger_.joint_.getVelocity()) < config_.block_speed))
    {
      if (!maybe_block_)  // 进来后maybe_block_就变成true了
      {
        block_time_ = time;  // 记录block_time_时间
        maybe_block_ = true;
      }
      if ((time - block_time_).toSec() >= config_.block_duration)  // 如果block的时间长于设定的时间
      {
        state_ = BLOCK;  // 状态变为block
        state_changed_ = true;
        ROS_INFO("[Shooter] Exit PUSH");  // 打印退出push
      }
    }
    else  // 满足以上if情况，maybe_block_就是true
      maybe_block_ = false;
  }
  else
    ROS_DEBUG("[Shooter] Wait for friction wheel");  // 给摩擦轮速度命令了，但是摩擦轮没转起来，shooter控制器在等摩擦轮
}

void Controller::block(const ros::Time& time, const ros::Duration& period)
{
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Shooter] Enter BLOCK");
    last_block_time_ = time;
    ctrl_trigger_.setCommand(ctrl_trigger_.joint_.getPosition() + config_.anti_block_angle);
  }
  if (std::abs(ctrl_trigger_.command_struct_.position_ - ctrl_trigger_.joint_.getPosition()) <
          config_.anti_block_threshold ||
      (time - last_block_time_).toSec() > config_.block_overtime)
  {
    normalize();

    state_ = PUSH;
    state_changed_ = true;
    ROS_INFO("[Shooter] Exit BLOCK");
  }
}

//命令摩擦轮转，后面那个是在manual里面调节速度的，按一下+或者-5
void Controller::setSpeed(const rm_msgs::ShootCmd& cmd)
{
  ctrl_friction_l_.setCommand(cmd_.wheel_speed + config_.extra_wheel_speed);
  ctrl_friction_r_.setCommand(-cmd_.wheel_speed - config_.extra_wheel_speed);
}

void Controller::normalize()
{
  //拨盘push角度等于2pai除以洞数
  double push_angle = 2. * M_PI / static_cast<double>(push_per_rotation_);
  //offset生效的地方
  ctrl_trigger_.setCommand(push_angle * std::floor((ctrl_trigger_.joint_.getPosition() + 0.01) / push_angle));
}

//为了动态调参写的，重新加载参数
void Controller::reconfigCB(rm_shooter_controllers::ShooterConfig& config, uint32_t /*level*/)
{
  ROS_INFO("[Shooter] Dynamic params change");
  if (!dynamic_reconfig_initialized_)
  {
    Config init_config = *config_rt_buffer.readFromNonRT();  // config init use yaml
    config.block_effort = init_config.block_effort;
    config.block_speed = init_config.block_speed;
    config.block_duration = init_config.block_duration;
    config.block_overtime = init_config.block_overtime;
    config.anti_block_angle = init_config.anti_block_angle;
    config.anti_block_threshold = init_config.anti_block_threshold;
    config.forward_push_threshold = init_config.forward_push_threshold;
    config.exit_push_threshold = init_config.exit_push_threshold;
    config.extra_wheel_speed = init_config.extra_wheel_speed;
    dynamic_reconfig_initialized_ = true;
  }
  Config config_non_rt{ .block_effort = config.block_effort,
                        .block_speed = config.block_speed,
                        .block_duration = config.block_duration,
                        .block_overtime = config.block_overtime,
                        .anti_block_angle = config.anti_block_angle,
                        .anti_block_threshold = config.anti_block_threshold,
                        .forward_push_threshold = config.forward_push_threshold,
                        .exit_push_threshold = config.exit_push_threshold,
                        .extra_wheel_speed = config.extra_wheel_speed };
  config_rt_buffer.writeFromNonRT(config_non_rt);
}

}  // namespace rm_shooter_controllers

PLUGINLIB_EXPORT_CLASS(rm_shooter_controllers::Controller, controller_interface::ControllerBase)
