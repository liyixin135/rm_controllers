//
// Created by guanlin on 25-4-3.
//

#include "rm_gimbal_controllers/bullet_solver/solver.h"

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
              .wait_next_armor_delay = getParam(controller_nh, "wait_next_armor_delay", 0.105),
              .wait_diagonal_armor_delay = getParam(controller_nh, "wait_diagonal_armor_delay", 0.105),
              .dt = getParam(controller_nh, "dt", 0.),
              .timeout = getParam(controller_nh, "timeout", 0.),
              .ban_shoot_duration = getParam(controller_nh, "ban_shoot_duration", 0.0),
              .gimbal_switch_duration = getParam(controller_nh, "gimbal_switch_duration", 0.0),
              .max_switch_angle = getParam(controller_nh, "max_switch_angle", 40.0),
              .min_switch_angle = getParam(controller_nh, "min_switch_angle", 2.0),
              .min_shoot_beforehand_vel = getParam(controller_nh, "min_shoot_beforehand_vel", 4.5),
              .max_chassis_angular_vel = getParam(controller_nh, "max_chassis_angular_vel", 8.5),
              .max_track_target_vel = getParam(controller_nh, "max_track_target_vel", 10.0),
              .track_rotate_target_delay = getParam(controller_nh, "track_rotate_target_delay", 0.),
              .track_move_target_delay = getParam(controller_nh, "track_move_target_delay", 0.),
              .min_fit_switch_count = getParam(controller_nh, "min_fit_switch_count", 3) };
  config_rt_buffer_.initRT(config_);

  d_srv_ = new dynamic_reconfigure::Server<rm_gimbal_controllers::BulletSolverConfig>(controller_nh);
  dynamic_reconfigure::Server<rm_gimbal_controllers::BulletSolverConfig>::CallbackType cb =
      [this](auto&& PH1, auto&& PH2) { reconfigCB(PH1, PH2); };
  d_srv_->setCallback(cb);

  tracked_target_kinematic_ = std::make_unique<TrackedTargetKinematic>();
  untracked_target_kinematic_ = std::make_unique<UntrackedTargetKinematic>();
  target_selector_ = std::make_unique<TargetSelector>();
  state_pub_ = std::make_unique<realtime_tools::RealtimePublisher<rm_msgs::BulletSolverState>>(
      controller_nh, "bullet_solver_state", 10.);
}

void BulletSolver::selectTarget(geometry_msgs::Point pos, geometry_msgs::Vector3 vel, double bullet_speed, double yaw,
                                double v_yaw, double r1, double r2, double dz, int armors_num)
{
  config_ = *config_rt_buffer_.readFromRT();
  bullet_speed_ = bullet_speed;
  resistance_coff_ = getResistanceCoefficient(bullet_speed_) != 0 ? getResistanceCoefficient(bullet_speed_) : 0.001;
  // 将全局变量所都赋值进去
  target_selector_->reset(pos, vel, yaw, v_yaw, r1, r2, bullet_speed, resistance_coff_, config_.max_track_target_vel,
                          config_.delay, armors_num);
  // current_switch_state_是int类型,后面那个东西是切换装甲板的状态，noswitch,readyswitch,startswitch
  // 当前状态是没有切换，获取状态是准备切换，准备切换时间更新
  if (current_switch_state_ == NO_SWITCH && target_selector_->getSwitchArmorState() == READY_SWITCH)
    ready_switch_armor_time_ = ros::Time::now();
  // 获取状态是开始切换，切换时间更新
  if (target_selector_->getSwitchArmorState() == START_SWITCH)
    switch_armor_time_ = ros::Time::now();
  // 更新当前状态
  current_switch_state_ = target_selector_->getSwitchArmorState();
  // 获取当前跟随装甲板
  target_armor_ = target_selector_->getTargetArmor();
  double r = (target_armor_ == FRONT || target_armor_ == BACK) ? r1 : r2;
  yaw += (M_PI * 2 / armors_num) * (target_armor_ - 1);
  if (target_armor_ == LEFT || target_armor_ == RIGHT)
    pos.z += dz;
  if (std::abs(v_yaw) < config_.max_track_target_vel)
  {
    track_target_ = true;
    tracked_target_kinematic_->reset(pos, vel, yaw, v_yaw, r);
  }
  else
  {
    track_target_ = false;
    untracked_target_kinematic_->reset(pos, vel, yaw, v_yaw, r);
  }
}

bool BulletSolver::solve()
{
  int count{};
  double error = 999;

  target_pos_ = track_target_ ? tracked_target_kinematic_->position(0.) : untracked_target_kinematic_->position(0.);

  double temp_z = target_pos_.z;
  while (error >= 0.001)
  {
    output_yaw_ = std::atan2(target_pos_.y, target_pos_.x);
    double target_rho = std::sqrt(std::pow(target_pos_.x, 2) + std::pow(target_pos_.y, 2));
    output_pitch_ = std::atan2(temp_z, target_rho);
    fly_time_ =
        (-std::log(1 - target_rho * resistance_coff_ / (bullet_speed_ * std::cos(output_pitch_)))) / resistance_coff_;
    double real_z = (bullet_speed_ * std::sin(output_pitch_) + (config_.g / resistance_coff_)) *
                        (1 - std::exp(-resistance_coff_ * fly_time_)) / resistance_coff_ -
                    config_.g * fly_time_ / resistance_coff_;

    target_pos_ = track_target_ ? tracked_target_kinematic_->position(fly_time_) :
                                  untracked_target_kinematic_->position(fly_time_);

    double target_yaw = std::atan2(target_pos_.y, target_pos_.x);
    double error_theta = target_yaw - output_yaw_;
    double error_z = target_pos_.z - real_z;
    temp_z += error_z;
    error = std::sqrt(std::pow(error_theta * target_rho, 2) + std::pow(error_z, 2));
    count++;

    if (count >= 20 || std::isnan(error))
      return false;
  }
  return true;
}

double BulletSolver::getGimbalError(double yaw_real, double pitch_real)
{
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
    double delay = config_.delay;
    geometry_msgs::Point target_pos_after_fly_time_and_delay{};
    target_pos_after_fly_time_and_delay = untracked_target_kinematic_->position(fly_time_ + delay);
    error = std::sqrt(std::pow(target_pos_.x - target_pos_after_fly_time_and_delay.x, 2) +
                      std::pow(target_pos_.y - target_pos_after_fly_time_and_delay.y, 2) +
                      std::pow(target_pos_.z - target_pos_after_fly_time_and_delay.z, 2));
  }
  return error;
}

void BulletSolver::getSelectedArmorPosAndVel(geometry_msgs::Point& armor_pos, geometry_msgs::Vector3& armor_vel)
{
  if (track_target_)
  {
    armor_pos = tracked_target_kinematic_->position(0.);
    armor_vel = tracked_target_kinematic_->velocity(0.);
  }
  else
  {
    armor_pos = untracked_target_kinematic_->position(0.);
    armor_vel = untracked_target_kinematic_->velocity(0.);
  }
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

void BulletSolver::publishState()
{
  if (state_pub_->trylock())
  {
    state_pub_->msg_.fly_time = fly_time_;
    state_pub_->msg_.current_armor = target_armor_;
    state_pub_->unlockAndPublish();
  }
}

void BulletSolver::judgeShootBeforehand(const ros::Time& time, double v_yaw)
{
  int shoot_beforehand_cmd{};
  if (!track_target_)
    shoot_beforehand_cmd = rm_msgs::ShootBeforehandCmd::JUDGE_BY_ERROR;
  else if (target_selector_->getSwitchArmorState() == READY_SWITCH &&
           (ros::Time::now() - ready_switch_armor_time_).toSec() < config_.gimbal_switch_duration)
    shoot_beforehand_cmd = rm_msgs::ShootBeforehandCmd::BAN_SHOOT;
  else if ((ros::Time::now() - switch_armor_time_).toSec() < config_.gimbal_switch_duration - config_.delay)
    shoot_beforehand_cmd = rm_msgs::ShootBeforehandCmd::BAN_SHOOT;
  else if (((ros::Time::now() - switch_armor_time_).toSec() < config_.gimbal_switch_duration))
    shoot_beforehand_cmd = rm_msgs::ShootBeforehandCmd::ALLOW_SHOOT;
  else
    shoot_beforehand_cmd = rm_msgs::ShootBeforehandCmd::JUDGE_BY_ERROR;
  if (shoot_beforehand_cmd_pub_->trylock())
  {
    shoot_beforehand_cmd_pub_->msg_.stamp = time;
    shoot_beforehand_cmd_pub_->msg_.cmd = shoot_beforehand_cmd;
    shoot_beforehand_cmd_pub_->unlockAndPublish();
  }
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
    config.wait_next_armor_delay = init_config.wait_next_armor_delay;
    config.wait_diagonal_armor_delay = init_config.wait_diagonal_armor_delay;
    config.dt = init_config.dt;
    config.timeout = init_config.timeout;
    config.ban_shoot_duration = init_config.ban_shoot_duration;
    config.gimbal_switch_duration = init_config.gimbal_switch_duration;
    config.max_switch_angle = init_config.max_switch_angle;
    config.min_switch_angle = init_config.min_switch_angle;
    config.min_shoot_beforehand_vel = init_config.min_shoot_beforehand_vel;
    config.max_chassis_angular_vel = init_config.max_chassis_angular_vel;
    config.max_track_target_vel = init_config.max_track_target_vel;
    config.track_rotate_target_delay = init_config.track_rotate_target_delay;
    config.track_move_target_delay = init_config.track_move_target_delay;
    config.min_fit_switch_count = init_config.min_fit_switch_count;
    dynamic_reconfig_initialized_ = true;
  }
  Config config_non_rt{ .resistance_coff_qd_10 = config.resistance_coff_qd_10,
                        .resistance_coff_qd_15 = config.resistance_coff_qd_15,
                        .resistance_coff_qd_16 = config.resistance_coff_qd_16,
                        .resistance_coff_qd_18 = config.resistance_coff_qd_18,
                        .resistance_coff_qd_30 = config.resistance_coff_qd_30,
                        .g = config.g,
                        .delay = config.delay,
                        .wait_next_armor_delay = config.wait_next_armor_delay,
                        .wait_diagonal_armor_delay = config.wait_diagonal_armor_delay,
                        .dt = config.dt,
                        .timeout = config.timeout,
                        .ban_shoot_duration = config.ban_shoot_duration,
                        .gimbal_switch_duration = config.gimbal_switch_duration,
                        .max_switch_angle = config.max_switch_angle,
                        .min_switch_angle = config.min_switch_angle,
                        .min_shoot_beforehand_vel = config.min_shoot_beforehand_vel,
                        .max_chassis_angular_vel = config.max_chassis_angular_vel,
                        .max_track_target_vel = config.max_track_target_vel,
                        .track_rotate_target_delay = config.track_rotate_target_delay,
                        .track_move_target_delay = config.track_move_target_delay,
                        .min_fit_switch_count = config.min_fit_switch_count };
  config_rt_buffer_.writeFromNonRT(config_non_rt);
}
}  // namespace rm_gimbal_controllers
