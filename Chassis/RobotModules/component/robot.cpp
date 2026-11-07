/**
 *******************************************************************************
 * @file      :robot.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
#include "robot.hpp"
#include "robot_limits.hpp"
#include "imu.hpp"
#include "comm_task.hpp"
#include "tick.hpp"
#include "runtime_monitor.hpp"

namespace robot {
namespace {
bool Positive(float v) {
  return control::Finite(v) && v > 0.0f;
}

bool AllWheels(const WheelFeedbackSnapshot &s) {
  for (bool v : s.valid)
    if (!v)
      return false;
  return true;
}
}

bool Robot::init(const ChassisRobotParams &p,
                 Chassis *chassis,
                 hello_world::imu::Imu *imu,
                 GimbalChassisComm *comm) {
  if (initialized_ || chassis == nullptr || imu == nullptr || comm == nullptr ||
      p.ready_stable_ms == 0 || p.recovery_timeout_ms <= p.ready_stable_ms ||
      p.recovery_timeout_ms > 0x7FFFFFFFu)
    return false;
  params_ = p;
  chassis_ = chassis;
  imu_ = imu;
  comm_ = comm;
  follow_pid_.init(p.follow_pid);
  machine_.configure(p.ready_stable_ms, p.recovery_timeout_ms);
  initialized_ = true;
  resetControl();
  publish();
  return true;
}

bool Robot::paramsReady() const {
  return initialized_ && chassis_->controlReady() && follow_pid_.controlReady() &&
         params_.max_body_rate_rad_s == limits::kMaxBodyRateRadS &&
         Positive(params_.max_axis_speed_m_s) &&
         params_.max_axis_speed_m_s <= board_protocol::kMaxVelocity &&
         Positive(params_.max_body_rate_rad_s) &&
         params_.max_body_rate_rad_s <= board_protocol::kMaxVelocity &&
         Positive(params_.linear_accel_m_s2) && Positive(params_.angular_accel_rad_s2) &&
         control::Finite(params_.follow_filter_tau_s) && params_.follow_filter_tau_s >= 0.0f &&
         Positive(params_.follow_align_enter_rad) &&
         params_.follow_align_enter_rad < params_.follow_align_exit_rad &&
         params_.follow_align_exit_rad < control::kPi;
}

void Robot::resetControl() {
  if (chassis_ != nullptr)
    chassis_->stop();
  follow_pid_.reset();
  follow_rate_filter_.reset();
  follow_alignment_.reset();
  last_command_ = {};
  last_mode_ = ChassisMode::Disabled;
  ever_active_ = false;
  has_follow_error_ = false;
  last_follow_error_ = 0;
}

void Robot::publish() {
  GimbalChassisComm::ChassisData state{};
  state.life = machine_.state();
  state.local_ready = machine_.localReady();
  state.mode = diagnostic_.chassis_mode;
  state.emergency_stop = state.life == LifeState::Dead;
  // 手动云台目标在云台生成，C2G可选角度接口始终不激活。
  state.target_valid = false;
  comm_->setLocalChassis(state);
  diagnostic_.life = machine_.state();
  diagnostic_.reason = machine_.reason();
  diagnostic_.local_ready = machine_.localReady();
}

void Robot::update(float dt) {
  if (!initialized_)
    return;
  const uint32_t now = hello_world::tick::GetTickMs();
  const bool late = updated_ && static_cast<uint32_t>(now - diagnostic_.last_update_ms) > 20;
  diagnostic_.last_update_ms = now;
  updated_ = true;
  hello_world::imu::ImuSnapshot imu{};
  const bool imu_ok = imu_->getSnapshot(imu);
  WheelFeedbackSnapshot wheels{};
  GetWheelFeedbackSnapshot(wheels);
  const bool motors_ok = AllWheels(wheels);
  GimbalChassisComm::GimbalSnapshot peer{};
  comm_->getGimbalSnapshot(peer);
  GimbalChassisComm::VelocitySnapshot velocity{};
  const bool command_ok = comm_->getVelocitySnapshot(velocity);
  control::LifeInput in{};
  in.operator_permitted = peer.online && peer.data.remote_online && peer.data.operator_armed;
  in.peer_ready = peer.online && peer.data.local_ready;
  in.peer_working = peer.online && peer.data.life == LifeState::Working;
  in.peer_stopped = peer.data.emergency_stop;
  in.local_ready = imu_ok && motors_ok;
  in.local_reason = imu_ok ? control::StopReason::MotorNotReady : control::StopReason::ImuNotReady;
  if (!peer.online)
    in.inhibit = control::StopReason::PeerOffline;
  else if (!peer.data.remote_online)
    in.inhibit = control::StopReason::RemoteOffline;
  else if (!peer.data.operator_armed)
    in.inhibit = control::StopReason::EmergencyStop;
  else if (!runtime::CanMotionAllowed())
    in.inhibit = control::StopReason::CommunicationFault;
  else if (!paramsReady() || imu_->state() == hello_world::imu::ImuState::Uninitialized)
    in.inhibit = control::StopReason::Configuration;
  else if (imu_->state() == hello_world::imu::ImuState::Fault)
    in.inhibit = control::StopReason::ImuNotReady;
  else if (late || !control::Finite(dt) || dt <= 0 || dt > 0.02f)
    in.inhibit = control::StopReason::Timing;
  else
    in.inhibit = control::StopReason::None;
  const auto previous = machine_.state();
  machine_.step(in, now);
  if (previous != machine_.state()) {
    resetControl();
    if (machine_.state() == LifeState::Working)
      working_since_ = now;
  }
  diagnostic_.active = false;
  diagnostic_.chassis_mode = velocity.data.mode;
  const bool peer_running = machine_.canRun(in.peer_working, in.peer_stopped);
  const bool yaw_ok =
      peer.data.yaw_feedback_valid && board_protocol::AngleValid(peer.data.feedback.yaw);
  bool active = peer_running && command_ok && yaw_ok;
  if (peer_running && !active &&
      (ever_active_ || static_cast<uint32_t>(now - working_since_) >= 100u)) {
    machine_.abort(control::StopReason::CommandInvalid);
    resetControl();
  }
  if (active && machine_.state() == LifeState::Working) {
    const auto mode = velocity.data.mode;
    if (mode != last_mode_) {
      follow_pid_.reset();
      follow_rate_filter_.reset();
      follow_alignment_.reset();
      has_follow_error_ = false;
      last_command_ = {};
      last_mode_ = mode;
    }
    const auto &request = velocity.data.command;
    ChassisCommand next{};
    next.vx = control::Slew(
        last_command_.vx,
        control::Clamp(request.vx, -params_.max_axis_speed_m_s, params_.max_axis_speed_m_s),
        params_.linear_accel_m_s2,
        dt);
    next.vy = control::Slew(
        last_command_.vy,
        control::Clamp(request.vy, -params_.max_axis_speed_m_s, params_.max_axis_speed_m_s),
        params_.linear_accel_m_s2,
        dt);
    float requested_rate = request.w;
    bool follow_aligned = false;

    if (mode == ChassisMode::Follow) {
      const float yaw_error = control::Wrap(peer.data.feedback.yaw);

      // 误差跨过±π时，清除上一侧的PID和滤波历史。
      if (has_follow_error_ &&
          control::Abs(yaw_error - last_follow_error_) > control::kPi) {
        follow_pid_.reset();
        follow_rate_filter_.reset();
      }

      follow_aligned = follow_alignment_.update(
          yaw_error,
          imu.gyro[2],
          now,
          peer.age_ms,
          params_.follow_align_enter_rad,
          params_.follow_align_exit_rad);

      if (follow_aligned) {
        follow_pid_.reset();
        requested_rate = 0.0f;
      } else {
        requested_rate = follow_pid_.calculateError(yaw_error, dt);
      }

      last_follow_error_ = yaw_error;
      has_follow_error_ = true;
    } else if (mode != ChassisMode::Separate && mode != ChassisMode::Spin) {
      active = false;
    }

    const float limited_rate = control::Clamp(
        requested_rate, -params_.max_body_rate_rad_s, params_.max_body_rate_rad_s);
    if (mode == ChassisMode::Follow && limited_rate != requested_rate) {
      follow_pid_.reset();
    }

    // 跟随角速度先低通，再通过角加速度限制。
    float rate_target = limited_rate;
    if (mode == ChassisMode::Follow) {
      rate_target = follow_rate_filter_.update(limited_rate, params_.follow_filter_tau_s, dt);
    }
    next.w = control::Slew(
        last_command_.w, rate_target, params_.angular_accel_rad_s2, dt);

    const bool no_translation = request.vx == 0.0f && request.vy == 0.0f;
    if (mode == ChassisMode::Follow && limited_rate == 0.0f) {
      // 对齐且仍有角度误差时，只让旋转停下，平移继续沿自身斜坡减速。
      if (no_translation && (!follow_aligned || last_follow_error_ == 0.0f)) {
        next.vx = 0.0f;
        next.vy = 0.0f;
      }

      // 滤波尾值足够小，而且本周期能在限加速度内停稳时，才把角速度置零。
      constexpr float kFollowStoppedRateRadS = 0.000001f;
      const float max_rate_step = params_.angular_accel_rad_s2 * dt;
      if ((follow_aligned || no_translation) &&
          control::Abs(rate_target) <= kFollowStoppedRateRadS &&
          control::Abs(last_command_.w) <= max_rate_step) {
        next.w = 0.0f;
        follow_rate_filter_.reset();
      }
    } else if (mode != ChassisMode::Follow &&
               no_translation &&
               limited_rate == 0.0f) {
      next = {};
      follow_rate_filter_.reset();
    }

    ChassisControlInput input{};
    input.command = next;
    input.relative_yaw_rad = peer.data.feedback.yaw;
    input.dt_s = dt;
    input.enabled = active;
    input.command_valid = command_ok;
    input.relative_yaw_valid = yaw_ok;
    input.wheel_feedback_valid = wheels.valid;
    if (!active || !chassis_->update(input)) {
      machine_.abort(control::StopReason::ControlFailed);
      resetControl();
    } else {
      last_command_ = next;
      ever_active_ = true;
      diagnostic_.active = true;
    }
  } else {
    // 未获运行许可直接停车，不让低通历史延长输出。
    chassis_->stop();
    follow_rate_filter_.reset();
    follow_alignment_.reset();
  }
  action_ = diagnostic_.active
                ? control::MotorAction::Control
                : (machine_.state() == LifeState::Dead ? control::MotorAction::Disable
                                                       : control::MotorAction::EnableZero);
  publish();
}

control::MotorAction Robot::motorAction() const {
  if (!runtime::CanMotionAllowed() || !initialized_ || !updated_ ||
      static_cast<uint32_t>(hello_world::tick::GetTickMs() - diagnostic_.last_update_ms) >= 20u)
    return control::MotorAction::Disable;
  if (action_ != control::MotorAction::Control)
    return action_;
  // 编码前再检查来源时效，不能把周期开始时的许可无限延长。
  GimbalChassisComm::GimbalSnapshot peer{};
  GimbalChassisComm::VelocitySnapshot command{};
  hello_world::imu::ImuSnapshot imu{};
  WheelFeedbackSnapshot wheels{};
  GetWheelFeedbackSnapshot(wheels);
  if (!imu_->getSnapshot(imu) || !AllWheels(wheels) || !comm_->getGimbalSnapshot(peer) ||
      !comm_->getVelocitySnapshot(command) || !peer.data.operator_armed ||
      !peer.data.remote_online || peer.data.emergency_stop || !peer.data.local_ready ||
      peer.data.life != LifeState::Working)
    return control::MotorAction::Disable;
  return action_;
}
} // namespace robot
