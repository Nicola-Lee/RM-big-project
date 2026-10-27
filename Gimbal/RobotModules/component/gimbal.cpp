/**
 *******************************************************************************
 * @file      :gimbal.cpp
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
#include "gimbal.hpp"
#include <cmath>
#include "motor_DaMiao.hpp"
#include "pid.hpp"

namespace robot {
bool Gimbal::NormalizeYaw(float angle, float &normalized) {
  normalized = 0.0f;
  if (!IsFinite(angle)) return false;
  normalized = WrapDifference(std::fmod(angle, kTwoPi));
  return true;
}

bool Gimbal::init(const GimbalParams &params,
                  hello_world::motor::DaMiao *yaw, hello_world::motor::DaMiao *pitch,
                  hello_world::pid::Pid *yaw_pid, hello_world::pid::Pid *pitch_pid) {
  stop();
  configured_ = false;
  yaw_ = pitch_ = nullptr;
  yaw_pid_ = pitch_pid_ = nullptr;
  if (!ValidParams(params) || yaw == nullptr || pitch == nullptr || yaw == pitch ||
      yaw_pid == nullptr || pitch_pid == nullptr || yaw_pid == pitch_pid ||
      yaw->id() != 2 || pitch->id() != 1 ||
      yaw->get_input_type() != hello_world::motor::InputType::kTorq ||
      pitch->get_input_type() != hello_world::motor::InputType::kTorq)
    return false;
  params_ = params;
  yaw_ = yaw;
  pitch_ = pitch;
  yaw_pid_ = yaw_pid;
  pitch_pid_ = pitch_pid;
  configured_ = true;
  stop();
  return true;
}

void Gimbal::stop() {
  if (yaw_ != nullptr) yaw_->setInput(0.0f);
  if (pitch_ != nullptr) pitch_->setInput(0.0f);
  if (yaw_pid_ != nullptr) yaw_pid_->reset();
  if (pitch_pid_ != nullptr) pitch_pid_->reset();
  result_ = {};
  target_initialized_ = false;
  has_yaw_error_ = false;
  last_yaw_error_ = 0.0f;
}

bool Gimbal::update(const GimbalControlInput &input) {
  const auto fail = [this]() { stop(); return false; };
  if (!configured_ || !ControlParamsReady(params_) || !InputReady(input)) return fail();
  using Dm = hello_world::motor::DaMiao;
  // 驱动收到故障状态时 is_enabled 可能仍保留旧值，必须同时核对状态码。
  if (!yaw_->is_enabled() || !pitch_->is_enabled() ||
      yaw_->status_code() != Dm::StatusCode::kMotorEnabled ||
      pitch_->status_code() != Dm::StatusCode::kMotorEnabled ||
      yaw_->current_ctrl_mode() != Dm::CtrlMode::kMIT ||
      pitch_->current_ctrl_mode() != Dm::CtrlMode::kMIT ||
      yaw_->get_input_type() != hello_world::motor::InputType::kTorq ||
      pitch_->get_input_type() != hello_world::motor::InputType::kTorq)
    return fail();

  GimbalResult next{};
  GimbalMotion desired = input.target;
  next.feedback.pitch = pitch_->angle();
  if (!NormalizeYaw(yaw_->angle(), next.feedback.yaw) ||
      !NormalizeYaw(input.target.yaw, desired.yaw) || !IsFinite(next.feedback.pitch) ||
      next.feedback.pitch < -kPi || next.feedback.pitch >= kPi)
    return fail();
  next.pitch_target_clamped = desired.pitch < params_.pitch_min_rad ||
                              desired.pitch > params_.pitch_max_rad;

  if (!target_initialized_) {
    // 第一个有效周期只建立目标；忽略旧目标，不输出恢复瞬间的力矩突变。
    // 若反馈已超软件边界，捕获边界目标，下一周期只允许向范围内恢复。
    next.target = {next.feedback.yaw,
                   Clamp(next.feedback.pitch, params_.pitch_min_rad, params_.pitch_max_rad)};
    yaw_->setInput(0.0f);
    pitch_->setInput(0.0f);
    yaw_pid_->reset();
    pitch_pid_->reset();
    next.active = true;
    next.target_captured = true;
    result_ = next;
    target_initialized_ = true;
    return true;
  }

  // 限制每周期的目标变化；先检查乘法，避免极端配置产生无穷大。
  if (!IsFinite(params_.yaw_target_rate_rad_s * input.dt_s) ||
      !IsFinite(params_.pitch_target_rate_rad_s * input.dt_s)) return fail();
  next.target = StepTarget(desired, result_.target, params_, input.dt_s);
  next.error = {WrapDifference(next.target.yaw - next.feedback.yaw),
                next.target.pitch - next.feedback.pitch};
  // 最短角方向在 +/-pi 处切换时，清除该轴全部 PID 历史，避免 D 项 2pi/dt 尖峰。
  if (has_yaw_error_ && YawBranchChanged(next.error.yaw, last_yaw_error_)) {
    yaw_pid_->reset();
    next.yaw_pid_restarted = true;
  }
  const float yaw_rate = WrapDifference(next.target.yaw - result_.target.yaw) / input.dt_s;
  const float pitch_rate = (next.target.pitch - result_.target.pitch) / input.dt_s;
  const float yaw_ff = params_.yaw_velocity_ff * yaw_rate;
  const float pitch_ff = params_.pitch_velocity_ff * pitch_rate +
      params_.pitch_gravity_cos_nm * std::cos(next.feedback.pitch) +
      params_.pitch_gravity_sin_nm * std::sin(next.feedback.pitch);
  if (!IsFinite(yaw_ff) || !IsFinite(pitch_ff)) return fail();
  const float yaw_torque = yaw_pid_->calculateError(next.error.yaw, input.dt_s, yaw_ff);
  const float pitch_torque = pitch_pid_->calculateError(next.error.pitch, input.dt_s, pitch_ff);
  if (!IsFinite(yaw_torque) || !IsFinite(pitch_torque)) return fail();
  next.yaw_torque_nm = Clamp(yaw_torque, -params_.yaw_torque_limit_nm, params_.yaw_torque_limit_nm);
  const float pitch_limited = Clamp(pitch_torque, -params_.pitch_torque_limit_nm, params_.pitch_torque_limit_nm);
  next.pitch_torque_nm = LimitPitchTorque(next.feedback.pitch, pitch_limited,
                                         params_.pitch_min_rad, params_.pitch_max_rad);
  next.pitch_output_blocked = next.pitch_torque_nm != pitch_limited;
  // 模块限幅比 PID 限幅更小时，清除积分历史，避免持续积累受限方向的输出。
  if (next.yaw_torque_nm != yaw_torque) yaw_pid_->reset();
  if (next.pitch_torque_nm != pitch_torque) pitch_pid_->reset();

  const auto yaw_status = yaw_->setInput(next.yaw_torque_nm);
  const auto pitch_status = pitch_->setInput(next.pitch_torque_nm);
  using Status = hello_world::motor::Status;
  if (yaw_status == Status::kInputTypeError || pitch_status == Status::kInputTypeError)
    return fail();
  if (yaw_status == Status::kInputValueOverflow) yaw_pid_->reset();
  if (pitch_status == Status::kInputValueOverflow) pitch_pid_->reset();
  next.yaw_torque_nm = yaw_->getInput();
  next.pitch_torque_nm = pitch_->getInput();
  if (!IsFinite(next.yaw_torque_nm) || !IsFinite(next.pitch_torque_nm)) return fail();
  last_yaw_error_ = next.error.yaw;
  has_yaw_error_ = true;
  next.active = true;
  result_ = next;
  return true;
}
}  // namespace robot
