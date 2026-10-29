/**
 *******************************************************************************
 * @file      :chassis.cpp
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
#include "chassis.hpp"

#include <cmath>
#include "motor_base.hpp"
#include "pid.hpp"

namespace robot {
bool Chassis::controlReady() const {
  if(!configured_ || params_.max_wheel_speed_rad_s<=0.0f)return false;
  for(auto *pid:pids_)if(pid==nullptr||!pid->controlReady())return false;
  return true;
}

bool Chassis::ToChassisFrame(const ChassisCommand &command,
                             float relative_yaw_rad,
                             ChassisFrameVelocity &body) {
  body = {};
  if (!IsFinite(command.vx) || !IsFinite(command.vy) ||
      !IsFinite(command.w) || !IsFinite(relative_yaw_rad)) return false;
  constexpr float kPi = 3.14159265358979323846f;
  constexpr float kTwoPi = 2.0f * kPi;
  float delta = std::fmod(relative_yaw_rad, kTwoPi);
  if (delta >= kPi) delta -= kTwoPi;
  if (delta < -kPi) delta += kTwoPi;
  // delta = 云台 - 底盘。云台左转 90 度时，云台向前应变为底盘向左。
  const auto rotated = RotatePlanar(command, std::cos(delta), std::sin(delta));
  if (!IsFinite(rotated.vx) || !IsFinite(rotated.vy)) return false;
  body = rotated;
  return true;
}

bool Chassis::init(const ChassisParams &params, const Motors &motors,
                   const Pids &pids) {
  stop();
  configured_ = false;
  motors_ = {};
  pids_ = {};
  if (!ValidParams(params)) return false;
  for (std::size_t i = 0; i < motors.size(); ++i) {
    if (motors[i] == nullptr || pids[i] == nullptr || motors[i]->id() != i + 1 ||
        motors[i]->get_input_type() != hello_world::motor::InputType::kRaw)
      return false;
    for (std::size_t j = 0; j < i; ++j)
      if (pids[i] == pids[j] || motors[i] == motors[j]) return false;
  }
  params_ = params;
  motors_ = motors;
  pids_ = pids;
  configured_ = true;
  stop();
  return true;
}

void Chassis::stop() {
  for (auto *motor : motors_)
    if (motor != nullptr) motor->setInput(0.0f);
  for (auto *pid : pids_)
    if (pid != nullptr) pid->reset();
  result_ = {};
}

bool Chassis::update(const ChassisControlInput &input) {
  const auto fail = [this]() { stop(); return false; };
  if (!configured_ || !input.enabled || !input.command_valid ||
      !input.relative_yaw_valid || !IsFinite(input.dt_s) ||
      input.dt_s <= 0.0f || input.dt_s > 0.02f)
    return fail();
  WheelValues feedback{};
  for (std::size_t i = 0; i < motors_.size(); ++i) {
    if (!input.wheel_feedback_valid[i] ||
        motors_[i]->get_input_type() != hello_world::motor::InputType::kRaw)
      return fail();
    // vel() 已应用 dir 和减速比，单位为轮子输出端 rad/s，不再二次除减速比。
    feedback[i] = motors_[i]->vel();
    if (!IsFinite(feedback[i])) return fail();
  }
  ChassisFrameVelocity body{};
  ChassisResult next{};
  if (!ToChassisFrame(input.command, input.relative_yaw_rad, body) ||
      !SolveBody(body, params_, next)) return fail();
  // 有效的全零速度命令表示零电流停车；不执行制动PID，也不保留积分。
  // 保留输入、反馈和配置检查；停车成功不应使Robot误判为控制故障。
  if(input.command.vx==0.0f&&input.command.vy==0.0f&&input.command.w==0.0f){
    stop();
    next.active=true;
    result_=next;
    return true;
  }
  WheelValues feedforward{};
  for (std::size_t i = 0; i < motors_.size(); ++i) {
    feedforward[i] = params_.velocity_feedforward_gain[i] * next.wheel_target_rad_s[i];
    if (!IsFinite(feedforward[i]) ||
        !IsFinite(next.wheel_target_rad_s[i] - feedback[i])) return fail();
  }
  // 先算完四轮，再写电机对象；前馈与 PID 共用 Pid 内部的总输出限幅。
  for (std::size_t i = 0; i < motors_.size(); ++i) {
    next.raw_current[i] = pids_[i]->calculate(
        next.wheel_target_rad_s[i], feedback[i], input.dt_s, feedforward[i]);
    if (!IsFinite(next.raw_current[i])) return fail();
  }
  for (std::size_t i = 0; i < motors_.size(); ++i) {
    const auto status = motors_[i]->setInput(next.raw_current[i]);
    if (status == hello_world::motor::Status::kInputTypeError) return fail();
    // 电机组件还有硬限幅，记录实际值，不能把轮速直接当电流码发送。
    next.raw_current[i] = motors_[i]->getInput();
  }
  next.active = true;
  result_ = next;
  return true;
}
}  // namespace robot
