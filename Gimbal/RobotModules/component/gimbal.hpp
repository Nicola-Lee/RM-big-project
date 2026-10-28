/**
 *******************************************************************************
 * @file      :gimbal.hpp
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
#ifndef GIMBAL_ROBOT_MODULES_GIMBAL_HPP_
#define GIMBAL_ROBOT_MODULES_GIMBAL_HPP_

#include <limits>
#include "module_state.hpp"

namespace hello_world::motor { class DaMiao; }
namespace hello_world::pid { class Pid; }

namespace robot {
struct GimbalParams {
  // 以下角度均使用 motor->angle() 标定后的关节坐标：yaw 向左、pitch 下俯为正。
  // 未测量时保持 false；不能用零值冒充实车限位。
  bool pitch_limits_calibrated = false;
  float pitch_min_rad = 0.0f;
  float pitch_max_rad = 0.0f;
  // 目标变化率限制（不是实际速度保护）；0 表示未配置，禁止位置控制。
  float yaw_target_rate_rad_s = 0.0f;
  float pitch_target_rate_rad_s = 0.0f;
  // N*m；需结合实际机构设置，不能直接照搬电机最大转矩。
  float yaw_torque_limit_nm = 0.0f;
  float pitch_torque_limit_nm = 0.0f;
  // 速度前馈作用于内部限速后的目标变化率，单位 N*m/(rad/s)。
  float yaw_velocity_ff = 0.0f;
  float pitch_velocity_ff = 0.0f;
  // 重力补偿 = A*cos(pitch) + B*sin(pitch)，N*m；系数包含零点/重心方向。
  // 这是固定底盘姿态下的标定模型，未使用 IMU 做全姿态重力补偿。
  float pitch_gravity_cos_nm = 0.0f;
  float pitch_gravity_sin_nm = 0.0f;
};

struct GimbalControlInput {
  GimbalMotion target{};  // 关节角目标，rad；yaw 是周期角，不表示多圈位置命令
  GimbalMode mode = GimbalMode::Disabled;
  float dt_s = 0.001f;
  bool enabled = false;  // Robot 允许正常控制；不等于请求发送使能命令
  bool target_valid = false;
  bool yaw_feedback_valid = false;    // 必须含通信新鲜度，不能只判断数值有限
  bool pitch_feedback_valid = false;
};

struct GimbalResult {
  GimbalMotion target{};       // 限位、限速后的内部目标
  GimbalMotion feedback{};     // 实际关节角
  GimbalMotion error{};
  float yaw_torque_nm = 0.0f;
  float pitch_torque_nm = 0.0f;
  bool active = false;
  bool target_captured = false;  // 本周期为重新进入控制，捕获当前位置并保持零转矩
  bool pitch_target_clamped = false;
  bool pitch_output_blocked = false;
  bool yaw_pid_restarted = false;  // 最短角误差跨 +/-pi，已清除 D/I 历史
};

class Gimbal {
 public:
  static constexpr float kPi = 3.14159265358979323846f;
  static constexpr float kTwoPi = 2.0f * kPi;
  // 只借用对象；初始化/重新配置与 update 均应在同一控制上下文串行进行。
  bool init(const GimbalParams &params,
            hello_world::motor::DaMiao *yaw, hello_world::motor::DaMiao *pitch,
            hello_world::pid::Pid *yaw_pid, hello_world::pid::Pid *pitch_pid);
  bool update(const GimbalControlInput &input);
  // 清零转矩和控制历史；电机 enable/disable 由 Robot/通信层统一安排。
  void stop();
  bool configured() const { return configured_; }
  bool controlReady() const;
  float pitchMin() const { return params_.pitch_min_rad; }
  float pitchMax() const { return params_.pitch_max_rad; }
  float yawTargetRateLimit() const { return params_.yaw_target_rate_rad_s; }
  float pitchTargetRateLimit() const { return params_.pitch_target_rate_rad_s; }
  const GimbalResult &result() const { return result_; }

  static bool NormalizeYaw(float angle, float &normalized);
  static constexpr bool IsFinite(float v) {
    return v >= -std::numeric_limits<float>::max() &&
           v <= std::numeric_limits<float>::max();
  }
  static constexpr float Clamp(float v, float low, float high) {
    return v < low ? low : (v > high ? high : v);
  }
  // 调用者先归一化两个角；其差值在 [-2pi,2pi] 内，单次修正足够。
  // 固定选择 [-pi,pi)，恰好相差180度时选择负方向。
  static constexpr float WrapDifference(float difference) {
    return difference >= kPi ? difference - kTwoPi
         : difference < -kPi ? difference + kTwoPi : difference;
  }
  static constexpr bool ValidParams(const GimbalParams &p) {
    if (!IsFinite(p.pitch_min_rad) || !IsFinite(p.pitch_max_rad) ||
        !IsFinite(p.yaw_target_rate_rad_s) || p.yaw_target_rate_rad_s < 0.0f ||
        !IsFinite(p.pitch_target_rate_rad_s) || p.pitch_target_rate_rad_s < 0.0f ||
        !IsFinite(p.yaw_torque_limit_nm) || p.yaw_torque_limit_nm < 0.0f ||
        !IsFinite(p.pitch_torque_limit_nm) || p.pitch_torque_limit_nm < 0.0f ||
        !IsFinite(p.yaw_velocity_ff) || !IsFinite(p.pitch_velocity_ff) ||
        !IsFinite(p.pitch_gravity_cos_nm) || !IsFinite(p.pitch_gravity_sin_nm))
      return false;
    // pitch 工作区间不跨电机反馈的 +/-pi 分界；两端须留在分界内。
    return !p.pitch_limits_calibrated ||
           (p.pitch_min_rad > -kPi && p.pitch_max_rad < kPi &&
            p.pitch_min_rad < p.pitch_max_rad);
  }
  static constexpr bool ControlParamsReady(const GimbalParams &p) {
    return ValidParams(p) && p.pitch_limits_calibrated &&
           p.yaw_target_rate_rad_s > 0.0f && p.pitch_target_rate_rad_s > 0.0f &&
           p.yaw_torque_limit_nm > 0.0f && p.pitch_torque_limit_nm > 0.0f;
  }
  // 非零转矩必须经过这一判定；电机自身状态另由 update 检查。
  static constexpr bool InputReady(const GimbalControlInput &input) {
    return input.enabled && input.mode == GimbalMode::JointAngle &&
           input.target_valid && input.yaw_feedback_valid && input.pitch_feedback_valid &&
           IsFinite(input.dt_s) && input.dt_s > 0.0f && input.dt_s <= 0.02f &&
           IsFinite(input.target.yaw) && IsFinite(input.target.pitch);
  }
  // desired.yaw/previous.yaw 已归一化；参数和 dt 已检查。无 HAL 依赖。
  static constexpr GimbalMotion StepTarget(const GimbalMotion &desired,
                                           const GimbalMotion &previous,
                                           const GimbalParams &p, float dt_s) {
    const float yaw_step = Clamp(WrapDifference(desired.yaw - previous.yaw),
                                 -p.yaw_target_rate_rad_s * dt_s,
                                  p.yaw_target_rate_rad_s * dt_s);
    const float pitch_goal = Clamp(desired.pitch, p.pitch_min_rad, p.pitch_max_rad);
    const float pitch_step = Clamp(pitch_goal - previous.pitch,
                                   -p.pitch_target_rate_rad_s * dt_s,
                                    p.pitch_target_rate_rad_s * dt_s);
    return {WrapDifference(previous.yaw + yaw_step),
            Clamp(previous.pitch + pitch_step, p.pitch_min_rad, p.pitch_max_rad)};
  }
  static constexpr bool YawBranchChanged(float error, float previous_error) {
    const float difference = error - previous_error;
    return difference > kPi || difference < -kPi;
  }
  // 下俯为正；达到边界时只阻断继续向外的转矩，允许向内恢复。
  static constexpr float LimitPitchTorque(float angle, float torque,
                                           float low, float high) {
    return ((angle <= low && torque < 0.0f) ||
            (angle >= high && torque > 0.0f)) ? 0.0f : torque;
  }

 private:
  GimbalParams params_{};
  hello_world::motor::DaMiao *yaw_ = nullptr;
  hello_world::motor::DaMiao *pitch_ = nullptr;
  hello_world::pid::Pid *yaw_pid_ = nullptr;
  hello_world::pid::Pid *pitch_pid_ = nullptr;
  GimbalResult result_{};
  bool configured_ = false;
  bool target_initialized_ = false;
  bool has_yaw_error_ = false;
  float last_yaw_error_ = 0.0f;
};
}  // namespace robot
#endif
