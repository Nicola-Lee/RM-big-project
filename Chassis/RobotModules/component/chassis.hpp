/**
 *******************************************************************************
 * @file      :chassis.hpp
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
#ifndef CHASSIS_ROBOT_MODULES_CHASSIS_HPP_
#define CHASSIS_ROBOT_MODULES_CHASSIS_HPP_

#include <array>
#include <cstddef>
#include <limits>
#include "module_state.hpp"

namespace hello_world::motor { class Motor; }
namespace hello_world::pid { class Pid; }

namespace robot {
// 数组固定顺序：左前 ID1、左后 ID2、右后 ID3、右前 ID4。
using WheelValues = std::array<float, 4>;

struct ChassisParams {
  float wheel_radius_m = 0.07786f;
  float half_length_m = 0.15338f;  // 前后轮心距的一半
  float half_width_m = 0.15338f;   // 左右轮心距的一半
  // 输出轴/轮子的角速度上限，rad/s。0 表示尚未配置，禁止控制输出。
  float max_wheel_speed_rad_s = 0.0f;
  // 速度前馈：kV * 限幅后的目标轮速；单位 raw/(rad/s)，默认关闭。
  WheelValues velocity_feedforward_gain{};
};

struct ChassisControlInput {
  ChassisCommand command{};       // 所有模式下，平移指令均为云台坐标系
  float relative_yaw_rad = 0.0f;  // 云台朝向 - 底盘朝向，向左为正
  float dt_s = 0.001f;
  bool enabled = false;          // 由 Robot 的生命状态/急停判定提供
  bool command_valid = false;    // 包括命令来源在线和数据未超时
  bool relative_yaw_valid = false;  // 包括角度来源在线和数据未超时
  std::array<bool, 4> wheel_feedback_valid{};  // 四轮各自的反馈时效
};

struct ChassisResult {
  ChassisFrameVelocity requested_body{};  // 变换后、轮速限幅前
  ChassisFrameVelocity limited_body{};    // 与限幅后的轮速相对应
  WheelValues wheel_target_rad_s{};
  WheelValues raw_current{};              // 实际写入电机对象的 kRaw 电流码
  float wheel_scale = 0.0f;
  bool active = false;
};

class Chassis {
 public:
  using Motors = std::array<hello_world::motor::Motor *, 4>;
  using Pids = std::array<hello_world::pid::Pid *, 4>;
  // 仅初始化/重新配置时调用；借用四个独立电机、PID 对象，不分配堆内存。
  bool init(const ChassisParams &params, const Motors &motors, const Pids &pids);
  // 由同一个控制任务串行调用；每周期必须重新提供输入和有效性。
  // 无效输入/失能：四轮零电流，清除目标、前馈结果及 PID 历史，返回 false。
  bool update(const ChassisControlInput &input);
  void stop();
  const ChassisResult &result() const { return result_; }
  bool configured() const { return configured_; }
  static bool ToChassisFrame(const ChassisCommand &command,
                             float relative_yaw_rad,
                             ChassisFrameVelocity &body);
  // R(delta)；c、s 分别为 cos(delta)、sin(delta)，由 ToChassisFrame 提供。
  static constexpr ChassisFrameVelocity RotatePlanar(
      const ChassisCommand &command, float c, float s) {
    return {command.vx * c - command.vy * s,
            command.vx * s + command.vy * c, command.w};
  }
  static constexpr bool IsFinite(float value) {
    return value >= -std::numeric_limits<float>::max() &&
           value <= std::numeric_limits<float>::max();
  }
  static constexpr bool ValidParams(const ChassisParams &p) {
    if (!IsFinite(p.wheel_radius_m) || p.wheel_radius_m <= 0.0f ||
        !IsFinite(p.half_length_m) || p.half_length_m <= 0.0f ||
        !IsFinite(p.half_width_m) || p.half_width_m <= 0.0f ||
        !IsFinite(p.max_wheel_speed_rad_s) || p.max_wheel_speed_rad_s < 0.0f)
      return false;
    for (float gain : p.velocity_feedforward_gain)
      if (!IsFinite(gain) || gain < 0.0f) return false;
    return true;
  }
  // X 型 45 度麦轮。正轮速表示该轮推动车体向前；电机 dir 需实机标定。
  // 纯计算接口：不访问电机/PID，可用于静态检查或离线观察。
  static constexpr bool SolveBody(const ChassisFrameVelocity &body,
                                  const ChassisParams &p,
                                  ChassisResult &out) {
    out = {};
    if (!ValidParams(p) || p.max_wheel_speed_rad_s <= 0.0f ||
        !IsFinite(body.vx) || !IsFinite(body.vy) || !IsFinite(body.w))
      return false;
    // double 中间量避免有限 float 输入在求和/除半径时溢出。
    const double vx = body.vx, vy = body.vy;
    const double turn = (static_cast<double>(p.half_length_m) + p.half_width_m) * body.w;
    const double r = p.wheel_radius_m;
    const std::array<double, 4> wheels{
        (vx - vy - turn) / r, (vx + vy - turn) / r,
        (vx - vy + turn) / r, (vx + vy + turn) / r};
    double peak = 0.0;
    for (double wheel : wheels) {
      const double magnitude = wheel < 0.0 ? -wheel : wheel;
      if (magnitude > peak) peak = magnitude;
    }
    const double scale = peak > p.max_wheel_speed_rad_s
                             ? p.max_wheel_speed_rad_s / peak : 1.0;
    out.requested_body = body;
    out.limited_body = {static_cast<float>(vx * scale),
                        static_cast<float>(vy * scale),
                        static_cast<float>(static_cast<double>(body.w) * scale)};
    out.wheel_scale = static_cast<float>(scale);
    for (std::size_t i = 0; i < wheels.size(); ++i)
      out.wheel_target_rad_s[i] = static_cast<float>(wheels[i] * scale);
    return true;
  }
 private:
  ChassisParams params_{};
  Motors motors_{};
  Pids pids_{};
  ChassisResult result_{};
  bool configured_ = false;
};
}  // namespace robot
#endif
