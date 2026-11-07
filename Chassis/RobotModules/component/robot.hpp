/**
 *******************************************************************************
 * @file      :robot.hpp
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
#ifndef CHASSIS_ROBOT_MODULES_ROBOT_HPP_
#define CHASSIS_ROBOT_MODULES_ROBOT_HPP_
#include "chassis.hpp"
#include "pid.hpp"
#include "gimbal_chassis_comm.hpp"
#include "robot_control.hpp"
namespace hello_world::imu {class Imu;}
namespace robot {
// 仅供底盘跟随角速度使用；保存的output_单位为rad/s。
class FollowRateFilter {
 public:
  constexpr void reset() {
    output_ = 0.0f;
  }

  constexpr float update(float input, float tau_s, float dt_s) {
    if (!control::Finite(input) || !control::Finite(tau_s) || tau_s < 0.0f ||
        !control::Finite(dt_s) || dt_s <= 0.0f || dt_s > 0.02f) {
      reset();
      return output_;
    }
    if (tau_s == 0.0f) {
      output_ = input; // 时间常数为0时直接旁路。
      return output_;
    }
    const float alpha = dt_s / (tau_s + dt_s);
    output_ += alpha * (input - output_);
    return output_;
  }

  constexpr float output() const {
    return output_;
  }

 private:
  float output_ = 0.0f;
};

// 跟随外环的对齐滞回。角速度由相隔至少20ms的两次云台反馈估算。
class FollowAlignment {
 public:
  constexpr void reset() {
    *this = {};
  }

  constexpr bool update(float yaw_error,
                        float body_rate,
                        uint32_t now,
                        uint32_t feedback_age,
                        float enter_angle,
                        float exit_angle) {
    constexpr uint32_t kRateWindowMs = 20;
    constexpr float kEnterRateRadS = 0.08f;
    constexpr float kResumeRelativeRateRadS = 0.16f;

    // 反馈过旧或底盘角速度无效时，清除对齐状态和角速度历史。
    if (feedback_age > kRateWindowMs || !control::Finite(body_rate)) {
      reset();
      return false;
    }
    // 用反馈收到的时间计算采样间隔，避免把重复读到的帧当成新样本。
    const uint32_t sample_ms = now - feedback_age;
    if (!sample_started_) {
      sample_started_ = true;
      sample_ms_ = sample_ms;
      sample_error_ = yaw_error;
    }

    const uint32_t elapsed_ms = sample_ms - sample_ms_;
    const bool rate_updated = elapsed_ms >= kRateWindowMs;
    if (rate_updated) {
      rate_valid_ = elapsed_ms <= 2 * kRateWindowMs;
      relative_rate_ = control::Wrap(yaw_error - sample_error_) /
                       (static_cast<float>(elapsed_ms) * 0.001f);
      sample_ms_ = sample_ms;
      sample_error_ = yaw_error;
    }

    const float angle = control::Abs(yaw_error);
    const float relative_speed = control::Abs(relative_rate_);

    if (aligned_) {
      // 已对齐时，用退出角度和恢复速度判断是否重新跟随。
      if (angle >= exit_angle ||
          !rate_valid_ ||
          relative_speed >= kResumeRelativeRateRadS) {
        aligned_ = false;
      }
    } else {
      // 进入对齐需要新算出的相对角速度，也需要底盘本身接近静止。
      if (angle <= enter_angle &&
          rate_updated &&
          rate_valid_ &&
          relative_speed <= kEnterRateRadS &&
          control::Abs(body_rate) <= kEnterRateRadS) {
        aligned_ = true;
      }
    }
    return aligned_;
  }

 private:
  uint32_t sample_ms_ = 0;
  float sample_error_ = 0.0f;
  float relative_rate_ = 0.0f;
  bool sample_started_ = false;
  bool rate_valid_ = false;
  bool aligned_ = false;
};

struct ChassisRobotParams {
  float max_axis_speed_m_s=0.0f;
  float max_body_rate_rad_s=0.0f;
  float linear_accel_m_s2=0.0f;
  float angular_accel_rad_s2=0.0f;
  hello_world::pid::PidParams follow_pid{}; // 输出单位rad/s，独立于四轮速度PID
  float follow_filter_tau_s = 0.0f; // 跟随输出低通时间常数，s；0为关闭。
  float follow_align_enter_rad = control::kPi / 180.0f;
  float follow_align_exit_rad = 2.0f * control::kPi / 180.0f;
  uint32_t ready_stable_ms=100;
  uint32_t recovery_timeout_ms=30000;
};
class Robot {
 public:
  bool init(const ChassisRobotParams &params,Chassis *chassis,
            hello_world::imu::Imu *imu,GimbalChassisComm *comm);
  void update(float dt_s=0.001f);
  control::MotorAction motorAction()const;
  control::Diagnostics diagnostics()const{return diagnostic_;}
 private:
  bool paramsReady()const;
  void resetControl();
  void publish();
  ChassisRobotParams params_{};
  Chassis *chassis_=nullptr;hello_world::imu::Imu *imu_=nullptr;
  GimbalChassisComm *comm_=nullptr;
  hello_world::pid::Pid follow_pid_{};
  FollowRateFilter follow_rate_filter_{};
  FollowAlignment follow_alignment_{};
  control::LifeMachine machine_{};
  control::Diagnostics diagnostic_{};
  ChassisCommand last_command_{};
  ChassisMode last_mode_=ChassisMode::Disabled;
  control::MotorAction action_=control::MotorAction::Disable;
  bool initialized_=false,updated_=false,ever_active_=false;
  uint32_t working_since_=0;
  float last_follow_error_=0.0f;
  bool has_follow_error_=false;
};
}  // namespace robot
#endif
