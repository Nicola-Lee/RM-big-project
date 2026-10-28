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
struct ChassisRobotParams {
  float max_axis_speed_m_s=0.0f;
  float max_body_rate_rad_s=0.0f;
  float linear_accel_m_s2=0.0f;
  float angular_accel_rad_s2=0.0f;
  hello_world::pid::PidParams follow_pid{}; // 输出单位rad/s，独立于四轮速度PID
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
