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
#ifndef GIMBAL_ROBOT_MODULES_ROBOT_HPP_
#define GIMBAL_ROBOT_MODULES_ROBOT_HPP_
#include "gimbal.hpp"
#include "gimbal_chassis_comm.hpp"
#include "robot_control.hpp"
namespace hello_world::imu {class Imu;}
namespace hello_world::motor {class DaMiao;}
namespace robot {
struct GimbalRobotParams {
  bool switch_mapping_confirmed=false;
  float deadband=0.03f;
  float forward_sign=1.0f,left_sign=-1.0f,yaw_sign=-1.0f,pitch_sign=-1.0f,wheel_sign=1.0f;
  float max_forward_m_s=0.0f,max_left_m_s=0.0f;
  float max_manual_body_rate_rad_s=0.0f,max_spin_rate_rad_s=0.0f;
  float max_yaw_input_rate_rad_s=0.0f,max_pitch_input_rate_rad_s=0.0f;
  bool require_neutral_on_recovery=true;
  float neutral_threshold=0.05f;
  uint32_t neutral_stable_ms=100,ready_stable_ms=100,recovery_timeout_ms=30000;
};
class Robot {
 public:
  bool init(const GimbalRobotParams &params,Gimbal *gimbal,hello_world::imu::Imu *imu,
            GimbalChassisComm *comm,hello_world::motor::DaMiao *yaw,
            hello_world::motor::DaMiao *pitch);
  void update(float dt_s=0.001f);
  control::MotorAction motorAction()const;
  control::Diagnostics diagnostics()const{return diagnostic_;}
 private:
  bool paramsReady()const;
  void resetControl();
  void publish(bool remote_online,bool operator_armed,ChassisMode mode);
  GimbalRobotParams params_{};
  Gimbal *gimbal_=nullptr;hello_world::imu::Imu *imu_=nullptr;
  GimbalChassisComm *comm_=nullptr;
  hello_world::motor::DaMiao *yaw_=nullptr,*pitch_=nullptr;
  control::LifeMachine machine_{};
  control::NeutralLatch neutral_{};
  control::Diagnostics diagnostic_{};
  control::MotorAction action_=control::MotorAction::Disable;
  float heading_target_=0.0f,pitch_target_=0.0f;
  bool initialized_=false,updated_=false,target_initialized_=false;
  uint32_t last_enable_request_ms_=0;
  bool enable_requested_=false;
};
}  // namespace robot
#endif
