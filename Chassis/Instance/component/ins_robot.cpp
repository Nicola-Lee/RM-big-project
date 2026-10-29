/*
 * @Author: Nicola Lee nlee15428@gmail.com
 * @Date: 2026-10-07 13:04:26
 * @LastEditors: Nicola Lee nlee15428@gmail.com
 * @LastEditTime: 2026-10-29 20:06:55
 * @FilePath: \omni_skeleton\Chassis\Instance\component\ins_robot.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
 *******************************************************************************
 * @file      :ins_robot.cpp
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
/* Includes ------------------------------------------------------------------*/
#include "ins_robot.hpp"
#include "robot.hpp"
#include "robot_limits.hpp"
#include "ins_imu.hpp"
#include "ins_chassis_gimbal_comm.hpp"
#include "chassis.hpp"
#include "ins_motor.hpp"
#include "ins_pid.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
robot::Robot *GetRobot() {
  static robot::Robot controller;
  static const bool initialized=[](){
    robot::ChassisRobotParams params;
    // 待实机配置：分量速度m/s、角速度rad/s、加速度m/s^2和rad/s^2。
    params.max_axis_speed_m_s=0.3f;
    params.max_body_rate_rad_s=robot::limits::kMaxBodyRateRadS;
    params.linear_accel_m_s2=0.3f;
    params.angular_accel_rad_s2=0.5f;
    // 底盘跟随PID参数调节
    params.follow_pid={.kp=0.0f, .ki=0.0f, .kd=0.0f,
      .integral_limit=0.1f,.output_limit=0.5f};
    return controller.init(params,GetChassis(),GetImu(),GetGimbalChassisComm());
  }();
  return initialized?&controller:nullptr;
}

robot::Chassis *GetChassis() {
  static robot::Chassis chassis;
  static const bool initialized = []() {
    robot::ChassisParams params;
    // 实车 X 型麦轮尺寸；轮速上限需调试后填写，0 表示尚未配置。
    params.wheel_radius_m = 0.07786f;
    params.half_length_m = 0.15338f;
    params.half_width_m = 0.15338f;
    params.max_wheel_speed_rad_s = 10.0f;
    // 前馈先保持关闭
    params.velocity_feedforward_gain = {0.0f, 0.0f, 0.0f, 0.0f};
    robot::Chassis::Motors motors{};
    robot::Chassis::Pids pids{};
    for (int i = 0; i < 4; ++i) {
      motors[i] = GetMotorWheel(i);
      pids[i] = GetPidMotorWheel(i);
    }
    return chassis.init(params, motors, pids);
  }();
  return initialized ? &chassis : nullptr;
}

/* Private function definitions ----------------------------------------------*/
