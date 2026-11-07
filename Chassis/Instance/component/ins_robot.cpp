/*
 * @Author: Nicola Lee nlee15428@gmail.com
 * @Date: 2026-10-07 13:04:26
 * @LastEditors: Nicola Lee nlee15428@gmail.com
 * @LastEditTime: 2026-11-06 22:31:04
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
    // 分量速度m/s、角速度rad/s、加速度m/s^2和rad/s^2限幅
    params.max_axis_speed_m_s=0.3f;
    params.max_body_rate_rad_s=robot::limits::kMaxBodyRateRadS;
    params.linear_accel_m_s2=0.3f;
    params.angular_accel_rad_s2=2.0f;
    // 跟随输出上限保留1.2rad/s，小陀螺使用共享总上限。
    params.follow_pid = {
      .kp = 4.0f,
      .ki = 0.0f,
      .kd = 0.0f,
      .integral_limit = 0.1f,
      .output_limit = 1.2f,
    };

    // 跟随角速度低通，单位为秒。设为0关闭。
    params.follow_filter_tau_s = 0.01f;

    // 对齐进入角为1度，退出角为2度。参数单位为rad。
    params.follow_align_enter_rad = robot::control::kPi / 180.0f;
    params.follow_align_exit_rad = 2.0f * robot::control::kPi / 180.0f;

    return controller.init(params,GetChassis(),GetImu(),GetGimbalChassisComm());
  }();
  return initialized?&controller:nullptr;
}

robot::Chassis *GetChassis() {
  static robot::Chassis chassis;
  static const bool initialized = []() {
    robot::ChassisParams params;

    params.wheel_radius_m = 0.07786f;
    params.half_length_m = 0.15338f;
    params.half_width_m = 0.15338f;
    // 实车平移轴互换，在云台到车体坐标变换之后应用。
    params.swap_translation_axes = true;
    // 互换后实测前后、左右均反向，统一反转两个平移分量。
    params.reverse_translation_axes = true;
    params.max_wheel_speed_rad_s = 10.0f;
    // 前馈
    params.velocity_feedforward_gain = {95.0f, 95.0f, 95.0f, 95.0f};//空载时参数为45
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
