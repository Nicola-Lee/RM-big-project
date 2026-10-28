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
#include "ins_imu.hpp"
#include "ins_chassis_gimbal_comm.hpp"
#include "gimbal.hpp"
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
    robot::GimbalRobotParams params;
    // 已由用户确认：左拨杆上/中/下=分离/跟随/小陀螺；右下急停，中/上允许恢复。
    params.switch_mapping_confirmed=true;
    params.deadband=0.03f;
    // 正方向：前进/左移/左转/下俯/底盘逆时针；实际遥控方向仍需上板核对。
    params.forward_sign=1.0f;params.left_sign=-1.0f;
    params.yaw_sign=-1.0f;params.pitch_sign=-1.0f;params.wheel_sign=1.0f;
    // 待实机配置；零值表示尚未配置，状态机禁止进入正常控制。
    params.max_forward_m_s=0.0f;params.max_left_m_s=0.0f;
    params.max_manual_body_rate_rad_s=0.0f;params.max_spin_rate_rad_s=0.0f;
    params.max_yaw_input_rate_rad_s=0.0f;params.max_pitch_input_rate_rad_s=0.0f;
    // 恢复时先回中100ms，再进行双方就绪握手，防止恢复即追赶旧摇杆请求。
    params.require_neutral_on_recovery=true;
    auto *yaw=static_cast<hello_world::motor::DM_J4310 *>(GetMotorYaw());
    auto *pitch=static_cast<hello_world::motor::DM_J4310 *>(GetMotorPitch());
    return controller.init(params,GetGimbal(),GetImu(),GetGimbalChassisComm(),yaw,pitch);
  }();
  return initialized?&controller:nullptr;
}

robot::Gimbal *GetGimbal() {
  static robot::Gimbal gimbal;
  static const bool initialized = []() {
    robot::GimbalParams params;
    // 必须按当前 angle_offset/dir 标定后的 angle() 填写；pitch 下俯为正。
    // PDF 第8页要求软件限位，第11页仅给零点偏移0.02480，未给上下限。
    // 未标定时对象可初始化，但 update 禁止输出。
    params.pitch_limits_calibrated = false;
    params.pitch_min_rad = 0.0f;
    params.pitch_max_rad = 0.0f;
    // 目标变化率（rad/s）和模块转矩上限（N*m），0 表示待配置。
    params.yaw_target_rate_rad_s = 0.0f;
    params.pitch_target_rate_rad_s = 0.0f;
    params.yaw_torque_limit_nm = 0.0f;
    params.pitch_torque_limit_nm = 0.0f;
    // 前馈默认关闭；位置 PID 参数继续在 ins_pid.cpp 中分别整定。
    params.yaw_velocity_ff = 0.0f;
    params.pitch_velocity_ff = 0.0f;
    params.pitch_gravity_cos_nm = 0.0f;
    params.pitch_gravity_sin_nm = 0.0f;
    // 两个 GetMotor 函数实际都返回 DM_J4310，这里转换为达妙基类使用状态接口。
    auto *yaw = static_cast<hello_world::motor::DM_J4310 *>(GetMotorYaw());
    auto *pitch = static_cast<hello_world::motor::DM_J4310 *>(GetMotorPitch());
    return gimbal.init(params, yaw, pitch, GetPidMotorYaw(), GetPidMotorPitch());
  }();
  return initialized ? &gimbal : nullptr;
}

/* Private function definitions ----------------------------------------------*/
