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
  /* TODO【新人任务】:
   * 返回本板顶层状态机 robot::Robot 单例。
   *
   * 你要做：
   *   1. 在本文件静态定义一个 robot::Robot 对象（静态存储期，程序启动即构造）。
   *   2. 返回它的地址。
   *
   * 约束 / 提示：
   *   - robot::Robot 类的定义由你在 RobotModules/component/robot.hpp 补全（见该文件 TODO），
   *     本文件只负责实例化并返回地址。
   */
  return nullptr;
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
