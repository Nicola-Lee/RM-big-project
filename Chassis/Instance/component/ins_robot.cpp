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

robot::Chassis *GetChassis() {
  static robot::Chassis chassis;
  static const bool initialized = []() {
    robot::ChassisParams params;
    // 实车 X 型麦轮尺寸；轮速上限需调试后填写，0 表示尚未配置。
    params.wheel_radius_m = 0.07786f;
    params.half_length_m = 0.15338f;
    params.half_width_m = 0.15338f;
    params.max_wheel_speed_rad_s = 0.0f;
    // ID1、ID2、ID3、ID4 各自的 kV，单位 raw/(rad/s)。先保持关闭。
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
