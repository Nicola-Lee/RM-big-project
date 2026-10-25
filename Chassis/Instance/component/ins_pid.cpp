/*
 * @Author: Nicola Lee nlee15428@gmail.com
 * @Date: 2026-09-19 15:52:24
 * @LastEditors: Nicola Lee nlee15428@gmail.com
 * @LastEditTime: 2026-10-25 21:32:45
 * @FilePath: \omni_skeleton\Chassis\Instance\component\ins_pid.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
 *******************************************************************************
 * @file      :ins_pid.cpp
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
#include "ins_pid.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
hello_world::pid::Pid *GetPidMotorWheel(int idx) {
  /* TODO【新人任务】:
   * 返回对应的 PID 单例。
   *
   * 你要做：
   *   1. 先自己实现一个 PID 类（位置式 / 增量式，含 Kp/Ki/Kd、积分限幅、输出限幅），
   *      建议单独新建文件，类名放到 hello_world::pid 命名空间并命名为 Pid（或改这里
   *      的返回类型以匹配你的类名）。
   *   2. 静态定义各控制回路的 PID 对象并整定参数（考核会检查稳 / 快 / 无超调）。
   */
  if (idx < 0 || idx >= 4) {
    return nullptr;
  }

  static constexpr float kWheelIntegralLimit = 0.0f;
  static constexpr float kWheelOutputLimit = 0.0f;

  static const hello_world::pid::PidParams params[4] = {
      // 左前轮，ID1
      {.kp = 0.0f, .ki = 0.0f, .kd = 0.0f,
      .integral_limit = kWheelIntegralLimit,
      .output_limit = kWheelOutputLimit},

      // 左后轮，ID2
      {.kp = 0.0f, .ki = 0.0f, .kd = 0.0f,
      .integral_limit = kWheelIntegralLimit,
      .output_limit = kWheelOutputLimit},

      // 右后轮，ID3
      {.kp = 0.0f, .ki = 0.0f, .kd = 0.0f,
      .integral_limit = kWheelIntegralLimit,
      .output_limit = kWheelOutputLimit},

      // 右前轮，ID4
      {.kp = 0.0f, .ki = 0.0f, .kd = 0.0f,
      .integral_limit = kWheelIntegralLimit,
      .output_limit = kWheelOutputLimit}
};

  static hello_world::pid::Pid pids[4] = {
      hello_world::pid::Pid(params[0]),
      hello_world::pid::Pid(params[1]),
      hello_world::pid::Pid(params[2]),
      hello_world::pid::Pid(params[3])
  };
  return &pids[idx];
}

/* Private function definitions ----------------------------------------------*/
void ResetChassisPids()
{
  for (int idx = 0; idx < 4; ++idx) {
    auto *pid = GetPidMotorWheel(idx);

    if (pid != nullptr) {
      pid->reset();
    }
  }
}