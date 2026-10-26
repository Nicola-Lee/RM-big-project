/*
 * @Author: Nicola Lee nlee15428@gmail.com
 * @Date: 2026-09-19 15:52:24
 * @LastEditors: Nicola Lee nlee15428@gmail.com
 * @LastEditTime: 2026-10-26 19:32:05
 * @FilePath: \omni_skeleton\Gimbal\Instance\component\ins_imu.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
 *******************************************************************************
 * @file      :ins_imu.cpp
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
#include "ins_imu.hpp"
#include "imu.hpp"

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
hello_world::imu::Imu *GetImu() {
  /* TODO【新人任务】:
   * 返回本板 IMU 单例。
   *
   * 你要做：
   *   1. 静态定义一个 Imu 对象（类由你在 inc/imu.hpp 补全：BMI088 原始读取 +
   *      Mahony 姿态 + 开机零漂）。
   *   2. 返回其地址。
   *
   * 约束 / 提示：
   *   - Imu 类的定义见 inc/imu.hpp 的 TODO；本文件只负责实例化并返回地址。
   *   - 姿态角 / 零漂未就绪前，上层状态机应保持在 Dead / Resurrection（见 RobotModules TODO）。
   */
  return nullptr;
}

/* Private function definitions ----------------------------------------------*/
