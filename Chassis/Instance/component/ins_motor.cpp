/**
 *******************************************************************************
 * @file      :ins_motor.cpp
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
#include "ins_motor.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
namespace {
  hello_world::motor::OptionalParams WheelOptions() {
  hello_world::motor::OptionalParams opt;
  opt.input_type = hello_world::motor::InputType::kRaw;
  opt.dir = hello_world::motor::kDirRev;
  opt.ex_redu_rat = 1.0f;
  opt.angle_offset = 0.0f;
  return opt;
  }
}
hello_world::motor::Motor *GetMotorWheel(int idx) {
  /* TODO【新人任务】:
   * 返回第 idx 号轮电机（M3508）单例。
   *
   * 你要做：
   *   1. 静态定义 4 个轮电机对象并存入数组（可用 hello_world::motor::CreateMotor
   *      (hello_world::motor::MotorType::kM3508, id, opt) 工厂，或直接构造 M3508）。
   *   2. 校验 idx ∈ [0,4)，返回对应元素的地址。
   *   3. 每个电机的 CAN id、方向、减速比按实机接线配置（见考核说明文档）。
   *
   * 约束 / 提示：
   *   - 电机设备类已提供：inc/motor_M3508.hpp（M3508）、inc/motor.hpp（CreateMotor 工厂、
   *     OptionalParams 配置项）。
   */
  if (idx < 0 || idx >= 4) return nullptr;
  // 初始时云台朝向为x轴正向，左侧为y轴正向，按照象限确定电机ID
  static hello_world::motor::M3508 motors[] = {
      hello_world::motor::M3508(1, WheelOptions()),
      hello_world::motor::M3508(2, WheelOptions()),
      hello_world::motor::M3508(3, WheelOptions()),
      hello_world::motor::M3508(4, WheelOptions())};
  return &motors[idx];
}

/* Private function definitions ----------------------------------------------*/
