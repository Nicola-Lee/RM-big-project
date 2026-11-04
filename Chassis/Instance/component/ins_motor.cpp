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
hello_world::motor::OptionalParams WheelOptions(hello_world::motor::Dir direction) {
  hello_world::motor::OptionalParams opt;
  opt.input_type = hello_world::motor::InputType::kRaw;
  // dir同时应用于反馈和输出，使各轮正轮速都对应带有底盘+X分量的驱动方向。
  opt.dir = direction;
  opt.ex_redu_rat = 1.0f;
  opt.angle_offset = 0.0f;
  return opt;
}
} // namespace
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
  // 轮位以图示底盘坐标为准，不随云台转动：ID1(+X,+Y)、ID2(-X,+Y)、ID3(-X,-Y)、ID4(+X,-Y)。
  // 已按用户实车观察修正：后两轮ID2/3反向，前两轮ID1/4保持原配置。
  static hello_world::motor::M3508 motors[] = {
      hello_world::motor::M3508(1, WheelOptions(hello_world::motor::kDirRev)),
      hello_world::motor::M3508(2, WheelOptions(hello_world::motor::kDirFwd)),
      hello_world::motor::M3508(3, WheelOptions(hello_world::motor::kDirFwd)),
      hello_world::motor::M3508(4, WheelOptions(hello_world::motor::kDirRev))};
  return &motors[idx];
}

/* Private function definitions ----------------------------------------------*/
