/*
 * @Author: Nicola Lee nlee15428@gmail.com
 * @Date: 2026-09-19 15:52:24
 * @LastEditors: Nicola Lee nlee15428@gmail.com
 * @LastEditTime: 2026-10-25 20:56:49
 * @FilePath: \omni_skeleton\Chassis\Instance\component\ins_pid.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
 *******************************************************************************
 * @file      :ins_pid.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CHASSIS_INSTANCE_INS_PID_HPP_
#define CHASSIS_INSTANCE_INS_PID_HPP_

/* Includes ------------------------------------------------------------------*/
#include "pid.hpp"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
namespace hello_world {
namespace pid {
class Pid;
}  // namespace pid
}  // namespace hello_world
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
/* 获取第 idx 号轮电机速度环 PID 单例（idx ∈ [0,4)）。 */
hello_world::pid::Pid *GetPidMotorWheel(int idx);

void ResetChassisPids();
#endif /* CHASSIS_INSTANCE_INS_PID_HPP_ */
