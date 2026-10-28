/**
 *******************************************************************************
 * @file      :comm_task.hpp
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
#ifndef CHASSIS_TASK_COMM_TASK_HPP_
#define CHASSIS_TASK_COMM_TASK_HPP_

/* Includes ------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif
/* Exported function prototypes ----------------------------------------------*/
void CommTaskInit(void);
void CommTask(void);
#ifdef __cplusplus
}
#include "gimbal_chassis_comm.hpp"
// 第9项Robot每周期发布本地生命状态/模式；传输层不把心跳等同于Working。
bool PublishChassisBoardState(const robot::GimbalChassisComm::ChassisData &state);
#endif

#endif /* CHASSIS_TASK_COMM_TASK_HPP_ */
