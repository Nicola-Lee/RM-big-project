/**
 *******************************************************************************
 * @file      :main_task.hpp
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
#ifndef GIMBAL_TASK_MAIN_TASK_HPP_
#define GIMBAL_TASK_MAIN_TASK_HPP_

/* Includes ------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif
/* Exported function prototypes ----------------------------------------------*/
void MainTaskInit(void);
void MainTask(void);

// 前台IMU调度状态；kImuPollSampling包括零漂采集和姿态收敛。
typedef enum {
  kImuPollWaitingForInit = 0,
  kImuPollWaitingForConfig,
  kImuPollWaitingForTimer,
  kImuPollTimingError,
  kImuPollStarting,
  kImuPollSampling,
  kImuPollReady,
  kImuPollFault
} ImuPollStatus;

// 只从main.c主循环调用；控制中断通过GetImu()->getSnapshot()读取有效数据。
void ImuPoll(void);
// 主循环末尾调用；只有这里可以刷新硬件看门狗。
void WatchdogPoll(void);
ImuPollStatus GetImuPollStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* GIMBAL_TASK_MAIN_TASK_HPP_ */
