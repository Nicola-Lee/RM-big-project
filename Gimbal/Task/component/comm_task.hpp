/*
 * @Author: Nicola Lee nlee15428@gmail.com
 * @Date: 2026-09-19 15:12:00
 * @LastEditors: Nicola Lee nlee15428@gmail.com
 * @LastEditTime: 2026-10-25 20:29:18
 * @FilePath: \omni_skeleton\Gimbal\Task\component\comm_task.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
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
#ifndef GIMBAL_TASK_COMM_TASK_HPP_
#define GIMBAL_TASK_COMM_TASK_HPP_

/* Includes ------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif
/* Exported function prototypes ----------------------------------------------*/
void CommTaskInit(void);
void CommTask(void);
void RemoteControlRxPoll(void);
#ifdef __cplusplus
} 

#include <cstdint>
#include "gimbal_chassis_comm.hpp"

//一次读取到的遥控器数据
struct RemoteControlSnapshot {
  bool online = false;

  float rc_lv = 0.0f;
  float rc_lh = 0.0f;
  float rc_rv = 0.0f;
  float rc_rh = 0.0f;
  float rc_wheel = 0.0f;

  //0表示当前没有有效控制数据，不是正常拨杆档位
  uint8_t rc_l_switch = 0;
  uint8_t rc_r_switch = 0;

  uint32_t last_valid_rx_ms = 0;
  uint32_t valid_frame_count = 0;
};

//返回true表示在线 false表示当前数据不能用于控制
bool GetRemoteControlSnapshot(RemoteControlSnapshot &snapshot);

// 第9项Robot本地计算云台目标角；这里仅发布底盘速度和云台状态。
bool PublishGimbalBoardState(const robot::GimbalChassisComm::GimbalData &state);
bool PublishChassisVelocity(const robot::ChassisCommand &command,
                            robot::ChassisMode mode, bool control_enabled);
struct GimbalMotorFeedbackSnapshot {
  robot::GimbalMotion angle{};
  bool yaw_valid=false;
  bool pitch_valid=false;
  uint32_t yaw_last_rx_ms=0;
  uint32_t pitch_last_rx_ms=0;
};
// 由现有CAN回调在电机decode成功后记录，0=yaw，1=pitch。
void RecordGimbalMotorFeedback(int axis);
void GetGimbalMotorFeedbackSnapshot(GimbalMotorFeedbackSnapshot &snapshot);

#endif

#endif /* GIMBAL_TASK_COMM_TASK_HPP_ */
