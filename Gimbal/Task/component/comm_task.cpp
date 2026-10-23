/**
 *******************************************************************************
 * @file      :comm_task.cpp
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
#include "comm_task.hpp"
#include "HW_can.hpp"
#include "can.h"
#include "iwdg.h"
#include "ins_motor.hpp"
#include "ins_chassis_gimbal_comm.hpp"

/* Exported function definitions ---------------------------------------------*/
static volatile bool comm_initialized = false;

void CommTaskInit(void) {
  /* TODO【新人任务】:
   * 通讯层一次性初始化：
   *   1. 初始化 CAN 滤波器：inc/HW_can.hpp 的 CanFilter_Init(&hcan1) / CanFilter_Init(&hcan2)。
   *   2. 启动遥控器串口空闲 + DMA 接收（18 字节 DBUS，见 inc/DT7.hpp 注释）：
   *      调用 HAL_UARTEx_ReceiveToIdle_DMA 并记录接收缓冲区。
   *   3. 启动 / 刷新独立看门狗 IWDG（inc/iwdg.h）。
   */
  GetMotorYaw();
  GetMotorPitch();
  GetGimbalChassisComm();

  CanFilter_Init(&hcan1);
  CanFilter_Init(&hcan2);

  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_CAN_Start(&hcan2) != HAL_OK) {
    Error_Handler();
  }

  // CAN1接收yaw反馈
  if (HAL_CAN_ActivateNotification(
          &hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
    Error_Handler();
  }

  // CAN2接收pitch反馈
  if (HAL_CAN_ActivateNotification(
          &hcan2, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK) {
    Error_Handler();
  }

  comm_initialized = true;
  //遥控接收也还没写
}

void CommTask(void) {
  /* TODO【新人任务】:
   * 每个通讯周期（1ms）调用一次，把各模块 encode 出的 8 字节 CAN 帧发出去：
   *   1. 遍历电机：调用 encode(data) 得到 8 字节 + tx_id()，再 CAN_Send_Msg 发出。
   *   2. 双板通讯对象 encode 后同样发送。
   *   3. 刷新 IWDG。
   */
  if (!comm_initialized) {
    return;
  }

  auto *yaw =
      static_cast<hello_world::motor::DM_J4310 *>(GetMotorYaw());

  auto *pitch =
      static_cast<hello_world::motor::DM_J4310 *>(GetMotorPitch());

  if (yaw == nullptr || pitch == nullptr) {
    return;
  }
// 先检查空闲再编码
if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0) {
  yaw->setInput(0.0f);//目前先出零
  yaw->disable();

  uint8_t data[8] = {0};
  if (yaw->encode(data)) {
    CAN_Send_Msg(&hcan1, data, yaw->tx_id(), 8);
  }
}

if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) > 0) {
  pitch->setInput(0.0f);
  pitch->disable();

  uint8_t data[8] = {0};
  if (pitch->encode(data)) {
    CAN_Send_Msg(&hcan2, data, pitch->tx_id(), 8);
  }
}

  HAL_IWDG_Refresh(&hiwdg);
  //还差双板协议编码
}

/* Private function definitions ----------------------------------------------*/
/* TODO【新人任务】:
 * 实现 UART 空闲中断回调 HAL_UARTEx_RxEventCallback：
 *   - 收到满 18 字节后调用 DT7.decode() 解包遥控器数据，并重新启动下一轮接收；
 *   - 该回调同样是 C 链接（extern "C"）。
 * 注意：CAN 的接收分发在 src/HW_can.cpp 的两个回调里（电机 decode / 双板协议解码），
 *      不要在这里重复写 CAN 接收逻辑。
 */
