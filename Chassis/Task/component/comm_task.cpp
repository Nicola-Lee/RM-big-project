/*
 * @Author: miunoki 13567781025@163.com
 * @Date: 2026-09-19 15:11:39
 * @LastEditors: miunoki 13567781025@163.com
 * @LastEditTime: 2026-09-19 21:14:38
 * @FilePath: \omni_skeleton\Chassis\Task\component\comm_task.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
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
  for (int idx = 0; idx < 4; ++idx) {
    GetMotorWheel(idx);
  }

  // 创建双板通信对象
  GetGimbalChassisComm();

  // 配置CAN滤波器
  CanFilter_Init(&hcan1);
  CanFilter_Init(&hcan2);

  // 启用CAN
  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_CAN_Start(&hcan2) != HAL_OK) {
    Error_Handler();
  }

  // CAN1消息进入FIFO0
  if (HAL_CAN_ActivateNotification(
          &hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
    Error_Handler();
  }

  // CAN2消息进入FIFO1
  if (HAL_CAN_ActivateNotification(
          &hcan2, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK) {
    Error_Handler();
  }

  comm_initialized = true;
  // 遥控器接收还未写
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
  uint8_t tx_data[8] = {0};

  for (int idx = 0; idx < 4; ++idx) {
    auto *motor = GetMotorWheel(idx);

    if (motor == nullptr) {
      return;
    }

    // 先保持零电流
    motor->setInput(0.0f);

    // 写入每台电机的控制信息
    if (!motor->encode(tx_data)) {
      return;
    }
  }

  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0) {
    return;
  }

  //电机控制量合并成一帧，通过底盘CAN2发给各电机
  CAN_Send_Msg(
      &hcan2,
      tx_data,
      hello_world::motor::M3508::GetTxId(1),
      8
  );

  HAL_IWDG_Refresh(&hiwdg);
}

/* Private function definitions ----------------------------------------------*/
/* TODO【新人任务】:
 * 实现 UART 空闲中断回调 HAL_UARTEx_RxEventCallback：
 *   - 收到满 18 字节后调用 DT7.decode() 解包遥控器数据，并重新启动下一轮接收；
 *   - 该回调同样是 C 链接（extern "C"）。
 * 注意：CAN 的接收分发在 src/HW_can.cpp 的两个回调里（电机 decode / 双板协议解码），
 *      不要在这里重复写 CAN 接收逻辑。
 */
