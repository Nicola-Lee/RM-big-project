/**
 *******************************************************************************
 * @file      : HW_can.cpp
 * @brief     : CAN 收发基础封装实现
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-09-18      Hello World     1. 考核骨架：CanFilter_Init + CAN_Send_Msg
 *                                                全量提供，接收分发回调掏空
 *******************************************************************************
 * @attention :
 *  接收中断回调 HAL_CAN_RxFifo0/1MsgPendingCallback 已在此预留，仅完成「取 8 字节 +
 *  再次使能接收中断」，具体「按帧 ID 分发」逻辑留给新人：
 *    - M3508 / DM 电机反馈：调用对应电机的 decode(can?_rx_data)
 *    - 双板通讯帧：调用新人自写的协议解码
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "HW_can.hpp"

#include "can.h"

#include "ins_motor.hpp"
#include "ins_chassis_gimbal_comm.hpp"
#include "gimbal_chassis_comm.hpp"
#include "comm_task.hpp"
#include "runtime_monitor.hpp"
#include "remote_control_forward.hpp"
#include "wheel_pid_debug.hpp"
/* Private variables ---------------------------------------------------------*/
static CAN_RxHeaderTypeDef rx_header1;
static CAN_RxHeaderTypeDef rx_header2;
static uint8_t can1_rx_data[8];
static uint8_t can2_rx_data[8];


/* Exported function definitions ---------------------------------------------*/

void CanFilter_Init(CAN_HandleTypeDef *hcan)
{
  CAN_FilterTypeDef can_filter = {0};

  can_filter.SlaveStartFilterBank = 14;
  /* 全收：掩码全 0，任何 ID 均通过 */
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh = 0x0000;
  can_filter.FilterIdLow = 0x0000;
  can_filter.FilterMaskIdHigh = 0x0000;
  can_filter.FilterMaskIdLow = 0x0000;
  can_filter.FilterActivation = ENABLE;

  if (hcan == &hcan1) {
    can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    can_filter.FilterBank = 0;
  } else if (hcan == &hcan2) {
    can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    can_filter.FilterBank = 14;
  } else {
    return;
  }

  if (HAL_CAN_ConfigFilter(hcan, &can_filter) != HAL_OK) {
    Error_Handler();
  }
}

void CAN_Send_Msg(CAN_HandleTypeDef *hcan, uint8_t *msg, uint32_t id, uint8_t len) {
  // 兼容旧接口；普通电机运动帧不能绕过Robot许可。安全零转矩/失能由CommTask专用路径发送。
  const bool control=(id>=1&&id<=0x0F)||id==0x200||id==0x1FF||
      (id==robot::board_protocol::kVelocityId&&msg!=nullptr&&len==8&&(msg[7]&4)!=0);
  if(hcan==&hcan1)robot::runtime::TrySend(robot::runtime::CanBus::One,id,msg,control,len);
  else if(hcan==&hcan2)robot::runtime::TrySend(robot::runtime::CanBus::Two,id,msg,control,len);
  else robot::runtime::SetFatal(robot::runtime::Fatal::InvalidCanArgument);
}

/**
 * @brief   CAN1 FIFO0 接收中断回调（新人填写分发逻辑）
 * @param   hcan: CAN 句柄
 * @retval  None
 * @note    此处只取了 8 字节并重新使能接收中断，具体分发由新人实现
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan != &hcan1) {
    return;
  }
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header1,
                            can1_rx_data) == HAL_OK) {
    /* TODO【新人任务】: 在此按 rx_header1.StdId 路由 8 字节 can1_rx_data。
      * 判定方法：把 StdId 与各电机的接收 ID（GetRxId）及双板通讯帧 ID 逐一比对：
      *   - 命中某电机的接收 ID：调用该电机 decode(can1_rx_data) 解包反馈；
      *   - 命中双板通讯帧 ID：调用自己的 GimbalChassisComm 解码；
      *   - 都未命中：丢弃（返回前记得重新使能接收中断，见下方调用）。
      */
    if (rx_header1.IDE == CAN_ID_STD &&
      rx_header1.RTR == CAN_RTR_DATA &&
      rx_header1.DLC == 8) {
      if(robot::remote_forward::IsId(rx_header1.StdId))
        ReceiveForwardedRemote(rx_header1.StdId,can1_rx_data);
      if constexpr(robot::wheel_debug::kWheelPidDebugEnabled){
        robot::wheel_debug::Receive(rx_header1.StdId,can1_rx_data);
      }

      auto *yaw = GetMotorYaw();

      if (yaw != nullptr && 
          rx_header1.StdId == 
              hello_world::motor::DaMiao::GetRxId(yaw->id())) {

        if (yaw->decode(can1_rx_data)) RecordGimbalMotorFeedback(0);
        
      } else if (rx_header1.StdId == robot::board_protocol::kChassisStatusId) {
        auto *comm = GetGimbalChassisComm();

        if (comm != nullptr) {
          comm->decode(rx_header1.StdId, can1_rx_data, rx_header1.DLC);
        }
      }
    }
  }
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * @brief   CAN2 FIFO1 接收中断回调（新人填写分发逻辑）
 * @param   hcan: CAN 句柄
 * @retval  None
 * @note    同上，具体分发由新人实现
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan != &hcan2) {
    return;
  }
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header2,
                            can2_rx_data) == HAL_OK) {
    /* TODO【新人任务】: 同 FIFO0，按 rx_header2.StdId 路由 8 字节 can2_rx_data。 */
    if (rx_header2.IDE == CAN_ID_STD &&
        rx_header2.RTR == CAN_RTR_DATA &&
        rx_header2.DLC == 8) {

      auto *pitch = GetMotorPitch();
      if (pitch != nullptr &&
          rx_header2.StdId ==
              hello_world::motor::DaMiao::GetRxId(pitch->id())) {

        if (pitch->decode(can2_rx_data)) RecordGimbalMotorFeedback(1);
      }
    }
  }
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
}

namespace {
bool RuntimeBus(CAN_HandleTypeDef *hcan,robot::runtime::CanBus &bus){
  if(hcan==&hcan1){bus=robot::runtime::CanBus::One;return true;}
  if(hcan==&hcan2){bus=robot::runtime::CanBus::Two;return true;}
  return false;
}
}
extern "C" void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *h){
  robot::runtime::CanBus b;if(RuntimeBus(h,b))robot::runtime::OnCanComplete(b,0);
}
extern "C" void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *h){
  robot::runtime::CanBus b;if(RuntimeBus(h,b))robot::runtime::OnCanComplete(b,1);
}
extern "C" void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *h){
  robot::runtime::CanBus b;if(RuntimeBus(h,b))robot::runtime::OnCanComplete(b,2);
}
extern "C" void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *h){
  robot::runtime::CanBus b;if(RuntimeBus(h,b))robot::runtime::OnCanAbort(b,0);
}
extern "C" void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *h){
  robot::runtime::CanBus b;if(RuntimeBus(h,b))robot::runtime::OnCanAbort(b,1);
}
extern "C" void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *h){
  robot::runtime::CanBus b;if(RuntimeBus(h,b))robot::runtime::OnCanAbort(b,2);
}
extern "C" void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *h){
  robot::runtime::CanBus b;if(!RuntimeBus(h,b))return;
  const uint32_t saved=__get_PRIMASK();__disable_irq();
  robot::runtime::OnCanError(b,HAL_CAN_GetError(h));
  // 已将错误复制到诊断，再清HAL累计位，避免旧错误污染后续事件分类。
  HAL_CAN_ResetError(h);__set_PRIMASK(saved);
}
