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
#include "ins_motor.hpp"
#include "ins_chassis_gimbal_comm.hpp"
#include "gimbal_chassis_comm.hpp"
#include "ins_imu.hpp"
#include "imu.hpp"
#include "tick.hpp"
#include "ins_robot.hpp"
#include "robot.hpp"
#include "runtime_monitor.hpp"
#include <cmath>
/* Exported function definitions ---------------------------------------------*/
static volatile bool comm_initialized = false;


namespace {
// 所有发送统一记录入邮箱结果、忙状态和完成回调；此处不等待总线。
bool TryQueueBoardFrame(uint32_t id, const uint8_t data[8]) {
  const bool control=id==robot::board_protocol::kVelocityId&&(data[7]&4)!=0;
  return robot::runtime::TrySend(robot::runtime::CanBus::One,id,data,control);
}
}

namespace {
uint32_t wheel_last_rx_ms[4]{};
bool wheel_has_feedback[4]{};
}
void RecordWheelFeedback(int index) {
  if(index<0||index>=4)return;
  wheel_last_rx_ms[index]=hello_world::tick::GetTickMs();wheel_has_feedback[index]=true;
}
void GetWheelFeedbackSnapshot(WheelFeedbackSnapshot &out) {
  const uint32_t saved=__get_PRIMASK();__disable_irq();
  const uint32_t now=hello_world::tick::GetTickMs();out={};
  for(int i=0;i<4;++i){
    auto *motor=GetMotorWheel(i);out.velocity[i]=motor->vel();
    out.valid[i]=robot::board_protocol::Fresh(wheel_has_feedback[i],now,wheel_last_rx_ms[i])&&
                 std::isfinite(out.velocity[i]);
  }
  __set_PRIMASK(saved);
}

bool PublishChassisBoardState(const robot::GimbalChassisComm::ChassisData &state) {
  return GetGimbalChassisComm()->setLocalChassis(state);
}
namespace {
void TransmitBoardStatus() {
  auto *comm=GetGimbalChassisComm();
  auto state=comm->localChassis();
  hello_world::imu::ImuSnapshot imu{};
  state.imu_ready=GetImu()->getSnapshot(imu);
  // local_ready还需由Robot确认电机、配置等；此处只能进一步收紧。
  state.local_ready=state.local_ready&&state.imu_ready;
  if(state.life==robot::LifeState::Working&&!state.local_ready)state.emergency_stop=true;
  if(state.emergency_stop)state.target_valid=false;
  if(!comm->setLocalChassis(state)){robot::runtime::SetFatal(robot::runtime::Fatal::Encoder);return;}
  uint8_t data[8]{};
  if(robot::runtime::CanMailboxAvailable(robot::runtime::CanBus::One)&&comm->encode(data)&&
     TryQueueBoardFrame(comm->tx_id(),data))comm->onTxQueued(comm->tx_id());
}
}

void CommTaskInit(void) {
  /* TODO【新人任务】:
   * 通讯层一次性初始化：
   *   1. 初始化 CAN 滤波器：inc/HW_can.hpp 的 CanFilter_Init(&hcan1) / CanFilter_Init(&hcan2)。
   *   2. 启动遥控器串口空闲 + DMA 接收（18 字节 DBUS，见 inc/DT7.hpp 注释）：
   *      调用 HAL_UARTEx_ReceiveToIdle_DMA 并记录接收缓冲区。
   *   3. IWDG由main初始化、WatchdogPoll集中刷新；通信任务只记录进度。
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

  const uint32_t tx_error_notifications=CAN_IT_TX_MAILBOX_EMPTY|CAN_IT_BUSOFF|
      CAN_IT_ERROR_WARNING|CAN_IT_ERROR_PASSIVE|CAN_IT_ERROR;
  if(HAL_CAN_ActivateNotification(&hcan1,tx_error_notifications|CAN_IT_RX_FIFO0_OVERRUN)!=HAL_OK||
     HAL_CAN_ActivateNotification(&hcan2,tx_error_notifications|CAN_IT_RX_FIFO1_OVERRUN)!=HAL_OK){
    Error_Handler();
  }
  comm_initialized = true;
  // 本板遥控操作来自云台速度帧，不启动第二套DBUS接收。
}

void CommTask(void) {
  using namespace robot::runtime;
  if(!comm_initialized){SetFatal(Fatal::MissingObject);return;}
  ServiceCan();
  TransmitBoardStatus();
  auto *controller=GetRobot();
  if(controller==nullptr){SetControlPermit(false);SetFatal(Fatal::MissingObject);return;}
  const bool allow_control=controller->motorAction()==robot::control::MotorAction::Control;
  SetControlPermit(allow_control);
  // 即使本次CAN2邮箱忙，也先清理不被许可的控制量，然后继续完成本轮处理。
  uint8_t data[8]{};
  for(int i=0;i<4;++i){
    auto *motor=GetMotorWheel(i);
    if(motor==nullptr){SetFatal(Fatal::MissingObject);return;}
    if(!allow_control)motor->setInput(0.0f);
    if(!motor->encode(data)){SetFatal(Fatal::Encoder);return;}
  }
  if(CanMailboxAvailable(CanBus::Two))
    TrySend(CanBus::Two,hello_world::motor::M3508::GetTxId(1),data,allow_control);
  // 邮箱忙/提交失败是有界处理结果，不跳过任务完成记录，更不在这里直接喂狗。
  Mark(Task::Communication);
}

/* Private function definitions ----------------------------------------------*/
/* TODO【新人任务】:
 * 实现 UART 空闲中断回调 HAL_UARTEx_RxEventCallback：
 *   - 收到满 18 字节后调用 DT7.decode() 解包遥控器数据，并重新启动下一轮接收；
 *   - 该回调同样是 C 链接（extern "C"）。
 * 注意：CAN 的接收分发在 src/HW_can.cpp 的两个回调里（电机 decode / 双板协议解码），
 *      不要在这里重复写 CAN 接收逻辑。
 */
