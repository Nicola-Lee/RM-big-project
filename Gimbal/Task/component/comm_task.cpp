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
#include "gimbal_chassis_comm.hpp"
#include "ins_imu.hpp"
#include "imu.hpp"
#include <cstdint>
#include "ins_comm.hpp"
#include "usart.h"
#include "tick.hpp"

/* Exported function definitions ---------------------------------------------*/
static volatile bool comm_initialized = false;
static constexpr uint16_t kRemoteRxBufferSize = 128;//DMA缓存区容量
static uint8_t remote_rx_buffer[kRemoteRxBufferSize] = {0};//数据写入
static uint16_t remote_frame_length = 0;//目前缓存中已有几个字节
static uint16_t remote_rx_last_pos = 0;//上次处理到的DMA缓冲区位置
static uint8_t remote_frame_buffer[hello_world::remote_control::kRcRxDataLen] = {0};//暂存的DBUS数据
static volatile uint32_t remote_valid_frame_count = 0;//累计的有效帧数
static volatile uint32_t remote_last_valid_rx_ms = 0;//最近一次收到控制帧的时间
static volatile bool remote_has_valid_frame = false;//上电后是否受到过有效帧
static volatile bool remote_rx_started = false;//UART DMA接收是否已经启动
static bool remote_frame_aligned = false;//是否已经对齐到DBUS帧头
static constexpr uint32_t kRemoteOfflineTimeoutMs = 100;//遥控器离线判定阈值

static constexpr uint32_t kRemoteRetryIntervalMs = 10;//恢复失败后尝试时间的间隔
static uint32_t remote_last_retry_ms = 0;//上次尝试重新启动接收的时间

static volatile uint32_t remote_uart_error_count = 0;//调试参数：错误次数
static volatile uint32_t remote_last_uart_error = HAL_UART_ERROR_NONE;//最近一次错误码

bool GetRemoteControlSnapshot(RemoteControlSnapshot &snapshot)
{
  // 默认离线，所有控制量为零
  RemoteControlSnapshot result{};

  // 保存原中断状态，再短暂关闭中断
  const uint32_t saved_primask = __get_PRIMASK();
  __disable_irq();

  const uint32_t now = hello_world::tick::GetTickMs();

  result.last_valid_rx_ms = remote_last_valid_rx_ms;
  result.valid_frame_count = remote_valid_frame_count;

  // 必须已初始化、接收已启动、收到过有效帧，而且没有超时
  if (comm_initialized &&
      remote_rx_started &&
      huart3.RxState == HAL_UART_STATE_BUSY_RX &&
      remote_has_valid_frame &&
      static_cast<uint32_t>(now - result.last_valid_rx_ms) <
          kRemoteOfflineTimeoutMs) {

    const auto *remote = GetRemoteControl();

    result.rc_lv = remote->rc_lv();
    result.rc_lh = remote->rc_lh();
    result.rc_rv = remote->rc_rv();
    result.rc_rh = remote->rc_rh();
    result.rc_wheel = remote->rc_wheel();

    result.rc_l_switch =
        static_cast<uint8_t>(remote->rc_l_switch());

    result.rc_r_switch =
        static_cast<uint8_t>(remote->rc_r_switch());

    result.online = true;
  }

  // 恢复进入函数前的中断状态
  __set_PRIMASK(saved_primask);

  snapshot = result;
  return result.online;
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart != &huart3) {
    return;
  }

  // 立即让读取接口返回离线
  remote_rx_started = false;
  remote_has_valid_frame = false;

  remote_last_uart_error = huart->ErrorCode;
  remote_uart_error_count = remote_uart_error_count + 1;

  // 不在中断中阻塞重试，由主循环负责恢复
}


// 只能从主循环调用
void RemoteControlRxPoll(void)
{
  if (!comm_initialized) {
    return;
  }

  // 正常运行时不停止、不重启 DMA
  if (remote_rx_started &&
      huart3.RxState == HAL_UART_STATE_BUSY_RX) {
    return;
  }

  const uint32_t now = hello_world::tick::GetTickMs();

  if (static_cast<uint32_t>(now - remote_last_retry_ms) <
      kRemoteRetryIntervalMs) {
    return;
  }

  remote_last_retry_ms = now;

  // 恢复期间保持离线
  uint32_t saved_primask = __get_PRIMASK();
  __disable_irq();

  remote_rx_started = false;
  remote_has_valid_frame = false;

  __set_PRIMASK(saved_primask);

  // 此函数可能等待 DMA 停止，所以必须在主循环中执行
  if (HAL_UART_AbortReceive(&huart3) != HAL_OK) {
    return;
  }

  // 旧接收停止后，丢弃旧的拼帧状态
  remote_rx_last_pos = 0;
  remote_frame_length = 0;
  remote_frame_aligned = false;

  const uint32_t errors_before = remote_uart_error_count;

  const HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(
      &huart3,
      remote_rx_buffer,
      kRemoteRxBufferSize
  );

  saved_primask = __get_PRIMASK();
  __disable_irq();

  // 防止启动期间发生的新错误被成功返回值覆盖
  remote_rx_started =
      (status == HAL_OK &&
       remote_uart_error_count == errors_before &&
       huart3.RxState == HAL_UART_STATE_BUSY_RX);

  __set_PRIMASK(saved_primask);
}


namespace {
// 状态/速度帧固定8字节。直接检查HAL提交结果，计数和忙状态策略留第10项。
bool TryQueueBoardFrame(uint32_t id, const uint8_t data[8]) {
  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)==0) return false;
  CAN_TxHeaderTypeDef header{};
  header.StdId=id;header.IDE=CAN_ID_STD;header.RTR=CAN_RTR_DATA;header.DLC=8;
  uint32_t mailbox=0;
  return HAL_CAN_AddTxMessage(&hcan1,&header,const_cast<uint8_t *>(data),&mailbox)==HAL_OK;
}
}

namespace {
uint32_t joint_last_rx_ms[2]={0,0};
bool joint_has_feedback[2]={false,false};
}
void RecordGimbalMotorFeedback(int axis) {
  if(axis<0||axis>1)return;
  joint_last_rx_ms[axis]=hello_world::tick::GetTickMs();
  joint_has_feedback[axis]=true;
}
void GetGimbalMotorFeedbackSnapshot(GimbalMotorFeedbackSnapshot &out) {
  const uint32_t saved=__get_PRIMASK();__disable_irq();
  const uint32_t now=hello_world::tick::GetTickMs();
  auto *yaw=static_cast<hello_world::motor::DM_J4310 *>(GetMotorYaw());
  auto *pitch=static_cast<hello_world::motor::DM_J4310 *>(GetMotorPitch());
  out={};out.yaw_last_rx_ms=joint_last_rx_ms[0];out.pitch_last_rx_ms=joint_last_rx_ms[1];
  using Dm=hello_world::motor::DaMiao;
  out.yaw_valid=robot::board_protocol::Fresh(joint_has_feedback[0],now,joint_last_rx_ms[0])&&
      yaw->is_enabled()&&yaw->status_code()==Dm::StatusCode::kMotorEnabled;
  out.pitch_valid=robot::board_protocol::Fresh(joint_has_feedback[1],now,joint_last_rx_ms[1])&&
      pitch->is_enabled()&&pitch->status_code()==Dm::StatusCode::kMotorEnabled;
  if(out.yaw_valid)out.angle.yaw=yaw->angle();
  if(out.pitch_valid)out.angle.pitch=pitch->angle();
  __set_PRIMASK(saved);
  out.yaw_valid=out.yaw_valid&&robot::board_protocol::AngleValid(out.angle.yaw);
  out.pitch_valid=out.pitch_valid&&robot::board_protocol::AngleValid(out.angle.pitch);
  if(!out.yaw_valid)out.angle.yaw=0;
  if(!out.pitch_valid)out.angle.pitch=0;
}
bool PublishGimbalBoardState(const robot::GimbalChassisComm::GimbalData &state) {
  return GetGimbalChassisComm()->setLocalGimbal(state);
}
bool PublishChassisVelocity(const robot::ChassisCommand &command,
                            robot::ChassisMode mode,bool control_enabled) {
  RemoteControlSnapshot remote{};
  const bool online=GetRemoteControlSnapshot(remote);
  robot::GimbalChassisComm::VelocityData data{};
  data.command=command;data.mode=mode;data.remote_online=online;
  data.emergency_stop=!control_enabled||!online;
  data.valid=control_enabled&&online&&mode!=robot::ChassisMode::Disabled;
  auto *comm=GetGimbalChassisComm();
  if(!robot::board_protocol::Valid(data)) {
    // 非法发布立即覆盖旧速度，不能继续发送上一个有效命令。
    comm->publishVelocity({});return false;
  }
  return comm->publishVelocity(data);
}
namespace {
void TransmitBoardStatusAndVelocity() {
  auto *comm=GetGimbalChassisComm();
  auto state=comm->localGimbal();
  hello_world::imu::ImuSnapshot imu{};
  GimbalMotorFeedbackSnapshot joint{};GetGimbalMotorFeedbackSnapshot(joint);
  RemoteControlSnapshot remote{};const bool remote_online=GetRemoteControlSnapshot(remote);
  state.imu_ready=GetImu()->getSnapshot(imu);
  state.feedback=joint.angle;state.yaw_feedback_valid=joint.yaw_valid;
  state.pitch_feedback_valid=joint.pitch_valid;
  state.local_ready=state.local_ready&&state.imu_ready&&joint.yaw_valid&&joint.pitch_valid;
  state.emergency_stop=state.emergency_stop||!remote_online||!state.local_ready;
  if(!comm->setLocalGimbal(state))return;
  uint8_t data[8]{};
  if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)>0&&comm->encode(data)&&
     TryQueueBoardFrame(comm->tx_id(),data))comm->onTxQueued(comm->tx_id());
  // 速度与状态分开尝试发送；不等待对方应答，不阻塞控制中断。
  if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)>0&&comm->encodeVelocity(data,remote_online))
    TryQueueBoardFrame(robot::board_protocol::kVelocityId,data);
}
}

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

  GetRemoteControl();
  //初始化接收状态
  remote_rx_last_pos = 0;
  remote_frame_length = 0;
  remote_valid_frame_count = 0;
  remote_last_valid_rx_ms = 0;
  remote_has_valid_frame = false;
  remote_rx_started = false;
  remote_frame_aligned = false;

  remote_last_retry_ms = hello_world::tick::GetTickMs();
  remote_uart_error_count = 0;
  remote_last_uart_error = HAL_UART_ERROR_NONE;

  HAL_StatusTypeDef rx_status = HAL_UARTEx_ReceiveToIdle_DMA(
      &huart3,
      remote_rx_buffer,
      kRemoteRxBufferSize
  );

  const uint32_t saved_primask = __get_PRIMASK();
  __disable_irq();

  remote_rx_started =
      (rx_status == HAL_OK &&
      remote_uart_error_count == 0 &&
      huart3.RxState == HAL_UART_STATE_BUSY_RX);

  __set_PRIMASK(saved_primask);

  comm_initialized = true;
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

  TransmitBoardStatusAndVelocity();
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
// 处理 DMA 缓冲区中的新增数据

static bool IsRemoteFrameValid(const uint8_t *data)//字段合理性检查
{
  //取出四个摇杆通道和一个拨轮通道的原始值
  const uint16_t channels[5] = {
      static_cast<uint16_t>(
          (data[0] | (data[1] << 8)) & 0x07FF),

      static_cast<uint16_t>(
          ((data[1] >> 3) | (data[2] << 5)) & 0x07FF),

      static_cast<uint16_t>(
          ((data[2] >> 6) | (data[3] << 2) |
           (data[4] << 10)) & 0x07FF),

      static_cast<uint16_t>(
          ((data[4] >> 1) | (data[5] << 7)) & 0x07FF),

      static_cast<uint16_t>(
          data[16] | (data[17] << 8))
  };

  //按现有解码器的 1024 ± 660 检查名义通道范围
  for (uint16_t channel : channels) {
    if (channel < 364 || channel > 1684) {
      return false;
    }
  }

  //两个拨杆的合法值为 1、2、3，不能为 0
  const uint8_t left_switch = (data[5] >> 6) & 0x03;
  const uint8_t right_switch = (data[5] >> 4) & 0x03;

  if (left_switch == 0 || right_switch == 0) {
    return false;
  }

  //鼠标按键字段只能为 0 或 1
  if (data[12] > 1 || data[13] > 1) {
    return false;
  }

  return true;
}

static void ProcessRemoteRxByte(uint8_t byte)
{
  //尚未确定帧起点时，不接收可能错位的数据
  if (!remote_frame_aligned) {
    return;
  }

  //将新字节放入拼帧缓存
  remote_frame_buffer[remote_frame_length++] = byte;

  //不足 18 字节，继续等待后面的字节
  if (remote_frame_length <
      hello_world::remote_control::kRcRxDataLen) {
    return;
  }

  //已经凑齐 18 字节，为下一帧复位计数
  remote_frame_length = 0;

  //异常帧不解码、不刷新有效接收时间
  if (!IsRemoteFrameValid(remote_frame_buffer)) {
    remote_frame_aligned = false;
    return;
  }

  //完整且有效，立即解码
  GetRemoteControl()->decode(remote_frame_buffer);

  //只在有效帧到来时更新这些状态
  remote_last_valid_rx_ms = hello_world::tick::GetTickMs();
  remote_has_valid_frame = true;
  remote_valid_frame_count = remote_valid_frame_count + 1;
}

static void ProcessRemoteRxPosition(uint16_t position, bool idle_event)
{
  if (position == 0 || position > kRemoteRxBufferSize) {
    return;
  }

  if (position > remote_rx_last_pos) {
    //没有越过数组边界，直接处理两次位置之间的数据
    for (uint16_t i = remote_rx_last_pos; i < position; ++i) {
      ProcessRemoteRxByte(remote_rx_buffer[i]);
    }

  } else if (position < remote_rx_last_pos) {
    //越过边界，先处理缓冲区末尾
    for (uint16_t i = remote_rx_last_pos;
         i < kRemoteRxBufferSize; ++i) {
      ProcessRemoteRxByte(remote_rx_buffer[i]);
    }

    //再处理缓冲区开头
    for (uint16_t i = 0; i < position; ++i) {
      ProcessRemoteRxByte(remote_rx_buffer[i]);
    }
  }
  remote_rx_last_pos = position;

  //如果空闲，丢弃残余数据，重置拼帧状态
  if (idle_event) {
  remote_frame_length = 0;
  remote_frame_aligned = true;
  }
}


extern "C" void HAL_UARTEx_RxEventCallback(
    UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart != &huart3 || !remote_rx_started) {
    return;
  }

  if (Size == 0 || Size > kRemoteRxBufferSize) {
    return;
  }

  HAL_UART_RxEventTypeTypeDef event =
      HAL_UARTEx_GetRxEventType(huart);

  switch (event) {
    case HAL_UART_RXEVENT_HT:    // DMA 写到缓冲区一半
    case HAL_UART_RXEVENT_TC:    // DMA 写到缓冲区末尾
    case HAL_UART_RXEVENT_IDLE:  // 串口检测到空闲
      ProcessRemoteRxPosition(
          Size,
          event == HAL_UART_RXEVENT_IDLE
      );
      break;

    default:
      break;
  }

  // 循环 DMA 会继续接收，这里不重新启动 DMA。
}