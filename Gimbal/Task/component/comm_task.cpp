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
#include "ins_robot.hpp"
#include "robot.hpp"
#include "runtime_monitor.hpp"
#include "wheel_pid_debug.hpp"
#include "remote_control_forward.hpp"
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

namespace {
robot::remote_forward::Receiver forwarded_remote;
}
// Ozone简单全局观察名；local UART接收状态与CAN遥控在线状态分开。
volatile bool remote_forward_online=false;
volatile bool remote_forward_emergency=true;
volatile uint32_t remote_forward_rejected=0;

void ReceiveForwardedRemote(uint32_t id,const uint8_t data[8]){
  if constexpr(!robot::remote_forward::kReceiverOnChassis)return;
  if(!comm_initialized)return;
  const uint32_t saved=__get_PRIMASK();__disable_irq();
  robot::remote_forward::CanFrame packet{};
  for(unsigned i=0;i<8;++i)packet[i]=data[i];
  const auto result=forwarded_remote.accept(id,packet,hello_world::tick::GetTickMs());
  if(result==robot::remote_forward::Result::Complete){
    GetRemoteControl()->decode(forwarded_remote.frame.data());
    remote_last_valid_rx_ms=forwarded_remote.valid_ms;
    remote_valid_frame_count=remote_valid_frame_count+1;
    remote_has_valid_frame=true;
  }else if(result==robot::remote_forward::Result::Stopped||result==robot::remote_forward::Result::Invalid){
    remote_has_valid_frame=false;
  }
  __set_PRIMASK(saved);
}

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

  // 只选择一个物理输入来源；转发来源不依赖本地USART3的DMA状态。
  remote_forward_online=forwarded_remote.fresh(now);
  remote_forward_emergency=forwarded_remote.urgent_stop;
  remote_forward_rejected=forwarded_remote.rejected;
  const bool input_ready=robot::remote_forward::kReceiverOnChassis
      ? forwarded_remote.fresh(now)
      : (remote_rx_started&&huart3.RxState==HAL_UART_STATE_BUSY_RX);
  // 必须有完整、新鲜的有效帧；部分分帧不能刷新有效期。
  if (comm_initialized &&
      input_ready &&
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

    if(robot::remote_forward::kReceiverOnChassis&&forwarded_remote.urgent_stop)
      result.rc_r_switch=2; // 下档分帧到达即可收紧；完整新帧才允许解除。
    result.online = true;
  }

  // 恢复进入函数前的中断状态
  __set_PRIMASK(saved_primask);

  snapshot = result;
  return result.online;
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (robot::remote_forward::kReceiverOnChassis || huart != &huart3) {
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
  if constexpr(robot::remote_forward::kReceiverOnChassis)return;
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
// 所有发送统一记录入邮箱结果、忙状态和完成回调；此处不等待总线。
bool TryQueueBoardFrame(uint32_t id, const uint8_t data[8]) {
  const bool control=id==robot::board_protocol::kVelocityId&&(data[7]&4)!=0;
  return robot::runtime::TrySend(robot::runtime::CanBus::One,id,data,control);
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
  out.yaw_fault=robot::board_protocol::Fresh(joint_has_feedback[0],now,joint_last_rx_ms[0])&&
      static_cast<uint8_t>(yaw->status_code())>=8;
  out.pitch_fault=robot::board_protocol::Fresh(joint_has_feedback[1],now,joint_last_rx_ms[1])&&
      static_cast<uint8_t>(pitch->status_code())>=8;
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
  // 发布时再次核对本地拨杆，避免CAN接收中断更新急停后仍带出旧许可。
  const bool armed=online&&robot::control::OperatorArmed(remote.rc_r_switch);
  const bool mode_matches=robot::control::ModeFromSwitch(remote.rc_l_switch)==mode;
  data.emergency_stop=!control_enabled||!armed||!mode_matches;
  data.valid=control_enabled&&armed&&mode_matches&&mode!=robot::ChassisMode::Disabled;
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
  state.remote_online=remote_online;
  state.operator_armed=state.operator_armed&&remote_online&&
      robot::control::OperatorArmed(remote.rc_r_switch)&&
      robot::control::ModeFromSwitch(remote.rc_l_switch)!=robot::ChassisMode::Disabled;
  state.emergency_stop=state.emergency_stop||!state.operator_armed||
      (state.life==robot::LifeState::Working&&!state.local_ready);
  if(!comm->setLocalGimbal(state)){robot::runtime::SetFatal(robot::runtime::Fatal::Encoder);return;}
  uint8_t data[8]{};
  if(robot::runtime::CanMailboxAvailable(robot::runtime::CanBus::One)&&comm->encode(data)&&
     TryQueueBoardFrame(comm->tx_id(),data))comm->onTxQueued(comm->tx_id());
  // 速度与状态分开尝试发送；不等待对方应答，不阻塞控制中断。
  if(robot::runtime::CanMailboxAvailable(robot::runtime::CanBus::One)&&comm->encodeVelocity(data,remote_online))
    TryQueueBoardFrame(robot::board_protocol::kVelocityId,data);
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

  if constexpr(!robot::remote_forward::kReceiverOnChassis){
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
  }

  const uint32_t tx_error_notifications=CAN_IT_TX_MAILBOX_EMPTY|CAN_IT_BUSOFF|
      CAN_IT_ERROR_WARNING|CAN_IT_ERROR_PASSIVE|CAN_IT_ERROR;
  if(HAL_CAN_ActivateNotification(&hcan1,tx_error_notifications|CAN_IT_RX_FIFO0_OVERRUN)!=HAL_OK||
     HAL_CAN_ActivateNotification(&hcan2,tx_error_notifications|CAN_IT_RX_FIFO1_OVERRUN)!=HAL_OK){
    Error_Handler();
  }
  comm_initialized = true;
}

void CommTask(void) {
  using namespace robot::runtime;
  if(!comm_initialized){SetFatal(Fatal::MissingObject);return;}
  ServiceCan();
  if constexpr(robot::wheel_debug::kWheelPidDebugEnabled){
    robot::wheel_debug::Communicate();
    Mark(Task::Communication);
    return;
  }
  auto *yaw=static_cast<hello_world::motor::DM_J4310 *>(GetMotorYaw());
  auto *pitch=static_cast<hello_world::motor::DM_J4310 *>(GetMotorPitch());
  auto *controller=GetRobot();
  if(yaw==nullptr||pitch==nullptr||controller==nullptr){SetFatal(Fatal::MissingObject);return;}
  const auto action=controller->motorAction();
  const bool allow_control=action==robot::control::MotorAction::Control;
  SetControlPermit(allow_control);
  if(!allow_control){yaw->setInput(0.0f);pitch->setInput(0.0f);}
  if(action==robot::control::MotorAction::Disable){yaw->disable();pitch->disable();}
  // encode会消耗达妙的一次性指令，先检查邮箱，忙时保留到下一周期。
  if(CanMailboxAvailable(CanBus::One)){
    uint8_t data[8]{};
    if(!yaw->encode(data)){SetFatal(Fatal::Encoder);return;}
    TrySend(CanBus::One,yaw->tx_id(),data,allow_control);
  }
  if(CanMailboxAvailable(CanBus::Two)){
    uint8_t data[8]{};
    if(!pitch->encode(data)){SetFatal(Fatal::Encoder);return;}
    TrySend(CanBus::Two,pitch->tx_id(),data,allow_control);
  }
  TransmitBoardStatusAndVelocity();
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
// 处理 DMA 缓冲区中的新增数据

static bool IsRemoteFrameValid(const uint8_t *data)
{
  robot::remote_forward::Frame frame{};
  for(unsigned i=0;i<frame.size();++i)frame[i]=data[i];
  return robot::remote_forward::Valid(frame);
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
  if (robot::remote_forward::kReceiverOnChassis || huart != &huart3 || !remote_rx_started) {
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