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
#include "tick.hpp"

/* Exported function definitions ---------------------------------------------*/
static volatile bool comm_initialized = false;
// 云台只消费底盘CAN转发的完整遥控帧；物理UART接收由底盘负责。
static_assert(robot::remote_forward::kReceiverOnChassis,
              "Gimbal remote input requires chassis CAN forwarding");
static volatile uint32_t remote_valid_frame_count = 0;
static volatile uint32_t remote_last_valid_rx_ms = 0;
static volatile bool remote_has_valid_frame = false;
static constexpr uint32_t kRemoteOfflineTimeoutMs = robot::remote_forward::kTimeoutMs;

namespace {
robot::remote_forward::Receiver forwarded_remote;
}
// Ozone简单全局观察名：来自底盘CAN转发的遥控状态。
volatile bool remote_forward_online=false;
volatile bool remote_forward_emergency=true;
volatile uint32_t remote_forward_rejected=0;

void ReceiveForwardedRemote(uint32_t id,const uint8_t data[8]){
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

  // 仅使用CAN转发重组结果；不依赖云台本地UART状态。
  const bool input_ready=forwarded_remote.fresh(now);
  remote_forward_online=input_ready;
  remote_forward_emergency=forwarded_remote.urgent_stop;
  remote_forward_rejected=forwarded_remote.rejected;
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

    if(forwarded_remote.urgent_stop)
      result.rc_r_switch=2; // 下档分帧到达即可收紧；完整新帧才允许解除。
    result.online = true;
  }

  // 恢复进入函数前的中断状态
  __set_PRIMASK(saved_primask);

  snapshot = result;
  return result.online;
}

// Cube生成的main.c仍调用此C接口，保留空入口以维持原主循环。
// 云台不再执行本地遥控UART接收、重启或错误恢复。
void RemoteControlRxPoll(void) {}

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
   *   2. 创建DT7解码对象并清零CAN遥控有效状态；实际UART接收在底盘。
   *      CAN1收到0x215～0x217后，由ReceiveForwardedRemote重组并解码。
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
  // 初始化来自底盘CAN转发的有效帧状态。
  remote_valid_frame_count = 0;
  remote_last_valid_rx_ms = 0;
  remote_has_valid_frame = false;

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
