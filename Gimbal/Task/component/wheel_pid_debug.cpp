#include "wheel_pid_debug_port.hpp"
#include "comm_task.hpp"
#include "ins_motor.hpp"

namespace robot::wheel_debug {
Snapshot g_wheel_debug_snapshot{};
namespace {
Link reply;
Trigger trigger;
Sender sender;
Timing timing;
bool initialized=false;
bool RemoteSafe(RemoteControlSnapshot &r){
  return GetRemoteControlSnapshot(r)&&r.rc_l_switch==1&&
    control::NearZero(r.rc_lv,r.rc_lh,r.rc_rv,r.rc_rh,r.rc_wheel,kNeutralThreshold);
}
void DisableJoints(){
  auto *yaw=static_cast<hello_world::motor::DM_J4310 *>(GetMotorYaw());
  auto *pitch=static_cast<hello_world::motor::DM_J4310 *>(GetMotorPitch());
  yaw->setInput(0.0f);pitch->setInput(0.0f);yaw->disable();pitch->disable();
}
void Observe(uint32_t now){
  auto &s=g_wheel_debug_snapshot;
  s.now_ms=now;s.state=trigger.state;s.reason=trigger.reason;s.session=trigger.session;
  if(trigger.state==State::Running)s.elapsed_ms=uint32_t(now-trigger.started_ms);
}
}
void Initialize(){
  if constexpr(!kWheelPidDebugEnabled)return;
  Guard lock;
  initialized=GetMotorYaw()!=nullptr&&GetMotorPitch()!=nullptr;
  if(!initialized){runtime::SetFatal(runtime::Fatal::MissingObject);return;}
  runtime::SetControlPermit(false);DisableJoints();
}
void Receive(uint32_t id,const uint8_t data[8]){
  if constexpr(!kWheelPidDebugEnabled)return;
  if(id!=kReplyId)return;
  Guard lock;reply.accept(Decode(data),Now(),false);
}
void Update(){
  if constexpr(!kWheelPidDebugEnabled)return;
  Guard lock;const uint32_t now=Now();
  RemoteControlSnapshot remote{};const bool remote_safe=RemoteSafe(remote);
  trigger.step(remote_safe,remote.rc_r_switch,initialized&&runtime::CanMotionAllowed(),reply,now);
  if(!timing.step(now))trigger.fault(Reason::Timing);
  if(trigger.state!=State::Running)runtime::SetControlPermit(false);
  Observe(now);
}
void Communicate(){
  if constexpr(!kWheelPidDebugEnabled)return;
  Guard lock;const uint32_t now=Now();
  if(!initialized){runtime::SetFatal(runtime::Fatal::MissingObject);return;}
  RemoteControlSnapshot remote{};
  // 此处只能收紧许可，启动/实验计时只由控制任务推进。
  if(!RemoteSafe(remote))trigger.fault(Reason::RemoteOrSwitch);
  else if(!runtime::CanMotionAllowed())trigger.fault(Reason::Communication);
  else if(!reply.fresh(now))trigger.fault(Reason::PeerOffline);
  else if(trigger.state==State::Running&&remote.rc_r_switch!=1)trigger.fault(Reason::Released);
  const bool allowed=trigger.state==State::Running;
  runtime::SetControlPermit(allowed);
  // 调试许可只用于0x213运行许可帧；两根云台轴始终零输入并反复下发失能。
  DisableJoints();
  auto *yaw=static_cast<hello_world::motor::DM_J4310 *>(GetMotorYaw());
  auto *pitch=static_cast<hello_world::motor::DM_J4310 *>(GetMotorPitch());
  for(unsigned i=0;i<2;++i){
    auto *motor=i==0?yaw:pitch;
    const auto bus=i==0?runtime::CanBus::One:runtime::CanBus::Two;
    if(runtime::CanMailboxAvailable(bus)){
      uint8_t data[8]{};
      if(!motor->encode(data)){runtime::SetFatal(runtime::Fatal::Encoder);return;}
      runtime::TrySend(bus,motor->tx_id(),data,false);
    }
  }
  sender.send(kCommandId,trigger.session,uint8_t(trigger.phase()),allowed,now);
  Observe(now);
}
Snapshot GetSnapshot(){Guard lock;return g_wheel_debug_snapshot;}
} // namespace robot::wheel_debug
