#include "wheel_pid_debug_port.hpp"
#include "comm_task.hpp"
#include "ins_motor.hpp"
#include "ins_pid.hpp"
#include "ins_robot.hpp"
#include "chassis.hpp"
#include <cmath>

namespace robot::wheel_debug {
Snapshot g_wheel_debug_snapshot{};
// 每5ms取样，640个槽位保留最近约3.2秒；30秒实验循环覆盖，不阻塞1ms任务。
std::array<Snapshot,kWheelPidDebugEnabled?640:1> g_wheel_debug_trace{};
uint32_t g_wheel_debug_trace_count=0,g_wheel_debug_trace_head=0;
namespace {
Link command;
WheelGate gate;
Sender sender;
Timing timing;
Chassis *chassis=nullptr;
bool initialized=false;
uint32_t last_sample_ms=0;
float requested_rad_s=0.0f; // 最近一次Update送入原单轮接口的目标
constexpr unsigned index=kMotorId-1;

bool Healthy(WheelFeedbackSnapshot &feedback,Reason &reason){
  if(!initialized||chassis==nullptr||!chassis->singleWheelReady(index)){
    reason=Reason::Configuration;return false;
  }
  if(!runtime::CanMotionAllowed()){reason=Reason::Communication;return false;}
  GetWheelFeedbackSnapshot(feedback);
  if(!feedback.valid[index]){reason=Reason::Feedback;return false;}
  reason=Reason::None;return true;
}
void Stop(){
  requested_rad_s=0.0f;
  runtime::SetControlPermit(false);
  if(chassis!=nullptr)chassis->stop();
}
void Observe(uint32_t now,const WheelFeedbackSnapshot &feedback,bool terminal){
  auto &s=g_wheel_debug_snapshot;
  s.now_ms=now;s.state=gate.state;s.reason=gate.reason;s.session=gate.session;
  s.elapsed_ms=gate.state==State::Running||terminal?uint32_t(now-gate.started_ms):s.elapsed_ms;
  s.feedback_valid=feedback.valid[index];s.actual_rad_s=feedback.velocity[index];
  s.requested_rad_s=0;s.target_rad_s=0;s.feedforward=0;s.pid_output=0;s.raw_current=0;
  s.speed_limited=false;s.output_limited=false;
  if(gate.state==State::Running){
    const auto &r=chassis->result();
    s.requested_rad_s=requested_rad_s;s.target_rad_s=r.wheel_target_rad_s[index];
    s.feedforward=chassis->wheelFeedforward(index)*s.target_rad_s;
    s.pid_output=GetPidMotorWheel(index)->output();s.raw_current=r.raw_current[index];
    s.speed_limited=s.target_rad_s!=s.requested_rad_s;
    s.output_limited=std::fabs(s.pid_output)>=GetPidMotorWheel(index)->outputLimit()||s.raw_current!=s.pid_output;
  }
  s.error_rad_s=s.target_rad_s-s.actual_rad_s;
  if(terminal||(gate.state==State::Running&&
      (g_wheel_debug_trace_count==0||uint32_t(now-last_sample_ms)>=5u))){
    g_wheel_debug_trace[g_wheel_debug_trace_head]=s;
    g_wheel_debug_trace_head=(g_wheel_debug_trace_head+1)%g_wheel_debug_trace.size();
    if(g_wheel_debug_trace_count<g_wheel_debug_trace.size())++g_wheel_debug_trace_count;
    last_sample_ms=now;
  }
}
}
void Initialize(){
  if constexpr(!kWheelPidDebugEnabled)return;
  Guard lock;
  chassis=GetChassis(); // 借用原四个电机/PID，零增益可创建但不可运行
  initialized=chassis!=nullptr;
  if(!initialized)runtime::SetFatal(runtime::Fatal::MissingObject);
  Stop();
}
void Receive(uint32_t id,const uint8_t data[8]){
  if constexpr(!kWheelPidDebugEnabled)return;
  if(id!=kCommandId)return;
  Guard lock;command.accept(Decode(data),Now(),true);
}
void Update(){
  if constexpr(!kWheelPidDebugEnabled)return;
  Guard lock;const uint32_t now=Now();
  WheelFeedbackSnapshot feedback{};Reason reason=Reason::None;
  bool healthy=Healthy(feedback,reason);
  if(!timing.step(now)){healthy=false;reason=Reason::Timing;}
  const State previous=gate.state;
  gate.step(healthy,reason,command,now);
  if(gate.state==State::Running){
    if(previous!=State::Running){
      chassis->stop();g_wheel_debug_trace_count=0;g_wheel_debug_trace_head=0;
    }
    requested_rad_s=TargetRadS(uint32_t(now-gate.started_ms));
    if(!chassis->updateSingleWheel(index,requested_rad_s,feedback.valid[index],0.001f))gate.fault(Reason::Control);
  }
  if(gate.state!=State::Running)Stop();
  Observe(now,feedback,previous==State::Running&&gate.state!=State::Running);
}
void Communicate(){
  if constexpr(!kWheelPidDebugEnabled)return;
  Guard lock;const uint32_t now=Now();
  WheelFeedbackSnapshot feedback{};Reason reason=Reason::None;
  const bool was_running=gate.state==State::Running;
  // 发送前再次检查许可；只有Update可以把Armed推进到Running。
  if(was_running){
    if(!Healthy(feedback,reason))gate.fault(reason);
    else if(!command.fresh(now))gate.fault(Reason::PeerOffline);
    else if(command.frame.code!=uint8_t(Phase::Run)||command.frame.session!=gate.session)
      gate.fault(Reason::Released);
    else if(uint32_t(now-gate.started_ms)>=kDurationMs){gate.state=State::Finished;gate.reason=Reason::Duration;}
  }
  const bool allowed=gate.state==State::Running;
  runtime::SetControlPermit(allowed);
  if(!allowed)Stop();
  if(was_running&&!allowed)Observe(now,feedback,true);
  uint8_t data[8]{};
  for(unsigned i=0;i<4;++i){
    auto *motor=GetMotorWheel(i);
    if(motor==nullptr){runtime::SetFatal(runtime::Fatal::MissingObject);Stop();return;}
    if(!allowed||i!=index)motor->setInput(0.0f);
    if(!motor->encode(data)){runtime::SetFatal(runtime::Fatal::Encoder);Stop();return;}
  }
  if(runtime::CanMailboxAvailable(runtime::CanBus::Two))
    runtime::TrySend(runtime::CanBus::Two,hello_world::motor::M3508::GetTxId(1),data,allowed);
  sender.send(kReplyId,gate.session,uint8_t(gate.state),false,now);
}
Snapshot GetSnapshot(){Guard lock;return g_wheel_debug_snapshot;}
} // namespace robot::wheel_debug
