#ifndef ROBOT_COMPONENTS_ROBOT_CONTROL_HPP_
#define ROBOT_COMPONENTS_ROBOT_CONTROL_HPP_
#include <cstdint>
#include <limits>
#include "module_state.hpp"

namespace robot::control {
inline constexpr float kPi=3.14159265358979323846f;
enum class StopReason:uint8_t {
  None, NotInitialized, RemoteOffline, EmergencyStop, PeerOffline, Configuration,
  ImuNotReady, MotorNotReady, PeerNotReady, RecoveryTimeout, CommandInvalid, ControlFailed, Timing, NeutralRequired, CommunicationFault
};
enum class MotorAction:uint8_t {Disable, EnableZero, Control};
constexpr bool Finite(float v){return v>=-std::numeric_limits<float>::max()&&v<=std::numeric_limits<float>::max();}
constexpr float Abs(float v){return v<0?-v:v;}
constexpr float Clamp(float v,float lo,float hi){return v<lo?lo:(v>hi?hi:v);}
// 输入为两个归一化角的差/和，范围[-2pi,2pi]；恰好180度选负方向。
constexpr float Wrap(float d){return d>=kPi?d-2*kPi:(d< -kPi?d+2*kPi:d);}
constexpr float Stick(float v,float deadband,float sign){
  const float a=Abs(v);return a<=deadband?0.0f:sign*(v<0?-1.0f:1.0f)*(a-deadband)/(1-deadband);
}
constexpr ChassisMode ModeFromSwitch(uint8_t s){
  return s==1?ChassisMode::Separate:(s==3?ChassisMode::Follow:(s==2?ChassisMode::Spin:ChassisMode::Disabled));
}
constexpr bool OperatorArmed(uint8_t right){return right==1||right==3;}
constexpr float Slew(float previous,float target,float rate,float dt){return previous+Clamp(target-previous,-rate*dt,rate*dt);}
constexpr float HeadingTarget(float previous,float manual_rate,float dt){return Wrap(previous+manual_rate*dt);}
constexpr float JointYawTarget(float joint_yaw,float heading_target,float heading){return Wrap(joint_yaw+Wrap(heading_target-heading));}
constexpr bool NearZero(float a,float b,float c,float d,float e,float threshold){
  return Abs(a)<=threshold&&Abs(b)<=threshold&&Abs(c)<=threshold&&Abs(d)<=threshold&&Abs(e)<=threshold;
}
struct NeutralLatch {
  bool timing=false,qualified=false;uint32_t since=0;
  constexpr void reset(){timing=false;qualified=false;since=0;}
  constexpr bool update(bool neutral,uint32_t now,uint32_t duration){
    if(qualified)return true;
    if(!neutral){timing=false;return false;}
    if(!timing){timing=true;since=now;}
    qualified=static_cast<uint32_t>(now-since)>=duration;return qualified;
  }
};
struct LifeInput {
  bool operator_permitted=false;
  StopReason inhibit=StopReason::NotInitialized;
  bool local_ready=false;
  StopReason local_reason=StopReason::ImuNotReady;
  bool peer_ready=false,peer_working=false,peer_stopped=true;
};
class LifeMachine {
 public:
  constexpr void configure(uint32_t stable_ms,uint32_t timeout_ms){
    stable_ms_=stable_ms;timeout_ms_=timeout_ms;state_=LifeState::Dead;
    reason_=StopReason::NotInitialized;blocked_=false;ready_=false;timing_ready_=false;
  }
  constexpr void abort(StopReason reason){state_=LifeState::Dead;reason_=reason;ready_=false;timing_ready_=false;}
  constexpr void step(const LifeInput &in,uint32_t now){
    if(!in.operator_permitted){
      blocked_=false; // 急停/撤销许可后可重新发起恢复
      abort(in.inhibit==StopReason::None?StopReason::EmergencyStop:in.inhibit);
      return;
    }
    if(in.inhibit!=StopReason::None){abort(in.inhibit);return;}
    if(blocked_){abort(StopReason::RecoveryTimeout);return;}
    if(state_==LifeState::Dead){state_=LifeState::Resurrection;entered_=now;timing_ready_=false;ready_=false;}
    if(state_==LifeState::Working){
      if(!in.local_ready){abort(in.local_reason);return;}
      if(!in.peer_ready||in.peer_stopped){abort(StopReason::PeerNotReady);return;}
      reason_=StopReason::None;return;
    }
    if(!in.local_ready){timing_ready_=false;ready_=false;reason_=in.local_reason;}
    else {
      if(!timing_ready_){timing_ready_=true;ready_since_=now;}
      ready_=static_cast<uint32_t>(now-ready_since_)>=stable_ms_;
      reason_=ready_?StopReason::PeerNotReady:StopReason::None;
    }
    if(ready_&&in.peer_ready&&!in.peer_stopped){state_=LifeState::Working;reason_=StopReason::None;return;}
    if(static_cast<uint32_t>(now-entered_)>=timeout_ms_){blocked_=true;abort(StopReason::RecoveryTimeout);}
  }
  constexpr LifeState state()const{return state_;}
  constexpr StopReason reason()const{return reason_;}
  constexpr bool localReady()const{return ready_;}
  constexpr bool canRun(bool peer_working,bool peer_stopped)const{return state_==LifeState::Working&&peer_working&&!peer_stopped;}
 private:
  LifeState state_=LifeState::Dead;StopReason reason_=StopReason::NotInitialized;
  uint32_t stable_ms_=100,timeout_ms_=30000,entered_=0,ready_since_=0;
  bool ready_=false,timing_ready_=false,blocked_=false;
};
struct Diagnostics {
  LifeState life=LifeState::Dead;StopReason reason=StopReason::NotInitialized;
  bool local_ready=false,active=false;ChassisMode chassis_mode=ChassisMode::Disabled;
  uint32_t last_update_ms=0;
};
}  // namespace robot::control
#endif
