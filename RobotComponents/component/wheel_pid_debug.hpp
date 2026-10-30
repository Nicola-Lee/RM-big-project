#ifndef ROBOT_COMPONENTS_WHEEL_PID_DEBUG_HPP_
#define ROBOT_COMPONENTS_WHEEL_PID_DEBUG_HPP_
#include <array>
#include <bit>
#include <cstdint>
#include <cmath>
#include "robot_control.hpp"

namespace robot::wheel_debug {
// 唯一调试开关。修改后必须重新编译、烧录两块板；遥控器不能切换固件模式。
inline constexpr bool kWheelPidDebugEnabled = false;
// 仅指定实验输入，PID、前馈及各级限幅仍取原来的底盘配置。
inline constexpr uint8_t kMotorId = 1;          // 左前1、左后2、右后3、右前4
enum class Waveform:uint8_t {Constant,Sine};
inline constexpr Waveform kWaveform = Waveform::Sine;
inline constexpr float kTargetRadS = 1.0f;     // Constant模式的轮子输出轴目标 rad/s
inline constexpr float kSineAmplitudeRadS = 5.0f;
inline constexpr float kSineFrequencyHz = 0.1f; // 周期10秒
inline constexpr float kSineOffsetRadS = 0.0f;
inline constexpr float kSinePhaseRad = 0.0f;
inline constexpr uint32_t kDurationMs = 30000;
inline constexpr uint32_t kPrepareMs = 100;
inline constexpr uint32_t kFreshMs = 100;
inline constexpr uint32_t kTransmitMs = 10;
inline constexpr float kNeutralThreshold = 0.05f;
inline constexpr uint32_t kCommandId = 0x213;  // 云台 -> 底盘
inline constexpr uint32_t kReplyId = 0x214;    // 底盘 -> 云台
inline constexpr uint32_t kProtocolVersion = 2;
static_assert(kMotorId >= 1 && kMotorId <= 4);
static_assert(kWaveform==Waveform::Constant||kWaveform==Waveform::Sine);
static_assert(control::Finite(kTargetRadS));
static_assert(control::Finite(kSineAmplitudeRadS)&&kSineAmplitudeRadS>=0.0f);
static_assert(control::Finite(kSineFrequencyHz)&&kSineFrequencyHz>0.0f);
static_assert(control::Finite(kSineOffsetRadS)&&control::Finite(kSinePhaseRad));
static_assert(control::Finite(kSineOffsetRadS+kSineAmplitudeRadS)&&
              control::Finite(kSineOffsetRadS-kSineAmplitudeRadS));
static_assert(kDurationMs > 0 && kDurationMs <= 30000);

// 从本次实验启动时刻计时；未限幅目标仍由原Chassis/PID/电机执行限制。
constexpr float TargetRadS(uint32_t elapsed_ms) {
  if(elapsed_ms>=kDurationMs)return 0.0f;
  if constexpr(kWaveform==Waveform::Constant)return kTargetRadS;
  constexpr float kTwoPi=6.2831853071795864769f;
  const float t_s=static_cast<float>(elapsed_ms)*0.001f;
  return kSineOffsetRadS+kSineAmplitudeRadS*
      std::sin(kTwoPi*kSineFrequencyHz*t_s+kSinePhaseRad);
}

// 验证两板使用同一实验输入及协议；PID/前馈/限幅只有底盘原配置这一份。
constexpr uint32_t HashWord(uint32_t h, uint32_t v) {
  for (unsigned i=0;i<4;++i) { h=(h^(v&255u))*16777619u; v>>=8; }
  return h;
}
constexpr uint32_t ConfigTag() {
  uint32_t h=HashWord(2166136261u,kProtocolVersion);
  for (uint32_t v : {uint32_t(kMotorId),uint32_t(kWaveform),
       std::bit_cast<uint32_t>(kTargetRadS),std::bit_cast<uint32_t>(kSineAmplitudeRadS),
       std::bit_cast<uint32_t>(kSineFrequencyHz),std::bit_cast<uint32_t>(kSineOffsetRadS),
       std::bit_cast<uint32_t>(kSinePhaseRad),
       kDurationMs,kPrepareMs,kFreshMs,kTransmitMs,std::bit_cast<uint32_t>(kNeutralThreshold)})
    h=HashWord(h,v);
  return h;
}
enum class Phase:uint8_t {Stop,Prepare,Run};
enum class State:uint8_t {Waiting,Armed,Running,Finished,Fault};
enum class Reason:uint8_t {None,PreparationRequired,RemoteOrSwitch,PeerOffline,
  Configuration,Feedback,Communication,Timing,Duration,Released,PeerStopped,Control};
struct Frame {
  uint32_t tag=ConfigTag();
  uint16_t sequence=0;
  uint8_t session=0,code=0;
};
constexpr std::array<uint8_t,8> Encode(Frame f) {
  return {uint8_t(f.tag>>24),uint8_t(f.tag>>16),uint8_t(f.tag>>8),uint8_t(f.tag),
          uint8_t(f.sequence>>8),uint8_t(f.sequence),f.session,f.code};
}
constexpr Frame Decode(const uint8_t *p) {
  return {(uint32_t(p[0])<<24)|(uint32_t(p[1])<<16)|(uint32_t(p[2])<<8)|p[3],
          uint16_t((uint16_t(p[4])<<8)|p[5]),p[6],p[7]};
}
struct Link {
  Frame frame{};
  uint32_t received_ms=0;
  bool seen=false,compatible=false;
  constexpr bool fresh(uint32_t now) const {
    return seen&&compatible&&uint32_t(now-received_ms)<kFreshMs;
  }
  constexpr bool accept(Frame f,uint32_t now,bool command) {
    if(f.tag!=ConfigTag()||f.code>(command?2u:4u)) { compatible=false; return false; }
    if(seen) {
      const uint16_t delta=uint16_t(f.sequence-frame.sequence);
      if(delta==0) return false; // 重复报文绝不刷新许可
      if(delta>=0x8000u) {
        // 对端重启可在静止阶段重同步；运行帧绝不能重同步/恢复输出。
        if(uint32_t(now-received_ms)<kFreshMs||f.code>1u) return false;
      }
    }
    frame=f;received_ms=now;seen=true;compatible=true;return true;
  }
};

// 只使用纯状态/时间，不访问 HAL，便于编译期覆盖启动、超时及重触发条件。
struct WheelGate {
  State state=State::Waiting;
  Reason reason=Reason::PreparationRequired;
  uint8_t session=0;
  uint32_t started_ms=0;
  bool saw_stop=false;
  constexpr void fault(Reason r) {state=State::Fault;reason=r;saw_stop=false;}
  constexpr void step(bool healthy,Reason failure,const Link &link,uint32_t now) {
    if(!healthy){fault(failure);return;}
    if(!link.fresh(now)){fault(link.seen&&!link.compatible?Reason::Configuration:Reason::PeerOffline);return;}
    const auto p=static_cast<Phase>(link.frame.code);
    if(p==Phase::Stop){
      state=State::Waiting;reason=Reason::PreparationRequired;saw_stop=true;return;
    }
    if(p==Phase::Prepare){
      if(state==State::Running){fault(Reason::Released);return;}
      if(saw_stop){state=State::Armed;reason=Reason::None;session=link.frame.session;saw_stop=false;}
      else if(state==State::Armed&&session!=link.frame.session)fault(Reason::PeerStopped);
      return;
    }
    if(session!=link.frame.session){fault(Reason::PeerStopped);return;}
    if(state==State::Armed){state=State::Running;reason=Reason::None;started_ms=now;}
    if(state==State::Running&&uint32_t(now-started_ms)>=kDurationMs){
      state=State::Finished;reason=Reason::Duration;
    }
    // Finished/Fault 收到后续 Run 保持停止，必须重新 Stop -> Prepare。
  }
};
struct Trigger {
  State state=State::Waiting;
  Reason reason=Reason::PreparationRequired;
  uint8_t session=0,previous_right=0;
  uint32_t prepare_since=0,started_ms=0,armed_ms=0;
  bool preparing=false;
  constexpr Phase phase() const {
    return state==State::Running?Phase::Run:(state==State::Armed?Phase::Prepare:Phase::Stop);
  }
  constexpr void fault(Reason r){state=State::Fault;reason=r;preparing=false;}
  constexpr void step(bool remote_safe,uint8_t right,bool healthy,const Link &peer,uint32_t now){
    const bool rising=right==1&&previous_right!=1;previous_right=right;
    if(!remote_safe||!(right==1||right==2||right==3)){fault(Reason::RemoteOrSwitch);return;}
    if(!healthy){fault(Reason::Communication);return;}
    if(!peer.fresh(now)){fault(peer.seen&&!peer.compatible?Reason::Configuration:Reason::PeerOffline);return;}
    const auto ps=static_cast<State>(peer.frame.code);
    if(state==State::Running){
      if(right!=1){state=State::Finished;reason=Reason::Released;preparing=false;return;}
      if(uint32_t(now-started_ms)>=kDurationMs){state=State::Finished;reason=Reason::Duration;return;}
      if(peer.frame.session!=session||ps==State::Fault||ps==State::Waiting||ps==State::Finished||
         (ps==State::Armed&&uint32_t(now-started_ms)>=kFreshMs))fault(Reason::PeerStopped);
      return;
    }
    if(state==State::Armed){
      if(ps==State::Fault||uint32_t(now-armed_ms)>=5000u){fault(Reason::PeerStopped);return;}
      if(rising){
        if(ps==State::Armed&&peer.frame.session==session){state=State::Running;reason=Reason::None;started_ms=now;}
        else fault(Reason::PreparationRequired);
      }
      return;
    }
    // 上电右拨杆已经在上档时永不启动；下档+中立连续100ms才能准备新实验。
    if(right!=2||ps!=State::Waiting){preparing=false;return;}
    if(!preparing){preparing=true;prepare_since=now;}
    if(uint32_t(now-prepare_since)>=kPrepareMs){
      ++session;state=State::Armed;reason=Reason::None;armed_ms=now;preparing=false;
    }
  }
};

struct Snapshot {
  uint32_t now_ms=0,elapsed_ms=0;
  uint8_t motor_id=kMotorId,session=0;
  State state=State::Waiting;
  Reason reason=Reason::PreparationRequired;
  float requested_rad_s=0,target_rad_s=0,actual_rad_s=0,error_rad_s=0;
  float feedforward=0,pid_output=0,raw_current=0; // pid_output 含前馈及PID总限幅
  bool speed_limited=false,output_limited=false,feedback_valid=false;
};
// 两板各有自己的实现。所有调用仍从现有TIM6控制/通信入口执行。
void Initialize();
void Update();
void Communicate();
void Receive(uint32_t id,const uint8_t data[8]);
Snapshot GetSnapshot();
} // namespace robot::wheel_debug
#endif
