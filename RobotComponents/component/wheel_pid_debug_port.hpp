#ifndef ROBOT_COMPONENTS_WHEEL_PID_DEBUG_PORT_HPP_
#define ROBOT_COMPONENTS_WHEEL_PID_DEBUG_PORT_HPP_
#include "wheel_pid_debug.hpp"
#include "runtime_monitor.hpp"
#include "system.hpp"
#include "tick.hpp"
namespace robot::wheel_debug {
// 所有调试状态只在短临界区内更新；不包含SPI、等待、打印或动态分配。
struct Guard {
  uint32_t saved=__get_PRIMASK();
  Guard(){__disable_irq();}
  ~Guard(){__set_PRIMASK(saved);}
};
inline uint32_t Now(){return hello_world::tick::GetTickMs();}
struct Timing {
  uint32_t last=0;
  bool seen=false;
  bool step(uint32_t now){
    const bool okay=!seen||uint32_t(now-last)<runtime::kTaskTimeoutMs;
    last=now;seen=true;return okay;
  }
};
struct Sender {
  uint16_t sequence=0;
  uint32_t last=0;
  uint8_t last_code=255,last_session=0;
  bool seen=false;
  void send(uint32_t id,uint8_t session,uint8_t code,bool control,uint32_t now){
    if(seen&&code==last_code&&session==last_session&&uint32_t(now-last)<kTransmitMs)return;
    if(!runtime::CanMailboxAvailable(runtime::CanBus::One))return;
    const auto bytes=Encode({ConfigTag(),sequence,session,code});
    if(runtime::TrySend(runtime::CanBus::One,id,bytes.data(),control)){
      ++sequence;last=now;last_code=code;last_session=session;seen=true;
    }
  }
};
extern Snapshot g_wheel_debug_snapshot; // 调试器直接观察，不必执行GetSnapshot()
} // namespace robot::wheel_debug
#endif
