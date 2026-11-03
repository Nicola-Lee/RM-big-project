#ifndef ROBOT_COMPONENTS_GIMBAL_PID_DEBUG_PORT_HPP_
#define ROBOT_COMPONENTS_GIMBAL_PID_DEBUG_PORT_HPP_

#include "gimbal_pid_debug.hpp"
#include "runtime_monitor.hpp"
#include "system.hpp"
#include "tick.hpp"

namespace robot::gimbal_debug {
struct Guard {
  uint32_t saved;
  Guard() : saved(__get_PRIMASK()) {
    __disable_irq();
  }
  ~Guard() {
    __set_PRIMASK(saved);
  }
};

inline uint32_t Now() {
  return hello_world::tick::GetTickMs();
}

struct Timing {
  uint32_t last_ms = 0;
  bool seen = false;
  bool step(uint32_t now) {
    const bool valid = !seen || uint32_t(now - last_ms) < runtime::kTaskTimeoutMs;
    seen = true;
    last_ms = now;
    return valid;
  }
};

struct Sender {
  uint16_t sequence = 0;
  uint32_t last_ms = 0;
  uint8_t last_session = 0;
  uint8_t last_code = 255;
  bool seen = false;

  void send(uint32_t id, uint8_t session, uint8_t code, uint32_t now) {
    const bool same = seen && session == last_session && code == last_code;
    if (same && uint32_t(now - last_ms) < kTransmitMs) {
      return;
    }
    if (!runtime::CanMailboxAvailable(runtime::CanBus::One)) {
      return;
    }
    const auto data = Encode({ConfigTag(), sequence, session, code});
    // 此握手只确认底盘保持零输出，不能授予轮电机运动许可。
    if (runtime::TrySend(runtime::CanBus::One, id, data.data(), false)) {
      ++sequence;
      last_ms = now;
      last_session = session;
      last_code = code;
      seen = true;
    }
  }
};
} // namespace robot::gimbal_debug
#endif
