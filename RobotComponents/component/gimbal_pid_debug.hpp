#ifndef ROBOT_COMPONENTS_GIMBAL_PID_DEBUG_HPP_
#define ROBOT_COMPONENTS_GIMBAL_PID_DEBUG_HPP_

#include <array>
#include <bit>
#include <cstdint>
#include <cmath>
#include "robot_control.hpp"
#include "wheel_pid_debug.hpp"

namespace robot::gimbal_debug {

// 修改这些实验配置后，两块板都需要重新编译、烧录。
inline constexpr bool kEnabled = true;
enum class Axis : uint8_t { Pitch, Yaw };
inline constexpr Axis kAxis = Axis::Pitch;

enum class PitchWaveform : uint8_t { Step, Sine };
inline constexpr PitchWaveform kPitchWaveform = PitchWaveform::Sine;
inline constexpr float kPitchSineCenterRad = 0.0f;      // 标定后的pitch零点
inline constexpr float kPitchSineAmplitudeRad = 0.34906585f; // 20度
inline constexpr uint32_t kPitchSinePeriodMs = 10000;  // 10秒一个周期
// 完成pitch保持测试后，调yaw前手动置true。它不替代运行中的保持检查。
inline constexpr bool kPitchHoldConfirmed = false;
inline constexpr float kStepRad = 0.05235988f;       // 相对启动位置增加3度
inline constexpr uint32_t kDurationMs = 30000;
inline constexpr uint32_t kFreshMs = 100;
inline constexpr uint32_t kPrepareMs = 100;
inline constexpr uint32_t kEnableTimeoutMs = 3000;
inline constexpr uint32_t kArmedTimeoutMs = 5000;
inline constexpr uint32_t kHoldTimeoutMs = 5000;
inline constexpr uint32_t kHoldStableMs = 500;
inline constexpr float kPitchMarginRad = 0.03490659f; // 距软件边界至少2度
inline constexpr float kHoldToleranceRad = 0.01745329f;
inline constexpr float kHoldMaxErrorRad = 0.08726646f;
inline constexpr float kHoldMaxSpeedRadS = 0.05f;
inline constexpr float kNeutralThreshold = 0.05f;
inline constexpr float kWheelStillSpeedRadS = 0.1f;
inline constexpr uint32_t kCommandId = 0x218;
inline constexpr uint32_t kReplyId = 0x219;
inline constexpr uint32_t kTransmitMs = 10;
inline constexpr unsigned kTraceCapacity = 640;
inline constexpr uint32_t kTracePeriodMs = 5;

static_assert(!(kEnabled && wheel_debug::kWheelPidDebugEnabled),
              "Select either wheel PID debug or gimbal PID debug");
static_assert(kAxis == Axis::Pitch || kAxis == Axis::Yaw);
static_assert(kPitchWaveform == PitchWaveform::Step ||
              kPitchWaveform == PitchWaveform::Sine);
static_assert(control::Finite(kPitchSineCenterRad));
static_assert(control::Finite(kPitchSineAmplitudeRad) && kPitchSineAmplitudeRad >= 0.0f);
static_assert(control::Finite(kPitchSineCenterRad - kPitchSineAmplitudeRad));
static_assert(control::Finite(kPitchSineCenterRad + kPitchSineAmplitudeRad));
static_assert(kPitchSinePeriodMs > 0);
static_assert(control::Finite(kStepRad));
static_assert(control::Abs(kStepRad) < control::kPi);
static_assert(kDurationMs > 0 && kDurationMs <= 30000);

enum class Command : uint8_t { Stop, Prepare, Run };
enum class Reply : uint8_t { Stopped, Ready, Fault };
enum class State : uint8_t {
  Waiting, Preparing, Armed, HoldingPitch, EnablingYaw, Running, Finished, Fault
};
enum class Reason : uint8_t {
  None, PreparationRequired, Configuration, RemoteOrSwitch, PeerOffline,
  Communication, Timing, MotorFault, FeedbackTimeout, PitchRange,
  PitchHold, Control, Released, Duration, PreparationTimeout, WheelMoving
};

// 正弦波需要的峰值角速度，用于与原模块的目标限速核对。
constexpr float PitchSineMaxRateRadS() {
  return 2.0f * control::kPi * kPitchSineAmplitudeRad *
         (1000.0f / static_cast<float>(kPitchSinePeriodMs));
}

// elapsed_ms从本次上档启动开始计时；取周期余数避免长时间浮点相位累积。
constexpr float PitchTargetRad(float captured_rad, uint32_t elapsed_ms) {
  if constexpr (kPitchWaveform == PitchWaveform::Step) {
    return captured_rad + kStepRad;
  }
  const uint32_t phase_ms = elapsed_ms % kPitchSinePeriodMs;
  const float phase = 2.0f * control::kPi *
                      (static_cast<float>(phase_ms) /
                       static_cast<float>(kPitchSinePeriodMs));
  return kPitchSineCenterRad + kPitchSineAmplitudeRad * std::sin(phase);
}

constexpr uint32_t ConfigTag() {
  uint32_t hash = wheel_debug::HashWord(2166136261u, 0x47444202u);
  const uint32_t words[] = {
      uint32_t(kAxis), uint32_t(kPitchHoldConfirmed), uint32_t(kPitchWaveform),
      std::bit_cast<uint32_t>(kPitchSineCenterRad),
      std::bit_cast<uint32_t>(kPitchSineAmplitudeRad), kPitchSinePeriodMs,
      std::bit_cast<uint32_t>(kStepRad), kDurationMs, kFreshMs, kPrepareMs,
      kEnableTimeoutMs, kArmedTimeoutMs, kHoldTimeoutMs, kHoldStableMs,
      std::bit_cast<uint32_t>(kPitchMarginRad),
      std::bit_cast<uint32_t>(kHoldToleranceRad),
      std::bit_cast<uint32_t>(kHoldMaxErrorRad),
      std::bit_cast<uint32_t>(kHoldMaxSpeedRadS),
      std::bit_cast<uint32_t>(kNeutralThreshold),
      std::bit_cast<uint32_t>(kWheelStillSpeedRadS), kTransmitMs
  };
  for (uint32_t word : words) {
    hash = wheel_debug::HashWord(hash, word);
  }
  return hash;
}

struct Frame {
  uint32_t tag = ConfigTag();
  uint16_t sequence = 0;
  uint8_t session = 0;
  uint8_t code = 0;
};

constexpr std::array<uint8_t, 8> Encode(const Frame &frame) {
  return {uint8_t(frame.tag >> 24), uint8_t(frame.tag >> 16),
          uint8_t(frame.tag >> 8), uint8_t(frame.tag),
          uint8_t(frame.sequence >> 8), uint8_t(frame.sequence),
          frame.session, frame.code};
}

constexpr Frame Decode(const uint8_t *data) {
  return {(uint32_t(data[0]) << 24) | (uint32_t(data[1]) << 16) |
              (uint32_t(data[2]) << 8) | data[3],
          uint16_t((uint16_t(data[4]) << 8) | data[5]), data[6], data[7]};
}

struct Link {
  Frame frame{};
  uint32_t received_ms = 0;
  bool seen = false;
  bool compatible = true;

  constexpr bool fresh(uint32_t now) const {
    return seen && compatible && uint32_t(now - received_ms) < kFreshMs;
  }

  constexpr bool accept(const Frame &next, uint32_t now) {
    if (next.tag != ConfigTag() || next.code > 2) {
      compatible = false;
      return false;
    }
    if (seen) {
      const uint16_t delta = uint16_t(next.sequence - frame.sequence);
      if (delta == 0) {
        return false;
      }
      if (delta >= 0x8000u) {
        const bool recent = uint32_t(now - received_ms) < kFreshMs;
        // 对端重启只允许通过Stopped/Stop重同步，不能用Run恢复许可。
        if (recent || next.code != 0) {
          return false;
        }
      }
    }
    frame = next;
    received_ms = now;
    seen = true;
    compatible = true;
    return true;
  }
};

// 底盘只能确认零输出。Ready表示接收到了本次Prepare，不授予轮电机许可。
struct FreezeGate {
  Reply reply = Reply::Stopped;
  Reason reason = Reason::None;
  uint8_t session = 0;
  bool saw_stop = false;

  constexpr void step(const Link &command, bool healthy, Reason failure, uint32_t now) {
    reason = Reason::None;
    if (!healthy) {
      reply = Reply::Fault;
      reason = failure;
      saw_stop = false;
      return;
    }
    if (!command.fresh(now)) {
      reply = Reply::Stopped;
      reason = command.compatible ? Reason::PeerOffline : Reason::Configuration;
      saw_stop = false;
      return;
    }
    const auto phase = static_cast<Command>(command.frame.code);
    if (phase == Command::Stop) {
      reply = Reply::Stopped;
      session = command.frame.session;
      saw_stop = true;
    } else if (phase == Command::Prepare) {
      if (saw_stop) {
        reply = Reply::Ready;
        session = command.frame.session;
        saw_stop = false;
      } else if (reply != Reply::Ready || session != command.frame.session) {
        reply = Reply::Fault;
        reason = Reason::PreparationRequired;
      }
    } else if (reply != Reply::Ready || session != command.frame.session) {
      reply = Reply::Fault;
      reason = Reason::PreparationRequired;
    }
  }
};

constexpr bool PitchStartValid(float angle, float low, float high, Axis axis) {
  if (!control::Finite(angle) || !control::Finite(low) || !control::Finite(high)) {
    return false;
  }
  const float minimum = low + kPitchMarginRad;
  const float maximum = high - kPitchMarginRad;
  if (angle < minimum || angle > maximum) {
    return false;
  }
  if (axis == Axis::Pitch) {
    if constexpr (kPitchWaveform == PitchWaveform::Sine) {
      const float lowest = kPitchSineCenterRad - kPitchSineAmplitudeRad;
      const float highest = kPitchSineCenterRad + kPitchSineAmplitudeRad;
      return lowest >= minimum && highest <= maximum;
    }
    const float goal = angle + kStepRad;
    return goal >= minimum && goal <= maximum;
  }
  return true;
}

// 纯状态机：不执行HAL、不写电机，便于验证启停及pitch保持条件。
struct Input {
  bool configuration = false;
  bool remote_safe = false;
  uint8_t right = 0;
  bool healthy = false;
  bool timing = false;
  bool peer_fresh = false;
  bool peer_compatible = true;
  Reply peer_reply = Reply::Stopped;
  uint8_t peer_session = 0;
  bool motor_fault = false;
  bool pitch_feedback = false;
  bool yaw_feedback = false;
  bool position_safe = false;
  bool hold_safe = false;
  bool hold_stable = false;
};

struct Session {
  State state = State::Waiting;
  Reason reason = Reason::PreparationRequired;
  uint8_t session = 0;
  uint8_t previous_right = 0;
  uint32_t entered_ms = 0;
  uint32_t started_ms = 0;
  uint32_t prepare_since = 0;
  uint32_t stable_since = 0;
  bool preparing = false;
  bool stabilizing = false;
  bool started_this_tick = false;
  bool yaw_started_this_tick = false;

  constexpr bool pitchControl() const {
    return state == State::HoldingPitch || state == State::EnablingYaw ||
           state == State::Running;
  }

  constexpr bool yawControl() const {
    return kAxis == Axis::Yaw && state == State::Running;
  }

  constexpr bool pitchEnable() const {
    return state == State::Preparing || state == State::Armed || pitchControl();
  }

  constexpr bool yawEnable() const {
    return kAxis == Axis::Yaw &&
           (state == State::EnablingYaw || state == State::Running);
  }

  constexpr Command command() const {
    if (pitchControl()) {
      return Command::Run;
    }
    if (state == State::Preparing || state == State::Armed) {
      return Command::Prepare;
    }
    return Command::Stop;
  }

  constexpr void fault(Reason cause) {
    state = State::Fault;
    reason = cause;
    preparing = false;
    stabilizing = false;
  }

  constexpr void step(const Input &in, uint32_t now) {
    started_this_tick = false;
    yaw_started_this_tick = false;
    const bool rising = in.right == 1 && previous_right != 1;
    previous_right = in.right;

    if (!in.configuration || !in.peer_compatible) {
      fault(Reason::Configuration);
      return;
    }
    if (!in.remote_safe) {
      fault(Reason::RemoteOrSwitch);
      return;
    }
    if (!in.healthy || in.peer_reply == Reply::Fault) {
      fault(Reason::Communication);
      return;
    }
    if (!in.timing) {
      fault(Reason::Timing);
      return;
    }
    if (!in.peer_fresh) {
      fault(Reason::PeerOffline);
      return;
    }
    if (in.motor_fault) {
      fault(Reason::MotorFault);
      return;
    }

    if (pitchControl()) {
      if (in.right != 1) {
        state = State::Finished;
        reason = Reason::Released;
        return;
      }
      if (uint32_t(now - started_ms) >= kDurationMs) {
        state = State::Finished;
        reason = Reason::Duration;
        return;
      }
      if (in.peer_reply != Reply::Ready || in.peer_session != session) {
        fault(Reason::PeerOffline);
        return;
      }
      if (!in.pitch_feedback || !in.position_safe) {
        fault(in.pitch_feedback ? Reason::PitchRange : Reason::FeedbackTimeout);
        return;
      }
      if (kAxis == Axis::Yaw && !in.hold_safe) {
        fault(Reason::PitchHold);
        return;
      }
      if (yawControl() && !in.yaw_feedback) {
        fault(Reason::FeedbackTimeout);
        return;
      }
      if (state == State::HoldingPitch) {
        if (!in.hold_stable) {
          stabilizing = false;
        } else if (!stabilizing) {
          stabilizing = true;
          stable_since = now;
        } else if (uint32_t(now - stable_since) >= kHoldStableMs) {
          state = State::EnablingYaw;
          entered_ms = now;
        }
        if (state == State::HoldingPitch &&
            uint32_t(now - entered_ms) >= kHoldTimeoutMs) {
          fault(Reason::PitchHold);
        }
      } else if (state == State::EnablingYaw) {
        if (in.yaw_feedback) {
          state = State::Running;
          yaw_started_this_tick = true;
        } else if (uint32_t(now - entered_ms) >= kEnableTimeoutMs) {
          fault(Reason::FeedbackTimeout);
        }
      }
      return;
    }

    if (state == State::Preparing || state == State::Armed) {
      if (in.right != 2 && !(state == State::Armed && rising)) {
        fault(Reason::PreparationRequired);
        return;
      }
      if (state == State::Preparing) {
        if (in.peer_reply == Reply::Ready && in.peer_session == session &&
            in.pitch_feedback) {
          if (!in.position_safe) {
            fault(Reason::PitchRange);
            return;
          }
          state = State::Armed;
          reason = Reason::None;
          entered_ms = now;
        } else if (uint32_t(now - entered_ms) >= kEnableTimeoutMs) {
          fault(Reason::FeedbackTimeout);
        }
      } else if (uint32_t(now - entered_ms) >= kArmedTimeoutMs) {
        fault(Reason::PreparationTimeout);
      } else if (rising) {
        if (!in.pitch_feedback || !in.position_safe ||
            in.peer_reply != Reply::Ready || in.peer_session != session) {
          fault(Reason::PitchRange);
          return;
        }
        state = kAxis == Axis::Pitch ? State::Running : State::HoldingPitch;
        reason = Reason::None;
        entered_ms = now;
        started_ms = now;
        started_this_tick = true;
        stabilizing = false;
      }
      return;
    }

    if (in.right != 2 || in.peer_reply != Reply::Stopped) {
      preparing = false;
      return;
    }
    if (!preparing) {
      preparing = true;
      prepare_since = now;
    }
    if (uint32_t(now - prepare_since) >= kPrepareMs) {
      ++session;
      state = State::Preparing;
      reason = Reason::None;
      entered_ms = now;
      preparing = false;
    }
  }
};

struct Snapshot {
  State state = State::Waiting;
  Reason reason = Reason::PreparationRequired;
  Axis axis = kAxis;
  uint8_t session = 0;
  uint32_t elapsed_ms = 0;
  float requested_rad = 0;
  float target_rad = 0;
  float actual_rad = 0;
  float error_rad = 0;
  float feedforward_nm = 0;
  float torque_nm = 0;
  float pitch_hold_rad = 0;
  float pitch_actual_rad = 0;
  float pitch_error_rad = 0;
  bool feedback_valid = false;
  bool pitch_control = false;
  bool yaw_control = false;
  bool output_limited = false;
};

void Initialize();
void Update();
void Communicate();
void Receive(uint32_t id, const uint8_t data[8]);
extern Snapshot g_gimbal_debug_snapshot;
// 云台保留选中轴开始运动后的前约3.2秒，最后一个槽用于停车快照。
extern std::array<Snapshot, kEnabled ? kTraceCapacity : 1> g_gimbal_debug_trace;
extern uint32_t g_gimbal_debug_trace_count;

} // namespace robot::gimbal_debug
#endif
