#include "gimbal_pid_debug_port.hpp"
#include "comm_task.hpp"
#include "ins_robot.hpp"
#include "ins_motor.hpp"
#include "ins_pid.hpp"
#include "gimbal.hpp"
#include <cmath>

namespace robot::gimbal_debug {
Snapshot g_gimbal_debug_snapshot{};
std::array<Snapshot, kEnabled ? kTraceCapacity : 1> g_gimbal_debug_trace{};
uint32_t g_gimbal_debug_trace_count = 0;

namespace {
using Motor = hello_world::motor::DaMiao;
Gimbal *gimbal = nullptr;
Motor *yaw = nullptr;
Motor *pitch = nullptr;
Link peer;
Session flow;
Sender sender;
Timing timing;
float pitch_hold_rad = 0;
float pitch_goal_rad = 0;
float yaw_goal_rad = 0;
std::array<uint32_t, 2> last_enable_ms{};
std::array<bool, 2> enable_sent{};
uint32_t last_sample_ms = 0;
bool was_controlling = false;

bool RemoteSafe(RemoteControlSnapshot &remote) {
  if (!GetRemoteControlSnapshot(remote)) {
    return false;
  }
  if (remote.rc_l_switch != 1) {
    return false;
  }
  return control::NearZero(remote.rc_lv, remote.rc_lh,
                           remote.rc_rv, remote.rc_rh,
                           remote.rc_wheel, kNeutralThreshold);
}

bool ConfigurationReady() {
  if (gimbal == nullptr || yaw == nullptr || pitch == nullptr) {
    return false;
  }
  const bool need_yaw = kAxis == Axis::Yaw;
  if (need_yaw && !kPitchHoldConfirmed) {
    return false;
  }
  if (kAxis == Axis::Pitch && kPitchWaveform == PitchWaveform::Sine) {
    const float required_rate = PitchSineMaxRateRadS();
    if (!control::Finite(required_rate) ||
        required_rate > gimbal->pitchTargetRateLimit()) {
      return false;
    }
  }
  return gimbal->debugReady(need_yaw);
}

void Stop() {
  runtime::SetControlPermit(false);
  if (gimbal != nullptr) {
    gimbal->stop();
  }
  if (yaw != nullptr) {
    yaw->setInput(0.0f);
    yaw->disable();
  }
  if (pitch != nullptr) {
    pitch->setInput(0.0f);
    pitch->disable();
  }
}

bool PositionSafe(const GimbalMotorFeedbackSnapshot &feedback) {
  if (gimbal == nullptr || !feedback.pitch_valid) {
    return false;
  }
  const float angle = feedback.angle.pitch;
  if (!flow.pitchControl()) {
    return PitchStartValid(angle, gimbal->pitchMin(), gimbal->pitchMax(), kAxis);
  }
  return angle >= gimbal->pitchMin() + kPitchMarginRad &&
         angle <= gimbal->pitchMax() - kPitchMarginRad;
}

void Observe(uint32_t now, const GimbalMotorFeedbackSnapshot &feedback) {
  Snapshot snapshot{};
  snapshot.state = flow.state;
  snapshot.reason = flow.reason;
  snapshot.session = flow.session;
  snapshot.elapsed_ms = g_gimbal_debug_snapshot.elapsed_ms;
  snapshot.pitch_hold_rad = pitch_hold_rad;
  snapshot.pitch_actual_rad = feedback.angle.pitch;
  snapshot.pitch_error_rad = pitch_hold_rad - feedback.angle.pitch;
  snapshot.pitch_control = flow.pitchControl();
  snapshot.yaw_control = flow.yawControl();
  snapshot.feedback_valid = kAxis == Axis::Pitch ? feedback.pitch_valid : feedback.yaw_valid;
  snapshot.actual_rad = kAxis == Axis::Pitch ? feedback.angle.pitch : feedback.angle.yaw;
  if (flow.pitchControl()) {
    snapshot.elapsed_ms = uint32_t(now - flow.started_ms);
    const auto &result = gimbal->result();
    if (kAxis == Axis::Pitch) {
      snapshot.requested_rad = pitch_goal_rad;
      snapshot.target_rad = result.target.pitch;
      snapshot.error_rad = result.error.pitch;
      snapshot.pitch_error_rad = result.error.pitch;
      snapshot.feedforward_nm = result.pitch_feedforward_nm;
      snapshot.torque_nm = result.pitch_torque_nm;
      snapshot.output_limited = result.pitch_output_limited;
    } else {
      snapshot.requested_rad = yaw_goal_rad;
      snapshot.target_rad = result.target.yaw;
      snapshot.error_rad = result.error.yaw;
      snapshot.feedforward_nm = result.yaw_feedforward_nm;
      snapshot.torque_nm = result.yaw_torque_nm;
      snapshot.output_limited = result.yaw_output_limited;
    }
  }
  g_gimbal_debug_snapshot = snapshot;
  const bool test_running = flow.state == State::Running;
  const bool sample_due = g_gimbal_debug_trace_count == 0 ||
                          uint32_t(now - last_sample_ms) >= kTracePeriodMs;
  if (test_running && sample_due && g_gimbal_debug_trace_count < kTraceCapacity - 1) {
    g_gimbal_debug_trace[g_gimbal_debug_trace_count++] = snapshot;
    last_sample_ms = now;
  }
  if (was_controlling && !flow.pitchControl() &&
      g_gimbal_debug_trace_count < kTraceCapacity) {
    g_gimbal_debug_trace[g_gimbal_debug_trace_count++] = snapshot;
  }
  was_controlling = flow.pitchControl();
}

void QueueAxis(Motor *motor, runtime::CanBus bus, unsigned index,
               bool enable, bool feedback_valid, bool control_enabled,
               uint32_t now) {
  if (!enable) {
    motor->setInput(0.0f);
    motor->disable();
    enable_sent[index] = false;
  } else if (!feedback_valid) {
    motor->setInput(0.0f);
    const bool retry_due = !enable_sent[index] ||
                           uint32_t(now - last_enable_ms[index]) >= 20;
    if (retry_due) {
      motor->enable();
      last_enable_ms[index] = now;
      enable_sent[index] = true;
    }
  } else if (!control_enabled) {
    motor->setInput(0.0f);
  }
  // encode会消耗一次性使能/失能指令，必须先确认邮箱可用。
  if (!runtime::CanMailboxAvailable(bus)) {
    return;
  }
  uint8_t data[8]{};
  if (!motor->encode(data)) {
    runtime::SetFatal(runtime::Fatal::Encoder);
    flow.fault(Reason::Control);
    Stop();
    return;
  }
  runtime::TrySend(bus, motor->tx_id(), data, control_enabled);
}
} // namespace

void Initialize() {
  Guard lock;
  gimbal = GetGimbal();
  yaw = static_cast<Motor *>(GetMotorYaw());
  pitch = static_cast<Motor *>(GetMotorPitch());
  if (gimbal == nullptr || yaw == nullptr || pitch == nullptr) {
    runtime::SetFatal(runtime::Fatal::MissingObject);
  }
  Stop();
}

void Receive(uint32_t id, const uint8_t data[8]) {
  if (id != kReplyId) {
    return;
  }
  Guard lock;
  peer.accept(Decode(data), Now());
}

void Update() {
  Guard lock;
  const uint32_t now = Now();
  if (gimbal == nullptr || yaw == nullptr || pitch == nullptr) {
    flow.fault(Reason::Configuration);
    Stop();
    g_gimbal_debug_snapshot.state = flow.state;
    g_gimbal_debug_snapshot.reason = flow.reason;
    return;
  }
  RemoteControlSnapshot remote{};
  GimbalMotorFeedbackSnapshot feedback{};
  GetGimbalMotorFeedbackSnapshot(feedback);
  Input input{};
  input.configuration = ConfigurationReady();
  input.remote_safe = RemoteSafe(remote);
  input.right = remote.rc_r_switch;
  input.healthy = runtime::CanMotionAllowed();
  input.timing = timing.step(now);
  input.peer_fresh = peer.fresh(now);
  input.peer_compatible = peer.compatible;
  input.peer_reply = static_cast<Reply>(peer.frame.code);
  input.peer_session = peer.frame.session;
  input.motor_fault = feedback.pitch_fault || (flow.yawEnable() && feedback.yaw_fault);
  input.pitch_feedback = feedback.pitch_valid;
  input.yaw_feedback = feedback.yaw_valid;
  input.position_safe = PositionSafe(feedback);
  const float hold_error = std::fabs(pitch_hold_rad - feedback.angle.pitch);
  input.hold_safe = hold_error <= kHoldMaxErrorRad;
  input.hold_stable = hold_error <= kHoldToleranceRad &&
                      std::fabs(pitch->vel()) <= kHoldMaxSpeedRadS &&
                      !gimbal->result().pitch_output_limited;
  flow.step(input, now);

  if (flow.started_this_tick) {
    g_gimbal_debug_trace_count = 0;
    pitch_hold_rad = feedback.angle.pitch;
    pitch_goal_rad = PitchTargetRad(pitch_hold_rad, 0);
    yaw_goal_rad = 0.0f;
    gimbal->stop();
  }
  if (flow.yaw_started_this_tick) {
    yaw_goal_rad = control::Wrap(feedback.angle.yaw + kStepRad);
  }
  if (flow.pitchControl()) {
    if (kAxis == Axis::Pitch) {
      const uint32_t elapsed_ms = uint32_t(now - flow.started_ms);
      pitch_goal_rad = PitchTargetRad(pitch_hold_rad, elapsed_ms);
    }
    GimbalControlInput control_input{};
    control_input.mode = GimbalMode::JointAngle;
    control_input.enabled = true;
    control_input.target_valid = true;
    control_input.pitch_feedback_valid = feedback.pitch_valid;
    control_input.yaw_feedback_valid = feedback.yaw_valid;
    control_input.target.pitch = kAxis == Axis::Pitch ? pitch_goal_rad : pitch_hold_rad;
    control_input.target.yaw = yaw_goal_rad;
    if (!gimbal->updateDebug(control_input, flow.yawControl())) {
      flow.fault(Reason::Control);
    }
  }
  if (!flow.pitchControl()) {
    Stop();
  }
  Observe(now, feedback);
}

void Communicate() {
  Guard lock;
  const uint32_t now = Now();
  if (gimbal == nullptr || yaw == nullptr || pitch == nullptr) {
    flow.fault(Reason::Configuration);
    Stop();
    return;
  }
  RemoteControlSnapshot remote{};
  GimbalMotorFeedbackSnapshot feedback{};
  GetGimbalMotorFeedbackSnapshot(feedback);
  // 发送前只收紧许可；开始实验及状态推进由Update负责。
  if (!RemoteSafe(remote)) {
    flow.fault(Reason::RemoteOrSwitch);
  } else if (!runtime::CanMotionAllowed()) {
    flow.fault(Reason::Communication);
  } else if (!peer.compatible) {
    flow.fault(Reason::Configuration);
  } else if (!peer.fresh(now)) {
    flow.fault(Reason::PeerOffline);
  } else if (flow.pitchControl()) {
    if (remote.rc_r_switch != 1) {
      flow.fault(Reason::Released);
    } else if (!feedback.pitch_valid || !PositionSafe(feedback)) {
      flow.fault(Reason::PitchRange);
    } else if (peer.frame.code != uint8_t(Reply::Ready) ||
               peer.frame.session != flow.session) {
      flow.fault(Reason::PeerOffline);
    } else if (uint32_t(now - flow.started_ms) >= kDurationMs) {
      flow.state = State::Finished;
      flow.reason = Reason::Duration;
    } else if (kAxis == Axis::Yaw &&
               std::fabs(pitch_hold_rad - feedback.angle.pitch) > kHoldMaxErrorRad) {
      flow.fault(Reason::PitchHold);
    } else if (flow.yawControl() && !feedback.yaw_valid) {
      flow.fault(Reason::FeedbackTimeout);
    }
  }
  const bool pitch_control = flow.pitchControl();
  const bool yaw_control = flow.yawControl();
  runtime::SetControlPermit(pitch_control);
  if (!pitch_control) {
    gimbal->stop();
  }
  QueueAxis(yaw, runtime::CanBus::One, 0, flow.yawEnable(),
            feedback.yaw_valid, yaw_control, now);
  QueueAxis(pitch, runtime::CanBus::Two, 1, flow.pitchEnable(),
            feedback.pitch_valid, pitch_control, now);
  sender.send(kCommandId, flow.session, uint8_t(flow.command()), now);
  Observe(now, feedback);
}
} // namespace robot::gimbal_debug
