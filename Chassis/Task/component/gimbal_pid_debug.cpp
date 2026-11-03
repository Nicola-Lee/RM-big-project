#include "gimbal_pid_debug_port.hpp"
#include "comm_task.hpp"
#include "ins_robot.hpp"
#include "ins_motor.hpp"
#include "chassis.hpp"
#include <cmath>

namespace robot::gimbal_debug {
Snapshot g_gimbal_debug_snapshot{};

namespace {
Link command;
Sender sender;
Timing timing;
Chassis *chassis = nullptr;
FreezeGate gate;

void StopWheels() {
  runtime::SetControlPermit(false);
  if (chassis != nullptr) {
    chassis->stop();
  }
  for (int index = 0; index < 4; ++index) {
    auto *motor = GetMotorWheel(index);
    if (motor != nullptr) {
      motor->setInput(0.0f);
    }
  }
}

Reason WheelStopStatus() {
  WheelFeedbackSnapshot feedback{};
  GetWheelFeedbackSnapshot(feedback);
  for (unsigned index = 0; index < 4; ++index) {
    if (!feedback.valid[index]) {
      return Reason::FeedbackTimeout;
    }
    if (!std::isfinite(feedback.velocity[index])) {
      return Reason::FeedbackTimeout;
    }
    if (std::fabs(feedback.velocity[index]) > kWheelStillSpeedRadS) {
      return Reason::WheelMoving;
    }
  }
  return Reason::None;
}
} // namespace

void Initialize() {
  Guard lock;
  chassis = GetChassis();
  StopWheels();
}

void Receive(uint32_t id, const uint8_t data[8]) {
  if (id != kCommandId) {
    return;
  }
  Guard lock;
  command.accept(Decode(data), Now());
}

void Update() {
  Guard lock;
  const uint32_t now = Now();
  StopWheels();
  auto &snapshot = g_gimbal_debug_snapshot;
  const bool timing_ok = timing.step(now);
  const Reason wheel_status = WheelStopStatus();
  const bool wheels_ok = wheel_status == Reason::None;
  const bool healthy = chassis != nullptr && runtime::CanMotionAllowed() &&
                       timing_ok && wheels_ok;
  Reason failure = Reason::Communication;
  if (!timing_ok) {
    failure = Reason::Timing;
  } else if (!wheels_ok) {
    failure = wheel_status;
  }
  gate.step(command, healthy, failure, now);
  snapshot.reason = gate.reason;
  snapshot.state = gate.reply == Reply::Ready ? State::Armed : State::Waiting;
  if (gate.reply == Reply::Fault) {
    snapshot.state = State::Fault;
  }
  snapshot.session = gate.session;
}

void Communicate() {
  Guard lock;
  StopWheels();
  uint8_t data[8]{};
  for (int index = 0; index < 4; ++index) {
    auto *motor = GetMotorWheel(index);
    if (motor == nullptr || !motor->encode(data)) {
      runtime::SetFatal(runtime::Fatal::Encoder);
      return;
    }
  }
  if (runtime::CanMailboxAvailable(runtime::CanBus::Two)) {
    runtime::TrySend(runtime::CanBus::Two,
                     hello_world::motor::M3508::GetTxId(1), data, false);
  }
  sender.send(kReplyId, gate.session, uint8_t(gate.reply), Now());
}
} // namespace robot::gimbal_debug
