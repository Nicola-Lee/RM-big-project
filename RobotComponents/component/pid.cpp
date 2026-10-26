#include "pid.hpp"
#include <algorithm>
#include <cmath>

namespace hello_world {
namespace pid {

Pid::Pid(const PidParams &params)
{
  init(params);
}

void Pid::init(const PidParams &params)
{
  params_ = params;

  // 参数出现 NaN 或无穷大时，改用默认零参数
  if (!std::isfinite(params_.kp) ||
      !std::isfinite(params_.ki) ||
      !std::isfinite(params_.kd) ||
      !std::isfinite(params_.integral_limit) ||//积分限幅
      !std::isfinite(params_.output_limit) ||//输出限幅
      params_.integral_limit < 0.0f ||
      params_.output_limit < 0.0f) {
    params_ = PidParams{};
  }

  reset();
}

void Pid::reset()
{
  integral_output_ = 0.0f;
  last_error_ = 0.0f;
  output_ = 0.0f;
  has_last_error_ = false;
}

float Pid::calculate(float target, float feedback, float dt_s,
                     float feedforward)
{
  if (!std::isfinite(target) || !std::isfinite(feedback)) {
    reset();
    return 0.0f;
  }

  return calculateError(target - feedback, dt_s, feedforward);
}

float Pid::calculateError(float error, float dt_s, float feedforward)
{
  // 周期必须是有限正数，误差也必须有效
  if (!std::isfinite(error) ||
      !std::isfinite(dt_s) ||
      !std::isfinite(feedforward) ||
      dt_s <= 0.0f) {
    reset();
    return 0.0f;
  }

  const float p = params_.kp * error;

  float d = 0.0f;
  if (has_last_error_ && params_.kd != 0.0f) {
    d = params_.kd * ((error - last_error_) / dt_s);
  }

  const float base = p + d + feedforward;

  const float integral_raw =
      integral_output_ + params_.ki * error * dt_s;

  if (!std::isfinite(base) || !std::isfinite(integral_raw)) {
    reset();
    return 0.0f;
  }

  // 积分项限幅
  float next_integral = std::clamp(
      integral_raw,
      -params_.integral_limit,
      params_.integral_limit
  );

  const float candidate_output = base + next_integral;

  if (!std::isfinite(candidate_output)) {
    reset();
    return 0.0f;
  }

  // 输出限幅
  if (candidate_output > params_.output_limit &&
      next_integral > integral_output_) {

    next_integral = std::clamp(
        params_.output_limit - base,
        integral_output_,
        next_integral
    );

    } else if (candidate_output < -params_.output_limit &&
             next_integral < integral_output_) {

    next_integral = std::clamp(
        -params_.output_limit - base,
        next_integral,
        integral_output_
    );
  }

  // 保存本周期状态，供下一次计算使用
  integral_output_ = next_integral;
  last_error_ = error;
  has_last_error_ = true;

  // 最终输出限幅
  output_ = std::clamp(
      base + integral_output_,
      -params_.output_limit,
      params_.output_limit
  );

  return output_;
}

}  // namespace pid
}  // namespace hello_world