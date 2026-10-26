#ifndef ROBOT_COMPONENTS_PID_HPP_
#define ROBOT_COMPONENTS_PID_HPP_

namespace hello_world {
namespace pid {

// PID 参数
struct PidParams {
  float kp = 0.0f;
  float ki = 0.0f;
  float kd = 0.0f;

  // 积分项输出的绝对值上限
  float integral_limit = 0.0f;

  // PID 总输出的绝对值上限
  float output_limit = 0.0f;
};

class Pid {
 public:
  Pid() = default;

  explicit Pid(const PidParams &params);

  // 设置参数并清除历史状态
  void init(const PidParams &params);

  // 计算误差
  // dt_s单位为秒
  float calculate(float target, float feedback, float dt_s,
                float feedforward = 0.0f);

  float calculateError(float error, float dt_s,
                      float feedforward = 0.0f);

  // 清除
  void reset();

  // 获取上一次计算结果
  float output() const { return output_; }

 private:
  PidParams params_{};

  // 每个 PID 对象分别保存自己的运行状态
  float integral_output_ = 0.0f;
  float last_error_ = 0.0f;
  float output_ = 0.0f;
  bool has_last_error_ = false;
};

}  // namespace pid
}  // namespace hello_world

#endif /* ROBOT_COMPONENTS_PID_HPP_ */