#ifndef ROBOT_COMPONENTS_ROBOT_LIMITS_HPP_
#define ROBOT_COMPONENTS_ROBOT_LIMITS_HPP_
#include "robot_control.hpp"
namespace robot::limits {
// 所有底盘模式共用的角速度指令上限，rad/s。0表示待实车配置。
// 修改后必须重新编译并同时更新两板固件；不代表实际角速度已被测量限幅。
inline constexpr float kMaxBodyRateRadS = 0.0f;
// 最不利情况：底盘与操作者要求的云台转向相反，关节需补偿两者之和。
constexpr bool YawRateBudgetValid(float body, float separate, float spin,
                                  float operator_rate, float joint_rate) {
  return control::Finite(body) && control::Finite(separate) && control::Finite(spin) &&
         control::Finite(operator_rate) && control::Finite(joint_rate) &&
         body > 0.0f && separate > 0.0f && spin > 0.0f && operator_rate > 0.0f &&
         separate <= body && spin <= body &&
         control::Finite(body + operator_rate) && joint_rate >= body + operator_rate;
}
}  // namespace robot::limits
#endif
