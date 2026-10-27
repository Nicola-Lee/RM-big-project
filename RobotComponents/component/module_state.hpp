/**
 *******************************************************************************
 * @file      :module_state.hpp
 * @brief     : 两板共享状态、坐标系与运动数据类型
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MODULE_STATE_HPP_
#define MODULE_STATE_HPP_

#include <cstdint>

namespace robot {

// 固定枚举值供两板共同使用 CAN 编解码仍需逐字段处理及合法性检查
enum class LifeState : uint8_t {
  Dead = 0,          // 未就绪、急停或失效禁止正常控制输出
  Resurrection = 1,  // 恢复、自检，等待各模块满足就绪条件
  Working = 2        // 就绪后响应有效控制指令
};

enum class ControlSource : uint8_t {
  None = 0,
  RemoteControl = 1
};

enum class ChassisMode : uint8_t {
  Disabled = 0,
  Separate = 1,  // 分离：w 来自遥控映射，不自动跟随云台
  Follow = 2,    // 跟随：w 来自朝向误差闭环
  Spin = 3       // 小陀螺：w 为受控自旋角速度
};

enum class GimbalMode : uint8_t {
  Disabled = 0,
  JointAngle = 1  // 闭环控制状态
};

constexpr bool IsValid(LifeState value)
{
  return value == LifeState::Dead || value == LifeState::Resurrection ||
         value == LifeState::Working;
}

constexpr bool IsValid(ControlSource value)
{
  return value == ControlSource::None || value == ControlSource::RemoteControl;
}

constexpr bool IsValid(ChassisMode value)
{
  return value == ChassisMode::Disabled || value == ChassisMode::Separate ||
         value == ChassisMode::Follow || value == ChassisMode::Spin;
}

constexpr bool IsValid(GimbalMode value)
{
  return value == GimbalMode::Disabled || value == GimbalMode::JointAngle;
}

// 两种速度的数值形式相同，但含义不同，禁止隐式互转和跨坐标系相加
enum class MotionFrame : uint8_t { Gimbal = 0, Chassis = 1 };

// vx、vy：m/s；w：底盘自身绕竖直 Z 轴的角速度，rad/s，逆时针为正
// 云台系：X 沿枪管在水平面的朝向，Y 向左，Z 向上
// 底盘系：X 沿车头方向，Y 向车体左方，Z 向上
// 坐标变换只旋转 vx、vy；w 仍是底盘角速度指令，不是云台转速
// 本结构仅描述数据和逐分量运算，不执行坐标变换、限幅或有效性判断
template <MotionFrame Frame>
struct ChassisMotion {
  float vx = 0.0f;
  float vy = 0.0f;
  float w = 0.0f;

  constexpr ChassisMotion operator+(const ChassisMotion &rhs) const
  {
    return {vx + rhs.vx, vy + rhs.vy, w + rhs.w};
  }

  constexpr ChassisMotion operator-(const ChassisMotion &rhs) const
  {
    return {vx - rhs.vx, vy - rhs.vy, w - rhs.w};
  }

  constexpr ChassisMotion operator-() const { return {-vx, -vy, -w}; }

  constexpr ChassisMotion operator*(float scale) const
  {
    return {vx * scale, vy * scale, w * scale};
  }

  constexpr ChassisMotion &operator+=(const ChassisMotion &rhs)
  {
    vx += rhs.vx;
    vy += rhs.vy;
    w += rhs.w;
    return *this;
  }

  constexpr ChassisMotion &operator-=(const ChassisMotion &rhs)
  {
    vx -= rhs.vx;
    vy -= rhs.vy;
    w -= rhs.w;
    return *this;
  }

  constexpr ChassisMotion &operator*=(float scale)
  {
    vx *= scale;
    vy *= scale;
    w *= scale;
    return *this;
  }
};

template <MotionFrame Frame>
constexpr ChassisMotion<Frame> operator*(float scale,
                                         const ChassisMotion<Frame> &motion)
{
  return motion * scale;
}

using GimbalFrameVelocity = ChassisMotion<MotionFrame::Gimbal>;
using ChassisFrameVelocity = ChassisMotion<MotionFrame::Chassis>;
// 遥控映射和模式模块给出的底盘指令，始终使用云台系。
using ChassisCommand = GimbalFrameVelocity;

// 云台关节角，单位 rad；目标和反馈必须先统一参考零点与正方向。
// yaw 绕 +Z 按右手法则（向左为正）；pitch 绕 +Y 按右手法则（下俯为正）。
// 它们不是 IMU 姿态角的直接别名；后续模块负责测量量到关节量的映射。
// 零角度是参考零位目标，不表示失能；失能由 GimbalMode/生命状态决定。
// 普通减法不处理角度回绕，yaw 最短角差和 pitch 限位由控制模块处理。
struct GimbalMotion {
  float yaw = 0.0f;
  float pitch = 0.0f;

  constexpr GimbalMotion operator+(const GimbalMotion &rhs) const
  {
    return {yaw + rhs.yaw, pitch + rhs.pitch};
  }

  constexpr GimbalMotion operator-(const GimbalMotion &rhs) const
  {
    return {yaw - rhs.yaw, pitch - rhs.pitch};
  }

  constexpr GimbalMotion operator-() const { return {-yaw, -pitch}; }

  constexpr GimbalMotion operator*(float scale) const
  {
    return {yaw * scale, pitch * scale};
  }

  constexpr GimbalMotion &operator+=(const GimbalMotion &rhs)
  {
    yaw += rhs.yaw;
    pitch += rhs.pitch;
    return *this;
  }

  constexpr GimbalMotion &operator-=(const GimbalMotion &rhs)
  {
    yaw -= rhs.yaw;
    pitch -= rhs.pitch;
    return *this;
  }

  constexpr GimbalMotion &operator*=(float scale)
  {
    yaw *= scale;
    pitch *= scale;
    return *this;
  }
};

constexpr GimbalMotion operator*(float scale, const GimbalMotion &motion)
{
  return motion * scale;
}

}  // namespace robot

#endif /* MODULE_STATE_HPP_ */
