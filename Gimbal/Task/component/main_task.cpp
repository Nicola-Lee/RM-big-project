/**
 *******************************************************************************
 * @file      :main_task.cpp
 * @brief     :
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
/* Includes ------------------------------------------------------------------*/
#include "main_task.hpp"
#include "comm_task.hpp"
#include "tim.h"
#include "ins_pid.hpp"
#include "ins_robot.hpp"
#include "robot.hpp"
#include "ins_imu.hpp"
#include "imu.hpp"
#include "runtime_monitor.hpp"
#include "wheel_pid_debug.hpp"
#include <cstdint>


namespace {
// IMU_TIMING_HELPERS_BEGIN
constexpr uint32_t ImuCounterFrequency(uint32_t pclk_hz,
                                        bool apb_divided, uint32_t prescaler) {
  const uint64_t timer_hz = static_cast<uint64_t>(pclk_hz) * (apb_divided ? 2u : 1u);
  const uint64_t frequency = timer_hz / (static_cast<uint64_t>(prescaler) + 1u);
  return frequency <= 0xFFFFFFFFull ? static_cast<uint32_t>(frequency) : 0u;
}
constexpr uint32_t ImuSampleCounts(uint32_t frequency_hz) {
  // 向上取整，使两次采样至少间隔 1ms；当前配置分别为1000和84000计数。
  return static_cast<uint32_t>((static_cast<uint64_t>(frequency_hz) + 999u) / 1000u);
}
constexpr uint32_t ImuElapsedCounts(uint32_t now, uint32_t previous) {
  return now - previous;  // 无符号减法处理一次32位回绕
}
constexpr float ImuElapsedSeconds(uint32_t counts, uint32_t frequency_hz) {
  return static_cast<float>(counts) / static_cast<float>(frequency_hz);
}
// IMU_TIMING_HELPERS_END

hello_world::imu::Imu *imu_instance = nullptr;
bool imu_poll_initialized = false;
bool imu_sampling_started = false;
uint32_t imu_counter_hz = 0;
uint32_t imu_sample_counts = 0;
uint32_t imu_timer_prescaler = 0;
uint32_t imu_last_count = 0;
uint32_t imu_last_ms = 0;
volatile ImuPollStatus imu_poll_status = kImuPollWaitingForInit;
}  // namespace

ImuPollStatus GetImuPollStatus(void) {
  return imu_poll_status;
}

void ImuPoll(void) {
  // BMI088启动含HAL_Delay，采样是阻塞SPI；仅允许中断开放时从主循环调用。
  if (__get_IPSR() != 0 || __get_PRIMASK() != 0 || !imu_poll_initialized) return;
  auto *imu = imu_instance;
  if (imu == nullptr) {
    imu_poll_status = kImuPollFault;
    return;
  }
  using State = hello_world::imu::ImuState;
  const auto state = imu->state();
  if (state == State::Uninitialized) {
    imu_poll_status = kImuPollWaitingForConfig;
    return;
  }
  if (state == State::Fault) {
    imu_poll_status = kImuPollFault;
    return;  // 保留故障，不在每次轮询时重复初始化或重设姿态零点
  }

  const bool timers_running = htim2.Instance == TIM2 && htim6.Instance == TIM6 &&
      (htim2.Instance->CR1 & TIM_CR1_CEN) != 0 &&
      (htim6.Instance->CR1 & TIM_CR1_CEN) != 0 &&
      (htim6.Instance->DIER & TIM_DIER_UIE) != 0;
  if (!timers_running) {
    if (imu_sampling_started) {
      imu->update(0.0f);  // 非法dt使现有Imu进入Fault，不进行SPI读取
      imu_poll_status = kImuPollFault;
    } else {
      imu_poll_status = kImuPollWaitingForTimer;
    }
    return;
  }

  if (!imu_sampling_started) {
    if (state != State::Configured || htim2.Instance->ARR != 0xFFFFFFFFu) {
      imu_poll_status = kImuPollTimingError;
      return;
    }
    RCC_ClkInitTypeDef clocks{};
    uint32_t latency = 0;
    HAL_RCC_GetClockConfig(&clocks, &latency);
    imu_timer_prescaler = htim2.Instance->PSC;
    imu_counter_hz = ImuCounterFrequency(HAL_RCC_GetPCLK1Freq(),
        clocks.APB1CLKDivider != RCC_HCLK_DIV1, imu_timer_prescaler);
    if (imu_counter_hz < 1000u) {
      imu_poll_status = kImuPollTimingError;
      return;
    }
    imu_sample_counts = ImuSampleCounts(imu_counter_hz);
    imu_poll_status = kImuPollStarting;
    // 此时TIM6通信已运行（集中喂狗在主循环末尾），阻塞初始化发生在前台，不占用控制中断。
    if (!imu->start()) {
      imu_poll_status = kImuPollFault;
      return;
    }
    // 初始化耗时不计入第一帧dt；启动后再开始测量采样间隔。
    imu_last_count = __HAL_TIM_GET_COUNTER(&htim2);
    imu_last_ms = HAL_GetTick();
    imu_sampling_started = true;
    robot::runtime::Mark(robot::runtime::Task::Imu);
    imu_poll_status = kImuPollSampling;
    return;
  }

  // 采样期间不允许更改TIM2分频/周期；停顿超过20ms不能伪装成短采样间隔。
  // 毫秒检查同时避免TIM2停顿多圈后的模运算把长间隔误判成短间隔。
  const uint32_t now_ms = HAL_GetTick();
  if (htim2.Instance->ARR != 0xFFFFFFFFu ||
      htim2.Instance->PSC != imu_timer_prescaler ||
      static_cast<uint32_t>(now_ms - imu_last_ms) > 20u) {
    imu->update(0.0f);
    imu_poll_status = kImuPollFault;
    return;
  }
  const uint32_t now_count = __HAL_TIM_GET_COUNTER(&htim2);
  const uint32_t elapsed = ImuElapsedCounts(now_count, imu_last_count);
  if (elapsed < imu_sample_counts) return;
  const float dt_s = ImuElapsedSeconds(elapsed, imu_counter_hz);
  imu_last_count = now_count;
  imu_last_ms = now_ms;
  // 一次只处理一个当前样本，使用实际dt；校准/收敛/快照由Imu类推进。
  const bool updated=imu->update(dt_s);
  // 一次读取/处理已完成；硬件读取失败且进入Fault也是有界处理结果。
  robot::runtime::Mark(robot::runtime::Task::Imu);
  if (!updated) {
    imu_poll_status = kImuPollFault;
    return;
  }
  imu_poll_status = imu->state() == State::Ready ? kImuPollReady : kImuPollSampling;
}

/* Exported function definitions ---------------------------------------------*/
void MainTaskInit(void) {
  // 只执行一次；外设已由main.c初始化，尚未开启TIM6周期中断。
  if (imu_poll_initialized) return;
  robot::runtime::Initialize();
  imu_instance = GetImu();
  if constexpr(robot::wheel_debug::kWheelPidDebugEnabled){
    robot::wheel_debug::Initialize();
    CommTaskInit();
    imu_poll_initialized=true;
    return;
  }
  /* TODO【新人任务】:
   * 一次性初始化（由 main() 在 while(1) 之前调用一次）：
   *   1. 通过 Instance 层的 GetXxx() 拿到电机 / IMU / 遥控器 / 双板通讯 / Robot 等对象，
   *      按依赖顺序完成初始化（电机使能、IMU 零漂采集、CAN 滤波器等）。
   *   2. 把各模块注册进 Robot（registerXxx）。
   *   3. 调用 CommTaskInit() 完成通讯初始化（UART 接收、IWDG 等）。
   * 注意：不要在这里写任何周期性逻辑。
   */
  GetPidMotorYaw();
  GetPidMotorPitch();

  // 在启动通信和周期控制前创建云台、电机与两个 PID 单例；此处不使能电机。
  if (GetGimbal() == nullptr) {
    Error_Handler();
    return;
  }
  if(GetRobot()==nullptr){
    Error_Handler();
    return;
  }
  CommTaskInit();
  imu_poll_initialized = true;
  imu_poll_status = imu_instance->state() == hello_world::imu::ImuState::Uninitialized
                        ? kImuPollWaitingForConfig
                        : (imu_instance->state() == hello_world::imu::ImuState::Fault
                               ? kImuPollFault : kImuPollWaitingForTimer);
}

void MainTask(void) {
  if constexpr(robot::wheel_debug::kWheelPidDebugEnabled){
    robot::wheel_debug::Update();
    robot::runtime::Mark(robot::runtime::Task::Control);
    return;
  }
  // 原TIM6的1ms入口。IMU阻塞采样继续在主循环，控制只读取快照。
  auto *controller=GetRobot();
  if(controller==nullptr){robot::runtime::SetFatal(robot::runtime::Fatal::MissingObject);return;}
  controller->update(0.001f);
  robot::runtime::Mark(robot::runtime::Task::Control);
}

void WatchdogPoll(void) {
  using State=hello_world::imu::ImuState;
  const auto state=imu_instance==nullptr?State::Uninitialized:imu_instance->state();
  const bool sampling=state==State::Calibrating||state==State::Converging||state==State::Ready;
  robot::runtime::ServiceWatchdog(sampling);
}

/* Private function definitions ----------------------------------------------*/
/* TODO【新人任务】:
 * 实现 htim6 周期中断回调 HAL_TIM_PeriodElapsedCallback：
 *   - 先判中断来自 htim6（避免把别的定时器中断误当成 1ms tick）；
 *   - 再调用 MainTask() 与 CommTask()。
 * 注意：
 *   - main.c 已 HAL_TIM_Base_Start_IT(&htim6)，这里只需补中断里的分发；
 *   - HAL 回调是 C 链接，需用 extern "C" 包裹，否则中断里找不到符号。
 */
extern "C" void HAL_TIM_PeriodElapsedCallback(
    TIM_HandleTypeDef *htim) {

  if (htim != &htim6) {
    return;
  }

  MainTask();
  CommTask();
}