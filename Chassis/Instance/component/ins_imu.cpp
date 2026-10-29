/**
 *******************************************************************************
 * @file      :ins_imu.cpp
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
#include "ins_imu.hpp"
#include "imu.hpp"
#include "spi.h"

namespace {
// 配置集中在此处；两块板分别填写，确认后重新编译烧录。
// 现有 GPIO 已初始化 PA4/PB0 为输出高电平，但未标明传感器对应关系。
constexpr bool kImuChipSelectConfirmed = true;  // 暂按加速度计PA4、陀螺仪PB0使用
constexpr bool kImuMountingConfirmed = false;

// 暂用以下片选对应关系；上板时仍需通过芯片ID读取核验。
constexpr uint32_t kAccCsPin = GPIO_PIN_4;   // GPIOA
constexpr uint32_t kGyroCsPin = GPIO_PIN_0;  // GPIOB

// v_module = R * v_sensor。此单位阵只是占位，不代表已确认实车安装。
// 底盘模块坐标：X车头向前、Y车体向左、Z向上。
// 例如模块 X 对应传感器 -Y，则 R 的第一行为 {0,-1,0}。
constexpr float kImuRotation[9] = {
    1.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 1.0f};
}  // namespace

hello_world::imu::Imu *GetImu() {
  static hello_world::imu::Imu imu;
  // 必须由 MainTaskInit 在 GPIO/SPI 初始化后、控制中断启动前首次调用。
  // 只保存配置，不在这里执行包含 SPI/延时的 start()。
  static const bool configuration_applied = []() {
    if (!kImuChipSelectConfirmed || !kImuMountingConfirmed) return false;
    hello_world::imu::BMI088HWConfig hardware;
    hardware.hspi = &hspi1;
    hardware.acc_cs_port = GPIOA;
    hardware.acc_cs_pin = kAccCsPin;
    hardware.gyro_cs_port = GPIOB;
    hardware.gyro_cs_pin = kGyroCsPin;

    hello_world::imu::ImuParams params;
    params.sample_frequency_hz = 1000.0f;  // 与 ImuPoll 的 1ms 调度一致
    params.mahony_kp = 0.25f;
    params.mahony_ki = 0.0f;
    params.calibration_samples = 2000;
    params.convergence_stable_samples = 500;
    params.calibration_timeout_ms = 15000;
    params.convergence_timeout_ms = 10000;
    params.data_timeout_ms = 20;
    return imu.init(hardware, kImuRotation, params);
  }();
  (void)configuration_applied;
  // 未确认配置也返回真实对象，其状态保持 Uninitialized，ready() 为 false。
  return &imu;
}
