/*
 * @Author: Nicola Lee nlee15428@gmail.com
 * @Date: 2026-09-19 15:52:24
 * @LastEditors: Nicola Lee nlee15428@gmail.com
 * @LastEditTime: 2026-10-29 19:10:07
 * @FilePath: \omni_skeleton\Gimbal\Instance\component\ins_imu.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
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
constexpr bool kImuMountingConfirmed = true;  // 用户已确认传感器三轴与云台三轴同向

// 暂用以下片选对应关系；上板时仍需通过芯片ID读取核验。
constexpr uint32_t kAccCsPin = GPIO_PIN_4;   // GPIOA暂定未加速度计
constexpr uint32_t kGyroCsPin = GPIO_PIN_0;  // GPIOB暂定为陀螺仪

// v_module = R * v_sensor。三轴与云台同向，使用单位矩阵。
// 云台模块坐标：X枪口向前、Y云台向左、Z向上。
// 无需交换坐标轴或取反；片选仍需独立确认。
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
