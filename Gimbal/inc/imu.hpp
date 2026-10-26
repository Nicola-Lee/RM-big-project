/**
 *******************************************************************************
 * @file      : imu.hpp
 * @brief     : IMU 设备封装（新人填写）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2024-12-01      Jinletian       1. 初版编写完成
 *  V2.0.0      2026-09-18      Hello World     1. 考核骨架：掏空 Imu 封装类，
 *                                                只保留 BMI088 芯片驱动 + Mahony 姿态
 *******************************************************************************
 * @attention :
 *  本文件已掏空。新人需基于以下两个已提供的组件自行实现 Imu 封装类：
 *    - BMI088：芯片级原始读取（加速度 m/s² / 角速度 rad/s / 温度）
 *    - ahrs::Mahony：姿态解算（输入角速度 + 加速度，输出四元数/姿态角）
 *  需要新人实现：
 *    - 开机零漂计算（静止采样求角速度均值）
 *    - 姿态角输出 roll()/pitch()/yaw()
 *******************************************************************************
 *  Copyright (c) 2025 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_IMU_IMU_HPP_
#define HW_COMPONENTS_DEVICES_IMU_IMU_HPP_

/* Includes ------------------------------------------------------------------*/
#include "BMI088.hpp"
#include "mahony.hpp"
#include <cstdint>

/* ============================================================================
 * 【新人任务】在本文件编写 Imu 封装类（namespace hello_world::imu）。
 *
 * 你要做：
 *   1. 组合已提供的 BMI088（芯片原始读取，加速度 m/s² / 角速度 rad/s，见 BMI088.hpp）
 *      与 Mahony（姿态解算 update(acc, gyro) → getEulerAngle，见 mahony.hpp / ahrs.hpp）。
 *   2. 实现开机零漂：静止采样一段时间的角速度均值，后续每帧扣除该零漂再送入 Mahony。
 *   3. 对外提供 roll() / pitch() / yaw()（单位 rad），以及标定是否完成的查询。
 *
 * 约束 / 提示：
 *   - Mahony 的加速度输入需含重力加速度项，单位与坐标系（x 前 y 左 z 上）见 ahrs_base.hpp。
 *   - 零漂未收敛前，底盘 / 云台状态机应保持在 Dead / Resurrection，不得进入 Working。
 *   - 本骨架只提供了 imu.hpp（无 imu.cpp）；若需 .cpp 实现文件，可自行在 src/ 下新增
 *     imu.cpp（CMake 已递归 glob *.c*，无需改配置）。
 * ==========================================================================*/
namespace hello_world {
namespace imu {

enum class ImuState : uint8_t {
  Uninitialized,  // 尚未配置
  Configured,     // 已保存配置
  Calibrating,    // 正在校准角速度零漂
  Converging,     // 正在等待姿态收敛
  Ready,          // 可以提供有效姿态
  Fault          // 异常
};

struct ImuParams {
  float sample_frequency_hz = 1000.0f;//采样频率
  float mahony_kp = 0.25f;
  float mahony_ki = 0.0f;

  uint32_t calibration_samples = 2000;//零漂计算采样数
  uint32_t convergence_stable_samples = 500;//姿态稳定后连续满足的样本数
  uint32_t calibration_timeout_ms = 15000;// 本轮零漂校准的总超时
  uint32_t convergence_timeout_ms = 10000;// 姿态收敛阶段的总超时
  uint32_t data_timeout_ms = 20;// 姿态快照有效期；达到这个时间即视为过期
};

// 一次性读取的一组姿态数据
struct ImuSnapshot {
  bool ready = false;
  bool calibrated = false;

  float roll = 0.0f;
  float pitch = 0.0f;
  float yaw = 0.0f;

  float gyro[3] = {0.0f, 0.0f, 0.0f};

  uint32_t update_ms = 0;
};

class Imu {
 public:
  Imu() = default;

  // 保存硬件配置、安装方向和算法参数
  bool init(const BMI088HWConfig &hw_config,
            const float rotation_matrix[9],
            const ImuParams &params = ImuParams(),
            const BMI088Config &sensor_config = BMI088Config());

  // 实际配置、检查芯片；只能在前台调用
  bool start();

  // 读取一次数据，并推进校准或姿态解算
  bool update(float dt_s);

  // 重新开始零漂校准
  void restartCalibration();

  ImuState state() const;
  bool calibrated() const;
  bool ready() const;
  BMI088ErrState sensorError() const;

  // 避免数据错位
  bool getSnapshot(ImuSnapshot &snapshot) const;

  // 姿态角
  float roll() const;
  float pitch() const;
  float yaw() const;

 private:
  BMI088 sensor_;
  hello_world::ahrs::Mahony mahony_;
  ImuParams params_{};

  ImuState state_ = ImuState::Uninitialized;
  BMI088ErrState sensor_error_ = kBMI088ErrStateNoErr;
  bool calibrated_ = false;

  // 零漂结果和统计状态
  float acc_[3] = {0.0f, 0.0f, 0.0f};
  float gyro_raw_[3] = {0.0f, 0.0f, 0.0f};
  float gyro_bias_[3] = {0.0f, 0.0f, 0.0f};
  float gyro_mean_[3] = {0.0f, 0.0f, 0.0f};
  float gyro_m2_[3] = {0.0f, 0.0f, 0.0f};

  uint32_t calibration_count_ = 0;
  uint32_t convergence_count_ = 0;
  uint32_t phase_start_ms_ = 0;

  // 姿态收敛后保存的开机参考姿态
  float reference_quat_[4] = {1.0f, 0.0f, 0.0f, 0.0f};

  // 是否已根据重力方向初始化过姿态
  bool attitude_initialized_ = false;

  // 是否已保存开机参考姿态
  bool reference_captured_ = false;

  ImuSnapshot snapshot_{};
};

}
}
#endif /* HW_COMPONENTS_DEVICES_IMU_IMU_HPP_ */
