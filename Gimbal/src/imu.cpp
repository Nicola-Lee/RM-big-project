#include "imu.hpp"
#include "tick.hpp"

#include <cmath>

namespace hello_world {
namespace imu {

namespace {

// 检查安装旋转矩阵：
// 1. 所有元素为有限数；
// 2. 各行长度为 1，且两两正交；
// 3. 行列式为 +1。
bool ValidRotation(const float *r)
{
  if (r == nullptr) {
    return false;
  }

  for (int i = 0; i < 9; ++i) {
    if (!std::isfinite(r[i])) {
      return false;
    }
  }

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      float dot = 0.0f;

      for (int k = 0; k < 3; ++k) {
        dot += r[3 * i + k] * r[3 * j + k];
      }

      const float expected = (i == j) ? 1.0f : 0.0f;

      if (std::fabs(dot - expected) > 0.001f) {
        return false;
      }
    }
  }

  const float det =
      r[0] * (r[4] * r[8] - r[5] * r[7]) -
      r[1] * (r[3] * r[8] - r[5] * r[6]) +
      r[2] * (r[3] * r[7] - r[4] * r[6]);

  return std::fabs(det - 1.0f) < 0.001f;
}

// 四元数顺序：[w, x, y, z]。
// 检查并归一化，避免浮点误差影响后续角度计算。
bool NormalizeQuaternion(float q[4])
{
  float norm_sq = 0.0f;

  for (int i = 0; i < 4; ++i) {
    if (!std::isfinite(q[i])) {
      return false;
    }

    norm_sq += q[i] * q[i];
  }

  if (!std::isfinite(norm_sq) || norm_sq < 0.000001f) {
    return false;
  }

  const float scale = 1.0f / std::sqrt(norm_sq);

  for (int i = 0; i < 4; ++i) {
    q[i] *= scale;
  }

  return true;
}

// 单位四元数转 Z-Y-X 欧拉角。
// 输出顺序：[roll, pitch, yaw]，单位 rad。
void QuaternionToEuler(const float q[4], float euler[3])
{
  euler[0] = std::atan2(
      2.0f * (q[0] * q[1] + q[2] * q[3]),
      1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]));

  float sin_pitch =
      2.0f * (q[0] * q[2] - q[3] * q[1]);

  // 防止浮点误差使 asin() 输入超出 [-1, 1]。
  if (sin_pitch > 1.0f) {
    sin_pitch = 1.0f;
  }

  if (sin_pitch < -1.0f) {
    sin_pitch = -1.0f;
  }

  euler[1] = std::asin(sin_pitch);

  euler[2] = std::atan2(
      2.0f * (q[0] * q[3] + q[1] * q[2]),
      1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
}

}  // namespace

bool Imu::init(const BMI088HWConfig &hw_config,
               const float rotation_matrix[9],
               const ImuParams &params,
               const BMI088Config &sensor_config)
{
  // 只允许在前台执行，且只配置一次。
  if (__get_IPSR() != 0 ||
      state_ != ImuState::Uninitialized) {
    return false;
  }

  // 两个片选引脚都必须是单个有效 GPIO 引脚。
  const bool pins_valid =
      hw_config.acc_cs_pin != 0 &&
      hw_config.gyro_cs_pin != 0 &&
      (hw_config.acc_cs_pin &
       (hw_config.acc_cs_pin - 1)) == 0 &&
      (hw_config.gyro_cs_pin &
       (hw_config.gyro_cs_pin - 1)) == 0 &&
      hw_config.acc_cs_pin <= GPIO_PIN_15 &&
      hw_config.gyro_cs_pin <= GPIO_PIN_15;

  if (hw_config.hspi == nullptr ||
      !IS_SPI_ALL_INSTANCE(hw_config.hspi->Instance) ||
      !IS_GPIO_ALL_INSTANCE(hw_config.acc_cs_port) ||
      !IS_GPIO_ALL_INSTANCE(hw_config.gyro_cs_port) ||
      !pins_valid ||
      !ValidRotation(rotation_matrix) ||
      !std::isfinite(params.sample_frequency_hz) ||
      params.sample_frequency_hz <= 0.0f ||
      !std::isfinite(params.mahony_kp) ||
      params.mahony_kp < 0.0f ||
      !std::isfinite(params.mahony_ki) ||
      params.mahony_ki < 0.0f ||
      params.calibration_samples < 2 ||
      params.convergence_stable_samples == 0 ||
      params.calibration_timeout_ms == 0 ||
      params.calibration_timeout_ms > 0x7FFFFFFFu ||
      params.convergence_timeout_ms == 0 ||
      params.convergence_timeout_ms > 0x7FFFFFFFu ||
      params.data_timeout_ms == 0 ||
      params.data_timeout_ms > 0x7FFFFFFFu ||
      sensor_config.acc_range > kBMI088AccRange24G ||
      sensor_config.acc_odr < kBMI088AccOdr12_5 ||
      sensor_config.acc_odr > kBMI088AccOdr1600 ||
      sensor_config.acc_osr < kBMI088AccOsr4 ||
      sensor_config.acc_osr > kBMI088AccOsrNormal ||
      sensor_config.gyro_range > kBMI088GyroRange125Dps ||
      sensor_config.gyro_odr_fbw > kBMI088GyroOdrFbw100_32) {
    state_ = ImuState::Fault;
    return false;
  }

  params_ = params;

  // 保存硬件配置和安装旋转矩阵。
  sensor_.init(hw_config, rotation_matrix, sensor_config);

  // 初始化姿态算法；本步骤还不进行姿态更新。
  mahony_.init(
      params_.sample_frequency_hz,
      params_.mahony_kp,
      params_.mahony_ki);

  snapshot_ = ImuSnapshot{};
  calibrated_ = false;
  state_ = ImuState::Configured;

  return true;
}

bool Imu::start()
{
  // 芯片初始化包含阻塞操作，只允许在前台执行。
  if (__get_IPSR() != 0 ||
      state_ != ImuState::Configured) {
    return false;
  }

  // false 表示不执行驱动中的自检流程。
  sensor_error_ = sensor_.imuInit(false);

  if (sensor_error_ != kBMI088ErrStateNoErr) {
    state_ = ImuState::Fault;
    return false;
  }

  state_ = ImuState::Calibrating;
  restartCalibration();

  return true;
}

void Imu::restartCalibration()
{
  if (__get_IPSR() != 0) {
    return;
  }

  if (state_ != ImuState::Calibrating &&
      state_ != ImuState::Converging &&
      state_ != ImuState::Ready) {
    return;
  }

  // 暂停中断，避免控制任务读取到重置一半的状态。
  const uint32_t saved_primask = __get_PRIMASK();
  __disable_irq();

  calibrated_ = false;
  state_ = ImuState::Calibrating;

  calibration_count_ = 0;
  convergence_count_ = 0;

  for (int i = 0; i < 3; ++i) {
    gyro_bias_[i] = 0.0f;
    gyro_mean_[i] = 0.0f;
    gyro_m2_[i] = 0.0f;
  }

  snapshot_ = ImuSnapshot{};

  __set_PRIMASK(saved_primask);

  phase_start_ms_ = hello_world::tick::GetTickMs();

  // 重新校准零漂时保留 reference_quat_，
  // 不在这里重新定义姿态零点。
}

bool Imu::update(float dt_s)
{
  // 本步骤使用阻塞 SPI 读取，只允许在前台执行。
  if (__get_IPSR() != 0) {
    return false;
  }

  if (state_ != ImuState::Calibrating &&
      state_ != ImuState::Converging &&
      state_ != ImuState::Ready) {
    return false;
  }

  const uint32_t now_ms =
      hello_world::tick::GetTickMs();

  const uint32_t phase_elapsed =
      static_cast<uint32_t>(now_ms - phase_start_ms_);

  const bool calibration_timeout =
      state_ == ImuState::Calibrating &&
      phase_elapsed >= params_.calibration_timeout_ms;

  const bool convergence_timeout =
      state_ == ImuState::Converging &&
      phase_elapsed >= params_.convergence_timeout_ms;

  const bool data_timeout =
      state_ == ImuState::Ready &&
      static_cast<uint32_t>(
          now_ms - snapshot_.update_ms) >=
          params_.data_timeout_ms;

  if (calibration_timeout ||
      convergence_timeout ||
      data_timeout) {
    state_ = ImuState::Fault;
    return false;
  }

  if (!std::isfinite(dt_s) ||
    dt_s <= 0.0f ||
    dt_s > 0.02f) {
    state_ = ImuState::Fault;
    return false;
  }

  float acc[3] = {0.0f, 0.0f, 0.0f};
  float gyro[3] = {0.0f, 0.0f, 0.0f};

  // 暂时不读取温度。
  if (!sensor_.getDataChecked(acc, gyro, nullptr)) {
    state_ = ImuState::Fault;
    return false;
  }

  float acc_norm_sq = 0.0f;

  for (int i = 0; i < 3; ++i) {
    if (!std::isfinite(acc[i]) ||
        !std::isfinite(gyro[i])) {
      state_ = ImuState::Fault;
      return false;
    }

    acc_norm_sq += acc[i] * acc[i];
  }

  // 排除全零或无效加速度。
  if (!std::isfinite(acc_norm_sq) ||
      acc_norm_sq < 0.000001f) {
    state_ = ImuState::Fault;
    return false;
  }

  // 保存本次原始读数
  for (int i = 0; i < 3; ++i) {
    acc_[i] = acc[i];
    gyro_raw_[i] = gyro[i];
  }

  // 静止采样，估计陀螺仪零漂。
  if (state_ == ImuState::Calibrating) {
    constexpr float kGravity = 9.80665f;     // m/s²
    constexpr float kAccTolerance = 1.0f;   // m/s²
    constexpr float kGyroLimit = 0.10f;     // rad/s
    constexpr float kGyroStdLimit = 0.02f;  // rad/s

    // 清空当前这轮采样统计。
    auto reset_samples = [this]() {
      calibration_count_ = 0;

      for (int i = 0; i < 3; ++i) {
        gyro_mean_[i] = 0.0f;
        gyro_m2_[i] = 0.0f;
      }
    };

    // 三轴角速度的模长平方。
    float gyro_norm_sq = 0.0f;

    for (int i = 0; i < 3; ++i) {
      gyro_norm_sq += gyro[i] * gyro[i];
    }

    // 静止候选条件：
    // 1. 角速度足够小；
    // 2. 加速度模长接近重力加速度。
    const bool stationary =
        std::isfinite(gyro_norm_sq) &&
        gyro_norm_sq <= kGyroLimit * kGyroLimit &&
        std::fabs(std::sqrt(acc_norm_sq) - kGravity)
            <= kAccTolerance;

    if (!stationary) {
      reset_samples();
      return true;
    }

    // Welford 算法：
    // 不保存全部样本，逐次更新均值和离差平方和。
    ++calibration_count_;

    const float n =
        static_cast<float>(calibration_count_);

    for (int i = 0; i < 3; ++i) {
      const float delta = gyro[i] - gyro_mean_[i];

      gyro_mean_[i] += delta / n;

      const float delta_after =
          gyro[i] - gyro_mean_[i];

      gyro_m2_[i] += delta * delta_after;
    }

    // 样本数量不足，下次 update() 继续采集。
    if (calibration_count_ < params_.calibration_samples) {
      return true;
    }

    // 检查整段采样是否稳定。
    // 每个瞬间的角速度较小，也可能存在持续抖动。
    for (int i = 0; i < 3; ++i) {
      const float variance =
          gyro_m2_[i] /
          static_cast<float>(calibration_count_ - 1);

      if (!std::isfinite(gyro_mean_[i]) ||
          !std::isfinite(variance) ||
          variance > kGyroStdLimit * kGyroStdLimit) {
        reset_samples();

        return true;
      }
    }

    // 一次性保存零漂和状态，
    // 避免中断读到只更新了一部分的结果。
    const uint32_t saved_primask = __get_PRIMASK();
    __disable_irq();

    for (int i = 0; i < 3; ++i) {
      gyro_bias_[i] = gyro_mean_[i];
    }

    calibrated_ = true;
    convergence_count_ = 0;
    state_ = ImuState::Converging;

    __set_PRIMASK(saved_primask);

    phase_start_ms_ =
        hello_world::tick::GetTickMs();
  }

  // 零漂校准完成后才运行姿态算法。
  if (state_ != ImuState::Converging &&
      state_ != ImuState::Ready) {
    return true;
  }

  // 1. 扣除三轴角速度零漂。
  float gyro_corrected[3];
  float gyro_norm_sq = 0.0f;

  for (int i = 0; i < 3; ++i) {
    gyro_corrected[i] = gyro[i] - gyro_bias_[i];

    gyro_norm_sq +=
        gyro_corrected[i] * gyro_corrected[i];
  }

  if (!std::isfinite(gyro_norm_sq)) {
    state_ = ImuState::Fault;
    return false;
  }

  // 2. 首次启动，根据重力方向初始化姿态。
  // 加速度只能确定倾斜方向，初始 yaw 暂设为 0。
  if (!attitude_initialized_) {
    const float roll =
        std::atan2(acc[1], acc[2]);

    const float pitch = std::atan2(
        -acc[0],
        std::sqrt(acc[1] * acc[1] + acc[2] * acc[2]));

    const float cr = std::cos(roll * 0.5f);
    const float sr = std::sin(roll * 0.5f);
    const float cp = std::cos(pitch * 0.5f);
    const float sp = std::sin(pitch * 0.5f);

    // 四元数顺序：[w, x, y, z]。
    float initial_q[4] = {
        cr * cp,
        sr * cp,
        cr * sp,
        -sr * sp
    };

    if (!NormalizeQuaternion(initial_q)) {
      state_ = ImuState::Fault;
      return false;
    }

    mahony_.init(
        initial_q,
        params_.sample_frequency_hz,
        params_.mahony_kp,
        params_.mahony_ki);

    attitude_initialized_ = true;
  }

  // 3. 使用实际时间间隔更新 Mahony。
  // acc 保留重力项，gyro_corrected 已扣除零漂。
  mahony_.update(acc, gyro_corrected, dt_s);

  float q[4];
  mahony_.getQuat(q);

  if (!NormalizeQuaternion(q)) {
    state_ = ImuState::Fault;
    return false;
  }

  // 4. 检查启动姿态是否连续稳定。
  if (state_ == ImuState::Converging) {
    constexpr float kGravity = 9.80665f;
    constexpr float kAccTolerance = 1.0f;
    constexpr float kStillGyro = 0.05f;
    constexpr float kCosThreeDegrees = 0.998629535f;

    // 根据当前四元数计算机体坐标系中的重力方向。
    const float predicted_gravity[3] = {
        2.0f * (q[1] * q[3] - q[0] * q[2]),
        2.0f * (q[0] * q[1] + q[2] * q[3]),
        1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2])
    };

    const float acc_norm = std::sqrt(acc_norm_sq);

    // 估计重力方向与测量方向的夹角余弦。
    float gravity_dot = 0.0f;

    for (int i = 0; i < 3; ++i) {
      gravity_dot +=
          predicted_gravity[i] * acc[i] / acc_norm;
    }

    const bool stable =
        std::fabs(acc_norm - kGravity) <= kAccTolerance &&
        gyro_norm_sq <= kStillGyro * kStillGyro &&
        gravity_dot >= kCosThreeDegrees;

    if (stable) {
      ++convergence_count_;
    } else {
      convergence_count_ = 0;
    }

    if (convergence_count_ <
        params_.convergence_stable_samples) {
      return true;
    }

    // 第一次收敛时保存开机参考姿态。
    // 后续重新校准零漂时，不重新定义姿态零点。
    if (!reference_captured_) {
      for (int i = 0; i < 4; ++i) {
        reference_quat_[i] = q[i];
      }

      reference_captured_ = true;
    }
  }

  // 5. 计算相对开机参考姿态。
  // 单位四元数的逆等于其共轭。
  const float reference_inverse[4] = {
      reference_quat_[0],
      -reference_quat_[1],
      -reference_quat_[2],
      -reference_quat_[3]
  };

  float relative_q[4];

  // q_relative = inverse(q_reference) * q_current
  arm_quaternion_product_single_f32(
      reference_inverse,
      q,
      relative_q);

  if (!NormalizeQuaternion(relative_q)) {
    state_ = ImuState::Fault;
    return false;
  }

  float euler[3];
  QuaternionToEuler(relative_q, euler);

  for (int i = 0; i < 3; ++i) {
    if (!std::isfinite(euler[i])) {
      state_ = ImuState::Fault;
      return false;
    }
  }

  // 6. 准备完整快照。
  ImuSnapshot next{};

  next.ready = true;
  next.calibrated = true;
  next.roll = euler[0];
  next.pitch = euler[1];
  next.yaw = euler[2];
  // q为机体系到重力参考系的旋转；取机体X轴在水平面的投影。
  const float heading_x = 1.0f - 2.0f * (q[2]*q[2] + q[3]*q[3]);
  const float heading_y = 2.0f * (q[1]*q[2] + q[0]*q[3]);
  const float heading_norm_sq = heading_x*heading_x + heading_y*heading_y;
  next.heading_valid = std::isfinite(heading_norm_sq) && heading_norm_sq > 0.000001f;
  next.heading_yaw = next.heading_valid ? std::atan2(heading_y,heading_x) : 0.0f;

  for (int i = 0; i < 3; ++i) {
    next.gyro[i] = gyro_corrected[i];
  }

  next.update_ms = hello_world::tick::GetTickMs();

  // 7. 一次性发布快照和 Ready 状态。
  const uint32_t saved_primask = __get_PRIMASK();
  __disable_irq();

  snapshot_ = next;
  state_ = ImuState::Ready;

  __set_PRIMASK(saved_primask);

  return true;
}

ImuState Imu::state() const
{
  return state_;
}

BMI088ErrState Imu::sensorError() const
{
  // 返回芯片初始化阶段的驱动错误码。
  // 后续读取失败通过 Fault 状态表示。
  return sensor_error_;
}

bool Imu::getSnapshot(ImuSnapshot &result) const
{
  // 同时取得快照、内部状态、参数和当前时间。
  const uint32_t saved_primask = __get_PRIMASK();
  __disable_irq();

  result = snapshot_;
  result.calibrated = calibrated_;

  const bool internally_ready =
      state_ == ImuState::Ready &&
      calibrated_ &&
      snapshot_.ready;

  const uint32_t timeout_ms =
      params_.data_timeout_ms;

  const uint32_t now_ms =
      hello_world::tick::GetTickMs();

  __set_PRIMASK(saved_primask);

  // 在临界区外检查副本，缩短关中断时间。
  const uint32_t age_ms =
      static_cast<uint32_t>(
          now_ms - result.update_ms);

  result.ready =
      internally_ready &&
      age_ms < timeout_ms;

  // 数据未就绪或者已经过期，对外返回无效。
  if (!result.ready) {
    result.roll = 0.0f;
    result.pitch = 0.0f;
    result.yaw = 0.0f;
    result.heading_yaw = 0.0f;
    result.heading_valid = false;

    for (int i = 0; i < 3; ++i) {
      result.gyro[i] = 0.0f;
    }
  }

  return result.ready;
}

bool Imu::calibrated() const
{
  ImuSnapshot data;
  getSnapshot(data);
  return data.calibrated;
}

bool Imu::ready() const
{
  ImuSnapshot data;
  return getSnapshot(data);
}

float Imu::roll() const
{
  ImuSnapshot data;
  getSnapshot(data);
  return data.roll;
}

float Imu::pitch() const
{
  ImuSnapshot data;
  getSnapshot(data);
  return data.pitch;
}

float Imu::yaw() const
{
  ImuSnapshot data;
  getSnapshot(data);
  return data.yaw;
}

}  // namespace imu
}  // namespace hello_world