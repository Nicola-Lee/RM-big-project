/**
 *******************************************************************************
 * @file      :gimbal_chassis_comm.hpp
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
#ifndef GIMBAL_CHASSIS_COMM_HPP_
#define GIMBAL_CHASSIS_COMM_HPP_
#include <cstdint>
#include <limits>
#include "module_state.hpp"

namespace robot {
// 本地就绪由Robot计算；online由接收时间推导，不在报文中自报在线。
struct BoardChassisData {
  LifeState life = LifeState::Dead;
  ChassisMode mode = ChassisMode::Disabled;
  bool imu_ready = false;
  bool local_ready = false;
  bool emergency_stop = true;
  GimbalMode requested_gimbal_mode = GimbalMode::Disabled;
  GimbalMotion gimbal_target{};  // 可选扩展；当前手动云台目标在云台本地生成
  bool target_valid = false;
};
struct BoardGimbalData {
  LifeState life = LifeState::Dead;
  GimbalMode mode = GimbalMode::Disabled;
  bool imu_ready = false;
  bool local_ready = false;
  bool emergency_stop = true;
  GimbalMotion feedback{};  // 已标定电机关节角，rad；不是IMU绝对姿态
  bool yaw_feedback_valid = false;
  bool pitch_feedback_valid = false;
};
struct BoardVelocityData {
  ChassisCommand command{};  // 云台系vx/vy(m/s)，底盘w请求(rad/s)，CCW为正
  ChassisMode mode = ChassisMode::Disabled;
  bool valid = false;
  bool remote_online = false;
  bool emergency_stop = true;
};

namespace board_protocol {
inline constexpr uint32_t kChassisStatusId = 0x210;
inline constexpr uint32_t kGimbalStatusId = 0x211;
inline constexpr uint32_t kVelocityId = 0x212;
inline constexpr uint8_t kVersion = 1;
inline constexpr uint32_t kTimeoutMs = 100;
inline constexpr uint32_t kPublishTimeoutMs = 20;
inline constexpr float kPi = 3.14159265358979323846f;
inline constexpr float kAngleScale = 10000.0f;
inline constexpr float kVelocityScale = 1000.0f;
// 只是协议可表示范围，不是实车速度上限；实际限速由控制模块/Robot配置。
inline constexpr float kMaxVelocity = 32.767f;
constexpr bool Finite(float v) {
  return v >= -std::numeric_limits<float>::max() && v <= std::numeric_limits<float>::max();
}
constexpr bool Fresh(bool received, uint32_t now, uint32_t last, uint32_t timeout = kTimeoutMs) {
  return received && static_cast<uint32_t>(now-last) < timeout;
}
struct FrameTracker {
  bool received = false;
  uint8_t sequence = 0;
  uint32_t last_ms = 0;
  constexpr bool online(uint32_t now) const { return Fresh(received,now,last_ms); }
  constexpr bool accept(uint8_t next, uint32_t now) {
    if (received) {
      const uint8_t delta = static_cast<uint8_t>(next-sequence);
      if (delta == 0) return false;  // 重复帧不能延长有效期，包括超时后
      if (delta >= 128 && online(now)) return false;  // 丢弃近期逆序帧
    }
    received = true; sequence = next; last_ms = now; return true;
  }
};
constexpr void Clear(uint8_t data[8]) { for(int i=0;i<8;++i)data[i]=0; }
constexpr void Put16(uint8_t *p, int32_t value) {
  const uint16_t u=static_cast<uint16_t>(value);p[0]=static_cast<uint8_t>(u>>8);p[1]=static_cast<uint8_t>(u);
}
constexpr int32_t Get16(const uint8_t *p) {
  const uint32_t u=(static_cast<uint32_t>(p[0])<<8)|p[1];
  return u>=0x8000u ? static_cast<int32_t>(u)-65536 : static_cast<int32_t>(u);
}
constexpr int32_t Quantize(float value, float scale) {
  const double x=static_cast<double>(value)*scale;
  return static_cast<int32_t>(x>=0 ? x+0.5 : x-0.5);
}
constexpr bool AngleValid(float a) { return Finite(a) && a>=-kPi && a<=kPi; }
constexpr bool Valid(const BoardChassisData &d) {
  return IsValid(d.life)&&IsValid(d.mode)&&IsValid(d.requested_gimbal_mode)&&
      (!d.target_valid || (!d.emergency_stop && d.requested_gimbal_mode==GimbalMode::JointAngle &&
                          AngleValid(d.gimbal_target.yaw)&&AngleValid(d.gimbal_target.pitch)));
}
constexpr bool Valid(const BoardGimbalData &d) {
  return IsValid(d.life)&&IsValid(d.mode)&&
      (!d.yaw_feedback_valid||AngleValid(d.feedback.yaw))&&
      (!d.pitch_feedback_valid||AngleValid(d.feedback.pitch));
}
constexpr bool Valid(const BoardVelocityData &d) {
  return IsValid(d.mode)&&(!d.valid ||
      (d.remote_online&&!d.emergency_stop&&d.mode!=ChassisMode::Disabled&&
       Finite(d.command.vx)&&Finite(d.command.vy)&&Finite(d.command.w)&&
       d.command.vx>=-kMaxVelocity&&d.command.vx<=kMaxVelocity&&
       d.command.vy>=-kMaxVelocity&&d.command.vy<=kMaxVelocity&&
       d.command.w>=-kMaxVelocity&&d.command.w<=kMaxVelocity));
}
// 发送时再次收紧权限：发布过期、本地急停、遥控离线均不能带出旧速度。
constexpr BoardVelocityData ConstrainVelocity(BoardVelocityData data, bool fresh,
                                               bool remote_online, bool stopped) {
  data.remote_online=data.remote_online&&remote_online;
  data.emergency_stop=data.emergency_stop||stopped||!data.remote_online;
  data.valid=data.valid&&fresh&&data.remote_online&&!data.emergency_stop;
  if(!data.valid)data.command={};
  return data;
}
constexpr bool VelocityUsable(const BoardVelocityData &data, bool peer_online,
                              bool command_online, bool peer_stopped) {
  return peer_online&&command_online&&!peer_stopped&&data.valid&&
         data.remote_online&&!data.emergency_stop;
}
// 0x210: flags,seq,yawBE16,pitchBE16,targetFlags,version。
constexpr bool Pack(const BoardChassisData &d,uint8_t seq,uint8_t out[8]) {
  Clear(out);if(!Valid(d))return false;
  out[0]=static_cast<uint8_t>(d.life)|(static_cast<uint8_t>(d.mode)<<2)|
         (d.imu_ready?0x10:0)|(d.local_ready?0x20:0)|(d.emergency_stop?0x40:0);
  out[1]=seq;
  if(d.target_valid){Put16(out+2,Quantize(d.gimbal_target.yaw,kAngleScale));Put16(out+4,Quantize(d.gimbal_target.pitch,kAngleScale));}
  out[6]=static_cast<uint8_t>(d.requested_gimbal_mode)|(d.target_valid?2:0);out[7]=kVersion;return true;
}
// 0x211: flags,seq,yawBE16,pitchBE16,reserved0,version。
constexpr bool Pack(const BoardGimbalData &d,uint8_t seq,uint8_t out[8]) {
  Clear(out);if(!Valid(d))return false;
  out[0]=static_cast<uint8_t>(d.life)|(static_cast<uint8_t>(d.mode)<<2)|
         (d.imu_ready?8:0)|(d.local_ready?0x10:0)|(d.yaw_feedback_valid?0x20:0)|
         (d.pitch_feedback_valid?0x40:0)|(d.emergency_stop?0x80:0);
  out[1]=seq;
  if(d.yaw_feedback_valid)Put16(out+2,Quantize(d.feedback.yaw,kAngleScale));
  if(d.pitch_feedback_valid)Put16(out+4,Quantize(d.feedback.pitch,kAngleScale));
  out[7]=kVersion;return true;
}
// 0x212: vxBE16,vyBE16,wBE16,seq,flags；flags高3位为版本。
constexpr bool Pack(const BoardVelocityData &d,uint8_t seq,uint8_t out[8]) {
  Clear(out);if(!Valid(d))return false;
  if(d.valid){Put16(out,Quantize(d.command.vx,kVelocityScale));Put16(out+2,Quantize(d.command.vy,kVelocityScale));Put16(out+4,Quantize(d.command.w,kVelocityScale));}
  out[6]=seq;out[7]=(kVersion<<5)|static_cast<uint8_t>(d.mode)|
      (d.valid?4:0)|(d.remote_online?8:0)|(d.emergency_stop?0x10:0);return true;
}
constexpr float DecodeYaw(int32_t q) {
  const float a=static_cast<float>(q)/kAngleScale;
  // +/-pi量化为+/-31416，允许一个量化边界，再统一到[-pi,pi)。
  return a>=kPi?a-2*kPi:(a< -kPi?a+2*kPi:a);
}
constexpr float DecodePitch(int32_t q) {
  const float a=static_cast<float>(q)/kAngleScale;
  return a>kPi?kPi:(a< -kPi?-kPi:a);
}
constexpr bool RawAngleValid(int32_t q) {return q>=-31416&&q<=31416;}
constexpr bool Unpack(const uint8_t in[8],BoardChassisData &out,uint8_t &seq) {
  if(in[7]!=kVersion||(in[0]&0x80)||(in[6]&0xFC))return false;
  BoardChassisData d{};d.life=static_cast<LifeState>(in[0]&3);d.mode=static_cast<ChassisMode>((in[0]>>2)&3);
  d.imu_ready=in[0]&0x10;d.local_ready=in[0]&0x20;d.emergency_stop=in[0]&0x40;
  d.requested_gimbal_mode=static_cast<GimbalMode>(in[6]&1);d.target_valid=in[6]&2;
  const int32_t y=Get16(in+2),p=Get16(in+4);
  if(!RawAngleValid(y)||!RawAngleValid(p)||(!d.target_valid&&(y!=0||p!=0)))return false;
  d.gimbal_target={DecodeYaw(y),DecodePitch(p)};
  if(!Valid(d))return false;
  out=d;seq=in[1];return true;
}
constexpr bool Unpack(const uint8_t in[8],BoardGimbalData &out,uint8_t &seq) {
  if(in[7]!=kVersion||in[6]!=0)return false;
  BoardGimbalData d{};d.life=static_cast<LifeState>(in[0]&3);d.mode=static_cast<GimbalMode>((in[0]>>2)&1);
  d.imu_ready=in[0]&8;d.local_ready=in[0]&0x10;d.yaw_feedback_valid=in[0]&0x20;
  d.pitch_feedback_valid=in[0]&0x40;d.emergency_stop=in[0]&0x80;
  const int32_t y=Get16(in+2),p=Get16(in+4);
  if(!RawAngleValid(y)||!RawAngleValid(p)||(!d.yaw_feedback_valid&&y!=0)||(!d.pitch_feedback_valid&&p!=0))return false;
  d.feedback={DecodeYaw(y),DecodePitch(p)};
  if(!Valid(d))return false;
  out=d;seq=in[1];return true;
}
constexpr bool Unpack(const uint8_t in[8],BoardVelocityData &out,uint8_t &seq) {
  if((in[7]>>5)!=kVersion)return false;
  BoardVelocityData d{};d.mode=static_cast<ChassisMode>(in[7]&3);d.valid=in[7]&4;
  d.remote_online=in[7]&8;d.emergency_stop=in[7]&0x10;
  const int32_t x=Get16(in),y=Get16(in+2),w=Get16(in+4);
  if(x==-32768||y==-32768||w==-32768||(!d.valid&&(x!=0||y!=0||w!=0)))return false;
  d.command={x/kVelocityScale,y/kVelocityScale,w/kVelocityScale};
  if(!Valid(d))return false;
  out=d;seq=in[6];return true;
}
}  // namespace board_protocol

class GimbalChassisComm {
 public:
  enum class CodePart:uint8_t {Chassis=0,Gimbal=1};
  using ChassisData=BoardChassisData;
  using GimbalData=BoardGimbalData;
  using VelocityData=BoardVelocityData;
  template<class T>struct Snapshot {T data{};bool online=false;uint32_t age_ms=0;};
  using ChassisSnapshot=Snapshot<ChassisData>;
  using GimbalSnapshot=Snapshot<GimbalData>;
  using VelocitySnapshot=Snapshot<VelocityData>;
  GimbalChassisComm(CodePart code_part,uint32_t tx_id,uint32_t rx_id);
  uint32_t tx_id()const{return tx_id_;}
  uint32_t rx_id()const{return rx_id_;}
  // 本地状态由Robot发布；收到的数据通过快照读取，禁止跨中断直接持有可写引用。
  bool setLocalChassis(const ChassisData &data);
  bool setLocalGimbal(const GimbalData &data);
  ChassisData localChassis()const;
  GimbalData localGimbal()const;
  bool publishVelocity(const VelocityData &data); // 每个新控制周期发布，超过20ms不发布即失效
  bool encode(uint8_t tx_data[8]);
  bool encodeVelocity(uint8_t tx_data[8],bool remote_online);
  void onTxQueued(uint32_t id); // 只有HAL成功提交状态帧后才递增心跳序号
  bool decode(uint32_t id,const uint8_t data[8],uint8_t length=8);
  bool decode(const uint8_t data[8]){return decode(rx_id_,data);}
  bool isOffline()const;
  bool getChassisSnapshot(ChassisSnapshot &out)const;
  bool getGimbalSnapshot(GimbalSnapshot &out)const;
  bool getVelocitySnapshot(VelocitySnapshot &out)const;
 private:
  CodePart code_part_;
  uint32_t tx_id_,rx_id_;
  ChassisData local_chassis_{},rx_chassis_{};
  GimbalData local_gimbal_{},rx_gimbal_{};
  VelocityData local_velocity_{},rx_velocity_{};
  board_protocol::FrameTracker peer_tracker_{},velocity_tracker_{};
  uint8_t tx_status_sequence_=0,tx_velocity_sequence_=0;
  uint32_t velocity_publish_ms_=0;
  bool velocity_published_=false;
};
}  // namespace robot
#endif
