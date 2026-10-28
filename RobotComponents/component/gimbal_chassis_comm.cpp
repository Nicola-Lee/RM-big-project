/**
 *******************************************************************************
 * @file      :gimbal_chassis_comm.cpp
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
#include "gimbal_chassis_comm.hpp"
#include "tick.hpp"
#include "system.hpp"
namespace {
struct InterruptGuard {
  uint32_t saved=__get_PRIMASK();
  InterruptGuard(){__disable_irq();}
  ~InterruptGuard(){__set_PRIMASK(saved);}
};
}
namespace robot {
GimbalChassisComm::GimbalChassisComm(CodePart part,uint32_t tx,uint32_t rx)
    :code_part_(part),tx_id_(tx),rx_id_(rx) {}
bool GimbalChassisComm::setLocalChassis(const ChassisData &data){
  if(code_part_!=CodePart::Chassis||!board_protocol::Valid(data))return false;
  InterruptGuard lock;local_chassis_=data;return true;
}
bool GimbalChassisComm::setLocalGimbal(const GimbalData &data){
  if(code_part_!=CodePart::Gimbal||!board_protocol::Valid(data))return false;
  InterruptGuard lock;local_gimbal_=data;return true;
}
GimbalChassisComm::ChassisData GimbalChassisComm::localChassis()const{
  InterruptGuard lock;return local_chassis_;
}
GimbalChassisComm::GimbalData GimbalChassisComm::localGimbal()const{
  InterruptGuard lock;return local_gimbal_;
}
bool GimbalChassisComm::publishVelocity(const VelocityData &data){
  if(code_part_!=CodePart::Gimbal)return false;
  const bool valid=board_protocol::Valid(data);
  InterruptGuard lock;local_velocity_=valid?data:VelocityData{};
  velocity_publish_ms_=hello_world::tick::GetTickMs();velocity_published_=true;
  ++tx_velocity_sequence_;return valid;
}
bool GimbalChassisComm::encode(uint8_t out[8]){
  if(out==nullptr)return false;
  ChassisData c;GimbalData g;uint8_t seq;
  {InterruptGuard lock;c=local_chassis_;g=local_gimbal_;seq=tx_status_sequence_;}
  return code_part_==CodePart::Chassis?board_protocol::Pack(c,seq,out):board_protocol::Pack(g,seq,out);
}
bool GimbalChassisComm::encodeVelocity(uint8_t out[8],bool remote_online){
  if(out==nullptr||code_part_!=CodePart::Gimbal)return false;
  VelocityData data;uint8_t seq;bool fresh;bool stopped;
  {InterruptGuard lock;data=local_velocity_;seq=tx_velocity_sequence_;
   fresh=board_protocol::Fresh(velocity_published_,hello_world::tick::GetTickMs(),
                              velocity_publish_ms_,board_protocol::kPublishTimeoutMs);
   stopped=local_gimbal_.emergency_stop;}
  data=board_protocol::ConstrainVelocity(data,fresh,remote_online,stopped);
  return board_protocol::Pack(data,seq,out);
}
void GimbalChassisComm::onTxQueued(uint32_t id){
  if(id==tx_id_){InterruptGuard lock;++tx_status_sequence_;}
}
bool GimbalChassisComm::decode(uint32_t id,const uint8_t in[8],uint8_t length){
  if(in==nullptr||length!=8)return false;
  ChassisData c;GimbalData g;VelocityData v;uint8_t seq=0;
  if(id==rx_id_){
    if(code_part_==CodePart::Chassis){if(!board_protocol::Unpack(in,g,seq))return false;}
    else {if(!board_protocol::Unpack(in,c,seq))return false;}
    InterruptGuard lock;const uint32_t now=hello_world::tick::GetTickMs();
    const bool accepted=peer_tracker_.accept(seq,now);
    // 即使是重复/逆序帧，停止标志也立即收紧输出；正常帧不能借此刷新时间。
    if(!accepted){
      if(code_part_==CodePart::Chassis){
        if(g.emergency_stop)rx_gimbal_.emergency_stop=true;
        if(!g.remote_online)rx_gimbal_.remote_online=false;
        if(!g.operator_armed)rx_gimbal_.operator_armed=false;
      }
      if(code_part_==CodePart::Gimbal&&c.emergency_stop)rx_chassis_.emergency_stop=true;
      return false;
    }
    if(code_part_==CodePart::Chassis)rx_gimbal_=g;else rx_chassis_=c;
    return true;
  }
  if(code_part_!=CodePart::Chassis||id!=board_protocol::kVelocityId||
      !board_protocol::Unpack(in,v,seq))return false;
  InterruptGuard lock;const uint32_t now=hello_world::tick::GetTickMs();
  // 速度帧的版本独立校验；可先缓存，但只有状态帧也在线时才能交给控制使用。
  if(!velocity_tracker_.accept(seq,now)){
    if(!v.valid||!v.remote_online||v.emergency_stop){
      rx_velocity_.valid=false;rx_velocity_.command={};rx_velocity_.emergency_stop=true;
    }
    return false;
  }
  rx_velocity_=v;return true;
}
bool GimbalChassisComm::isOffline()const{
  InterruptGuard lock;return !peer_tracker_.online(hello_world::tick::GetTickMs());
}
bool GimbalChassisComm::getChassisSnapshot(ChassisSnapshot &out)const{
  {InterruptGuard lock;const uint32_t now=hello_world::tick::GetTickMs();
   out.data=rx_chassis_;out.online=code_part_==CodePart::Gimbal&&peer_tracker_.online(now);
   out.age_ms=peer_tracker_.received?now-peer_tracker_.last_ms:0xFFFFFFFFu;}
  if(!out.online){out.data={};}
  if(out.data.emergency_stop){out.data.target_valid=false;out.data.gimbal_target={};}
  return out.online;
}
bool GimbalChassisComm::getGimbalSnapshot(GimbalSnapshot &out)const{
  {InterruptGuard lock;const uint32_t now=hello_world::tick::GetTickMs();
   out.data=rx_gimbal_;out.online=code_part_==CodePart::Chassis&&peer_tracker_.online(now);
   out.age_ms=peer_tracker_.received?now-peer_tracker_.last_ms:0xFFFFFFFFu;}
  if(!out.online){out.data={};}
  return out.online;
}
bool GimbalChassisComm::getVelocitySnapshot(VelocitySnapshot &out)const{
  bool peer_stopped;
  {InterruptGuard lock;const uint32_t now=hello_world::tick::GetTickMs();
   out.data=rx_velocity_;out.online=code_part_==CodePart::Chassis&&
       peer_tracker_.online(now)&&velocity_tracker_.online(now);
   out.age_ms=velocity_tracker_.received?now-velocity_tracker_.last_ms:0xFFFFFFFFu;
   peer_stopped=rx_gimbal_.emergency_stop;}
  if(!board_protocol::VelocityUsable(out.data,out.online,out.online,peer_stopped)){
    out.data.valid=false;out.data.command={};out.data.emergency_stop=true;
    if(!out.online)out.data.remote_online=false;
  }
  return out.online&&out.data.valid;
}
}  // namespace robot
