/**
 *******************************************************************************
 * @file      :robot.cpp
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
#include "robot.hpp"
#include "robot_limits.hpp"
#include "imu.hpp"
#include "motor_DaMiao.hpp"
#include "comm_task.hpp"
#include "tick.hpp"
#include "runtime_monitor.hpp"
namespace robot {
namespace {
bool Positive(float v){return control::Finite(v)&&v>0.0f&&v<=board_protocol::kMaxVelocity;}
bool Sign(float v){return v==1.0f||v==-1.0f;}
bool ChannelsValid(const RemoteControlSnapshot &r){
  return control::Finite(r.rc_lv)&&control::Finite(r.rc_lh)&&control::Finite(r.rc_rh)&&
         control::Finite(r.rc_rv)&&control::Finite(r.rc_wheel)&&
         control::Abs(r.rc_lv)<=1&&control::Abs(r.rc_lh)<=1&&control::Abs(r.rc_rh)<=1&&
         control::Abs(r.rc_rv)<=1&&control::Abs(r.rc_wheel)<=1;
}
}
bool Robot::init(const GimbalRobotParams &p,Gimbal *gimbal,hello_world::imu::Imu *imu,
                 GimbalChassisComm *comm,hello_world::motor::DaMiao *yaw,
                 hello_world::motor::DaMiao *pitch){
  if(initialized_||gimbal==nullptr||imu==nullptr||comm==nullptr||yaw==nullptr||pitch==nullptr||
     p.ready_stable_ms==0||p.recovery_timeout_ms<=p.ready_stable_ms||
     p.recovery_timeout_ms>0x7FFFFFFFu||p.neutral_stable_ms>0x7FFFFFFFu)return false;
  params_=p;gimbal_=gimbal;imu_=imu;comm_=comm;yaw_=yaw;pitch_=pitch;
  machine_.configure(p.ready_stable_ms,p.recovery_timeout_ms);initialized_=true;
  resetControl();publish(false,false,ChassisMode::Disabled);return true;
}
bool Robot::paramsReady()const{
  if(!initialized_||!params_.switch_mapping_confirmed||!gimbal_->controlReady()||
     !control::Finite(params_.deadband)||params_.deadband<0||params_.deadband>=1||
     !control::Finite(params_.neutral_threshold)||params_.neutral_threshold<params_.deadband||params_.neutral_threshold>=1||
     !Sign(params_.forward_sign)||!Sign(params_.left_sign)||!Sign(params_.yaw_sign)||
     !Sign(params_.pitch_sign)||!Sign(params_.wheel_sign))return false;
  if(!Positive(params_.max_forward_m_s)||!Positive(params_.max_left_m_s)||
     !Positive(params_.max_manual_body_rate_rad_s)||!Positive(params_.max_spin_rate_rad_s)||
     !Positive(params_.max_yaw_input_rate_rad_s)||!Positive(params_.max_pitch_input_rate_rad_s))return false;
  // 给底盘转动补偿留出目标变化率，不能只按操作手yaw速度设置关节限速。
  // 共享上限同时覆盖跟随模式；手动/小陀螺请求也不能超过底盘允许值。
  return Positive(limits::kMaxBodyRateRadS)&&
         limits::YawRateBudgetValid(limits::kMaxBodyRateRadS,
             params_.max_manual_body_rate_rad_s,params_.max_spin_rate_rad_s,
             params_.max_yaw_input_rate_rad_s,gimbal_->yawTargetRateLimit())&&
         gimbal_->pitchTargetRateLimit()>=params_.max_pitch_input_rate_rad_s;
}
void Robot::resetControl(){
  if(gimbal_!=nullptr)gimbal_->stop();
 neutral_.reset();target_initialized_=false;
  heading_target_=pitch_target_=0;enable_requested_=false;
}
void Robot::publish(bool remote_online,bool operator_armed,ChassisMode mode){
  GimbalChassisComm::GimbalData state{};
  state.life=machine_.state();state.local_ready=machine_.localReady();
  state.remote_online=remote_online;state.operator_armed=operator_armed;
  state.mode=operator_armed?GimbalMode::JointAngle:GimbalMode::Disabled;
  state.emergency_stop=state.life==LifeState::Dead;
  comm_->setLocalGimbal(state);
  diagnostic_.life=machine_.state();diagnostic_.reason=machine_.reason();
  diagnostic_.local_ready=machine_.localReady();diagnostic_.chassis_mode=mode;
}
void Robot::update(float dt){
  if(!initialized_)return;
  const uint32_t now=hello_world::tick::GetTickMs();
  const bool late=updated_&&static_cast<uint32_t>(now-diagnostic_.last_update_ms)>20u;
  diagnostic_.last_update_ms=now;updated_=true;
  RemoteControlSnapshot remote{};const bool remote_online=GetRemoteControlSnapshot(remote)&&ChannelsValid(remote);
  const auto mode=remote_online?control::ModeFromSwitch(remote.rc_l_switch):ChassisMode::Disabled;
  const bool operator_armed=remote_online&&params_.switch_mapping_confirmed&&
      control::OperatorArmed(remote.rc_r_switch)&&mode!=ChassisMode::Disabled;
  hello_world::imu::ImuSnapshot imu{};
  const bool imu_ok=imu_->getSnapshot(imu)&&imu.heading_valid&&control::Finite(imu.heading_yaw);
  GimbalMotorFeedbackSnapshot joint{};GetGimbalMotorFeedbackSnapshot(joint);
  const bool motor_ok=joint.yaw_valid&&joint.pitch_valid;
  GimbalChassisComm::ChassisSnapshot peer{};comm_->getChassisSnapshot(peer);
  const bool neutral=control::NearZero(remote.rc_lv,remote.rc_lh,remote.rc_rh,remote.rc_rv,
                                      remote.rc_wheel,params_.neutral_threshold);
  bool neutral_ok=!params_.require_neutral_on_recovery;
  if(machine_.state()!=LifeState::Dead&&params_.require_neutral_on_recovery)
    neutral_ok=neutral_.update(neutral,now,params_.neutral_stable_ms);
  control::LifeInput in{};in.operator_permitted=operator_armed;
  in.local_ready=imu_ok&&motor_ok&&neutral_ok;
  in.local_reason=!imu_ok?control::StopReason::ImuNotReady:
      (!motor_ok?control::StopReason::MotorNotReady:control::StopReason::NeutralRequired);
  in.peer_ready=peer.online&&peer.data.local_ready;
  in.peer_working=peer.online&&peer.data.life==LifeState::Working;in.peer_stopped=peer.data.emergency_stop;
  if(!remote_online)in.inhibit=control::StopReason::RemoteOffline;
  else if(!operator_armed)in.inhibit=control::StopReason::EmergencyStop;
  else if(!peer.online)in.inhibit=control::StopReason::PeerOffline;
  else if(!runtime::CanMotionAllowed())in.inhibit=control::StopReason::CommunicationFault;
  else if(!paramsReady()||imu_->state()==hello_world::imu::ImuState::Uninitialized)in.inhibit=control::StopReason::Configuration;
  else if(imu_->state()==hello_world::imu::ImuState::Fault)in.inhibit=control::StopReason::ImuNotReady;
  else if(joint.yaw_fault||joint.pitch_fault)in.inhibit=control::StopReason::MotorNotReady;
  else if(late||!control::Finite(dt)||dt<=0||dt>0.02f)in.inhibit=control::StopReason::Timing;
  else in.inhibit=control::StopReason::None;
  const auto previous=machine_.state();machine_.step(in,now);
  if(previous!=machine_.state()){
    // 从恢复进入Working时保留已取得的回中许可，其他迁移清除旧控制历史。
    const bool qualified=neutral_.qualified;resetControl();
    if(machine_.state()==LifeState::Working)neutral_.qualified=qualified;
  }
  diagnostic_.active=false;
  const bool active=machine_.canRun(in.peer_working,in.peer_stopped);
  if(active){
    if(!target_initialized_){
      heading_target_=imu.heading_yaw;
      pitch_target_=control::Clamp(joint.angle.pitch,gimbal_->pitchMin(),gimbal_->pitchMax());
      target_initialized_=true;
    }
    const float yaw_rate=control::Stick(remote.rc_rh,params_.deadband,params_.yaw_sign)*params_.max_yaw_input_rate_rad_s;
    const float pitch_rate=control::Stick(remote.rc_rv,params_.deadband,params_.pitch_sign)*params_.max_pitch_input_rate_rad_s;
    heading_target_=control::HeadingTarget(heading_target_,yaw_rate,dt);
    pitch_target_=control::Clamp(pitch_target_+pitch_rate*dt,gimbal_->pitchMin(),gimbal_->pitchMax());
    GimbalControlInput input{};
    input.target={control::JointYawTarget(joint.angle.yaw,heading_target_,imu.heading_yaw),pitch_target_};
    input.mode=GimbalMode::JointAngle;input.dt_s=dt;input.enabled=true;input.target_valid=true;
    input.yaw_feedback_valid=joint.yaw_valid;input.pitch_feedback_valid=joint.pitch_valid;
    if(!gimbal_->update(input)){
      machine_.abort(control::StopReason::ControlFailed);resetControl();
    }else diagnostic_.active=true;
  }else{gimbal_->stop();target_initialized_=false;}

  action_=diagnostic_.active?control::MotorAction::Control:
      (machine_.state()==LifeState::Dead?control::MotorAction::Disable:control::MotorAction::EnableZero);
  if(action_==control::MotorAction::EnableZero){
    // 零转矩使能不等待对方Working；间隔重试以免每帧覆盖正常MIT控制。
    yaw_->setInput(0);pitch_->setInput(0);
    if(!enable_requested_||static_cast<uint32_t>(now-last_enable_request_ms_)>=20u){
      if(!joint.yaw_valid)yaw_->enable();
      if(!joint.pitch_valid)pitch_->enable();
      last_enable_request_ms_=now;enable_requested_=true;
    }
  }
  publish(remote_online,operator_armed,mode);
  ChassisCommand velocity{};
  if(diagnostic_.active){
    velocity.vx=control::Stick(remote.rc_lv,params_.deadband,params_.forward_sign)*params_.max_forward_m_s;
    velocity.vy=control::Stick(remote.rc_lh,params_.deadband,params_.left_sign)*params_.max_left_m_s;
    const float wheel=control::Stick(remote.rc_wheel,params_.deadband,params_.wheel_sign);
    velocity.w=mode==ChassisMode::Follow?0.0f:wheel*(mode==ChassisMode::Spin?params_.max_spin_rate_rad_s:params_.max_manual_body_rate_rad_s);
  }
  if(!PublishChassisVelocity(velocity,mode,diagnostic_.active)&&diagnostic_.active){
    machine_.abort(control::StopReason::CommandInvalid);resetControl();action_=control::MotorAction::Disable;
    diagnostic_.active=false;publish(remote_online,operator_armed,mode);
  }
}
control::MotorAction Robot::motorAction()const{
  if(!runtime::CanMotionAllowed()||!initialized_||!updated_||static_cast<uint32_t>(hello_world::tick::GetTickMs()-diagnostic_.last_update_ms)>=20u)
    return control::MotorAction::Disable;
  RemoteControlSnapshot remote{};
  if(!GetRemoteControlSnapshot(remote)||!ChannelsValid(remote)||!control::OperatorArmed(remote.rc_r_switch)||
     control::ModeFromSwitch(remote.rc_l_switch)==ChassisMode::Disabled||comm_->isOffline())
    return control::MotorAction::Disable;
  GimbalMotorFeedbackSnapshot joint{};GetGimbalMotorFeedbackSnapshot(joint);
  if(joint.yaw_fault||joint.pitch_fault||imu_->state()==hello_world::imu::ImuState::Fault)
    return control::MotorAction::Disable;
  if(action_!=control::MotorAction::Control)return action_;
  GimbalChassisComm::ChassisSnapshot peer{};hello_world::imu::ImuSnapshot imu{};
  if(!imu_->getSnapshot(imu)||!imu.heading_valid||!joint.yaw_valid||!joint.pitch_valid||
     !comm_->getChassisSnapshot(peer)||peer.data.emergency_stop||!peer.data.local_ready||peer.data.life!=LifeState::Working)
    return control::MotorAction::Disable;
  return action_;
}
}  // namespace robot
