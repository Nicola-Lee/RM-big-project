#ifndef CHASSIS_REMOTE_PROBE_HPP_
#define CHASSIS_REMOTE_PROBE_HPP_
#include <array>
#include <cstdint>
#include "remote_control_forward.hpp"

namespace remote_probe {
// 保留原诊断名称供Ozone使用；现在是底盘物理遥控接收与转发入口。
inline constexpr bool kEnabled=robot::remote_forward::kReceiverOnChassis;
inline constexpr uint16_t kBufferSize=128;
inline constexpr unsigned kFrameSize=18;
using Frame=robot::remote_forward::Frame;
constexpr auto Channels(const Frame &f){return robot::remote_forward::Channels(f);}
constexpr bool Valid(const Frame &f){return robot::remote_forward::Valid(f);}
enum class Result {Ignored,Partial,Valid,Invalid};
struct Parser {
 Frame frame{};
 uint16_t length=0;
 bool aligned=false;
 constexpr Result push(uint8_t value){
   if(!aligned)return Result::Ignored;
   frame[length++]=value;
   if(length<kFrameSize)return Result::Partial;
   length=0;
   if(Valid(frame))return Result::Valid;
   aligned=false;return Result::Invalid;
 }
 constexpr void idle(){length=0;aligned=true;}
};
void Init();
void Poll(); // 原有主循环中的WatchdogPoll调用，UART恢复只在前台执行
void Forward(); // 1ms通信入口，每次最多提交一个CAN片段，不等待邮箱
}

// 使用简单全局名称，便于Ozone查找。遥控输入经CAN到云台，底盘不直接生成运动许可。
struct ChassisRemoteProbe {
 bool attempted=false,started=false,seen_valid=false;
 int32_t start_status=-1;
 uint32_t rx_events=0,idle_events=0,bytes=0;
 uint32_t valid_frames=0,rejected_frames=0;
 uint32_t uart_errors=0,last_uart_error=0,last_valid_ms=0;
 uint32_t forward_parts=0,forward_packets=0,forward_offline=0;
 uint16_t last_dma_position=0;
 std::array<uint16_t,5> raw_channels{}; // RH,RV,LH,LV,拨轮
 uint8_t left_switch=0,right_switch=0;
 float left_vertical=0,left_horizontal=0,right_vertical=0,right_horizontal=0,wheel=0;
 remote_probe::Frame last_frame{};
};
extern ChassisRemoteProbe chassis_remote_probe;
#endif
