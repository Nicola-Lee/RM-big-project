#include "remote_probe.hpp"
#include "ins_comm.hpp"
#include "usart.h"
#include "tick.hpp"
#include "runtime_monitor.hpp"

ChassisRemoteProbe chassis_remote_probe{};
namespace remote_probe {
namespace {
struct Guard{uint32_t saved=__get_PRIMASK();Guard(){__disable_irq();}~Guard(){__set_PRIMASK(saved);}};
std::array<uint8_t,kBufferSize> buffer{};
Parser parser;
uint16_t last_position=0;
uint32_t last_retry_ms=0;
constexpr bool Active(){return kEnabled;}
void Byte(uint8_t value){
 auto &s=chassis_remote_probe;++s.bytes;
 const auto result=parser.push(value);
 if(result==Result::Invalid){++s.rejected_frames;s.last_frame=parser.frame;s.seen_valid=false;}
 if(result!=Result::Valid)return;
 auto *remote=GetRemoteControl();remote->decode(parser.frame.data());
 s.last_frame=parser.frame;s.raw_channels=Channels(parser.frame);
 s.left_switch=uint8_t(remote->rc_l_switch());s.right_switch=uint8_t(remote->rc_r_switch());
 s.left_vertical=remote->rc_lv();s.left_horizontal=remote->rc_lh();
 s.right_vertical=remote->rc_rv();s.right_horizontal=remote->rc_rh();s.wheel=remote->rc_wheel();
 s.last_valid_ms=hello_world::tick::GetTickMs();s.seen_valid=true;++s.valid_frames;
}
void Start(){
 auto &s=chassis_remote_probe;
 s.started=false;s.seen_valid=false;parser={};last_position=0;s.last_dma_position=0;
 const uint32_t errors_before=s.uart_errors;
 const auto status=HAL_UARTEx_ReceiveToIdle_DMA(&huart3,buffer.data(),buffer.size());
 Guard lock;s.start_status=status;
 s.started=status==HAL_OK&&s.uart_errors==errors_before&&huart3.RxState==HAL_UART_STATE_BUSY_RX;
}
struct Source {Frame frame{};uint32_t count=0,ms=0;bool online=false;};
Source SourceSnapshot(){
 Guard lock;const uint32_t now=hello_world::tick::GetTickMs();const auto &s=chassis_remote_probe;
 return {s.last_frame,s.valid_frames,s.last_valid_ms,
   s.started&&s.seen_valid&&huart3.RxState==HAL_UART_STATE_BUSY_RX&&
   uint32_t(now-s.last_valid_ms)<robot::remote_forward::kTimeoutMs};
}
bool sending=false,has_sent=false,has_offline=false;
Source pending{};
unsigned part=0;
uint32_t last_count=0,last_offline_ms=0;
}
void Init(){
 if constexpr(!Active())return;
 auto &s=chassis_remote_probe;if(s.attempted)return;
 s.attempted=true;GetRemoteControl();last_retry_ms=hello_world::tick::GetTickMs();Start();
}
void Poll(){
 if constexpr(!Active())return;
 if(__get_IPSR()!=0||__get_PRIMASK()!=0)return;
 auto &s=chassis_remote_probe;if(!s.attempted)return;
 if(s.started&&huart3.RxState==HAL_UART_STATE_BUSY_RX)return;
 const auto now=hello_world::tick::GetTickMs();if(uint32_t(now-last_retry_ms)<10)return;
 last_retry_ms=now;
 {Guard lock;s.started=false;s.seen_valid=false;}
 if(HAL_UART_AbortReceive(&huart3)!=HAL_OK)return;
 Start();
}
void Forward(){
 if constexpr(!Active())return;
 using namespace robot;const auto source=SourceSnapshot();
 const uint32_t now=hello_world::tick::GetTickMs();
 if(!source.online){
   sending=false;
   if(has_offline&&uint32_t(now-last_offline_ms)<10)return;
   if(!runtime::CanMailboxAvailable(runtime::CanBus::One))return;
   const auto bytes=remote_forward::Encode({},uint8_t(source.count),0,0,false);
   if(runtime::TrySend(runtime::CanBus::One,remote_forward::kFirstId,bytes.data(),false)){
     has_offline=true;last_offline_ms=now;++chassis_remote_probe.forward_offline;
   }
   return;
 }
 has_offline=false;
 if(sending&&uint32_t(now-pending.ms)>=remote_forward::kTimeoutMs)sending=false;
 if(!sending){
   if(has_sent&&last_count==source.count)return;
   pending=source;part=0;sending=true;
 }
 if(!runtime::CanMailboxAvailable(runtime::CanBus::One))return;
 const auto bytes=remote_forward::Encode(pending.frame,uint8_t(pending.count),part,uint32_t(now-pending.ms),true);
 // 这是恢复/急停判断的输入，不依赖电机许可；否则会在启动握手时互相等待。
 if(runtime::TrySend(runtime::CanBus::One,remote_forward::kFirstId+part,bytes.data(),false)){
   ++chassis_remote_probe.forward_parts;
   if(++part==3){last_count=pending.count;has_sent=true;sending=false;++chassis_remote_probe.forward_packets;}
 }
}
void Event(uint16_t position,bool idle){
 auto &s=chassis_remote_probe;
 if(!Active()||!s.started||position==0||position>kBufferSize)return;
 ++s.rx_events;
 if(position>last_position){for(unsigned i=last_position;i<position;++i)Byte(buffer[i]);}
 else if(position<last_position){
   for(unsigned i=last_position;i<kBufferSize;++i)Byte(buffer[i]);
   for(unsigned i=0;i<position;++i)Byte(buffer[i]);
 }
 last_position=position;s.last_dma_position=position;
 if(idle){++s.idle_events;parser.idle();}
}
}
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart,uint16_t size){
 if(uart!=&huart3)return;
 const auto event=HAL_UARTEx_GetRxEventType(uart);
 switch(event){
 case HAL_UART_RXEVENT_HT:case HAL_UART_RXEVENT_TC:case HAL_UART_RXEVENT_IDLE:
   remote_probe::Event(size,event==HAL_UART_RXEVENT_IDLE);break;
 default:break;
 }
}
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart){
 if(uart!=&huart3||!remote_probe::kEnabled)return;
 auto &s=chassis_remote_probe;++s.uart_errors;s.last_uart_error=uart->ErrorCode;
 s.started=false;s.seen_valid=false; // 前台Poll恢复；旧数据不能因重启重新生效
}
