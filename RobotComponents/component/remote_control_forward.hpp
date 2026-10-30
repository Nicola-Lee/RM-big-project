#ifndef ROBOT_REMOTE_CONTROL_FORWARD_HPP_
#define ROBOT_REMOTE_CONTROL_FORWARD_HPP_
#include <array>
#include <cstdint>

namespace robot::remote_forward {
// 实车已测得遥控接收机接入底盘USART3。正常/调试固件均使用这一物理来源。
inline constexpr bool kReceiverOnChassis=true;
inline constexpr uint32_t kFirstId=0x215,kLastId=0x217;
inline constexpr uint32_t kTimeoutMs=100,kAssemblyMs=20,kAgeUnitMs=8;
inline constexpr uint8_t kVersion=0x40,kOnline=0x20,kStop=0x10,kAgeMask=0x0F;
using Frame=std::array<uint8_t,18>;
using CanFrame=std::array<uint8_t,8>;
constexpr bool IsId(uint32_t id){return id>=kFirstId&&id<=kLastId;}
constexpr std::array<uint16_t,5> Channels(const Frame &d){
 return {uint16_t((d[0]|(d[1]<<8))&0x7FF),
   uint16_t(((d[1]>>3)|(d[2]<<5))&0x7FF),
   uint16_t(((d[2]>>6)|(d[3]<<2)|(d[4]<<10))&0x7FF),
   uint16_t(((d[4]>>1)|(d[5]<<7))&0x7FF),uint16_t(d[16]|(d[17]<<8))};
}
constexpr bool Valid(const Frame &d){
 for(auto c:Channels(d))if(c<364||c>1684)return false;
 return ((d[5]>>6)&3)!=0&&((d[5]>>4)&3)!=0&&d[12]<=1&&d[13]<=1;
}
constexpr bool Stopped(const Frame &d){return ((d[5]>>4)&3)==2;}
constexpr CanFrame Encode(const Frame &frame,uint8_t sequence,unsigned part,
                           uint32_t age_ms,bool online){
 CanFrame out{};out[0]=sequence;
 online=online&&age_ms<kTimeoutMs&&part<3;
 out[1]=kVersion|(online?kOnline:0)|(!online||Stopped(frame)?kStop:0);
 if(online){
   out[1]|=uint8_t((age_ms+kAgeUnitMs-1)/kAgeUnitMs);
   for(unsigned i=0;i<6;++i)out[i+2]=frame[part*6+i];
 }
 return out;
}
enum class Result {Ignored,Partial,Complete,Stopped,Invalid};
struct Receiver {
 Frame frame{};
 uint32_t valid_ms=0,accepted_ms=0,completed=0,rejected=0,duplicates=0;
 bool has_frame=false,has_sequence=false,urgent_stop=true,stop_sequence_seen=false;
 uint8_t last_sequence=0,stop_sequence=0;
 Frame staging{};
 std::array<uint32_t,3> part_ms{},part_age{};
 std::array<bool,3> part_stop{};
 uint8_t mask=0,sequence=0;
 uint32_t assembly_ms=0,stop_ms=0;
 constexpr bool fresh(uint32_t now)const{return has_frame&&uint32_t(now-valid_ms)<kTimeoutMs;}
 constexpr void invalidate(){has_frame=false;urgent_stop=true;mask=0;}
 constexpr Result invalid(){++rejected;invalidate();return Result::Invalid;}
 constexpr Result accept(uint32_t id,const CanFrame &data,uint32_t now){
   if(!IsId(id))return Result::Ignored;
   const unsigned part=id-kFirstId;const uint8_t flags=data[1],seq=data[0];
   if((flags&0xC0)!=kVersion)return invalid();
   const uint32_t age=(flags&kAgeMask)*kAgeUnitMs;
   // 下档标志只会收紧许可；完成一组新的有效帧后才可能解除。
   if(flags&kStop)urgent_stop=true;
   if(!(flags&kOnline)||age>=kTimeoutMs){
     stop_sequence_seen=true;stop_sequence=seq;stop_ms=now;invalidate();return Result::Stopped;
   }
   if(stop_sequence_seen){
     const uint8_t delta=uint8_t(seq-stop_sequence);
     // 离线声明之后，邮箱中滞留的同序号或更旧分帧不能恢复许可。
     if(delta==0||(delta>=128&&uint32_t(now-stop_ms)<kTimeoutMs)){
       ++duplicates;return Result::Ignored;
     }
   }
   if(has_sequence){
     const uint8_t delta=uint8_t(seq-last_sequence);
     if(delta==0){++duplicates;return Result::Ignored;}
     if(delta>=128&&uint32_t(now-accepted_ms)<kTimeoutMs)return Result::Ignored;
   }
   if(mask&&uint32_t(now-assembly_ms)>=kAssemblyMs)mask=0;
   if(mask&&seq!=sequence){
     if(uint8_t(seq-sequence)>=128)return Result::Ignored;
     mask=0;
   }
   if(!mask){sequence=seq;assembly_ms=now;}
   const uint8_t bit=uint8_t(1u<<part);
   if(mask&bit){
     for(unsigned i=0;i<6;++i)if(staging[part*6+i]!=data[i+2])return invalid();
     ++duplicates;return Result::Ignored;
   }
   for(unsigned i=0;i<6;++i)staging[part*6+i]=data[i+2];
   part_ms[part]=now;part_age[part]=age;part_stop[part]=(flags&kStop)!=0;mask|=bit;
   if(mask!=7)return Result::Partial;
   mask=0;
   if(!Valid(staging))return invalid();
   uint32_t oldest_age=0;
   for(unsigned i=0;i<3;++i){
     if(part_stop[i]!=Stopped(staging))return invalid();
     const uint32_t elapsed=uint32_t(now-part_ms[i])+part_age[i];
     if(elapsed>oldest_age)oldest_age=elapsed;
   }
   if(oldest_age>=kTimeoutMs){invalidate();return Result::Stopped;}
   frame=staging;valid_ms=now-oldest_age;accepted_ms=now;
   has_frame=true;has_sequence=true;stop_sequence_seen=false;last_sequence=seq;urgent_stop=Stopped(frame);++completed;
   return Result::Complete;
 }
};
} // namespace robot::remote_forward
#endif
