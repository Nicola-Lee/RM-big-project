#ifndef ROBOT_COMPONENTS_RUNTIME_MONITOR_HPP_
#define ROBOT_COMPONENTS_RUNTIME_MONITOR_HPP_
#include <array>
#include <cstdint>

namespace robot::runtime {
// 软件检查周期/阈值；硬件IWDG配置继续使用两板现有的约0.3s/0.6s。
inline constexpr uint32_t kFeedIntervalMs=10;
inline constexpr uint32_t kTaskTimeoutMs=20;
inline constexpr uint32_t kCanStallMs=100;
inline constexpr uint32_t kCanRecoverMs=100;
inline constexpr uint32_t kCanCompletionFreshMs=20;
inline constexpr uint32_t kCanAbortRetryMs=100;
enum class Task:uint8_t {Control,Communication,Foreground,Imu};
enum class CanBus:uint8_t {One,Two};
enum class Fatal:uint8_t {None,MissingObject,Encoder,InvalidCanArgument,WatchdogHardware};
enum class FeedBlock:uint8_t {None,Interval,Control,Communication,Foreground,Imu,FatalError,NotInitialized};
struct Progress {
  uint32_t count=0,last_ms=0;bool seen=false;
  constexpr void mark(uint32_t now){++count;last_ms=now;seen=true;}
};
struct WatchdogPolicy {
  std::array<Progress,4> task{};
  std::array<uint32_t,4> consumed{};
  uint32_t last_feed_ms=0,feed_count=0;
  bool has_fed=false;
  FeedBlock blocked=FeedBlock::NotInitialized;
  constexpr void mark(Task t,uint32_t now){task[static_cast<unsigned>(t)].mark(now);}
  constexpr bool ready(uint32_t now,bool imu_required,bool fatal){
    if(fatal){blocked=FeedBlock::FatalError;return false;}
    if(has_fed&&static_cast<uint32_t>(now-last_feed_ms)<kFeedIntervalMs){blocked=FeedBlock::Interval;return false;}
    for(unsigned i=0;i<4;++i){
      if(i==static_cast<unsigned>(Task::Imu)&&!imu_required)continue;
      const auto &p=task[i];
      if(!p.seen||static_cast<uint32_t>(now-p.last_ms)>=kTaskTimeoutMs||p.count==consumed[i]){
        blocked=i==0?FeedBlock::Control:(i==1?FeedBlock::Communication:(i==2?FeedBlock::Foreground:FeedBlock::Imu));
        return false;
      }
    }
    blocked=FeedBlock::None;return true;
  }
  constexpr void commit(uint32_t now){
    for(unsigned i=0;i<4;++i)consumed[i]=task[i].count;
    last_feed_ms=now;++feed_count;has_fed=true;blocked=FeedBlock::None;
  }
};
struct CanDiagnostics {
  bool mailbox_busy=false,bus_off=false,error_passive=false,fault=false;
  uint32_t queued=0,completed=0,enqueue_failed=0,busy_skipped=0;
  uint32_t error_callbacks=0,transmit_failed=0,aborted=0,abort_requests=0,blocked_control=0;
  uint32_t last_enqueue_ms=0,last_complete_ms=0,last_error_ms=0,last_error_bits=0;
  uint32_t busy_since_ms=0;
  uint8_t pending_mask=0;
};
struct CanTracker {
  CanDiagnostics diagnostic{};
  std::array<uint32_t,3> queued_at{};
  std::array<bool,3> control_frame{};
  bool busy_timing=false,has_completion=false,recover_timing=false;
  bool failure_timing=false,abort_due=false,abort_started=false;
  uint32_t failure_since=0,recover_since=0,last_abort_ms=0;
  constexpr void queued(unsigned mailbox,uint32_t now,bool control=false){
    if(mailbox>=3)return;
    diagnostic.pending_mask|=static_cast<uint8_t>(1u<<mailbox);queued_at[mailbox]=now;control_frame[mailbox]=control;
    ++diagnostic.queued;diagnostic.last_enqueue_ms=now;
  }
  constexpr void completed(unsigned mailbox,uint32_t now){
    if(mailbox<3)diagnostic.pending_mask&=static_cast<uint8_t>(~(1u<<mailbox));
    ++diagnostic.completed;diagnostic.last_complete_ms=now;has_completion=true;failure_timing=false;
  }
  constexpr void aborted(unsigned mailbox){
    if(mailbox<3)diagnostic.pending_mask&=static_cast<uint8_t>(~(1u<<mailbox));
    ++diagnostic.aborted;
  }
  constexpr void failed(uint32_t now,uint32_t error){
    ++diagnostic.enqueue_failed;diagnostic.last_error_ms=now;diagnostic.last_error_bits=error;
    if(!failure_timing){failure_timing=true;failure_since=now;}
  }
  constexpr void transmitError(uint32_t now){
    ++diagnostic.transmit_failed;
    if(!failure_timing){failure_timing=true;failure_since=now;}
  }
  constexpr void error(uint32_t now,uint32_t bits){
    ++diagnostic.error_callbacks;diagnostic.last_error_ms=now;diagnostic.last_error_bits=bits;
  }
  constexpr void sample(bool busy,bool bus_off,bool passive,uint8_t hardware_pending,uint32_t now){
    diagnostic.mailbox_busy=busy;diagnostic.bus_off=bus_off;diagnostic.error_passive=passive;
    // 完成/错误IRQ可能正待处理，硬件已释放的邮箱不再用于判定旧报文滞留。
    diagnostic.pending_mask&=hardware_pending;
    if(busy){if(!busy_timing){busy_timing=true;diagnostic.busy_since_ms=now;}}
    else busy_timing=false;
    bool stalled=busy_timing&&static_cast<uint32_t>(now-diagnostic.busy_since_ms)>=kCanStallMs;
    for(unsigned i=0;i<3;++i)
      if((diagnostic.pending_mask&(1u<<i))&&static_cast<uint32_t>(now-queued_at[i])>=kCanStallMs)stalled=true;
    const bool repeated_fail=failure_timing&&static_cast<uint32_t>(now-failure_since)>=kCanStallMs;
    const bool trigger=bus_off||passive||stalled||repeated_fail;
    if(trigger){
      if(!diagnostic.fault)abort_due=true;
      diagnostic.fault=true;recover_timing=false;
    }else if(diagnostic.fault){
      const bool healthy=has_completion&&static_cast<uint32_t>(now-diagnostic.last_complete_ms)<kCanCompletionFreshMs&&!busy;
      if(!healthy)recover_timing=false;
      else{
        if(!recover_timing){recover_timing=true;recover_since=now;}
        if(static_cast<uint32_t>(now-recover_since)>=kCanRecoverMs){diagnostic.fault=false;recover_timing=false;abort_due=false;}
      }
    }
    if(diagnostic.fault&&hardware_pending!=0&&(!abort_started||static_cast<uint32_t>(now-last_abort_ms)>=kCanAbortRetryMs))abort_due=true;
  }
  constexpr uint8_t controlMask()const{
    uint8_t mask=0;for(unsigned i=0;i<3;++i)if(control_frame[i]&&(diagnostic.pending_mask&(1u<<i)))mask|=static_cast<uint8_t>(1u<<i);
    return mask;
  }
  constexpr void abortRequested(uint32_t now){
    ++diagnostic.abort_requests;last_abort_ms=now;abort_started=true;abort_due=false;
  }
};
struct Snapshot {
  WatchdogPolicy watchdog{};
  std::array<CanDiagnostics,2> can{};
  Fatal fatal=Fatal::None;
  bool initialized=false;
  bool control_permitted=false;
};
// 以下接口由每块板编译各自独立的实例。所有快照/回调更新均保留原中断状态。
void Initialize();
void Mark(Task task);
void SetFatal(Fatal reason);
void ServiceWatchdog(bool imu_sampling_expected); // 唯一硬件喂狗入口，仅主循环调用
Snapshot GetSnapshot();
void ServiceCan();                       // 非阻塞采样/请求取消旧报文
void SetControlPermit(bool allowed); // 撤销许可时取消仍在邮箱中的旧运动帧
bool CanMotionAllowed();                 // 故障只禁止运动，不直接禁止喂狗
bool CanMailboxAvailable(CanBus bus);     // 忙时记录本次跳过，不等待
bool TrySend(CanBus bus,uint32_t id,const uint8_t *data,bool requires_control=false,uint8_t length=8);
void OnCanComplete(CanBus bus,unsigned mailbox);
void OnCanAbort(CanBus bus,unsigned mailbox);
void OnCanError(CanBus bus,uint32_t bits);
}  // namespace robot::runtime
#endif
