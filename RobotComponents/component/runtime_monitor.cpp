#include "runtime_monitor.hpp"
#include "system.hpp"
#include "tick.hpp"
#include "iwdg.h"
#include "can.h"
#include <initializer_list>

namespace robot::runtime {
namespace {
struct Guard {
  uint32_t saved=__get_PRIMASK();
  Guard(){__disable_irq();}
  ~Guard(){__set_PRIMASK(saved);}
};
WatchdogPolicy watchdog;
std::array<CanTracker,2> buses{};
Fatal fatal_error=Fatal::None;
bool initialized=false;
bool control_permitted=false;
unsigned Index(CanBus bus){return static_cast<unsigned>(bus);}
CAN_HandleTypeDef *Handle(CanBus bus){return bus==CanBus::One?&hcan1:&hcan2;}
uint8_t Pending(CAN_HandleTypeDef *h){
  uint8_t mask=0;
  if(HAL_CAN_IsTxMessagePending(h,CAN_TX_MAILBOX0))mask|=1;
  if(HAL_CAN_IsTxMessagePending(h,CAN_TX_MAILBOX1))mask|=2;
  if(HAL_CAN_IsTxMessagePending(h,CAN_TX_MAILBOX2))mask|=4;
  return mask;
}
void Sample(CanBus bus,uint32_t now){
  auto *h=Handle(bus);const uint32_t esr=h->Instance->ESR;
  buses[Index(bus)].sample(HAL_CAN_GetTxMailboxesFreeLevel(h)==0,
      (esr&CAN_ESR_BOFF)!=0,(esr&CAN_ESR_EPVF)!=0,Pending(h),now);
}
bool MotionAllowed(){return initialized&&fatal_error==Fatal::None&&!buses[0].diagnostic.fault&&!buses[1].diagnostic.fault;}
}
void Initialize(){
  Guard lock;if(initialized)return;
  watchdog={};buses={};fatal_error=Fatal::None;control_permitted=false;initialized=true;
}
void Mark(Task task){
  Guard lock;if(initialized)watchdog.mark(task,hello_world::tick::GetTickMs());
}
void SetFatal(Fatal reason){
  Guard lock;if(fatal_error==Fatal::None)fatal_error=reason;
  if(reason!=Fatal::None)SetControlPermit(false);
}
void ServiceWatchdog(bool imu_expected){
  // 定时中断即使仍活着，也不能替卡死的前台刷新IWDG。
  if(__get_IPSR()!=0||__get_PRIMASK()!=0)return;
  Guard lock;if(!initialized)return;
  const uint32_t now=hello_world::tick::GetTickMs();watchdog.mark(Task::Foreground,now);
  if(!watchdog.ready(now,imu_expected,fatal_error!=Fatal::None))return;
  if(HAL_IWDG_Refresh(&hiwdg)==HAL_OK)watchdog.commit(now);
  else SetFatal(Fatal::WatchdogHardware);
}
Snapshot GetSnapshot(){
  Guard lock;Snapshot s{};s.watchdog=watchdog;s.fatal=fatal_error;s.initialized=initialized;s.control_permitted=control_permitted;
  for(unsigned i=0;i<2;++i)s.can[i]=buses[i].diagnostic;
  return s;
}
void ServiceCan(){
  Guard lock;if(!initialized)return;
  const uint32_t now=hello_world::tick::GetTickMs();
  for(CanBus bus:{CanBus::One,CanBus::Two}){
    Sample(bus,now);auto &state=buses[Index(bus)];auto *h=Handle(bus);
    if(state.abort_due){
      const uint8_t mask=Pending(h);
      // HAL函数仅设置ABRQ请求位，不在此等待取消完成。
      if(mask!=0){
        if(HAL_CAN_AbortTxRequest(h,mask)!=HAL_OK)state.failed(now,HAL_CAN_GetError(h));
        state.abortRequested(now);
      }else state.abort_due=false;
    }
  }
}
void SetControlPermit(bool allowed){
  Guard lock;if(!initialized)return;
  allowed=allowed&&MotionAllowed();
  if(control_permitted&&!allowed){
    const uint32_t now=hello_world::tick::GetTickMs();
    for(CanBus bus:{CanBus::One,CanBus::Two}){
      auto *h=Handle(bus);auto &state=buses[Index(bus)];
      const uint8_t mask=state.controlMask()&Pending(h);
      if(mask!=0){
        if(HAL_CAN_AbortTxRequest(h,mask)!=HAL_OK)state.failed(now,HAL_CAN_GetError(h));
        state.abortRequested(now);
      }
    }
  }
  control_permitted=allowed;
}
bool CanMotionAllowed(){
  Guard lock;if(!initialized)return false;
  const uint32_t now=hello_world::tick::GetTickMs();Sample(CanBus::One,now);Sample(CanBus::Two,now);
  return MotionAllowed();
}
bool CanMailboxAvailable(CanBus bus){
  Guard lock;if(!initialized)return false;
  if(Index(bus)>1){SetFatal(Fatal::InvalidCanArgument);return false;}
  auto *h=Handle(bus);Sample(bus,hello_world::tick::GetTickMs());
  if(HAL_CAN_GetTxMailboxesFreeLevel(h)==0||
     (h->Instance->TSR&(CAN_TSR_RQCP0|CAN_TSR_RQCP1|CAN_TSR_RQCP2))!=0){
    ++buses[Index(bus)].diagnostic.busy_skipped;return false;
  }
  return true;
}
bool TrySend(CanBus bus,uint32_t id,const uint8_t *data,bool requires_control,uint8_t length){
  Guard lock;
  if(!initialized||Index(bus)>1||id>0x7FFu||data==nullptr||length>8){SetFatal(Fatal::InvalidCanArgument);return false;}
  const uint32_t now=hello_world::tick::GetTickMs();Sample(CanBus::One,now);Sample(CanBus::Two,now);
  auto &state=buses[Index(bus)];auto *h=Handle(bus);
  if(requires_control&&(!control_permitted||!MotionAllowed())){++state.diagnostic.blocked_control;return false;}
  if(HAL_CAN_GetTxMailboxesFreeLevel(h)==0){++state.diagnostic.busy_skipped;return false;}
  CAN_TxHeaderTypeDef header{};header.StdId=id;header.IDE=CAN_ID_STD;header.RTR=CAN_RTR_DATA;header.DLC=length;
  // 若上一帧的完成IRQ还未处理，等其清除RQCP后再复用邮箱，避免旧回调覆盖新记录。
  if((h->Instance->TSR&(CAN_TSR_RQCP0|CAN_TSR_RQCP1|CAN_TSR_RQCP2))!=0){
    ++state.diagnostic.busy_skipped;return false;
  }
  uint8_t payload[8]{};for(unsigned i=0;i<length;++i)payload[i]=data[i];
  uint32_t mailbox=0;
  // 提交和记录合在同一短临界区，避免完成IRQ抢先于queued记录。
  if(HAL_CAN_AddTxMessage(h,&header,payload,&mailbox)!=HAL_OK){
    state.failed(now,HAL_CAN_GetError(h));HAL_CAN_ResetError(h);return false;
  }
  const unsigned slot=mailbox==CAN_TX_MAILBOX0?0u:(mailbox==CAN_TX_MAILBOX1?1u:2u);
  state.queued(slot,now,requires_control);return true;
}
void OnCanComplete(CanBus bus,unsigned mailbox){
  Guard lock;if(initialized)buses[Index(bus)].completed(mailbox,hello_world::tick::GetTickMs());
}
void OnCanAbort(CanBus bus,unsigned mailbox){Guard lock;if(initialized)buses[Index(bus)].aborted(mailbox);}
void OnCanError(CanBus bus,uint32_t bits){
  Guard lock;if(!initialized)return;
  const uint32_t now=hello_world::tick::GetTickMs();auto &state=buses[Index(bus)];state.error(now,bits);
  constexpr uint32_t tx_errors=HAL_CAN_ERROR_TX_TERR0|HAL_CAN_ERROR_TX_TERR1|HAL_CAN_ERROR_TX_TERR2|
                               HAL_CAN_ERROR_TX_ALST0|HAL_CAN_ERROR_TX_ALST1|HAL_CAN_ERROR_TX_ALST2;
  if(bits&tx_errors)state.transmitError(now);
  Sample(bus,now);
}
}  // namespace robot::runtime
