/*
 * @Author: Nicola Lee nlee15428@gmail.com
 * @Date: 2026-09-19 15:52:24
 * @LastEditors: Nicola Lee nlee15428@gmail.com
 * @LastEditTime: 2026-10-23 21:16:03
 * @FilePath: \omni_skeleton\Gimbal\Instance\component\ins_comm.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
 *******************************************************************************
 * @file      :ins_comm.cpp
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
/* Includes ------------------------------------------------------------------*/
#include "ins_comm.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
hello_world::remote_control::DT7 *GetRemoteControl() {
  /* TODO【新人任务】:
   * 返回遥控器解码对象（DT7）单例。
   *
   * 你要做：
   *   1. 静态定义一个 hello_world::remote_control::DT7 对象并返回其地址。
   *   2. （可选）如果你自建了其它需要共享的通信对象，也在此暴露访问器。
   *
   * 约束 / 提示：
   *   - DT7只负责解码18字节DBUS；物理UART接收和CAN转发在底盘完成。
   *   - 云台解码入口：Task/component/comm_task.cpp 的 ReceiveForwardedRemote。
   *     CAN1三片完整重组并校验后，才调用此对象的decode。
   */
  static hello_world::remote_control::DT7 remote_control;
  return &remote_control;
}

/* Private function definitions ----------------------------------------------*/
