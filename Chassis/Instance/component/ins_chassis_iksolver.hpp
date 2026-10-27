/**
 *******************************************************************************
 * @file      :ins_chassis_iksolver.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CHASSIS_INSTANCE_INS_CHASSIS_IKSOLVER_HPP_
#define CHASSIS_INSTANCE_INS_CHASSIS_IKSOLVER_HPP_

/* Includes ------------------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
namespace robot { class Chassis; }
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
// 兼容原骨架入口：逆解已合并进 robot::Chassis，返回与 GetChassis() 相同的对象。
robot::Chassis *GetChassisIkSolver();

#endif /* CHASSIS_INSTANCE_INS_CHASSIS_IKSOLVER_HPP_ */
