/**
 *******************************************************************************
 * @file      :ins_chassis_iksolver.cpp
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
#include "ins_chassis_iksolver.hpp"
#include "ins_robot.hpp"

robot::Chassis *GetChassisIkSolver() {
  return GetChassis();
}
