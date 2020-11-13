/*
 * Balance.hpp
 *
 *  Created on: 2020年11月13日
 *      Author: Administrator
 */

#ifndef BALANCE_HPP_
#define BALANCE_HPP_

#include "hitsic_common.h"
#include "inc_stdlib.hpp"

#include "sys_pitmgr.hpp"
#include "sys_extint.hpp"
#include "drv_imu_invensense.hpp"
#include "lib_pidctrl.h"

#include "app_menu.hpp"
#include "sc_ftm.h"

extern inv::mpu6050_t imu_6050;
extern float imu6050_accl[3];//加速度计参数
extern float imu6050_gyro[3];//角速度计参数
extern float imu_filterAngAccl, imu_filterAngGyro;//加速度计算得角速度
float imu_filterCompTgReciprocal; //tg的倒数，详见学长c文件157行
void Balance_MenuInit(menu_list_t *menuList);
void Balance_Init();
void Balance_Angle();

#endif /* BALANCE_HPP_ */


