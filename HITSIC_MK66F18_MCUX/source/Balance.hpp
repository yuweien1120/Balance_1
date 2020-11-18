/*
 * Balance.h
 *
 *  Created on: 2020年11月14日
 *      Author: 0
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
extern float imu6050_accl[3];
extern float imu6050_gyro[3];
/*初始化部分*/
void Balance_MenuInit(menu_list_t *menuList);
void Balance_Init();
/*角度融合*/
extern float Angle_def;//参考初始角度
extern float Angle_accl;
extern float Angle_gyro;
extern float Angle_filter;
extern float Angle[3];//存放加速度计计算角度，角速度计计算角度，滤波融合角度
void AngleFilter_Init();
void AngleFilter_update(uint32_t updatetime_ms);
/*直立环*/
extern float Balance_pidoutput;
extern float Angle_set;
extern pidCtrl_t Balance_Pid;
void Balance_Angle();
void CTRL_MotorUpdate(float motorL, float motorR);
/*速度环*/
void Balance_Speed();
extern float speed[2];
#endif /* BALANCE_HPP_ */
