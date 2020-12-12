/*
 * Balance.cpp
 *
 *  Created on: 2020年11月14日
 *      Author: 0
 */

#include "Balance.hpp"
#include "image.h"
pitMgr_t *balance_angle=nullptr;
pitMgr_t *balance_speed=nullptr;
pitMgr_t *balance_dir=nullptr;
/*Balance初始化部分*/
void Balance_Init()
{

    balance_angle= pitMgr_t::insert(5U, 4U,Balance_Angle, pitMgr_t::enable);
    assert(balance_angle);
    balance_speed= pitMgr_t::insert(15U,2U,Balance_Speed, pitMgr_t::enable);
    assert(balance_speed);
    balance_dir=pitMgr_t::insert(5U,3U,Balance_Dir, pitMgr_t::enable);
    assert(balance_dir);
}
void Balance_MenuInit(menu_list_t *menuList)
{
    static menu_list_t *filterMenuList =
                    MENU_ListConstruct("imu6050", 32, menuList);
        assert(filterMenuList);
        MENU_ListInsert(menuList, MENU_ItemConstruct(menuType, filterMenuList, "imu6050", 0, 0));
        {

            MENU_ListInsert(filterMenuList, MENU_ItemConstruct(varfType, &imu6050_accl[0], "accl.x", 0U,
                    menuItem_data_NoSave | menuItem_data_NoLoad));
            MENU_ListInsert(filterMenuList, MENU_ItemConstruct(varfType, &imu6050_accl[1], "accl.y", 0U,
                    menuItem_data_NoSave | menuItem_data_NoLoad));
            MENU_ListInsert(filterMenuList, MENU_ItemConstruct(varfType, &imu6050_accl[2], "accl.z", 0U,
                    menuItem_data_NoSave | menuItem_data_NoLoad));
            MENU_ListInsert(filterMenuList, MENU_ItemConstruct(varfType, &imu6050_gyro[0], "gyro.x", 0U,
                    menuItem_data_NoSave | menuItem_data_NoLoad));
            MENU_ListInsert(filterMenuList, MENU_ItemConstruct(varfType, &imu6050_gyro[1], "gyro.y", 0U,
                    menuItem_data_NoSave | menuItem_data_NoLoad));
            MENU_ListInsert(filterMenuList, MENU_ItemConstruct(varfType, &imu6050_gyro[2], "gyro.z", 0U,
                    menuItem_data_NoSave | menuItem_data_NoLoad));
            MENU_ListInsert(filterMenuList, MENU_ItemConstruct(varfType, &Angle[2], "Angle_filter", 0U,
                                menuItem_data_NoSave | menuItem_data_NoLoad));
        }
     static menu_list_t *BalanceMenuList=MENU_ListConstruct("Balance", 32, menuList);
     assert(BalanceMenuList);
     MENU_ListInsert(menuList, MENU_ItemConstruct(menuType, BalanceMenuList, "Balance", 0, 0));
     {
        MENU_ListInsert(BalanceMenuList, MENU_ItemConstruct(nullType, NULL, "ENB", 0, 0));
        MENU_ListInsert(BalanceMenuList, MENU_ItemConstruct(variType, &ctrl_angCtrlEn[0], "ang.en", 21U,
                menuItem_data_region));
        MENU_ListInsert(BalanceMenuList, MENU_ItemConstruct(variType, &ctrl_spdCtrlEn[0], "spd.en", 22U,
                menuItem_data_region));
        MENU_ListInsert(BalanceMenuList, MENU_ItemConstruct(variType, &ctrl_dirCtrlEn[0], "dir.en", 23U,
                menuItem_data_region));
        MENU_ListInsert(BalanceMenuList, MENU_ItemConstruct(nullType, NULL, "ANG", 0, 0));
        MENU_ListInsert(BalanceMenuList, MENU_ItemConstruct(varfType, &Angle_set, "angSet", 9U,
                menuItem_data_region));
        MENU_ListInsert(BalanceMenuList, MENU_ItemConstruct(varfType, &Balance_Pid.kp, "ang.kp", 10U,
                menuItem_data_region));
        MENU_ListInsert(BalanceMenuList, MENU_ItemConstruct(varfType, &Balance_Pid.ki, "ang.ki", 11U,
                menuItem_data_region));
        MENU_ListInsert(BalanceMenuList, MENU_ItemConstruct(varfType, &Balance_Pid.kd, "ang.kd", 12U,
                menuItem_data_region));
        MENU_ListInsert(BalanceMenuList, MENU_ItemConstruct(varfType, &Balance_pidoutput, "ang.out", 0U,
                menuItem_data_NoSave | menuItem_data_NoLoad));
        MENU_ListInsert(BalanceMenuList, MENU_ItemConstruct(nullType, NULL, "SPD", 0, 0));
        MENU_ListInsert(BalanceMenuList, MENU_ItemConstruct(varfType, &speed_L, "speed_L", 0U,
                        menuItem_data_NoSave | menuItem_data_NoLoad));
        MENU_ListInsert(BalanceMenuList, MENU_ItemConstruct(varfType, &speed_R, "speed_R", 0U,
                        menuItem_data_NoSave | menuItem_data_NoLoad));
        MENU_ListInsert(BalanceMenuList, MENU_ItemConstruct(varfType, &Speed_Pid.kp, "spd.kp", 13U,
                        menuItem_data_region));
        MENU_ListInsert(BalanceMenuList, MENU_ItemConstruct(varfType, &Speed_Pid.ki, "spd.ki", 14U,
                        menuItem_data_region));
        MENU_ListInsert(BalanceMenuList, MENU_ItemConstruct(varfType, &Speed_Pid.kd, "spd.kd", 15U,
                        menuItem_data_region));
        MENU_ListInsert(BalanceMenuList, MENU_ItemConstruct(varfType, &speed_set, "spd_set", 16U,
                                menuItem_data_region));
        MENU_ListInsert(BalanceMenuList, MENU_ItemConstruct(varfType, &Speed_pidoutput, "spd.out", 0U,
                        menuItem_data_NoSave | menuItem_data_NoLoad));
        MENU_ListInsert(BalanceMenuList, MENU_ItemConstruct(nullType, NULL, "DIR", 0, 0));
        MENU_ListInsert(BalanceMenuList, MENU_ItemConstruct(varfType, &kp_1, "kp_1", 17U,
                                menuItem_data_region));
        MENU_ListInsert(BalanceMenuList, MENU_ItemConstruct(varfType, &Dir_Pid.kp, "dir.kp", 18U,
                                menuItem_data_region));
        MENU_ListInsert(BalanceMenuList, MENU_ItemConstruct(varfType, &Dir_Pid.ki, "dir.ki", 19U,
                                menuItem_data_region));
        MENU_ListInsert(BalanceMenuList, MENU_ItemConstruct(varfType, &Dir_Pid.kd, "dir.kd", 20U,
                                menuItem_data_region));
        MENU_ListInsert(BalanceMenuList, MENU_ItemConstruct(varfType, &threshold, "11", 21U,
                                menuItem_data_region));
        MENU_ListInsert(BalanceMenuList, MENU_ItemConstruct(varfType, &Dir_pidoutput, "dir.out", 0U,
                menuItem_data_NoSave | menuItem_data_NoLoad));
        MENU_ListInsert(BalanceMenuList, MENU_ItemConstruct(varfType, &mid_err, "mid_err", 0U,
                        menuItem_data_NoSave | menuItem_data_NoLoad));
//        MENU_ListInsert(BalanceMenuList, MENU_ItemConstruct(varfType, &stop_cishu, "22", 0U,
//                        menuItem_data_NoSave | menuItem_data_NoLoad));
     }
//     static menu_list_t *ImageMenuList=MENU_ListConstruct("Image", 32, menuList);
//          assert(ImageMenuList);
//          MENU_ListInsert(menuList, MENU_ItemConstruct(menuType, ImageMenuList, "Image", 0, 0));
//                  {
//
//                      MENU_ListInsert(ImageMenuList, MENU_ItemConstruct(varfType,&threshold, "threshold", 0U,
//                              menuItem_data_NoSave | menuItem_data_NoLoad));
//                  }
}
float imu6050_accl[3] = {0.0f, 0.0f, 0.0f};
float imu6050_gyro[3] = {0.0f, 0.0f, 0.0f};
float Angle[3]={0.0f,0.0f,0.0f};
/*滤波部分*/
float &gyro_y=imu6050_gyro[1];//利用y轴角速度积分计算角度
float &accl_x=imu6050_accl[0];//利用x轴加速度反三角计算角度
float Angle_def=0.0f;
float Angle_accl=0.0f;
float Angle_gyro=0.0f;
float Angle_filter=0.0f;
void AngleFilter_Init()
{
    const uint32_t sampletime=1000;
    float intergration = 0.0f;
    for(uint32_t i = 0; i < sampletime; i++)
    {
        imu_6050.ReadSensorBlocking();
        imu_6050.Convert(&imu6050_accl[0], &imu6050_accl[1], &imu6050_accl[2], &imu6050_gyro[0], &imu6050_gyro[1], &imu6050_gyro[2]);
        AngleFilter_update(0);
        intergration+=Angle_accl;
        SDK_DelayAtLeastUs(1000,CLOCK_GetFreq(kCLOCK_CoreSysClk));
    }
    Angle_def=intergration/((float)sampletime);
    Angle_gyro=Angle_def;
    Angle_filter=Angle_def;
}

void AngleFilter_update(uint32_t updatetime_ms)
{
    float accl_x_t=accl_x;//临时变量，用于限幅和计算
    //float dT = ((float)updatetime_ms) * (0.001f);
    if(accl_x_t>9.70f)
    {
        accl_x_t=9.70f;
    }
    else if(accl_x_t<-9.70f)
    {
        accl_x_t=-9.70f;
    }//限幅

    Angle_accl=asin(accl_x_t/9.80f)*(180.0f)/(3.1415926f);//利用加速度计算角度
    Angle_gyro+=gyro_y*((float)updatetime_ms) * (0.001f);//角速度积分
    Angle_filter+=(gyro_y+((-Angle_filter+Angle_accl))*(3.1415926f))*((float)updatetime_ms) * (0.001f);//滤波融合
    Angle[0]=Angle_accl;
    Angle[1]=Angle_gyro;
    Angle[2]=Angle_filter;
}
/*直立环部分*/
int32_t ctrl_angCtrlEn[3] = {0, 0, 1}; ///< 直立环使能
pidCtrl_t Balance_Pid =
{
    .kp = 0.0f, .ki = 0.0f, .kd = 0.0f,
    .errCurr = 0.0f, .errIntg = 0.0f, .errDiff = 0.0f, .errPrev = 0.0f,
};
float Angle_set=-59.0;//机械零点
float Balance_pidoutput = 0.0f;
void Balance_Angle()
{
    if(kStatus_Success == imu_6050.ReadSensorBlocking())
    {
        imu_6050.Convert(&imu6050_accl[0], &imu6050_accl[1], &imu6050_accl[2], &imu6050_gyro[0], &imu6050_gyro[1], &imu6050_gyro[2]);
        AngleFilter_update(5);
    }
    if(1 == ctrl_angCtrlEn[0])
    {
       PIDCTRL_ErrUpdate(&Balance_Pid,Angle_set-Angle_filter-Speed_pidoutput);
       Balance_pidoutput=PIDCTRL_CalcPIDGain(&Balance_Pid);
    }
    else
    {
        Balance_pidoutput=0.0f;
    }
    CTRL_MotorUpdate(Balance_pidoutput+Dir_pidoutput,Balance_pidoutput-Dir_pidoutput);
}
/*速度环*/
int32_t ctrl_spdCtrlEn[3] = {0, 0, 1}; ///< 速度环使能
float speed_set=0.0f;
float speed_L=0.0f;
float speed_R=0.0f;
float speed_avg=0.0f;//两边速度的平均值
float Speed_pidoutput=0.0;
float speed_pidoutput_filter[10]={0,0,0,0,0,0,0,0,0,0};
int filter_count=0;
float sum=0.0f;
pidCtrl_t Speed_Pid =
{
    .kp = 0.0f, .ki = 0.0f, .kd = 0.0f,
    .errCurr = 0.0f, .errIntg = 0.0f, .errDiff = 0.0f, .errPrev = 0.0f,
};

void Balance_Speed()
{
        speed_R=-((float)SCFTM_GetSpeed(ENCO_L_PERIPHERAL))/(5729.0*0.015f);
        SCFTM_ClearSpeed(ENCO_L_PERIPHERAL);
        speed_L=((float)SCFTM_GetSpeed(ENCO_R_PERIPHERAL))/(5729.0*0.015f);
        SCFTM_ClearSpeed(ENCO_R_PERIPHERAL);
        speed_avg=(speed_L+speed_R)/(2.0f);
        PIDCTRL_ErrUpdate(&Speed_Pid, speed_avg - speed_set);
        Speed_pidoutput=PIDCTRL_CalcPIDGain(&Speed_Pid);
        if(1 == ctrl_spdCtrlEn[0])
        {
             /*窗口滤波部分*/
            speed_pidoutput_filter[filter_count]= PIDCTRL_CalcPIDGain(&Speed_Pid);
            filter_count++;
            if(filter_count==10)
               filter_count=0;
            sum=0.0;
            for(int i=0;i<10;i++)
            {
                sum+=speed_pidoutput_filter[i];
            }
            Speed_pidoutput=sum/10;
        }
        else
        {
            for(int i=0;i<10;i++)
            {
                speed_pidoutput_filter[i]=0.0f;
            }
            filter_count=0;
            sum=0.0;
            Speed_pidoutput=0.0f;
        }

}
/*转向环*/
int32_t ctrl_dirCtrlEn[3] = {0, 0, 1}; ///< 转向环使能
pidCtrl_t Dir_Pid =
{
    .kp = 0.0f, .ki = 0.0f, .kd = 0.0f,
    .errCurr = 0.0f, .errIntg = 0.0f, .errDiff = 0.0f, .errPrev = 0.0f,
};
float kp_1=0.0f;
float mid_err=0.0f;
float Dir_pidoutput=0.0f;
float w_dir=0.0f;
void Balance_Dir()
{
    mid_err=mid_line[60]-94;
    /*中线偏差限幅*/
    if(mid_err>90)
    {
        mid_err=90;
    }
    else if(mid_err<-90)
    {
        mid_err=-90;
    }
    if(1 == ctrl_dirCtrlEn[0])
    {
        if(imu6050_gyro[2]>0)
            w_dir=sqrt(imu6050_gyro[2]*imu6050_gyro[2]+imu6050_gyro[0]*imu6050_gyro[0])*(3.1415926f)/(180.0f);
        else
            w_dir=-sqrt(imu6050_gyro[2]*imu6050_gyro[2]+imu6050_gyro[0]*imu6050_gyro[0])*(3.1415926f)/(180.0f);
        PIDCTRL_ErrUpdate(&Dir_Pid, kp_1*mid_err*speed_avg-w_dir);
        Dir_pidoutput = PIDCTRL_CalcPIDGain(&Dir_Pid);
        if(Dir_pidoutput>30.0f)
        {
            Dir_pidoutput=30.0f;
        }
        else if(Dir_pidoutput<-30.0f)
        {
            Dir_pidoutput=-30.0f;
        }
    }
    else
    {
        Dir_pidoutput=0.0f;
    }
}
void CTRL_MotorUpdate(float motorL, float motorR)
{
    /** 左电机满载 **/
    if(motorL > 100.0f)
    {
        motorR -= (motorL - 100.0f);
        motorL = 100.0f;
    }
    if(motorL < -100.0f)
    {
        motorR -= (motorL + 100.0f);
        motorL = -100.0f;
    }
    /** 右电机满载 **/
    if(motorR > 100.0f)
    {
        motorL -= (motorL - 100.0f);
        motorR = 100.0f;
    }
    if(motorR < -100.0f)
    {
        motorL -= (motorL + 100.0f);
        motorR = -100.0f;
    }
    /** 反转保护 **/
    if(motorL < 0.0f && motorR > 0.0f)
    {
        motorL = 0.0f;
    }
    if(motorL > 0.0f && motorR < 0.0f)
    {
        motorR = 0.0f;
    }
    if(motorL == 0.0f)
    {
        motorR = 0.0f;
    }
    if(motorR == 0.0f)
    {
        motorL = 0.0f;
    }
    if(motorL > 0)
    {
        SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_0, 20000U, motorL);
        SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_1, 20000U, 0.0f);
    }
    else
    {
        SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_0, 20000U, 0.0f);
        SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_1, 20000U, -motorL);
    }

    if(motorR > 0)
    {
        SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_2, 20000U, motorR);
        SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_3, 20000U, 0.0f);
    }
    else
    {
        SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_2, 20000U, 0.0f);
        SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_3, 20000U, -motorR);
    }
}

