/*
 * Balance.cpp
 *
 *  Created on: 2020年11月14日
 *      Author: 0
 */

#include "Balance.hpp"
pitMgr_t *balance_angle=nullptr;
pitMgr_t *balance_speed=nullptr;
/*Balance初始化部分*/
void Balance_Init()
{
    balance_angle= pitMgr_t::insert(5U, 4U,Balance_Angle, pitMgr_t::enable);
    assert(balance_angle);
    balance_speed= pitMgr_t::insert(20U, 2U,Balance_Speed, pitMgr_t::enable);
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
        MENU_ListInsert(BalanceMenuList, MENU_ItemConstruct(variType, &Ang_En, "ang.en", 0U,
                       menuItem_data_NoSave | menuItem_data_NoLoad | menuItem_dataExt_HasMinMax));
        MENU_ListInsert(BalanceMenuList, MENU_ItemConstruct(variType, &Spd_En, "spd.en", 0U,
                       menuItem_data_NoSave | menuItem_data_NoLoad | menuItem_dataExt_HasMinMax));
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
        MENU_ListInsert(BalanceMenuList, MENU_ItemConstruct(varfType, &speed_set, "speed_set", 14U,
                menuItem_data_region));
        MENU_ListInsert(BalanceMenuList, MENU_ItemConstruct(varfType, &Speed_Pid.kp, "spd.kp", 15U,
                menuItem_data_region));
        MENU_ListInsert(BalanceMenuList, MENU_ItemConstruct(varfType, &Speed_Pid.ki, "spd.ki", 16U,
                menuItem_data_region));
        MENU_ListInsert(BalanceMenuList, MENU_ItemConstruct(varfType, &Speed_Pid.kd, "spd.kd", 17U,
                menuItem_data_region));
        MENU_ListInsert(BalanceMenuList, MENU_ItemConstruct(varfType, &Speed_pidoutput, "spd.out", 0U,
                menuItem_data_NoSave | menuItem_data_NoLoad));
        MENU_ListInsert(BalanceMenuList, MENU_ItemConstruct(varfType, &speed_L, "speed_L", 0U,
                menuItem_data_region));
        MENU_ListInsert(BalanceMenuList, MENU_ItemConstruct(varfType, &speed_R, "speed_R", 0U,
                menuItem_data_region));

     }
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
/*直立环*/
int32_t Ang_En=0;
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
    if(1 == Ang_En)
    {
        PIDCTRL_ErrUpdate(&Balance_Pid,Angle_set-Angle_filter-Speed_pidoutput);
        Balance_pidoutput=PIDCTRL_CalcPIDGain(&Balance_Pid);
        CTRL_MotorUpdate(Balance_pidoutput,Balance_pidoutput);
    }
    else
    {
        Balance_pidoutput = 0.0f;
    }

}
/*速度环*/
int32_t Spd_En=0;
float speed_L=0.0f;
float speed_R=0.0f;
float speed_avg=0.0f;//两边速度的平均值
pidCtrl_t Speed_Pid =
{
    .kp = 0.0f, .ki = 0.0f, .kd = 0.0f,
    .errCurr = 0.0f, .errIntg = 0.0f, .errDiff = 0.0f, .errPrev = 0.0f,
};
float speed_pidoutput_filter[10]={0.0f};
int filter_count=0;
float sum=0.0;
float Speed_pidoutput=0.0;
float speed_set=0.0;
void Balance_Speed()
{
    speed_L=((float)SCFTM_GetSpeed(ENCO_L_PERIPHERAL));//没写完，需要看车子走一米编码器的读数
    SCFTM_ClearSpeed(ENCO_L_PERIPHERAL);
    speed_R=((float)SCFTM_GetSpeed(ENCO_R_PERIPHERAL));
    SCFTM_ClearSpeed(ENCO_R_PERIPHERAL);
    speed_avg=(speed_L+speed_R)/(2.0f);

    PIDCTRL_ErrUpdate(&Speed_Pid, speed_avg - speed_set);
    /*窗口滤波部分*/
    if(1 == Spd_En)
    {
      speed_pidoutput_filter[filter_count]= PIDCTRL_CalcPIDGain(&Speed_Pid);
      filter_count++;
      if(filter_count==10)
        filter_count==0;
      sum=0.0;
      for(int i=0;i<10;i++)
      {
        sum+=speed_pidoutput_filter[i];
      }
      Speed_pidoutput=sum/10;
    }
    else
    {
        filter_count=0;
        for(int i=0;i<10;i++)
        {
            speed_pidoutput_filter[i]=0.0f;
        }
        Speed_pidoutput=0.0f;
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
