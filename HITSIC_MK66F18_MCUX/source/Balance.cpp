/*
 * Balance.cpp
 *
 *  Created on: 2020年11月13日
 *      Author: Administrator
 */

#include <Balance.hpp>
pitMgr_t *balance_angle=nullptr;
void Balance_Init()
{
    balance_angle= pitMgr_t::insert(5U, 4U,Balance_Angle, pitMgr_t::enable);
    assert(balance_angle);
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
        }
}
float imu6050_accl[3] = {0.0f, 0.0f, 0.0f};
float imu6050_gyro[3] = {0.0f, 0.0f, 0.0f};
float imu_filterAngAccl=0.0f, imu_filterAngGyro=0.0f;
float imu_filterCompTgReciprocal = 1.0f / 0.8f;
pidCtrl_t ctrl_angFilter =
{
    .kp = 0.0f, .ki = 0.0f, .kd = 0.0f,
    .errCurr = 0.0f, .errIntg = 0.0f, .errDiff = 0.0f, .errPrev = 0.0f,
};
void Balance_Angle()
{
    if(kStatus_Success == imu_6050.ReadSensorBlocking())
    {
        imu_6050.Convert(&imu6050_accl[0], &imu6050_accl[1], &imu6050_accl[2], &imu6050_gyro[0], &imu6050_gyro[1], &imu6050_gyro[2]);
    }
}




