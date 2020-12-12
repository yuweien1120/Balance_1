/*
 * image.h
 *
 *  Created on: 2020年11月16日
 *      Author: 0
 */

#ifndef IMAGE_H_
#define IMAGE_H_
#include <math.h>
#include "image.h"
#include "inc_stdlib.hpp"

#define MISS 255
#define CAMERA_H  120                            //图片高度
#define CAMERA_W  188                            //图片宽度
#define FAR_LINE 1//图像处理上边界
#define NEAR_LINE 113//图像处理下边界
#define LEFT_SIDE 0//图像处理左边界
#define RIGHT_SIDE 187//图像处理右边界
#define MISS 255
#define white_num_MAX 10//每行最多允许白条数

/////////////////////////////
#define black 0
#define white 1
#define blue  2
#define green 3
#define red   4
#define gray  5
#define purple 6
///////////////////////////

extern uint8_t IMG[CAMERA_H][CAMERA_W];//二值化后图像数组
extern uint8_t image_Buffer_0[CAMERA_H][CAMERA_W];
extern uint8_t* fullBuffer;//指向灰度图的首地址
extern uint8_t mid_line[CAMERA_H];
extern uint8_t mid_line_last[CAMERA_H];
extern uint8_t left_line[CAMERA_H], right_line[CAMERA_H];//赛道的左右边界
extern uint8_t threshold;
extern uint8_t stop_cishu;//斑马线计次数
extern uint8_t stop_num[2];
void head_clear(void);
void THRE(uint8_t* fullBuffer);
int find_f(int a);
void search_white_range();
void find_all_connect();
void find_road();
uint8_t find_continue(uint8_t i_start, uint8_t j_start);
void ordinary_two_line(void);
void image_main();
void get_mid_line(void);

void my_memset(uint8_t* ptr, uint8_t num, uint8_t size);
int Abs(int num);
void find_rightdown_point(int start_point, int end_point);//寻找右下拐点
void find_leftdown_point(int start_point, int end_point);//寻找左下拐点
void find_rightup_point(int start_point, int end_point);
void find_leftup_point(int start_point, int end_point);
void regression(int type, int startline, int endline);
int judge(void);//判断此次中线是否符合要求
void banmaxian(void);


#endif /* IMAGE_H_ */
