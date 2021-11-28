//#ifndef _motor_h
//#define _motor_h

//#include "headfile.h"
////左右电机直接输出占空比
//extern int32   Left_MOTOR_Duty;
//extern int32   Right_MOTOR_Duty;
//extern int32   Speed_motor_duty;                //和速度有关的占空比

//void motor_init(void);
//void motor_pid(int16 expect_speed);
//void motor_control(int32 duty_l, int32 duty_r);
//    
//#endif


/*
 * @Description: 电机驱动头文件
 * @Version: v1.0
 * @Autor: 郑有才
 * @Date: 2021-11-28 09:34:50
 * @LastEditors: 郑有才
 * @LastEditTime: 2021-11-28 10:11:36
 */
#ifndef _MOTOR_h
#define _MOTOR_h

#define DIR_L           A0
#define DIR_R           A2

#define PWM_TIM         TIM_5
#define PWM_L           TIM_5_CH2_A01
#define PWM_R           TIM_5_CH4_A03

#define MOTOR_HZ        10000           //10KHZ

extern int32    Left_MOTOR_Duty;
extern int32    Right_MOTOR_Duty;
extern int32   Left_MOTOR_Really_Speed;
extern int32   Right_MOTOR_Really_Speed;
extern int32   Left_MOTOR_Really_Speed_Last;
extern int32   Right_MOTOR_Really_Speed_Last;

void motor_init();
void motor_control(int32 left, int32 right);
void motor_export();
int32 range_protect(int32 duty, int32 min, int32 max);

#endif
