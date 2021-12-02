///*
// *                        _oo0oo_
// *                       o8888888o
// *                       88" . "88
// *                       (| -_- |)
// *                       0\  =  /0
// *                     ___/`---'\___
// *                   .' \\|     |// '.
// *                  / \\|||  :  |||// \
// *                 / _||||| -:- |||||- \
// *                |   | \\\  - /// |   |
// *                | \_|  ''\---/''  |_/ |
// *                \  .-\__  '-'  ___/-. /
// *              ___'. .'  /--.--\  `. .'___
// *           ."" '<  `.___\_<|>_/___.' >' "".
// *          | | :  `- \`.;`\ _ /`;.`/ - ` : | |
// *          \  \ `_.   \_ __\ /__ _/   .-` /  /
// *      =====`-.____`.___ \_____/___.-`___.-'=====
// *                        `=---='
// * 
// * 
// *      ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// * 
// *            佛祖保佑       永不宕机     永无BUG
// */

///*
// * @Description: 
// * @Version: v1.0
// * @Autor: 郑有才
// * @Date: 2021-11-24 14:46:45
// * @LastEditors: 郑有才
// * @LastEditTime: 2021-11-26 16:58:18
// */

//#include "motor.h"

//#define DIR_L				A0
//#define DIR_R				A2

//#define PWM_TIM				TIM_5
//#define PWM_L				TIM_5_CH2_A01
//#define PWM_R				TIM_5_CH4_A03

////左右电机直接输出占空比
//int32   Left_MOTOR_Duty   = 0;
//int32   Right_MOTOR_Duty  = 0;
//int32   Speed_motor_duty  = 0;                //和速度有关的占空比

//void motor_init(void)
//{
//    // 请注意 这里接线时请务必不要接错 A0/A1一组 A2/A3一组 分别控制左右电机
//	gpio_init(DIR_L, GPO, GPIO_HIGH, GPO_PUSH_PULL);								// GPIO 初始化为输出 默认上拉输出高
//	gpio_init(DIR_R, GPO, GPIO_HIGH, GPO_PUSH_PULL);								// GPIO 初始化为输出 默认上拉输出高

//	pwm_init(PWM_TIM, PWM_L, 10000, 0);												// PWM 通道2 初始化频率10KHz 占空比初始为0
//	pwm_init(PWM_TIM, PWM_R, 10000, 0);												// PWM 通道4 初始化频率10KHz 占空比初始为0
//}

//void motor_pid(int16 expect_speed)
//{
//    
//}


//void motor_control(int32 duty_l, int32 duty_r)
//{
//    //对占空比限幅
//	duty_l = limit(duty_l, PWM_DUTY_MAX);
//	duty_r = limit(duty_r, PWM_DUTY_MAX);
//    
//    if(duty_l >= 0)											// 左侧正转
//    {
//        gpio_set(DIR_L, GPIO_HIGH);							// DIR输出高电平
//        pwm_duty_updata(PWM_TIM, PWM_L, duty_l);		    // 计算占空比
//    }
//    else													// 左侧反转
//    {
//        gpio_set(DIR_L, GPIO_LOW);							// DIR输出低电平
//        pwm_duty_updata(PWM_TIM, PWM_L, -duty_l);			// 计算占空比1
//    }
//    
//    if(duty_r >= 0)											// 右侧正转
//    {
//        gpio_set(DIR_R, GPIO_HIGH);							// DIR输出高电平
//        pwm_duty_updata(PWM_TIM, PWM_R, duty_r);			// 计算占空比
//    }
//    else													// 右侧反转
//    {
//        gpio_set(DIR_R, GPIO_LOW);							// DIR输出低电平
//        pwm_duty_updata(PWM_TIM, PWM_R, -duty_r);			// 计算占空比
//    }
//}



/*
 * @Description: 电机底层驱动
 * @Version: v1.0
 * @Autor: 郑有才
 * @Date: 2021-11-28 09:33:28
 * @LastEditors: 郑有才
 * @LastEditTime: 2021-11-28 10:11:00
 */

/*
 *                        _oo0oo_
 *                       o8888888o
 *                       88" . "88
 *                       (| -_- |)
 *                       0\  =  /0
 *                     ___/`---'\___
 *                   .' \\|     |// '.
 *                  / \\|||  :  |||// \
 *                 / _||||| -:- |||||- \
 *                |   | \\\  - /// |   |
 *                | \_|  ''\---/''  |_/ |
 *                \  .-\__  '-'  ___/-. /
 *              ___'. .'  /--.--\  `. .'___
 *           ."" '<  `.___\_<|>_/___.' >' "".
 *          | | :  `- \`.;`\ _ /`;.`/ - ` : | |
 *          \  \ `_.   \_ __\ /__ _/   .-` /  /
 *      =====`-.____`.___ \_____/___.-`___.-'=====
 *                        `=---='
 * 
 * 
 *      ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * 
 *            佛祖保佑       永不宕机     永无BUG
 */

#include "headfile.h"

/*--------------------------变量声明------------------------------*/
//左右电机直接输出占空比
int32   Left_MOTOR_Duty   = 0;
int32   Right_MOTOR_Duty  = 0;

//直接获取速度
int32   Left_MOTOR_Really_Speed  = 0;         
int32   Right_MOTOR_Really_Speed = 0;         
int32   Left_MOTOR_Really_Speed_Last = 0;
int32   Right_MOTOR_Really_Speed_Last = 0;

//绝对值速度
int32   Left_MOTOR_Speed  = 0;                //FTM1
int32   Right_MOTOR_Speed = 0;                //FTM2
int32   MOTOR_Speed       = 0;

/**
 * @description: 初始电机
 * @param {*}
 * @return {*}
 * @author: 郑有才
 */
void motor_init()
{
    // 请注意 这里接线时请务必不要接错 A0/A1一组 A2/A3一组 分别控制左右电机
	gpio_init(DIR_L, GPO, GPIO_HIGH, GPO_PUSH_PULL);								// GPIO 初始化为输出 默认上拉输出高
	gpio_init(DIR_R, GPO, GPIO_HIGH, GPO_PUSH_PULL);								// GPIO 初始化为输出 默认上拉输出高

	pwm_init(PWM_TIM, PWM_L, 10000, 0);												// PWM 通道2 初始化频率10KHz 占空比初始为0
	pwm_init(PWM_TIM, PWM_R, 10000, 0);												// PWM 通道4 初始化频率10KHz 占空比初始为0
}

/**
 * @description: 电机控制
 * @param {int32} left
 * @param {int32} right
 * @return {*}
 * @author: 郑有才
 */
void motor_control(int32 left, int32 right)
{
    Left_MOTOR_Duty     = left;
    Right_MOTOR_Duty    = right;
    motor_export();
}

/**
 * @description: 电机输出
 * @param {*}
 * @return {*}
 * @author: 郑有才
 */
void motor_export()
{
    Left_MOTOR_Duty=range_protect(Left_MOTOR_Duty, -18000, 18000);
    Right_MOTOR_Duty=range_protect(Right_MOTOR_Duty, -18000, 18000);

    if(Left_MOTOR_Duty >= 0)                // 左侧正转
    {
        gpio_set(DIR_L, GPIO_HIGH);         // DIR输出高电平
        pwm_duty_updata(PWM_TIM, PWM_L, Left_MOTOR_Duty);   // 计算占空比a
    }
    else 
    {
        gpio_set(DIR_L, GPIO_LOW);
        pwm_duty_updata(PWM_TIM, PWM_L, -Left_MOTOR_Duty);
    }

    if(Right_MOTOR_Duty >= 0)                // 左侧正转
    {
        gpio_set(DIR_R, GPIO_LOW);         // DIR输出高电平
        pwm_duty_updata(PWM_TIM, PWM_R, Right_MOTOR_Duty);   // 计算占空比a
    }
    else 
    {
        gpio_set(DIR_R, GPIO_HIGH);
        pwm_duty_updata(PWM_TIM, PWM_R, -Right_MOTOR_Duty);
    }
}

/**
 * @description: 限幅保护
 * @param {int32} duty
 * @param {int32} min
 * @param {int32} max
 * @return {*}
 * @author: 郑有才
 */
int32 range_protect(int32 duty, int32 min, int32 max)
{
	if (duty >= max)
	{
		return max;
	}
	if (duty <= min)
	{
		return min;
	}
	else
	{
		return duty;
	}
}
