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
// *            ���汣��       ����崻�     ����BUG
// */

///*
// * @Description: 
// * @Version: v1.0
// * @Autor: ֣�в�
// * @Date: 2021-11-24 14:46:45
// * @LastEditors: ֣�в�
// * @LastEditTime: 2021-11-26 16:58:18
// */

//#include "motor.h"

//#define DIR_L				A0
//#define DIR_R				A2

//#define PWM_TIM				TIM_5
//#define PWM_L				TIM_5_CH2_A01
//#define PWM_R				TIM_5_CH4_A03

////���ҵ��ֱ�����ռ�ձ�
//int32   Left_MOTOR_Duty   = 0;
//int32   Right_MOTOR_Duty  = 0;
//int32   Speed_motor_duty  = 0;                //���ٶ��йص�ռ�ձ�

//void motor_init(void)
//{
//    // ��ע�� �������ʱ����ز�Ҫ�Ӵ� A0/A1һ�� A2/A3һ�� �ֱ�������ҵ��
//	gpio_init(DIR_L, GPO, GPIO_HIGH, GPO_PUSH_PULL);								// GPIO ��ʼ��Ϊ��� Ĭ�����������
//	gpio_init(DIR_R, GPO, GPIO_HIGH, GPO_PUSH_PULL);								// GPIO ��ʼ��Ϊ��� Ĭ�����������

//	pwm_init(PWM_TIM, PWM_L, 10000, 0);												// PWM ͨ��2 ��ʼ��Ƶ��10KHz ռ�ձȳ�ʼΪ0
//	pwm_init(PWM_TIM, PWM_R, 10000, 0);												// PWM ͨ��4 ��ʼ��Ƶ��10KHz ռ�ձȳ�ʼΪ0
//}

//void motor_pid(int16 expect_speed)
//{
//    
//}


//void motor_control(int32 duty_l, int32 duty_r)
//{
//    //��ռ�ձ��޷�
//	duty_l = limit(duty_l, PWM_DUTY_MAX);
//	duty_r = limit(duty_r, PWM_DUTY_MAX);
//    
//    if(duty_l >= 0)											// �����ת
//    {
//        gpio_set(DIR_L, GPIO_HIGH);							// DIR����ߵ�ƽ
//        pwm_duty_updata(PWM_TIM, PWM_L, duty_l);		    // ����ռ�ձ�
//    }
//    else													// ��෴ת
//    {
//        gpio_set(DIR_L, GPIO_LOW);							// DIR����͵�ƽ
//        pwm_duty_updata(PWM_TIM, PWM_L, -duty_l);			// ����ռ�ձ�1
//    }
//    
//    if(duty_r >= 0)											// �Ҳ���ת
//    {
//        gpio_set(DIR_R, GPIO_HIGH);							// DIR����ߵ�ƽ
//        pwm_duty_updata(PWM_TIM, PWM_R, duty_r);			// ����ռ�ձ�
//    }
//    else													// �Ҳ෴ת
//    {
//        gpio_set(DIR_R, GPIO_LOW);							// DIR����͵�ƽ
//        pwm_duty_updata(PWM_TIM, PWM_R, -duty_r);			// ����ռ�ձ�
//    }
//}



/*
 * @Description: ����ײ�����
 * @Version: v1.0
 * @Autor: ֣�в�
 * @Date: 2021-11-28 09:33:28
 * @LastEditors: ֣�в�
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
 *            ���汣��       ����崻�     ����BUG
 */

#include "headfile.h"

/*--------------------------��������------------------------------*/
//���ҵ��ֱ�����ռ�ձ�
int32   Left_MOTOR_Duty   = 0;
int32   Right_MOTOR_Duty  = 0;

//ֱ�ӻ�ȡ�ٶ�
int32   Left_MOTOR_Really_Speed  = 0;         
int32   Right_MOTOR_Really_Speed = 0;         
int32   Left_MOTOR_Really_Speed_Last = 0;
int32   Right_MOTOR_Really_Speed_Last = 0;

//����ֵ�ٶ�
int32   Left_MOTOR_Speed  = 0;                //FTM1
int32   Right_MOTOR_Speed = 0;                //FTM2
int32   MOTOR_Speed       = 0;

/**
 * @description: ��ʼ���
 * @param {*}
 * @return {*}
 * @author: ֣�в�
 */
void motor_init()
{
    // ��ע�� �������ʱ����ز�Ҫ�Ӵ� A0/A1һ�� A2/A3һ�� �ֱ�������ҵ��
	gpio_init(DIR_L, GPO, GPIO_HIGH, GPO_PUSH_PULL);								// GPIO ��ʼ��Ϊ��� Ĭ�����������
	gpio_init(DIR_R, GPO, GPIO_HIGH, GPO_PUSH_PULL);								// GPIO ��ʼ��Ϊ��� Ĭ�����������

	pwm_init(PWM_TIM, PWM_L, 10000, 0);												// PWM ͨ��2 ��ʼ��Ƶ��10KHz ռ�ձȳ�ʼΪ0
	pwm_init(PWM_TIM, PWM_R, 10000, 0);												// PWM ͨ��4 ��ʼ��Ƶ��10KHz ռ�ձȳ�ʼΪ0
}

/**
 * @description: �������
 * @param {int32} left
 * @param {int32} right
 * @return {*}
 * @author: ֣�в�
 */
void motor_control(int32 left, int32 right)
{
    Left_MOTOR_Duty     = left;
    Right_MOTOR_Duty    = right;
    motor_export();
}

/**
 * @description: ������
 * @param {*}
 * @return {*}
 * @author: ֣�в�
 */
void motor_export()
{
    Left_MOTOR_Duty=range_protect(Left_MOTOR_Duty, -12000, 12000);
    Right_MOTOR_Duty=range_protect(Right_MOTOR_Duty, -12000, 12000);

    if(Left_MOTOR_Duty >= 0)                // �����ת
    {
        gpio_set(DIR_L, GPIO_HIGH);         // DIR����ߵ�ƽ
        pwm_duty_updata(PWM_TIM, PWM_L, Left_MOTOR_Duty);   // ����ռ�ձ�a
    }
    else 
    {
        gpio_set(DIR_L, GPIO_LOW);
        pwm_duty_updata(PWM_TIM, PWM_L, -Left_MOTOR_Duty);
    }

    if(Right_MOTOR_Duty >= 0)                // �����ת
    {
        gpio_set(DIR_R, GPIO_LOW);         // DIR����ߵ�ƽ
        pwm_duty_updata(PWM_TIM, PWM_R, Right_MOTOR_Duty);   // ����ռ�ձ�a
    }
    else 
    {
        gpio_set(DIR_R, GPIO_HIGH);
        pwm_duty_updata(PWM_TIM, PWM_R, -Right_MOTOR_Duty);
    }
}

/**
 * @description: �޷�����
 * @param {int32} duty
 * @param {int32} min
 * @param {int32} max
 * @return {*}
 * @author: ֣�в�
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
