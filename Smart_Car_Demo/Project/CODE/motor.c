#include "motor.h"

#define DIR_L				A0
#define DIR_R				A1

#define PWM_TIM				TIM_5
#define PWM_L				TIM_5_CH2_A01
#define PWM_R				TIM_5_CH4_A03

void motor_init(void)
{
    // ��ע�� �������ʱ����ز�Ҫ�Ӵ� A0/A1һ�� A2/A3һ�� �ֱ�������ҵ��
	gpio_init(DIR_L, GPO, GPIO_HIGH, GPO_PUSH_PULL);								// GPIO ��ʼ��Ϊ��� Ĭ�����������
	gpio_init(DIR_R, GPO, GPIO_HIGH, GPO_PUSH_PULL);								// GPIO ��ʼ��Ϊ��� Ĭ�����������

	pwm_init(PWM_TIM, PWM_L, 10000, 0);												// PWM ͨ��2 ��ʼ��Ƶ��10KHz ռ�ձȳ�ʼΪ0
	pwm_init(PWM_TIM, PWM_R, 10000, 0);												// PWM ͨ��4 ��ʼ��Ƶ��10KHz ռ�ձȳ�ʼΪ0
}

void motor_pid(int16 expect_speed)
{
    
}


void motor_control(int32 duty_l, int32 duty_r)
{
    //��ռ�ձ��޷�
	duty_l = limit(duty_l, PWM_DUTY_MAX);
	duty_r = limit(duty_r, PWM_DUTY_MAX);
    
    if(duty_l >= 0)											// �����ת
    {
        gpio_set(DIR_L, GPIO_HIGH);							// DIR����ߵ�ƽ
        pwm_duty_updata(PWM_TIM, PWM_L, duty_l);		    // ����ռ�ձ�
    }
    else													// ��෴ת
    {
        gpio_set(DIR_L, GPIO_LOW);							// DIR����͵�ƽ
        pwm_duty_updata(PWM_TIM, PWM_L, -duty_l);			// ����ռ�ձ�
    }
    
    if(duty_r >= 0)											// �Ҳ���ת
    {
        gpio_set(DIR_R, GPIO_HIGH);							// DIR����ߵ�ƽ
        pwm_duty_updata(PWM_TIM, PWM_R, duty_r);			// ����ռ�ձ�
    }
    else													// �Ҳ෴ת
    {
        gpio_set(DIR_R, GPIO_LOW);							// DIR����͵�ƽ
        pwm_duty_updata(PWM_TIM, PWM_R, -duty_r);			// ����ռ�ձ�
    }
}