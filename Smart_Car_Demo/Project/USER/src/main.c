/*
 * @Description: 
 * @Version: v1.0
 * @Autor: ֣�в�
 * @Date: 2021-11-24 14:46:45
 * @LastEditors: ֣�в�
 * @LastEditTime: 2021-12-07 10:50:47
 */
/*********************************************************************************************************************
* COPYRIGHT NOTICE
* Copyright (c) 2019,��ɿƼ�
* All rights reserved.
* ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897
*
* �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
* ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
*
* @file				main
* @company			�ɶ���ɿƼ����޹�˾
* @author			��ɿƼ�(QQ3184284598)
* @version			�鿴doc��version�ļ� �汾˵��
* @Software			IAR 8.3 or MDK 5.24
* @Target core		MM32F3277
* @Taobao			https://seekfree.taobao.com/
* @date				2021-02-22
********************************************************************************************************************/

#include "headfile.h"

// *************************** ����˵�� ***************************
// 
// ������Ҫ׼����ɿƼ� MM32F3277 ���İ�һ��
// 
// ����������Ҫ׼����ɿƼ� CMSIS-DAP ���������� �� ARM ���������� һ��
// 
// �������Ǹ��չ��� ������ͬѧ����ֲʹ��
// 
// ���µĹ��̻��߹����ƶ���λ�����ִ�����²���
// ��һ�� �ر��������д򿪵��ļ�
// �ڶ��� project->clean  �ȴ��·�����������
// 
// *************************** ����˵�� ***************************

// **************************** �궨�� ****************************
// **************************** �궨�� ****************************

// **************************** �������� ****************************
// **************************** �������� ****************************

// **************************** �������� ****************************



int main(void)
{
    uint8 tm=0;

	camera_sem = rt_sem_create("camera", 0, RT_IPC_FLAG_FIFO);
    
    Balance_Init();
    mt9v03x_init();
    ICM20602_Init();
    display_init();
    encoder_init();
    buzzer_init();
    button_init();
    motor_init();
    seekfree_wireless_init();
    PID_Parameter_Init(&MOTOR_PID);	    //�ٶȻ�PID������ʼ��
    PID_Parameter_Init(&Angle_PID);	    //�ǶȻ�PID������ʼ��
    PID_Parameter_Init(&Ang_Vel_PID);	//���ٶȻ�PID������ʼ��
    PID_Parameter_Init(&Direct_PID);	//ת��PID������ʼ��
    PID_Parameter_Init(&Distance_PID);	//λ�û�PID������ʼ��
    // elec_init();
    
    timer_pit_init();
	
	
	gpio_init(B13, GPO, 0, GPO_PUSH_PULL);
	

	while(1)
	{
		//�ȴ�����ͷ�ɼ����
        rt_sem_take(camera_sem, RT_WAITING_FOREVER);
		
        camera_dif = Camera_Control();
        mt9v03x_finish_flag=0;
		printf("camera_dif: %f\r\n", camera_dif);
		showBeacon();
		gpio_toggle(B13);
		
		
	}
}
// **************************** �������� ****************************

