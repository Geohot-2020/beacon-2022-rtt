/*
 * @Description: 
 * @Version: v1.0
 * @Autor: 郑有才
 * @Date: 2021-11-24 14:46:45
 * @LastEditors: 郑有才
 * @LastEditTime: 2021-12-07 10:50:47
 */
/*********************************************************************************************************************
* COPYRIGHT NOTICE
* Copyright (c) 2019,逐飞科技
* All rights reserved.
* 技术讨论QQ群：一群：179029047(已满)  二群：244861897
*
* 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
* 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
*
* @file				main
* @company			成都逐飞科技有限公司
* @author			逐飞科技(QQ3184284598)
* @version			查看doc内version文件 版本说明
* @Software			IAR 8.3 or MDK 5.24
* @Target core		MM32F3277
* @Taobao			https://seekfree.taobao.com/
* @date				2021-02-22
********************************************************************************************************************/

#include "headfile.h"

// *************************** 例程说明 ***************************
// 
// 测试需要准备逐飞科技 MM32F3277 核心板一块
// 
// 调试下载需要准备逐飞科技 CMSIS-DAP 调试下载器 或 ARM 调试下载器 一个
// 
// 本例程是个空工程 用来给同学们移植使用
// 
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完
// 
// *************************** 例程说明 ***************************

// **************************** 宏定义 ****************************
// **************************** 宏定义 ****************************

// **************************** 变量定义 ****************************
// **************************** 变量定义 ****************************

// **************************** 代码区域 ****************************



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
    PID_Parameter_Init(&MOTOR_PID);	    //速度环PID参数初始化
    PID_Parameter_Init(&Angle_PID);	    //角度环PID参数初始化
    PID_Parameter_Init(&Ang_Vel_PID);	//角速度环PID参数初始化
    PID_Parameter_Init(&Direct_PID);	//转向环PID参数初始化
    PID_Parameter_Init(&Distance_PID);	//位置环PID参数初始化
    // elec_init();
    
    timer_pit_init();
	
	
	gpio_init(B13, GPO, 0, GPO_PUSH_PULL);
	

	while(1)
	{
		//等待摄像头采集完毕
        rt_sem_take(camera_sem, RT_WAITING_FOREVER);
		
        camera_dif = Camera_Control();
        mt9v03x_finish_flag=0;
		printf("camera_dif: %f\r\n", camera_dif);
		showBeacon();
		gpio_toggle(B13);
		
		
	}
}
// **************************** 代码区域 ****************************

