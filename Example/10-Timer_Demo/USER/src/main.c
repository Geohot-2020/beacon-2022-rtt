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
// 本例程演示RTT定时器使用方式
// 
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完
// 
// *************************** 例程说明 ***************************

// **************************** 宏定义 ****************************
// **************************** 宏定义 ****************************

// **************************** 变量定义 ****************************
rt_timer_t timer1;											// 定时器控制块指针
// **************************** 变量定义 ****************************

// **************************** 代码区域 ****************************
void timerout1 (void *parameter);
int timer_example (void);

// 使用INIT_APP_EXPORT宏自动初始化，也可以通过在其他线程内调用timer_example函数进行初始化
INIT_APP_EXPORT(timer_example);

int main(void)
{
	gpio_init(B13, GPO, 0, GPO_PUSH_PULL);

	while(1)
	{
		rt_thread_mdelay(100);
		gpio_toggle(B13);
	}
}

//------------------------------------------------------------
// @brief		定时器超时函数
// @param		parameter	参数
// @return		void
// @note		定时器超时函数，当时间到之后会自动运行该函数
// Sample usage:
//------------------------------------------------------------
void timerout1 (void *parameter)
{
	static rt_uint8_t count = 0;
	rt_kprintf("timerout!!!\n");

	// 执行10次后停止该定时器
	if(count++ >= 10)
	{
		rt_timer_stop(timer1);
		rt_kprintf("stop timer1!\n");

		rt_timer_delete(timer1);
		rt_kprintf("delete timer1!!!\n");
	}
}

//------------------------------------------------------------
// @brief		线程创建以及启动
// @param		void		空
// @return		void
// Sample usage:
//------------------------------------------------------------
int timer_example (void)
{
	// 创建一个定时器 周期100个tick
	timer1 = rt_timer_create(
		"timer1",											// timer1表示定时器的名称，8个字符内。
		timerout1,											// timerout1表示时间到了之后需要执行的函数
		RT_NULL,											// RT_NULL表示不需要传递参数。
		100,												// 100表示定时器的超时时间为100个系统tick，系统周期为1毫秒，则100表示100毫秒
		RT_TIMER_FLAG_PERIODIC);							// RT_TIMER_FLAG_PERIODIC表示定时器以周期运行  如果设置为RT_TIMER_FLAG_ONE_SHOT则只会运行一次

	// 首先检查定时器控制块不是空，则启动定时器
	if(timer1 != RT_NULL)
	{
		rt_timer_start(timer1);
	}

	return 0;
}
// **************************** 代码区域 ****************************
