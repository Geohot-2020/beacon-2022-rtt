/*********************************************************************************************************************
* COPYRIGHT NOTICE
* Copyright (c) 2019,��ɿƼ�
* All rights reserved.
* ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897
*
* �����������ݰ�Ȩ������ɿƼ����У�δ����������������ҵ��;��
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
// ��������ʾ�ź�����ʹ��
// 
// ���µĹ��̻��߹����ƶ���λ�����ִ�����²���
// ��һ�� �ر��������д򿪵��ļ�
// �ڶ��� project->clean  �ȴ��·�����������
// 
// *************************** ����˵�� ***************************

// **************************** �궨�� ****************************
// **************************** �궨�� ****************************

// **************************** �������� ****************************
static rt_sem_t dynamic_sem = RT_NULL;							// ����ָ���ź�����ָ��
// **************************** �������� ****************************

// **************************** �������� ****************************
void release_semaphore_thread_entry (void *parameter);
void get_semaphore_thread_entry (void *parameter);
int semaphore_example (void);

INIT_APP_EXPORT(semaphore_example);      //Ӧ�ó�ʼ��

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
// @brief		�ͷ��ź����߳���ں���
// @param		parameter	����
// @return		void
// Sample usage:
//------------------------------------------------------------
void release_semaphore_thread_entry(void *parameter)
{
	static rt_uint16_t num = 0;
	while(1)
	{
		if(num++ < 10)
		{
			rt_kprintf("\n release_semaphore_thread_entry release a semaphore. \n");
			rt_sem_release(dynamic_sem);						// �ͷ��ź���
			rt_thread_mdelay(1000);								// ��ʱ1000ms�����ó�CPUʹ��Ȩ
		}
		else
		{
			rt_kprintf("\n demo end \n");
			return;
		}
	}
}

//------------------------------------------------------------
// @brief		��ȡ�ź����߳���ں���
// @param		parameter	����
// @return		void
// Sample usage:
//------------------------------------------------------------
void get_semaphore_thread_entry(void *parameter)
{
	static rt_err_t result;
	static rt_uint8_t sum = 0;

	while(1)
	{
		// ͨ�����õȴ��ķ�ʽ��ȡ�ź�������ȡ������ִ��sum++�Ĳ���
		result = rt_sem_take(dynamic_sem, RT_WAITING_FOREVER);

		if(result == RT_EOK)									// ��ȡ�ź����ɹ�
		{
			sum++;
			rt_kprintf("\n get_semaphore_thread_entry take a semaphore. sum semaphore = %d \n", sum);
		}
	}
}
//-------------------------------------------------------------------------------------------------------------------
// @brief		�̴����Լ�����
// @param		void
// @return		void
// Sample usage:
//-------------------------------------------------------------------------------------------------------------------
int semaphore_example(void)
{
	// �����ź���
	dynamic_sem = rt_sem_create("dynamic_semaphore", 0 ,RT_IPC_FLAG_FIFO);
	// �����߳̿��ƿ�
	rt_thread_t tid;

	tid = rt_thread_create(
		"release_semaphore",									// �߳�����
		release_semaphore_thread_entry,							// �߳���ں���
		RT_NULL,												// �̲߳���
		256,													// ջ�ռ��С
		4,														// �����߳����ȼ�����ֵԽС�����ȼ�Խ�ߣ�0Ϊ������ȼ���
																// ����ͨ���޸�rt_config.h�е�RT_THREAD_PRIORITY_MAX�궨��(Ĭ��ֵΪ8)���޸����֧�ֵ����ȼ�
		10);													// ʱ��Ƭ

	if(tid != RT_NULL)											// �̴߳����ɹ�
	{
		rt_thread_startup(tid);									// ���и��߳�
	}

	tid = rt_thread_create(
		"get_semaphore",										// �߳�����
		get_semaphore_thread_entry,								// �߳���ں���
		RT_NULL,												// �̲߳���
		256,													// ջ�ռ��С
		3,														// �����߳����ȼ�����ֵԽС�����ȼ�Խ�ߣ�0Ϊ������ȼ���
																// ����ͨ���޸�rt_config.h�е�RT_THREAD_PRIORITY_MAX�궨��(Ĭ��ֵΪ8)���޸����֧�ֵ����ȼ�
		10);													// ʱ��Ƭ

	if(tid != RT_NULL)											// �̴߳����ɹ�
	{
		rt_thread_startup(tid);									// ���и��߳�
	}

	return 0;
}
// **************************** �������� ****************************