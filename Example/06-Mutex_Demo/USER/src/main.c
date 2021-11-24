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
// ��������ʾ��������ʹ��
// 
// ���µĹ��̻��߹����ƶ���λ�����ִ�����²���
// ��һ�� �ر��������д򿪵��ļ�
// �ڶ��� project->clean  �ȴ��·�����������
// 
// *************************** ����˵�� ***************************

// **************************** �궨�� ****************************
// **************************** �궨�� ****************************

// **************************** �������� ****************************
static rt_mutex_t dynamic_mutex = RT_NULL;						//�ź������ƿ�ָ��

static rt_uint32_t num1 = 0;
static rt_uint32_t num2 = 0;
// **************************** �������� ****************************

// **************************** �������� ****************************
void thread1_entry (void *parameter);
void thread2_entry (void *parameter);
int mutex_example(void);

// ʹ��INIT_APP_EXPORT���Զ���ʼ����Ҳ����ͨ���������߳��ڵ���mutex_example�������г�ʼ��
INIT_APP_EXPORT(mutex_example);									// Ӧ�ó�ʼ��

// ʹ�û�����֮ǰ ���Ȳ鿴 rtconfig.h �е� RT_USING_MUTEX �궨���Ƿ���
// ʹ�û�����֮ǰ ���Ȳ鿴 rtconfig.h �е� RT_USING_MUTEX �궨���Ƿ���
// ʹ�û�����֮ǰ ���Ȳ鿴 rtconfig.h �е� RT_USING_MUTEX �궨���Ƿ���

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
void thread1_entry (void *parameter)
{
	while(1)
	{
		rt_mutex_take(dynamic_mutex, RT_WAITING_FOREVER);		// ��ȡ��������һֱ�ȴ�
		num1++;
		rt_thread_mdelay(100);
		num2++;
		rt_mutex_release(dynamic_mutex);						// �ͷŻ�����
	}
}

//------------------------------------------------------------
// @brief		��ȡ�ź����߳���ں���
// @param		parameter	����
// @return		void
// Sample usage:
//------------------------------------------------------------
void thread2_entry (void *parameter)
{
	rt_thread_t tid;

	while(1)
	{
		// �߳�2��ȡ�����������ж�num1��num2 ��ֵ�Ƿ���ͬ����ͬ��ʾ�����������������á�
		rt_mutex_take(dynamic_mutex, RT_WAITING_FOREVER);		// ��ȡ��������һֱ�ȴ�
		if(num1 == num2)
		{
			rt_kprintf("\n num1 = num2 , num1 = %d  num2 = %d\n", num1, num2);
		}
		else
		{
			rt_kprintf("\n num1 != num2, num1 = %d  num2 = %d\n", num1, num2);
		}
		if(10 == num1)											// ����10�κ� ɾ�����������߳�
		{
			// ɾ��������
			rt_mutex_delete(dynamic_mutex);
			rt_kprintf("\n delete dynamic_mutex!\n");

			// ��ȡ�߳�1�Ŀ��ƿ�ָ��
			tid = rt_thread_find("thread1");

			// ɾ���߳�1
			rt_thread_delete(tid);
			rt_kprintf("\n delete thread1!\n");

			// �˳�ѭ�� �߳�2�Զ�ɾ��
			break;
		}
		rt_mutex_release(dynamic_mutex);						// �ͷŻ�����
	}

	rt_kprintf("\n thread2 exit!\n");
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		�̴߳����Լ�����
// @param		void
// @return		void
// Sample usage:
//-------------------------------------------------------------------------------------------------------------------
int mutex_example(void)
{
	// �߳̿��ƿ�ָ��
	rt_thread_t tid;

	// ������̬������
	dynamic_mutex = rt_mutex_create("dynamic mutex", RT_IPC_FLAG_FIFO);
	if (dynamic_mutex == RT_NULL)								// �߳̿��ƿ鴴���ɹ�
	{
		rt_kprintf("create dynamic mutex failed.\n");
		return -1;
	}
	// �����߳̿��ƿ�
	tid = rt_thread_create(
		"thread1",												// �߳�����
		thread1_entry,											// �߳���ں���
		RT_NULL,												// �̲߳���
		256,													// ջ�ռ��С
		3,														// �����߳����ȼ�����ֵԽС�����ȼ�Խ�ߣ�0Ϊ������ȼ���
																// ����ͨ���޸�rt_config.h�е�RT_THREAD_PRIORITY_MAX�궨��(Ĭ��ֵΪ8)���޸����֧�ֵ����ȼ�
		10);													// ʱ��Ƭ

	if(tid != RT_NULL)											// �̴߳����ɹ�
	{
		// ���и��߳�
		rt_thread_startup(tid);
	}

	tid = rt_thread_create(
		"thread2",												// �߳�����
		thread2_entry,											// �߳���ں���
		RT_NULL,												// �̲߳���
		256,													// ջ�ռ��С
		4,														// �����߳����ȼ�����ֵԽС�����ȼ�Խ�ߣ�0Ϊ������ȼ���
																// ����ͨ���޸�rt_config.h�е�RT_THREAD_PRIORITY_MAX�궨��(Ĭ��ֵΪ8)���޸����֧�ֵ����ȼ�
		10);													// ʱ��Ƭ

	if(tid != RT_NULL)											// �̴߳����ɹ�
	{
		// ���и��߳�
		rt_thread_startup(tid);
	}

	return 0;
}
// **************************** �������� ****************************