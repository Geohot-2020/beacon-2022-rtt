/*********************************************************************************************************************
* COPYRIGHT NOTICE
* Copyright (c) 2019,��ɿƼ�
* All rights reserved.
* ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897
*
* �����������ݰ�Ȩ������ɿƼ����У�δ����������������ҵ��;��
* ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
*
* @file				common.c
* @company			�ɶ���ɿƼ����޹�˾
* @author			��ɿƼ�(QQ3184284598)
* @version			�鿴doc��version�ļ� �汾˵��
* @Software			IAR 8.32.4 or MDK 5.28
* @Target core		MM32F3277
* @Taobao			https://seekfree.taobao.com/
* @date				2021-02-22
********************************************************************************************************************/
 
#include "common.h"
#include "hal_misc.h"

CAMERA_TYPE_enum camera_type;													// ����ͷ���ͱ���
uint8 *camera_buffer_addr;														// ����ͷ��������ַָ��

//-------------------------------------------------------------------------------------------------------------------
// @brief		�жϳ�ʼ��
// @param		irqn			�жϺţ����Բ鿴 reg_common.h �ļ��е� IRQn_Type ö���嶨��
// @param		priority		ѡ����ж����ȼ� ��Χ [0-7]
// @param		status			ʹ�ܻ���ʧ��
// @return		void
// Sample usage:				nvic_init(EXTI0_IRQn, 0, ENABLE);				// �ⲿ�ж�0ʹ��,��ռ���ȼ����
//-------------------------------------------------------------------------------------------------------------------
void nvic_init(IRQn_Type irqn, uint8 priority, FunctionalState status)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = irqn;                                  // �жϺ�����
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= priority & 0x07;		// ��ռ���ȼ�ֵԽС�����ȼ�Խ��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority= 0;							// ��Ӧ���ȼ�ֵԽС�����ȼ�Խ��
	NVIC_InitStructure.NVIC_IRQChannelCmd = status;								// ʹ��
	NVIC_Init(&NVIC_InitStructure);
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		ȫ���ж�ʹ��
// @return		void
// Sample usage:				nvic_interrput_enable();
//-------------------------------------------------------------------------------------------------------------------
void nvic_interrput_enable (void)
{
	__ASM volatile("cpsie i");
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		ȫ���ж�ʧ��
// @return		void
// Sample usage:				nvic_interrput_disable();
//-------------------------------------------------------------------------------------------------------------------
void nvic_interrput_disable (void)
{
	__ASM volatile("cpsid i");
}

void NMI_Handler(void)
{
	while(1);
}
//void HardFault_Handler(void)
//{
//	//Ӳ���Ϸ�  ����������Խ�����ʹ����δ��ʼ�����豸
//	//������ʾ���һ�����⣬��������˵�ҵ������ʼ������ô����Ҫ��HardFault��
//	//�ܶ�����Ϊ�Լ������жϣ�Ȼ�����ж�����ʹ�������裬Ȼ�����ĳ�ʼ��˳�����ȳ�ʼ���ж�Ȼ���ٳ�ʼ������
//	//��ô��������ˣ��жϳ�ʼ�����֮��Ჿ���ж�ֱ�ӽ����жϺ��������������Ҳ������жϺ�������ô�ͻ�����ж�����ʹ�õ������軹û����ʼ��
//	//���Դ����Ҫע���������
//	while(1);
//}
void MemManage_Handler(void)
{
	while(1);
}
void BusFault_Handler(void)
{
	while(1);
}
void UsageFault_Handler(void)
{
	while(1);
}
void SVC_Handler(void)
{
	while(1);
}
void DebugMon_Handler(void)
{
	while(1);
}
//void PendSV_Handler(void)
//{
//	while(1);
//}