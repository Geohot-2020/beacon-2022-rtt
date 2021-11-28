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

/*
 * @Description: ����̰߳���
 * @Version: v1.0
 * @Autor: ֣�в�
 * @Date: 2021-11-24 14:46:45
 * @LastEditors: ֣�в�
 * @LastEditTime: 2021-11-26 16:58:57
 */
#include "headfile.h"

#define KEY_1   D0  // ���������ϰ�����Ӧ����
#define KEY_2   D1  // ���������ϰ�����Ӧ����
#define KEY_3   D2  // ���������ϰ�����Ӧ����
#define KEY_4   D3  // ���������ϰ�����Ӧ����

//����״̬����
uint8 key1_status = 1;
uint8 key2_status = 1;
uint8 key3_status = 1;
uint8 key4_status = 1;

//��һ�ο���״̬����
uint8 key1_last_status;
uint8 key2_last_status;
uint8 key3_last_status;
uint8 key4_last_status;

//�����ź���
rt_sem_t key1_sem;
rt_sem_t key2_sem;
rt_sem_t key3_sem;
rt_sem_t key4_sem;

void button_entry(void *parameter)
{

    //���水��״̬
    key1_last_status = key1_status;
    key2_last_status = key2_status;
    key3_last_status = key3_status;
    key4_last_status = key4_status;
    
    //��ȡ��ǰ����״̬
    key1_status = gpio_get(KEY_1);
    key2_status = gpio_get(KEY_2);
    key3_status = gpio_get(KEY_3);
    key4_status = gpio_get(KEY_4);
    
    //��⵽��������֮�󲢷ſ� �ͷ�һ���ź���
    if(key1_status && !key1_last_status)    
    {
        rt_sem_release(key1_sem);
        rt_mb_send(buzzer_mailbox, 100);
    }
    if(key2_status && !key2_last_status)    
    {
        rt_sem_release(key2_sem);
        rt_mb_send(buzzer_mailbox, 300);
    }
    if(key3_status && !key3_last_status)    
    {
        rt_sem_release(key3_sem);
        rt_mb_send(buzzer_mailbox, 600);
    }
    if(key4_status && !key4_last_status)    
    {
        rt_sem_release(key4_sem);
        rt_mb_send(buzzer_mailbox, 1000);
    }
    
    
}

void button_init(void)
{
    rt_timer_t timer1;
    
    gpio_init(KEY_1, GPI, GPIO_HIGH, GPI_PULL_UP);          // ��ʼ��ΪGPIO�������� Ĭ�������ߵ�ƽ
	gpio_init(KEY_2, GPI, GPIO_HIGH, GPI_PULL_UP);          
	gpio_init(KEY_3, GPI, GPIO_HIGH, GPI_PULL_UP);          
	gpio_init(KEY_4, GPI, GPIO_HIGH, GPI_PULL_UP);          
    
    
    key1_sem = rt_sem_create("key1", 0, RT_IPC_FLAG_FIFO);  //�����������ź��������������¾��ͷ��ź���������Ҫʹ�ð����ĵط���ȡ�ź�������
    key2_sem = rt_sem_create("key2", 0, RT_IPC_FLAG_FIFO);  
    key3_sem = rt_sem_create("key3", 0, RT_IPC_FLAG_FIFO);  
    key4_sem = rt_sem_create("key4", 0, RT_IPC_FLAG_FIFO);  
    
    timer1 = rt_timer_create("button", button_entry, RT_NULL, 20, RT_TIMER_FLAG_PERIODIC);

    if(RT_NULL != timer1) 
    {
        rt_timer_start(timer1);
    }
}
