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
 * @Description: ������
 * @Version: v1.0
 * @Autor: ֣�в�
 * @Date: 2021-11-24 14:46:45
 * @LastEditors: ֣�в�
 * @LastEditTime: 2021-11-26 16:59:21
 */
#include "buzzer.h"
#define BUZZER_PIN			D12			// ���������Ϸ�������Ӧ����


rt_mailbox_t buzzer_mailbox;


void buzzer_entry(void *parameter)
{
    uint32 mb_data;
    while(1)
    {
        //�����������ݣ����û������������ȴ����ͷ�CPU����Ȩ
        rt_mb_recv(buzzer_mailbox, &mb_data, RT_WAITING_FOREVER);

        gpio_set(BUZZER_PIN, 1);    //�򿪷�����
        rt_thread_mdelay(mb_data);  //��ʱ
        gpio_set(BUZZER_PIN, 0);    //�رշ�����
    }
}





void buzzer_init(void)
{
    rt_thread_t tid;
    
    //��ʼ����������ʹ�õ�GPIO
    gpio_init(BUZZER_PIN, GPO, 0, GPO_PUSH_PULL);
    
    //��������
    buzzer_mailbox = rt_mb_create("buzzer", 5, RT_IPC_FLAG_FIFO);
    
    //�������������߳�
    tid = rt_thread_create("buzzer", buzzer_entry, RT_NULL, 256, 20, 2);
    
    //�����߳�
    if(RT_NULL != tid)
    {
        rt_thread_startup(tid);
    }
}
