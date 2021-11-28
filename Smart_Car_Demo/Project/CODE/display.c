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
 * @Description: ��ʾ
 * @Version: v1.0
 * @Autor: ֣�в�
 * @Date: 2021-11-24 14:46:45
 * @LastEditors: ֣�в�
 * @LastEditTime: 2021-11-26 16:59:42
 */
#include "headfile.h"


void display_entry(void *parameter)
{
    while(1)
    {
        ips200_displayimage032(mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
        ips200_showfloat(0, 8, GYRO_Real.Y,2,2);
        ips200_showfloat(0, 9, Tar_Ang_Vel.Y,2,2);
        ips200_showint16(0, 10, speed_l);
        ips200_showint16(0, 11, speed_r);
        ips200_showint16(0, 12, Left_MOTOR_Duty);
        ips200_showint16(0, 13, Right_MOTOR_Duty);
    }
    
}






void display_init(void)
{
    rt_thread_t tid;
    
    //��ʼ����Ļ
    ips200_init();
    
    //������ʾ�߳� ���ȼ�����Ϊ6 
    tid = rt_thread_create("display", display_entry, RT_NULL, 512, 31, 30);
    
    //������ʾ�߳�
    if(RT_NULL != tid)
    {
        rt_thread_startup(tid);
    }
}
