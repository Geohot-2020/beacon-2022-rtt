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
 *            佛祖保佑       永不宕机     永无BUG
 */

/*
 * @Description: 显示
 * @Version: v1.0
 * @Autor: 郑有才
 * @Date: 2021-11-24 14:46:45
 * @LastEditors: 郑有才
 * @LastEditTime: 2021-11-26 16:59:42
 */
#include "headfile.h"


void display_entry(void *parameter)
{
    while(1)
    {
        ips200_displayimage032(mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
        ips200_showfloat(0, 8, GYRO_Real.Y,2,2);
        ips200_showfloat(0, 9, accangle,2,2);
			ips200_showfloat(0, 17, Tar_Ang_Vel.Y,2,2);
			ips200_showfloat(0, 18, Target_Angle.Y,2,2);
        ips200_showint16(0, 10, speed_l);
        ips200_showint16(0, 11, speed_r);
        ips200_showint16(0, 12, Left_MOTOR_Duty);
        ips200_showint16(0, 13, Right_MOTOR_Duty);
    }
    
}






void display_init(void)
{
    rt_thread_t tid;
    
    //初始化屏幕
    ips200_init();
    
    //创建显示线程 优先级设置为6 
    tid = rt_thread_create("display", display_entry, RT_NULL, 512, 31, 30);
    
    //启动显示线程
    if(RT_NULL != tid)
    {
        rt_thread_startup(tid);
    }
}
