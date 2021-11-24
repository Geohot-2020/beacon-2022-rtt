#include "headfile.h"






void display_entry(void *parameter)
{
    while(1)
    {
        ips200_displayimage032(mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
        ips200_showfloat(0, 8, accangle,2,2);
        ips200_showfloat(0, 9, Attitude_Angle.Y,2,2);
        ips200_showint16(0, 10, speed_l);
        ips200_showint16(0, 11, speed_r);
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
