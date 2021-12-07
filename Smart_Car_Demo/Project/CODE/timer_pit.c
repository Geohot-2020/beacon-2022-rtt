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
 * @Description: 主要控制代码
 * @Version: v1.0
 * @Autor: 郑有才
 * @Date: 2021-11-24 14:46:45
 * @LastEditors: 郑有才
 * @LastEditTime: 2021-11-30 16:24:11
 */

#include "headfile.h"


S_FLOAT_XYZ 
	GYRO_Real,		    //陀螺仪转化后的数据
	ACC_Real,		    //加速度计转化后的数据
	Attitude_Angle,    	//当前角度
	Last_Angle,		    //上次角度
    Target_Angle,	    //目标角度
        Target_Angle_Grow;      //目标角度增长

S_INT16_XYZ
	GYRO,			    // 陀螺仪原始数据
	GYRO_Offset,	    // 陀螺仪温飘
	GYRO_Last,		    // 陀螺仪上次数据
	ACC, 			    // 加速度计数据
	ACC_Offset,		    // 加速度计温飘
	ACC_Last;		    // 加速度计上次数据

S_INT32_XYZ
	Tar_Ang_Vel,	    // 目标角速度
	Tar_Ang_Vel_Last,	// 上次目标角速度
        Tar_Ang_Vel_Grow;       // 目标角速度增长

int32 
	Speed_Now = 0,	        // 当前实际速度
        Speed_Now_Last=0,       // 上次实际速度
	Speed_Min = 0,	        // 左右最小速度
	Speed_Set = 0, 	        // 目标设定速度
	Theory_Duty = 0,        // 理论直立占空比
	Vel_Set = 0,	        // 目标转向角速度
    Direct_Parameter = 0,   // 转向系数
	Direct_Last = 0,
    Speed_dif = 0;

float Target_Angle_min=-73,       //跑动前倾最大角度
      Target_Angle_max=-50;       //跑动后仰最大角度

float accangle;


/**
 * @description: 串级PID
 * @param {void} *parameter
 * @return {*}
 * @author: 郑有才
 */
void timer1_pit_entry(void *parameter)
{

    static uint32 time;
    time++;

    /**
     * @description: 角速度环2ms，调PI；i大幅低频来回晃，过大高频抖动，勉强立住；加p到稍微抖动，没到高频振动，基本上能立
     * @param {*}
     * @return {*}
     * @author: 郑有才
     */    
    if(0 == (time%2))
    {
        ICM20602_GetData(&GYRO, &ACC);  
        Data_Filter();  //原始数据滤波
        accangle = asin(ACC_Real.X);    
        KalmanFilter(accangle);     //姿态解算，卡尔曼滤波
        //printf("%.2f, %.2f\n",accangle, Attitude_Angle.Y);
        // seekfree_wireless_send_buff((uint8 *)"\r\nSEEKFREE wireless test.", 25);		// 发送测试信息
        Theory_Duty += -PID_Increase(&Ang_Vel_PID, Ang_Vel, (int32)GYRO_Real.Y*10, (int32)(Tar_Ang_Vel.Y)); //增量式PID
        Theory_Duty = range_protect(Theory_Duty, -18000, 18000);  //限幅

        /*--------------------转向环，用角速度环做内环控制转向,加I解决电机转速问题------------------*/
            Direct_Parameter = -PID_Realize(&Direct_PID, Direct, (int32)GYRO_Real.Z, Speed_Min);
            Direct_Parameter = range_protect(Direct_Parameter, -18000, 18000);
        Direct_Last = Direct_Last*0.2 + Direct_Parameter*0.8 - camera_dif*10;
        
        Left_MOTOR_Duty = Theory_Duty - Direct_Last;
        Right_MOTOR_Duty = Theory_Duty + Direct_Last;

        motor_control(-Left_MOTOR_Duty, -Right_MOTOR_Duty);
        //motor_control(0, 7000);
    }

    /**
     * @description: 角度环10ms，调PD；稳稳立住，且拍前瞻力气大，车会往前冲；d响应速度，加了往前不会冲很多，过大都会高频抖动
     * @param {*}
     * @return {*}
     * @author: 郑有才
     */  
    if(0 == (time%10))
    {
        encoder_get();      //获取当前速度
        Tar_Ang_Vel.Y = PID_Realize(&Angle_PID, Angle, (int32)(accangle*100), (int32)Target_Angle.Y);      //增量式PID
        Tar_Ang_Vel.Y = range_protect(Tar_Ang_Vel.Y, -15000, 15000);
    }

    /**
     * @description: 摄像头采集， 20ms采集一次
     * @param {*}
     * @return {*}
     * @author: 郑有才
     */    
    if(0 == (time%20))
    {
        //rt_sem_take(camera_sem, RT_WAITING_FOREVER);
        //camera_dif = Camera_Control();
			//printf("camera_dif: %f\r\n", camera_dif);
    }

    /**
     * @description: 速度环50ms，调P；
     * @param {*}
     * @return {*}
     * @author: 郑有才
     */    
    if(0 == (time%50))
    {
        Target_Angle.Y = PID_Realize(&MOTOR_PID, MOTOR, Speed_Now, Speed_Set);
        Target_Angle.Y += Zero_Angle*100;
        Target_Angle.Y = range_protect((int32)Target_Angle.Y, Target_Angle_min, Target_Angle_max);
			//Target_Angle.Y = Zero_Angle*100;
    }

}


void timer_pit_init(void)
{
    rt_timer_t timer;
    
    //创建一个定时器 周期运行
    timer = rt_timer_create("timer1", timer1_pit_entry, RT_NULL, 1, RT_TIMER_FLAG_PERIODIC);
    
    //启动定时器
    if(RT_NULL != timer)
    {
        rt_timer_start(timer);
    }

    
}

/* 初始化用到的一些变量 */
void Balance_Init(void)
{
    Attitude_Angle.X = 0;
    Target_Angle.Y = 0;
    Tar_Ang_Vel.Y = 0;
    Tar_Ang_Vel.Z = 0;
		Speed_Set = 100;
}


