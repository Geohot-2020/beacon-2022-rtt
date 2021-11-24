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
	Direct_Last = 0;

float Target_Angle_min=-100,       //跑动前倾最大角度
      Target_Angle_max=3;       //跑动后仰最大角度

float accangle;


//串级PID
void timer1_pit_entry(void *parameter)
{
    static uint32 time;
    time++;
    /*-------------------------电机控制角速度环1ms---------------------------*/
    ICM20602_GetData(&GYRO, &ACC);  //读取陀螺仪数据
    Data_Filter();			        //对原始数据滑动滤波
    /* 角速度环做最内环控制直立 */


    //姿态解算
    accangle = asin(ACC_Real.Y);
    KalmanFilter(accangle);
    //printf("%.2f, %.2f\n",accangle, Attitude_Angle.Y);

    if(0 == (time%100))
    {
        //-------------------------------------------------采集编码器数据
        encoder_get();
			
        //-------------------------------------------------电机控制速度环
    }

    if(0 == (time%5))
    {
        //-------------------------------------------------采集加速度数据
        // get_icm20602_accdata_spi();

        // //采集电磁信号
        // elec_get();
        
        // //根据电磁信号计算车身位置
        // elec_calculate();
			
				//-------------------------------------------------将图像计算出车身位置，进行车模控制
				//-------------------------------------------------互补滤波
				//-------------------------------------------------电机控制角度环
				
    }
    
		
    //控制电机转动
    motor_control(10000, 10000);
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
    Target_Angle.X = 0;
    Tar_Ang_Vel.Y = 0;
    Tar_Ang_Vel.Z = 0;
}

INIT_APP_EXPORT(Balance_Init);
