/*
 * @Description: 
 * @Version: v1.0
 * @Autor: 郑有才
 * @Date: 2021-11-24 14:46:45
 * @LastEditors: 郑有才
 * @LastEditTime: 2021-12-07 16:30:46
 */
#ifndef _timer_pit_h
#define _timer_pit_h

extern S_FLOAT_XYZ 
	GYRO_Real,		    //陀螺仪转化后的数据
	ACC_Real,		    //加速度计转化后的数据
	Attitude_Angle,	    //当前角度 
	Last_Angle,		    // 上次角度
	Target_Angle,	    // 目标角度
        Target_Angle_Grow;      // 目标角度增长

extern S_INT16_XYZ
	GYRO,			    // 陀螺仪原始数据
	GYRO_Offset,	    // 陀螺仪温飘
	GYRO_Last,		    // 陀螺仪上次数据
	ACC, 			    // 加速度计数据
	ACC_Offset,		    // 加速度计温飘
	ACC_Last;		    // 加速度计上次数据

extern S_INT32_XYZ
	Tar_Ang_Vel,	    // 目标角速度
	Tar_Ang_Vel_Last,	// 上次目标角速度
        Tar_Ang_Vel_Grow;       // 目标角速度增长

extern int32 
	Speed_Now,		    // 当前实际速度
        Speed_Now_Last,       // 上次实际速度
	Speed_Min,		    // 左右最小速度
	Speed_Set, 		    // 目标设定速度
    Theory_Duty,    // 理论直立占空比
	Vel_Set,		    // 目标转向角速度
    Direct_Parameter,   // 转向系数
	Direct_Last,
	Speed_dif;

extern float Target_Angle_min,       //跑动前倾最大角度
             Target_Angle_max;       //跑动后仰最大角度

extern float accangle;

extern uint8  cameraFlag;

void timer_pit_init(void);
void Balance_Init(void);
#endif
