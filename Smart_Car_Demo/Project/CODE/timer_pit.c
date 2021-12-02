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
 * @Description: ��Ҫ���ƴ���
 * @Version: v1.0
 * @Autor: ֣�в�
 * @Date: 2021-11-24 14:46:45
 * @LastEditors: ֣�в�
 * @LastEditTime: 2021-11-30 16:24:11
 */

#include "headfile.h"


S_FLOAT_XYZ 
	GYRO_Real,		    //������ת���������
	ACC_Real,		    //���ٶȼ�ת���������
	Attitude_Angle,    	//��ǰ�Ƕ�
	Last_Angle,		    //�ϴνǶ�
    Target_Angle,	    //Ŀ��Ƕ�
        Target_Angle_Grow;      //Ŀ��Ƕ�����

S_INT16_XYZ
	GYRO,			    // ������ԭʼ����
	GYRO_Offset,	    // ��������Ʈ
	GYRO_Last,		    // �������ϴ�����
	ACC, 			    // ���ٶȼ�����
	ACC_Offset,		    // ���ٶȼ���Ʈ
	ACC_Last;		    // ���ٶȼ��ϴ�����

S_INT32_XYZ
	Tar_Ang_Vel,	    // Ŀ����ٶ�
	Tar_Ang_Vel_Last,	// �ϴ�Ŀ����ٶ�
        Tar_Ang_Vel_Grow;       // Ŀ����ٶ�����

int32 
	Speed_Now = 0,	        // ��ǰʵ���ٶ�
        Speed_Now_Last=0,       // �ϴ�ʵ���ٶ�
	Speed_Min = 0,	        // ������С�ٶ�
	Speed_Set = 0, 	        // Ŀ���趨�ٶ�
	Theory_Duty = 0,        // ����ֱ��ռ�ձ�
	Vel_Set = 0,	        // Ŀ��ת����ٶ�
    Direct_Parameter = 0,   // ת��ϵ��
	Direct_Last = 0,
    Speed_dif = 0;

float Target_Angle_min=-75,       //�ܶ�ǰ�����Ƕ�
      Target_Angle_max=-65;       //�ܶ��������Ƕ�

float accangle;


/**
 * @description: ����PID
 * @param {void} *parameter
 * @return {*}
 * @author: ֣�в�
 */
void timer1_pit_entry(void *parameter)
{

    static uint32 time;
    time++;

    /**
     * @description: ���ٶȻ�2ms����PI��i�����Ƶ���ػΣ������Ƶ��������ǿ��ס����p����΢������û����Ƶ�񶯣�����������
     * @param {*}
     * @return {*}
     * @author: ֣�в�
     */    
    if(0 == (time%2))
    {
        ICM20602_GetData(&GYRO, &ACC);  
        Data_Filter();  //ԭʼ�����˲�
        accangle = asin(ACC_Real.X);    
        KalmanFilter(accangle);     //��̬���㣬�������˲�
        //printf("%.2f, %.2f\n",accangle, Attitude_Angle.Y);
        // seekfree_wireless_send_buff((uint8 *)"\r\nSEEKFREE wireless test.", 25);		// ���Ͳ�����Ϣ
        Theory_Duty += -PID_Increase(&Ang_Vel_PID, Ang_Vel, (int32)GYRO_Real.Y*10, (int32)(Tar_Ang_Vel.Y)); //����ʽPID
        Theory_Duty = range_protect(Theory_Duty, -18000, 18000);  //�޷�

        /*--------------------ת�򻷣��ý��ٶȻ����ڻ�����ת��------------------*/
            Direct_Parameter = -PID_Realize(&Direct_PID, Direct, (int32)GYRO_Real.Z, Speed_Min);
            Direct_Parameter = range_protect(Direct_Parameter, -18000, 18000);
        Direct_Last = Direct_Last*0.2 + Direct_Parameter*0.8;
        
        Left_MOTOR_Duty = Theory_Duty - Direct_Last ;
        Right_MOTOR_Duty = Theory_Duty + Direct_Last;

        motor_control(-Left_MOTOR_Duty, -Right_MOTOR_Duty);
        //motor_control(0, 7000);
    }

    /**
     * @description: �ǶȻ�10ms����PD��������ס������ǰհ�����󣬳�����ǰ�壻d��Ӧ�ٶȣ�������ǰ�����ܶ࣬���󶼻��Ƶ����
     * @param {*}
     * @return {*}
     * @author: ֣�в�
     */  
    if(0 == (time%10))
    {
        encoder_get();      //��ȡ��ǰ�ٶ�
        Tar_Ang_Vel.Y = PID_Realize(&Angle_PID, Angle, (int32)(accangle*100), (int32)Target_Angle.Y);      //����ʽPID
        Tar_Ang_Vel.Y = range_protect(Tar_Ang_Vel.Y, -15000, 15000);
    }

    /**
     * @description: �ٶȻ�50ms����P��
     * @param {*}
     * @return {*}
     * @author: ֣�в�
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
    
    //����һ����ʱ�� ��������
    timer = rt_timer_create("timer1", timer1_pit_entry, RT_NULL, 1, RT_TIMER_FLAG_PERIODIC);
    
    //������ʱ��
    if(RT_NULL != timer)
    {
        rt_timer_start(timer);
    }

    
}

/* ��ʼ���õ���һЩ���� */
void Balance_Init(void)
{
    Attitude_Angle.X = 0;
    Target_Angle.Y = 0;
    Tar_Ang_Vel.Y = 0;
    Tar_Ang_Vel.Z = 0;
		Speed_Set = 0;
}


