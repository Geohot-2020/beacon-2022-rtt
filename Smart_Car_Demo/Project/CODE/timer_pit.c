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
 * @LastEditTime: 2021-11-26 16:55:32
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
	Direct_Last = 0;

float Target_Angle_min=-100,       //�ܶ�ǰ�����Ƕ�
      Target_Angle_max=3;       //�ܶ��������Ƕ�

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
     * @description: ���ٶȻ�2ms
     * @param {*}
     * @return {*}
     * @author: ֣�в�
     */    
    if(0 == (time%2))
    {
        ICM20602_GetData(&GYRO, &ACC);  
        Data_Filter();  //ԭʼ�����˲�
        accangle = asin(ACC_Real.Y);    
        KalmanFilter(accangle);     //��̬���㣬�������˲�
        // printf("%.2f, %.2f\n",accangle, Attitude_Angle.Y);
        Theory_Duty -= PID_Increase(&Ang_Vel_PID, Ang_Vel, (int32)GYRO_Real.Y*10, (int32)(Tar_Ang_Vel.Y)); //����ʽPID
        Theory_Duty = range_protect(Theory_Duty, -10000, 10000);  //�޷�

            /*--------------------ת��------------------*/
        
        Left_MOTOR_Duty = Theory_Duty;
        Right_MOTOR_Duty = Theory_Duty;

         motor_control(-Left_MOTOR_Duty, -Right_MOTOR_Duty);
        //motor_control(3000, 3000);
    }

    /**
     * @description: �ǶȻ�10ms
     * @param {*}
     * @return {*}
     * @author: ֣�в�
     */  
    if(0 == (time%10))
    {
        encoder_get();      //��ȡ��ǰ�ٶ�

        Tar_Ang_Vel.Y = -PID_Realize(&Angle_PID, Angle, (int32)(accangle*100), (int32)Target_Angle.Y);      //����ʽPID
        Tar_Ang_Vel.Y = range_protect(Tar_Ang_Vel.Y, -8000, 8000);
    }



    // static uint32 time;
    // time++;
    // /*-------------------------������ƽ��ٶȻ�1ms---------------------------*/
    // ICM20602_GetData(&GYRO, &ACC);  //��ȡ����������
    // Data_Filter();			        //��ԭʼ���ݻ����˲�
    
    // /* ���ٶȻ������ڻ�����ֱ�� */
    // Theory_Duty += -PID_Increase(&Ang_Vel_PID, Ang_Vel, (int32)(GYRO_Real.Y*10), (int32)(Tar_Ang_Vel.Y));    //����ֱ��PWM
    // Theory_Duty = range_protect(Theory_Duty, -5000, 5000);

    // // Direct_Last = Direct_Last*0.2 + Direct_Parameter*0.8;       //������һ�εĽ��ٶȻ�


    // motor_control(Left_MOTOR_Duty, Right_MOTOR_Duty);
	// 	//motor_control(-3000,3000);
    // //��̬����
    // accangle = asin(ACC_Real.Y);
    // KalmanFilter(accangle);
    // //printf("%.2f, %.2f\n",accangle, Attitude_Angle.Y);

    // if(0 == (time%100))
    // {
    //     //-------------------------------------------------�ɼ�����������
    //     encoder_get();
			
    //     //-------------------------------------------------��������ٶȻ�
    // }

    // if(0 == (time%5))
    // {
    //     //-------------------------------------------------�ɼ����ٶ�����
    //     // get_icm20602_accdata_spi();

    //     // //�ɼ�����ź�
    //     // elec_get();
        
    //     // //���ݵ���źż��㳵��λ��
    //     // elec_calculate();
			
	// 			//-------------------------------------------------��ͼ����������λ�ã����г�ģ����
	// 			//-------------------------------------------------�����˲�
	// 			//-------------------------------------------------������ƽǶȻ�
				
    // }
    
		
    // //���Ƶ��ת��
    // // motor_control(10000, 10000); 

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
    Target_Angle.Y = -55;
    Tar_Ang_Vel.Y = 0;
    Tar_Ang_Vel.Z = 0;
}


