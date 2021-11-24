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


//����PID
void timer1_pit_entry(void *parameter)
{
    static uint32 time;
    time++;
    /*-------------------------������ƽ��ٶȻ�1ms---------------------------*/
    ICM20602_GetData(&GYRO, &ACC);  //��ȡ����������
    Data_Filter();			        //��ԭʼ���ݻ����˲�
    /* ���ٶȻ������ڻ�����ֱ�� */


    //��̬����
    accangle = asin(ACC_Real.Y);
    KalmanFilter(accangle);
    //printf("%.2f, %.2f\n",accangle, Attitude_Angle.Y);

    if(0 == (time%100))
    {
        //-------------------------------------------------�ɼ�����������
        encoder_get();
			
        //-------------------------------------------------��������ٶȻ�
    }

    if(0 == (time%5))
    {
        //-------------------------------------------------�ɼ����ٶ�����
        // get_icm20602_accdata_spi();

        // //�ɼ�����ź�
        // elec_get();
        
        // //���ݵ���źż��㳵��λ��
        // elec_calculate();
			
				//-------------------------------------------------��ͼ����������λ�ã����г�ģ����
				//-------------------------------------------------�����˲�
				//-------------------------------------------------������ƽǶȻ�
				
    }
    
		
    //���Ƶ��ת��
    motor_control(10000, 10000);
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
    Target_Angle.X = 0;
    Tar_Ang_Vel.Y = 0;
    Tar_Ang_Vel.Z = 0;
}

INIT_APP_EXPORT(Balance_Init);
