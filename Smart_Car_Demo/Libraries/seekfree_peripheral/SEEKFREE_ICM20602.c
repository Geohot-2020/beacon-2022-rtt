/*********************************************************************************************************************
* COPYRIGHT NOTICE
* Copyright (c) 2018,��ɿƼ�
* All rights reserved.
* ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897
*
* �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
* ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
*
* @file				SEEKFREE_ICM20602
* @company			�ɶ���ɿƼ����޹�˾
* @author			��ɿƼ�(QQ3184284598)
* @version			�鿴doc��version�ļ� �汾˵��
* @Software			IAR 8.3 or MDK 5.24
* @Taobao			https://seekfree.taobao.com/
* @date				2020-11-23
* @note
* 					���߶��壺
* 					------------------------------------
* 					���IICͨ��
* 					ģ��ܽ�			��Ƭ���ܽ�
* 					SCL					�鿴 SEEKFREE_ICM20602 �ļ��ڵ� SEEKFREE_SCL�궨��
* 					SDA					�鿴 SEEKFREE_ICM20602 �ļ��ڵ� SEEKFREE_SDA�궨��
* 					Ӳ��SPIͨ��
* 										�鿴 SEEKFREE_ICM20602 �ļ��ڵ� ICM20602_SCK_PIN �궨��
* 										�鿴 SEEKFREE_ICM20602 �ļ��ڵ� ICM20602_MOSI_PIN �궨��
* 										�鿴 SEEKFREE_ICM20602 �ļ��ڵ� ICM20602_MISO_PIN �궨��
* 										�鿴 SEEKFREE_ICM20602 �ļ��ڵ� ICM20602_CS_PIN �궨��
* 					------------------------------------
********************************************************************************************************************/

#include "zf_systick.h"
#include "zf_gpio.h"
#include "zf_spi.h"
#include "SEEKFREE_IIC.h"
#include "SEEKFREE_ICM20602.h"

int16 icm_gyro_x,icm_gyro_y,icm_gyro_z;
int16 icm_acc_x,icm_acc_y,icm_acc_z;
char Offset_OK = 0;
float Zero_Angle;
//-------------------------------------------------------------------------------------------------------------------
// ���º�����ʹ�����IICͨ��
//-------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
// @brief		ICM20602�Լ캯��
// @param		NULL
// @return		void
// @since		v1.0
// Sample usage:
// @note		�ڲ����� �û��������
//-------------------------------------------------------------------------------------------------------------------
static void icm20602_self1_check(void)
{
	uint8 dat=0;
	while(0x12 != dat)																// �ж� ID �Ƿ���ȷ
	{
		dat = simiic_read_reg(ICM20602_DEV_ADDR,ICM20602_WHO_AM_I,SIMIIC);			// ��ȡICM20602 ID
		systick_delay_ms(10);
		//��������ԭ�������¼���
		//1 ICM20602���ˣ�������µ������ĸ��ʼ���
		//2 ���ߴ������û�нӺ�
		//3 ��������Ҫ����������裬������3.3V
		//4 ����û�е���ģ��IIC�ĳ�ʼ������
	}
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		��ʼ��ICM20602
// @param		NULL
// @return		void
// @since		v1.0
// Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void icm20602_init(void)
{
	simiic_init();
	systick_delay_ms(10);  //�ϵ���ʱ

	//���
	icm20602_self1_check();

	//��λ
	simiic_write_reg(ICM20602_DEV_ADDR,ICM20602_PWR_MGMT_1,0x80);					// ��λ�豸
	systick_delay_ms(2);															// ��ʱ
	while(0x80 & simiic_read_reg(ICM20602_DEV_ADDR,ICM20602_PWR_MGMT_1,SIMIIC));	// �ȴ���λ���

	//���ò���
	simiic_write_reg(ICM20602_DEV_ADDR,ICM20602_PWR_MGMT_1,0x01);					// ʱ������
	simiic_write_reg(ICM20602_DEV_ADDR,ICM20602_PWR_MGMT_2,0x00);					// ���������Ǻͼ��ٶȼ�
	simiic_write_reg(ICM20602_DEV_ADDR,ICM20602_CONFIG,0x01);						// 176HZ 1KHZ
	simiic_write_reg(ICM20602_DEV_ADDR,ICM20602_SMPLRT_DIV,0x07);					// �������� SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)
	simiic_write_reg(ICM20602_DEV_ADDR,ICM20602_GYRO_CONFIG,0x18);					// ��2000 dps
	simiic_write_reg(ICM20602_DEV_ADDR,ICM20602_ACCEL_CONFIG,0x10);					// ��8g
	simiic_write_reg(ICM20602_DEV_ADDR,ICM20602_ACCEL_CONFIG_2,0x03);				// Average 4 samples   44.8HZ   //0x23 Average 16 samples
}


//-------------------------------------------------------------------------------------------------------------------
// @brief		��ȡICM20602���ٶȼ�����
// @param		NULL
// @return		void
// @since		v1.0
// Sample usage:				ִ�иú�����ֱ�Ӳ鿴��Ӧ�ı�������
//-------------------------------------------------------------------------------------------------------------------
void get_icm20602_accdata(void)
{
	uint8 dat[6];

	simiic_read_regs(ICM20602_DEV_ADDR, ICM20602_ACCEL_XOUT_H, dat, 6, SIMIIC);  
	icm_acc_x = (int16)(((uint16)dat[0]<<8 | dat[1]));
	icm_acc_y = (int16)(((uint16)dat[2]<<8 | dat[3]));
	icm_acc_z = (int16)(((uint16)dat[4]<<8 | dat[5]));
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		��ȡICM20602����������
// @param		NULL
// @return		void
// @since		v1.0
// Sample usage:				ִ�иú�����ֱ�Ӳ鿴��Ӧ�ı�������
//-------------------------------------------------------------------------------------------------------------------
void get_icm20602_gyro(void)
{
	uint8 dat[6];

	simiic_read_regs(ICM20602_DEV_ADDR, ICM20602_GYRO_XOUT_H, dat, 6, SIMIIC);  
	icm_gyro_x = (int16)(((uint16)dat[0]<<8 | dat[1]));
	icm_gyro_y = (int16)(((uint16)dat[2]<<8 | dat[3]));
	icm_gyro_z = (int16)(((uint16)dat[4]<<8 | dat[5]));
}

//-------------------------------------------------------------------------------------------------------------------
// ���Ϻ�����ʹ�����IICͨ��
//-------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
// ���º�����ʹ��Ӳ��SPIͨ�� ��Ƚ�IIC �ٶȱ�IIC��ǳ���
//-------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
// @brief		ICM20602 SPIд�Ĵ���
// @param		cmd				�Ĵ�����ַ
// @param		val				��Ҫд�������
// @return		void
// @since		v1.0
// Sample usage:
// @note		�ڲ����� �û��������
//-------------------------------------------------------------------------------------------------------------------
static void icm_spi_w_reg_byte(uint8 cmd, uint8 val)
{
	uint8 dat[2];
	ICM20602_CS(0);
	dat[0] = cmd | ICM20602_SPI_W;
	dat[1] = val;

	spi_mosi(ICM20602_SPI, dat, dat, 2);
	ICM20602_CS(1);
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		ICM20602 SPI���Ĵ���
// @param		cmd				�Ĵ�����ַ
// @param		*val			�������ݵĵ�ַ
// @return		void
// @since		v1.0
// Sample usage:
// @note		�ڲ����� �û��������
//-------------------------------------------------------------------------------------------------------------------
static void icm_spi_r_reg_byte(uint8 cmd, uint8 *val)
{
	uint8 dat[2];
	ICM20602_CS(0);
	dat[0] = cmd | ICM20602_SPI_R;
	dat[1] = *val;

	spi_mosi(ICM20602_SPI, dat, dat, 2);

	*val = dat[1];
	ICM20602_CS(1);
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		ICM20602 SPI���ֽڶ��Ĵ���
// @param		cmd				�Ĵ�����ַ
// @param		*val			�������ݵĵ�ַ
// @param		num				��ȡ����
// @return		void
// @since		v1.0
// Sample usage:
// @note		�ڲ����� �û��������
//-------------------------------------------------------------------------------------------------------------------
static void icm_spi_r_reg_bytes(uint8 * val, uint8 num)
{
	ICM20602_CS(0);
	spi_mosi(ICM20602_SPI, val, val, num);
	ICM20602_CS(1);
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		ICM20602�Լ캯��
// @param		NULL
// @return		void
// @since		v1.0
// Sample usage:
// @note		�ڲ����� �û��������
//-------------------------------------------------------------------------------------------------------------------
static void icm20602_self3_check(void)
{
	uint8 dat = 0;

	while(0x12 != dat)																// �ж� ID �Ƿ���ȷ
	{
		icm_spi_r_reg_byte(ICM20602_WHO_AM_I, &dat);								// ��ȡICM20602 ID
		systick_delay_ms(10);
		//��������ԭ�������¼���
		//1 ICM20602���ˣ�������µ������ĸ��ʼ���
		//2 ���ߴ������û�нӺ�
		//3 ��������Ҫ����������裬������3.3V
	}
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		��ʼ��ICM20602
// @param		NULL
// @return		void
// @since		v1.0
// Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void icm20602_init_spi(void)
{
	uint8 val = 0x0;

	systick_delay_ms(10);  //�ϵ���ʱ

	spi_init(ICM20602_SPI, ICM20602_SCK_PIN, ICM20602_MOSI_PIN, ICM20602_MISO_PIN, SPI_NSS_NULL, 0, SystemCoreClock/4);	// Ӳ��SPI��ʼ��

	gpio_init(ICM20602_CS_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);

	icm20602_self3_check();//���

	icm_spi_w_reg_byte(ICM20602_PWR_MGMT_1,0x80);//��λ�豸
	systick_delay_ms(2);
	do																				// �ȴ���λ�ɹ�
	{
		icm_spi_r_reg_byte(ICM20602_PWR_MGMT_1,&val);
	}while(0x41 != val);

	icm_spi_w_reg_byte(ICM20602_PWR_MGMT_1,		0x01);								// ʱ������
	icm_spi_w_reg_byte(ICM20602_PWR_MGMT_2,		0x00);								// ���������Ǻͼ��ٶȼ�
	icm_spi_w_reg_byte(ICM20602_CONFIG,			0x01);								// 176HZ 1KHZ
	icm_spi_w_reg_byte(ICM20602_SMPLRT_DIV,		0x07);								// �������� SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)
	icm_spi_w_reg_byte(ICM20602_GYRO_CONFIG,	0x18);								// ��2000 dps
	icm_spi_w_reg_byte(ICM20602_ACCEL_CONFIG,	0x10);								// ��8g
	icm_spi_w_reg_byte(ICM20602_ACCEL_CONFIG_2,	0x03);								// Average 4 samples   44.8HZ   //0x23 Average 16 samples
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		��ȡICM20602���ٶȼ�����
// @param		NULL
// @return		void
// @since		v1.0
// Sample usage:				ִ�иú�����ֱ�Ӳ鿴��Ӧ�ı�������
//-------------------------------------------------------------------------------------------------------------------
void get_icm20602_accdata_spi(void)
{
	struct
	{
		uint8 reg;
		uint8 dat[6];
	}buf;

	buf.reg = ICM20602_ACCEL_XOUT_H | ICM20602_SPI_R;

	icm_spi_r_reg_bytes(&buf.reg, 7);
	icm_acc_x = (int16)(((uint16)buf.dat[0]<<8 | buf.dat[1]));
	icm_acc_y = (int16)(((uint16)buf.dat[2]<<8 | buf.dat[3]));
	icm_acc_z = (int16)(((uint16)buf.dat[4]<<8 | buf.dat[5]));
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		��ȡICM20602����������
// @param		NULL
// @return		void
// @since		v1.0
// Sample usage:				ִ�иú�����ֱ�Ӳ鿴��Ӧ�ı�������
//-------------------------------------------------------------------------------------------------------------------
void get_icm20602_gyro_spi(void)
{
	struct
	{
		uint8 reg;
		uint8 dat[6];
	}buf;

	buf.reg = ICM20602_GYRO_XOUT_H | ICM20602_SPI_R;

	icm_spi_r_reg_bytes(&buf.reg, 7);
	icm_gyro_x = (int16)(((uint16)buf.dat[0]<<8 | buf.dat[1]));
	icm_gyro_y = (int16)(((uint16)buf.dat[2]<<8 | buf.dat[3]));
	icm_gyro_z = (int16)(((uint16)buf.dat[4]<<8 | buf.dat[5]));
}

//-------------------------------------------------------------------------------------------------------------------
// ���Ϻ�����ʹ��Ӳ��SPIͨ�� ��Ƚ�IIC �ٶȱ�IIC��ǳ���
//-------------------------------------------------------------------------------------------------------------------

#define AcceRatio 	4096.0f
#define GyroRatio 	16.4f
#define Gyro_Gr		0.0010653	// ���ٶȱ�ɻ���	�˲�����Ӧ����2000��ÿ��
#define ACC_FILTER_NUM 5		// ���ٶȼ��˲����
#define GYRO_FILTER_NUM 1		// �������˲����
int32 ACC_X_BUF[ACC_FILTER_NUM], ACC_Y_BUF[ACC_FILTER_NUM], ACC_Z_BUF[ACC_FILTER_NUM];	// �˲���������
int32 GYRO_X_BUF[GYRO_FILTER_NUM], GYRO_Y_BUF[GYRO_FILTER_NUM], GYRO_Z_BUF[GYRO_FILTER_NUM];
/*
 * ��������Data_Filter
 * ����  �����ݻ����˲�
 * ����  ����
 * ���  ����
 * ����  ���ⲿ����
 */
void Data_Filter(void)	// �����˲�
{
	uint8 i;
	float ACC_Angle;
	int64 temp1 = 0, temp2 = 0, temp3 = 0, temp4 = 0, temp5 = 0, temp6 = 0;
	ACC_X_BUF[0] = ACC.X;	// ���»�����������
	ACC_Y_BUF[0] = ACC.Y;
	ACC_Z_BUF[0] = ACC.Z;
	GYRO_X_BUF[0] = GYRO.X;
	GYRO_Y_BUF[0] = GYRO.Y;
	GYRO_Z_BUF[0] = GYRO.Z;
	
	for(i=0;i<ACC_FILTER_NUM;i++)
	{
		temp1 += ACC_X_BUF[i];
		temp2 += ACC_Y_BUF[i];
		temp3 += ACC_Z_BUF[i];
		
	}
	for(i=0;i<GYRO_FILTER_NUM;i++)
	{
		temp4 += GYRO_X_BUF[i];
		temp5 += GYRO_Y_BUF[i];
		temp6 += GYRO_Z_BUF[i];
	}
	
	ACC_Real.X = temp1 / ACC_FILTER_NUM / AcceRatio;
	ACC_Real.Y = temp2 / ACC_FILTER_NUM / AcceRatio;
	ACC_Real.Z = temp3 / ACC_FILTER_NUM / AcceRatio;
	GYRO_Real.X = temp4 / GYRO_FILTER_NUM / GyroRatio;
	GYRO_Real.Y = temp5 / GYRO_FILTER_NUM / GyroRatio;
	GYRO_Real.Z = temp6 / GYRO_FILTER_NUM / GyroRatio;
	
	for(i = 0; i < ACC_FILTER_NUM - 1; i++)
	{
		ACC_X_BUF[ACC_FILTER_NUM-1-i] = ACC_X_BUF[ACC_FILTER_NUM-2-i];
		ACC_Y_BUF[ACC_FILTER_NUM-1-i] = ACC_Y_BUF[ACC_FILTER_NUM-2-i];
		ACC_Z_BUF[ACC_FILTER_NUM-1-i] = ACC_Z_BUF[ACC_FILTER_NUM-2-i];
		
	}
	for(i = 0; i < GYRO_FILTER_NUM - 1; i++)
	{
		GYRO_X_BUF[GYRO_FILTER_NUM-1-i] = GYRO_X_BUF[GYRO_FILTER_NUM-2-i];
		GYRO_Y_BUF[GYRO_FILTER_NUM-1-i] = GYRO_Y_BUF[GYRO_FILTER_NUM-2-i];
		GYRO_Z_BUF[GYRO_FILTER_NUM-1-i] = GYRO_Z_BUF[GYRO_FILTER_NUM-2-i];
	}
}


//�Ǿ��󿨶����˲�����Щ�������ø�
#define Peried 1/500.0f		//��������������
#define Q 2000.0f				//��������2.0		ԽС����Խ�������ټ��ٶȼ�Խ��Խƽ��
#define R 2000.0f			//��������5000.0	ԽС���ټ��ٶȼ�Խ��
float KalmanGain = 1.0f;	//����������

void KalmanFilter(float ACC_Angle)
{
	//�������˲��ֲ�����
    static float Priori_Estimation = 0;//�������
    static float Posterior_Estimation = 0;//�������
    static float Priori_Convariance = 0;//���鷽��
    static float Posterior_Convariance = 0;//���鷽��
		
	//�������˲�
    //1.ʱ�����(Ԥ��) : X(k|k-1) = A(k,k-1)*X(k-1|k-1) + B(k)*u(k) 
    Priori_Estimation = Posterior_Estimation - GYRO_Real.X*Peried;		//������ƣ����ֻ�ýǶ�
	if (Priori_Estimation != Priori_Estimation)
	{
		Priori_Estimation = 0;
	}
	
    //2.��������Э����  : P(k|k-1) = A(k,k-1)*P(k-1|k-1)*A(k,k-1)'+Q(k) 
    Priori_Convariance = (float)sqrt( Posterior_Convariance * Posterior_Convariance + Q * Q );
	if (Priori_Convariance != Priori_Convariance)
	{
		Priori_Convariance = 0;
	}
	
    //  ������������ƣ���������  
    // 1.���㿨��������  : K(k) = P(k|k-1)*H(k)' / (H(k)*P(k|k-1)*H(k)' + R(k)) /
    KalmanGain = (float)sqrt( Priori_Convariance * Priori_Convariance / ( Priori_Convariance * Priori_Convariance + R * R ) );
	if (KalmanGain != KalmanGain)
	{
		KalmanGain = 1;
	}
	
    //2.��������(У��): X(k|k) = X(k|k-1)+K(k)*(Z(k)-H(k)*X(k|k-1)) 
    Posterior_Estimation  = Priori_Estimation + KalmanGain * (ACC_Angle - Priori_Estimation );
	if (Posterior_Estimation != Posterior_Estimation)
	{
		Posterior_Estimation = 0;
	}
	
    // 3.���º���Э����  : P(k|k) =��I-K(k)*H(k)��*P(k|k-1) 
    Posterior_Convariance = (float)sqrt(( 1 - KalmanGain ) * Priori_Convariance * Priori_Convariance );
	if (Posterior_Convariance != Posterior_Convariance)
	{
		Posterior_Convariance = 0;
	}
	
    //�õ����սǶ� 
    Attitude_Angle.Y = Posterior_Estimation;
	
	if (Attitude_Angle.Y != Attitude_Angle.Y)
	{
		Attitude_Angle.Y = 1;
	}
}

/*
 * ��������Get_Attitude
 * ����  ����̬����
 * ����  ����
 * ���  ����
 * ����  ���ⲿ����
 */
void Get_Attitude(void)	// ��̬����
{
	IMUupdate( GYRO_Real.Y*Gyro_Gr*GyroRatio, 
                          GYRO_Real.X*Gyro_Gr*GyroRatio,
			  GYRO_Real.Z*Gyro_Gr*GyroRatio, 
			  ACC_Real.X * AcceRatio, 
			  ACC_Real.Y * AcceRatio, 
			  ACC_Real.Z * AcceRatio);	// ��̬�����ŷ����
//һ�׻����˲�

}


//===============================��Ԫ��============================================
#define Kp 1.6f //10.0f             	// ����������Ƽ��ٶȼ�/�����Ƶ���������
#define Ki 0.001f//1.2f // //0.008f  	// �����������������ƫ���������
#define halfT 0.001f                   	// �������ڵ�һ��
float q0 = 1, q1 = 0, q2 = 0, q3 = 0; 	// ������Ʒ����������Ԫ��
float exInt = 0, eyInt = 0, ezInt = 0; 	// ���Ż��ִ���
/*
 * ��������IMUupdate
 * ����  ����Ԫ�ؽ���ŷ����
 * ����  �������� ���ٶȼ�
 * ���  ����
 * ����  ���ڲ�����
 */
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
	float norm;
	float vx, vy, vz;
	float ex, ey, ez;

	// �Ȱ���Щ�õõ���ֵ���
	float q0q0 = q0*q0;
	float q0q1 = q0*q1;
	float q0q2 = q0*q2;
	float q1q1 = q1*q1;
	float q1q3 = q1*q3;
	float q2q2 = q2*q2;
	float q2q3 = q2*q3;
	float q3q3 = q3*q3;

	if (ax*ay*az == 0)
	{
		return;
	}
		
	norm = sqrt(ax*ax + ay*ay + az*az);	// acc���ݹ�һ��
	ax = ax / norm;
	ay = ay / norm;
	az = az / norm;

	// estimated direction of gravity and flux (v and w)	�����������������/��Ǩ
	vx = 2*(q1q3 - q0q2);									// ��Ԫ����xyz�ı�ʾ
	vy = 2*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3 ;

	// error is sum of cross product between reference direction of fields and direction measured by sensors
	ex = (ay*vz - az*vy) ;		// �������������õ���־������
	ey = (az*vx - ax*vz) ;
	ez = (ax*vy - ay*vx) ;

	exInt = exInt + ex * Ki;	// �������л���
	eyInt = eyInt + ey * Ki;
	ezInt = ezInt + ez * Ki;

	// adjusted gyroscope measurements
	gx = gx + Kp*ex + exInt;	// �����PI�󲹳��������ǣ����������Ư��
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;	// �����gz����û�й۲��߽��н��������Ư�ƣ����ֳ����ľ��ǻ����������Լ�

	// integrate quaternion rate and normalise	// ��Ԫ�ص�΢�ַ���
	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
	q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
	q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
	q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

	// normalise quaternion
	norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 / norm;
	q1 = q1 / norm;
	q2 = q2 / norm;
	q3 = q3 / norm;
	
	Attitude_Angle.Y = asin(-2*q1*q3 + 2*q0*q2) * 57.3; // pitch
//	Attitude_Angle.X = atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1)*57.3; // roll
//	Attitude_Angle.Z = atan2(2*q1*q2 + 2*q0*q3, -2*q2*q2 - 2*q3*q3 + 1)*57.3; // yaw
//	Attitude_Angle.Z = 0;
}
/*
 * ��������ICM20602_Offset
 * ����  ���������ɼ���ƫ
 * ����  ����
 * ���  ����
 * ����  ���ڲ�����
 */
void ICM20602_Offset(void)
{
	uint8 i, Count = 100;
	int64 temp[6] = {0};
	
	GYRO_Offset.X = 0;
	GYRO_Offset.Y = 0;
	GYRO_Offset.Z = 0;
	
	for (i = 0; i < Count; i++)
	{
		ICM20602_GetData(&GYRO, &ACC);	// ��ȡ����������
		rt_thread_mdelay(2);
		
		temp[0] += ACC.X;
		temp[1] += ACC.Y;
		temp[2] += ACC.Z;
		
		temp[3] += GYRO.X;
		temp[4] += GYRO.Y;
		temp[5] += GYRO.Z;
	}
	ACC_Offset.X = temp[0] / Count;
	ACC_Offset.Y = temp[1] / Count;
	ACC_Offset.Z = temp[2] / Count;
	
	GYRO_Offset.X = temp[3] / Count;
	GYRO_Offset.Y = temp[4] / Count;
	GYRO_Offset.Z = temp[5] / Count;
	
	Offset_OK = 1;
}

/*
 * ��������ICM20602_GetData
 * ����  ����ô�������������
 * ����  ��*GYRO ������		*ACC ���ٶȼ�
 * ���  ����
 * ����  ���ⲿ����
 */
void ICM20602_GetData(S_INT16_XYZ *GYRO, S_INT16_XYZ *ACC)
{
  get_icm20602_gyro_spi();
  get_icm20602_accdata_spi();
	if (Offset_OK)
	{
          
		ACC->X = icm_acc_x;	// ��ȡ���ٶȼ�ԭʼ����
		ACC->Y = icm_acc_y;
		ACC->Z = icm_acc_z;
		
		GYRO->X = -(icm_gyro_x + GYRO_Offset.X);	// ��ȡ������ԭʼ����
		GYRO->Y = -(icm_gyro_y + GYRO_Offset.Y);
		GYRO->Z = -(icm_gyro_z + GYRO_Offset.Z);
	}
	else
	{
		ACC->X = icm_acc_x;	// ��ȡ���ٶȼ�ԭʼ����
		ACC->Y = icm_acc_y;
		ACC->Z = icm_acc_z;
		
		GYRO->X = -icm_gyro_x;	// ��ȡ������ԭʼ���ݲ���һ��
		GYRO->Y = -icm_gyro_y;
		GYRO->Z = -icm_gyro_z;
	}
}

void ICM20602_Init()
{
    Zero_Angle = -0.71;
  Target_Angle.Y = Zero_Angle*100;
  icm20602_init_spi();            //�����ǳ�ʼ��
  rt_thread_mdelay(1000);
  ICM20602_Offset();
  
  for (int i = 0; i < 200; i++)
  {
    for (int j = 0; j < 5; j++)
      {
      ICM20602_GetData(&GYRO, &ACC);	// ��ȡ����������
      Data_Filter();					// ��ԭʼ���ݻ����˲�
//      Get_Attitude();	// ��̬����
      }
//    Get_Attitude();	// ��̬����
    accangle = asin(ACC_Real.Y);
    KalmanFilter(accangle);
    //Tar_Ang_Vel.Y = PID_Realize(&Angle_PID, Angle, (int32)(Attitude_Angle.Y*100), Zero_Angle*100);
    
  }
}

/**
 * @description: ������˹��ͨ�˲�,�����ã�д����һ����ȥ
 * @param {*}
 * @return {*}
 * @author: ֣�в�
 */

typedef struct
{
 //volatile 
   float Input_Butter[3];
 //volatile 
   float Output_Butter[3];
}Butter_BufferData;


typedef struct
{
  float a[3];
  float b[3];
}Butter_Parameter;


float LPButterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter)

{
	static int LPB_Cnt=0;

	/* ���ٶȼ�Butterworth�˲� */

	/* ��ȡ����x(n) */

	Buffer->Input_Butter[2]=curr_input;

	if(LPB_Cnt>=500)
	{
		/* Butterworth�˲� */
		Buffer->Output_Butter[2]=
		Parameter->b[0] * Buffer->Input_Butter[2]
		+Parameter->b[1] * Buffer->Input_Butter[1]
		+Parameter->b[2] * Buffer->Input_Butter[0]
		-Parameter->a[1] * Buffer->Output_Butter[1]
		-Parameter->a[2] * Buffer->Output_Butter[0];
	}
	else
	{
		Buffer->Output_Butter[2]=Buffer->Input_Butter[2];
		LPB_Cnt++;
	}

	/* x(n) ���б��� */
	Buffer->Input_Butter[0]=Buffer->Input_Butter[1];
	Buffer->Input_Butter[1]=Buffer->Input_Butter[2];

	/* y(n) ���б��� */
	Buffer->Output_Butter[0]=Buffer->Output_Butter[1];
	Buffer->Output_Butter[1]=Buffer->Output_Butter[2];

	return (Buffer->Output_Butter[2]);
}

