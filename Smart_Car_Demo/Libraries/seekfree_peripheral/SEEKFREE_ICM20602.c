/*********************************************************************************************************************
* COPYRIGHT NOTICE
* Copyright (c) 2018,逐飞科技
* All rights reserved.
* 技术讨论QQ群：一群：179029047(已满)  二群：244861897
*
* 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
* 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
*
* @file				SEEKFREE_ICM20602
* @company			成都逐飞科技有限公司
* @author			逐飞科技(QQ3184284598)
* @version			查看doc内version文件 版本说明
* @Software			IAR 8.3 or MDK 5.24
* @Taobao			https://seekfree.taobao.com/
* @date				2020-11-23
* @note
* 					接线定义：
* 					------------------------------------
* 					软件IIC通信
* 					模块管脚			单片机管脚
* 					SCL					查看 SEEKFREE_ICM20602 文件内的 SEEKFREE_SCL宏定义
* 					SDA					查看 SEEKFREE_ICM20602 文件内的 SEEKFREE_SDA宏定义
* 					硬件SPI通信
* 										查看 SEEKFREE_ICM20602 文件内的 ICM20602_SCK_PIN 宏定义
* 										查看 SEEKFREE_ICM20602 文件内的 ICM20602_MOSI_PIN 宏定义
* 										查看 SEEKFREE_ICM20602 文件内的 ICM20602_MISO_PIN 宏定义
* 										查看 SEEKFREE_ICM20602 文件内的 ICM20602_CS_PIN 宏定义
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
//-------------------------------------------------------------------------------------------------------------------
// 以下函数是使用软件IIC通信
//-------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
// @brief		ICM20602自检函数
// @param		NULL
// @return		void
// @since		v1.0
// Sample usage:
// @note		内部调用 用户无需关心
//-------------------------------------------------------------------------------------------------------------------
static void icm20602_self1_check(void)
{
	uint8 dat=0;
	while(0x12 != dat)																// 判断 ID 是否正确
	{
		dat = simiic_read_reg(ICM20602_DEV_ADDR,ICM20602_WHO_AM_I,SIMIIC);			// 读取ICM20602 ID
		systick_delay_ms(10);
		//卡在这里原因有以下几点
		//1 ICM20602坏了，如果是新的这样的概率极低
		//2 接线错误或者没有接好
		//3 可能你需要外接上拉电阻，上拉到3.3V
		//4 可能没有调用模拟IIC的初始化函数
	}
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		初始化ICM20602
// @param		NULL
// @return		void
// @since		v1.0
// Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void icm20602_init(void)
{
	simiic_init();
	systick_delay_ms(10);  //上电延时

	//检测
	icm20602_self1_check();

	//复位
	simiic_write_reg(ICM20602_DEV_ADDR,ICM20602_PWR_MGMT_1,0x80);					// 复位设备
	systick_delay_ms(2);															// 延时
	while(0x80 & simiic_read_reg(ICM20602_DEV_ADDR,ICM20602_PWR_MGMT_1,SIMIIC));	// 等待复位完成

	//配置参数
	simiic_write_reg(ICM20602_DEV_ADDR,ICM20602_PWR_MGMT_1,0x01);					// 时钟设置
	simiic_write_reg(ICM20602_DEV_ADDR,ICM20602_PWR_MGMT_2,0x00);					// 开启陀螺仪和加速度计
	simiic_write_reg(ICM20602_DEV_ADDR,ICM20602_CONFIG,0x01);						// 176HZ 1KHZ
	simiic_write_reg(ICM20602_DEV_ADDR,ICM20602_SMPLRT_DIV,0x07);					// 采样速率 SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)
	simiic_write_reg(ICM20602_DEV_ADDR,ICM20602_GYRO_CONFIG,0x18);					// ±2000 dps
	simiic_write_reg(ICM20602_DEV_ADDR,ICM20602_ACCEL_CONFIG,0x10);					// ±8g
	simiic_write_reg(ICM20602_DEV_ADDR,ICM20602_ACCEL_CONFIG_2,0x03);				// Average 4 samples   44.8HZ   //0x23 Average 16 samples
}


//-------------------------------------------------------------------------------------------------------------------
// @brief		获取ICM20602加速度计数据
// @param		NULL
// @return		void
// @since		v1.0
// Sample usage:				执行该函数后，直接查看对应的变量即可
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
// @brief		获取ICM20602陀螺仪数据
// @param		NULL
// @return		void
// @since		v1.0
// Sample usage:				执行该函数后，直接查看对应的变量即可
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
// 以上函数是使用软件IIC通信
//-------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
// 以下函数是使用硬件SPI通信 相比较IIC 速度比IIC快非常多
//-------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
// @brief		ICM20602 SPI写寄存器
// @param		cmd				寄存器地址
// @param		val				需要写入的数据
// @return		void
// @since		v1.0
// Sample usage:
// @note		内部调用 用户无需关心
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
// @brief		ICM20602 SPI读寄存器
// @param		cmd				寄存器地址
// @param		*val			接收数据的地址
// @return		void
// @since		v1.0
// Sample usage:
// @note		内部调用 用户无需关心
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
// @brief		ICM20602 SPI多字节读寄存器
// @param		cmd				寄存器地址
// @param		*val			接收数据的地址
// @param		num				读取数量
// @return		void
// @since		v1.0
// Sample usage:
// @note		内部调用 用户无需关心
//-------------------------------------------------------------------------------------------------------------------
static void icm_spi_r_reg_bytes(uint8 * val, uint8 num)
{
	ICM20602_CS(0);
	spi_mosi(ICM20602_SPI, val, val, num);
	ICM20602_CS(1);
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		ICM20602自检函数
// @param		NULL
// @return		void
// @since		v1.0
// Sample usage:
// @note		内部调用 用户无需关心
//-------------------------------------------------------------------------------------------------------------------
static void icm20602_self3_check(void)
{
	uint8 dat = 0;

	while(0x12 != dat)																// 判断 ID 是否正确
	{
		icm_spi_r_reg_byte(ICM20602_WHO_AM_I, &dat);								// 读取ICM20602 ID
		systick_delay_ms(10);
		//卡在这里原因有以下几点
		//1 ICM20602坏了，如果是新的这样的概率极低
		//2 接线错误或者没有接好
		//3 可能你需要外接上拉电阻，上拉到3.3V
	}
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		初始化ICM20602
// @param		NULL
// @return		void
// @since		v1.0
// Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void icm20602_init_spi(void)
{
	uint8 val = 0x0;

	systick_delay_ms(10);  //上电延时

	spi_init(ICM20602_SPI, ICM20602_SCK_PIN, ICM20602_MOSI_PIN, ICM20602_MISO_PIN, SPI_NSS_NULL, 0, SystemCoreClock/4);	// 硬件SPI初始化

	gpio_init(ICM20602_CS_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);

	icm20602_self3_check();//检测

	icm_spi_w_reg_byte(ICM20602_PWR_MGMT_1,0x80);//复位设备
	systick_delay_ms(2);
	do																				// 等待复位成功
	{
		icm_spi_r_reg_byte(ICM20602_PWR_MGMT_1,&val);
	}while(0x41 != val);

	icm_spi_w_reg_byte(ICM20602_PWR_MGMT_1,		0x01);								// 时钟设置
	icm_spi_w_reg_byte(ICM20602_PWR_MGMT_2,		0x00);								// 开启陀螺仪和加速度计
	icm_spi_w_reg_byte(ICM20602_CONFIG,			0x01);								// 176HZ 1KHZ
	icm_spi_w_reg_byte(ICM20602_SMPLRT_DIV,		0x07);								// 采样速率 SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)
	icm_spi_w_reg_byte(ICM20602_GYRO_CONFIG,	0x18);								// ±2000 dps
	icm_spi_w_reg_byte(ICM20602_ACCEL_CONFIG,	0x10);								// ±8g
	icm_spi_w_reg_byte(ICM20602_ACCEL_CONFIG_2,	0x03);								// Average 4 samples   44.8HZ   //0x23 Average 16 samples
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		获取ICM20602加速度计数据
// @param		NULL
// @return		void
// @since		v1.0
// Sample usage:				执行该函数后，直接查看对应的变量即可
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
// @brief		获取ICM20602陀螺仪数据
// @param		NULL
// @return		void
// @since		v1.0
// Sample usage:				执行该函数后，直接查看对应的变量即可
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
// 以上函数是使用硬件SPI通信 相比较IIC 速度比IIC快非常多
//-------------------------------------------------------------------------------------------------------------------

#define AcceRatio 	4096.0f
#define GyroRatio 	16.4f
#define Gyro_Gr		0.0010653	// 角速度变成弧度	此参数对应陀螺2000度每秒
#define ACC_FILTER_NUM 5		// 加速度计滤波深度
#define GYRO_FILTER_NUM 1		// 陀螺仪滤波深度
int32 ACC_X_BUF[ACC_FILTER_NUM], ACC_Y_BUF[ACC_FILTER_NUM], ACC_Z_BUF[ACC_FILTER_NUM];	// 滤波缓存数组
int32 GYRO_X_BUF[GYRO_FILTER_NUM], GYRO_Y_BUF[GYRO_FILTER_NUM], GYRO_Z_BUF[GYRO_FILTER_NUM];
/*
 * 函数名：Data_Filter
 * 描述  ：数据滑动滤波
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 */
void Data_Filter(void)	// 数据滤波
{
	uint8 i;
	float ACC_Angle;
	int64 temp1 = 0, temp2 = 0, temp3 = 0, temp4 = 0, temp5 = 0, temp6 = 0;
	ACC_X_BUF[0] = ACC.X;	// 更新滑动窗口数组
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


//非矩阵卡尔曼滤波，这些参数不用改
#define Peried 1/500.0f		//卡尔曼积分周期
#define Q 2000.0f				//过程噪声2.0		越小积分越慢，跟踪加速度计越慢越平滑
#define R 2000.0f			//测量噪声5000.0	越小跟踪加速度计越快
float KalmanGain = 1.0f;	//卡尔曼增益

void KalmanFilter(float ACC_Angle)
{
	//卡尔曼滤波局部参量
    static float Priori_Estimation = 0;//先验估计
    static float Posterior_Estimation = 0;//后验估计
    static float Priori_Convariance = 0;//先验方差
    static float Posterior_Convariance = 0;//后验方差
		
	//卡尔曼滤波
    //1.时间更新(预测) : X(k|k-1) = A(k,k-1)*X(k-1|k-1) + B(k)*u(k) 
    Priori_Estimation = Posterior_Estimation - GYRO_Real.X*Peried;		//先验估计，积分获得角度
	if (Priori_Estimation != Priori_Estimation)
	{
		Priori_Estimation = 0;
	}
	
    //2.更新先验协方差  : P(k|k-1) = A(k,k-1)*P(k-1|k-1)*A(k,k-1)'+Q(k) 
    Priori_Convariance = (float)sqrt( Posterior_Convariance * Posterior_Convariance + Q * Q );
	if (Priori_Convariance != Priori_Convariance)
	{
		Priori_Convariance = 0;
	}
	
    //  卡尔曼后验估计：测量更新  
    // 1.计算卡尔曼增益  : K(k) = P(k|k-1)*H(k)' / (H(k)*P(k|k-1)*H(k)' + R(k)) /
    KalmanGain = (float)sqrt( Priori_Convariance * Priori_Convariance / ( Priori_Convariance * Priori_Convariance + R * R ) );
	if (KalmanGain != KalmanGain)
	{
		KalmanGain = 1;
	}
	
    //2.测量更新(校正): X(k|k) = X(k|k-1)+K(k)*(Z(k)-H(k)*X(k|k-1)) 
    Posterior_Estimation  = Priori_Estimation + KalmanGain * (ACC_Angle - Priori_Estimation );
	if (Posterior_Estimation != Posterior_Estimation)
	{
		Posterior_Estimation = 0;
	}
	
    // 3.更新后验协方差  : P(k|k) =（I-K(k)*H(k)）*P(k|k-1) 
    Posterior_Convariance = (float)sqrt(( 1 - KalmanGain ) * Priori_Convariance * Priori_Convariance );
	if (Posterior_Convariance != Posterior_Convariance)
	{
		Posterior_Convariance = 0;
	}
	
    //得到最终角度 
    Attitude_Angle.Y = Posterior_Estimation;
	
	if (Attitude_Angle.Y != Attitude_Angle.Y)
	{
		Attitude_Angle.Y = 1;
	}
}

/*
 * 函数名：Get_Attitude
 * 描述  ：姿态解算
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 */
void Get_Attitude(void)	// 姿态解算
{
	IMUupdate( GYRO_Real.Y*Gyro_Gr*GyroRatio, 
                          GYRO_Real.X*Gyro_Gr*GyroRatio,
			  GYRO_Real.Z*Gyro_Gr*GyroRatio, 
			  ACC_Real.X * AcceRatio, 
			  ACC_Real.Y * AcceRatio, 
			  ACC_Real.Z * AcceRatio);	// 姿态解算出欧拉角
//一阶互补滤波

}


//===============================四元素============================================
#define Kp 1.6f //10.0f             	// 比例增益控制加速度计/磁力计的收敛速率
#define Ki 0.001f//1.2f // //0.008f  	// 整体增益控制陀螺仪偏差的收敛率
#define halfT 0.001f                   	// 采样周期的一半
float q0 = 1, q1 = 0, q2 = 0, q3 = 0; 	// 代表估计方向的四重奏元素
float exInt = 0, eyInt = 0, ezInt = 0; 	// 缩放积分错误
/*
 * 函数名：IMUupdate
 * 描述  ：四元素解算欧拉角
 * 输入  ：陀螺仪 加速度计
 * 输出  ：无
 * 调用  ：内部调用
 */
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
	float norm;
	float vx, vy, vz;
	float ex, ey, ez;

	// 先把这些用得到的值算好
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
		
	norm = sqrt(ax*ax + ay*ay + az*az);	// acc数据归一化
	ax = ax / norm;
	ay = ay / norm;
	az = az / norm;

	// estimated direction of gravity and flux (v and w)	估计重力方向和流量/变迁
	vx = 2*(q1q3 - q0q2);									// 四元素中xyz的表示
	vy = 2*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3 ;

	// error is sum of cross product between reference direction of fields and direction measured by sensors
	ex = (ay*vz - az*vy) ;		// 向量外积在相减得到差分就是误差
	ey = (az*vx - ax*vz) ;
	ez = (ax*vy - ay*vx) ;

	exInt = exInt + ex * Ki;	// 对误差进行积分
	eyInt = eyInt + ey * Ki;
	ezInt = ezInt + ez * Ki;

	// adjusted gyroscope measurements
	gx = gx + Kp*ex + exInt;	// 将误差PI后补偿到陀螺仪，即补偿零点漂移
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;	// 这里的gz由于没有观测者进行矫正会产生漂移，表现出来的就是积分自增或自减

	// integrate quaternion rate and normalise	// 四元素的微分方程
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
 * 函数名：ICM20602_Offset
 * 描述  ：传感器采集零偏
 * 输入  ：无
 * 输出  ：无
 * 调用  ：内部调用
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
		ICM20602_GetData(&GYRO, &ACC);	// 读取陀螺仪数据
		systick_delay_ms(2);
		
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
 * 函数名：ICM20602_GetData
 * 描述  ：获得传感器所有数据
 * 输入  ：*GYRO 陀螺仪		*ACC 加速度计
 * 输出  ：无
 * 调用  ：外部调用
 */
void ICM20602_GetData(S_INT16_XYZ *GYRO, S_INT16_XYZ *ACC)
{
  get_icm20602_gyro_spi();
  get_icm20602_accdata_spi();
	if (Offset_OK)
	{
          
		ACC->X = icm_acc_x;	// 获取加速度计原始数据
		ACC->Y = icm_acc_y;
		ACC->Z = icm_acc_z;
		
		GYRO->X = -(icm_gyro_x + GYRO_Offset.X);	// 获取陀螺仪原始数据
		GYRO->Y = -(icm_gyro_y + GYRO_Offset.Y);
		GYRO->Z = -(icm_gyro_z + GYRO_Offset.Z);
	}
	else
	{
		ACC->X = icm_acc_x;	// 获取加速度计原始数据
		ACC->Y = icm_acc_y;
		ACC->Z = icm_acc_z;
		
		GYRO->X = -icm_gyro_x;	// 获取陀螺仪原始数据并归一化
		GYRO->Y = -icm_gyro_y;
		GYRO->Z = -icm_gyro_z;
	}
}


