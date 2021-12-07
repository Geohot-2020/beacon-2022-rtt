/*
 * @Description: 图片处理头文件
 * @Version: v1.0
 * @Autor: 郑有才
 * @Date: 2021-12-05 11:39:48
 * @LastEditors: 郑有才
 * @LastEditTime: 2021-12-07 15:54:04
 */

#ifndef _CAMERA_h
#define _CAMERA_h

extern uint8 Image_Use[MT9V03X_H][MT9V03X_W];
extern uint8 Bin_Image[MT9V03X_H][MT9V03X_W];
extern unsigned short Threshold;
extern uint8 keyv;
extern volatile uint8 dotcnt;
extern volatile uint8 dotlie[100];
extern volatile uint8 dothang[100];
extern volatile short sumlie,sumhang;
extern float camera_dif;
extern uint8 tm;
extern rt_sem_t camera_sem;


void Get_Use_Image(void);
void Get_Bin_Image(unsigned char mode);
short GetOSTU (unsigned char tmImage[MT9V03X_H][MT9V03X_W]);
void lq_sobel (unsigned char imageIn[MT9V03X_H][MT9V03X_W], unsigned char imageOut[MT9V03X_H][MT9V03X_W], unsigned char Threshold);
void lq_sobelAutoThreshold (unsigned char imageIn[MT9V03X_H][MT9V03X_W], unsigned char imageOut[MT9V03X_H][MT9V03X_W]);
void Bin_Image_Filter (void);
void Seek_Beacon (void);
float Camera_Control(void);
void showBeacon();
void camera_init(void);
#endif
