/*
 * @Description: 匿名上位机,整定陀螺仪波形
 * @Version: v1.0
 * @Autor: 郑有才
 * @Date: 2021-12-02 14:31:16
 * @LastEditors: 郑有才
 * @LastEditTime: 2021-12-02 14:39:52
 */
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

#include "headfile.h"
unsigned char data_to_send[50];

/**
 * @description: 匿名上位机用户协议，使用时占用MCU资源较大，跑车时不使用
 * @param {short} data1
 * @param {short} data2
 * @param {short} data3
 * @param {short} data4
 * @param {short} data5
 * @param {short} data6
 * @param {short} data7
 * @param {short} data8
 * @return {*}
 * @author: 郑有才
 */
void ANO_DT_send_int16byte16(short data1, short data2, short data3, short data4, short data5, short data6, short data7, short data8 )
{
  unsigned char  _cnt=0;
  unsigned char  sum = 0, i = 0;
  
  data_to_send[_cnt++] = 0xAA;      //匿名协议帧头  0xAAAA
  data_to_send[_cnt++] = 0xAA;
  data_to_send[_cnt++] = 0xF1;      //使用用户协议帧0xF1
  data_to_send[_cnt++] = 16;        //8个int16_t 长度 16个字节
  
  data_to_send[_cnt++]=(unsigned short)(data1>>8);
  data_to_send[_cnt++]=(unsigned char)(data1);
  
  data_to_send[_cnt++]=(unsigned short)(data2>>8);
  data_to_send[_cnt++]=(unsigned char)(data2);
  
  data_to_send[_cnt++]=(unsigned short)(data3>>8);
  data_to_send[_cnt++]=(unsigned char)(data3);
  
  data_to_send[_cnt++]=(unsigned short)(data4>>8);
  data_to_send[_cnt++]=(unsigned char)(data4);
  
  data_to_send[_cnt++]=(unsigned short)(data5>>8);
  data_to_send[_cnt++]=(unsigned char)(data5);
  
  data_to_send[_cnt++]=(unsigned short)(data6>>8);
  data_to_send[_cnt++]=(unsigned char)(data6);
  
  data_to_send[_cnt++]=(unsigned short)(data7>>8);
  data_to_send[_cnt++]=(unsigned char)(data7);
  
  data_to_send[_cnt++]=(unsigned short)(data8>>8);
  data_to_send[_cnt++]=(unsigned char)(data8);
  
  sum = 0;
  for(i=0;i<_cnt;i++)
    sum += data_to_send[i];
  data_to_send[_cnt++]=sum;
  
  //UART_PutBuff(UART1, data_to_send, _cnt);     //可以修改不同的串口发送数据;
}

/**
 * @description: 
 * @param {short} data1
 * @param {short} data2
 * @param {short} data3
 * @param {short} data4
 * @param {short} data5
 * @param {short} data6
 * @param {short} data7
 * @param {short} data8
 * @return {*}
 * @author: 郑有才
 */
void ANO_DT_send_int16(short data1, short data2, short data3, short data4, short data5, short data6, short data7, short data8 )
{
  unsigned char  _cnt=0;
  unsigned char  sum = 0, i = 0;
  
  data_to_send[_cnt++] = 0xAA;      //匿名协议帧头  0xAAAA
  data_to_send[_cnt++] = 0xAA;
  data_to_send[_cnt++] = 0xF1;      //使用用户协议帧0xF1
  data_to_send[_cnt++] = 16;        //8个int16_t 长度 16个字节
  
  data_to_send[_cnt++]=(unsigned short)(data1>>8);
  data_to_send[_cnt++]=(unsigned char)(data1);
  
  data_to_send[_cnt++]=(unsigned short)(data2>>8);
  data_to_send[_cnt++]=(unsigned char)(data2);
  
  data_to_send[_cnt++]=(unsigned short)(data3>>8);
  data_to_send[_cnt++]=(unsigned char)(data3);
  
  data_to_send[_cnt++]=(unsigned short)(data4>>8);
  data_to_send[_cnt++]=(unsigned char)(data4);
  
  data_to_send[_cnt++]=(unsigned short)(data5>>8);
  data_to_send[_cnt++]=(unsigned char)(data5);
  
  data_to_send[_cnt++]=(unsigned short)(data6>>8);
  data_to_send[_cnt++]=(unsigned char)(data6);
  
  data_to_send[_cnt++]=(unsigned short)(data7>>8);
  data_to_send[_cnt++]=(unsigned char)(data7);
  
  data_to_send[_cnt++]=(unsigned short)(data8>>8);
  data_to_send[_cnt++]=(unsigned char)(data8);
  
  sum = 0;
  for(i=0;i<_cnt;i++)
    sum += data_to_send[i];
  data_to_send[_cnt++]=sum;
  
  //UART_PutBuff(UART1, data_to_send, _cnt);     //可以修改不同的串口发送数据;
}

/**
 * @description: 
 * @param {short} data1
 * @param {short} data2
 * @param {short} data3
 * @param {short} data4
 * @param {short} data5
 * @param {short} data6
 * @param {short} data7
 * @param {short} data8
 * @param {short} data9
 * @return {*}
 * @author: 郑有才
 */
void ANO_DT_send_int16byte18(short data1, short data2, short data3, short data4, short data5, short data6, short data7, short data8, short data9 )
{
  unsigned char  _cnt=0;
  unsigned char  sum = 0, i = 0;
  
  data_to_send[_cnt++] = 0xAA;      //匿名协议帧头  0xAAAA
  data_to_send[_cnt++] = 0xAA;
  data_to_send[_cnt++] = 0xF1;      //使用用户协议帧0xF1
  data_to_send[_cnt++] = 18;        //9个int16_t 长度 18个字节
  
  data_to_send[_cnt++]=(unsigned short)(data1>>8);
  data_to_send[_cnt++]=(unsigned char)(data1);
  
  data_to_send[_cnt++]=(unsigned short)(data2>>8);
  data_to_send[_cnt++]=(unsigned char)(data2);
  
  data_to_send[_cnt++]=(unsigned short)(data3>>8);
  data_to_send[_cnt++]=(unsigned char)(data3);
  
  data_to_send[_cnt++]=(unsigned short)(data4>>8);
  data_to_send[_cnt++]=(unsigned char)(data4);
  
  data_to_send[_cnt++]=(unsigned short)(data5>>8);
  data_to_send[_cnt++]=(unsigned char)(data5);
  
  data_to_send[_cnt++]=(unsigned short)(data6>>8);
  data_to_send[_cnt++]=(unsigned char)(data6);
  
  data_to_send[_cnt++]=(unsigned short)(data7>>8);
  data_to_send[_cnt++]=(unsigned char)(data7);
  
  data_to_send[_cnt++]=(unsigned short)(data8>>8);
  data_to_send[_cnt++]=(unsigned char)(data8);
  
  data_to_send[_cnt++]=(unsigned short)(data9>>8);
  data_to_send[_cnt++]=(unsigned char)(data9);
  
  sum = 0;
  for(i=0;i<_cnt;i++)
    sum += data_to_send[i];
  data_to_send[_cnt++]=sum;
  
  //UART_PutBuff(UART1, data_to_send, _cnt);     //可以修改不同的串口发送数据;
}
