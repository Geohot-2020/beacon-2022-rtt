/*
 * @Description: 图片处理函数
 * @Version: v1.0
 * @Autor: 郑有才
 * @Date: 2021-12-05 11:38:26
 * @LastEditors: 郑有才
 * @LastEditTime: 2021-12-07 19:56:39
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

/** 压缩后之后用于存放屏幕显示数据  */
uint8 Image_Use[MT9V03X_H][MT9V03X_W];
/** 二值化后图像 **/
uint8    Bin_Image[MT9V03X_H][MT9V03X_W];
/** 阈值 **/
unsigned short Threshold = 0;
uint8 keyv = 35;
uint8 tm=0;

volatile uint8 dotcnt = 0;
volatile uint8 dotlie[100];
volatile uint8 dothang[100];
volatile short sumlie=0,sumhang=0;
float camera_dif = 0;

rt_sem_t camera_sem;
/**
 * @description: 图像缩放到使用的大小
 * @param {*}
 * @return {*}
 * @author: 郑有才
 */
void Get_Use_Image(void)
{
	//rt_enter_critical();
    short i = 0, j = 0, row = 0, line = 0;

    for (i = 0; i < MT9V03X_H; i++)
    {
        for (j = 0; j < MT9V03X_W; j++)
        {
            Image_Use[row][line] = mt9v03x_image[i][j];
            line++;
        }
        line = 0;
        row++;
    }
	//rt_exit_critical();
}

/**
 * @description: 图像二值化到Bin_Image[][]
 * @param {unsigned char} mode
 * @return {*}
 * @author: 郑有才
 */
void Get_Bin_Image(unsigned char mode)
{
	//rt_enter_critical();
    unsigned short i = 0, j = 0;
    unsigned long tv = 0;

    if (mode == 0)
    {
        Threshold = GetOSTU(Image_Use);     //大津法阈值
    }
    else if (mode == 1)
    {
        //累加
        for (i = 0; i < MT9V03X_H; i++)
        {
            for (j = 0; j < MT9V03X_W; j++)
            {
                tv += Image_Use[i][j];  //累加
            }
        }
        Threshold = (unsigned short)(tv / MT9V03X_H / MT9V03X_W);   //求平均值，光线越暗越小，全黑35，对屏幕160，一般100
        Threshold = Threshold + keyv;
    }
    else if (mode == 2)
    {
        Threshold = 80;     //手动调节阈值
        lq_sobel(Image_Use, Bin_Image, (unsigned char) Threshold);

        return;
    }
    else if (mode == 3)
    {
        lq_sobelAutoThreshold(Image_Use, Bin_Image);  //动态调节阈值
        return;
    }

    /* 二值化 */
    for (i = 0; i < MT9V03X_H; i++)
    {
        for (j = 0; j < MT9V03X_W; j++)
        {
            if (Image_Use[i][j] > Threshold)    //数值越大，显示的内容越多，较浅的图像也能显示出来
                Bin_Image[i][j] = 1;
            else
                Bin_Image[i][j] = 0;
        }
    }
		
		//rt_exit_critical();
		
}


/*************************************************************************
 *  函数名称：short GetOSTU (unsigned char tmImage[LCDH][LCDW])
 *  功能说明：大津法求阈值大小
 *  参数说明：tmImage ： 图像数据
 *  函数返回：无
 *  修改时间：2011年10月28日
 *  备    注：  GetOSTU(Image_Use);//大津法阈值
Ostu方法又名最大类间差方法，通过统计整个图像的直方图特性来实现全局阈值T的自动选取，其算法步骤为：
1) 先计算图像的直方图，即将图像所有的像素点按照0~255共256个bin，统计落在每个bin的像素点数量
2) 归一化直方图，也即将每个bin中像素点数量除以总的像素点
3) i表示分类的阈值，也即一个灰度级，从0开始迭代 1
4) 通过归一化的直方图，统计0~i 灰度级的像素(假设像素值在此范围的像素叫做前景像素) 所占整幅图像
        的比例w0，        并统计前景像素的平均灰度u0；统计i~255灰度级的像素(假设像素值在此范围的像素叫做背
        景像素)  * 所占整幅图像的比例w1，并统计背景像素的平均灰度u1；
5) 计算前景像素和背景像素的方差 g = w0*w1*(u0-u1) (u0-u1)
6) i++；转到4)，直到i为256时结束迭代
7) 将最大g相应的i值作为图像的全局阈值
缺陷:OSTU算法在处理光照不均匀的图像的时候，效果会明显不好，因为利用的是全局像素信息。
*************************************************************************/
short GetOSTU (unsigned char tmImage[MT9V03X_H][MT9V03X_W])
{
    signed short i, j;
    unsigned long Amount = 0;
    unsigned long PixelBack = 0;
    unsigned long PixelshortegralBack = 0;
    unsigned long Pixelshortegral = 0;
    signed long PixelshortegralFore = 0;
    signed long PixelFore = 0;
    float OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // 类间方差;
    signed short MinValue, MaxValue;
    signed short Threshold = 0;
    unsigned char HistoGram[256];              //

    for (j = 0; j < 256; j++)
        HistoGram[j] = 0; //初始化灰度直方图

    for (j = 0; j < MT9V03X_H; j++)
    {
        for (i = 0; i < MT9V03X_W; i++)
        {
            HistoGram[tmImage[j][i]]++; //统计灰度级中每个像素在整幅图像中的个数
        }
    }

    for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++);        //获取最小灰度的值
    for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--); //获取最大灰度的值

    if (MaxValue == MinValue)
        return MaxValue;         // 图像中只有一个颜色
    if (MinValue + 1 == MaxValue)
        return MinValue;        // 图像中只有二个颜色

    for (j = MinValue; j <= MaxValue; j++)
        Amount += HistoGram[j];        //  像素总数

    Pixelshortegral = 0;
    for (j = MinValue; j <= MaxValue; j++)
    {
        Pixelshortegral += HistoGram[j] * j;        //灰度值总数
    }
    SigmaB = -1;
    for (j = MinValue; j < MaxValue; j++)
    {
        PixelBack = PixelBack + HistoGram[j];     //前景像素点数
        PixelFore = Amount - PixelBack;           //背景像素点数
        OmegaBack = (float) PixelBack / Amount;   //前景像素百分比
        OmegaFore = (float) PixelFore / Amount;   //背景像素百分比
        PixelshortegralBack += HistoGram[j] * j;  //前景灰度值
        PixelshortegralFore = Pixelshortegral - PixelshortegralBack;  //背景灰度值
        MicroBack = (float) PixelshortegralBack / PixelBack;   //前景灰度百分比
        MicroFore = (float) PixelshortegralFore / PixelFore;   //背景灰度百分比
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);   //计算类间方差
        if (Sigma > SigmaB)                    //遍历最大的类间方差g //找出最大类间方差以及对应的阈值
        {
            SigmaB = Sigma;
            Threshold = j;
        }
    }
    return Threshold;                        //返回最佳阈值;
}

/*!
 * @brief    基于soble边沿检测算子的一种边沿检测
 *
 * @param    imageIn    输入数组
 *           imageOut   输出数组      保存的二值化后的边沿信息
 *           Threshold  阈值
 *
 * @return
 *
 * @note
 *
 * @example
 *
 * @date     2020/5/15
 */
void lq_sobel (unsigned char imageIn[MT9V03X_H][MT9V03X_W], unsigned char imageOut[MT9V03X_H][MT9V03X_W], unsigned char Threshold)
{
    /** 卷积核大小 */
    short KERNEL_SIZE = 3;
    short xStart = KERNEL_SIZE / 2;
    short xEnd = MT9V03X_H - KERNEL_SIZE / 2;
    short yStart = KERNEL_SIZE / 2;
    short yEnd = MT9V03X_W - KERNEL_SIZE / 2;
    short i, j, k;
    short temp[4];
    for (i = yStart; i < yEnd; i++)
    {
        for (j = xStart; j < xEnd; j++)
        {
            /* 计算不同方向梯度幅值  */
            temp[0] = -(short) imageIn[i - 1][j - 1] + (short) imageIn[i - 1][j + 1]     //{{-1, 0, 1},
            - (short) imageIn[i][j - 1] + (short) imageIn[i][j + 1]        // {-1, 0, 1},
            - (short) imageIn[i + 1][j - 1] + (short) imageIn[i + 1][j + 1];    // {-1, 0, 1}};

            temp[1] = -(short) imageIn[i - 1][j - 1] + (short) imageIn[i + 1][j - 1]     //{{-1, -1, -1},
            - (short) imageIn[i - 1][j] + (short) imageIn[i + 1][j]       // { 0,  0,  0},
            - (short) imageIn[i - 1][j + 1] + (short) imageIn[i + 1][j + 1];    // { 1,  1,  1}};

            temp[2] = -(short) imageIn[i - 1][j] + (short) imageIn[i][j - 1]       //  0, -1, -1
            - (short) imageIn[i][j + 1] + (short) imageIn[i + 1][j]       //  1,  0, -1
            - (short) imageIn[i - 1][j + 1] + (short) imageIn[i + 1][j - 1];    //  1,  1,  0

            temp[3] = -(short) imageIn[i - 1][j] + (short) imageIn[i][j + 1]       // -1, -1,  0
            - (short) imageIn[i][j - 1] + (short) imageIn[i + 1][j]       // -1,  0,  1
            - (short) imageIn[i - 1][j - 1] + (short) imageIn[i + 1][j + 1];    //  0,  1,  1

            temp[0] = abs(temp[0]);
            temp[1] = abs(temp[1]);
            temp[2] = abs(temp[2]);
            temp[3] = abs(temp[3]);

            /* 找出梯度幅值最大值  */
            for (k = 1; k < 4; k++)
            {
                if (temp[0] < temp[k])
                {
                    temp[0] = temp[k];
                }
            }

            if (temp[0] > Threshold)
            {
                imageOut[i][j] = 1;
            }
            else
            {
                imageOut[i][j] = 0;
            }
        }
    }
}

/*!
 * @brief    基于soble边沿检测算子的一种自动阈值边沿检测
 *
 * @param    imageIn    输入数组
 *           imageOut   输出数组      保存的二值化后的边沿信息
 *
 * @return
 *
 * @note
 *
 * @example
 *
 * @date     2020/5/15
 */
void lq_sobelAutoThreshold (unsigned char imageIn[MT9V03X_H][MT9V03X_W], unsigned char imageOut[MT9V03X_H][MT9V03X_W])
{
    /** 卷积核大小 */
    short KERNEL_SIZE = 3;
    short xStart = KERNEL_SIZE / 2;
    short xEnd = MT9V03X_W - KERNEL_SIZE / 2;
    short yStart = KERNEL_SIZE / 2;
    short yEnd = MT9V03X_H - KERNEL_SIZE / 2;
    short i, j, k;
    short temp[4];
    for (i = yStart; i < yEnd; i++)
    {
        for (j = xStart; j < xEnd; j++)
        {
            /* 计算不同方向梯度幅值  */
            temp[0] = -(short) imageIn[i - 1][j - 1] + (short) imageIn[i - 1][j + 1]     //{{-1, 0, 1},
            - (short) imageIn[i][j - 1] + (short) imageIn[i][j + 1]       // {-1, 0, 1},
            - (short) imageIn[i + 1][j - 1] + (short) imageIn[i + 1][j + 1];    // {-1, 0, 1}};

            temp[1] = -(short) imageIn[i - 1][j - 1] + (short) imageIn[i + 1][j - 1]     //{{-1, -1, -1},
            - (short) imageIn[i - 1][j] + (short) imageIn[i + 1][j]       // { 0,  0,  0},
            - (short) imageIn[i - 1][j + 1] + (short) imageIn[i + 1][j + 1];    // { 1,  1,  1}};

            temp[2] = -(short) imageIn[i - 1][j] + (short) imageIn[i][j - 1]       //  0, -1, -1
            - (short) imageIn[i][j + 1] + (short) imageIn[i + 1][j]       //  1,  0, -1
            - (short) imageIn[i - 1][j + 1] + (short) imageIn[i + 1][j - 1];    //  1,  1,  0

            temp[3] = -(short) imageIn[i - 1][j] + (short) imageIn[i][j + 1]       // -1, -1,  0
            - (short) imageIn[i][j - 1] + (short) imageIn[i + 1][j]       // -1,  0,  1
            - (short) imageIn[i - 1][j - 1] + (short) imageIn[i + 1][j + 1];    //  0,  1,  1

            temp[0] = abs(temp[0]);
            temp[1] = abs(temp[1]);
            temp[2] = abs(temp[2]);
            temp[3] = abs(temp[3]);

            /* 找出梯度幅值最大值  */
            for (k = 1; k < 4; k++)
            {
                if (temp[0] < temp[k])
                {
                    temp[0] = temp[k];
                }
            }

            /* 使用像素点邻域内像素点之和的一定比例    作为阈值  */
            temp[3] = (short) imageIn[i - 1][j - 1] + (short) imageIn[i - 1][j] + (short) imageIn[i - 1][j + 1]
                    + (short) imageIn[i][j - 1] + (short) imageIn[i][j] + (short) imageIn[i][j + 1]
                    + (short) imageIn[i + 1][j - 1] + (short) imageIn[i + 1][j] + (short) imageIn[i + 1][j + 1];

            if (temp[0] > temp[3] / 12.0f)
            {
                imageOut[i][j] = 1;
            }
            else
            {
                imageOut[i][j] = 0;
            }

        }
    }
}

/**
 * @description: 过滤噪点
 * @param {*}
 * @return {*}
 * @author: 郑有才
 */
void Bin_Image_Filter (void)
{
    uint16 nr; //行
    uint16 nc; //列

    for (nr = 1; nr < MT9V03X_H - 1; nr++)
    {
        for (nc = 1; nc < MT9V03X_W - 1; nc = nc + 1)
        {
            if ((Bin_Image[nr][nc] == 0)
                    && (Bin_Image[nr - 1][nc] + Bin_Image[nr + 1][nc] + Bin_Image[nr][nc + 1] + Bin_Image[nr][nc - 1] > 2))
            {
                Bin_Image[nr][nc] = 1;
            }
            else if ((Bin_Image[nr][nc] == 1)
                    && (Bin_Image[nr - 1][nc] + Bin_Image[nr + 1][nc] + Bin_Image[nr][nc + 1] + Bin_Image[nr][nc - 1] < 2))
            {
                Bin_Image[nr][nc] = 0;
            }
        }
    }
}

/**
 * @description: 找灯
 * @param {*}
 * @return {*}
 * @author: 郑有才
 */
void Seek_Beacon (void)
{
	//rt_enter_critical();
  uint8 nr=0; //行
  uint8 nc=0; //列
  
  dotcnt=0;
  for (nr = 1; nr < MT9V03X_H - 1; nr++)
  {
    for (nc = 1; nc < MT9V03X_H - 1; nc++)
    {
      if ((Bin_Image[nr - 1][nc] + Bin_Image[nr + 1][nc] + Bin_Image[nr][nc + 1] + Bin_Image[nr][nc - 1] > 1))
      {
          dothang[dotcnt]=nr;
          dotlie[dotcnt++]=nc;
      }
    }
  }
  //rt_exit_critical();
}

float Camera_Control(void)
{
	//rt_enter_critical();
    float dif;
    Get_Use_Image();
    Get_Bin_Image(1);
    Seek_Beacon();
    if(dotcnt)
    {
        sumlie=0;sumhang=0;
        for (tm=0; tm<dotcnt; tm++)
        {
            sumlie+=dotlie[tm];
            sumhang+=dothang[tm];
        }
        sumlie = sumlie/dotcnt;
        sumhang  = sumhang/dotcnt;
        dif = sumlie-93;
        return dif;
    }
	//rt_exit_critical();
}

/**
 * @description: 展示二值化后的赛道图，显示可能有误
 * @param {*}
 * @return {*}
 * @author: 郑有才
 */
void showBeacon()
{
    uint32 i,j;
                
    uint16 color = 0;
	uint16 temp = 0;
	
    uint16 coord_x = 0;
    uint16 coord_y = 0;
    
    coord_x = MT9V03X_W;
    coord_y = MT9V03X_H;
    ips200_address_set(0,0,coord_x-1,coord_y-1);
    
    for(j=0;j<coord_y;j++)
    {
        for(i=0;i<coord_x;i++)
        {
            if(Bin_Image[i][j])
            {
                ips200_drawpoint(i,j,WHITE);
            }
            else 
            {
                ips200_drawpoint(i,j,BLACK);
            }
        }
    }
}

/**
 * @description: camera线程入口
 * @param {void} *parameter
 * @return {*}
 * @author: 郑有才
 */
void camera_entry(void *parameter)
{
    while(1)
    {

            rt_sem_take(camera_sem, RT_WAITING_FOREVER);
            rt_sem_control(camera_sem, RT_IPC_CMD_RESET, RT_NULL); 
            
            camera_dif = Camera_Control();
            //printf("%f\r\n",camera_dif);
            mt9v03x_finish_flag=0;

        
        
        rt_thread_mdelay(0);
         //showBeacon();
    }
}

/**
 * @description: 
 * @param {*}
 * @return {*}
 * @author: 郑有才
 */
void camera_init(void)
{
    rt_thread_t tid;

    //初始化摄像头
    mt9v03x_init();

    //创建图片处理线程， 优先级设置为10
    tid = rt_thread_create("camera", camera_entry, RT_NULL, 1024, 6, 30);

    //启动图片线程
    if(RT_NULL != tid) 
    {
        rt_thread_startup(tid);
    }
}



