#include "zf_common_headfile.h"
#include "car.h"
#include "arm.h"
#include "display.h"
#include "images.h"
#include "all_init.h"
#include "imu660ra.h"
#include "wireless_uart.h"
#include "uart_for_art.h"
int mypow(int x, int y)  
{  
    if (y == 1) return x;  
    int result = 0;  
    int tmp = mypow(x, y/2);  
    if((y&1) != 0) //奇数  
    {  
        result = x * tmp * tmp;  
    }  
    else  
    {  
        result = tmp * tmp;  
    }  
  
    return result;  
}  
uint8 Threshold;
uint8 image_two_value[MT9V03X_H][MT9V03X_W];
const uint8  Standard_Road_Wide[MT9V03X_H];//标准赛宽数组
volatile int Left_Line[MT9V03X_H]; //左边线数组
volatile int Right_Line[MT9V03X_H];//右边线数组
volatile int Mid_Line[MT9V03X_H];  //中线数组
volatile int Up_Line[MT9V03X_W]; //上线数组
volatile int Road_Wide[MT9V03X_H]; //实际赛宽数组
volatile int White_Column[MT9V03X_W];//每列白列长度
volatile int Search_Stop_Line;     //搜索截止行,只记录长度，想要坐标需要用视野高度减去该值
volatile int Boundry_Start_Left;   //左右边界起始点
volatile int Boundry_Start_Right;  //第一个非丢线点,常规边界起始点
volatile int Left_Lost_Time;       //边界丢线数
volatile int Right_Lost_Time;
volatile int Both_Lost_Time;//两边同时丢线数
int Longest_White_Column_Left[2]; //最长白列,[0]是最长白列的长度，也就是Search_Stop_Line搜索截止行，[1】是第某列
int Longest_White_Column_Right[2];//最长白列,[0]是最长白列的长度，也就是Search_Stop_Line搜索截止行，[1】是第某列
int Left_Lost_Flag[MT9V03X_H] ; //左丢线数组，丢线置1，没丢线置0
int Right_Lost_Flag[MT9V03X_H]; //右丢线数组，丢线置1，没丢线置0
volatile int Left_Up_Guai[2];    //四个拐点的坐标存储，[0]存y，第某行，{1}存x，第某列
volatile int Right_Up_Guai[2];   //四个拐点的坐标存储，[0]存y，第某行，{1}存x，第某列
//十字
volatile int Cross_Flag;
volatile int  Left_Cross_Flag;
volatile int  Right_Cross_Flag;
volatile int Cross_State;
volatile int Left_Down_Find; //十字使用，找到被置行数，没找到就是0
volatile int Left_Up_Find;   //四个拐点标志
volatile int Right_Down_Find;
volatile int Right_Up_Find;
//环岛
volatile int Island_State;     //环岛状态标志
volatile int Island_Flag; //环岛标志
volatile int Left_Island_Flag; //左右环岛标志
volatile int Right_Island_Flag;

volatile int Right_point=0;
volatile int Left_point=0;
volatile int L_point_flag=0;
volatile int R_point_flag=0;

//斑马线
volatile int Zebra_Flag=0;
volatile int change_count;

const uint8 Weight[MT9V03X_H]=
{
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,              //图像最远端00 ——09 行权重
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,             //图像最远端10 ——19 行权重
        19,17,15,13,11,9, 7, 5, 3, 1,              //图像最远端20 ——29 行权重
        6, 7, 9,11,13,15,17,19,20,20,              //图像最远端30 ——39 行权重
		19,17,15,13,11, 9, 7, 5, 3, 1,              //图像最远端40 ——49 行权重
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,              //图像最远端50 ——59 行权重
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,              //图像最远端60 ——69 行权重

};


/*-------------------------------------------------------------------------------------------------------------------
  @brief     快速大津求阈值，来自山威
  @param     image       图像数组
             col         列 ，宽度
             row         行，长度
  @return    null
  Sample     threshold=my_adapt_threshold(mt9v03x_image[0],MT9V03X_W, MT9V03X_H);//山威快速大津
  @note      据说比传统大津法快一点，使用效果差不多
-------------------------------------------------------------------------------------------------------------------*/
uint8 otsuThreshold(uint8 *image, uint16 col, uint16 row)
{
    #define GrayScale 256
    uint16 Image_Width  = col;
    uint16 Image_Height = row;
    int X; uint16 Y;
    uint8* data = image;
    int HistGram[GrayScale] = {0};
	
	uint32 Amount = 0;
    uint32 PixelBack = 0;
    uint32 PixelIntegralBack = 0;
    uint32 PixelIntegral = 0;
    int32 PixelIntegralFore = 0;
    int32 PixelFore = 0;
    double OmegaBack=0, OmegaFore=0, MicroBack=0, MicroFore=0, SigmaB=0, Sigma=0; 
    int16 MinValue=0, MaxValue=0;
    uint8 Threshold = 0;
	
	
    for (Y = 0; Y <Image_Height; Y++) //Y<Image_Height??Y =Image_Height;
    {
        //Y=Image_Height;
        for (X = 0; X < Image_Width; X++)
        {
        HistGram[(int)data[Y*Image_Width + X]]++; 
        }
    }
    for (MinValue = 0; MinValue < 256 && HistGram[MinValue] == 0; MinValue++) ;        
    for (MaxValue = 255; MaxValue > MinValue && HistGram[MinValue] == 0; MaxValue--) ; 

    if (MaxValue == MinValue)
    {
        return MaxValue;          
    }
    if (MinValue + 1 == MaxValue)
    {
        return MinValue;      
    }

    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        Amount += HistGram[Y];        
    }

    PixelIntegral = 0;
    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        PixelIntegral += HistGram[Y] * Y;
    }
    SigmaB = -1;
    for (Y = MinValue; Y < MaxValue; Y++)
    {
          PixelBack = PixelBack + HistGram[Y];    
          PixelFore = Amount - PixelBack;        
          OmegaBack = (double)PixelBack / Amount;
          OmegaFore = (double)PixelFore / Amount;
          PixelIntegralBack += HistGram[Y] * Y;  
          PixelIntegralFore = PixelIntegral - PixelIntegralBack;
          MicroBack = (double)PixelIntegralBack / PixelBack;
          MicroFore = (double)PixelIntegralFore / PixelFore;
          Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);
          if (Sigma > SigmaB)
          {
              SigmaB = Sigma;
              Threshold = (uint8)Y;
          }
    }
   return Threshold;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     图像二值化处理函数
  @param     原始图像数组，高度，宽度，二值化阈值
  @return    二值化之后的图像数组
  Sample     Image_Binarization(Threshold);//图像二值化
  @note      二值化处理，0x00黑，0xff是白，已经用宏定义替换过了
-------------------------------------------------------------------------------------------------------------------*/
void Image_Binarization(int threshold)//图像二值化
{
    uint16 i,j;
    for(i=0;i<MT9V03X_H;i++)
    {
        for(j=0;j<MT9V03X_W;j++)
        {
            if(mt9v03x_image[i][j]>=threshold)
                image_two_value[i][j]=IMG_WHITE;//白
            else
                image_two_value[i][j]=IMG_BLACK;//黑
        }
    }
}

//像素滤波
void Pixle_Filter()
{
  int nr;  //行
  int nc;  //列

  for (nr = 10; nr < 50; nr++) {
    for (nc = 10; nc < 100; nc = nc + 1) {
      if ((image_two_value[nr][nc] == 0) && (image_two_value[nr - 1][nc] + image_two_value[nr + 1][nc] +
              image_two_value[nr][nc + 1] + image_two_value[nr][nc - 1] >=
                                   3)) {
          image_two_value[nr][nc] = 1;
      }
    }
  }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     双最长白列巡线
  @param     null
  @return    null
  Sample     Longest_White_Column_Left();
  @note      最长白列巡线，寻找初始边界，丢线，最长白列等基础元素，后续读取这些变量来进行赛道识别
-------------------------------------------------------------------------------------------------------------------*/
void Longest_White_Column()//最长白列巡线
{
    int i, j;
    int start_column=20;//最长白列的搜索区间
    int end_column=MT9V03X_W-20;
    int left_border = 0, right_border = 0;//临时存储赛道位置
    Longest_White_Column_Left[0] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
    Longest_White_Column_Left[1] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
    Longest_White_Column_Right[0] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
    Longest_White_Column_Right[1] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
    Right_Lost_Time = 0;    //边界丢线数
    Left_Lost_Time  = 0;
    Boundry_Start_Left  = 0;//第一个非丢线点,常规边界起始点
    Boundry_Start_Right = 0;
    Both_Lost_Time = 0;//两边同时丢线数
 
    for (i = 0; i <=MT9V03X_H-1; i++)//数据清零
    {
        Right_Lost_Flag[i] = 0;
        Left_Lost_Flag[i] = 0;
        Left_Line[i] = 0;
        Right_Line[i] = MT9V03X_W-1;
    }
    for(i=0;i<=MT9V03X_W-1;i++)
    {
        White_Column[i] = 0;
    }
 
    //从左到右，从下往上，遍历全图记录范围内的每一列白点数量
    for (j =start_column; j<=end_column; j++)
    {
        for (i = MT9V03X_H - 1; i >= 0; i--)
        {
            if(image_two_value[i][j] == IMG_BLACK)
                break;
            else
                White_Column[j]++;
        }
    }
 
    //从左到右找左边最长白列
    Longest_White_Column_Left[0] =0;
    for(i=start_column;i<=end_column;i++)
    {
        if (Longest_White_Column_Left[0] < White_Column[i])//找最长的那一列
        {
            Longest_White_Column_Left[0] = White_Column[i];//【0】是白列长度
            Longest_White_Column_Left[1] = i;              //【1】是下标，第j列
        }
    }
    //从右到左找右左边最长白列
    Longest_White_Column_Right[0] = 0;//【0】是白列长度
    for(i=end_column;i>=start_column;i--)//从右往左，注意条件，找到左边最长白列位置就可以停了
    {
        if (Longest_White_Column_Right[0] < White_Column[i])//找最长的那一列
        {
            Longest_White_Column_Right[0] = White_Column[i];//【0】是白列长度
            Longest_White_Column_Right[1] = i;              //【1】是下标，第j列
        }
    }
	
    Search_Stop_Line = Longest_White_Column_Left[0];//搜索截止行选取左或者右区别不大，他们两个理论上是一样的
    for (i = MT9V03X_H - 1; i >=MT9V03X_H-Search_Stop_Line; i--)//常规巡线
    {
        for (j = Longest_White_Column_Right[1]; j <= MT9V03X_W - 1 - 2; j++)
        {
            if (image_two_value[i][j] ==IMG_WHITE && image_two_value[i][j + 1] == IMG_BLACK && image_two_value[i][j + 2] == IMG_BLACK)//白黑黑，找到右边界
            {
                right_border = j;
                Right_Lost_Flag[i] = 0; //右丢线数组，丢线置1，不丢线置0
                break;
            }
            else if(j>=MT9V03X_W-1-2)//没找到右边界，把屏幕最右赋值给右边界
            {
                right_border = j;
                Right_Lost_Flag[i] = 1; //右丢线数组，丢线置1，不丢线置0
                break;
            }
        }
        for (j = Longest_White_Column_Left[1]; j >= 0 + 2; j--)//往左边扫描
        {
            if (image_two_value[i][j] ==IMG_WHITE && image_two_value[i][j - 1] == IMG_BLACK && image_two_value[i][j - 2] == IMG_BLACK)//黑黑白认为到达左边界
            {
                left_border = j;
                Left_Lost_Flag[i] = 0; //左丢线数组，丢线置1，不丢线置0
                break;
            }
            else if(j<=0+2)
            {
                left_border = j;//找到头都没找到边，就把屏幕最左右当做边界
                Left_Lost_Flag[i] = 1; //左丢线数组，丢线置1，不丢线置0
                break;
            }
        }
		
        Left_Line [i] = left_border;       //左边线线数组
        Right_Line[i] = right_border;      //右边线线数组
		Mid_Line[i]=(Left_Line [i]+Right_Line[i])/2;
    }
	for(i=1;i<MT9V03X_W;i++)
	{
			for(j=MT9V03X_H;j>=10;j--) 
			{
				   if(image_two_value[j-1][i]==IMG_BLACK && image_two_value[j][i]==IMG_WHITE && image_two_value[j+1][i]==IMG_WHITE) 
					{
							Up_Line[i] = j; 
						    break;
					}
			}
	}
 //边界分析数据
    for (i = MT9V03X_H - 1; i >= 0; i--)//提取赛道数据
    {
        if (Left_Lost_Flag[i]  == 1)//单边丢线数
            Left_Lost_Time++;
        if (Right_Lost_Flag[i] == 1)
            Right_Lost_Time++;
        if (Left_Lost_Flag[i] == 1 && Right_Lost_Flag[i] == 1)//双边丢线数
            Both_Lost_Time++;
        if (Boundry_Start_Left ==  0 && Left_Lost_Flag[i]  != 1)//记录第一个非丢线点，边界起始点
            Boundry_Start_Left = i;
        if (Boundry_Start_Right == 0 && Right_Lost_Flag[i] != 1)
            Boundry_Start_Right = i;
        Road_Wide[i]=Right_Line[i]-Left_Line[i];
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     左补线
  @param     补线的起点，终点
  @return    null
  Sample     Left_Add_Line(int x1,int y1,int x2,int y2);
  @note      补的直接是边界，点最好是可信度高的,不要乱补
-------------------------------------------------------------------------------------------------------------------*/
void Left_Add_Line(int x1,int y1,int x2,int y2)//左补线,补的是边界
{
    int i,max,a1,a2;
    int hx;
    if(x1>=MT9V03X_W-1)//起始点位置校正，排除数组越界的可能
       x1=MT9V03X_W-1;
    else if(x1<=0)
        x1=0;
     if(y1>=MT9V03X_H-1)
        y1=MT9V03X_H-1;
     else if(y1<=0)
        y1=0;
     if(x2>=MT9V03X_W-1)
        x2=MT9V03X_W-1;
     else if(x2<=0)
             x2=0;
     if(y2>=MT9V03X_H-1)
        y2=MT9V03X_H-1;
     else if(y2<=0)
             y2=0;
    a1=y1;
    a2=y2;
    if(a1>a2)//坐标互换
    {
        max=a1;
        a1=a2;
        a2=max;
    }
    for(i=a1;i<=a2;i++)//根据斜率补线即可
    {
        hx=(i-y1)*(x2-x1)/(y2-y1)+x1;
        if(hx>=MT9V03X_W)
            hx=MT9V03X_W;
        else if(hx<=0)
            hx=0;
        Left_Line[i]=hx;
    }
}
/*-------------------------------------------------------------------------------------------------------------------
  @brief     右补线
  @param     补线的起点，终点
  @return    null
  Sample     Right_Add_Line(int x1,int y1,int x2,int y2);
  @note      补的直接是边界，点最好是可信度高的，不要乱补
-------------------------------------------------------------------------------------------------------------------*/
void Right_Add_Line(int x1,int y1,int x2,int y2)//右补线,补的是边界
{
    int i,max,a1,a2;
    int hx;
    if(x1>=MT9V03X_W-1)//起始点位置校正，排除数组越界的可能
       x1=MT9V03X_W-1;
    else if(x1<=0)
        x1=0;
    if(y1>=MT9V03X_H-1)
        y1=MT9V03X_H-1;
    else if(y1<=0)
        y1=0;
    if(x2>=MT9V03X_W-1)
        x2=MT9V03X_W-1;
    else if(x2<=0)
        x2=0;
    if(y2>=MT9V03X_H-1)
        y2=MT9V03X_H-1;
    else if(y2<=0)
         y2=0;
    a1=y1;
    a2=y2;
    if(a1>a2)//坐标互换
    {
        max=a1;
        a1=a2;
        a2=max;
    }
    for(i=a1;i<=a2;i++)//根据斜率补线即可
    {
        hx=(i-y1)*(x2-x1)/(y2-y1)+x1;
        if(hx>=MT9V03X_W)
            hx=MT9V03X_W;
        else if(hx<=0)
            hx=0;
        Right_Line[i]=hx;
    }
}
/*-------------------------------------------------------------------------------------------------------------------
  @brief     左边界延长
  @param     延长起始行数，延长到某行
  @return    null
  Sample     Stop_Detect(void)
  @note      从起始点向上找5个点，算出斜率，向下延长，直至结束点
-------------------------------------------------------------------------------------------------------------------*/
void Lengthen_Left_Boundry(int start,int end)
{
    int i,t;
    float k=0;
    if(start>=MT9V03X_H-1)//起始点位置校正，排除数组越界的可能
        start=MT9V03X_H-1;
    else if(start<=0)
        start=0;
    if(end>=MT9V03X_H-1)
        end=MT9V03X_H-1;
    else if(end<=0)
        end=0;
    if(end<start)//++访问，坐标互换
    {
        t=end;
        end=start;
        start=t;
    }

    if(start<=5)//因为需要在开始点向上找3个点，对于起始点过于靠上，不能做延长，只能直接连线
    {
         Left_Add_Line(Left_Line[start],start,Left_Line[end],end);
    }

    else
    {
        k=(float)(Left_Line[start]-Left_Line[start-4])/5.0;//这里的k是1/斜率
        for(i=start;i<=end;i++)
        {
            Left_Line[i]=(int)(i-start)*k+Left_Line[start];//(x=(y-y1)*k+x1),点斜式变形
            if(Left_Line[i]>=MT9V03X_W-1)
            {
                Left_Line[i]=MT9V03X_W-1;
            }
            else if(Left_Line[i]<=0)
            {
                Left_Line[i]=0;
            }
        }
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     右左边界延长
  @param     延长起始行数，延长到某行
  @return    null
  Sample     Stop_Detect(void)
  @note      从起始点向上找3个点，算出斜率，向下延长，直至结束点
-------------------------------------------------------------------------------------------------------------------*/
void Lengthen_Right_Boundry(int start,int end)
{
    int i,t;
    float k=0;
    if(start>=MT9V03X_H-1)//起始点位置校正，排除数组越界的可能
        start=MT9V03X_H-1;
    else if(start<=0)
        start=0;
    if(end>=MT9V03X_H-1)
        end=MT9V03X_H-1;
    else if(end<=0)
        end=0;
    if(end<start)//++访问，坐标互换
    {
        t=end;
        end=start;
        start=t;
    }

    if(start<=5)//因为需要在开始点向上找3个点，对于起始点过于靠上，不能做延长，只能直接连线
    {
        Right_Add_Line(Right_Line[start],start,Right_Line[end],end);
    }
    else
    {
        k=(float)(Right_Line[start]-Right_Line[start-4])/5.0;//这里的k是1/斜率
        for(i=start;i<=end;i++)
        {
            Right_Line[i]=(int)(i-start)*k+Right_Line[start];//(x=(y-y1)*k+x1),点斜式变形
            if(Right_Line[i]>=MT9V03X_W-1)
            {
                Right_Line[i]=MT9V03X_W-1;
            }
            else if(Right_Line[i]<=0)
            {
                Right_Line[i]=0;
            }
        }
    }
}
/*-------------------------------------------------------------------------------------------------------------------
  @brief     找下面的两个拐点，供十字使用
  @param     搜索的范围起点，终点
  @return    修改两个全局变量
             Right_Down_Find=0;
             Left_Down_Find=0;
  Sample     Find_Down_Point(int start,int end)
  @note      运行完之后查看对应的变量，注意，没找到时对应变量将是0
-------------------------------------------------------------------------------------------------------------------*/
void Find_Down_Point(int start,int end)
{
    int i,t;
    Right_Down_Find=0;
    Left_Down_Find=0;
    if(start<end)
    {
        t=start;
        start=end;
        end=t;
    }
    if(start>=MT9V03X_H-1-5)//下面5行数据不稳定，不能作为边界点来判断，舍弃
        start=MT9V03X_H-1-5;
    if(end<=MT9V03X_H-Search_Stop_Line)
        end=MT9V03X_H-Search_Stop_Line;
    if(end<=5)
       end=5;
    for(i=start;i>=end;i--)
    {
        if(Left_Down_Find==0&&//只找第一个符合条件的点
           abs(Left_Line[i]-Left_Line[i+1])<=5&&//角点的阈值可以更改
           abs(Left_Line[i+1]-Left_Line[i+2])<=5&&
           abs(Left_Line[i+2]-Left_Line[i+3])<=5&&
              (Left_Line[i]-Left_Line[i-2])>=8&&
              (Left_Line[i]-Left_Line[i-3])>=15&&
              (Left_Line[i]-Left_Line[i-4])>=15)
        {
            Left_Down_Find=i;//获取行数即可
        }
        if(Right_Down_Find==0&&//只找第一个符合条件的点
           abs(Right_Line[i]-Right_Line[i+1])<=5&&//角点的阈值可以更改
           abs(Right_Line[i+1]-Right_Line[i+2])<=5&&
           abs(Right_Line[i+2]-Right_Line[i+3])<=5&&
              (Right_Line[i]-Right_Line[i-2])<=-8&&
              (Right_Line[i]-Right_Line[i-3])<=-15&&
              (Right_Line[i]-Right_Line[i-4])<=-15)
        {
            Right_Down_Find=i;
        }
        if(Left_Down_Find!=0&&Right_Down_Find!=0)//两个找到就退出
        {
            break;
        }
    }
}
 
/*-------------------------------------------------------------------------------------------------------------------
  @brief     找上面的两个拐点，供十字使用
  @param     搜索的范围起点，终点
  @return    修改两个全局变量
             Left_Up_Find=0;
             Right_Up_Find=0;
  Sample     Find_Up_Point(int start,int end)
  @note      运行完之后查看对应的变量，注意，没找到时对应变量将是0
-------------------------------------------------------------------------------------------------------------------*/
void Find_Up_Point(int start,int end)
{
    int i,t;
    Left_Up_Find=0;
    Right_Up_Find=0;
    if(start<end)
    {
        t=start;
        start=end;
        end=t;
    }
    if(end<=MT9V03X_H-Search_Stop_Line)
        end=MT9V03X_H-Search_Stop_Line;
    if(end<=5)//及时最长白列非常长，也要舍弃部分点，防止数组越界
        end=5;
    if(start>=MT9V03X_H-1-5)//下面5行数据不稳定，不能作为边界点来判断，舍弃
        start=MT9V03X_H-1-5;
    for(i=start;i>=end;i--)
    {
        if(Left_Up_Find==0&&//只找第一个符合条件的点
           abs(Left_Line[i]-Left_Line[i-1])<=5&&
           abs(Left_Line[i-1]-Left_Line[i-2])<=5&&
           abs(Left_Line[i-2]-Left_Line[i-3])<=5&&
              (Left_Line[i]-Left_Line[i+2])>=8&&
              (Left_Line[i]-Left_Line[i+3])>=15&&
              (Left_Line[i]-Left_Line[i+4])>=15)
        {
            Left_Up_Find=i;//获取行数即可
        }
        if(Right_Up_Find==0&&//只找第一个符合条件的点
           abs(Right_Line[i]-Right_Line[i-1])<=5&&//下面两行位置差不多
           abs(Right_Line[i-1]-Right_Line[i-2])<=5&&
           abs(Right_Line[i-2]-Right_Line[i-3])<=5&&
              (Right_Line[i]-Right_Line[i+2])<=-8&&
              (Right_Line[i]-Right_Line[i+3])<=-15&&
              (Right_Line[i]-Right_Line[i+4])<=-15)
        {
            Right_Up_Find=i;//获取行数即可
        }
        if(Left_Up_Find!=0&&Right_Up_Find!=0)//下面两个找到就出去
        {
            break;
        }
    }
    if(abs(Right_Up_Find-Left_Up_Find)>=30)//纵向撕裂过大，视为误判
    {
        Right_Up_Find=0;
        Left_Up_Find=0;
    }
}
float Slope_Calculate(uint8 begin, uint8 end, volatile int *border)
{
	float xsum = 0, ysum = 0, xysum = 0, x2sum = 0;
	int16 i = 0;
	float result = 0;
	static float resultlast;

	for (i = begin; i < end; i++)
	{
		xsum += i;
		ysum += border[i];
		xysum += i * (border[i]);
		x2sum += i * i;

	}
	if ((end - begin)*x2sum - xsum * xsum) 
	{
		result = ((end - begin)*xysum - xsum * ysum) / ((end - begin)*x2sum - xsum * xsum);
		resultlast = result;
	}
	else
	{
		result = resultlast;
	}
	return -result;
}
void Cross_Detect()
{
    int down_search_start=0;//下角点搜索开始行
	if(Cross_Flag==0)//处理十字时需要强制执行1，防止抢占
	{
		Cross_Flag=0;
		Left_Cross_Flag=0;
		Right_Cross_Flag=0;
	}	
    if(Island_Flag==0)//与环岛互斥开
    {
        Left_Up_Find=0;
        Right_Up_Find=0;
        if(Both_Lost_Time>=10)//十字必定有双边丢线，在有双边丢线的情况下再开始找角点
        {
            Find_Up_Point( MT9V03X_H-1, 0 );
            if(Left_Up_Find==0&&Right_Up_Find==0)//只要没有同时找到两个上点，直接结束
            {
                return;
            }
        }
        if(Left_Up_Find!=0&&Right_Up_Find!=0)//找到两个上点，就认为找到十字了
        {
		    Cross_Flag=1;//确定对应标志位，便于各元素互斥掉
			Cross_State=1;
            down_search_start=Left_Up_Find>Right_Up_Find?Left_Up_Find:Right_Up_Find;//用两个上拐点坐标靠下者作为下点的搜索上限
            Find_Down_Point(MT9V03X_H-5,down_search_start+2);//在上拐点下2行作为下角点的截止行
            if(Left_Down_Find<=Left_Up_Find)
            {
                Left_Down_Find=0;//下点不可能比上点还靠上
            }
            if(Right_Down_Find<=Right_Up_Find)
            {
                Right_Down_Find=0;//下点不可能比上点还靠上
            }
            if(Left_Down_Find!=0&&Right_Down_Find!=0)
            {//四个点都在，无脑连线，这种情况显然很少
                Left_Add_Line (Left_Line [Left_Up_Find ],Left_Up_Find ,Left_Line [Left_Down_Find ] ,Left_Down_Find);
                Right_Add_Line(Right_Line[Right_Up_Find],Right_Up_Find,Right_Line[Right_Down_Find],Right_Down_Find);
            }
            else if(Left_Down_Find==0&&Right_Down_Find!=0)//11//这里使用的是斜率补线
            {//三个点                                     //01
                Lengthen_Left_Boundry(Left_Up_Find-1,MT9V03X_H-1);
                Right_Add_Line(Right_Line[Right_Up_Find],Right_Up_Find,Right_Line[Right_Down_Find],Right_Down_Find);
            }
            else if(Left_Down_Find!=0&&Right_Down_Find==0)//11
            {//三个点                                      //10
                Left_Add_Line (Left_Line [Left_Up_Find ],Left_Up_Find ,Left_Line [Left_Down_Find ] ,Left_Down_Find);
                Lengthen_Right_Boundry(Right_Up_Find-1,MT9V03X_H-1);
            }
            else if(Left_Down_Find==0&&Right_Down_Find==0)//11
            {//就俩上点                                    //00
                Lengthen_Left_Boundry (Left_Up_Find-1,MT9V03X_H-1);
                Lengthen_Right_Boundry(Right_Up_Find-1,MT9V03X_H-1);
            }

        }
        else
        {
			if(Cross_Flag==0)//处理十字时需要强制执行1，防止抢占
			{
				Cross_Flag=0;
				Left_Cross_Flag=0;
				Right_Cross_Flag=0;
			}	
        }
    }
}
/****************圆环**************/
/*-------------------------------------------------------------------------------------------------------------------
  @brief     左赛道连续性检测
  @param     起始点，终止点
  @return    连续返回0，不连续返回断线出行数
  Sample     Continuity_Change_Left(int start,int end);
  @note      连续性的阈值设置为5，可更改
-------------------------------------------------------------------------------------------------------------------*/
int Continuity_Change_Left(int start,int end)//连续性阈值设置为5
{
    int i;
    int t;
    int continuity_change_flag=0;
    if(Left_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没必要判断了
       return 1;
    if(Search_Stop_Line<=5)//搜所截止行很矮
       return 1;
    if(start>=MT9V03X_H-1-5)//数组越界保护
        start=MT9V03X_H-1-5;
    if(end<=5)
       end=5;
    if(start<end)//都是从下往上计算的，反了就互换一下
    {
       t=start;
       start=end;
       end=t;
    }
    
    for(i=start;i>=end;i--)
    {
       if(abs(Left_Line[i]-Left_Line[i-1])>=5)//连续判断阈值是5,可更改
       {
            continuity_change_flag=i;
            break;
       }
    }
    return continuity_change_flag;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     右赛道连续性检测
  @param     起始点，终止点
  @return    连续返回0，不连续返回断线出行数
  Sample     continuity_change_flag=Continuity_Change_Right(int start,int end)
  @note      连续性的阈值设置为5，可更改
-------------------------------------------------------------------------------------------------------------------*/
int Continuity_Change_Right(int start,int end)
{
    int i;
    int t;
    int continuity_change_flag=0;
    if(Right_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没必要判断了
       return 1;
    if(start>=MT9V03X_H-5)//数组越界保护
        start=MT9V03X_H-5;
    if(end<=5)
       end=5;
    if(start<end)//都是从下往上计算的，反了就互换一下
    {
       t=start;
       start=end;
       end=t;
    }

    for(i=start;i>=end;i--)
    {
        if(abs(Right_Line[i]-Right_Line[i-1])>=5)//连续性阈值是5，可更改
       {
            continuity_change_flag=i;
            break;
       }
    }
    return continuity_change_flag;
}
/*-------------------------------------------------------------------------------------------------------------------
  @brief     左下角点检测
  @param     起始点，终止点
  @return    返回角点所在的行数，找不到返回0
  Sample     Find_Left_Down_Point(int start,int end);
  @note      角点检测阈值可根据实际值更改
-------------------------------------------------------------------------------------------------------------------*/
int Find_Left_Down_Point(int start,int end)//找四个角点，返回值是角点所在的行数
{
    int i,t;
    int left_down_line=0;
    if(Left_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没有拐点判断的意义
       return left_down_line;
    if(start<end)
    {
        t=start;
        start=end;
        end=t;
    }
    if(start>=MT9V03X_H-1-5)//下面5行数据不稳定，不能作为边界点来判断，舍弃
        start=MT9V03X_H-1-5;
    if(end<=MT9V03X_H-Search_Stop_Line)
        end=MT9V03X_H-Search_Stop_Line;
    if(end<=5)
       end=5;
    for(i=start;i>=end;i--)
    {
        if(left_down_line==0&&//只找第一个符合条件的点
           abs(Left_Line[i]-Left_Line[i+1])<=5&&//角点的阈值可以更改
           abs(Left_Line[i+1]-Left_Line[i+2])<=5&&  
           abs(Left_Line[i+2]-Left_Line[i+3])<=5&&
              (Left_Line[i]-Left_Line[i-2])>=5&&
              (Left_Line[i]-Left_Line[i-3])>=10&&
              (Left_Line[i]-Left_Line[i-4])>=10)
        {
            left_down_line=i;//获取行数即可
            break;
        }
    }
    return left_down_line;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     左上角点检测
  @param     起始点，终止点
  @return    返回角点所在的行数，找不到返回0
  Sample     Find_Left_Up_Point(int start,int end);
  @note      角点检测阈值可根据实际值更改
-------------------------------------------------------------------------------------------------------------------*/
int Find_Left_Up_Point(int start,int end)//找四个角点，返回值是角点所在的行数
{
    int i,t;
    int left_up_line=0;
    if(Left_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没有拐点判断的意义
       return left_up_line;
    if(start<end)
    {
        t=start;
        start=end;
        end=t;
    }
    if(end<=MT9V03X_H-Search_Stop_Line)//搜索截止行往上的全都不判
        end=MT9V03X_H-Search_Stop_Line;
    if(end<=5)//及时最长白列非常长，也要舍弃部分点，防止数组越界
        end=5;
    if(start>=MT9V03X_H-1-5)
        start=MT9V03X_H-1-5;
    for(i=start;i>=end;i--)
    {
        if(left_up_line==0&&//只找第一个符合条件的点
           abs(Left_Line[i]-Left_Line[i-1])<=5&&
           abs(Left_Line[i-1]-Left_Line[i-2])<=5&&  
           abs(Left_Line[i-2]-Left_Line[i-3])<=5&&
              (Left_Line[i]-Left_Line[i+2])>=8&& 
              (Left_Line[i]-Left_Line[i+3])>=15&&
              (Left_Line[i]-Left_Line[i+4])>=15)
        {
            left_up_line=i;//获取行数即可
            break;
        }
    }
    return left_up_line;//如果是MT9V03X_H-1，说明没有这么个拐点
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     右下角点检测
  @param     起始点，终止点
  @return    返回角点所在的行数，找不到返回0
  Sample     Find_Right_Down_Point(int start,int end);
  @note      角点检测阈值可根据实际值更改
-------------------------------------------------------------------------------------------------------------------*/
int Find_Right_Down_Point(int start,int end)//找四个角点，返回值是角点所在的行数
{
    int i,t;
    int right_down_line=0;
    if(Right_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没有拐点判断的意义
        return right_down_line;
    if(start<end)
    {
        t=start;
        start=end;
        end=t;
    }
    if(start>=MT9V03X_H-1-5)//下面5行数据不稳定，不能作为边界点来判断，舍弃
        start=MT9V03X_H-1-5;
    if(end<=MT9V03X_H-Search_Stop_Line)
        end=MT9V03X_H-Search_Stop_Line;
    if(end<=5)
       end=5;
    for(i=start;i>=end;i--)
    {
        if(right_down_line==0&&//只找第一个符合条件的点
           abs(Right_Line[i]-Right_Line[i+1])<=5&&//角点的阈值可以更改
           abs(Right_Line[i+1]-Right_Line[i+2])<=5&&  
           abs(Right_Line[i+2]-Right_Line[i+3])<=5&&
              (Right_Line[i]-Right_Line[i-2])<=-5&&
              (Right_Line[i]-Right_Line[i-3])<=-10&&
              (Right_Line[i]-Right_Line[i-4])<=-10)
        {
            right_down_line=i;//获取行数即可
            break;
        }
    }
    return right_down_line;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     右上角点检测
  @param     起始点，终止点
  @return    返回角点所在的行数，找不到返回0
  Sample     Find_Right_Up_Point(int start,int end);
  @note      角点检测阈值可根据实际值更改
-------------------------------------------------------------------------------------------------------------------*/
int Find_Right_Up_Point(int start,int end)//找四个角点，返回值是角点所在的行数
{
    int i,t;
    int right_up_line=0;
    if(Right_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没有拐点判断的意义
        return right_up_line;
    if(start<end)
    {
        t=start;
        start=end;
        end=t;
    }    
    if(end<=MT9V03X_H-Search_Stop_Line)//搜索截止行往上的全都不判
        end=MT9V03X_H-Search_Stop_Line;
    if(end<=5)//及时最长白列非常长，也要舍弃部分点，防止数组越界
        end=5;
    if(start>=MT9V03X_H-1-5)
        start=MT9V03X_H-1-5;
    for(i=start;i>=end;i--)
    {
        if(right_up_line==0&&//只找第一个符合条件的点
           abs(Right_Line[i]-Right_Line[i-1])<=5&&//下面两行位置差不多
           abs(Right_Line[i-1]-Right_Line[i-2])<=5&&  
           abs(Right_Line[i-2]-Right_Line[i-3])<=5&&
              (Right_Line[i]-Right_Line[i+2])<=-8&&
              (Right_Line[i]-Right_Line[i+3])<=-15&&
              (Right_Line[i]-Right_Line[i+4])<=-15)
        {
            right_up_line=i;//获取行数即可
            break;
        }
    }
    return right_up_line;
}
/*-------------------------------------------------------------------------------------------------------------------
  @brief     单调性突变检测
  @param     起始点，终止行
  @return    点所在的行数，找不到返回0
  Sample     Find_Right_Up_Point(int start,int end);
  @note      前5后5它最大（最小），那他就是角点
-------------------------------------------------------------------------------------------------------------------*/
int Monotonicity_Change_Left(int start,int end)//单调性改变，返回值是单调性改变点所在的行数
{
    int i;
    int monotonicity_change_line=0;
    if(Left_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没有单调性判断的意义
       return monotonicity_change_line;
    if(start>=MT9V03X_H-1-5)//数组越界保护，在判断第i个点时
       start=MT9V03X_H-1-5; //要访问它前后5个点，数组两头的点要不能作为起点终点
    if(end<=5)
        end=5;
    if(start<=end)//递减计算，入口反了，直接返回0
      return monotonicity_change_line;
    for(i=start;i>=end;i--)//会读取前5后5数据，所以前面对输入范围有要求
    {
        if(Left_Line[i]==Left_Line[i+5]&&Left_Line[i]==Left_Line[i-5]&&
        Left_Line[i]==Left_Line[i+4]&&Left_Line[i]==Left_Line[i-4]&&
        Left_Line[i]==Left_Line[i+3]&&Left_Line[i]==Left_Line[i-3]&&
        Left_Line[i]==Left_Line[i+2]&&Left_Line[i]==Left_Line[i-2]&&
        Left_Line[i]==Left_Line[i+1]&&Left_Line[i]==Left_Line[i-1])
        {//一堆数据一样，显然不能作为单调转折点
            continue;
        }
        else if(Left_Line[i]>=Left_Line[i+5]&&Left_Line[i]>=Left_Line[i-5]&&
        Left_Line[i]>=Left_Line[i+4]&&Left_Line[i]>=Left_Line[i-4]&&
        Left_Line[i]>=Left_Line[i+3]&&Left_Line[i]>=Left_Line[i-3]&&
        Left_Line[i]>=Left_Line[i+2]&&Left_Line[i]>=Left_Line[i-2]&&
        Left_Line[i]>=Left_Line[i+1]&&Left_Line[i]>=Left_Line[i-1])
        {//就很暴力，这个数据是在前5，后5中最大的（可以取等），那就是单调突变点
            monotonicity_change_line=i;
            break;
        }
    }
    return monotonicity_change_line;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     单调性突变检测
  @param     起始点，终止行
  @return    点所在的行数，找不到返回0
  Sample     Find_Right_Up_Point(int start,int end);
  @note      前5后5它最大（最小），那他就是角点
-------------------------------------------------------------------------------------------------------------------*/
int Monotonicity_Change_Right(int start,int end)//单调性改变，返回值是单调性改变点所在的行数
{
    int i;
    int monotonicity_change_line=0;

    if(Right_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没有单调性判断的意义
        return monotonicity_change_line;
    if(start>=MT9V03X_H-1-5)//数组越界保护
        start=MT9V03X_H-1-5;
     if(end<=5)
        end=5;
    if(start<=end)
        return monotonicity_change_line;
    for(i=start;i>=end;i--)//会读取前5后5数据，所以前面对输入范围有要求
    {
        if(Right_Line[i]==Right_Line[i+5]&&Right_Line[i]==Right_Line[i-5]&&
        Right_Line[i]==Right_Line[i+4]&&Right_Line[i]==Right_Line[i-4]&&
        Right_Line[i]==Right_Line[i+3]&&Right_Line[i]==Right_Line[i-3]&&
        Right_Line[i]==Right_Line[i+2]&&Right_Line[i]==Right_Line[i-2]&&
        Right_Line[i]==Right_Line[i+1]&&Right_Line[i]==Right_Line[i-1])
        {//一堆数据一样，显然不能作为单调转折点
            continue;
        }
        else if(Right_Line[i]<=Right_Line[i+5]&&Right_Line[i]<=Right_Line[i-5]&&
        Right_Line[i]<=Right_Line[i+4]&&Right_Line[i]<=Right_Line[i-4]&&
        Right_Line[i]<=Right_Line[i+3]&&Right_Line[i]<=Right_Line[i-3]&&
        Right_Line[i]<=Right_Line[i+2]&&Right_Line[i]<=Right_Line[i-2]&&
        Right_Line[i]<=Right_Line[i+1]&&Right_Line[i]<=Right_Line[i-1])
        {//就很暴力，这个数据是在前5，后5中最大的，那就是单调突变点
            monotonicity_change_line=i;
            break;
        }
    }
    return monotonicity_change_line;
}
/*-------------------------------------------------------------------------------------------------------------------
  @brief     通过斜率，定点补线--
  @param     k       输入斜率
             startY  输入起始点纵坐标
             endY    结束点纵坐标
  @return    null
  Sample     K_Add_Boundry_Left(float k,int startY,int endY);
  @note      补得线直接贴在边线上
-------------------------------------------------------------------------------------------------------------------*/
void K_Add_Boundry_Left(float k,int startX,int startY,int endY)
{
    int i,t;
    if(startY>=MT9V03X_H-1)
        startY=MT9V03X_H-1;
    else if(startY<=0)
        startY=0;
    if(endY>=MT9V03X_H-1)
        endY=MT9V03X_H-1;
    else if(endY<=0)
        endY=0;
    if(startY<endY)//--操作，start需要大
    {
        t=startY;
        startY=endY;
        endY=t;
    }
    for(i=startY;i>=endY;i--)
    {
        Left_Line[i]=(int)((i-startY)/k+startX);//(y-y1)=k(x-x1)变形，x=(y-y1)/k+x1
        if(Left_Line[i]>=MT9V03X_W-1)
        {
            Left_Line[i]=MT9V03X_W-1;
        }
        else if(Left_Line[i]<=0)
        {
            Left_Line[i]=0;
        }
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     通过斜率，定点补线
  @param     k       输入斜率
             startY  输入起始点纵坐标
             endY    结束点纵坐标
  @return    null    直接补边线
  Sample     K_Add_Boundry_Right(float k,int startY,int endY);
  @note      补得线直接贴在边线上
-------------------------------------------------------------------------------------------------------------------*/
void K_Add_Boundry_Right(float k,int startX,int startY,int endY)
{
    int i,t;
    if(startY>=MT9V03X_H-1)
        startY=MT9V03X_H-1;
    else if(startY<=0)
        startY=0;
    if(endY>=MT9V03X_H-1)
        endY=MT9V03X_H-1;
    else if(endY<=0)
        endY=0;
    if(startY<endY)
    {
        t=startY;
        startY=endY;
        endY=t;
    }
    for(i=startY;i>=endY;i--)
    {
        Right_Line[i]=(int)((i-startY)/k+startX);//(y-y1)=k(x-x1)变形，x=(y-y1)/k+x1
        if(Right_Line[i]>=MT9V03X_W-1)
        {
            Right_Line[i]=MT9V03X_W-1;
        }
        else if(Right_Line[i]<=0)
        {
            Right_Line[i]=0;
        }
    }
}
void Island_Detect()
{
    static float k=0;//3和5状态的k
    static int island_state_5_down[2]={0};//状态5时即将离开环岛，左右边界边最低点，[0]存y，第某行，{1}存x，第某列
    static int island_state_3_up[2]={0};//状态3时即将进入环岛用，左右上面角点[0]存y，第某行，{1}存x，第某列
    static int left_down_guai[2]={0};//四个拐点的坐标存储，[0]存y，第某行，{1}存x，第某列
    static int right_down_guai[2]={0};//四个拐点的坐标存储，[0]存y，第某行，{1}存x，第某列
    int monotonicity_change_line[2];//单调性改变点坐标，[0]寸某行，[1]寸某列
    int monotonicity_change_left_flag=0;//不转折是0
    int monotonicity_change_right_flag=0;//不转折是0
    int continuity_change_right_flag=0; //连续是0
    int continuity_change_left_flag=0;  //连续是0
    //以下是常规判断法
    continuity_change_left_flag=Continuity_Change_Left(MT9V03X_H-1-5,10);//连续性判断
    continuity_change_right_flag=Continuity_Change_Right(MT9V03X_H-1-5,10);
    monotonicity_change_right_flag=Monotonicity_Change_Right(MT9V03X_H-1-10,10);
    monotonicity_change_left_flag=Monotonicity_Change_Left(MT9V03X_H-1-10,10);
    if(Cross_Flag==0&&Island_Flag==0)//
    {
        continuity_change_left_flag=Continuity_Change_Left(MT9V03X_H-1-5,10);//连续性判断
        continuity_change_right_flag=Continuity_Change_Right(MT9V03X_H-1-5,10);
        if(Left_Island_Flag==0)//左环
        {
            if(monotonicity_change_right_flag==0&&//右边是单调的
               continuity_change_left_flag!=0&&//左边是不连续的
               continuity_change_right_flag==0&&//左环岛右边是连续的
               Left_Lost_Time>=10&& //左边丢线很多
               Left_Lost_Time<=50&& //也不能全丢了
               Right_Lost_Time<=10&&//右边丢线较少
               Search_Stop_Line>=MT9V03X_H*0.95&&//搜索截止行看到很远
               Boundry_Start_Left>=MT9V03X_H-20&&Boundry_Start_Right>=MT9V03X_H-20&&//边界起始点靠下
               Both_Lost_Time<=10)//双边丢线少
            {
                left_down_guai[0]=Find_Left_Down_Point(MT9V03X_H-1,20);//找左下角点
                if(left_down_guai[0]>=30)//条件1很松，在这里判断拐点，位置不对，则是误判，跳出
                {
                    Island_State=1;
                    Left_Island_Flag=1;
					Island_Flag=1;
                }
                else//误判，归零
                {
                    Island_State=0;
                    Left_Island_Flag=0;
					Island_Flag=0;
                }
            }
        }
        if(Right_Island_Flag==0)//右环
        {
            if(monotonicity_change_left_flag==0&&
               continuity_change_left_flag==0&&//右环岛左边是连续的
               continuity_change_right_flag!=1&&//右边是不连续的
               Right_Lost_Time>=10&&           //右丢线多
               Right_Lost_Time<=50&&           //右丢线不能太多
               Left_Lost_Time<=10&&            //左丢线少
               Search_Stop_Line>=MT9V03X_H*0.95&&//搜索截止行看到很远
               Boundry_Start_Left>=MT9V03X_H-20&&Boundry_Start_Right>=MT9V03X_H-20&&//边界起始点靠下
               Both_Lost_Time<=10)
            {
                right_down_guai[0]=Find_Right_Down_Point(MT9V03X_H-1,20);//右下点
                if(right_down_guai[0]>=30)//条件1很松，在这里加判拐点，位置不对，则是误判，跳出
                {
                    Island_State=1;
                    Right_Island_Flag=1;
					Island_Flag=1;
                }
                else
                {
                    Island_State=0;
                    Right_Island_Flag=0;
					Island_Flag=0;
                }
            }
        }
    }
}
/**********************************************************上线计算*****************************************************************************/
const uint8 Weight2[MT9V03X_W]=
{
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,              //图像最远端00 ——09 行权重
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,              //图像最远端10 ——19 行权重
        19,17,15,19,17, 19,17,15,13,11,             //图像最远端20 ——29 行权重
        6, 7, 9,11,13,15,17,19,20,20,              //图像最远端30 ——39 行权重
       19,17,15,13,11, 9, 7, 5, 3, 1,              //图像最远端40 ——49 行权重
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,              //图像最远端50 ——59 行权重
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,              //图像最远端60 ——69 行权重

};
int Up_error()
{
    int j=0,i=0,Up_sum=0,l_len=0,r_len=0;
    float weight_count=0;
    //常规误差
	if(Island_Flag==0)//非环岛
	{
		for(j=MT9V03X_W-1;j>=5;j--)//常规误差计算
		{
			Up_sum+=Up_Line[j]*Weight[j];//右移1位，等效除2
			weight_count+=Weight[j];
		}
		Up_sum=Up_sum/weight_count;
	}
    else
    {
        for(j=30;j<=90;j++)
        {
            Up_sum+=Up_Line[j];
        }
        Up_sum=(float)Up_sum/60.0;
    }
	for(i=30;i<59;i++)
	{
		l_len+=Up_Line[i];
	}
	l_len=(l_len/30)-10;
	for(i=60;i<90;i++)
	{
		r_len+=Up_Line[i];
	}	
	r_len=(r_len/30)-10;
	lr_err=r_len-l_len-1;
    return Up_sum; 
}
/*-------------------------------------------------------------------------------------------------------------------
  @brief     找左右两个拐点
  @param     无
  @return    修改两个全局变量
             Right_point=0;
             Left_point=0;
  Sample     Find_Point()
  @note      运行完之后查看对应的变量，注意，没找到时对应变量将是0
-------------------------------------------------------------------------------------------------------------------*/
void Find_Point()
{
    int i,t;
    Right_point=0;
    Left_point=0;
    for(i=5;i<=65;i++)
    {
        if(Left_Down_Find==0&&//只找第一个符合条件的点
           abs(Left_Line[i]-Left_Line[i-1])<=5&&//角点的阈值可以更改
           abs(Left_Line[i-1]-Left_Line[i-2])<=5&&
           abs(Left_Line[i-2]-Left_Line[i-3])<=5&&
              (Left_Line[i]-Left_Line[i+2])>=5&&
              (Left_Line[i]-Left_Line[i+3])>=5&&
              (Left_Line[i]-Left_Line[i+4])>=5)
        {
            Left_point=Left_Line[i];//获取列数即可
			if(Longest_White_Column_Left[0]>50)
				L_point_flag=1;
			else L_point_flag=0;
        }
        if(Right_Down_Find==0&&//只找第一个符合条件的点
           abs(Right_Line[i]-Right_Line[i-1])<=5&&//角点的阈值可以更改
           abs(Right_Line[i-1]-Right_Line[i-2])<=5&&
           abs(Right_Line[i-2]-Right_Line[i-3])<=5&&
              (Right_Line[i]-Right_Line[i+2])<=-5&&
              (Right_Line[i]-Right_Line[i+3])<=-5&&
              (Right_Line[i]-Right_Line[i+4])<=-5)
        {
            Right_point=Right_Line[i];
			if(Longest_White_Column_Right[0]>50)
				R_point_flag=1;
			else R_point_flag=0;
        }
        if(Left_point!=0&&Right_point!=0)//两个找到就退出
        {
            break;
        }
    }
}
/*********************************************************************************************************************************************/
int Mid_error()
{
    int i,err=0,l_side=0,r_side=0;
    float weight_count=0;
    //常规误差
	if(Island_Flag==0)//非环岛
	{
		for(i=MT9V03X_H-1;i>=MT9V03X_H-Search_Stop_Line-1;i--)//常规误差计算
		{
			err+=(57-((Left_Line[i]+Right_Line[i])>>1))*Weight[i];//右移1位，等效除2
			weight_count+=Weight[i];
		}
		err=err/weight_count;
	}
    else
    {
         for(i=5;i<=54;i++)
         {
             err+=(57-((Left_Line[i]+Right_Line[i])>>1));
         }
		err=(float)err/50.0;
    }
	for(i=60;i<=69;i++) 
	{
		l_side+=Left_Line[i];
	}
	for(i=60;i<=69;i++) 
	{
		r_side+=Right_Line[i];
	}
	l_side/=10;
	r_side/=10;
	translation=57-((l_side+r_side)>>1);//计算偏移量
	abs(err)<4? err=0:err;
	if(abs(err)>25)
	{
		translation=0;
	}
    return err; 
}
/*-------------------------------------------------------------------------------------------------------------------
  @brief     斑马线判断
  @param     null
  @return    null
  Sample     Zebra_Detect();
  @note      
-------------------------------------------------------------------------------------------------------------------*/
void Zebra_Detect(void)
{
    int j=0;
    int change_count=0;
    if(Island_Flag || Cross_Flag)
		return;
    if(Search_Stop_Line>=60)
    {
        for(j=10;j<=MT9V03X_W-10;j++)
        {
            if(abs(White_Column[j]-White_Column[j+1])>=25)
            {
                change_count++;
            }
        }
    }
    if(change_count>=10)
    {
		if(Zebra_Flag==1 && abs(Odom.milesY)>80)
		{
			Zebra_Flag=2;
			Odom_init();
			return;
		}
		else if(Zebra_Flag==0 && abs(Odom.milesY)>80)
		{
			Zebra_Flag=1;
			Odom_init();
			return;
		}
    }
}

void Camera_Pro(void)        //摄像头处理总函数
{  
     if(mt9v03x_finish_flag)//先处理图像，再清除标志位
     {
			 Threshold=otsuThreshold(mt9v03x_image[0],MT9V03X_W, MT9V03X_H);//大津算阈值
			 Image_Binarization(Threshold);//图像二值化
			 mt9v03x_finish_flag=0;//标志位清除
     }
	 Longest_White_Column();
	 lr_sum=Up_error()-49;
	 Find_Point();
	 if(Cross_Flag==0 && Zebra_Flag==0 && Island_Flag==0 && regular_card_state==0)
	 {
		Cross_Detect();
	 }
	 if(Island_Flag==0 && Cross_Flag==0 && Zebra_Flag==0 && regular_card_state==0)
	 {
		Island_Detect();
	 }
	 if(Cross_Flag==0 || Island_Flag==0 || regular_card_state==0 )
	 {
		Zebra_Detect();
	 }
	 spin=Mid_error();
}

