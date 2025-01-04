#include "zf_common_headfile.h"
#ifndef _IMAGES_H_
#define _IMAGES_H_

#define IMG_BLACK     0      //0x00
#define IMG_WHITE     255      //0xff

//dispaly
/**********数组***********/
extern uint8 image_two_value[MT9V03X_H][MT9V03X_W];
extern int Longest_White_Column_Left[2]; //最长白列,[0]是最长白列的长度，也就是Search_Stop_Line搜索截止行，[1】是第某列
extern int Longest_White_Column_Right[2];//最长白列,[0]是最长白列的长度，也就是Search_Stop_Line搜索截止行，[1】是第某列
extern int Left_Lost_Flag[MT9V03X_H] ; //左丢线数组，丢线置1，没丢线置0
extern int Right_Lost_Flag[MT9V03X_H]; //右丢线数组，丢线置1，没丢线置0
extern volatile int Left_Line[MT9V03X_H]; //左边线数组
extern volatile int Right_Line[MT9V03X_H];//右边线数组
extern volatile int Mid_Line[MT9V03X_H];  //中线数组
extern volatile int Up_Line[MT9V03X_W]; 
extern volatile int White_Column[MT9V03X_W];
/********变量************/
//截至行
extern volatile int Search_Stop_Line;
//十字
extern volatile int Cross_Flag;
extern volatile int Left_Cross_Flag;
extern volatile int Right_Cross_Flag;
extern volatile int Cross_State;
extern volatile int Left_Down_Find; //十字使用，找到被置行数，没找到就是0
extern volatile int Left_Up_Find;   //四个拐点标志
extern volatile int Right_Down_Find;
extern volatile int Right_Up_Find;
extern float Mid_k;
//环岛
extern volatile int Island_State;     //环岛状态标志
extern volatile int Left_Island_Flag; //左右环岛标志
extern volatile int Right_Island_Flag;
extern volatile int	Island_Flag;
extern volatile int L_point_flag;
extern volatile int R_point_flag;
//横移时角点
extern volatile int Right_point;
extern volatile int Left_point;
//斑马线
extern volatile int Zebra_Flag;
extern volatile int change_count;

//相关参数显示。debug使用
//    ips200_show_uint(0*16,50,narrow_road_count,5);
//    ips200_show_uint(1*16,50,change_count,5);
/****************函数******************/
uint8  otsuThreshold(uint8 *image, uint16 col, uint16 row);
void Longest_White_Column(void);
void Image_Binarization(int threshold);
void Image_Binarization(int threshold);
void Camera_Pro(void);
void Find_Up_Point(int start,int end);
void Find_Down_Point(int start,int end);
void Lengthen_Left_Boundry(int start,int end);
void Lengthen_Right_Boundry(int start,int end);
void Left_Add_Line(int x1,int y1,int x2,int y2);
void Right_Add_Line(int x1,int y1,int x2,int y2);
void Cross_Detect(void);
void Show_Boundry(void);
void Island_Detect(void);
int Monotonicity_Change_Right(int start,int end);
int Monotonicity_Change_Left(int start,int end);
int Continuity_Change_Right(int start,int end);
int Continuity_Change_Left(int start,int end);
int Find_Left_Down_Point(int start,int end);
int Find_Left_Up_Point(int start,int end);
int Find_Right_Down_Point(int start,int end);
int Find_Right_Up_Point(int start,int end);
int Get_Road_Wide(int start_line,int end_line);
void K_Add_Boundry_Left(float k,int startX,int startY,int endY);
void K_Add_Boundry_Right(float k,int startX,int startY,int endY);
int Mid_error(void);
void Find_Point();
float Slope_Calculate(uint8 begin, uint8 end, volatile int *border);
#endif


