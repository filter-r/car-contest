#include "zf_common_headfile.h"
#include "car.h"
#include "arm.h"
#include "display.h"
#include "images.h"
#include "all_init.h"
#include "imu660ra.h"
#include "wireless_uart.h"
#include "uart_for_art.h"


#define IPS200_TYPE     (IPS200_TYPE_SPI)

/*-------------------------------------------------------------------------------------------------------------------
  @brief     边界显示，用于图传，显示到屏幕上，
  @param     null
  @return    null
  Sample     直接调用
  @note      显示左中右边界，中线，
                                           正常情况下不要用，因为直接在原图上写入了边界信息
                                           会对元素判断造成干扰的，调试时候调用
-------------------------------------------------------------------------------------------------------------------*/
void Show_Boundry(void)
{
    int16 i,j;
    for(i=MT9V03X_H-1;i>=MT9V03X_H-Search_Stop_Line;i--)//从最底下往上扫描
    {
        image_two_value[i][Left_Line[i]+1]=IMG_BLACK;
        image_two_value[i][(Left_Line[i]+Right_Line[i])>>1]=IMG_BLACK;
        image_two_value[i][Right_Line[i]-1]=IMG_BLACK;
    }
	for(j=MT9V03X_W-1;j>=5;j--)
	{
		image_two_value[Up_Line[j]+1][j]=IMG_BLACK;
	}
	ips200_displayimage03x(image_two_value[0], MT9V03X_W, MT9V03X_H);
}
void Display_Init(void)
{
    ips200_init(IPS200_TYPE_SPI);    
    ips200_clear();
}
void IPS_display(void)
{
		Show_Boundry();
	   //75行显示双最长白列数据
	    ips200_show_string(0,75,"bailie:");
	    ips200_show_int(70,75, Longest_White_Column_Right[0],3);//【0】是白列长度
        ips200_show_int(95,75, Longest_White_Column_Right[1],3);//【1】是下标，第j列)
        ips200_show_int(150,75, Longest_White_Column_Left[0],3);//【0】是白列长度
        ips200_show_int(175,75, Longest_White_Column_Left[1],3);//【1】是下标，第j列)
	   //十字
	    ips200_show_string(0,95,"Cross:");
		ips200_show_uint(80,95,Cross_state,3);
        ips200_show_uint(100,95,Cross_Flag,3);
		ips200_show_string(120,95,"L:");
		ips200_show_uint(140,95,Left_Cross_Flag,3);
		ips200_show_string(160,95,"R:");
		ips200_show_uint(180,95,Right_Cross_Flag,3);
	    //环岛
	    ips200_show_string(0,120,"Round:");
		ips200_show_uint(80,120,Island_state,3);
		ips200_show_uint(100,120,Island_Flag,3);
		ips200_show_string(120,120,"L:");	
	    ips200_show_uint(140,120,Left_Island_Flag,3);
		ips200_show_string(160,120,"R:");
		ips200_show_uint(180,120,Right_Island_Flag,3);
		ips200_show_int(200,105,num_card,2);
			//里程计
		ips200_show_string(0,145,"Odom:");
		ips200_show_float(65, 145,Odom.milesX, 3, 1);
		ips200_show_float(115, 145,Odom.milesY, 3, 1);
		ips200_show_float(165, 145,Odom.angle, 3, 1);
//			//陀螺仪
//		ips200_show_string(0,195,"imu:");
//		ips200_show_float(45, 195, imu_angle_data.acc.x, 3,2);
//		ips200_show_float(110, 195,imu_angle_data.acc.y, 3,2);
//		ips200_show_float(170, 195,imu_angle_data.acc.z, 3,2);
//		ips200_show_float(45, 215,imu_angle_data.gyro.x, 3,2);
//		ips200_show_float(110, 215,imu_angle_data.gyro.y, 3,2);
//		ips200_show_float(170, 215,imu_angle_data.gyro.z, 3,2);
			//横向
		ips200_show_string(0,170,"dir_heng:");
		ips200_show_float(90, 170, lr_sum, 3,2);
		ips200_show_float(170, 170, lr_err, 3,2);
			//
		ips200_show_string(0,195,"dir_shu:");
		ips200_show_int(90,195, spin,3);//中线误差
		ips200_show_int(170,195, translation,3);
		//上线角点
		ips200_show_string(0,220,"point:");
		ips200_show_uint(95,220,L_point_flag,3);
		ips200_show_uint(120,220,Left_point,3);
		ips200_show_uint(145,220,R_point_flag,3);
		ips200_show_uint(170,220,Right_point,3);
		//卡片坐标
		ips200_show_char(140,15,target_existence);
		ips200_show_int(140,35,coordinate_sub.x,3);
		ips200_show_int(200,35,coordinate_sub.y,3);
		//斑马线
		ips200_show_string(0,245,"Zebra:");
		ips200_show_int(80,245,Zebra_Flag,3);
		ips200_show_int(130,245,zebra_state,3);
		ips200_show_int(180,245,change_count,3);
		//类别
		ips200_show_string(0,270,"small:");
		ips200_show_int(80,270,class_of_target,5);
		ips200_show_string(120,270,"big:");
		ips200_show_int(180,270,card_bigtype,5);
		
		ips200_show_int(0,300,regular_card_state,5);
		ips200_show_int(150,300,Odom.milesX,3);
		
}