#include "zf_common_headfile.h"
#include "car.h"
#include "arm.h"
#include "display.h"
#include "images.h"
#include "all_init.h"
#include "imu660ra.h"
#include "wireless_uart.h"
#include "uart_for_art.h"

imu_data_struck imu_original_data,zero_offset,imu_filtered_data, imu_angle_data;

void imu_init(){

	imu660ra_init();
	imu_get_zero_offset();
}

void imu_get()
{
	imu660ra_get_acc();
	imu660ra_get_gyro();

	imu_original_data.acc.x = (float)imu660ra_acc_x - zero_offset.acc.x;
	imu_original_data.acc.y = (float)imu660ra_acc_y - zero_offset.acc.y;
	imu_original_data.acc.z = (float)imu660ra_acc_z - zero_offset.acc.z;
	imu_original_data.gyro.x = (float)imu660ra_gyro_x - zero_offset.gyro.x;
	imu_original_data.gyro.y = (float)imu660ra_gyro_y - zero_offset.gyro.y;
	imu_original_data.gyro.z = (float)imu660ra_gyro_z - zero_offset.gyro.z;
}
#define   OFFSET_COUNT_TIME   400

void imu_get_zero_offset()
{
	uint16 i;
	int64 temp[6] = {0};

	for(i=1; i<=OFFSET_COUNT_TIME; i++){
	
		imu_get();
		rt_thread_mdelay(5);
//		temp[0]+= (imu660ra_acc_x);
//		temp[1]+= (imu660ra_acc_y);
//		temp[2]+= (imu660ra_acc_z);
		temp[3]+= (imu660ra_gyro_x);
		temp[4]+= (imu660ra_gyro_y);
		temp[5]+= (imu660ra_gyro_z);
	}

//	zero_offset.acc.x = (float)(temp[0] / OFFSET_COUNT_TIME);
//	zero_offset.acc.y = (float)(temp[1] / OFFSET_COUNT_TIME);
//	zero_offset.acc.z = (float)(temp[2] / OFFSET_COUNT_TIME);
	zero_offset.gyro.x = (float)(temp[3] / OFFSET_COUNT_TIME);
	zero_offset.gyro.y = (float)(temp[4] / OFFSET_COUNT_TIME);
	zero_offset.gyro.z = (float)(temp[5] / OFFSET_COUNT_TIME);

}

float myatan(float v)
{
    float v2 = v*v;
    return (v*(1.6867629106f + v2*0.4378497304f)/(1.6867633134f + v2));
}

double my_atan2(double y, double x) {
    // ?? x ? y ?? 0,??? 0
    if (x == 0.0 && y == 0.0) {
        return 0.0;
    }

    double angle = myatan(y / x); // ?? y/x ?????

    // ?? x ? y ?????????
    if (x < 0) {
        angle += 3.1415926f; // ?? p
    } else if (y < 0) {
        angle += 2 * 3.1415926f; // ?? 2p
    }

    return angle;
}
float myfilter(float x){

	if(x>=1)
		return x;
	else if(x<=-1)
		return x;
	else return 0.f;
}
float my_sqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *) &y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *) &i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
#define alpha 0.3f
void imu_filter(){

	// imu_filter_data.gx = LowPassfilter2order(imu.gx);
	// imu_filter_data.gy = LowPassfilter2order(imu.gy);
	// imu_filter_data.gz = LowPassfilter2order(imu.gz);
		
	float acc_angle_x,acc_angle_y;
	static float gyro_angle_x,gyro_angle_y;

	// angles based on accelerometer
	acc_angle_x = my_atan2((imu_original_data.acc.y/4096.f), (imu_original_data.acc.z/4096.f))* 180 / 3.1415926f;                                     // roll
	acc_angle_y = my_atan2(-imu_original_data.acc.x/4096, my_sqrt(imu_original_data.acc.y/4096*imu_original_data.acc.y/4096 + imu_original_data.acc.z/4096*imu_original_data.acc.z/4096))* 180 / 3.1415926f ;    // pitch
	gyro_angle_x+= (imu_original_data.gyro.x/16.4f)/1000.f;
	gyro_angle_y+= (imu_original_data.gyro.y/16.4f)/1000.f;

	// complementary filter
	imu_angle_data.gyro.x = gyro_angle_x * 0.96f + acc_angle_x * 0.04f;
	imu_angle_data.gyro.y = gyro_angle_y * 0.96f + acc_angle_y * 0.04f;


}
void imu_to_angle(){

	imu_angle_data.gyro.x += myfilter(imu_original_data.gyro.x/16.4f)/1000.f;
	imu_angle_data.gyro.y += myfilter(imu_original_data.gyro.y/16.4f)/1000.f;
	imu_angle_data.gyro.z += myfilter(imu_original_data.gyro.z/16.4f)/1000.f;


	Odom.angle= imu_angle_data.gyro.z;
}
void imu_data_clean(){
	imu_angle_data.gyro.x=0;
	imu_angle_data.gyro.y=0;
	imu_angle_data.gyro.z=0;
	Odom.angle=0;
}
