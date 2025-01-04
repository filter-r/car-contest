#ifndef _IMU660RA_H
#define _IMU660RA_H

typedef struct 
{
    XYZ gyro;
    XYZ acc;
}imu_data_struck;


extern imu_data_struck imu_original_data,zero_offset,imu_angle_data;


void imu_init();
void imu_get();
void imu_get_zero_offset();
void imu_to_angle();


#endif