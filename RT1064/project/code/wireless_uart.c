#include "zf_common_headfile.h"
#include "car.h"
#include "arm.h"
#include "display.h"
#include "images.h"
#include "all_init.h"
#include "imu660ra.h"
#include "wireless_uart.h"
#include "uart_for_art.h"

void wireless_uart()
{
	//电机转速
	printf("%f,%f,%f,%f,%f,%f,%f,%f\n",MotorLF.Fit_Speed,MotorLF.Speed,MotorRF.Fit_Speed,MotorRF.Speed,MotorLR.Fit_Speed,MotorLR.Speed,MotorRR.Fit_Speed,MotorRR.Speed);
	//占空比
	//printf("%hd,%hd,%hd,%hd\n",MotorLF.duty,MotorRF.duty,MotorLR.duty,MotorRR.duty);picture_location
	//printf("%d\n",picture_location[0]);
}






