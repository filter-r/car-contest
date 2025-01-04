#include "zf_common_headfile.h"
#include "car.h"
#include "arm.h"
#include "display.h"
#include "images.h"
#include "all_init.h"
#include "imu660ra.h"
#include "wireless_uart.h"
#include "uart_for_art.h"


void elec_gpio_init()
{
	gpio_init(first_elec, GPO, 1, GPIO_PIN_CONFIG);
	gpio_init(second_elec, GPO, 1, GPIO_PIN_CONFIG);
	gpio_init(third_elec, GPO, 1, GPIO_PIN_CONFIG);
	gpio_init(main_elec, GPO, 1, GPIO_PIN_CONFIG);
}

void All_Init(void)
{	
	elec_gpio_init();
	imu_init();
	//rt_thread_mdelay(20);
	mt9v03x_init();
	PID_Init_All();
	encoder_init();
	MotorInit();
	Odom_init();
	Display_Init();
	
	uart4_init();
	uart1_init();
	arm_init();
	
	
	//wireless_uart_init();
}

