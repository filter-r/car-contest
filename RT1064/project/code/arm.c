
#include "zf_common_headfile.h"
#include "car.h"
#include "arm.h"
#include "display.h"
#include "images.h"
#include "all_init.h"
#include "imu660ra.h"
#include "wireless_uart.h"
#include "uart_for_art.h"

#define servo_low  PWM4_MODULE2_CHA_C30
#define servo_mid  PWM1_MODULE3_CHA_D0
#define servo_high PWM1_MODULE3_CHB_D1

// low:700(mid)
// mid:ตอะก
// high:ธ฿ะก

void arm_init(){
  
     pwm_init(servo_low, 50, 700);
     pwm_init(servo_mid, 50, 850);
     pwm_init(servo_high, 50, 900);
	
}

void arm_pick_to_first()
{ 
	 
	gpio_init(main_elec, GPO, 1, GPIO_PIN_CONFIG);
	pwm_init(servo_low,50,700);
	pwm_init(servo_high,50,450);
	rt_thread_mdelay(300);
	pwm_init(servo_mid,50,420);
	
	
	
	rt_thread_mdelay(1500);
	pwm_init(servo_mid,50,850);
	pwm_init(servo_high,50,510);
	rt_thread_mdelay(600);
	pwm_init(servo_low,50,500);
	rt_thread_mdelay(600);
	pwm_init(servo_mid,50,780);
	pwm_init(servo_high,50,580);
	rt_thread_mdelay(200);
	gpio_init(main_elec, GPO, 0, GPIO_PIN_CONFIG);
	
	rt_thread_mdelay(800);
	pwm_init(servo_high,50,420);
	rt_thread_mdelay(300);
	pwm_init(servo_low,50,700);
	rt_thread_mdelay(1000);
    pwm_init(servo_mid, 50, 850);
    pwm_init(servo_high, 50, 900);
}
void arm_pick_to_second()
{
	
	gpio_init(main_elec, GPO, 1, GPIO_PIN_CONFIG);
	pwm_init(servo_low,50,700);
	pwm_init(servo_high,50,450);
	rt_thread_mdelay(300);
	pwm_init(servo_mid,50,420);
	
	
	
	rt_thread_mdelay(1500);
	pwm_init(servo_mid,50,850);
	pwm_init(servo_high,50,510);
	rt_thread_mdelay(600);
	pwm_init(servo_low,50,920);
	rt_thread_mdelay(600);
	pwm_init(servo_mid,50,780);
	pwm_init(servo_high,50,540);
	rt_thread_mdelay(200);
	gpio_init(main_elec, GPO, 0, GPIO_PIN_CONFIG);
	
	rt_thread_mdelay(800);
	pwm_init(servo_high,50,420);
	rt_thread_mdelay(300);
	pwm_init(servo_low,50,700);
	rt_thread_mdelay(1000);
    pwm_init(servo_mid, 50, 850);
    pwm_init(servo_high, 50, 900);
}
void arm_pick_to_third()
{
   gpio_init(main_elec, GPO, 1, GPIO_PIN_CONFIG);
	pwm_init(servo_low,50,700);
	pwm_init(servo_high,50,450);
	rt_thread_mdelay(300);
	pwm_init(servo_mid,50,420);
	
	
	
	rt_thread_mdelay(1500);
	pwm_init(servo_mid,50,850);
	pwm_init(servo_high,50,510);
	rt_thread_mdelay(1000);
	pwm_init(servo_low,50,1095);
	rt_thread_mdelay(1000);
	pwm_init(servo_mid,50,700);
	rt_thread_mdelay(300);
	pwm_init(servo_high,50,430);
	rt_thread_mdelay(200);
	gpio_init(main_elec, GPO, 0, GPIO_PIN_CONFIG);
	
	rt_thread_mdelay(800);
	pwm_init(servo_high,50,420);
	rt_thread_mdelay(300);
	pwm_init(servo_low,50,700);
	rt_thread_mdelay(1000);
    pwm_init(servo_mid, 50, 850);
    pwm_init(servo_high, 50, 900);
}
void arm_pick_to_forth()
{
	gpio_init(main_elec, GPO, 1, GPIO_PIN_CONFIG);
	pwm_init(servo_low,50,700);
	pwm_init(servo_high,50,450);
	rt_thread_mdelay(300);
	pwm_init(servo_mid,50,420);
	
	
	
	rt_thread_mdelay(1500);
	pwm_init(servo_mid,50,720);
	pwm_init(servo_high,50,340);
	rt_thread_mdelay(600);
	pwm_init(servo_low,50,250);
	rt_thread_mdelay(1200);
	pwm_init(servo_mid,50,620);
	rt_thread_mdelay(200);
	gpio_init(main_elec, GPO, 0, GPIO_PIN_CONFIG);
	rt_thread_mdelay(400);
	pwm_init(servo_mid,50,720);
	pwm_init(servo_high,50,340);
	rt_thread_mdelay(600);
	pwm_init(servo_low,50,700);
	rt_thread_mdelay(1000);
	pwm_init(servo_mid, 50, 850);
     pwm_init(servo_high, 50, 900);
}
void arm_pick_to_five()
{
	gpio_init(main_elec, GPO, 1, GPIO_PIN_CONFIG);
	pwm_init(servo_low,50,700);
	pwm_init(servo_high,50,450);
	rt_thread_mdelay(300);
	pwm_init(servo_mid,50,420);
	
	
	
	rt_thread_mdelay(1500);
	pwm_init(servo_mid,50,720);
	pwm_init(servo_high,50,330);
	rt_thread_mdelay(600);
	pwm_init(servo_low,50,365);
	rt_thread_mdelay(800);
	pwm_init(servo_mid,50,650);
	rt_thread_mdelay(400);
	gpio_init(main_elec, GPO, 0, GPIO_PIN_CONFIG);
	rt_thread_mdelay(400);
	pwm_init(servo_mid,50,720);
	pwm_init(servo_high,50,330);
	rt_thread_mdelay(600);
	pwm_init(servo_low,50,700);
	rt_thread_mdelay(1000);
	pwm_init(servo_mid, 50, 850);
     pwm_init(servo_high, 50, 900);
}
void arm_pick_from_first()
{
	gpio_init(main_elec, GPO, 1, GPIO_PIN_CONFIG);
	rt_thread_mdelay(600);
	pwm_init(servo_high,50,480);
	rt_thread_mdelay(400);
	pwm_init(servo_low,50,470);
	rt_thread_mdelay(600);
	pwm_init(servo_high,50,680);
	rt_thread_mdelay(600);
	pwm_init(servo_mid, 50, 795);
	
	rt_thread_mdelay(600);
	pwm_init(servo_mid,50,850);
	pwm_init(servo_high,50,480);
	rt_thread_mdelay(600);
	pwm_init(servo_low,50,700);
	rt_thread_mdelay(1000);
	pwm_init(servo_high,50,450);
	pwm_init(servo_mid,50,600);
	rt_thread_mdelay(600);
	gpio_init(main_elec, GPO, 0, GPIO_PIN_CONFIG);
	
	rt_thread_mdelay(1000);
	pwm_init(servo_low, 50, 700);
    pwm_init(servo_mid, 50, 850);
    pwm_init(servo_high, 50, 900);
	
}
void arm_pick_from_second()
{
	gpio_init(main_elec, GPO, 1, GPIO_PIN_CONFIG);
	pwm_init(servo_high,50,450);
	rt_thread_mdelay(600);
	pwm_init(servo_low,50,895);
	rt_thread_mdelay(1000);
	pwm_init(servo_high,50,650);
	rt_thread_mdelay(600);
	pwm_init(servo_mid, 50, 750);
		
	rt_thread_mdelay(600);
	pwm_init(servo_mid,50,850);
	pwm_init(servo_high,50,480);
	rt_thread_mdelay(600);
	pwm_init(servo_low,50,700);
	rt_thread_mdelay(1000);
	pwm_init(servo_high,50,450);
	pwm_init(servo_mid,50,600);
	rt_thread_mdelay(600);
	gpio_init(main_elec, GPO, 0, GPIO_PIN_CONFIG);
	
	rt_thread_mdelay(1000);
	pwm_init(servo_low, 50, 700);
    pwm_init(servo_mid, 50, 850);
    pwm_init(servo_high, 50, 900);
}
void arm_pick_from_third()
{
	gpio_init(main_elec, GPO, 1, GPIO_PIN_CONFIG);
	pwm_init(servo_high,50,450);
	rt_thread_mdelay(600);
	pwm_init(servo_low,50,1085);
	rt_thread_mdelay(1000);
	pwm_init(servo_high,50,560);
	rt_thread_mdelay(600);
	pwm_init(servo_mid, 50, 680);
	
	rt_thread_mdelay(600);
	pwm_init(servo_high,50,450);
	pwm_init(servo_mid, 50, 850);
	rt_thread_mdelay(600);
	pwm_init(servo_low,50,700);
	rt_thread_mdelay(1000);
	pwm_init(servo_high,50,450);
	pwm_init(servo_mid,50,600);
	rt_thread_mdelay(600);
	gpio_init(main_elec, GPO, 0, GPIO_PIN_CONFIG);
	
	rt_thread_mdelay(1000);
	pwm_init(servo_low, 50, 700);
    pwm_init(servo_mid, 50, 850);
    pwm_init(servo_high, 50, 900);
}
void arm_pick_from_forth()
{
	gpio_init(main_elec, GPO, 1, GPIO_PIN_CONFIG);
	pwm_init(servo_high,50,450);
	rt_thread_mdelay(600);
	pwm_init(servo_low,50,250);
	rt_thread_mdelay(1000);
	pwm_init(servo_high,50,550);
	rt_thread_mdelay(1000);
	pwm_init(servo_mid, 50, 620);
	rt_thread_mdelay(1000);
	
	pwm_init(servo_high,50,450);
	pwm_init(servo_mid, 50, 850);
	rt_thread_mdelay(600);
	pwm_init(servo_low,50,700);
	rt_thread_mdelay(1200);
	pwm_init(servo_high,50,450);
	pwm_init(servo_mid,50,600);
	rt_thread_mdelay(600);
	gpio_init(main_elec, GPO, 0, GPIO_PIN_CONFIG);
	
	rt_thread_mdelay(1000);
	pwm_init(servo_low, 50, 700);
    pwm_init(servo_mid, 50, 850);
    pwm_init(servo_high, 50, 900);
}
void arm_pick_from_fifth()
{
	gpio_init(main_elec, GPO, 1, GPIO_PIN_CONFIG);
	pwm_init(servo_high,50,450);
	rt_thread_mdelay(600);
	pwm_init(servo_low,50,365);
	rt_thread_mdelay(1000);
	pwm_init(servo_high,50,470);
	rt_thread_mdelay(600);
	pwm_init(servo_mid, 50, 620);
	
	rt_thread_mdelay(1000);
	pwm_init(servo_high,50,450);
	pwm_init(servo_mid, 50, 850);
	rt_thread_mdelay(1000);
	pwm_init(servo_low, 50, 700);
	rt_thread_mdelay(1000);
	pwm_init(servo_high,50,450);
	pwm_init(servo_mid,50,600);
	rt_thread_mdelay(600);
	gpio_init(main_elec, GPO, 0, GPIO_PIN_CONFIG);
	
	rt_thread_mdelay(1000);
	pwm_init(servo_low, 50, 700);
    pwm_init(servo_mid, 50, 850);
    pwm_init(servo_high, 50, 900);
}
	

