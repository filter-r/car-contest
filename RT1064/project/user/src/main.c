/*********************************************************************************************************************
* RT1064DVL6A Opensourec Library ����RT1064DVL6A ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
* 
* ���ļ��� RT1064DVL6A ��Դ���һ����
* 
* RT1064DVL6A ��Դ�� ���������
* �����Ը���������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù�������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
* 
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
* 
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
* 
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ����֤Э�� ������������Ϊ���İ汾
* ��������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ����֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
* 
* �ļ�����          main
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          IAR 8.32.4 or MDK 5.33
* ����ƽ̨          RT1064DVL6A
* ��������          https://seekfree.taobao.com/
* 
* �޸ļ�¼
* ����              ����                ��ע
* 2022-09-21        SeekFree            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "car.h"
#include "arm.h"
#include "display.h"
#include "images.h"
#include "all_init.h"
#include "imu660ra.h"
#include "wireless_uart.h"
#include "uart_for_art.h"

#define TIMER_ENABLE 1
#define THREAD_ENABLE 1

rt_thread_t th1;
rt_thread_t th2;
rt_thread_t th3;

static struct rt_event movementEvent;
static rt_timer_t tim1=RT_NULL;
static struct rt_timer tim2;

uint8 cnt=0;

enum 
{
	event_key1 =1,
	event_key2,
	event_key3,
	event_key4,
	event_key5,
	event_key6,
	event_key7,
	event_key8,
	event_key9,


};


#define h event_key1


void th1_entry()
{
	while(1){
	
		Camera_Pro();
		rt_thread_mdelay(150);

	}
}

void th2_entry()
{
	while(1){
		
		Set_Car_Movement();
		rt_thread_mdelay(150);
	}
}
void th3_entry()
{
	while(1){
		Judge_cardtype();
		rt_thread_mdelay(150);
	}
}

void timer_10ms_entry()
{
	encoder_get();
	if(++cnt==1)
		return;
	Set_Motor_Movement();
	encoder_to_miles();
	
}
void timer_1ms_entry(){

	imu_get();
	imu_to_angle();
}
void thread_create(){

	th1 = rt_thread_create("th1", th1_entry, RT_NULL, 2048, 10, 22);
	th2 = rt_thread_create("th2", th2_entry, RT_NULL, 1024, 12, 21);
	th3 = rt_thread_create("th3", th3_entry, RT_NULL, 1024, 9, 20);
}
void timer_create()
{
	tim1 = rt_timer_create("tim1", timer_10ms_entry, RT_NULL, 10, RT_TIMER_FLAG_PERIODIC);
	rt_timer_init(&tim2, "tim2", timer_1ms_entry, RT_NULL, 1, RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_HARD_TIMER );
}

void main()
{	
	timer_create();
	thread_create();
	All_Init();
	#if TIMER_ENABLE
		if(tim1 != RT_NULL)
			rt_timer_start(tim1);
		rt_timer_start(&tim2);
	#endif
	#if THREAD_ENABLE
		if(th1 != RT_NULL)
			rt_thread_startup(th1);
		if(th2 != RT_NULL)
			rt_thread_startup(th2);
		if(th3 != RT_NULL)
			rt_thread_startup(th3);
	#endif

	rt_event_init(&movementEvent,"movementEvent", RT_IPC_FLAG_FIFO);
	Main_Switch=Patrol_Mode;
	while(1)
	{		
		IPS_display();
		rt_thread_mdelay(10);
	}
}

