#include "zf_common_headfile.h"
#include "car.h"
#include "arm.h"
#include "display.h"
#include "images.h"
#include "all_init.h"
#include "imu660ra.h"
#include "wireless_uart.h"
#include "uart_for_art.h"


#define ENCODER_RR QTIMER1_ENCODER2
#define ENCODER_LR QTIMER1_ENCODER1
#define ENCODER_RF QTIMER2_ENCODER1
#define ENCODER_LF QTIMER2_ENCODER2

#define MotorRF_PWM                 (PWM2_MODULE2_CHA_C10)
#define MotorLF_PWM                 (PWM2_MODULE3_CHA_D2)

#define MotorLR_PWM                 (PWM2_MODULE0_CHA_C6)
#define MotorRR_PWM                 (PWM2_MODULE1_CHA_C8)

#define MotorRF_DIR               (C11 )
#define MotorLF_DIR               (D3)//Left Front

#define MotorLR_DIR               (C7 )//Left Rear
#define MotorRR_DIR               (C9 )

#define wheel_radius 3 
#define rx 8.7 
#define ry 7.7
double my_abs(double x)
{
	if(x<0) return -x;
	else return x;
}
/***************************************************************Init********************************************************************/
MotorValue_Struct MotorLF,MotorRF,MotorLR,MotorRR;
PID_Struct M1_S_PID, M2_S_PID, M3_S_PID, M4_S_PID, XY_PID, Z_PID,Turn_PID,AIMX_PID,AIMY_PID;

float Pid_Value_Array [10][5]=
{
	//M1~M4(IncrementPID)
	{28, 0.9 , 0.08,   1200},
	{28, 0.9 , 0.08,   1200},
	{28, 0.9 , 0.08,   1200},
	{28, 0.9 , 0.08,   1200},

	//Other(PositPID)
	{0.75 , 0.001 , 0 , 10},//&XY
	{0.8 , 0 , 1 ,0},//&Z

	{0.9, 0 , 0 , 10},//&AIMX
	{0.9 ,0 , 0 , 10},//&AIMY
	{6.0 , 0 , 15.0 ,0},//&Turn
};
void PID_Init(PID_Struct *sptr,char n)
{
    sptr->Kp=Pid_Value_Array[n-1][0];
    sptr->Ki=Pid_Value_Array[n-1][1];
    sptr->Kd=Pid_Value_Array[n-1][2];
	sptr->NowErr=0;
    sptr->LastErr=0;
    sptr->LastOut=0;
    sptr->PastErr=0;
    sptr->SumError=0;
	sptr->OutPut=0;
    sptr->SumErrorMax=Pid_Value_Array[n-1][3];
}
void PID_Init_All(void)
{
        PID_Init(&M1_S_PID,1);
		PID_Init(&M2_S_PID,2);
		PID_Init(&M3_S_PID,3);
		PID_Init(&M4_S_PID,4);
		PID_Init(&XY_PID,5);
		PID_Init(&Z_PID,6);
		PID_Init(&AIMX_PID,7);
		PID_Init(&AIMY_PID,8);
		PID_Init(&Turn_PID,9);
}
void MotorInit(){
    pwm_init(MotorLF_PWM, 17000, 0);                                                
    pwm_init(MotorRF_PWM, 17000, 0);                                                  
    pwm_init(MotorLR_PWM, 17000, 0);                                               
    pwm_init(MotorRR_PWM, 17000, 0);                                                  

    gpio_init(MotorLF_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            
    gpio_init(MotorRF_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            
    gpio_init(MotorLR_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            
    gpio_init(MotorRR_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                           
}
void encoder_init(){

    encoder_quad_init(ENCODER_LR, QTIMER1_ENCODER1_CH1_C0, QTIMER1_ENCODER1_CH2_C1);   
    encoder_quad_init(ENCODER_RR, QTIMER1_ENCODER2_CH1_C2, QTIMER1_ENCODER2_CH2_C24);
    encoder_quad_init(ENCODER_RF, QTIMER2_ENCODER1_CH1_C3, QTIMER2_ENCODER1_CH2_C4);
    encoder_quad_init(ENCODER_LF, QTIMER2_ENCODER2_CH1_C5, QTIMER2_ENCODER2_CH2_C25);
    
}
Odometry Odom={0};
void Odom_Init(Odometry *sptr)
{
    sptr->milesX=0.0;
    sptr->milesY=0.0;
}
void Odom_init()
{
	Odom_Init(&Odom);
}
void setPWM_F(short duty, gpio_pin_enum DIR_pin, pwm_channel_enum PWM_ch) {
	bool negative= duty>0;
	if(!negative)
        duty= -duty;
	
	gpio_set_level(DIR_pin, negative);
	pwm_set_duty(PWM_ch,duty);
}
void setPWM_RF(short duty, gpio_pin_enum DIR_pin, pwm_channel_enum PWM_ch){
	bool negative= duty>0;
	if(duty>=0){
		gpio_set_level(DIR_pin, 1);
		pwm_set_duty(PWM_ch,duty);
	}
	else if(duty<0){
		gpio_set_level(DIR_pin, 1);
		pwm_set_duty(PWM_ch,-duty);
	}
}
void setPWM_R(short duty, gpio_pin_enum DIR_pin, pwm_channel_enum PWM_ch) {
	bool negative= duty>0;
	if(!negative)
        duty= -duty;
	
	gpio_set_level(DIR_pin, !negative);
	pwm_set_duty(PWM_ch,duty);
}
int LimitProtect( int max, int min, float data)
{
    if(data<min){
        return min;}
    else if(data>=max){
        return max;}
    else 
		return data;
}
/******************************************************************PID******************************************************************/
short PositPID(PID_Struct *User, float NowDat, float SetDat)
{
    float NowError,OutPut;
    NowError=SetDat-NowDat;
    User->SumError+=NowError;  
    if(User->SumError>=User->SumErrorMax){
		User->SumError= User->SumErrorMax;}
    else if(User->SumError <= -User->SumErrorMax){
		User->SumError= -User->SumErrorMax;}
    OutPut=User->Kp * NowError
            +User->Ki*User->SumError
            +User->Kd*(NowError - User->LastErr);
    User->PastErr=User->LastErr;
    User->LastErr=NowError;
    User->LastOut=OutPut;   
    return OutPut;
}

int IncrementPID(PID_Struct *User, float Nowdat, float Setdat)
{
	float NowError;
    NowError=Setdat-Nowdat;
    User->OutPut+=User->Kp* (NowError-User->LastErr)+User->Ki* NowError;
    User->PastErr=User->LastErr;  
	User->LastErr=NowError;
	if(User->OutPut>5000) User->OutPut=5000;
	else if(User->OutPut<-5000) User->OutPut=-5000;
    return  User->OutPut;
	
}
int TurnPID(PID_Struct *User, float Nowdat, float Setdat)
{
    float NowError,OutPut;
    NowError=Setdat-Nowdat;
    OutPut=User->Kp*NowError
           	+User->Ki*User->SumError
           	+User->Kd*(NowError-User->LastErr);
    User->PastErr=User->LastErr;    
	User->LastErr=NowError;
	if(OutPut>80) 
		OutPut=80;
	else if(OutPut<-80)
		OutPut=-80;
    return OutPut;
}
int card_xerror_pid(PID_Struct *User, float Nowdat, float Setdat){
    float NowError,OutPut;
    NowError=Setdat-Nowdat;
    User->SumError+=NowError;  
    OutPut=User->Kp*NowError+User->Ki*User->SumError+User->Kd*(NowError-User->LastErr);
	User->LastErr=NowError;
    if(OutPut>=50)
        OutPut=50;
    if(OutPut<=-50)
        OutPut=-50;
    return (int)OutPut;
}
int card_yerror_pid(PID_Struct *User, float Nowdat, float Setdat){
    float NowError,OutPut;
	NowError=Setdat-Nowdat;
    User->SumError+=NowError;  
    OutPut=User->Kp*NowError+User->Ki*User->SumError+User->Kd*(NowError-User->LastErr);
    User->LastErr=NowError;
    if(OutPut>=30)
        OutPut=30;
    if(OutPut<=-30)
        OutPut=-30;
    return (int)OutPut;
}
int card_xdistance_pid(PID_Struct *User, float Nowdat, float Setdat){
    float NowError,OutPut;
	NowError=Setdat-Nowdat;
    User->SumError+=NowError;  
    OutPut=User->Kp*NowError+User->Ki*User->SumError+User->Kd*(NowError-User->LastErr);
    User->LastErr=NowError;
    if(OutPut>=20)
        OutPut=20;
    if(OutPut<=-20)
        OutPut=-20;
    return (int)OutPut;
}
int card_ydistance_pid(PID_Struct *User, float Nowdat, float Setdat){
    float NowError,OutPut;
	NowError=Setdat-Nowdat;
    User->SumError+=NowError;  
    OutPut=User->Kp*NowError+User->Ki*User->SumError+User->Kd*(NowError-User->LastErr);
    User->LastErr=NowError;
    if(OutPut>=30)
        OutPut=30;
    if(OutPut<=-30)
        OutPut=-30;
    return (int)OutPut;
}
/**********************************************************************************************************************************/
// period==10ms
void encoder_get()
{
	MotorRR.Encoder = encoder_get_count(ENCODER_RR);                           
	MotorLR.Encoder = encoder_get_count(ENCODER_LR);                                                                     
	MotorRF.Encoder = encoder_get_count(ENCODER_RF);                              
	MotorLF.Encoder = encoder_get_count(ENCODER_LF); 
									 
	//cm/s
	MotorLF.Speed = - 0.78890440519832977528358790316729f*MotorLF.Encoder;
	MotorRF.Speed =  0.78890440519832977528358790316729f*MotorRF.Encoder;
	MotorLR.Speed = - 0.78890440519832977528358790316729f*MotorLR.Encoder;
	MotorRR.Speed =  0.78890440519832977528358790316729f*MotorRR.Encoder;
		
	encoder_clear_count(ENCODER_LF);
	encoder_clear_count(ENCODER_RF);
	encoder_clear_count(ENCODER_LR);                                   
	encoder_clear_count(ENCODER_RR);     
}

//period==10ms
void encoder_to_miles(){
    
    Odom.speedX = (MotorLF.Speed- MotorRF.Speed- MotorLR.Speed+ MotorRR.Speed)*0.25f;
    Odom.speedY = (MotorLF.Speed+ MotorRF.Speed+ MotorLR.Speed+ MotorRR.Speed)*0.25f;
    Odom.milesX += (Odom.speedX/100);
    Odom.milesY += (Odom.speedY/100);

}

//period==10ms
void encoder_to_angle(){
	
	Odom.speedZ= ((-MotorLF.Speed)+ (-MotorRF.Speed)+ MotorLR.Speed+ MotorRR.Speed)*0.25/(rx+ry)/10;
	Odom.angle += Odom.speedZ/100;
	// cos(Odom.angle);
}

void Judge_cardtype()
{//0 0 -aim
 //1 0 -let
 //0 1 -num
	if(alpha==0 && beta==0)
	{
		uart_write_byte(UART_1, 'a');
	}
	if(alpha==1 && beta==0)
	{
		uart_write_byte(UART_1, 'l');
	}
	if(alpha==0 && beta==1)
	{
		uart_write_byte(UART_1, 'n');
	}
	if(x+y == 0)
	{
		target_existence='F';
		coordinate_sub.x=0;
		coordinate_sub.y=0;	
	}
	else 
	{
		target_existence='T';
		coordinate_sub.x=168-x;
		coordinate_sub.y=y-157;	
	}
	switch(class_of_target)
	{
		case 67://dagger C
		case 66://explosive B
		case 69://fire_axe E
		case 65://firearms A
		case 68://spontoon D
			card_bigtype=1;
			break;
		case 73://bulletproof_vest I
		case 70://first_aid_kit F
		case 71://flashlight G
		case 75://helmet K
		case 72://intercom H
		case 74://telescope J
			card_bigtype=2;
			break;
		case 77://ambulance M
		case 78://armoured_car N
		case 76://fire_engine L
		case 79://motorcycle O
			card_bigtype=3;
			break;
		default:break;
	}	
}

/****************************************************************************************************************************************/

char card_flag;
char Main_Switch;
int num_card=0;

short int translation=0;//Patrol_Mode
short int straight=65;
short int spin=0;

short int x_mistake=0;//Forward_Switch
short int y_mistake=0;


short int traveled_milex;//Backward_Switch
short int traveled_miley;

XYZ coordinate_sub;//AIM

short int lr_err=0;//Sideward_Mode
short int lr_sum=0;
short int transerve=50;

short int Fit_angle;
short int Fit_milex;
short int Fit_miley;
short int Left_card_flag;
short int Right_card_flag;



short int Free_speedx=0;//Free_Mode
short int Free_speedy=0;

//state
int Island_state=0;
int Cross_state=0;
int zebra_state=0;
int regular_card_state=0;
//flag
bool alpha=0;
bool beta=0;

int box_1[2]={0},box_2[2]={0},box_3[2]={0},box_4[2]={0},box_5[2]={0}; 

int mile_record=0;
int traveled_mileX;
/****************************************************************************************************************************************/
//regular_card
void Process_regular_card()
{
	if(regular_card_state==0)
	{
		if(target_existence=='T' && abs(coordinate_sub.y)<40)
		{
			Odom_init();
			if(coordinate_sub.x<-10)
			{
				imu_angle_data.gyro.z=0;
				Fit_angle=-90; 
				Main_Switch=Angle_Mode;
				regular_card_state=1;
				Odom_init();
			}
			if(coordinate_sub.x>10)
			{
				imu_angle_data.gyro.z=0;
				Fit_angle=90;
				Main_Switch=Angle_Mode;
				regular_card_state=1;
				Odom_init();
			}
		}
	}
	else if(regular_card_state==1) 
	{
		Odom_init();
		if(Main_Switch==Angle_Mode && abs(abs(Odom.angle)-abs(Fit_angle))<1)  
		{
			Main_Switch=AIM_Mode;
			regular_card_state=2;
			Odom_init();
		}
	}
	else if(regular_card_state==2)
	{
		if(target_existence=='T' && abs(coordinate_sub.y)<3 && abs(coordinate_sub.x)<3)
		{
			Main_Switch=Stop_Mode;
			rt_thread_mdelay(1000);
			if(card_bigtype==1)
			{
				arm_pick_to_first();
				Odom_init();
				regular_card_state=3;
			}
			else if(card_bigtype==2)
			{
				arm_pick_to_second();
				Odom_init();
				regular_card_state=3;
			}
			else if(card_bigtype==3)
			{
				arm_pick_to_third();
				Odom_init();
				regular_card_state=3;
			}
			else
				return;
		}
	}
	else if(regular_card_state==3)
	{
		Fit_angle=0;
		Main_Switch=Angle_Mode;
		Odom_init();
		regular_card_state=4;
	}
	else if(regular_card_state==4)
	{
		if(Main_Switch==Angle_Mode && abs(abs(Odom.angle)-abs(Fit_angle))<3)
		{
			Odom_init();
			Main_Switch=Patrol_Mode;
			regular_card_state=5;
		}
	}
	else if(regular_card_state==5)
	{	
		if(Odom.milesY>60)
		{
			regular_card_state=0;
		}
	}
}
//Island_card
void Process_Island_card()
{ 
	if(Left_Island_Flag)
	{
		if(Island_state == 0)
		{
			imu_angle_data.gyro.z=0;
			Fit_angle= 90;
			Main_Switch= Angle_Mode;
			Island_state= 1;
		}
		else if(Island_state == 1 )
		{
			if( abs(abs(Odom.angle)- abs(Fit_angle))< 2){
				Odom_init();
				Free_speedx= 35;
				Free_speedy= 2;
				Main_Switch= Free_Mode;
				Island_state= 2;
			}
		}
		else if(Island_state == 2)
		{			
			while(1){
				if(70<abs(Odom.milesX) && abs(Odom.milesX)<125 && target_existence == 'T')
				{
					if(abs(coordinate_sub.y) < 100 && abs(coordinate_sub.y) < 100)
					{
						Main_Switch= Stop_Mode;
						Island_state= 3;
						break;
					}			
				}
				else if(abs(Odom.milesX)>125)
				{
					Fit_angle= 0;
					Main_Switch= Angle_Mode;
					if(Main_Switch == Angle_Mode && abs(abs(Odom.angle)- abs(Fit_angle))< 3)
					{
						Main_Switch=Patrol_Mode;
						Island_state=19;
						break;
					}
				}	
				else{
					rt_thread_mdelay(1);
				}		
			}
		}
		else if(Island_state == 19)
		{
			if(abs(Odom.milesY)>150)
			{
				Right_Island_Flag=0;
				Island_state= 0;
				Island_Flag=0;
				num_card=0;
				alpha=0;
				beta=0;
				box_1[0]=0,box_1[1]=0;
				box_2[0]=0,box_2[1]=0;
				box_3[0]=0,box_3[1]=0;
				box_4[0]=0,box_4[1]=0;
				box_5[0]=0,box_5[1]=0;
			}
		}
		else if(Island_state == 3)
		{
			imu_angle_data.gyro.z=0;
			Fit_angle= 0;
			Main_Switch= AIM_Mode;
			Island_state= 4;
		}
		else if(Island_state == 4)
		{
			if(target_existence=='T' && abs(coordinate_sub.y)<3 && abs(coordinate_sub.x)<3)
			{
				Main_Switch= Stop_Mode;
				if(num_card == 0)
				{
					num_card++;
					box_1[0] = class_of_target;
					box_1[1]++;
					arm_pick_to_first();
					rt_thread_mdelay(2000);
					Island_state= 5;
				}
				else if(num_card == 1)
				{
					num_card++;
					if(class_of_target == box_1[0])
					{
						box_1[1]++;
						arm_pick_to_first();
						rt_thread_mdelay(2000);				
						Island_state= 5;
					}
					else 
					{
						box_2[0] = class_of_target;
						box_2[1]++;
						arm_pick_to_second();
						rt_thread_mdelay(2000);
						Island_state= 5;
					}
				}
				else if(num_card == 2)
				{
					num_card++;
					if(class_of_target == box_1[0])
					{
						box_1[1]++;
						arm_pick_to_first();
						rt_thread_mdelay(2000);
						Island_state= 5;
					}
					else if(class_of_target == box_2[0])
					{
						box_2[1]++;
						arm_pick_to_second();
						rt_thread_mdelay(2000);
						Island_state= 5;
					}
					else
					{
						box_3[0] = class_of_target;
						box_3[1]++;
						arm_pick_to_third();
						rt_thread_mdelay(3000);
						
						Island_state= 5;
					}
				}
				else if(num_card == 3)
				{
					num_card++;
					if(class_of_target == box_1[0])
					{
						box_1[1]++;
						arm_pick_to_first();
						rt_thread_mdelay(2000);
						Island_state= 5;
					}
					else if(class_of_target == box_2[0])
					{
						box_2[1]++;
						arm_pick_to_second();
						rt_thread_mdelay(2000);
						Island_state= 5;
					}
					else if(class_of_target == box_3[0])
					{
						box_3[1]++;
						arm_pick_to_third();
						rt_thread_mdelay(3000);
						Island_state= 5;
					}	
					else{
						box_4[0] = class_of_target;
						box_4[1]++;
						arm_pick_to_forth();
						rt_thread_mdelay(4000);
						
						Island_state= 5;
					}
				}
				else if(num_card == 4)
				{
					num_card++;
					if(class_of_target == box_1[0])
					{
						box_1[1]++;
						arm_pick_to_first();
						rt_thread_mdelay(2000);
						Island_state= 5;
					}
					else if(class_of_target == box_2[0])
					{
						box_2[1]++;
						arm_pick_to_second();
						rt_thread_mdelay(2000);
						Island_state= 5;
					}
						
					else if(class_of_target == box_3[0])
					{
						box_3[1]++;
						arm_pick_to_third();
						rt_thread_mdelay(3000);
						Island_state= 5;
					}	
					else if(class_of_target == box_4[0])
					{
						box_4[1]++;
						arm_pick_to_forth();
						rt_thread_mdelay(4000);
						Island_state= 5;
					}	
					else{
						box_5[0] = class_of_target;
						box_5[1]++;
						arm_pick_to_five();
						rt_thread_mdelay(4000);
						
						Island_state= 5;
					}
				}
				else if(num_card == 5)
				{
					num_card++;
					if(class_of_target == box_1[0])
					{
						box_1[1]++;
						arm_pick_to_first();
						rt_thread_mdelay(2000);
						Island_state= 5;
					}
					else if(class_of_target == box_2[0])
					{
						box_2[1]++;
						arm_pick_to_second();
						rt_thread_mdelay(2000);
						Island_state= 5;
					}
						
					else if(class_of_target == box_3[0])
					{
						box_3[1]++;
						arm_pick_to_third();
						rt_thread_mdelay(3000);
						Island_state= 5;
					}	
					else if(class_of_target == box_4[0])
					{
						box_4[1]++;
						arm_pick_to_forth();
						rt_thread_mdelay(4000);
						Island_state= 5;
					}	
					else{
						box_5[0] = class_of_target;
						box_5[1]++;
						arm_pick_to_five();
						rt_thread_mdelay(4000);
						Island_state= 5;
					}
				}
				else 
				{
					Island_state = 6;
				}
			}
		}
		else if(Island_state == 5)
		{
			if(target_existence == 'T' && abs(coordinate_sub.y)< 80 && abs(coordinate_sub.x)< 80){
				Main_Switch= AIM_Mode;		
				Island_state= 4;
			}
			else if(target_existence == 'F'  ){
				Island_state = 6;
				alpha=1;
				beta=0;
			}		
		}
		else if(Island_state == 6)
		{	
			imu_angle_data.gyro.z=0;
			Fit_angle= 180;
			Main_Switch= Angle_Mode;
			Island_state = 7;
		}
		else if(Island_state == 7)
		{
			if(abs(abs(Odom.angle)- abs(Fit_angle))< 3){
				Odom_init();
				if(target_existence == 'T')
				{
					Main_Switch= AIM_Mode;
					Island_state= 8;
				}
			}
		}
		else if(Island_state == 8)
		{
			if(target_existence == 'T' && abs(coordinate_sub.y)< 4 && abs(coordinate_sub.x)< 4){
				Main_Switch= Stop_Mode;
				rt_thread_mdelay(2000);
				if(class_of_target == box_1[0])
				{
					Odom_init();
					while(box_1[1]>0)
					{	
						num_card--;
						box_1[1]--;
						arm_pick_from_first();
						rt_thread_mdelay(1000);
					
					}
					Island_state= 9;
				}
				else if(class_of_target == box_2[0])
				{
					Odom_init();
					while(box_2[1]>0)
					{
						num_card--;
						box_2[1]--;
						arm_pick_from_second();
						rt_thread_mdelay(1000);
						
					}
					Island_state= 9;
				}
				else if(class_of_target == box_3[0])
				{
					Odom_init();
					while(box_3[1]>0)
					{
						num_card--;
						box_3[1]--;
						arm_pick_from_third();
						rt_thread_mdelay(1000);
						
					}
					Island_state= 9;
				}
				else if(class_of_target == box_4[0])
				{
					Odom_init();
					while(box_4[1]>0)
					{
						num_card--;
						box_4[1]--;
						arm_pick_from_forth();
						rt_thread_mdelay(2000);
						
					}
					Island_state= 9;					
				}
				else if(class_of_target == box_5[0])
				{
					Odom_init();
					while(box_5[1]>0)
					{
						num_card--;
						box_5[1]--;
						arm_pick_from_fifth();
						rt_thread_mdelay(2000);
						
					}
					Island_state= 9;
				}
				else
				{
					Odom_init();
					Island_state = 9;
				}
			}
		}
		else if(Island_state == 9)
		{
			Free_speedx=8;
			Free_speedy=-20;
			Main_Switch = Free_Mode;
			if(abs(Odom.milesY)>50)
			{
				Main_Switch= Stop_Mode;
				Island_state= 10;
			}
		}
		else if(Island_state == 10)
		{
			imu_angle_data.gyro.z=0;
			Fit_angle= -90;
			Main_Switch = Angle_Mode;
			Island_state= 11;	
		}
		else if(Island_state == 11)
		{
			if(abs(abs(Odom.angle)- abs(Fit_angle))< 3){
				Free_speedx=0;
				Free_speedy=20;
				Odom_init();
				imu_angle_data.gyro.z=0;
				Fit_angle=0;
				Main_Switch = Free_Mode;
				Island_state= 12;
			}
		}
		else if(Island_state == 12)
		{
			if(abs(Odom.milesY)>20)
			{
				if(target_existence == 'T'){
				Odom_init();
				Fit_angle = Odom.angle;
				Main_Switch = AIM_Mode;
				Island_state = 13;
				}
			}
		}
		else if(Island_state == 13)
		{
			if(target_existence == 'T' && abs(coordinate_sub.x)< 4 && abs(coordinate_sub.y)< 4){
				//mile_record=abs(Odom.milesY);
				Odom_init();
				Free_speedx=10;
				Free_speedy=-15;
				Main_Switch = Free_Mode;
				Island_state=14;
				
			}
		}
		else if(Island_state == 14)
		{
			if(abs(Odom.milesY)>7)
			{
				Main_Switch= Stop_Mode;
				Odom_init();
				rt_thread_mdelay(2000);
				if(class_of_target == box_1[0])
					{					
						while(box_1[1]>0)
						{
							num_card--;
							box_1[1]--;
							arm_pick_from_first();
							rt_thread_mdelay(1000);
							
						}
						Island_state= 15;
					}
					else if(class_of_target == box_2[0])
					{
						while(box_2[1]>0)
						{
							num_card--;
							box_2[1]--;
							arm_pick_from_second();
							rt_thread_mdelay(1000);
							
						}
						Island_state= 15;
					}
					else if(class_of_target == box_3[0])
					{
						while(box_3[1]>0)
						{
							num_card--;
							box_3[1]--;
							arm_pick_from_third();
							rt_thread_mdelay(1000);
							
						}
						Island_state= 15;
					}
					else if(class_of_target == box_4[0])
					{
						while(box_4[1]>0)
						{
							num_card--;
							box_4[1]--;
							arm_pick_from_forth();
							rt_thread_mdelay(2000);	
						}
						Island_state= 15;					
					}
					else if(class_of_target == box_5[0])
					{
						while(box_5[1]>0)
						{
							num_card--;
							box_5[1]--;
							arm_pick_from_fifth();
							rt_thread_mdelay(2000);
							
						}
						Island_state= 15;
					}
					else
					{
						Island_state = 15;
					}
			}
		
		}
		else if(Island_state == 15)
		{
			Free_speedx=-8;
			Free_speedy=-25;
			Main_Switch = Free_Mode;
			if(abs(Odom.milesY)>30)
			{
				transerve=25;
				Main_Switch= Sideward_Mode;
				Island_state = 16;
			}
		}
		else if(Island_state == 16)
		{
			if(target_existence == 'T' && abs(coordinate_sub.x)< 40 && abs(Odom.angle)>abs(Fit_angle)+30){
				Odom_init();
				Fit_angle = Odom.angle;
				Main_Switch = AIM_Mode;
				Island_state = 13;
			}
			else if(abs(Odom.angle)>170)
			{
				if(L_point_flag && Left_point<90)
				{
					Fit_angle = Odom.angle;
					Main_Switch = Patrol_Mode;
					Odom_init();
					Island_state = 17;
				}
				else if(abs(Odom.angle)>180)
				{
					Odom_init();
					Free_speedx=10;
					Free_speedy=0;
					Fit_angle = Odom.angle;
					Main_Switch = Free_Mode;
					Island_state = 18;
				}
			}
		}
		else if(Island_state == 17)
		{
			if(abs(Odom.milesY)>80)
			{
				regular_card_state=0;
				Left_Island_Flag=0;
				Island_state=0;
				Island_Flag=0;
				num_card=0;
				alpha=0;
				beta=0;
				box_1[0]=0,box_1[1]=0;
				box_2[0]=0,box_2[1]=0;
				box_3[0]=0,box_3[1]=0;
				box_4[0]=0,box_4[1]=0;
				box_5[0]=0,box_5[1]=0;
			}
		}
		else if(Island_state == 18)
		{
			if(abs(Odom.milesX)>15)
			{
				Main_Switch = Patrol_Mode;
				Odom_init();
				Island_state = 17;
			}
		}
	}
	if(Right_Island_Flag)
	{
		if(Island_state == 0)
		{
			imu_angle_data.gyro.z=0;
			Fit_angle= -90;
			Main_Switch= Angle_Mode;
			Island_state= 1;
		}
		else if(Island_state == 1 )
		{
			if( abs(abs(Odom.angle)- abs(Fit_angle))< 2){
				Odom_init();
				Free_speedx= -35;
				Free_speedy= 2;
				Main_Switch= Free_Mode;
				Island_state= 2;
			}
		}
		else if(Island_state == 2)
		{			
			while(1){
				if(70<abs(Odom.milesX) && abs(Odom.milesX)<125 && target_existence == 'T')
				{
					if(abs(coordinate_sub.y) < 100 && abs(coordinate_sub.y) < 100)
					{
						Main_Switch= Stop_Mode;
						Island_state= 3;
						break;
					}			
				}
				else if(abs(Odom.milesX)>125)
				{
					Fit_angle= 0;
					Main_Switch= Angle_Mode;
					if(Main_Switch == Angle_Mode && abs(abs(Odom.angle)- abs(Fit_angle))< 3)
					{
						Main_Switch=Patrol_Mode;
						Island_state=19;
						break;
					}
				}	
				else{
					rt_thread_mdelay(1);
				}		
			}
		}
		else if(Island_state == 19)
		{
			if(abs(Odom.milesY)>150)
			{
				Right_Island_Flag=0;
				Island_state= 0;
				Island_Flag=0;
				num_card=0;
				alpha=0;
				beta=0;
				box_1[0]=0,box_1[1]=0;
				box_2[0]=0,box_2[1]=0;
				box_3[0]=0,box_3[1]=0;
				box_4[0]=0,box_4[1]=0;
				box_5[0]=0,box_5[1]=0;
			}
		}
		else if(Island_state == 3)
		{
			imu_angle_data.gyro.z=0;
			Fit_angle= 0;
			Main_Switch= AIM_Mode;
			Island_state= 4;
		}
		else if(Island_state == 4)
		{
			if(target_existence=='T' &&  abs(coordinate_sub.y)<3 && abs(coordinate_sub.x)<3)
			{
				Main_Switch= Stop_Mode;
				if(num_card == 0)
				{
					num_card++;
					box_1[0] = class_of_target;
					box_1[1]++;
					arm_pick_to_first();
					rt_thread_mdelay(2000);
					Island_state= 5;
				}
				else if(num_card == 1)
				{
					num_card++;
					if(class_of_target == box_1[0])
					{
						box_1[1]++;
						arm_pick_to_first();
						rt_thread_mdelay(2000);				
						Island_state= 5;
					}
					else 
					{
						box_2[0] = class_of_target;
						box_2[1]++;
						arm_pick_to_second();
						rt_thread_mdelay(2000);
						Island_state= 5;
					}
				}
				else if(num_card == 2)
				{
					num_card++;
					if(class_of_target == box_1[0])
					{
						box_1[1]++;
						arm_pick_to_first();
						rt_thread_mdelay(2000);
						Island_state= 5;
					}
					else if(class_of_target == box_2[0])
					{
						box_2[1]++;
						arm_pick_to_second();
						rt_thread_mdelay(2000);
						Island_state= 5;
					}
					else
					{
						box_3[0] = class_of_target;
						box_3[1]++;
						arm_pick_to_third();
						rt_thread_mdelay(3000);
						
						Island_state= 5;
					}
				}
				else if(num_card == 3)
				{
					num_card++;
					if(class_of_target == box_1[0])
					{
						box_1[1]++;
						arm_pick_to_first();
						rt_thread_mdelay(2000);
						Island_state= 5;
					}
					else if(class_of_target == box_2[0])
					{
						box_2[1]++;
						arm_pick_to_second();
						rt_thread_mdelay(2000);
						Island_state= 5;
					}
					else if(class_of_target == box_3[0])
					{
						box_3[1]++;
						arm_pick_to_third();
						rt_thread_mdelay(3000);
						Island_state= 5;
					}	
					else{
						box_4[0] = class_of_target;
						box_4[1]++;
						arm_pick_to_forth();
						rt_thread_mdelay(4000);
						
						Island_state= 5;
					}
				}
				else if(num_card == 4)
				{
					num_card++;
					if(class_of_target == box_1[0])
					{
						box_1[1]++;
						arm_pick_to_first();
						rt_thread_mdelay(2000);
						Island_state= 5;
					}
					else if(class_of_target == box_2[0])
					{
						box_2[1]++;
						arm_pick_to_second();
						rt_thread_mdelay(2000);
						Island_state= 5;
					}
						
					else if(class_of_target == box_3[0])
					{
						box_3[1]++;
						arm_pick_to_third();
						rt_thread_mdelay(3000);
						Island_state= 5;
					}	
					else if(class_of_target == box_4[0])
					{
						box_4[1]++;
						arm_pick_to_forth();
						rt_thread_mdelay(4000);
						Island_state= 5;
					}	
					else{
						box_5[0] = class_of_target;
						box_5[1]++;
						arm_pick_to_five();
						rt_thread_mdelay(4000);
						
						Island_state= 5;
					}
				}
				else if(num_card == 5)
				{
					num_card++;
					if(class_of_target == box_1[0])
					{
						box_1[1]++;
						arm_pick_to_first();
						rt_thread_mdelay(2000);
						Island_state= 5;
					}
					else if(class_of_target == box_2[0])
					{
						box_2[1]++;
						arm_pick_to_second();
						rt_thread_mdelay(2000);
						Island_state= 5;
					}
						
					else if(class_of_target == box_3[0])
					{
						box_3[1]++;
						arm_pick_to_third();
						rt_thread_mdelay(3000);
						Island_state= 5;
					}	
					else if(class_of_target == box_4[0])
					{
						box_4[1]++;
						arm_pick_to_forth();
						rt_thread_mdelay(4000);
						Island_state= 5;
					}	
					else{
						box_5[0] = class_of_target;
						box_5[1]++;
						arm_pick_to_five();
						rt_thread_mdelay(4000);
						Island_state= 5;
					}
				}
				else 
				{
					Island_state = 6;
				}
			}
		}
		else if(Island_state == 5)
		{
			if(target_existence == 'T' && abs(coordinate_sub.y)< 80 && abs(coordinate_sub.x)< 80){
				Main_Switch= AIM_Mode;		
				Island_state= 4;
			}
			else if(target_existence == 'F'  ){
				Island_state = 6;
				alpha=1;
				beta=0;
			}		
		}
		else if(Island_state == 6)
		{	
			imu_angle_data.gyro.z=0;
			Fit_angle= -180;
			Main_Switch= Angle_Mode;
			Island_state = 7;
		}
		else if(Island_state == 7)
		{
			if(abs(abs(Odom.angle)- abs(Fit_angle))< 3){
				Odom_init();
				if(target_existence == 'T')
				{
					Main_Switch= AIM_Mode;
					Island_state= 8;
				}
			}
		}
		else if(Island_state == 8)
		{
			if(target_existence == 'T' && abs(coordinate_sub.y)< 4 && abs(coordinate_sub.x)< 4){
				Main_Switch= Stop_Mode;
				rt_thread_mdelay(2000);
				if(class_of_target == box_1[0])
				{
					Odom_init();
					while(box_1[1]>0)
					{	
						num_card--;
						box_1[1]--;
						arm_pick_from_first();
						rt_thread_mdelay(1000);
					
					}
					Island_state= 9;
				}
				else if(class_of_target == box_2[0])
				{
					Odom_init();
					while(box_2[1]>0)
					{
						num_card--;
						box_2[1]--;
						arm_pick_from_second();
						rt_thread_mdelay(1000);
						
					}
					Island_state= 9;
				}
				else if(class_of_target == box_3[0])
				{
					Odom_init();
					while(box_3[1]>0)
					{
						num_card--;
						box_3[1]--;
						arm_pick_from_third();
						rt_thread_mdelay(1000);
						
					}
					Island_state= 9;
				}
				else if(class_of_target == box_4[0])
				{
					Odom_init();
					while(box_4[1]>0)
					{
						num_card--;
						box_4[1]--;
						arm_pick_from_forth();
						rt_thread_mdelay(2000);
						
					}
					Island_state= 9;					
				}
				else if(class_of_target == box_5[0])
				{
					Odom_init();
					while(box_5[1]>0)
					{
						num_card--;
						box_5[1]--;
						arm_pick_from_fifth();
						rt_thread_mdelay(2000);
						
					}
					Island_state= 9;
				}
				else
				{
					Odom_init();
					Island_state = 9;
				}
			}
		}
		else if(Island_state == 9)
		{
			Free_speedx=-2;
			Free_speedy=-20;
			Main_Switch = Free_Mode;
			if(abs(Odom.milesY)>50)
			{
				Main_Switch= Stop_Mode;
				Island_state= 10;
			}
		}
		else if(Island_state == 10)
		{
			imu_angle_data.gyro.z=0;
			Fit_angle= 90;
			Main_Switch = Angle_Mode;
			Island_state= 11;	
		}
		else if(Island_state == 11)
		{
			if(abs(abs(Odom.angle)- abs(Fit_angle))< 3){
				Free_speedx=0;
				Free_speedy=20;
				Odom_init();
				imu_angle_data.gyro.z=0;
				Fit_angle=0;
				Main_Switch = Free_Mode;
				Island_state= 12;
			}
		}
		else if(Island_state == 12)
		{
			if(abs(Odom.milesY)>20)
			{
				if(target_existence == 'T'){
				Odom_init();
				Fit_angle = Odom.angle;
				Main_Switch = AIM_Mode;
				Island_state = 13;
				}
			}
		}
		else if(Island_state == 13)
		{
			if(target_existence == 'T' && abs(coordinate_sub.x)< 4 && abs(coordinate_sub.y)< 4){
				//mile_record=abs(Odom.milesY);
				Odom_init();
				Free_speedx=10;
				Free_speedy=-15;
				Main_Switch = Free_Mode;
				Island_state=14;
				
			}
		}
		else if(Island_state == 14)
		{
			if(abs(Odom.milesY)>7)
			{
				Main_Switch= Stop_Mode;
				Odom_init();
				rt_thread_mdelay(2000);
				if(class_of_target == box_1[0])
					{					
						while(box_1[1]>0)
						{
							num_card--;
							box_1[1]--;
							arm_pick_from_first();
							rt_thread_mdelay(1000);
							
						}
						Island_state= 15;
					}
					else if(class_of_target == box_2[0])
					{
						while(box_2[1]>0)
						{
							num_card--;
							box_2[1]--;
							arm_pick_from_second();
							rt_thread_mdelay(1000);
							
						}
						Island_state= 15;
					}
					else if(class_of_target == box_3[0])
					{
						while(box_3[1]>0)
						{
							num_card--;
							box_3[1]--;
							arm_pick_from_third();
							rt_thread_mdelay(1000);
							
						}
						Island_state= 15;
					}
					else if(class_of_target == box_4[0])
					{
						while(box_4[1]>0)
						{
							num_card--;
							box_4[1]--;
							arm_pick_from_forth();
							rt_thread_mdelay(2000);	
						}
						Island_state= 15;					
					}
					else if(class_of_target == box_5[0])
					{
						while(box_5[1]>0)
						{
							num_card--;
							box_5[1]--;
							arm_pick_from_fifth();
							rt_thread_mdelay(2000);
							
						}
						Island_state= 15;
					}
					else
					{
						Island_state = 15;
					}
			}
		
		}
		else if(Island_state == 15)
		{
			Free_speedx=-8;
			Free_speedy=-25;
			Main_Switch = Free_Mode;
			if(abs(Odom.milesY)>30)
			{
				transerve=-25;
				Main_Switch= Sideward_Mode;
				Island_state = 16;
			}
		}
		else if(Island_state == 16)
		{
			if(target_existence == 'T' && abs(coordinate_sub.x)< 40 && abs(Odom.angle)>abs(Fit_angle)+30){
				Odom_init();
				Fit_angle = Odom.angle;
				Main_Switch = AIM_Mode;
				Island_state = 13;
			}
			else if(Odom.angle>170)
			{
				if(R_point_flag && Right_point>30)
				{
					Fit_angle = Odom.angle;
					Main_Switch = Patrol_Mode;
					Odom_init();
					Island_state = 17;
				}
				else if(abs(Odom.angle)>180)
				{
					Odom_init();
					Free_speedx=-10;
					Free_speedy=0;
					Fit_angle = Odom.angle;
					Main_Switch = Free_Mode;
					Island_state = 18;
				}
			}
		}
		else if(Island_state == 17)
		{
			if(abs(Odom.milesY)>80)
			{
				regular_card_state=0;
				Right_Island_Flag=0;
				Island_state=0;
				Island_Flag=0;
				num_card=0;
				alpha=0;
				beta=0;
				box_1[0]=0,box_1[1]=0;
				box_2[0]=0,box_2[1]=0;
				box_3[0]=0,box_3[1]=0;
				box_4[0]=0,box_4[1]=0;
				box_5[0]=0,box_5[1]=0;
			}
		}
		else if(Island_state == 18)
		{
			if(abs(Odom.milesX)>15)
			{
				Main_Switch = Patrol_Mode;
				Odom_init();
				Island_state = 17;
			}
		}
	}
}
//Cross_card
void Process_Cross_card()
{	
	int cnt=0;
	if(Cross_state==0)
	{
		for(int i=Longest_White_Column_Right[1];i<120;i++)
		{
			if(White_Column[i]>White_Column[i+1]) cnt++;
			if(cnt>25)
			{
				Left_Cross_Flag=1;
				cnt=0;
				break;
			}
			if(i==119)
			{
				cnt=0;
				break;
			}
		}
		for(int i=Longest_White_Column_Right[1];i>0;i--)
		{
			if(White_Column[i]>White_Column[i-1]) cnt++;
			if(cnt>25)
			{
				Right_Cross_Flag=1;
				cnt=0;
				break;
			}
			if(i==1)
			{
				cnt=0;
				break;
			}
		}	
		Cross_Flag=1;
		Main_Switch=Patrol_Mode;
		imu_angle_data.gyro.z=0;
		if(Left_Cross_Flag && !Right_Cross_Flag)
		{
			Cross_state=1;
		}
		else if(!Left_Cross_Flag && Right_Cross_Flag)
		{
			Cross_state=1;
		}
				
	}
	if(Left_Cross_Flag)
	{
		if(Cross_state==1)
		{
			Fit_angle=90;
			Main_Switch=Angle_Mode;
			Cross_state=2;
		}
		else if(Cross_state==2)
		{
			if(abs(abs(Odom.angle)-abs(Fit_angle))<2)
			{
				Odom_init();
				Free_speedx=10;Free_speedy=0;
				Main_Switch=Free_Mode;
				Cross_state=3;
			}
		}
		else if(Cross_state==3)
		{
			if(10<abs(Odom.milesX) && abs(Odom.milesX)<20 && target_existence == 'T')
			{
				if(abs(coordinate_sub.y) < 100 && abs(coordinate_sub.y) < 100)
				{
					Main_Switch=AIM_Mode;
					Cross_state=4;
				}
			}
			else if(abs(Odom.milesX)>20 && target_existence=='F')
			{
				imu_angle_data.gyro.z=0;
				Fit_angle=-90;
				Main_Switch=Angle_Mode;
				Odom_init();
				Cross_state=15;
			}
		}
		else if(Cross_state==15)
		{
			if(abs(abs(Odom.angle)-abs(Fit_angle))<3 && Main_Switch == Angle_Mode)
			{
					Main_Switch=Patrol_Mode;
			}
			else if(Main_Switch==Patrol_Mode && abs(Odom.milesY)>250)
			{
				Cross_state=0;
				Cross_Flag=0;
				Left_Cross_Flag=0;
				num_card=0;
				alpha=0;
				beta=0;
				box_1[0]=0,box_1[1]=0;
				box_2[0]=0,box_2[1]=0;
				box_3[0]=0,box_3[1]=0;
				box_4[0]=0,box_4[1]=0;
				box_5[0]=0,box_5[1]=0;	
			}
		}
		else if(Cross_state==4)
		{
			if(target_existence=='T' && abs(coordinate_sub.y)<3 && abs(coordinate_sub.x)<3)
			{
				Main_Switch= Stop_Mode;
				if(num_card == 0)
				{
					box_1[0] = class_of_target;
					box_1[1]++;
					num_card++;
					arm_pick_to_first();
					rt_thread_mdelay(2000);
					Cross_state= 5;
				}
				else if(num_card == 1)
				{
					num_card++;
					if(class_of_target == box_1[0])
					{
						box_1[1]++;
						arm_pick_to_first();
						rt_thread_mdelay(2000);		
						Cross_state= 5;
					}
					else 
					{
						box_2[0] = class_of_target;
						box_2[1]++;
						arm_pick_to_second();
						rt_thread_mdelay(2000);
						Cross_state= 5;
					}
				}
				else if(num_card == 2)
				{
					num_card++;
					if(class_of_target == box_1[0])
					{		
						box_1[1]++;
						arm_pick_to_first();
						rt_thread_mdelay(2000);
						Cross_state= 5;
					}
					else if(class_of_target == box_2[0])
					{				
						box_2[1]++;
						arm_pick_to_second();
						rt_thread_mdelay(2000);
						Cross_state= 5;
					}
					else
					{
						box_3[0] = class_of_target;
						box_3[1]++;
						arm_pick_to_third();				
						rt_thread_mdelay(3000);
						Cross_state= 5;
					}
				}
				else if(num_card == 3)
				{
					num_card++;
					if(class_of_target == box_1[0])
					{	
						box_1[1]++;		
						arm_pick_to_first();
						
						rt_thread_mdelay(2000);
						Cross_state= 5;
					}
					else if(class_of_target == box_2[0])
					{	
						box_2[1]++;	
						arm_pick_to_second();
										
						rt_thread_mdelay(2000);
						Cross_state= 5;
					}
					else if(class_of_target == box_3[0])
					{			
						box_3[1]++;		
						arm_pick_to_third();
						
						rt_thread_mdelay(3000);
						Cross_state= 5;
					}	
					else{
						box_4[1]++;
						box_4[0] = class_of_target;
						arm_pick_to_forth();
						
						rt_thread_mdelay(4000);
						Cross_state= 5;
					}
				}
				else if(num_card == 4)
				{
					num_card++;
					if(class_of_target == box_1[0])
					{
						
						box_1[1]++;
						arm_pick_to_first();
						rt_thread_mdelay(2000);
						Cross_state= 5;
					}
					else if(class_of_target == box_2[0])
					{
						
						box_2[1]++;
						arm_pick_to_second();
						rt_thread_mdelay(2000);
						Cross_state= 5;
					}
						
					else if(class_of_target == box_3[0])
					{					
						
						box_3[1]++;
						arm_pick_to_third();
						rt_thread_mdelay(3000);
						Cross_state= 5;
					}	
					else if(class_of_target == box_4[0])
					{			
						box_4[1]++;		
						arm_pick_to_forth();
						
						rt_thread_mdelay(4000);
						Cross_state= 5;
					}	
					else{
						box_5[0] = class_of_target;
						box_5[1]++;
						arm_pick_to_five();
						rt_thread_mdelay(4000);
						Cross_state= 5;
					}
				}
				else if(num_card == 5)
				{
					num_card++;
					if(class_of_target == box_1[0])
					{
						box_1[1]++;	
						arm_pick_to_first();										
						rt_thread_mdelay(2000);
						Cross_state= 5;
					}
					else if(class_of_target == box_2[0])
					{
						box_2[1]++;	
						arm_pick_to_second();										
						rt_thread_mdelay(2000);
						Cross_state= 5;
					}
						
					else if(class_of_target == box_3[0])
					{
						box_3[1]++;
						arm_pick_to_third();
						rt_thread_mdelay(2000);
						Cross_state= 5;
					}	
					else if(class_of_target == box_4[0])
					{
						box_4[1]++;
						arm_pick_to_forth();
						rt_thread_mdelay(4000);
						Cross_state= 5;
					}	
					else{
						box_5[0] = class_of_target;
						box_5[1]++;
						arm_pick_to_five();
						rt_thread_mdelay(4000);				
						Cross_state= 5;
					}
				}
				else 
				{
					Cross_state=6;
				}
			}
		}
		else if(Cross_state==5)
		{
			if(target_existence == 'T' && abs(coordinate_sub.y)< 60 && abs(coordinate_sub.x)< 60){
				Main_Switch= AIM_Mode;		
				Cross_state= 4;
			}
			else if( target_existence == 'F' ){//target_existence == 'F' 
				Cross_state = 6;
				alpha=1;
				beta=0;
			}		
		}
		else if(Cross_state==6)
		{
			imu_angle_data.gyro.z=0;
			Fit_angle=-180;
			Main_Switch=Angle_Mode;
			Cross_state=7;
		}
		else if(Cross_state==7)
		{
			if(abs(abs(Odom.angle)-abs(Fit_angle))<3)
			{
				Odom_init();
				Free_speedx=-5;
				Free_speedy=0;
				Main_Switch = Free_Mode;
				Cross_state=8;
			}
		}
		else if(Cross_state==8)
		{
			if(abs(Odom.milesX)>5)
			{
				imu_angle_data.gyro.z=0;
				Odom_init();
				transerve=-25;
				Main_Switch=Sideward_Mode;
				Cross_state=9;
			}
		}
		else if(Cross_state==9)
		{
			if(target_existence=='T' && abs(coordinate_sub.x)< 40)
			{
				Odom_init();
				Fit_angle= Odom.angle;
				Main_Switch=AIM_Mode;
				Cross_state=10;
			}
		}
		else if(Cross_state==10)
		{
			if(target_existence == 'T' && abs(coordinate_sub.x)< 4 && abs(coordinate_sub.y)< 4){
				Odom_init();
				Free_speedx=10;
				Free_speedy=-15;
				Main_Switch = Free_Mode;
				Cross_state=11;
			}
		}
		else if(Cross_state==11)
		{
			if(abs(Odom.milesY)>5)
			{
				Main_Switch = Stop_Mode;
				rt_thread_mdelay(2000);
				if(class_of_target == box_1[0])
				{
					Odom_init();
					while(box_1[1]>0)
					{
						num_card--;
						box_1[1]--;
						arm_pick_from_first();
						rt_thread_mdelay(1000);
						
					}
					Cross_state=12;
				}
				else if(class_of_target == box_2[0])
				{
					Odom_init();
					while(box_2[1]>0)
					{
						num_card--;
						box_2[1]--;
						arm_pick_from_second();
						rt_thread_mdelay(1000);
						
					}
					Cross_state=12;
				}
				else if(class_of_target == box_3[0])
				{
					Odom_init();
					while(box_3[1]>0)
					{
						num_card--;
						box_3[1]--;
						arm_pick_from_third();
						rt_thread_mdelay(1000);
						
					}
					Cross_state=12;
				}
				else if(class_of_target == box_4[0])
				{
					Odom_init();
					while(box_4[1]>0)
					{
						num_card--;
						box_4[1]--;
						arm_pick_from_forth();
						rt_thread_mdelay(2000);
						
					}
					Cross_state=12;				
				}
				else if(class_of_target == box_5[0])
				{
					Odom_init();
					while(box_5[1]>0)
					{
						num_card--;
						box_5[1]--;
						arm_pick_from_fifth();
						rt_thread_mdelay(2000);
						
					}
					Cross_state=12;
				}
				else
				{
					Odom_init();
					Cross_state=12;
				}
			}
		}
		else if(Cross_state==12)
		{
			Free_speedx=0;
			Free_speedy=-25;
			Main_Switch = Free_Mode;
			if(abs(Odom.milesY)>40)
			{
				Main_Switch= Sideward_Mode;
				Cross_state= 13;
			}
		}
		else if(Cross_state==13)
		{
			if(target_existence=='T' && abs(coordinate_sub.x)< 40 && abs(Odom.angle)>abs(Fit_angle)+30)
			{
				Odom_init();
				Fit_angle= Odom.angle;
				Main_Switch=AIM_Mode;
				Cross_state=9;
			}
			else if(abs(Odom.angle)>265)
			{
//				if(R_point_flag && Right_point>30)
//				{
//					Free_speedy=10,Free_speedx=0;
//					Main_Switch = Free_Mode;
//					Odom_init();
//					Cross_state = 13;
//				}
				    Free_speedy=10,Free_speedx=0;
					Main_Switch = Free_Mode;
					Odom_init();
					Cross_state = 14;
			}
		}
		else if(Cross_state==14)
		{
			if(abs(Odom.milesY)>8 && Main_Switch == Free_Mode)
			{
				imu_angle_data.gyro.z=0;
				Fit_angle=105;
				Main_Switch = Angle_Mode;
			}
			if(Main_Switch == Angle_Mode)
			{
				if(abs(abs(Odom.angle)-abs(Fit_angle))<2)
				{
					Odom_init();
					Main_Switch = Patrol_Mode;
					Cross_state=15;
				}
			
			}
		}
		else if(Cross_state==15)
		{
			if(Odom.milesY>40)
			{
				regular_card_state=0;
				Cross_state=0;
				Left_Cross_Flag=0;
				Cross_Flag=0;
				alpha=0;
				beta=0;
			}
		}
	}
	if(Right_Cross_Flag)
	{
		if(Cross_state==1)
		{
			Fit_angle=-90;
			Main_Switch=Angle_Mode;
			Cross_state=2;
		}
		else if(Cross_state==2)
		{
			if(abs(abs(Odom.angle)-abs(Fit_angle))<2)
			{
				Odom_init();
				Free_speedx=-10;Free_speedy=0;
				Main_Switch=Free_Mode;
				Cross_state=3;
			}
		}
		else if(Cross_state==3)
		{
			if(10<abs(Odom.milesX) && abs(Odom.milesX)<20 && target_existence == 'T')
			{
				if(abs(coordinate_sub.y) < 100 && abs(coordinate_sub.y) < 100)
				{
					Main_Switch=AIM_Mode;
					Cross_state=4;
				}
			}
			else if(abs(Odom.milesX)>20 && target_existence=='F')
			{
				imu_angle_data.gyro.z=0;
				Fit_angle=90;
				Main_Switch=Angle_Mode;
				Odom_init();
				Cross_state=15;
			}
		}
		else if(Cross_state==15)
		{
			if(abs(abs(Odom.angle)-abs(Fit_angle))<3 && Main_Switch == Angle_Mode)
			{
					Main_Switch=Patrol_Mode;
			}
			else if(Main_Switch==Patrol_Mode && abs(Odom.milesY)>250)
			{
				Cross_state=0;
				Cross_Flag=0;
				Left_Cross_Flag=0;
				num_card=0;
				alpha=0;
				beta=0;
				box_1[0]=0,box_1[1]=0;
				box_2[0]=0,box_2[1]=0;
				box_3[0]=0,box_3[1]=0;
				box_4[0]=0,box_4[1]=0;
				box_5[0]=0,box_5[1]=0;	
			}
		}
		else if(Cross_state==4)
		{
			if(target_existence=='T' && abs(coordinate_sub.y)<3 && abs(coordinate_sub.x)<3)
			{
				Main_Switch= Stop_Mode;
				if(num_card == 0)
				{
					num_card++;
					box_1[0] = class_of_target;
					box_1[1]++;
					arm_pick_to_first();
					rt_thread_mdelay(4000);
					Cross_state= 5;
				}
				else if(num_card == 1)
				{
					num_card++;
					if(class_of_target == box_1[0])
					{	
						box_1[1]++;
						arm_pick_to_first();
						rt_thread_mdelay(4000);
					
						Cross_state= 5;
					}
					else 
					{
						box_2[0] = class_of_target;
						box_2[1]++;
						arm_pick_to_second();
						rt_thread_mdelay(4000);
						Cross_state= 5;
					}
				}
				else if(num_card == 2)
				{
					num_card++;
					if(class_of_target == box_1[0])
					{
						box_1[1]++;
						arm_pick_to_first();
						rt_thread_mdelay(4000);
						Cross_state= 5;
					}
					else if(class_of_target == box_2[0])
					{
						box_2[1]++;
						arm_pick_to_second();
						rt_thread_mdelay(4000);
						Cross_state= 5;
					}
					else
					{
						box_3[0] = class_of_target;
						box_3[1]++;
						arm_pick_to_third();
						rt_thread_mdelay(4000);
						
						Cross_state= 5;
					}
				}
				else if(num_card == 3)
				{
					num_card++;
					if(class_of_target == box_1[0])
					{
						box_1[1]++;
						arm_pick_to_first();
						rt_thread_mdelay(4000);
						Cross_state= 5;
					}
					else if(class_of_target == box_2[0])
					{
						box_2[1]++;
						arm_pick_to_second();
						rt_thread_mdelay(4000);
						Cross_state= 5;
					}
					else if(class_of_target == box_3[0])
					{
						box_3[1]++;
						arm_pick_to_third();
						rt_thread_mdelay(4000);
						Cross_state= 5;
					}	
					else{
						box_4[0] = class_of_target;
						box_4[1]++;
						arm_pick_to_forth();
						rt_thread_mdelay(5000);
						
						Cross_state= 5;
					}
				}
				else if(num_card == 4)
				{
					num_card++;
					if(class_of_target == box_1[0])
					{
						box_1[1]++;
						arm_pick_to_first();
						rt_thread_mdelay(4000);
						Cross_state= 5;
					}
					else if(class_of_target == box_2[0])
					{
						box_2[1]++;
						arm_pick_to_second();
						rt_thread_mdelay(4000);
						Cross_state= 5;
					}
						
					else if(class_of_target == box_3[0])
					{
						box_3[1]++;
						arm_pick_to_third();
						rt_thread_mdelay(4000);
						Cross_state= 5;
					}	
					else if(class_of_target == box_4[0])
					{
						box_4[1]++;
						arm_pick_to_forth();
						rt_thread_mdelay(5000);
						Cross_state= 5;
					}	
					else{
						box_5[0] = class_of_target;
						box_5[1]++;
						arm_pick_to_five();
						rt_thread_mdelay(5000);
						
						Cross_state= 5;
					}
				}
				else if(num_card == 5)
				{
					num_card++;
					if(class_of_target == box_1[0])
					{
						box_1[1]++;
						arm_pick_to_first();
						rt_thread_mdelay(4000);
						Cross_state= 5;
					}
					else if(class_of_target == box_2[0])
					{
						box_2[1]++;
						arm_pick_to_second();
						rt_thread_mdelay(4000);
						Cross_state= 5;
					}
						
					else if(class_of_target == box_3[0])
					{
						box_3[1]++;
						arm_pick_to_third();
						rt_thread_mdelay(4000);
						Cross_state= 5;
					}	
					else if(class_of_target == box_4[0])
					{
						box_4[1]++;
						arm_pick_to_forth();
						rt_thread_mdelay(5000);
						Cross_state= 5;
					}	
					else{
						box_5[0] = class_of_target;
						box_5[1]++;
						arm_pick_to_five();
						rt_thread_mdelay(5000);
						
						Cross_state= 5;
					}
				}
				else 
				{
					Cross_state=6;
				}
			}
		}
		else if(Cross_state==5)
		{
			if(target_existence == 'T' && abs(coordinate_sub.y)< 60 && abs(coordinate_sub.x)< 60){
				Main_Switch= AIM_Mode;									
				Cross_state= 4;
			}
			else if( target_existence == 'F' ){//target_existence == 'F' 
				Cross_state = 6;
				alpha=1;
				beta=0;
			}		
		}
		else if(Cross_state==6)
		{
			imu_angle_data.gyro.z=0;
			Fit_angle=180;
			Main_Switch=Angle_Mode;
			Cross_state=7;
		}
		else if(Cross_state==7)
		{
			if(abs(abs(Odom.angle)-abs(Fit_angle))<3)
			{
				Odom_init();
				Free_speedx=5;
				Free_speedy=0;
				Main_Switch = Free_Mode;
				Cross_state=8;
			}
		}
		else if(Cross_state==8)
		{
			if(abs(Odom.milesX)>5)
			{
				imu_angle_data.gyro.z=0;
				Odom_init();
				transerve=25;
				Main_Switch=Sideward_Mode;
				Cross_state=9;
			}
		}
		else if(Cross_state==9)
		{
			if(target_existence=='T' && abs(coordinate_sub.x)< 40)
			{
				Odom_init();
				Fit_angle= Odom.angle;
				Main_Switch=AIM_Mode;
				Cross_state=10;
			}
		}
		else if(Cross_state==10)
		{
			if(target_existence == 'T' && abs(coordinate_sub.x)< 4 && abs(coordinate_sub.y)< 4){
				Odom_init();
				Free_speedx=10;
				Free_speedy=-15;
				Main_Switch = Free_Mode;
				Cross_state=11;
			}
		}
		else if(Cross_state==11)
		{
			if(abs(Odom.milesY)>5)
			{
				Main_Switch = Stop_Mode;
				rt_thread_mdelay(2000);
				if(class_of_target == box_1[0])
				{
					Odom_init();
					while(box_1[1]>0)
					{
						num_card--;
						box_1[1]--;
						arm_pick_from_first();
						rt_thread_mdelay(2000);
						
					}
					Cross_state=12;
				}
				else if(class_of_target == box_2[0])
				{
					Odom_init();
					while(box_2[1]>0)
					{
						num_card--;
						box_2[1]--;
						arm_pick_from_second();
						rt_thread_mdelay(2000);
						
					}
					Cross_state=12;
				}
				else if(class_of_target == box_3[0])
				{
					Odom_init();
					while(box_3[1]>0)
					{
						num_card--;
						box_3[1]--;
						arm_pick_from_third();
						rt_thread_mdelay(2000);
						
					}
					Cross_state=12;
				}
				else if(class_of_target == box_4[0])
				{
					Odom_init();
					while(box_4[1]>0)
					{
						num_card--;
						box_4[1]--;
						arm_pick_from_forth();
						rt_thread_mdelay(5000);
						
					}
					Cross_state=12;				
				}
				else if(class_of_target == box_5[0])
				{
					Odom_init();
					while(box_5[1]>0)
					{
						num_card--;
						box_5[1]--;
						arm_pick_from_fifth();
						rt_thread_mdelay(5000);
						
					}
					Cross_state=12;
				}
				else
				{
					Odom_init();
					Cross_state=12;
				}
			}
		}
		else if(Cross_state==12)
		{
			Free_speedx=0;
			Free_speedy=-25;
			Main_Switch = Free_Mode;
			if(abs(Odom.milesY)>35)
			{
				Main_Switch= Sideward_Mode;
				Cross_state= 13;
			}
		}
		else if(Cross_state==13)
		{
			if(target_existence=='T' && abs(coordinate_sub.x)< 40 && abs(Odom.angle)>abs(Fit_angle)+30)
			{
				Odom_init();
				Fit_angle= Odom.angle;
				Main_Switch=AIM_Mode;
				Cross_state=9;
			}
			else if(abs(Odom.angle)>250)
			{
				if(L_point_flag && Left_point<90)
				{
					Free_speedy=10,Free_speedx=0;
					Main_Switch = Free_Mode;
					Odom_init();
					Cross_state = 14;
				}
				else if(abs(Odom.angle)>265)
				{
					Free_speedy=10,Free_speedx=0;
					Main_Switch = Free_Mode;
					Odom_init();
					Cross_state = 14;
				}
			}
		}
		else if(Cross_state==14)
		{
			if(abs(Odom.milesY)>8 && Main_Switch == Free_Mode)
			{
				imu_angle_data.gyro.z=0;
				Fit_angle=-105;
				Main_Switch = Angle_Mode;
			}
			if(Main_Switch == Angle_Mode)
			{
				if(abs(abs(Odom.angle)-abs(Fit_angle))<2)
				{
					Odom_init();
					Main_Switch = Patrol_Mode;
					Cross_state=15;
				}
			
			}
		}
		else if(Cross_state==15)
		{
			if(Odom.milesY>40)
			{
				regular_card_state=0;
				Cross_state=0;
				Right_Cross_Flag=0;
				Cross_Flag=0;
				alpha=0;
				beta=0;
			}
		}
	}
}
//Right_zebra
void Process_Zebra_card()
{
	static uint8 gpio_flag;
	alpha=0;
	beta=1;
	if(zebra_state==0)
	{
		imu_angle_data.gyro.z=0;
		Fit_angle=-90;
		Main_Switch=Angle_Mode;
		zebra_state=1;
	}
	else if(zebra_state==1)
	{
		if(abs(abs(Odom.angle)-abs(Fit_angle))<3)
		{
			Odom_init();
            Free_speedx=0;Free_speedy=-25;
			Main_Switch=Free_Mode;
			zebra_state=2;
		}
	}
	else if(zebra_state==2)
	{
		if(abs(Odom.milesY)>20)
		{
			Odom_init();
			zebra_state=3;
		}
	}
	else if(zebra_state==3)
	{
		transerve=25;
		Main_Switch=Sideward_Mode;
		if(target_existence=='T' && abs(coordinate_sub.x)<35)
		{
			traveled_mileX=Odom.milesX;
			Odom.milesY=0;
			Free_speedx=0,Free_speedy=15;
			Main_Switch=Free_Mode;
			zebra_state=5;
		}
	}
	else if(zebra_state==4)
	{
		transerve=25;
		Main_Switch=Sideward_Mode;
		if(abs(Odom.milesX)>165)
		{
			Fit_angle=90;
			Main_Switch=Angle_Mode;
			zebra_state=12;
		}
		if(abs(Odom.milesX)-abs(traveled_mileX)>30)
		{
			if(target_existence=='T' && coordinate_sub.x<35 && coordinate_sub.x>-20)
			{
				traveled_mileX=Odom.milesX;
				Odom.milesY=0;
				Free_speedx=0,Free_speedy=25;
				Main_Switch=Free_Mode;
				zebra_state=5;
			}
		}
	}
	else if(zebra_state==5)
	{
		if(Odom.milesY>32)
		{
			Main_Switch=AIM_Mode;
			zebra_state=6;
		}
	
	}
	else if(zebra_state==6)
	{
		if(target_existence=='T' && abs(coordinate_sub.y)<3 && abs(coordinate_sub.x)<3)
		{
			Odom.milesY=0;		
			Free_speedx=0,Free_speedy=-15;
			imu_angle_data.gyro.z=0;
			Fit_angle=0;
			Main_Switch=Free_Mode;
			zebra_state=7;
		}
	}
	else if(zebra_state==7)
	{
		if(abs(Odom.milesY)>10)
		{
			Main_Switch=Stop_Mode;
			rt_thread_mdelay(1000);
			if(class_of_target == 49)
			{
				Fit_angle=55;
				Main_Switch=Angle_Mode;
				zebra_state=8;
			}
			else if(class_of_target == 50)
			{
				Fit_angle=-98;
				Main_Switch=Angle_Mode;
				zebra_state=8;
			}
			else if(class_of_target == 51)
			{
				Fit_angle=-152;
				Main_Switch=Angle_Mode;
				zebra_state=8;
			}
			else
			{
				zebra_state=6;
			}
		}
		
	}
	else if(zebra_state==8)
	{
		if(abs(abs(Odom.angle)-abs(Fit_angle))<3)
		{
			if(Fit_angle == 55)
			{
				rt_thread_mdelay(1000);
				gpio_init(first_elec, GPO, 0, GPIO_PIN_CONFIG);
				rt_thread_mdelay(2000);
				gpio_flag++;
				zebra_state=9;
			}
			else if(Fit_angle == -98)
			{
				rt_thread_mdelay(1000);
				gpio_init(second_elec, GPO, 0, GPIO_PIN_CONFIG);
				rt_thread_mdelay(4000);
				gpio_flag++;
				zebra_state=9;
			}
			else if(Fit_angle == -152)
			{
				rt_thread_mdelay(1000);
				gpio_init(third_elec, GPO, 0, GPIO_PIN_CONFIG);
				rt_thread_mdelay(2000);
				gpio_flag++;
				zebra_state=9;
			}	
		}
	}
	else if(zebra_state==9)
	{
		Fit_angle=0;
		Main_Switch=Angle_Mode;
		zebra_state=10;
	}
	else if(zebra_state==10)
	{
		if(abs(abs(Odom.angle)-abs(Fit_angle))<2)
		{
			Odom.milesY=0;
			Free_speedx=0,Free_speedy=-15;
			Main_Switch=Free_Mode;
			zebra_state=11;
		}
	}
	else if(zebra_state==11)
	{
		if(abs(Odom.milesY)>20)
		{
			if(gpio_flag == 3)
			{
				Fit_angle=90;
				Main_Switch=Angle_Mode;
				zebra_state=12;
			}
			else 
			{
				zebra_state=13;
			}
		}
		
	}
	else if(zebra_state==13)
	{
		Odom.milesX=traveled_mileX;
		Odom.milesY=0;
		zebra_state=4;
	}
	else if(zebra_state==12)
	{
		if(abs(abs(Odom.angle)-abs(Fit_angle))<3)
		{
			Main_Switch=Patrol_Mode;
			gpio_flag=0;
			zebra_state=14;
		}
	}
}
//Left_Zebra
//void Process_Zebra_card()
//{
//	static uint8 gpio_flag;
//	if(zebra_state==0)
//	{
//		imu_angle_data.gyro.z=0;
//		Fit_angle=90;
//		Main_Switch=Angle_Mode;
//		zebra_state=1;
//	}
//	else if(zebra_state==1)
//	{
//		if(abs(abs(Odom.angle)-abs(Fit_angle))<3)
//		{
//			Odom_init();
//            Free_speedx=-10;Free_speedy=-15;
//			Main_Switch=Free_Mode;
//			zebra_state=2;
//		}
//	}
//	else if(zebra_state==2)
//	{
//		if(abs(Odom.milesY)>20)
//		{
//			Odom_init();
//			zebra_state=3;
//		}
//	}
//	else if(zebra_state==3)
//	{
//		transerve=-25;
//		Main_Switch=Sideward_Mode;
//		if(abs(Odom.milesX)>25)
//		{
//			if(target_existence=='T' && abs(coordinate_sub.x)<30)
//			{
//				Odom_init();
//				Main_Switch=AIM_Mode;
//				zebra_state=4;
//			}
//		}
//	}
//	else if(zebra_state==4)
//	{
//		if(target_existence=='T' && abs(coordinate_sub.x)>180)
//		{
//			return;	
//		}
//		if(target_existence=='T' && abs(coordinate_sub.y)<3 && abs(coordinate_sub.x)<3)
//		{
//			Odom_init();		
//			imu_angle_data.gyro.z=0;
//			Free_speedx=7,Free_speedy=-15;
//			Main_Switch=Free_Mode;
//			zebra_state=5;
//		}
//	}
//	else if(zebra_state==5)
//	{
//		if(abs(Odom.milesY)>7)
//		{
//			Main_Switch=Stop_Mode;
//			if(class_of_target == 49)
//			{
//				Fit_angle=85;
//				Main_Switch=Angle_Mode;
//				zebra_state=6;
//			}
//			else if(class_of_target == 50)
//			{
//				Fit_angle=-85;
//				Main_Switch=Angle_Mode;
//				zebra_state=6;
//			}
//			else if(class_of_target == 51)
//			{
//				Fit_angle=-135;
//				Main_Switch=Angle_Mode;
//				zebra_state=6;
//			}
//		}
//	}
//	else if(zebra_state==6)
//	{
//		if(abs(abs(Odom.angle)-abs(Fit_angle))<3)
//		{
//			if(Fit_angle == 85)
//			{
//				gpio_init(first_elec, GPO, 0, GPIO_PIN_CONFIG);
//				rt_thread_mdelay(3000);
//				gpio_flag++;
//				zebra_state=7;
//			}
//			else if(Fit_angle == -85)
//			{
//				gpio_init(second_elec, GPO, 0, GPIO_PIN_CONFIG);
//				rt_thread_mdelay(3000);
//				gpio_flag++;
//				zebra_state=7;
//			}
//			else if(Fit_angle == -135)
//			{
//				gpio_init(third_elec, GPO, 0, GPIO_PIN_CONFIG);
//				rt_thread_mdelay(3000);
//				gpio_flag++;
//				zebra_state=7;
//			}	
//		}
//	}
//	else if(zebra_state==7)
//	{
//		Fit_angle=0;
//		Main_Switch=Angle_Mode;
//		zebra_state=8;
//	}
//	else if(zebra_state==8)
//	{
//		if(abs(abs(Odom.angle)-abs(Fit_angle))<2)
//		{
//			Odom_init();
//			Free_speedx=0,Free_speedy=-15;
//			Main_Switch=Free_Mode;
//			zebra_state=9;
//		}
//	}
//	else if(zebra_state==9)
//	{
//		if(abs(Odom.milesY)>28)
//		{
//			if(gpio_flag == 3)
//			{
//				Fit_angle=-85;
//				Main_Switch=Angle_Mode;
//				zebra_state=10;
//			}
//			else 
//			{
//				zebra_state=11;
//			}
//		}
//		
//	}
//	else if(zebra_state==11)
//	{
//		Odom_init();
//		zebra_state=3;
//	}
//	else if(zebra_state==10)
//	{
//		if(abs(abs(Odom.angle)-abs(Fit_angle))<3)
//		{
//			Main_Switch=Patrol_Mode;
//			gpio_flag=0;
//			zebra_state=12;
//		}
//	}
//}
void Set_Car_Movement()
{
	if(Island_Flag && !Cross_Flag && !Zebra_Flag)
	{
		Process_Island_card();
	}
	else if(Cross_Flag && !Island_Flag && !Zebra_Flag)
	{
		Process_Cross_card();
	}	
	else if(!Island_Flag && !Cross_Flag) 
	{
		if(Zebra_Flag==1)
		{
			Process_Zebra_card();
		}
		else if(Zebra_Flag==2)
		{
			Main_Switch=Patrol_Mode;
		}
		else 
		{
			Process_regular_card();
		}
	}	
}

void Set_Motor_Movement(void)
{
	float Car_SpeedX=0;
	float Car_SpeedY=0;
	float Car_SpeedZ=0;
	
	switch (Main_Switch)
	{
		case Stop_Mode:
		{
			Car_SpeedX=0;
			Car_SpeedY=0;
			Car_SpeedZ=0;
			break;
		}
		case Patrol_Mode:
		{
			Car_SpeedX=0.9*translation;
			Car_SpeedY=straight;
			Car_SpeedZ=1.3*spin;
			break;
		}
		case AIM_Mode:
		{
		    Car_SpeedX=card_xerror_pid(&AIMX_PID,coordinate_sub.x,0);
			Car_SpeedY=card_yerror_pid(&AIMY_PID,coordinate_sub.y,0);
			Car_SpeedZ=TurnPID(&Turn_PID,Odom.angle,Fit_angle);
			break;
		}
		case Forward_Switch:
		{
			Car_SpeedX=card_xdistance_pid(&XY_PID,coordinate_sub.x,0);
			Car_SpeedY=card_ydistance_pid(&XY_PID,coordinate_sub.y,0);
			Car_SpeedZ=TurnPID(&Z_PID, Odom.angle,Fit_angle );
			break;
		}
		case Backward_Switch:
		{
			Car_SpeedX=card_xerror_pid(&AIMX_PID,Odom.milesX,0);
			Car_SpeedY=card_yerror_pid(&AIMX_PID,Odom.milesY,0);
			Car_SpeedZ=0;//TurnPID(&Z_PID, Odom.angle,Fit_angle );
			break;
		}
		case Sideward_Mode:
		{
		    Car_SpeedX=transerve;
			Car_SpeedY=PositPID(&XY_PID,3.5*lr_sum,0);
			Car_SpeedZ=TurnPID(&Z_PID,9*lr_err,0);
			break;
		}
		case Angle_Mode:{
			Car_SpeedX=0;
			Car_SpeedY=0;
			Car_SpeedZ=TurnPID(&Turn_PID,Odom.angle,Fit_angle);
			break;
		}
		case Free_Mode:
		{
			Car_SpeedX=Free_speedx;//card_xdistance_pid(&XY_PID,Odom.milesX,Fit_milex);//Free_speedx;
			Car_SpeedY=Free_speedy;//card_ydistance_pid(&XY_PID,Odom.milesY,-60);//Free_speedy;
			Car_SpeedZ=TurnPID(&Turn_PID, Odom.angle,Fit_angle );
		}
		default:
			break;
	}
		short Car_MaxSpeedY= 800;//  300cm/s
		short Car_MaxSpeedX= 800*0.6;
		short Car_MaxSpeedZ= 800*0.6;
		short Car_MaxDuty= 6000;
		
		Car_SpeedY=LimitProtect(Car_MaxSpeedY,-Car_MaxSpeedY,Car_SpeedY);
		Car_SpeedX=LimitProtect(Car_MaxSpeedX,-Car_MaxSpeedX,Car_SpeedX);
		Car_SpeedZ=LimitProtect(Car_MaxSpeedZ,-Car_MaxSpeedZ,Car_SpeedZ);
	
		MotorLF.Fit_Speed = Car_SpeedY + Car_SpeedX - Car_SpeedZ;
		MotorRF.Fit_Speed = Car_SpeedY - Car_SpeedX + Car_SpeedZ;
		MotorLR.Fit_Speed = Car_SpeedY - Car_SpeedX - Car_SpeedZ;
		MotorRR.Fit_Speed = Car_SpeedY + Car_SpeedX + Car_SpeedZ;

		MotorLF.duty =IncrementPID(&M1_S_PID,MotorLF.Speed,MotorLF.Fit_Speed);
		MotorRF.duty =IncrementPID(&M2_S_PID,MotorRF.Speed,MotorRF.Fit_Speed);
		MotorLR.duty =IncrementPID(&M3_S_PID,MotorLR.Speed,MotorLR.Fit_Speed);
		MotorRR.duty =IncrementPID(&M4_S_PID,MotorRR.Speed,MotorRR.Fit_Speed);

		MotorLF.duty=LimitProtect(Car_MaxDuty,-Car_MaxDuty,MotorLF.duty);
		MotorRF.duty=LimitProtect(Car_MaxDuty,-Car_MaxDuty,MotorRF.duty);
		MotorLR.duty=LimitProtect(Car_MaxDuty,-Car_MaxDuty,MotorLR.duty);
		MotorRR.duty=LimitProtect(Car_MaxDuty,-Car_MaxDuty,MotorRR.duty);
	
		setPWM_F(MotorLF.duty, MotorLF_DIR, MotorLF_PWM);
		setPWM_F(MotorRF.duty, MotorRF_DIR, MotorRF_PWM);
		setPWM_R(MotorLR.duty, MotorLR_DIR, MotorLR_PWM);
		setPWM_R(MotorRR.duty, MotorRR_DIR, MotorRR_PWM);


}









