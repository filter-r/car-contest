#ifndef _CAR_H
#define _CAR_H


#include "zf_common_headfile.h"




typedef struct 
{
    float x,y,z;

}XYZ;
typedef struct{
    float milesX;
    float milesY;
    float speedX;
    float speedY;
    float speedZ;
    float angle;
}Odometry;
typedef struct{
    float Kp;
    float Ki;
    float Kd;
	float NowErr;
    float LastErr;
    float PastErr;   
    float LastOut;  
	float OutPut; 
    float SumError;  
    float SumErrorMax; 
}PID_Struct;

enum Switch
{
	Stop_Mode=0,
	Patrol_Mode=1,
	AIM_Mode=2,
    Angle_Mode=3,
	Forward_Switch=4,
	Backward_Switch=5,
	Sideward_Mode=6,
	Free_Mode=7
	
};
typedef struct{
    int Encoder; 
    float Speed;   
    float Fit_Speed;     
    short int duty;     
}MotorValue_Struct;



extern int zebra_state;
extern short int translation,straight,spin;
extern short int x_mistake,y_mistake;
extern short int lr_err,lr_sum,transerve;
extern char Main_Switch;
extern short int Right_card_flag;
extern short int Left_card_flag;
extern short int Fit_angle;
extern Odometry Odom;
extern int regular_card_state;
extern int num_card;
extern MotorValue_Struct MotorLF,MotorRF,MotorLR,MotorRR;
extern short int Free_speedx,Free_speedy;
extern int Island_state,Cross_state;
extern bool alpha;
extern bool beta;
extern PID_Struct 
	M1_S_PID,
	M2_S_PID,
	M3_S_PID,
	M4_S_PID,
    XY_PID,
    Z_PID,
	Turn_PID,
	AIMX_PID,
	AIMY_PID;

extern XYZ coordinate_sub;

void Judge_cardtype();
void PID_Init(PID_Struct *sptr,char n);
void PID_Init_All(void);
int LimitProtect( int max, int min, float data);
short PositPID(PID_Struct *User, float NowDat, float SetDat);
int IncrementPID(PID_Struct *User, float Nowdat, float Setdat);
int card_xerror_pid(PID_Struct *User, float Nowdat, float Setdat);
int card_yerror_pid(PID_Struct *User, float Nowdat, float Setdat);
int card_xdistance_pid(PID_Struct *User, float Nowdat, float Setdat);
int card_ydistance_pid(PID_Struct *User, float Nowdat, float Setdat);
int Angel_pid(PID_Struct *User, int NowAngel, int TargetAngel);
int TurnPID(PID_Struct *User, float Nowdat, float Setdat);
double my_abs(double x);
void MotorInit();
void setPWM_F(short duty, gpio_pin_enum DIR_pin, pwm_channel_enum PWM_ch) ;
void setPWM_R(short duty, gpio_pin_enum DIR_pin, pwm_channel_enum PWM_ch) ;
double my_abs(double x);
void Set_Motor_Movement(void);
void Set_Car_Movement();
void Process_Island_card();
void Process_Cross_card();
void Process_Zebra_card();
void Process_regular_card();
void encoder_init();
void encoder_get();
void encoder_to_miles();
void encoder_to_angle();
void Odom_init();
extern int Island_state;
extern int Cross_state;
extern int zebra_state;
extern int regular_card_state;
extern int traveled_mileX;

#endif