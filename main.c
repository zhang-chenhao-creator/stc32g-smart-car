/*---------------------------------------------------------------------*/
/* --- STC MCU Limited ------------------------------------------------*/
/* --- STC 1T Series MCU Demo Programme -------------------------------*/
/* --- Mobile: (86)13922805190 ----------------------------------------*/
/* --- Fax: 86-0513-55012956,55012947,55012969 ------------------------*/
/* --- Tel: 86-0513-55012928,55012929,55012966 ------------------------*/
/* --- Web: www.STCAI.com ---------------------------------------------*/
/* --- Web: www.STCMCUDATA.com  ---------------------------------------*/
/* --- BBS: www.STCAIMCU.com  -----------------------------------------*/
/* --- QQ:  800003751 -------------------------------------------------*/
/* ??????????????????,?????????????????STC???????????            */
/*---------------------------------------------------------------------*/

#include	"config.h"
#include	"STC32G_PWM.h"
#include	"STC32G_GPIO.h"
#include	"STC32G_NVIC.h"
#include	"STC32G_Timer.h"
#include	"STC32G_Delay.h"
#include  "iic.h"
#include  "font.h"
#include  "oled.h"




/*************	???????	**************

???PWM????? PWM5,PWM6,PWM7,PWM8 ????????????????PWM???.

4?????PWM????????????????????????????????????????.

PWM???????????????????????????65535.

?????, ?????? 24MHZ (???????"config.h"??????).

******************************************/

sbit AIN1=P4^5;
sbit AIN2=P2^7;
sbit BIN1=P2^5;
sbit BIN2=P2^6;

sbit zuo2=P0^3;
sbit zuo1=P0^4;
sbit zhong=P0^2;
sbit you1=P0^1;
sbit you2=P0^0;

sbit LED1=P4^1;
sbit LED2=P4^2;
sbit LED3=P4^4;
sbit TRIG = P0^6;
sbit ECHO = P1^4;
sbit KEY1 = P2^2;
sbit KEY2 = P2^3;
sbit KEY3 = P2^4;
sbit KEY4 = P1^3;
sbit KEY5 = P1^7;

volatile unsigned int time_us = 0;
bit measuring = 0;

PWMx_Duty PWMB_Duty;
/* MAIN_Fosc 24MHz / 1200 ~= 20kHz PWM for the motor driver. */
#define MOTOR_PWM_PERIOD 1200
u16 PWM_Period = MOTOR_PWM_PERIOD;
bit PWM5_Flag;
bit PWM6_Flag;

/* ?????? PID ???????????????? */
#define PID_SCALE      10
#define LINE_BASE_PWM  90

/* ?????? PID??????????? */
#define PID_KP_CENTER  4    // 0.8 直角转弯的Kp
#define PID_KI_CENTER  1    // 0.0 直角转弯的Ki
#define PID_KD_CENTER  20   // 1.0 直角转弯的Kd

/* ??1+?? ???? PID */
#define PID_KP_L1C     16   // 1.6
#define PID_KI_L1C     1
#define PID_KD_L1C     12   // 1.2

/* ??1+?? ???? PID */
#define PID_KP_R1C     16   // 1.6
#define PID_KI_R1C     1
#define PID_KD_R1C     12   // 1.2

/* ???? PWM ????????????100 ??????? */
#define LEFT_ADJUST    100
#define RIGHT_ADJUST   100

/* Five line sensor masks: 1 means black line detected. */
#define LINE_ZUO2_MASK     0x10
#define LINE_ZUO1_MASK     0x08
#define LINE_ZHONG_MASK    0x04
#define LINE_YOU1_MASK     0x02
#define LINE_YOU2_MASK     0x01
#define LINE_ALL_MASK      0x1F
#define LINE_LEFT_MASK     (LINE_ZUO2_MASK | LINE_ZUO1_MASK)
#define LINE_RIGHT_MASK    (LINE_YOU1_MASK | LINE_YOU2_MASK)

/* 90 degree corner parameters. Timer2 control period is about 2ms. */
#define TURN_LEFT              -1
#define TURN_RIGHT              1
#define TURN_CONFIRM_TICKS      3
#define TURN_MIN_TICKS          8
#define TURN_EXIT_TICKS         3
#define TURN_MAX_TICKS        180
#define TURN_OUTER_PWM         80
#define TURN_INNER_PWM         60
#define EDGE_TURN_OUTER_PWM    70
#define EDGE_TURN_INNER_PWM    40
#define INNER_SENSOR_TURN_PWM      70
#define OUTER_AUX_TURN_PWM        60

/* Brief line departure: both wheels keep moving forward with differential PWM. */
// 自动纠正直行的pid的转速
#define LINE_DEPART_FAST_PWM       91
#define LINE_DEPART_SLOW_PWM       70

/* Pivot is the primary sharp-corner strategy. One control tick is about 2ms. */
#define PIVOT_CONFIRM_TICKS     1
#define PIVOT_MIN_TICKS         8
#define PIVOT_MAX_TICKS       180
// #define PIVOT_SEARCH_FORWARD_PWM 65
// #define PIVOT_SEARCH_REVERSE_PWM 45
#define PIVOT_SEARCH_FORWARD_PWM 75 // 直角边转弯的大的PWM
#define PIVOT_SEARCH_REVERSE_PWM 80 // 直角边转弯的小的PWM
#define PIVOT_CAPTURE_FORWARD_PWM 91
#define PIVOT_CAPTURE_REVERSE_PWM 60
#define PIVOT_CAPTURE_EXIT_TICKS   3
#define PIVOT_PID_SCALE       10
#define PIVOT_PID_KP           8
#define PIVOT_PID_KI           2
#define PIVOT_PID_KD           20
#define PIVOT_PID_BASE_PWM     50
#define PIVOT_PID_MAX_CORR     20
#define PIVOT_PID_ERROR_SCALE  10
#define PIVOT_PID_SETTLE_TICKS  8
#define PIVOT_PID_LOST_TICKS    6

/* Junction detection. One control tick is about 2ms. */
#define JUNC_NONE                 0
#define JUNC_T_PREBIAS            1
#define JUNC_CROSS_PASS           2
#define JUNC_CROSS_PAIR_TICKS     40
#define JUNC_CROSS_STABLE_TICKS   2
#define JUNC_T_WINDOW_TICKS       5
#define JUNC_T_REQUIRED_HITS      3
#define JUNC_T_DECISION_TICKS    40
#define JUNC_DECISION_MAX_TICKS  41
#define JUNC_CROSS_MIN_TICKS     15
#define JUNC_CROSS_EXIT_TICKS     3
#define JUNC_CROSS_MAX_TICKS    100
#define JUNC_REARM_TICKS           5
#define JUNC_T_PREBIAS_FAST_PWM   100
#define JUNC_T_PREBIAS_SLOW_PWM   60
#define JUNC_EVIDENCE_INVALID    (JUNC_CROSS_PAIR_TICKS + 1)

#define LINE_ROUTE_MAIN           0
#define LINE_ROUTE_T_BRANCH       1
#define LINE_ROUTE_RETURN_FROM_DEAD 2
#define LINE_ROUTE_RETURN_PIVOT     3

#define PIVOT_PURPOSE_NONE          0
#define PIVOT_PURPOSE_NORMAL        1
#define PIVOT_PURPOSE_ENTER_T       2
#define PIVOT_PURPOSE_RETURN_MAIN   3

#define DEAD_NONE                   0
#define DEAD_LOST_CONFIRM           1
#define DEAD_UTURN_SEARCH           2
#define DEAD_UTURN_RECOVER          3
#define DEAD_FAULT                  4

#define DEAD_ARM_TICKS             35
#define DEAD_PRELINE_TICKS          3
#define ALL_WHITE_CONFIRM_TICKS   200

#define UTURN_OUTER_PWM             -80
#define UTURN_INNER_PWM             80
#define UTURN_MIN_TICKS           120
#define UTURN_SETTLE_TICKS          8
#define UTURN_LOST_TICKS            6
#define UTURN_REACQUIRE_MASK       (LINE_ZUO1_MASK | \
                                    LINE_ZHONG_MASK | \
                                    LINE_YOU1_MASK)

#define RETURN_NONE                 0
#define RETURN_CONFIRM              1
#define RETURN_ARM_TICKS           35
#define RETURN_CENTER_TICKS         5
#define RETURN_T_PAIR_TICKS        35
#define RETURN_T_STABLE_TICKS       2
#define RETURN_T_DECISION_TICKS    35
#define RETURN_T_MAX_TICKS         60
#define RETURN_EVIDENCE_INVALID   (RETURN_T_PAIR_TICKS + 1)

int pid_last_error_center = 0;
int pid_integral_center = 0;
int pid_last_error_l1c = 0;
int pid_integral_l1c = 0;
int pid_last_error_r1c = 0;
int pid_integral_r1c = 0;
unsigned char pid_div = 0;
volatile unsigned char test = 0;
signed char turn_dir = 0;
unsigned char turn_ticks = 0;
unsigned char turn_confirm_ticks = 0;
unsigned char turn_exit_ticks = 0;
signed char pivot_dir = 0;
int pivot_outer_pwm = 0;
int pivot_inner_pwm = 0;
unsigned int pivot_ticks = 0;
signed char pivot_pending_dir = 0;
unsigned char pivot_confirm_ticks = 0;
unsigned char pivot_purpose = PIVOT_PURPOSE_NONE;
bit pivot_recovering = 0;
int pivot_pid_last_error = 0;
int pivot_pid_integral = 0;
unsigned char pivot_pid_settle_ticks = 0;
unsigned char pivot_pid_lost_ticks = 0;
unsigned char junc_state = JUNC_NONE;
unsigned char junc_ticks = 0;
unsigned char junc_left_age = JUNC_EVIDENCE_INVALID;
unsigned char junc_right_age = JUNC_EVIDENCE_INVALID;
unsigned char junc_cross_ticks = 0;
unsigned char junc_exit_ticks = 0;
unsigned char junc_rearm_ticks = 0;
bit junc_blocked = 0;
signed char junc_turn_dir = 0;
signed char junc_t_candidate_dir = 0;
unsigned char junc_t_candidate_hits = 0;
unsigned char junc_t_window_ticks = 0;
signed char t_branch_dir = 0;
unsigned char line_route_state = LINE_ROUTE_MAIN;
unsigned char dead_state = DEAD_NONE;
unsigned int branch_ticks = 0;
unsigned char branch_center_ticks = 0;
unsigned char dead_lost_ticks = 0;
signed char uturn_dir = TURN_LEFT;
unsigned int uturn_ticks = 0;
unsigned char uturn_settle_ticks = 0;
unsigned char uturn_lost_ticks = 0;
int uturn_pid_last_error = 0;
unsigned char return_state = RETURN_NONE;
unsigned int return_ticks = 0;
unsigned char return_center_ticks = 0;
bit return_armed = 0;
unsigned char return_candidate_ticks = 0;
unsigned char return_left_age = RETURN_EVIDENCE_INVALID;
unsigned char return_right_age = RETURN_EVIDENCE_INVALID;
unsigned char return_pair_ticks = 0;
bit return_pair_confirmed = 0;

void Line_OnPivotSuccess(unsigned char line_mask);
void Line_OnPivotTimeout(void);

/*************	???????????	**************/



/*************  ??????????????? *****************/



/************************ IO?????? ****************************/
void	GPIO_config(void)
{

	
}

/************************ ????????? ****************************/
void	Timer_config(void)
{
	TIM_InitTypeDef		TIM_InitStructure;					//??????
	TIM_InitStructure.TIM_Mode      = TIM_16BitAutoReload;	//?????????,   TIM_16BitAutoReload,TIM_16Bit,TIM_8BitAutoReload,TIM_16BitAutoReloadNoMask
	TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_1T;		//???????,     TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
	TIM_InitStructure.TIM_ClkOut    = DISABLE;				//??????????????, ENABLE??DISABLE
	TIM_InitStructure.TIM_Value     = (u16)(65536UL - (MAIN_Fosc / 1000UL));		//???????, 1000??/??
	TIM_InitStructure.TIM_PS        = 0;					//8????????(n+1), 0~255
	TIM_InitStructure.TIM_Run       = ENABLE;				//??????????????????, ENABLE??DISABLE
	Timer_Inilize(Timer0,&TIM_InitStructure);				//?????Timer0	  Timer0,Timer1,Timer2,Timer3,Timer4
	NVIC_Timer0_Init(ENABLE,Priority_0);		//???????, ENABLE/DISABLE; ?????(?????) Priority_0,Priority_1,Priority_2,Priority_3
}

void Timer1_config(void)
{
    TIM_InitTypeDef TIM_InitStructure;

    TIM_InitStructure.TIM_Mode      = TIM_16BitAutoReload;              // 16????????
    TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_1T;                     // ???1T???????
    TIM_InitStructure.TIM_ClkOut    = DISABLE;                          // ????????????
		TIM_InitStructure.TIM_Value 		= (u16)(65536UL - (MAIN_Fosc / 100000UL)); // 10us???
    TIM_InitStructure.TIM_PS        = 0;                                // ??????
    TIM_InitStructure.TIM_Run       = ENABLE;                           // ???????????????

    Timer_Inilize(Timer1, &TIM_InitStructure);                          // ??????????1
    NVIC_Timer1_Init(ENABLE, Priority_0);                               // ???????????????????0
}

void Timer2_config(void)
{
    TIM_InitTypeDef TIM_InitStructure;

    TIM_InitStructure.TIM_Mode      = TIM_16BitAutoReload;              // 16????????
    TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_1T;                     // ???1T???????
    TIM_InitStructure.TIM_ClkOut    = DISABLE;                          // ????????????
		TIM_InitStructure.TIM_Value 		= (u16)(65536UL - (MAIN_Fosc / 10000UL)); // 100us???
    TIM_InitStructure.TIM_PS        = 0;                                // ??????
    TIM_InitStructure.TIM_Run       = ENABLE;                           // ???????????????

    Timer_Inilize(Timer2, &TIM_InitStructure);                          // ??????????2
    NVIC_Timer2_Init(ENABLE, Priority_0);                               // ???????????????????0
}

/***************  ?????????? *****************/
void Timer1_ISR_Handler (void) interrupt TMR1_VECTOR		//????????????????
{
	if (measuring) time_us++;
}

unsigned int Ultrasonic_GetDistance(void) 
{
    unsigned int Distance_mm = 0;
    TRIG = 0;
    _nop_(); _nop_(); _nop_(); _nop_();
    _nop_(); _nop_(); _nop_(); _nop_();
    TRIG = 1;
    _nop_(); _nop_(); _nop_(); _nop_();    // ?????10us
    _nop_(); _nop_(); _nop_(); _nop_();
    _nop_(); _nop_(); _nop_(); _nop_();
    TRIG = 0;

    // ??? ECHO ???????
    while (!ECHO);
    
    // ??????
    time_us = 0;
    measuring = 1;

    // ??? ECHO ???????????
    while (ECHO);

    measuring = 0;
		if(time_us/100<38)									//???????????38????????38??????????????????????Y??0
		{
			Distance_mm=(time_us*346)/100;						//???????25??C?????????????346m/s   ????????time_end???????10?????????????????????????????????????100
		}
    return Distance_mm;
}

/***************  PWM????????? *****************/
void	PWM_config(void)
{
	PWMx_InitDefine		PWMx_InitStructure;
	
//	PWMB_Duty.PWM5_Duty = 128;
//	PWMB_Duty.PWM6_Duty = 256;


	PWMx_InitStructure.PWM_Mode    =	CCMRn_PWM_MODE1;	//??,		CCMRn_FREEZE,CCMRn_MATCH_VALID,CCMRn_MATCH_INVALID,CCMRn_ROLLOVER,CCMRn_FORCE_INVALID,CCMRn_FORCE_VALID,CCMRn_PWM_MODE1,CCMRn_PWM_MODE2
	PWMx_InitStructure.PWM_Duty    = PWMB_Duty.PWM5_Duty;	//PWM???????, 0~Period
	PWMx_InitStructure.PWM_EnoSelect   = ENO5P;					//?????????,	ENO1P,ENO1N,ENO2P,ENO2N,ENO3P,ENO3N,ENO4P,ENO4N / ENO5P,ENO6P,ENO7P,ENO8P
	PWM_Configuration(PWM5, &PWMx_InitStructure);				//?????PWM,  PWMA,PWMB

	PWMx_InitStructure.PWM_Mode    =	CCMRn_PWM_MODE1;	//??,		CCMRn_FREEZE,CCMRn_MATCH_VALID,CCMRn_MATCH_INVALID,CCMRn_ROLLOVER,CCMRn_FORCE_INVALID,CCMRn_FORCE_VALID,CCMRn_PWM_MODE1,CCMRn_PWM_MODE2
	PWMx_InitStructure.PWM_Duty    = PWMB_Duty.PWM6_Duty;	//PWM???????, 0~Period
	PWMx_InitStructure.PWM_EnoSelect   = ENO6P;					//?????????,	ENO1P,ENO1N,ENO2P,ENO2N,ENO3P,ENO3N,ENO4P,ENO4N / ENO5P,ENO6P,ENO7P,ENO8P
	PWM_Configuration(PWM6, &PWMx_InitStructure);				//?????PWM,  PWMA,PWMB


	PWMx_InitStructure.PWM_Period   = PWM_Period; // MOTOR_PWM_PERIOD							//???????,   0~65535
	PWMx_InitStructure.PWM_DeadTime = 0;								//??????????????, 0~255
	PWMx_InitStructure.PWM_MainOutEnable= ENABLE;				//????????, ENABLE,DISABLE
	PWMx_InitStructure.PWM_CEN_Enable   = ENABLE;				//????????, ENABLE,DISABLE
	PWM_Configuration(PWMB, &PWMx_InitStructure);				//?????PWM????????,  PWMA,PWMB


		PWM6_USE_P21();
		PWM5_USE_P20();
	NVIC_PWM_Init(PWMB,DISABLE,Priority_0);
}

void PWM_Left(int pwm)     // positive: AIN1=0,AIN2=1; negative: AIN1=1,AIN2=0
{
	if(pwm >= 0)
	{
		AIN1 = 0;
		AIN2 = 1;
	}
	else
	{
		AIN1 = 1;
		AIN2 = 0;
		pwm = -pwm;
	}

	if(pwm > 100) pwm = 100;
	PWMB_Duty.PWM6_Duty = pwm*(PWM_Period/100);
}

void PWM_Right(int pwm)     // positive: BIN1=1,BIN2=0; negative: BIN1=0,BIN2=1
{
	if(pwm >= 0)
	{
		BIN1 = 1;
		BIN2 = 0;
	}
	else
	{
		BIN1 = 0;
		BIN2 = 1;
		pwm = -pwm;
	}

	if(pwm > 100) pwm = 100;
	PWMB_Duty.PWM5_Duty = pwm*(PWM_Period/100);
}

void PWM_Run(int pwm1,int pwm2)  // -100~100, negative means reverse
{
	PWM_Left(pwm1);
	PWM_Right(pwm2);
	UpdatePwm(PWMB, &PWMB_Duty);
}

void Motor_Reverse_Test(void)
{
	unsigned char i;

	PWM_Run(0, 0);
	for(i = 0; i < 3; i++)
	{
		LED1 = 1; delay_ms(130); LED1 = 0;
		LED2 = 1; delay_ms(130); LED2 = 0;
		LED3 = 1; delay_ms(130); LED3 = 0;
	}

	PWM_Run(90, 90);
	delay_ms(1000);
	PWM_Run(-90, 90);
	delay_ms(1500);
	PWM_Run(90, -90);
	delay_ms(1500);
	PWM_Run(0, 0);
	delay_ms(800);
}

// void test01(void)
// {
// 		PWM_Run(100,80);
// 		delay_ms(1000);
// 		PWM_Run(80,100);
// 		delay_ms(1000);
// 		PWM_Run(0,100);
// 		delay_ms(1000);
// 		PWM_Run(100,0);
// 		delay_ms(1000);
// }

unsigned char Line_GetMask(void)
{
	unsigned char mask = 0;

	if(zuo2) mask |= LINE_ZUO2_MASK;
	if(zuo1) mask |= LINE_ZUO1_MASK;
	if(zhong) mask |= LINE_ZHONG_MASK;
	if(you1) mask |= LINE_YOU1_MASK;
	if(you2) mask |= LINE_YOU2_MASK;

	return mask;
}

int Line_GetError_ByMask(unsigned char line_mask)
{
	int sum = 0;
	int cnt = 0;

	if(line_mask & LINE_ZUO2_MASK)  { sum -= 4; cnt++; }
	if(line_mask & LINE_ZUO1_MASK)  { sum -= 2; cnt++; }
	if(line_mask & LINE_ZHONG_MASK) { cnt++; }
	if(line_mask & LINE_YOU1_MASK)  { sum += 2; cnt++; }
	if(line_mask & LINE_YOU2_MASK)  { sum += 4; cnt++; }

	if(cnt == 0) return pid_last_error_center;
	return sum / cnt;
}

int Line_GetError(void)
{
	return Line_GetError_ByMask(Line_GetMask());
}

void Line_ResetPID(void)
{
	pid_last_error_center = 0;
	pid_integral_center = 0;
	pid_last_error_l1c = 0;
	pid_integral_l1c = 0;
	pid_last_error_r1c = 0;
	pid_integral_r1c = 0;
}

void Line_ClearSharpTurn(void)
{
	turn_dir = 0;
	turn_ticks = 0;
	turn_confirm_ticks = 0;
	turn_exit_ticks = 0;
}

signed char Line_GetSharpTurnDir(unsigned char line_mask)
{
	if((line_mask & LINE_ZUO2_MASK) && !(line_mask & (LINE_ZHONG_MASK | LINE_RIGHT_MASK)))
	{
		return TURN_LEFT;
	}
	if((line_mask & LINE_YOU2_MASK) && !(line_mask & (LINE_ZHONG_MASK | LINE_LEFT_MASK)))
	{
		return TURN_RIGHT;
	}
	return 0;
}

void Line_PID_Run_Custom_Mask(unsigned char line_mask, int base_pwm, int KP, int KI, int KD, int *pid_last_err, int *pid_integral)
{
	int error;
	int derivative;
	int correction;
	int left_pwm;
	int right_pwm;

	error = Line_GetError_ByMask(line_mask);
	if(KI != 0)
	{
		*pid_integral += error;
		if(*pid_integral > 80) *pid_integral = 80;
		if(*pid_integral < -80) *pid_integral = -80;
	}
	else
	{
		*pid_integral = 0;
	}

	derivative = error - *pid_last_err;
	*pid_last_err = error;

	correction = (KP * error + KI * (*pid_integral) + KD * derivative) / PID_SCALE;

	left_pwm = base_pwm + correction;
	right_pwm = base_pwm - correction;

	left_pwm = left_pwm * LEFT_ADJUST / 100;
	right_pwm = right_pwm * RIGHT_ADJUST / 100;

	if(left_pwm > 100) left_pwm = 100;
	if(left_pwm < 0) left_pwm = 0;
	if(right_pwm > 100) right_pwm = 100;
	if(right_pwm < 0) right_pwm = 0;

	PWM_Run(left_pwm, right_pwm);
}

void Line_PID_Run_Custom(int base_pwm, int KP, int KI, int KD, int *pid_last_err, int *pid_integral)
{
	Line_PID_Run_Custom_Mask(Line_GetMask(), base_pwm, KP, KI, KD, pid_last_err, pid_integral);
}

void Line_PID_Run_Mask(unsigned char line_mask, int base_pwm)
{
	Line_PID_Run_Custom_Mask(line_mask, base_pwm, PID_KP_CENTER, PID_KI_CENTER, PID_KD_CENTER, &pid_last_error_center, &pid_integral_center);
}

void Line_PID_Run(int base_pwm)
{
	Line_PID_Run_Mask(Line_GetMask(), base_pwm);
}

void Line_RecordCenterError(unsigned char line_mask)
{
	int error;

	error = Line_GetError_ByMask(line_mask);
	pid_last_error_center = error;
}

void Line_RunPivotLeft(int outer_pwm, int inner_pwm)
{
	PWM_Run(-inner_pwm, outer_pwm);
}

void Line_RunPivotRight(int outer_pwm, int inner_pwm)
{
	PWM_Run(outer_pwm, -inner_pwm);
}

void Line_ResetPivotCandidate(void)
{
	pivot_pending_dir = 0;
	pivot_confirm_ticks = 0;
}

void Line_ResetPivotPID(void)
{
	pivot_recovering = 0;
	pivot_pid_last_error = 0;
	pivot_pid_integral = 0;
	pivot_pid_settle_ticks = 0;
	pivot_pid_lost_ticks = 0;
}

void Line_ClearPivot(void)
{
	pivot_dir = 0;
	pivot_outer_pwm = 0;
	pivot_inner_pwm = 0;
	pivot_ticks = 0;
	pivot_purpose = PIVOT_PURPOSE_NONE;
	Line_ResetPivotCandidate();
	Line_ResetPivotPID();
}

void Line_RunPivot(void)
{
	if(pivot_dir == TURN_RIGHT)
	{
		Line_RunPivotRight(pivot_outer_pwm, pivot_inner_pwm);
	}
	else if(pivot_dir == TURN_LEFT)
	{
		Line_RunPivotLeft(pivot_outer_pwm, pivot_inner_pwm);
	}
}

void Line_StartPivotWithPurpose(signed char dir, int outer_pwm, int inner_pwm,
                                unsigned char purpose)
{
	pivot_dir = dir;
	pivot_outer_pwm = outer_pwm;
	pivot_inner_pwm = inner_pwm;
	pivot_ticks = 1;
	pivot_purpose = purpose;
	Line_ResetPivotCandidate();
	Line_ResetPivotPID();
	Line_ClearSharpTurn();
	Line_ResetPID();
	Line_RunPivot();
}

void Line_StartPivot(signed char dir, int outer_pwm, int inner_pwm)
{
	Line_StartPivotWithPurpose(dir, outer_pwm, inner_pwm,
	                          PIVOT_PURPOSE_NORMAL);
}

void Line_StartPivotRecovery(unsigned char line_mask)
{
	pivot_recovering = 1;
	pivot_pid_last_error = Line_GetError_ByMask(line_mask) * PIVOT_PID_ERROR_SCALE;
	pivot_pid_integral = 0;
	pivot_pid_settle_ticks = 0;
	pivot_pid_lost_ticks = 0;
}

void Line_RunPivotRecovery(unsigned char line_mask)
{
	bit valid_center;

	valid_center = ((line_mask & LINE_ZHONG_MASK) != 0) &&
	               ((line_mask & (LINE_ZUO2_MASK |
	                              LINE_YOU2_MASK)) == 0);

	if(!valid_center)
	{
		Line_ResetPivotPID();
		Line_RunPivot();
		return;
	}

	if(pivot_pid_settle_ticks < PIVOT_CAPTURE_EXIT_TICKS)
	{
		pivot_pid_settle_ticks++;
	}

	if(pivot_pid_settle_ticks >= PIVOT_CAPTURE_EXIT_TICKS)
	{
		Line_OnPivotSuccess(line_mask);
		return;
	}

	if(pivot_dir == TURN_RIGHT)
	{
		Line_RunPivotRight(PIVOT_CAPTURE_FORWARD_PWM,
		                   PIVOT_CAPTURE_REVERSE_PWM);
	}
	else
	{
		Line_RunPivotLeft(PIVOT_CAPTURE_FORWARD_PWM,
		                  PIVOT_CAPTURE_REVERSE_PWM);
	}
}

void Line_RunPivotState(unsigned char line_mask)
{
	bit center_reacquired;

	if(pivot_ticks >= PIVOT_MAX_TICKS)
	{
		Line_OnPivotTimeout();
		return;
	}

	if(pivot_recovering)
	{
		Line_RunPivotRecovery(line_mask);
		if(pivot_dir != 0) pivot_ticks++;
		return;
	}

	center_reacquired = ((line_mask & LINE_ZHONG_MASK) != 0) &&
	                    ((line_mask & (LINE_ZUO2_MASK | LINE_YOU2_MASK)) == 0);

	if((pivot_ticks >= PIVOT_MIN_TICKS) && center_reacquired)
	{
		Line_StartPivotRecovery(line_mask);
		Line_RunPivotRecovery(line_mask);
		if(pivot_dir != 0) pivot_ticks++;
		return;
	}

	Line_RunPivot();
	pivot_ticks++;
}

signed char Line_GetPivotCandidate(unsigned char line_mask)
{
	if((line_mask & LINE_YOU2_MASK) &&
	   !(line_mask & LINE_ZUO2_MASK))
	{
		return TURN_RIGHT;
	}

	if((line_mask & LINE_ZUO2_MASK) &&
	   !(line_mask & LINE_YOU2_MASK))
	{
		return TURN_LEFT;
	}

	return 0;
}

void Line_HandleAllBlack(void)
{
	Line_ClearPivot();
	Line_ClearSharpTurn();
	Line_ResetPID();
	PWM_Run(LINE_BASE_PWM, LINE_BASE_PWM);
}

void Line_HandleAllWhite(void)
{
	Line_ResetPivotCandidate();
	Line_ClearSharpTurn();
	Line_ResetPID();
	PWM_Run(LINE_BASE_PWM, LINE_BASE_PWM);
}

unsigned char Line_IsLeftJunctionEvidence(unsigned char line_mask)
{
	return ((line_mask & LINE_ZUO2_MASK) != 0) &&
	       ((line_mask & (LINE_ZUO1_MASK | LINE_ZHONG_MASK)) != 0);
}

unsigned char Line_IsRightJunctionEvidence(unsigned char line_mask)
{
	return ((line_mask & LINE_YOU2_MASK) != 0) &&
	       ((line_mask & (LINE_YOU1_MASK | LINE_ZHONG_MASK)) != 0);
}

unsigned char Line_IsLeftTCandidate(unsigned char line_mask)
{
	return ((line_mask & LINE_LEFT_MASK) == LINE_LEFT_MASK) &&
	       ((line_mask & (LINE_ZHONG_MASK | LINE_YOU1_MASK)) != 0) &&
	       ((line_mask & LINE_YOU2_MASK) == 0);
}

unsigned char Line_IsRightTCandidate(unsigned char line_mask)
{
	return ((line_mask & LINE_RIGHT_MASK) == LINE_RIGHT_MASK) &&
	       ((line_mask & (LINE_ZHONG_MASK | LINE_ZUO1_MASK)) != 0) &&
	       ((line_mask & LINE_ZUO2_MASK) == 0);
}

unsigned char Line_IsSingleSensor(unsigned char line_mask)
{
	return (line_mask != 0) && ((line_mask & (line_mask - 1)) == 0);
}

unsigned char Line_IsNarrowCenter(unsigned char line_mask)
{
	return ((line_mask & LINE_ZHONG_MASK) != 0) &&
	       ((line_mask & (LINE_ZUO2_MASK | LINE_YOU2_MASK)) == 0);
}

unsigned char Line_IsNarrowTrack(unsigned char line_mask)
{
	if(Line_IsSingleSensor(line_mask))
	{
		return 1;
	}

	return (line_mask != 0) &&
	       ((line_mask & (LINE_ZUO2_MASK | LINE_YOU2_MASK)) == 0);
}

void Line_ClearJunctionState(void)
{
	junc_state = JUNC_NONE;
	junc_ticks = 0;
	junc_left_age = JUNC_EVIDENCE_INVALID;
	junc_right_age = JUNC_EVIDENCE_INVALID;
	junc_cross_ticks = 0;
	junc_exit_ticks = 0;
	junc_turn_dir = 0;
	junc_t_candidate_dir = 0;
	junc_t_candidate_hits = 0;
	junc_t_window_ticks = 0;
}

void Line_BlockJunction(void)
{
	Line_ClearJunctionState();
	junc_blocked = 1;
	junc_rearm_ticks = 0;
}

void Line_UpdateJunctionRearm(unsigned char line_mask)
{
	if(!junc_blocked)
	{
		return;
	}

	if(Line_IsNarrowCenter(line_mask))
	{
		if(junc_rearm_ticks < JUNC_REARM_TICKS)
		{
			junc_rearm_ticks++;
		}

		if(junc_rearm_ticks >= JUNC_REARM_TICKS)
		{
			junc_blocked = 0;
			junc_rearm_ticks = 0;
		}
	}
	else
	{
		junc_rearm_ticks = 0;
	}
}

unsigned char Line_CanTrackJunction(void)
{
	if(junc_blocked)
	{
		return 0;
	}

	/* Branch and return routes have dedicated dead-end/T handlers. */
	if(line_route_state != LINE_ROUTE_MAIN)
	{
		return 0;
	}

	return 1;
}

void Line_StartCrossPass(void)
{
	junc_state = JUNC_CROSS_PASS;
	junc_ticks = 0;
	junc_cross_ticks = 0;
	junc_exit_ticks = 0;
	junc_turn_dir = 0;
	junc_t_candidate_dir = 0;
	junc_t_candidate_hits = 0;
	junc_t_window_ticks = 0;
	Line_ResetPivotCandidate();
	Line_ClearSharpTurn();
	Line_ResetPID();
	PWM_Run(LINE_BASE_PWM, LINE_BASE_PWM);
}

void Line_RecordJunctionEvidence(unsigned char line_mask)
{
	if(Line_IsLeftJunctionEvidence(line_mask))
	{
		junc_left_age = 0;
	}
	else if(junc_left_age < JUNC_EVIDENCE_INVALID)
	{
		junc_left_age++;
	}

	if(Line_IsRightJunctionEvidence(line_mask))
	{
		junc_right_age = 0;
	}
	else if(junc_right_age < JUNC_EVIDENCE_INVALID)
	{
		junc_right_age++;
	}
}

unsigned char Line_HasCrossEvidence(void)
{
	return (junc_left_age <= JUNC_CROSS_PAIR_TICKS) &&
	       (junc_right_age <= JUNC_CROSS_PAIR_TICKS);
}

unsigned char Line_TryCrossInterruptPivot(unsigned char line_mask)
{
	unsigned char left_evidence;
	unsigned char right_evidence;

	if((pivot_purpose != PIVOT_PURPOSE_NORMAL) ||
	   !Line_CanTrackJunction())
	{
		return 0;
	}

	left_evidence = Line_IsLeftJunctionEvidence(line_mask);
	right_evidence = Line_IsRightJunctionEvidence(line_mask);
	Line_RecordJunctionEvidence(line_mask);

	if((Line_HasCrossEvidence() || (junc_cross_ticks != 0)) &&
	   (left_evidence || right_evidence))
	{
		if(junc_cross_ticks < JUNC_CROSS_STABLE_TICKS)
		{
			junc_cross_ticks++;
		}
	}
	else
	{
		junc_cross_ticks = 0;
	}

	if(junc_cross_ticks >= JUNC_CROSS_STABLE_TICKS)
	{
		Line_ClearPivot();
		Line_StartCrossPass();
		return 1;
	}

	if(junc_cross_ticks != 0)
	{
		PWM_Run(LINE_BASE_PWM, LINE_BASE_PWM);
		return 1;
	}

	return 0;
}

signed char Line_GetTCandidateDir(unsigned char line_mask)
{
	if(Line_IsLeftTCandidate(line_mask))
	{
		return TURN_LEFT;
	}

	if(Line_IsRightTCandidate(line_mask))
	{
		return TURN_RIGHT;
	}

	return 0;
}

void Line_ResetTCandidate(void)
{
	junc_t_candidate_dir = 0;
	junc_t_candidate_hits = 0;
	junc_t_window_ticks = 0;
	junc_ticks = 0;
}

signed char Line_UpdateTCandidate(unsigned char line_mask)
{
	signed char candidate_dir;

	candidate_dir = Line_GetTCandidateDir(line_mask);
	if(candidate_dir != 0)
	{
		if(junc_t_candidate_dir != candidate_dir)
		{
			junc_t_candidate_dir = candidate_dir;
			junc_t_candidate_hits = 1;
			junc_t_window_ticks = 1;
			junc_ticks = 1;
		}
		else
		{
			if(junc_t_candidate_hits < JUNC_T_REQUIRED_HITS)
			{
				junc_t_candidate_hits++;
			}
			if(junc_t_window_ticks < JUNC_T_WINDOW_TICKS)
			{
				junc_t_window_ticks++;
			}
			if(junc_ticks < JUNC_T_DECISION_TICKS)
			{
				junc_ticks++;
			}
		}
	}
	else if(junc_t_candidate_dir != 0)
	{
		if(junc_t_window_ticks < JUNC_T_WINDOW_TICKS)
		{
			junc_t_window_ticks++;
		}
		if(junc_ticks < JUNC_T_DECISION_TICKS)
		{
			junc_ticks++;
		}
	}

	if(junc_t_candidate_hits >= JUNC_T_REQUIRED_HITS)
	{
		return junc_t_candidate_dir;
	}

	if(junc_t_window_ticks >= JUNC_T_WINDOW_TICKS)
	{
		Line_ResetTCandidate();
	}

	return 0;
}

void Line_StartTPrebias(signed char dir)
{
	junc_state = JUNC_T_PREBIAS;
	junc_turn_dir = dir;
	junc_t_candidate_dir = 0;
	junc_t_candidate_hits = 0;
	junc_t_window_ticks = 0;
	Line_ResetPivotCandidate();
	Line_ClearSharpTurn();
	Line_ResetPID();

	if(dir == TURN_LEFT)
	{
		PWM_Run(JUNC_T_PREBIAS_SLOW_PWM,
		        JUNC_T_PREBIAS_FAST_PWM);
	}
	else
	{
		PWM_Run(JUNC_T_PREBIAS_FAST_PWM,
		        JUNC_T_PREBIAS_SLOW_PWM);
	}
}

unsigned char Line_UpdateCrossBackground(unsigned char line_mask)
{
	unsigned char left_evidence;
	unsigned char right_evidence;

	if(!Line_CanTrackJunction())
	{
		return 0;
	}

	left_evidence = Line_IsLeftJunctionEvidence(line_mask);
	right_evidence = Line_IsRightJunctionEvidence(line_mask);
	Line_RecordJunctionEvidence(line_mask);

	if((Line_HasCrossEvidence() || (junc_cross_ticks != 0)) &&
	   (left_evidence || right_evidence))
	{
		if(junc_cross_ticks < JUNC_CROSS_STABLE_TICKS)
		{
			junc_cross_ticks++;
		}
	}
	else
	{
		junc_cross_ticks = 0;
	}

	if(junc_cross_ticks >= JUNC_CROSS_STABLE_TICKS)
	{
		Line_StartCrossPass();
		return 1;
	}

	if(junc_cross_ticks != 0)
	{
		Line_ResetTCandidate();
		PWM_Run(LINE_BASE_PWM, LINE_BASE_PWM);
		return 1;
	}

	return 0;
}

unsigned char Line_UpdateTJunctionBackground(unsigned char line_mask)
{
	signed char current_t_dir;
	signed char t_candidate_dir;

	if(!Line_CanTrackJunction())
	{
		return 0;
	}

	current_t_dir = Line_GetTCandidateDir(line_mask);
	t_candidate_dir = Line_UpdateTCandidate(line_mask);
	if(t_candidate_dir != 0)
	{
		Line_StartTPrebias(t_candidate_dir);
		return 1;
	}

	/* Keep wide cross/T candidate patterns at full straight speed. */
	if(current_t_dir != 0)
	{
		PWM_Run(LINE_BASE_PWM, LINE_BASE_PWM);
		return 1;
	}

	return 0;
}

void Line_RunJunction(unsigned char line_mask)
{
	unsigned char left_evidence;
	unsigned char right_evidence;
	signed char turn_dir_now;

	if(junc_state == JUNC_T_PREBIAS)
	{
		if(line_mask == 0)
		{
			Line_BlockJunction();
			Line_HandleAllWhite();
			return;
		}

		left_evidence = Line_IsLeftJunctionEvidence(line_mask);
		right_evidence = Line_IsRightJunctionEvidence(line_mask);
		Line_RecordJunctionEvidence(line_mask);
		junc_ticks++;

		if((Line_HasCrossEvidence() || (junc_cross_ticks != 0)) &&
		   (left_evidence || right_evidence))
		{
			if(junc_cross_ticks < JUNC_CROSS_STABLE_TICKS)
			{
				junc_cross_ticks++;
			}
		}
		else
		{
			junc_cross_ticks = 0;
		}

		if(junc_cross_ticks >= JUNC_CROSS_STABLE_TICKS)
		{
			Line_StartCrossPass();
			return;
		}

		if((junc_ticks >= JUNC_T_DECISION_TICKS) &&
		   (junc_ticks >= JUNC_DECISION_MAX_TICKS ||
		    junc_cross_ticks == 0))
		{
			turn_dir_now = junc_turn_dir;
			t_branch_dir = turn_dir_now;
			Line_BlockJunction();
			Line_StartPivotWithPurpose(turn_dir_now,
			                           PIVOT_SEARCH_FORWARD_PWM,
			                           PIVOT_SEARCH_REVERSE_PWM,
			                           PIVOT_PURPOSE_ENTER_T);
			return;
		}

		if(((junc_turn_dir == TURN_LEFT) && left_evidence) ||
		   ((junc_turn_dir == TURN_RIGHT) && right_evidence))
		{
			if(junc_turn_dir == TURN_LEFT)
			{
				PWM_Run(JUNC_T_PREBIAS_SLOW_PWM,
				        JUNC_T_PREBIAS_FAST_PWM);
			}
			else
			{
				PWM_Run(JUNC_T_PREBIAS_FAST_PWM,
				        JUNC_T_PREBIAS_SLOW_PWM);
			}
		}
		else
		{
			PWM_Run(LINE_BASE_PWM, LINE_BASE_PWM);
		}
		return;
	}

	if(junc_state == JUNC_CROSS_PASS)
	{
		PWM_Run(LINE_BASE_PWM, LINE_BASE_PWM);
		junc_ticks++;

		if((junc_ticks >= JUNC_CROSS_MIN_TICKS) &&
		   Line_IsNarrowCenter(line_mask))
		{
			if(junc_exit_ticks < JUNC_CROSS_EXIT_TICKS)
			{
				junc_exit_ticks++;
			}
		}
		else
		{
			junc_exit_ticks = 0;
		}

		if((junc_exit_ticks >= JUNC_CROSS_EXIT_TICKS) ||
		   (junc_ticks >= JUNC_CROSS_MAX_TICKS))
		{
			Line_BlockJunction();
			Line_ResetPID();
		}
	}
}

void Line_ResetBranchTracking(void)
{
	branch_ticks = 0;
	branch_center_ticks = 0;
}

void Line_ClearDeadState(void)
{
	dead_state = DEAD_NONE;
	dead_lost_ticks = 0;
	uturn_ticks = 0;
	uturn_settle_ticks = 0;
	uturn_lost_ticks = 0;
	uturn_pid_last_error = 0;
}

void Line_ResetReturnTracking(void)
{
	return_state = RETURN_NONE;
	return_ticks = 0;
	return_center_ticks = 0;
	return_armed = 0;
	return_candidate_ticks = 0;
	return_left_age = RETURN_EVIDENCE_INVALID;
	return_right_age = RETURN_EVIDENCE_INVALID;
	return_pair_ticks = 0;
	return_pair_confirmed = 0;
}

void Line_UpdateBranchTracking(unsigned char line_mask)
{
	if(line_route_state != LINE_ROUTE_T_BRANCH)
	{
		return;
	}

	if(branch_ticks < 1000)
	{
		branch_ticks++;
	}

	if(Line_IsNarrowCenter(line_mask))
	{
		if(branch_center_ticks < DEAD_PRELINE_TICKS)
		{
			branch_center_ticks++;
		}
	}
	else if(line_mask != 0)
	{
		branch_center_ticks = 0;
	}
}

unsigned char Line_IsDeadLost(unsigned char line_mask)
{
	return (line_route_state == LINE_ROUTE_T_BRANCH) &&
	       (branch_ticks >= DEAD_ARM_TICKS) &&
	       (branch_center_ticks >= DEAD_PRELINE_TICKS) &&
	       (line_mask == 0);
}

void Line_RunRouteFallback(unsigned char line_mask)
{
	if(line_mask == LINE_ALL_MASK)
	{
		Line_HandleAllBlack();
		return;
	}

	if(line_mask == 0)
	{
		Line_HandleAllWhite();
		return;
	}

	Line_ResetPID();
	Line_PID_Run_Mask(line_mask, LINE_BASE_PWM);
}

void Line_RunUturnMotor(void)
{
	/* Fixed right turn: left outer wheel is faster, both wheels move forward. */
	PWM_Run(UTURN_OUTER_PWM, UTURN_INNER_PWM);
}

void Line_EnterDeadFault(void)
{
	dead_state = DEAD_FAULT;
	dead_lost_ticks = 0;
	uturn_settle_ticks = 0;
	uturn_lost_ticks = 0;
	PWM_Run(0, 0);
}

void Line_StartDeadLostConfirm(void)
{
	dead_state = DEAD_LOST_CONFIRM;
	dead_lost_ticks = 0;
	Line_ClearSharpTurn();
	Line_ClearJunctionState();
	Line_ResetPivotCandidate();
	Line_ResetPID();
	PWM_Run(LINE_BASE_PWM, LINE_BASE_PWM);
}

void Line_StartUturn(void)
{
	dead_state = DEAD_UTURN_SEARCH;
	dead_lost_ticks = 0;
	uturn_ticks = 1;
	uturn_settle_ticks = 0;
	uturn_lost_ticks = 0;
	uturn_pid_last_error = 0;

	uturn_dir = TURN_RIGHT;

	Line_ClearPivot();
	Line_ClearSharpTurn();
	Line_ClearJunctionState();
	Line_ResetPID();
	Line_RunUturnMotor();
}

void Line_StartUturnRecovery(unsigned char line_mask)
{
	dead_state = DEAD_UTURN_RECOVER;
	uturn_pid_last_error =
		Line_GetError_ByMask(line_mask) * PIVOT_PID_ERROR_SCALE;
	uturn_settle_ticks = 0;
	uturn_lost_ticks = 0;
}

void Line_FinishUturn(unsigned char line_mask)
{
	Line_ClearDeadState();
	Line_ResetBranchTracking();
	Line_ResetReturnTracking();
	line_route_state = LINE_ROUTE_MAIN;
	t_branch_dir = 0;
	Line_BlockJunction();
	Line_ResetPID();
	Line_PID_Run_Mask(line_mask, LINE_BASE_PWM);
}

void Line_RunUturnRecovery(unsigned char line_mask)
{
	unsigned char valid_position;
	int error;
	int derivative;
	int correction;
	int left_pwm;
	int right_pwm;

	valid_position = (line_mask != 0) && (line_mask != LINE_ALL_MASK);
	if(valid_position)
	{
		error = Line_GetError_ByMask(line_mask) * PIVOT_PID_ERROR_SCALE;
		derivative = error - uturn_pid_last_error;
		uturn_pid_last_error = error;
		uturn_lost_ticks = 0;

		if((error == 0) && Line_IsNarrowCenter(line_mask))
		{
			if(uturn_settle_ticks < UTURN_SETTLE_TICKS)
			{
				uturn_settle_ticks++;
			}
		}
		else
		{
			uturn_settle_ticks = 0;
		}
	}
	else
	{
		error = uturn_pid_last_error;
		derivative = 0;
		uturn_settle_ticks = 0;
		if(uturn_lost_ticks < UTURN_LOST_TICKS)
		{
			uturn_lost_ticks++;
		}

		if(uturn_lost_ticks >= UTURN_LOST_TICKS)
		{
			dead_state = DEAD_UTURN_SEARCH;
			uturn_lost_ticks = 0;
			uturn_settle_ticks = 0;
			Line_RunUturnMotor();
			return;
		}
	}

	if(uturn_settle_ticks >= UTURN_SETTLE_TICKS)
	{
		Line_FinishUturn(line_mask);
		return;
	}

	correction = (PIVOT_PID_KP * error +
	              PIVOT_PID_KD * derivative) / PIVOT_PID_SCALE;
	if(correction > PIVOT_PID_MAX_CORR)
	{
		correction = PIVOT_PID_MAX_CORR;
	}
	if(correction < -PIVOT_PID_MAX_CORR)
	{
		correction = -PIVOT_PID_MAX_CORR;
	}

	left_pwm = PIVOT_PID_BASE_PWM + correction;
	right_pwm = PIVOT_PID_BASE_PWM - correction;
	PWM_Run(left_pwm, right_pwm);
}

void Line_RunDeadState(unsigned char line_mask)
{
	if(dead_state == DEAD_FAULT)
	{
		PWM_Run(0, 0);
		return;
	}

	if(dead_state == DEAD_LOST_CONFIRM)
	{
		if(line_mask == 0)
		{
			if(dead_lost_ticks < ALL_WHITE_CONFIRM_TICKS)
			{
				dead_lost_ticks++;
			}
		}
		else
		{
			Line_ClearDeadState();
			Line_RunRouteFallback(line_mask);
			return;
		}

		if(dead_lost_ticks >= ALL_WHITE_CONFIRM_TICKS)
		{
			Line_StartUturn();
			return;
		}

		PWM_Run(LINE_BASE_PWM, LINE_BASE_PWM);
		return;
	}

	if(dead_state == DEAD_UTURN_SEARCH)
	{
		if((uturn_ticks >= UTURN_MIN_TICKS) &&
		   ((line_mask & UTURN_REACQUIRE_MASK) != 0))
		{
			Line_FinishUturn(line_mask);
			return;
		}

		Line_RunUturnMotor();
		uturn_ticks++;
		return;
	}

	if(dead_state == DEAD_UTURN_RECOVER)
	{
		Line_RunUturnRecovery(line_mask);
		if(dead_state == DEAD_UTURN_RECOVER ||
		   dead_state == DEAD_UTURN_SEARCH)
		{
			uturn_ticks++;
		}
	}
}

void Line_UpdateReturnTracking(unsigned char line_mask)
{
	if(line_route_state != LINE_ROUTE_RETURN_FROM_DEAD)
	{
		return;
	}

	if(return_ticks < 1000)
	{
		return_ticks++;
	}

	if(Line_IsNarrowCenter(line_mask))
	{
		if(return_center_ticks < RETURN_CENTER_TICKS)
		{
			return_center_ticks++;
		}
	}
	else
	{
		return_center_ticks = 0;
	}

	if((return_ticks >= RETURN_ARM_TICKS) &&
	   (return_center_ticks >= RETURN_CENTER_TICKS))
	{
		return_armed = 1;
	}
}

void Line_ClearReturnCandidate(void)
{
	return_state = RETURN_NONE;
	return_candidate_ticks = 0;
	return_left_age = RETURN_EVIDENCE_INVALID;
	return_right_age = RETURN_EVIDENCE_INVALID;
	return_pair_ticks = 0;
	return_pair_confirmed = 0;
}

unsigned char Line_IsReturnJunctionCandidate(unsigned char line_mask)
{
	if((line_route_state != LINE_ROUTE_RETURN_FROM_DEAD) ||
	   !return_armed)
	{
		return 0;
	}

	return Line_IsLeftJunctionEvidence(line_mask) ||
	       Line_IsRightJunctionEvidence(line_mask);
}

void Line_RecordReturnEvidence(unsigned char line_mask)
{
	if(Line_IsLeftJunctionEvidence(line_mask))
	{
		return_left_age = 0;
	}
	else if(return_left_age < RETURN_EVIDENCE_INVALID)
	{
		return_left_age++;
	}

	if(Line_IsRightJunctionEvidence(line_mask))
	{
		return_right_age = 0;
	}
	else if(return_right_age < RETURN_EVIDENCE_INVALID)
	{
		return_right_age++;
	}
}

unsigned char Line_HasReturnPair(void)
{
	return (return_left_age <= RETURN_T_PAIR_TICKS) &&
	       (return_right_age <= RETURN_T_PAIR_TICKS);
}

void Line_StartReturnJunction(void)
{
	Line_ClearReturnCandidate();
	return_state = RETURN_CONFIRM;
	Line_ResetPivotCandidate();
	Line_ClearSharpTurn();
	Line_ResetPID();
}

void Line_RunReturnJunction(unsigned char line_mask)
{
	unsigned char left_evidence;
	unsigned char right_evidence;

	left_evidence = Line_IsLeftJunctionEvidence(line_mask);
	right_evidence = Line_IsRightJunctionEvidence(line_mask);
	Line_RecordReturnEvidence(line_mask);
	if(return_candidate_ticks < RETURN_T_MAX_TICKS)
	{
		return_candidate_ticks++;
	}

	if((Line_HasReturnPair() || (return_pair_ticks != 0)) &&
	   (left_evidence || right_evidence))
	{
		if(return_pair_ticks < RETURN_T_STABLE_TICKS)
		{
			return_pair_ticks++;
		}
	}
	else
	{
		return_pair_ticks = 0;
	}

	if(return_pair_ticks >= RETURN_T_STABLE_TICKS)
	{
		return_pair_confirmed = 1;
	}

	if(return_pair_confirmed &&
	   (return_candidate_ticks >= RETURN_T_DECISION_TICKS))
	{
		Line_ClearReturnCandidate();
		line_route_state = LINE_ROUTE_RETURN_PIVOT;
		Line_BlockJunction();
		Line_StartPivotWithPurpose(t_branch_dir,
		                           PIVOT_SEARCH_FORWARD_PWM,
		                           PIVOT_SEARCH_REVERSE_PWM,
		                           PIVOT_PURPOSE_RETURN_MAIN);
		return;
	}

	if(return_candidate_ticks >= RETURN_T_MAX_TICKS)
	{
		Line_ClearReturnCandidate();
		Line_RunRouteFallback(line_mask);
		return;
	}

	PWM_Run(LINE_BASE_PWM, LINE_BASE_PWM);
}

void Line_OnPivotSuccess(unsigned char line_mask)
{
	unsigned char completed_purpose;

	completed_purpose = pivot_purpose;
	Line_ClearPivot();

	if(completed_purpose == PIVOT_PURPOSE_ENTER_T)
	{
		line_route_state = LINE_ROUTE_T_BRANCH;
		Line_ClearDeadState();
		Line_ResetBranchTracking();
		Line_ResetReturnTracking();
	}
	else if(completed_purpose == PIVOT_PURPOSE_RETURN_MAIN)
	{
		line_route_state = LINE_ROUTE_MAIN;
		t_branch_dir = 0;
		Line_ClearDeadState();
		Line_ResetBranchTracking();
		Line_ResetReturnTracking();
		Line_BlockJunction();
	}
	else if(completed_purpose == PIVOT_PURPOSE_NORMAL)
	{
		Line_ClearJunctionState();
	}

	Line_ResetPID();
	Line_PID_Run_Mask(line_mask, LINE_BASE_PWM);
}

void Line_OnPivotTimeout(void)
{
	unsigned char failed_purpose;

	failed_purpose = pivot_purpose;
	Line_ClearPivot();

	if(failed_purpose == PIVOT_PURPOSE_ENTER_T)
	{
		t_branch_dir = 0;
		Line_EnterDeadFault();
		return;
	}

	if(failed_purpose == PIVOT_PURPOSE_RETURN_MAIN)
	{
		Line_EnterDeadFault();
		return;
	}

	PWM_Run(0, 0);
}

void Line_StartSharpTurn(signed char dir)
{
	turn_dir = dir;
	turn_ticks = 1;
	turn_exit_ticks = 0;
	turn_confirm_ticks = 0;
	Line_ResetPID();
}

void Line_RunSharpTurn(unsigned char line_mask)
{
	if(turn_ticks >= TURN_MAX_TICKS)
	{
		Line_ClearSharpTurn();
		PWM_Run(0, 0);
		return;
	}

	if((turn_ticks >= TURN_MIN_TICKS) && (line_mask & LINE_ZHONG_MASK))
	{
		turn_exit_ticks++;
		if(turn_exit_ticks >= TURN_EXIT_TICKS)
		{
			Line_ClearSharpTurn();
			Line_PID_Run_Mask(line_mask, LINE_BASE_PWM);
			return;
		}
	}
	else
	{
		turn_exit_ticks = 0;
	}

	if(turn_dir == TURN_LEFT)
	{
		PWM_Run(-TURN_INNER_PWM, TURN_OUTER_PWM);
	}
	else
	{
		PWM_Run(TURN_OUTER_PWM, -TURN_INNER_PWM);
	}

	turn_ticks++;
}

void Timer2_ISR_Handler (void) interrupt TMR2_VECTOR
{
	unsigned char line_mask;
	signed char pivot_candidate;

	if(test) return;

	if(++pid_div < 20) return;
	pid_div = 0;

	line_mask = Line_GetMask();

	if(dead_state != DEAD_NONE)
	{
		Line_RunDeadState(line_mask);
		return;
	}

	if(pivot_dir != 0)
	{
		if(Line_TryCrossInterruptPivot(line_mask))
		{
			return;
		}

		Line_RunPivotState(line_mask);
		return;
	}

	if(line_route_state == LINE_ROUTE_T_BRANCH)
	{
		Line_UpdateBranchTracking(line_mask);
	}

	if(line_route_state == LINE_ROUTE_RETURN_FROM_DEAD)
	{
		Line_UpdateReturnTracking(line_mask);

		if(return_state != RETURN_NONE)
		{
			Line_RunReturnJunction(line_mask);
			return;
		}

		if(Line_IsReturnJunctionCandidate(line_mask))
		{
			Line_StartReturnJunction();
			Line_RunReturnJunction(line_mask);
			return;
		}
	}

	if(junc_state != JUNC_NONE)
	{
		Line_RunJunction(line_mask);
		return;
	}

	Line_UpdateJunctionRearm(line_mask);
	if(Line_UpdateCrossBackground(line_mask))
	{
		return;
	}

	if(line_mask == LINE_ALL_MASK)
	{
		Line_HandleAllBlack();
		return;
	}

	if(line_mask == 0)
	{
		Line_ClearJunctionState();
		Line_StartDeadLostConfirm();
		Line_RunDeadState(line_mask);
		return;
	}

	pivot_candidate = Line_GetPivotCandidate(line_mask);
	if(pivot_candidate != 0)
	{
		if(pivot_pending_dir == pivot_candidate)
		{
			if(pivot_confirm_ticks < PIVOT_CONFIRM_TICKS)
			{
				pivot_confirm_ticks++;
			}
		}
		else
		{
			pivot_pending_dir = pivot_candidate;
			pivot_confirm_ticks = 1;
		}

		Line_RecordCenterError(line_mask);
		if(pivot_confirm_ticks >= PIVOT_CONFIRM_TICKS)
		{
			Line_ResetTCandidate();
			Line_StartPivot(pivot_candidate, PIVOT_SEARCH_FORWARD_PWM, PIVOT_SEARCH_REVERSE_PWM);
			return;
		}

		if(pivot_candidate == TURN_RIGHT)
		{
			PWM_Run(LINE_DEPART_FAST_PWM, LINE_DEPART_SLOW_PWM);
		}
		else
		{
			PWM_Run(LINE_DEPART_SLOW_PWM, LINE_DEPART_FAST_PWM);
		}
		return;
	}
	Line_ResetPivotCandidate();

	if(Line_UpdateTJunctionBackground(line_mask))
	{
		return;
	}

	if((line_mask & LINE_ZHONG_MASK) && (line_mask & LINE_ZUO1_MASK) && !(line_mask & LINE_YOU1_MASK))
	{
		Line_RecordCenterError(line_mask);
		PWM_Run(LINE_BASE_PWM-20, LINE_BASE_PWM);
		return;
	}

	if((line_mask & LINE_ZHONG_MASK) && (line_mask & LINE_YOU1_MASK) && !(line_mask & LINE_ZUO1_MASK))
	{
		Line_RecordCenterError(line_mask);
		PWM_Run(LINE_BASE_PWM, LINE_BASE_PWM-20);
		return;
	}

	if((line_mask & LINE_ZHONG_MASK) || ((line_mask & (LINE_ZUO1_MASK | LINE_YOU1_MASK)) == (LINE_ZUO1_MASK | LINE_YOU1_MASK)))
	{
		Line_PID_Run_Mask(line_mask, LINE_BASE_PWM);
		return;
	}

	if(line_mask == LINE_YOU1_MASK)
	{
		Line_ClearSharpTurn();
		Line_RecordCenterError(line_mask);
		PWM_Run(LINE_BASE_PWM, LINE_BASE_PWM-15);
		return;
	}
	if(line_mask == LINE_ZUO1_MASK)
	{
		Line_ClearSharpTurn();
		Line_RecordCenterError(line_mask);
		PWM_Run(LINE_BASE_PWM-15, LINE_BASE_PWM);
		return;
	}

	Line_PID_Run_Mask(line_mask, LINE_BASE_PWM);
}

/******************** ??????**************************/
void main(void)
{
//	unsigned char i;
	WTST = 0;		//????????????????????????0???CPU??????????????????
	EAXSFR();		//???SFR(XFR)???????
	CKCON = 0;      //??????XRAM???

	P2M1=0x00;
	P2M0=0xFF;
	P2M0 &= ~(1 << 2);  // M0=0		p2.2
	P2M0 &= ~(1 << 3);  // M0=0		p2.3
	P2M0 &= ~(1 << 4);  // M0=0		p2.4	
	P0M1=0x00;
	P0M0=0x00;
	P1M1=0x00;
	P1M0=0x00;
	P4M1=0x00;
	P4M0=0xFF;	
	P1M1 |= (1 << 4);   // M1=1
	P1M0 &= ~(1 << 4);  // M0=0	
	
	Timer_config();
	Timer1_config();
	Timer2_config();
	PWM_config();
	oled_Init();
	
	EA = 1;
	
	AIN1=0;
	AIN2=1;
	BIN1=1;
	BIN2=0;

	TRIG = 0;
	ECHO = 0;
	time_us = 0;
	if(test)
	{
		while(1)
		{
			Motor_Reverse_Test();
		}
	}

	while (1)
	{
//			unsigned int d = Ultrasonic_GetDistance();
//			oled_clear();
//			oled_ShowNum(56,2,d,5,16);
//			oled_ShowNum(10,2,1,2,16);		
//			delay_ms(300);	

//		if(KEY4 == 0)
//		{
//			LED1=0;
//			delay_ms(1000);
//			LED1=1;
//			delay_ms(1000);
//		}
//		if(KEY5 == 0)
//		{
//			LED2=0;
//			delay_ms(1000);
//			LED2=1;
//			delay_ms(1000);
//		}
//		if(KEY3 == 0)
//		{
//			LED3=0;
//			delay_ms(1000);
//			LED3=1;
//			delay_ms(1000);
//		}
//		for(i=10;i>0;i--)
//		{
//			oled_clear();
//			oled_ShowNum(10,2,i,2,16);		
//			delay_ms(1000);
//		}



//		if(T0_1ms)
//		{
//			
//			PWMB_Duty.PWM5_Duty = 2047;
//			PWMB_Duty.PWM6_Duty = 1;
//			T0_1ms = 0;
			
//			if(!PWM5_Flag)
//			{
//				PWMB_Duty.PWM5_Duty++;
//				if(PWMB_Duty.PWM5_Duty >= 2047) PWM5_Flag = 1;
//			}
//			else
//			{
//				PWMB_Duty.PWM5_Duty--;
//				if(PWMB_Duty.PWM5_Duty <= 0) PWM5_Flag = 0;
//			}
//			if(!PWM6_Flag)
//			{
//				PWMB_Duty.PWM6_Duty++;
//				if(PWMB_Duty.PWM6_Duty >= 2047) PWM6_Flag = 1;
//			}
//			else
//			{
//				PWMB_Duty.PWM6_Duty--;
//				if(PWMB_Duty.PWM6_Duty <= 0) PWM6_Flag = 0;
//			}
			
			
		}
	}




