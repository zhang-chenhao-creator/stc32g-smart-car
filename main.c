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
/* 如果要在程序中使用此代码,请在程序中注明使用了STC的资料及程序            */
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
#include  "pid.h"




/*************	功能说明	**************

高级PWM定时器 PWM5,PWM6,PWM7,PWM8 每个通道都可独立实现PWM输出.

4个通道PWM根据需要设置对应输出口，可通过示波器观察输出的信号.

PWM周期和占空比可以自定义设置，最高可达65535.

下载时, 选择时钟 24MHZ (用户可在"config.h"修改频率).

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
u16 PWM_Period = 2000; // 2000
bit PWM5_Flag;
bit PWM6_Flag;
int last_line_value = 0;
int line_pid_last_error = 0;
unsigned char line_base_speed = 80;
int line_pid_limit = 25;
unsigned char line_pid_tick = 0;
unsigned char line_lost_count = 0;
int line_pid_kp = 20;
int line_pid_kd = 13;
int machine_cnt = 0;

#define LINE_PID_SCALE 10

/*
 * 巡线状态机业务约定：
 * LINE      单传感器识别黑线，交给普通 PID 巡线。
 * JUNCTION  中线、左侧、右侧同时识别，按十字路口处理，只直行不转弯。
 * TURN      多传感器但不是十字，统一按 T 字、直角、普通转弯处理。
 * SEARCH    五路全白，按上一次转向方向原地旋转找线。
 * LOCKOUT   十字离开后的冷却状态，避免同一个十字区域重复触发。
 */
#define LINE_STATE_LINE 0
#define LINE_STATE_JUNCTION 1
#define LINE_STATE_TURN 2
#define LINE_STATE_SEARCH 3
#define LINE_STATE_LOCKOUT 4

#define TURN_LEFT 0
#define TURN_RIGHT 1
#define TURN_SPEED 85
#define SEARCH_SPEED 85
#define LOCKOUT_LINE_TICKS 20

/*
 * 协作调试重点：
 * junction_state 是当前赛道元素处理状态。
 * last_turn_dir 记住最近一次明确转向，默认右转；全白找线会复用它。
 * junction_tick 当前只记录压在十字上的时间，不参与退出保护。
 * lockout_tick 连续看到单线的次数，达到 LOCKOUT_LINE_TICKS 才退出冷却。
 */
unsigned char junction_state = LINE_STATE_LINE;
unsigned char last_turn_dir = TURN_RIGHT;
unsigned int junction_tick = 0;
unsigned char lockout_tick = 0;


/*************	本地函数声明	**************/



/*************  外部函数和变量声明 *****************/



/************************ IO口配置 ****************************/
void	GPIO_config(void)
{

	
}

/************************ 定时器配置 ****************************/
void	Timer_config(void)
{
	TIM_InitTypeDef		TIM_InitStructure;					//结构定义
	TIM_InitStructure.TIM_Mode      = TIM_16BitAutoReload;	//指定工作模式,   TIM_16BitAutoReload,TIM_16Bit,TIM_8BitAutoReload,TIM_16BitAutoReloadNoMask
	TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_1T;		//指定时钟源,     TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
	TIM_InitStructure.TIM_ClkOut    = DISABLE;				//是否输出高速脉冲, ENABLE或DISABLE
	TIM_InitStructure.TIM_Value     = (u16)(65536UL - (MAIN_Fosc / 1000UL));		//中断频率, 1000次/秒
	TIM_InitStructure.TIM_PS        = 0;					//8位预分频器(n+1), 0~255
	TIM_InitStructure.TIM_Run       = ENABLE;				//是否初始化后启动定时器, ENABLE或DISABLE
	Timer_Inilize(Timer0,&TIM_InitStructure);				//初始化Timer0	  Timer0,Timer1,Timer2,Timer3,Timer4
	NVIC_Timer0_Init(ENABLE,Priority_0);		//中断使能, ENABLE/DISABLE; 优先级(低到高) Priority_0,Priority_1,Priority_2,Priority_3
}

void Timer1_config(void)
{
    TIM_InitTypeDef TIM_InitStructure;

    TIM_InitStructure.TIM_Mode      = TIM_16BitAutoReload;              // 16位自动重装
    TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_1T;                     // 使用1T时钟（最快）
    TIM_InitStructure.TIM_ClkOut    = DISABLE;                          // 不输出高频脉冲
		TIM_InitStructure.TIM_Value 		= (u16)(65536UL - (MAIN_Fosc / 100000UL)); // 10us定时
    TIM_InitStructure.TIM_PS        = 0;                                // 不预分频
    TIM_InitStructure.TIM_Run       = ENABLE;                           // 初始化后立即启动

    Timer_Inilize(Timer1, &TIM_InitStructure);                          // 初始化定时器1
    NVIC_Timer1_Init(ENABLE, Priority_0);                               // 开启中断，设置优先级为0
}

void Timer2_config(void)
{
    TIM_InitTypeDef TIM_InitStructure;

    TIM_InitStructure.TIM_Mode      = TIM_16BitAutoReload;              // 16位自动重装
    TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_1T;                     // 使用1T时钟（最快）
    TIM_InitStructure.TIM_ClkOut    = DISABLE;                          // 不输出高频脉冲
		TIM_InitStructure.TIM_Value 		= (u16)(65536UL - (MAIN_Fosc / 10000UL)); // 100us定时
    TIM_InitStructure.TIM_PS        = 0;                                // 不预分频
    TIM_InitStructure.TIM_Run       = ENABLE;                           // 初始化后立即启动

    Timer_Inilize(Timer2, &TIM_InitStructure);                          // 初始化定时器2
    NVIC_Timer2_Init(ENABLE, Priority_0);                               // 开启中断，设置优先级为0
}

/***************  超声波函数 *****************/
void Timer1_ISR_Handler (void) interrupt TMR1_VECTOR		//进中断时已经清除标志
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
    _nop_(); _nop_(); _nop_(); _nop_();    // 等价大约10us
    _nop_(); _nop_(); _nop_(); _nop_();
    _nop_(); _nop_(); _nop_(); _nop_();
    TRIG = 0;

    // 等待 ECHO 高电平开始
    while (!ECHO);
    
    // 开始计时
    time_us = 0;
    measuring = 1;

    // 等待 ECHO 拉低（结束）
    while (ECHO);

    measuring = 0;
		if(time_us/100<38)									//判断是否小于38毫秒，大于38毫秒的就是超时，直接调到下面返回0
		{
			Distance_mm=(time_us*346)/100;						//计算距离，25°C空气中的音速为346m/s   因为上面的time_end的单位是10微秒，所以要得出单位为毫米的距离结果，还得除以100
		}
    return Distance_mm;
}

/***************  PWM初始化函数 *****************/
void	PWM_config(void)
{
	PWMx_InitDefine		PWMx_InitStructure;
	
//	PWMB_Duty.PWM5_Duty = 128;
//	PWMB_Duty.PWM6_Duty = 256;


	PWMx_InitStructure.PWM_Mode    =	CCMRn_PWM_MODE1;	//模式,		CCMRn_FREEZE,CCMRn_MATCH_VALID,CCMRn_MATCH_INVALID,CCMRn_ROLLOVER,CCMRn_FORCE_INVALID,CCMRn_FORCE_VALID,CCMRn_PWM_MODE1,CCMRn_PWM_MODE2
	PWMx_InitStructure.PWM_Duty    = PWMB_Duty.PWM5_Duty;	//PWM占空比时间, 0~Period
	PWMx_InitStructure.PWM_EnoSelect   = ENO5P;					//输出通道选择,	ENO1P,ENO1N,ENO2P,ENO2N,ENO3P,ENO3N,ENO4P,ENO4N / ENO5P,ENO6P,ENO7P,ENO8P
	PWM_Configuration(PWM5, &PWMx_InitStructure);				//初始化PWM,  PWMA,PWMB

	PWMx_InitStructure.PWM_Mode    =	CCMRn_PWM_MODE1;	//模式,		CCMRn_FREEZE,CCMRn_MATCH_VALID,CCMRn_MATCH_INVALID,CCMRn_ROLLOVER,CCMRn_FORCE_INVALID,CCMRn_FORCE_VALID,CCMRn_PWM_MODE1,CCMRn_PWM_MODE2
	PWMx_InitStructure.PWM_Duty    = PWMB_Duty.PWM6_Duty;	//PWM占空比时间, 0~Period
	PWMx_InitStructure.PWM_EnoSelect   = ENO6P;					//输出通道选择,	ENO1P,ENO1N,ENO2P,ENO2N,ENO3P,ENO3N,ENO4P,ENO4N / ENO5P,ENO6P,ENO7P,ENO8P
	PWM_Configuration(PWM6, &PWMx_InitStructure);				//初始化PWM,  PWMA,PWMB


	PWMx_InitStructure.PWM_Period   = PWM_Period; //2000							//周期时间,   0~65535
	PWMx_InitStructure.PWM_DeadTime = 0;								//死区发生器设置, 0~255
	PWMx_InitStructure.PWM_MainOutEnable= ENABLE;				//主输出使能, ENABLE,DISABLE
	PWMx_InitStructure.PWM_CEN_Enable   = ENABLE;				//使能计数器, ENABLE,DISABLE
	PWM_Configuration(PWMB, &PWMx_InitStructure);				//初始化PWM通用寄存器,  PWMA,PWMB


		PWM6_USE_P21();
		PWM5_USE_P20();
	NVIC_PWM_Init(PWMB,DISABLE,Priority_0);
}

void PWM_Left(int pwm)     //用0~100表示
{
	
	PWMB_Duty.PWM6_Duty = pwm*(PWM_Period/100);  //注意PWM_Period此时为100倍数 2000
	
}

void PWM_Right(int pwm)     //用0~100表示
{
	
	PWMB_Duty.PWM5_Duty = pwm*(PWM_Period/100);
	
}


void PWM_Run(int pwm1,int pwm2)  //左右PWM
{
	if(pwm1<0)
	{
		AIN1=1;
		AIN2=0;
		pwm1=-pwm1;
	}
	else
	{
		AIN1=0;
		AIN2=1;
	}
	if(pwm2<0)
	{
		BIN1=0;
		BIN2=1;
		pwm2=-pwm2;
	}
	else
	{
		BIN1=1;
		BIN2=0;
	}
	PWM_Left(pwm1);
	PWM_Right(pwm2);
	UpdatePwm(PWMB, &PWMB_Duty);
}

void test01(void)
{
		PWM_Run(100,80);
		delay_ms(1000);
		PWM_Run(80,100);
		delay_ms(1000);
		PWM_Run(0,100);
		delay_ms(1000);
		PWM_Run(100,0);
		delay_ms(1000);
}


/*
 * 普通巡线 PID：LINE 状态和 LOCKOUT 状态复用这一套控制。
 * line_sum 左侧为负、右侧为正；line_error 取反后，线偏右会让左轮更快、右轮更慢。
 */
void RunLinePid(unsigned char line_mask, int line_sum, unsigned char line_count)
{
		int line_value;
		int line_error;
		int pid_output;
		int left_pwm;
		int right_pwm;

		if(line_mask == 0 || line_count == 0)
		{
				return;
		}

		/* 多个传感器同时识别时取平均位置，作为 PID 的线位置输入。 */
		line_value = line_sum / line_count;
		last_line_value = line_value;

		line_error = -line_value;
		pid_output = ((line_pid_kp * line_error) + (line_pid_kd * (line_error - line_pid_last_error))) / LINE_PID_SCALE;
		line_pid_last_error = line_error;

		if(pid_output > line_pid_limit)
		{
				pid_output = line_pid_limit;
		}
		else if(pid_output < -line_pid_limit)
		{
				pid_output = -line_pid_limit;
		}

		/* pid_output 为正时向右修，左轮加速、右轮减速；为负时向左修。 */
		left_pwm = (int)line_base_speed - pid_output;
		right_pwm = (int)line_base_speed + pid_output;

		if(left_pwm > 100 || left_pwm < -100)
		{
				left_pwm = 100 * (left_pwm > 0 ? 1 : -1);
		}
		if(right_pwm > 100 || right_pwm < -100)
		{
				right_pwm = 100 * (right_pwm > 0 ? 1 : -1);
		}

		PWM_Run(left_pwm, right_pwm);
}
void Timer2_ISR_Handler (void) interrupt TMR2_VECTOR		//进中断时已经清除标志
{
		int line_sum;
		unsigned char line_count;
		unsigned char line_mask;
		unsigned char has_center;
		unsigned char has_left;
		unsigned char has_right;
		unsigned char is_white;
		unsigned char is_line;
		unsigned char is_junction;

		/* Timer2 为 100us 中断；每 10 次执行一次巡线逻辑，实际控制周期约 1ms。 */
		line_pid_tick++;
		if(line_pid_tick < 10)
		{
				return;
		}
		line_pid_tick = 0;

		/*
		 * 5 路传感器压成一个 bitmask，便于统一判断赛道元素：
		 * 0x10 左外 zuo2，0x08 左内 zuo1，0x04 中线 zhong，0x02 右内 you1，0x01 右外 you2。
		 */
		line_sum = 0;
		line_count = 0;
		line_mask = 0;

		if(zuo2 == 1)
		{
				line_mask |= 0x10;
				line_sum += -20;
				line_count++;
		}
		if(zuo1 == 1)
		{
				line_mask |= 0x08;
				line_sum += -10;
				line_count++;
		}
		if(zhong == 1)
		{
				line_mask |= 0x04;
				line_sum += 0;
				line_count++;
		}
		if(you1 == 1)
		{
				line_mask |= 0x02;
				line_sum += 10;
				line_count++;
		}
		if(you2 == 1)
		{
				line_mask |= 0x01;
				line_sum += 20;
				line_count++;
		}

		/* 基础业务判定：全白、单线、十字。剩余多个传感器组合统一交给 TURN。 */
		has_center = (line_mask & 0x04) ? 1 : 0;
		has_left = (line_mask & 0x18) ? 1 : 0;
		has_right = (line_mask & 0x03) ? 1 : 0;
		is_white = (line_mask == 0) ? 1 : 0;
		is_line = (line_mask == 0x10 || line_mask == 0x08 || line_mask == 0x04 || line_mask == 0x02 || line_mask == 0x01) ? 1 : 0;
		is_junction = (has_center && has_left && has_right) ? 1 : 0;

		/* 状态转移：先按当前传感器组合决定下一步状态，再在下面统一输出电机动作。 */
		if(is_white)
		{
				junction_state = LINE_STATE_SEARCH;
				if(line_lost_count < 250)
				{
						line_lost_count++;
				}
		}
		else
		{
				line_lost_count = 0;

				/* 十字期间持续直行；只要离开十字组合，就进入 LOCKOUT 防止重复识别。 */
				if(junction_state == LINE_STATE_JUNCTION)
				{
						if(is_junction)
						{
								if(junction_tick < 65535)
								{
										junction_tick++;
								}
						}
						else
						{
								junction_state = LINE_STATE_LOCKOUT;
								junction_tick = 0;
								lockout_tick = 0;
						}
				}
				/* LOCKOUT 内不重新触发十字，必须连续稳定看到单线后才恢复普通巡线。 */
				else if(junction_state == LINE_STATE_LOCKOUT)
				{
						if(is_line)
						{
								if(lockout_tick < 250)
								{
										lockout_tick++;
								}
								if(lockout_tick >= LOCKOUT_LINE_TICKS)
								{
										junction_state = LINE_STATE_LINE;
										lockout_tick = 0;
								}
						}
						else
						{
								lockout_tick = 0;
						}
				}
				else if(is_junction)
				{
						junction_state = LINE_STATE_JUNCTION;
						junction_tick = 0;
				}
				else if(is_line)
				{
						junction_state = LINE_STATE_LINE;
				}
				else
				{
						/* 多传感器非十字：只有单侧明确有线时更新方向；左右都有时沿用上次方向。 */
						if(has_left && !has_right)
						{
								last_turn_dir = TURN_LEFT;
						}
						else if(has_right && !has_left)
						{
								last_turn_dir = TURN_RIGHT;
						}
						junction_state = LINE_STATE_TURN;
				}
		}

		/* 状态动作：每个有效控制周期只输出一次 PWM。 */
		if(junction_state == LINE_STATE_SEARCH)
		{
				if(last_turn_dir == TURN_LEFT)
				{
						PWM_Run(-SEARCH_SPEED, SEARCH_SPEED);
				}
				else
				{
						PWM_Run(SEARCH_SPEED, -SEARCH_SPEED);
				}
		}
		else if(junction_state == LINE_STATE_JUNCTION)
		{
				PWM_Run(line_base_speed, line_base_speed);
		}
		else if(junction_state == LINE_STATE_TURN)
		{
				if(last_turn_dir == TURN_LEFT)
				{
						PWM_Run(-TURN_SPEED, TURN_SPEED);
				}
				else
				{
						PWM_Run(TURN_SPEED, -TURN_SPEED);
				}
		}
		else
		{
				RunLinePid(line_mask, line_sum, line_count);
		}
}
/******************** 主函数**************************/
void main(void)
{
//	unsigned char i;
	WTST = 0;		//设置程序指令延时参数，赋值为0可将CPU执行指令的速度设置为最快
	EAXSFR();		//扩展SFR(XFR)访问使能 
	CKCON = 0;      //提高访问XRAM速度

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
	// oled_Init();

	
	EA = 1;
	
	AIN1=0;
	AIN2=1;
	BIN1=1;
	BIN2=0;

	TRIG = 0;
	ECHO = 0;
	time_us = 0;
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




