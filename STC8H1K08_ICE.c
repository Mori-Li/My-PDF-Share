
/*---------------------------------------------------------------------*/
/* ICE风扇,STC8H1K28原方案*/				/*需要改成STC8H1K08*/
/*	引脚分布
		Back EMFADC== P3.6 			|		Back EMFADC == P3.6
			BMF_C 6  == P1.6			|			BMF_W 13  == P3.5	
			BMF_B 9  == P0,1			|			BMF_V 12  == P3.4	
			BMF_A 8  == P0,0			|			BMF_U 11  == P3.3
			Sw ADC7  == P1.7			| 		Sw ADC10  == P3.2
			
			C_PWM3L	 == P1.5			|			W_PWM4N	 == P1.7
			C_PWM3P	 == P1.4			|			W_PWM4P	 == P1.6
			B_PWM2L	 == P1.3			|			V_PWM3N	 == P1.5
			B_PWM2P	 == P1.2			|			V_PWM3P	 == P1.4
			A_PWM1L	 == P1.1			|			U_PWM2N	 == P1.3
			A_PWM1P	 == P1.0			|			U_PWM2P	 == P1.2
			
*/
/*---------------------------------------------------------------------*/

/*************	功能说明	**************

本程序试验使用STC8H1K08-TSSOP20来驱动无传感器无刷三相直流电机.

P1.7 ADC7接的电位器用于设定PWM占空比, 顺时针旋转电位器占空比增加，电机速度升高.
电位器ADC读数转换成0~1023，对应设定占空比0~1023/1024，

******************************************/

#define FOSC		24000000L	//定义主时钟

#include	"STC8Hxxx.h"
#include	"S-Curve.h"
#include	"svpwm-LS.h"

// 电机参数调整：先大致确定参数，然后启动，能启动后，占空比调整到启动占空比，看此时的转速，则启动终了转速比此时转速略小(60~80%)即可。
// 对于惯量较大的负载，小占空比启动，需要更长的启动时间才能强拖到指定的终了速度。过0检测噪声较大时(特别影响低速)，比较器设置回差电压10mV、20mV或30mV。
// 反电动势检测电路使用合适的RC值能滤除干扰，又不会导致明显的相位延时，一般RC值在10us以下。


#define		MOTOR_TYPE	0	//0:2212电机，1：8元风扇单机，2:12万转涵道风扇电机，3:28元全金属涵道风扇


#if	(MOTOR_TYPE == 0)					// 由于退磁时间很短(远小于PWM周期)，则PWM互补输出噪声更低，效率更高。12V供电带大疆9450桨，PWM=312，单端输出0.622A、2846rpm，互补输出0.579A、2902rpm。
	// 1503电机
	#define	MPP					7		// 电机磁极对, Magnetic Pole Pair，8元风扇电机为5对磁极
	#define	ROLL_MinSpeed		200		// 启动时检测到低于这个转速则认为电机停止，从静止开始启动，否则直接进入闭环
	#define	PRE_PWM_DUTY		80		// 预定位占空比, 一般为START_PWM_DUTY的0.5~1，较小的占空比冲击小，但启动力矩小。
	#define	START_PWM_DUTY	100 	// 启动时的占空比
	#define	STOP_PWM_DUTY		50		// 停止PWM，小于此PWM就停机， 根据电机特性适当改变
	#define	BeginSpeed			100		// 启动起始转速，转/分
	#define	FinishSpeed			1000		// 启动终了转速，转/分
	#define	D_StartTime			1000	// 加速时间，单位ms，不能小于500, 500~10922
	#define	PRE_STATE_TIME1		50		// 预定位时间1，ms, 最大值为2849ms
	#define	PRE_STATE_TIME2		35		// 预定位时间2，ms, 最大值为2849ms
	#define	PRE_STATE_TIME3		20		// 预定位时间3，ms, 最大值为2849ms

#endif

#define CONTEMPRATY 	1 		//  1:使用互补PWM， 0:使用单端PWM

/*改改*/
//sbit U_PWM2P = P1^0;		//原K28参数
//sbit U_PWM2N = P1^1;
//sbit V_PWM3P = P1^2;
//sbit V_PWM3N = P1^3;
//sbit W_PWM4P = P1^4;
//sbit W_PWM4N = P1^5;
sbit U_PWM2P = P1^2;			//K08参数
sbit U_PWM2N = P1^3;
sbit V_PWM3P = P1^4;
sbit V_PWM3N = P1^5;
sbit W_PWM4P = P1^6;
sbit W_PWM4N = P1^7;

/*改改*/
//sbit P_LED  = P2^3;	//LED指示灯
//sbit P_TEST = P3^7;	//测试脚
//sbit P_PPM  = P3^3;	//测试脚
sbit LED_S  	= P3^7;	//LED指示灯
//sbit TP2 		 	= P1^0;	//测试脚
//sbit TP1  		= P1^1;	//测试脚


typedef enum
{
	MOTOR_IDLE = 0,		//停止、空闲
	ROLL_CHECK,			//转动检测
	PRE_POSITION,		//预定位
	STARTING,			//启动
	MOTOR_RUN			//运行, 这个不能有逗号
} runFlagType;
runFlagType run_mode;

u8	fs_cnt = 0;			// 启动电机处理频率，对PWM周期计数，得到大约1000Hz的频率
u32	roll_N    = 0;		// 电机速度递增步进
u16	roll_sum  = 0;		// 调速累加器
u16	SartCnt = 0;		// 启动处理次数(采样周期)计数
u16	StartTime;			// 启动时间, 用户指定启动时间折算成采样周期(大约1ms)的个数，
u16 angle   = 0;		//当前转子电角度
u16 Ua, Ub, Uc;			//3 相PWM
u8	RollCheckIndex;		//转动检测索引
u16	RollPahseTimeUV;	//转动换相检测时间


u8	cmpo_state;		//PWMA中断-比较器连续8个输出状态
bit	B_pre_cmpo;		//上一个比较器输出电平
bit	B_cmpo;			//比较器有效输出电平（CMPO 8连1则为，8连0则为0）

u8	step;		//切换步骤
u16	PWM_Value;	//PWM占空比的值
u16	PWW_Set;	//目标PWM设置
bit	B_1ms;		//1ms定时标志
bit	B_4ms;		//4ms定时标志
u8	cnt_4ms;
u8	cnt_500ms;
bit	B_500ms;

u16	Sw_ADC10;				//Switch --->> Mori 单刀 P3.2  梁工原adc11
u16	Vbat;			//电池电压
u16	Current;		//电流

u8	TimeOut;	//堵转超时
bit	B_Timer3_OverFlow;

u8	PhaseTimeIndex;		//换相时间保存索引
u16	PhaseTimeTmp[4];	//4个换相时间缓存
u16	PhaseTime4sum;		//4次换相时间累加和, 用于中间计算
u16	PhaseTime;			//换相时间
u8	XiaoCiCnt;			//1:需要消磁(30度角延时), 2:正在消磁, 0已经消磁


u32	PhaseTimeSum;	//换相时间累加和，计算速度累加换相时间
u16	PhaseSumCnt;	//换相累加次数，用于计算转速
u32	RollSpeed;		//转速


/*************************/

void  delay_ms(unsigned int ms)
{
	unsigned int i;
	do
	{
		i = (FOSC+5000) / 10000UL;
		while(--i)	;
	}while(--ms);
}


void delay_us(u8 us)	//N us延时函数
{
	do
	{
		NOP(17);	//@24MHz
	}
	while(--us);
}

void  delay_N_10us(unsigned int j)
{
	unsigned char i;
	do
	{
		i = (FOSC + 150000UL) / 300000UL -4;
		_nop_();
		_nop_();
		while(--i)	;
	}while(--j);
}


void	Delay_500ns(void)
{
	NOP(6);
}


/************************ 比较器初始化函数 ****************************/
// 截止2025-6-2，旧版型号：STC8H1K08系列、STC8H1K28系列、STC8H3K64S2系列、STC8H3K64S4系列。
// 旧版的CMP+只能切换P3.7或ADC输入，不支持寄存器CMPEXCFG。
void	CMP_config(void)
{
	P_SW2 |= 0x80;		//SFR enable
	CMPCR1 = 0;
	CMPCR2 = 60;		//比较结果变化延时周期数, 0~63

	CMPCR1 |= (1<<7);	//1: 允许比较器,     0:关闭比较器
	CMPCR1 |= (0<<5);	//1: 允许上升沿中断, 0: 禁止
	CMPCR1 |= (0<<4);	//1: 允许下降沿中断, 0: 禁止
	CMPCR1 |= (1<<3);	//输入正极性选择, 0: 选择外部P3.7做正输入,    1: 由ADC_CHS[3:0]所选择的ADC输入端做正输入.
	CMPCR1 |= (1<<2);	//输入负极性选择, 0: 选择内部BandGap电压BGv做负输入, 1: 选择外部P3.6做输入
	CMPCR1 |= (0<<1);	//1: 允许比较结果输出到IO(P3.4或P4.1),  0: 比较结果禁止输出到IO

	CMPCR2 |= (0<<7);	//1: 比较器结果输出IO取反, 0: 不取反
	CMPCR2 |= (0<<6);	//0: 允许内部0.1uF滤波,    1: 关闭

	P3n_pure_input(0x7c);	//设置P3.6-CMP-为高阻输入
												//将输入的ADC通道设置为高阻,  P3.3--BMF_U, ，
																							//			P3.4--BMF_V
																								//		P3.5--BMF_W,
																									//	P3.2--Sw_adc
	ADC_CONTR = 0x80 + 8;	//打开ADC，选择ADC通道0~14
}



#define ADC_START	(1<<6)	/* 自动清0 */
#define ADC_FLAG	(1<<5)	/* 软件清0 */

#define	ADC_SPEED	3		/* 0~15, ADC时钟 = SYSclk/2/(n+1) */
#define	RES_FMT		(1<<5)	/* ADC结果格式 0: 左对齐, ADC_RES: D9 D8 D7 D6 D5 D4 D3 D2, ADC_RESL: D1 D0 0  0  0  0  0  0 */
							/*             1: 右对齐, ADC_RES: 0  0  0  0  0  0  D9 D8, ADC_RESL: D7 D6 D5 D4 D3 D2 D1 D0 */

#define CSSETUP		(1<<7)	/* 0~1,  ADC通道选择时间      0: 1个ADC时钟, 1: 2个ADC时钟,  默认0(默认1个ADC时钟) */
#define CSHOLD		(1<<5)	/* 0~3,  ADC通道选择保持时间  (n+1)个ADC时钟, 默认1(默认2个ADC时钟)                */
#define SMPDUTY		11		/* 10~31, ADC模拟信号采样时间  (n+1)个ADC时钟, 默认10(默认11个ADC时钟)				*/
							/* ADC转换时间: 10位ADC固定为10个ADC时钟, 12位ADC固定为12个ADC时钟. 				*/

void ADC_config(void)	//ADC初始化函数(为了使用ADC输入端做比较器信号, 实际没有启动ADC转换)
{
	P_SW2 |=  0x80;	//访问XSFR
	P3n_pure_input(Pin2);	//设置为高阻输入, P3.2--ADC10--滑动变阻器VR
	ADC_CONTR = 0x80 + 10;	//ADC on + ADC channel=10=P3.2
	ADCCFG = RES_FMT + ADC_SPEED;
	ADCTIM = CSSETUP + CSHOLD + SMPDUTY;
	EADC = 0;			//禁止ADC中断
}


//========================================================================
// 函数: u16	Get_ADC10bitResult(u8 channel))	//channel = 0~15
//========================================================================
u16	Get_ADC10bitResult(u8 channel)	//channel = 0~15
{
	u8 i;
	ADC_RES = 0;
	ADC_RESL = 0;
	ADC_CONTR = 0x80 | ADC_START | channel;
	NOP(5);			//
//	while((ADC_CONTR & ADC_FLAG) == 0)	;	//等待ADC结束
		i = 255;
		while(i != 0)	//超时处理
		{
			i--;
			if((ADC_CONTR & ADC_FLAG) != 0)	break;	//等待ADC结束
		}
	ADC_CONTR &= ~ADC_FLAG;
	return	((u16)ADC_RES * 256 + (u16)ADC_RESL);
}



/*******************************
UV表示A加载PWM，B接GND，C↓表示W相反电动势下降沿，C↑表示W相反电动势上升沿。两个转向的时序是，换相顺序相反，反电动势相反。
拍序   逆时针    
 0     UV, W↓              A
 1     UW, V↑            B   C
 2     VW, U↓    
 3     VU, W↑    
 4     WU, V↓    
 5     WV, U↑    

*******************************/


#define 	BMF_U 	11		// ADC通道11--P3.3
#define 	BMF_V 	12	  // ADC通道12--P3.4
#define 	BMF_W 	13 		// ADC通道13--P3.5		 似乎要改两处 1 CMP高阻设置 : 2 ADC_CONRT 寄存器值

void StepMotor(void) // 换相序列函数
{
	if(++step >= 6)	step = 0;
//P33 = ~P33;	//换相指示

	switch(step)
	{
	case 0:  // UV  U_PWM2P=pwm, PWM3_N=1
			PWMA_ENO = 0x00;	U_PWM2N=0;	W_PWM4N=0;
			Delay_500ns();
			PWMA_ENO = 0x04+0x08*CONTEMPRATY;					// 打开U相的高端PWM,	PWMA_ENO输出开启寄存器,U相PWM2P对应 0x04+0x08 
			V_PWM3N = 1;			// 打开V相的低端
			ADC_CONTR = 0x80+BMF_W;	// 选择P3.5作为ADC输入 即W相电压
			break;
	case 1:  // UW  U_PWM2P=pwm, PWM4_N=1
			PWMA_ENO = 0x00;	U_PWM2N=0;	V_PWM3N=0;	
			Delay_500ns();
			PWMA_ENO = 0x04+0x08*CONTEMPRATY;					// 打开U相的高端PWM
			W_PWM4N = 1;			// 打开W相的低端
			ADC_CONTR = 0x80+BMF_V;	// 选择P3.4作为ADC输入 即V相电压
			break;
	case 2:  // VW  V_PWM3P=pwm, PWM4_N=1
			PWMA_ENO = 0x00;	U_PWM2N=0;	V_PWM3N=0;
			Delay_500ns();
			PWMA_ENO = 0x10+0x20*CONTEMPRATY;					// 打开V相的高端PWM
			W_PWM4N = 1;			// 打开W相的低端
			Sw_ADC10  = ((Sw_ADC10  *7)>>3) + Get_ADC10bitResult(10);	//低通滤波结果为13位,   6: ADC8(电流), 7：ADC9(电压)，11：ADC11(电位器)
			ADC_CONTR = 0x80+BMF_U;	// 选择P3.3作为ADC输入 即U相电压
			break;
	case 3:  // VU  V_PWM3P=pwm, PWM2_N=1
			PWMA_ENO = 0x00;	V_PWM3N=0;	W_PWM4N=0;	
			Delay_500ns();
			PWMA_ENO = 0x10+0x20*CONTEMPRATY;					// 打开V相的高端PWM
			U_PWM2N = 1;			// 打开U相的低端
			ADC_CONTR = 0x80+BMF_W;	// 选择P3.5作为ADC输入 即W相电压
			break;
	case 4:  // WU  W_PWM4P=pwm, PWM2_N=1
			PWMA_ENO = 0x00;	V_PWM3N=0;	W_PWM4N=0;
			Delay_500ns();
			PWMA_ENO = 0x40+0x80*CONTEMPRATY;					// 打开W相的高端PWM
			U_PWM2N = 1;			// 打开U相的低端
			
			ADC_CONTR = 0x80+BMF_V;	// 选择P3.4作为ADC输入 即V相电压
			break;
	case 5:  // WV  W_PWM4P=pwm, PWM3_N=1
			PWMA_ENO = 0x00;	U_PWM2N=0;	W_PWM4N=0;
			Delay_500ns();
			PWMA_ENO = 0x40+0x80*CONTEMPRATY;					// 打开W相的高端PWM
			V_PWM3N = 1;			// 打开V相的低端
			ADC_CONTR = 0x80+BMF_U;	// 选择P3.3作为ADC输入 即U相电压
			break;

	default:
			break;
	}
}


void PWMA_config(void)
{
	P_SW2 |= 0x80;		//SFR enable

	U_PWM2P = 0;
	U_PWM2N = 0;
	V_PWM3P = 0;
	V_PWM3N = 0;
	W_PWM4P = 0;
	W_PWM4N = 0;
	P1n_push_pull(0x3f);

	PWMA_PSCR = 0;		// 预分频寄存器, 分频 Fck_cnt = Fck_psc/(PSCR[15:0}+1), 边沿对齐PWM频率 = SYSclk/((PSCR+1)*(AAR+1)), 中央对齐PWM频率 = SYSclk/((PSCR+1)*(AAR+1)*2).
	PWMA_DTR  = 12;		// 死区时间配置, n=0~127: DTR= n T,   0x80 ~(0x80+n), n=0~63: DTR=(64+n)*2T,
						//				0xc0 ~(0xc0+n), n=0~31: DTR=(32+n)*8T,   0xE0 ~(0xE0+n), n=0~31: DTR=(32+n)*16T,
	PWMA_ARR    = 1023;	// 自动重装载寄存器,  控制PWM周期
	PWMA_CCER1  = 0;
	PWMA_CCER2  = 0;
	PWMA_SR1    = 0;
	PWMA_SR2    = 0;
	PWMA_ENO    = 0;
	PWMA_PS     = 0;
	PWMA_IER    = 0;
//	PWMA_ISR_En = 0;

//	PWMA_IER   |= 0x01;		// 使能更新中断

/*	PWMA_CCMR1  = 0x68;		// 通道模式配置, PWM模式1, 预装载允许
	PWMA_CCR1   = 0;		// 比较值, 控制占空比(高电平时钟数)
	PWMA_CCER1 |= 0x05;		// 开启比较输出, 高电平有效
	PWMA_PS    |= 0;		// 选择IO, 0:选择P1.0 P1.1, 1:选择P2.0 P2.1, 2:选择P6.0 P6.1,
	PWMA_CCMR1  = 0x68;		// 通道模式配置, PWM模式1, 预装载允许
//	PWMA_ENO   |= 0x01;		// IO输出允许,  bit7: ENO4N, bit6: ENO4P, bit5: ENO3N, bit4: ENO3P,  bit3: ENO2N,  bit2: ENO2P,  bit1: ENO1N,  bit0: ENO1P
//	PWMA_IER   |= 0x02;		// 使能PWM1中断             */

	PWMA_CCMR2  = 0x68;		// 通道模式配置, PWM模式1, 预装载允许
	PWMA_CCR2   = 0;		// 比较值, 控制占空比(高电平时钟数)
	PWMA_CCER1 |= 0x50;		// 开启比较输出, 高电平有效
	PWMA_PS    |= (0<<2);	// 选择IO, 0:选择P1.2 P1.3, 1:选择P2.2 P2.3, 2:选择P6.2 P6.3,
	PWMA_CCMR2  = 0x68;		// 通道模式配置, PWM模式1, 预装载允许
//	PWMA_ENO   |= 0x04;		// IO输出允许,  bit7: ENO4N, bit6: ENO4P, bit5: ENO3N, bit4: ENO3P,  bit3: ENO2N,  bit2: ENO2P,  bit1: ENO1N,  bit0: ENO1P
//	PWMA_IER   |= 0x04;		// 使能PWM2中断

	PWMA_CCMR3  = 0x68;		// 通道模式配置, PWM模式1, 预装载允许
	PWMA_CCR3   = 0;		// 比较值, 控制占空比(高电平时钟数)
	PWMA_CCER2 |= 0x05;		// 开启比较输出, 高电平有效
	PWMA_PS    |= (0<<4);	// 选择IO, 0:选择P1.4 P1.5, 1:选择P2.4 P2.5, 2:选择P6.4 P6.5,
	PWMA_CCMR3  = 0x68;		// 通道模式配置, PWM模式1, 预装载允许
//	PWMA_ENO   |= 0x10;		// IO输出允许,  bit7: ENO4N, bit6: ENO4P, bit5: ENO3N, bit4: ENO3P,  bit3: ENO2N,  bit2: ENO2P,  bit1: ENO1N,  bit0: ENO1P
//	PWMA_IER   |= 0x08;		// 使能PWM3中断

  PWMA_CCMR4  = 0x68;		// 通道模式配置, PWM模式1, 预装载允许
	PWMA_CCR4   = 0;		// 比较值, 控制占空比(高电平时钟数)
  PWMA_CCER2 |= 0x50;		// 开启比较输出, 高电平有效
	PWMA_PS    |= (0<<6);	// 选择IO, 0:选择P1.6 P1.7, 1:选择P2.6 P2.7, 2:选择P6.6 P6.7,
  PWMA_CCMR4  = 0x68;		// 通道模式配置, PWM模式1, 预装载允许
//	PWMA_ENO   |= 0x40;		// IO输出允许,  bit7: ENO4N, bit6: ENO4P, bit5: ENO3N, bit4: ENO3P,  bit3: ENO2N,  bit2: ENO2P,  bit1: ENO1N,  bit0: ENO1P
//	PWMA_IER   |= 0x10;		// 使能PWM4中断

	PWMA_BKR    = 0x80;		// 主输出使能 相当于总开关
	PWMA_CR1    = 0x81;		// 使能计数器, 允许自动重装载寄存器缓冲, 边沿对齐模式, 向上计数,  bit7=1:写自动重装载寄存器缓冲(本周期不会被打扰), =0:直接写自动重装载寄存器本(周期可能会乱掉)
	PWMA_EGR    = 0x01;		//产生一次更新事件, 清除计数器和与分频计数器, 装载预分频寄存器的值
//	PWMA_ISR_En = PWMA_IER;	//设置标志允许通道1~4中断处理
}

//	PWMA_PS   = (0<<6)+(0<<4)+(0<<2)+0;	//选择IO, 4项从高到低(从左到右)对应PWM1 PWM2 PWM3 PWM4, 0:选择P1.x, 1:选择P2.x, 2:选择P6.x,
//  PWMA_PS    PWM4N PWM4P    PWM3N PWM3P    PWM2N PWM2P    PWM1N PWM1P
//    00       P1.7  P1.6     P1.5  P1.4     P1.3  P5.4     P1.1  P1.0
//    01       P2.7  P2.6     P2.5  P2.4     P2.3  P2.2     P2.1  P2.0
//    02       P6.7  P6.6     P6.5  P6.4     P6.3  P6.2     P6.1  P6.0
//    03       P3.3  P3.4      --    --       --    --       --    --



#define		PHASE_TIME_MAX	16380		//最大换相时间, us, 避免累加和溢出

void CMP_ISR(void) interrupt CMP_VECTOR		//比较器中断函数, 检测到反电动势过0事件
{
	u8	i;
	CMPCR1 &= ~(0x40+0x30);	// 软件清除中断标志位, 禁止比较器上升沿、下降沿中断中断

	if(XiaoCiCnt == 0)	//消磁后才检测过0事件,   XiaoCiCnt=1:需要消磁(30度角延时), =2:正在消磁, =0已经消磁
	{
		T4T3M &= ~(1<<3);	// Timer3停止运行
		PhaseTime = ((u16)T3H << 8) + T3L;	//读取Timer3测量到的换相时间(单位1us)
		T3H = 0;	T3L = 0;
		T4T3M |=  (1<<3);	//Timer3开始运行

		if(B_Timer3_OverFlow)	//切换时间间隔(Timer3)有溢出
		{
			B_Timer3_OverFlow = 0;
			PhaseTime = PHASE_TIME_MAX;	//换相时间最大8ms, 2212电机12V空转最高速130us切换一相(200RPS 12000RPM), 480mA
		}
		else if(PhaseTime >= PHASE_TIME_MAX)	PhaseTime = PHASE_TIME_MAX;	//换相时间最大8ms, 2212电机12V空转最高速130us切换一相(200RPS 12000RPM), 480mA

		PhaseTimeTmp[PhaseTimeIndex] = PhaseTime;	//保存本次换相时间
		if(++PhaseTimeIndex >= 4)	PhaseTimeIndex = 0;	//索引溢出归0
		for(PhaseTime4sum=0, i=0; i<4; i++)	PhaseTime4sum += PhaseTimeTmp[i];	//求8次换相时间累加和, 用于计算实时转速 = 1/(M*PhaseTime8sum*3/4) 转/秒 = 60/(M*PhaseTime8sum*3/4) 转/分，PhaseTime8sum*3/4为换相6次的时间，M为磁极对。
		PhaseTime = PhaseTime4sum >> 2;		//求8次换相时间的平均值
		//if(PhaseTime >= 80)	TimeOut = 125;	//堵转500ms超时

		T4T3M &= ~(1<<7);				//Timer4停止运行
		PhaseTime4sum = 0 - PhaseTime;		//Timer4 1us 2个计数
		T4H = (u8)(PhaseTime4sum >> 8);		//装载30度角延时
		T4L = (u8)PhaseTime4sum;
		T4T3M |=  (1<<7);	//Timer4开始运行
		XiaoCiCnt = 1;		//1:需要消磁(30度角延时), 2:正在消磁, 0已经消磁
	}
}

//============================ timer0初始化函数 ============================================
void Timer0_config(void)	//Timer0初始化函数, 用于产生1ms时隙
{
	Timer0_16bitAutoReload(); // T0工作于16位自动重装
	Timer0_1T();
	TH0 = (65536UL-FOSC/1000) / 256; //1ms
	TL0 = (65536UL-FOSC/1000) % 256;
	TR0 = 1; // 打开定时器0

	ET0 = 1;// 允许ET0中断
}
//=========================== timer0中断函数 =============================================
void Timer0_ISR(void) interrupt TIMER0_VECTOR	//Timer0中断函数, 用于产生1ms时隙
{
	B_1ms = 1;	//1ms定时标志
}

//============================ timer3初始化函数 ============================================
void	Timer3_Config(void)	//测量换相时间
{
	T4T3M &= 0xf0;		//停止计数, 定时模式, 12T模式, 不输出时钟
	T3H = 0;
	T3L = 0;
	TM3PS  = 1;			//预分频，获得更宽的测量范围, FOSC/12/2=1MHz
	IE2   |=  (1<<5);	//允许中断
//	T4T3M |=  (1<<3);	//开始运行
}

//=========================== timer3中断函数 =============================================
void timer3_ISR(void) interrupt TIMER3_VECTOR	//测量换相时间
{
	B_Timer3_OverFlow = 1;	//溢出标志
}

//============================ timer4初始化函数 ============================================
void	Timer4_Config(void)		//换相时间30度角延时以及退磁时间
{
	T4T3M &= 0x0f;		//停止计数, 定时模式, 12T模式, 不输出时钟
	T4H = 0;
	T4L = 0;
	IE2   |=  (1<<6);	//允许中断
//	T4T3M |=  (1<<7);	//开始运行
}

//=========================== timer4中断函数 =============================================
void timer4_ISR(void) interrupt TIMER4_VECTOR		//换相时间30度角延时以及退磁时间
{
	T4T3M &= ~(1<<7);		//Timer4停止运行
	if(XiaoCiCnt == 1)		//标记需要消磁. 每次检测到过0事件后第一次中断为30度角延时, 设置消磁延时.   1:需要消磁(30度角延时), 2:正在消磁, 0已经消磁
	{
		XiaoCiCnt = 2;		//1:需要消磁(30度角延时), 2:正在消磁, 0已经消磁
		if(run_mode == MOTOR_RUN)	StepMotor();	//电机正在运行, 则换相

										//消磁时间, 换相后线圈(电感)电流减小到0的过程中, 出现反电动势, 电流越大消磁时间越长, 过0检测要在这个时间之后
										//100%占空比时施加较重负载, 电机电流上升, 可以示波器看消磁时间.
										//实际上, 只要在换相后延时几十us才检测过零, 就可以了
		T4H = (u8)((65536UL - 10*2) >> 8);	//装载消磁延时, 避开刚换相时电机线圈电感的反电动势出现那一刻。
		T4L = (u8)( 65536UL - 10*2);
		T4T3M |=  (1<<7);	//Timer4开始运行
	}
	else if(XiaoCiCnt == 2)
	{
		XiaoCiCnt = 0;	//1:需要消磁(30度角延时), 2:正在消磁, 0已经消磁

		CMPCR1 &= ~(0x40+0x30);	// 软件清除中断标志位, 禁止比较器上升沿、下降沿中断中断
		if(step & 1)	CMPCR1 |= (1<<5);	//1: 允许上升沿中断, 0: 禁止
		else			CMPCR1 |= (1<<4);	//1: 允许下降沿中断, 0: 禁止
	}
}

u8	adc_index;
void	AdcConvert(void)
{			
		Sw_ADC10 = ((Sw_ADC10 *7)>>3) + Get_ADC10bitResult(10);	//低通滤波结果为13位,   10：ADC10(电位器)
}

void	LoadPwm(void)
{
	Ua = (u16)((((long)svpwm_dat[angle]             *PWM_Value)>>10)+512);	//计算U相占空比
	Ub = (u16)((((long)svpwm_dat[(angle -1365)%4096]*PWM_Value)>>10)+512);	//计算V相占空比, 比U相滞后120度
	Uc = (u16)((((long)svpwm_dat[(angle -2730)%4096]*PWM_Value)>>10)+512);	//计算W相占空比, 比U相滞后240度
	PWMA_CCR1 = Ua;
	PWMA_CCR2 = Ub;
	PWMA_CCR3 = Uc;
}


#if ((ROLL_MinSpeed*MPP) < 22)
	#error "ROLL_MinSpeed*MPP 乘稷要大于等于22!"
#else
#define	ROLL_MaxTime	((23437UL*60/ROLL_MinSpeed)/MPP)	//停转电周期时间 = (60/ROLL_MinSpeed/MPP)*1000*23.4375, ROLL_MinSpeed*MPP >=22
#endif

void PWMA_ISR(void) interrupt PWMA_VECTOR // 23.4375KHz（42.67us）中断率，用于处理启动电机，查询读取比较器结果
{
	u8 i;

	B_pre_cmpo = B_cmpo;	//保存上一个输出电平
	cmpo_state = (cmpo_state << 1) | (CMPCR1 & 1);	// 读取比较器状态，放在一个字节中, 连续8次检测到相同电平才认为有效
	if(cmpo_state == 0xff)	B_cmpo  = 1;	//连续8个采样为1, 则比较器输出1
	if(cmpo_state == 0x00)	B_cmpo  = 0;	//连续8个采样为0, 则比较器输出0

				//============================ 转动检测 =========================================
	if(run_mode == ROLL_CHECK)	//转动检测
	{
		if(++StartTime >= ROLL_MaxTime)	//超时未检测到转动，  ROLL_MaxTime 最大值为2849ms
		{
			StartTime = 0;
			SartCnt   = 0;
			V_PWM3N = 0;	W_PWM4N = 0;
			RollCheckIndex = 0;
			run_mode = PRE_POSITION;
		}
		else if(RollCheckIndex == 0)	//设置U相下降沿
		{
			ADC_CONTR = 0x80+BMF_U;	// 比较器CMP+选择U相
			StartTime = 0;
			PWMA_ENO  = 0;	U_PWM2P = 0;	U_PWM2N = 0;	V_PWM3P = 0;	V_PWM3N = 0;	W_PWM4P = 0;	W_PWM4N = 0;	//关闭PWM输出, 自由转动
			cmpo_state = 0x00;	B_cmpo  = 0;	//连续8个采样为0, 则比较器输出0
			RollCheckIndex = 1;
		}
		else if(RollCheckIndex == 1)	//等待U相下降沿
		{
			if(B_pre_cmpo && !B_cmpo)	//下降沿出现
			{
				ADC_CONTR = 0x80+BMF_V;	// 比较器CMP+选择V相
				StartTime = 0;
				cmpo_state = 0xFF;	B_cmpo  = 1;	//连续8个采样为1, 则比较器输出1
				RollCheckIndex = 2;
			}
		}
		else if(RollCheckIndex == 2)	//等待V相上升沿
		{
			if(!B_pre_cmpo && B_cmpo)	//上升沿出现
			{
				RollPahseTimeUV = StartTime;	//U相下降沿与V相上升沿时间
				StartTime = 0;
				ADC_CONTR = 0x80+BMF_U;	// 比较器CMP+选择U相
				cmpo_state = 0x00;	B_cmpo  = 0;	//连续8个采样为0, 则比较器输出0
				RollCheckIndex = 3;
			}
		}
		else if(RollCheckIndex == 3)	//等待U相下降沿
		{
			if(B_pre_cmpo && !B_cmpo)	//下降沿出现
			{
				StartTime = (StartTime + RollPahseTimeUV)/2;	//半个周期(3个换相时间)
				if(RollPahseTimeUV < StartTime)	//顺时针
				{
					V_PWM3N = 1;	W_PWM4N = 1;	//刹车
					StartTime = 0;
					RollCheckIndex = 4;
				}
				else	//逆时针
				{
					T4T3M &= ~(1<<3);	// Timer3停止运行
					T3H = 0;	T3L = 0;
					B_Timer3_OverFlow = 0;
					T4T3M |=  (1<<3);	//Timer3开始运行
					XiaoCiCnt = 0;	//消磁

					PhaseTime = StartTime *14;	//半个周期(3个换相时间)换相时间折算成us, PhaseTime = (StartTime/3) *42.667;
					for(i=0; i<4; i++)	PhaseTimeTmp[i] = PhaseTime;	//初始换相时间
					delay_N_10us(PhaseTime/20-34);	//30度角延时=PhaseTime/2/10, -34为修正8次采样时间340us
					step = 2;
					StepMotor();	//换相(对应U相反电动势下降沿)

					TimeOut = 200;	//堵转超时
					PWM_Value = START_PWM_DUTY;
					PWMA_IER = 0;	//如果电机已启动运行，则禁止PWMA中断
					run_mode = MOTOR_RUN;

					delay_us(50);
					CMPCR1 &= ~(0x40+0x30);	// 软件清除中断标志位, 禁止比较器上升沿、下降沿中断中断
					if(step & 1)	CMPCR1 |= (1<<5);	//1: 允许上升沿中断, 0: 禁止
					else			CMPCR1 |= (1<<4);	//1: 允许下降沿中断, 0: 禁止
				}
			}
		}
		else if(RollCheckIndex == 4)	//等待U相下降沿, 直到检测不到则停止
		{
			if(B_pre_cmpo && !B_cmpo)	//下降沿出现
			{
				StartTime = 0;
			}
		}
	}
				//============================ 预定位模式 =========================================
	else if(run_mode == PRE_POSITION)	//预定位模式
	{
		if(RollCheckIndex == 0)
		{
			CMPCR1 &= ~(0x40+0x30);	// 软件清除中断标志位, 禁止比较器上升沿、下降沿中断中断
			PWMA_ENO  = 0x3f;			// 全部打开PWM输出
			PWM_Value = PRE_PWM_DUTY;	// 设置预定位占空比
			SartCnt = 0;
			angle = 4096-680;
			LoadPwm();
			RollCheckIndex = 1;
		}
		else if(RollCheckIndex == 1)
		{
			if(++SartCnt == (PRE_STATE_TIME1*23))	//预定位1时间到, PRE_STATE_TIME1最大值为2849ms
			{
				SartCnt = 0;
				angle = 4096-340;
				LoadPwm();
				RollCheckIndex = 2;
			}
		}
		else if(RollCheckIndex == 2)
		{
			if(++SartCnt == (PRE_STATE_TIME2*23))	//预定位2时间到, PRE_STATE_TIME2最大值为2849ms
			{
				SartCnt = 0;
				angle = 0;
				LoadPwm();
				RollCheckIndex = 3;
			}
		}
		else if(RollCheckIndex == 3)
		{
			if(++SartCnt == (PRE_STATE_TIME3*23))	//预定位3时间到, PRE_STATE_TIME3最大值为2849ms
			{
				StartTime = D_StartTime*6;	//StartTime=t*1000/(1024*4/24)=t*375/64=t*6步
				roll_sum  = 0;
				roll_N    = 0;
				SartCnt  = 0;
				PWM_Value = START_PWM_DUTY;	// 设置启动占空比
				run_mode  = STARTING;
			}
		}
		AdcConvert();
	}

				//============================ 启动模式 =========================================
	else if(run_mode == STARTING)	//启动模式, 使用查表S曲线加速启动
	{
		if(++fs_cnt == 4)	// 使用256点的S曲线表，初速V1(转/分)，终速V2(转/分)，加速时间t(ms)，fs=24000000/1024/4 = 46875 /8 = 5859Hz，则加速总步数StartTime = t/1000*fs = t*1000/(1024*4/24) = t*375/64=t*6步
		{					// 查表步进 = 65536/StartTime, 步数SartCnt每次+1，查表得到速度系数 du=S_Curve[SartCnt*65536/StartTime/256] = S_Curve[SartCnt*256/StartTime], 当前速度 V = V1 + (V2-V1)*du/1024 (转/秒)
			fs_cnt = 0;		// 采样频率fs，16位累加器roll_sum，磁极对MPP，则电机转速 V=60*fs*roll_N/roll_sum/MPP, 转速步进 roll_N = V*MPP*sum/60/fs = V*MPP*65536/60/5859 = 0.1864*V*MPP = V*MPP*191/1024

			SartCnt++;
			if(SartCnt < StartTime)
			{
				roll_N = BeginSpeed + (u16)(((u32)S_Curve[(u8)((u32)SartCnt*256/StartTime)] *(FinishSpeed - BeginSpeed))/1024);	//当前速度 V = V1 + (V2-V1)*du/1024 = V1 + (V2-V1)*S_Curve[SartCnt*256/StartTime]/1024  (转/秒)
				roll_sum += (u16)(((u32)roll_N*MPP*191) >>10);	// 当前速度 = V*MPP*191/1024

				angle = (roll_sum >> 4);	//roll_sum是16位的，SVPWM表长度为12位
				LoadPwm();
			}
			else 	//强拖结束
			{
				PWMA_ENO   = 0;
				U_PWM2P=0;	V_PWM3P=0;	W_PWM4P=0;
				U_PWM2N=0;	V_PWM3N=0;	W_PWM4N=0;

				StartTime = 0;
				RollCheckIndex = 0;
				run_mode = ROLL_CHECK;	//进入转动检测模式
			}
		}
		AdcConvert();
	}

    PWMA_SR1 = 0;	//清除标志
}


/**********************************************/
void main(void)
{
	u8	i;
	u16	j;

	P2n_standard(0x08);
	P3n_standard(0x80);  //LED_init 3.7

	Sw_ADC10 = 0;

	PWMA_config();
	ADC_config();
	CMP_config();
	Timer0_config();	// Timer0初始化函数, 系统节拍
	Timer3_Config();	// Timer3初始化函数，换相时间检测
	Timer4_Config();	// Timer4初始化函数，换相时间控制
	PWW_Set = 0;
	TimeOut = 0;
	PhaseTimeIndex = 0;
	//run_mode = MOTOR_IDLE;
	run_mode = PRE_POSITION;  // 跳过IDLE/ROLL_CHECK，直接预定位

	EA  = 1; // 打开总中断

	cnt_4ms = 0;


	while (1)
	{
		if(B_1ms)		// 1ms时隙
		{
			B_1ms = 0;
			if(++cnt_4ms == 4)
			{
				cnt_4ms = 0;
				B_4ms   = 1;	//4ms时隙标志
			}

			if(run_mode == MOTOR_RUN)	//电机正在运行
			{
				PhaseTimeSum += PhaseTime;		// PhaseTime为换相时间
				PhaseSumCnt++;

				i = 0;
				if(PWM_Value < PWW_Set)	i = 1, PWM_Value++;	//油门跟随电位器
				if(PWM_Value > PWW_Set)	i = 1, PWM_Value--;
				if(i != 0)
				{
					PWMA_CCR1 = PWM_Value;
					PWMA_CCR2 = PWM_Value;
					PWMA_CCR3 = PWM_Value;
				}
			}

			if(run_mode == MOTOR_IDLE)	//如果是停机模式，占空比大于设定值, 则开始预定位、启动电机
			{
				if(PWW_Set >= START_PWM_DUTY)
				{
					LED_S = 0;		//LED指示灯亮
					PWMA_SR1 = 0;	//清除PWMA中断标志
					StartTime = 0;
					RollCheckIndex = 0;
					run_mode = ROLL_CHECK;	//进入转动检测模式
					PWMA_IER = 0x01;	// 使能更新中断
				}
			}
			else if(PWW_Set < STOP_PWM_DUTY)	// 非空闲模式，PWM过小停转
			{
				PWMA_IER  = 0;	//如果电机已启动运行，则禁止PWMA中断
				PWM_Value = 0;
				CMPCR1 &= ~(0x40+0x30);	// 软件清除中断标志位, 禁止比较器上升沿、下降沿中断中断
				PWMA_ENO   = 0;
				PWMA_CCR1 = 0;
				PWMA_CCR2 = 0;
				PWMA_CCR3 = 0;
				U_PWM2P=0;	V_PWM3P=0;	W_PWM4P=0;
				U_PWM2N=0;	V_PWM3N=0;	W_PWM4N=0;
				run_mode = MOTOR_IDLE;
			}

		}

		if(B_4ms)		// 4ms时隙
		{
			B_4ms = 0;

			if(++cnt_500ms == 125)
			{
				cnt_500ms = 0;
				B_500ms   = 1;	//500ms时隙标志
			}

			if(TimeOut != 0)
			{
				if(--TimeOut == 0)	//堵转超时
				{
//					PWMA_IER  = 0;	//禁止PWMA中断
//					PWM_Value = 0;
//					CMPCR1 &= ~(0x40+0x30);	// 软件清除中断标志位, 禁止比较器上升沿、下降沿中断中断
//					PWMA_CCR1 = 0;
//					PWMA_CCR2 = 0;
//					PWMA_CCR3 = 0;
//					U_PWM2P=0;	V_PWM3P=0;	W_PWM4P=0;
//					U_PWM2N=0;	V_PWM3N=0;	W_PWM4N=0;
//					delay_ms(250);	//堵转时,延时一点时间再启动
//					run_mode = MOTOR_IDLE;
				}
			}

			if (run_mode == MOTOR_IDLE)	//如果是停机模式，则做ADC
			{
				Sw_ADC10 = ((Sw_ADC10 *7)>>3) + Get_ADC10bitResult(10);	//低通滤波结果为13位,   7：ADC7(电位器)
			}

			j = Sw_ADC10;																					/*ADCC油门---Mori*/
			if(j != Sw_ADC10)	j = Sw_ADC10;
					//j = 800;			
			//PWW_Set  = j >> 3;	//油门是10位的																	/*ADCC油门---Mori*/
			PWW_Set  = 800;
		}

		if(B_500ms)	//500ms时隙
		{
			B_500ms = 0;

			// 计算每分钟转速	换相时间 PhaseTime = PhaseTimeSum / PhaseSumCnt,   转速 = 60*1000000/(6*PhaseTimeSum/PhaseSumCnt)/MPP = (10000000/MPP)*PhaseSumCnt/PhaseTimeSum
			if(run_mode == MOTOR_RUN)	RollSpeed = (u32)((10000000.0f/MPP) * PhaseSumCnt / PhaseTimeSum);	// 计算当前转速 = 60*1000000/(6*PhaseTimeSum/PhaseSumCnt)/MPP = (10000000/MPP)*PhaseSumCnt/PhaseTimeSum
			else						RollSpeed = 0;
			PhaseSumCnt  = 0;
			PhaseTimeSum = 0;
		}
	}
}

