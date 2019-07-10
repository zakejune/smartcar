#include <hidef.h>	/* common defines and macros */
#include "derivative.h"	/* derivative-specific definitions */

int left1=0; int left2=0; int right1=0; int right2=0;
int AR_LEFT=0;//left2-right2 int AR_RIGHT=0;
int CR=0;// 左边相加减右边相加
int preCR=0; int ppreCR=0; int mkp=0;
int mki=0; int mkd=0;

int ideal_speed=0;	// 设定速度int speed=0;
int s_ideal0[6]={75,80,42,42,42,42}; //	普通道、长直道、普通到弯、长直道到弯、弯内、偏离黑线
int s_ideal1[6]={70,75,42,42,42,42}; int s_ideal2[6]={62,70,42,40,41,40}; int s_ideal3[6]={54,66,42,40,41,40};


int table_mkp0[6]={30,30,30,30,30,30};	//ni 16.31	,shun 15.16 int table_mkp1[6]={25,25,25,25,25,25};
int table_mkp2[6]={5,4,4,20,20,20};
int table_mkp3[6]={4,4,4,10,8,9}; //	稳定速度

int table_mki0[6]={0,0,20,20,20,20}; int table_mki1[6]={0,0,20,20,20,20}; int table_mki2[6]={0,0,0,10,10,20}; int table_mki3[6]={0,0,0,0,0,0};

int table_mkd0[6]={0,0,0,0,0,0};
int table_mkd1[6]={0,0,0,0,0,0}; int table_mkd2[6]={0};
int table_mkd3[6]={0,0,0,0,0,0};

int s_table[6]; int b_mkp[6]=0; int b_mki[6]=0; int b_mkd[6]=0;
 



int	table_rkp0[7]={5,3,2,550,550,550,8};//	普通道中间、长直道低速、长直道高速、普到弯、直到弯、弯、普通道两边
int table_rkp1[7]={7,5,4,450,450,400,9}; int table_rkp2[7]={6,3,2,150,150,150,9}; int table_rkp3[7]={5,3,2,150,150,150,9};

int table_rkd0[7]={0,0,0,400,400,400,100}; int table_rkd1[7]={0,0,0,500,500,500,100}; int table_rkd2[7]={0,0,0,200,300,400,100}; int table_rkd3[7]={0,0,0,200,300,400,100};

int b_rkp[7]=0; int b_rkd[7]=0;

int rkp=0; int rkd=0;
int f=0;// pwmDTY	要加的值
int pref=0;




int Pulse_count=0;	// 脉冲数int ganhuang=0;
unsigned int ting=0; int i=0;
int Flag_Chute=0;	//道路标志
int GeneralCtn=0; int CurveCtn=0; int ChuteCtn=0; int WANCtn=0;
int Flag_gaosu=0;
unsigned char Flag_Pwm;// 知道转弯道标志

int flag=0;

/***********************
//PLL 超 频到	40MHZ
****************/
void PLL_Init(void) { CLKSEL=0X00;
 
PLLCTL_PLLON=1;
REFDV=0X80|0X01;
SYNR=0X40|0X04;
POSTDIV=0X00;
asm nop; asm nop;
while(!(CRGFLG_LOCK==1)); CLKSEL_PLLSEL=1;
}


//延时函数	cnt*1ms;
void delay(unsigned int cnt) { unsigned int loop_i,loop_j; for(loop_i=0;loop_i<cnt;loop_i++) {
loop_j=0x1300; while(loop_j--);
}
}


/*******************************
计数程序 //\\\\\\\\\\\\\\\
*********************************/
void PACBInit()	//PT7	获得脉冲值
{
PACTL=0X40;	//PT7 PIN,PACN32 16BIT,Rising edge,NOT INTERRUPT
TCTL3=0x40;		//c-输入捕捉	7 上升沿有效 , TIE_C7I=0;	// 通道 7 禁止中断
TIOS_IOS7=0;	// 每一位对应通道的	: 0 输入捕捉 ,1 输出比较
PACNT = 0;
}


void RTI_init(void)	//RTI	产生	10ms 的中断定时
{
asm sei;	// 关闭中断
RTICTL=0xC7;	//中断周期设置	10ms 中断一次 (或者让 RTICTL=0x59<
为 10.24ms 定 时 >)
CRGINT_RTIE=1;	// 实时中断有效，一旦	RTIF=1	则发出中断请求
asm cli;	// 开放中断
}





//舵机初始化
 
void PWM_rudder_init(void) { PWME_PWME3=0; PWME_PWME2=0;
PWMPRCLK_PCKB=2;//CLOCKB=BUS/4=10MHz PWMSCLB=2;//CLOCCSB=10/(2*2)=2.5MHz
PWMCTL_CON23=1;//	组 合 PWM23
PWMCLK_PCLK3=1;//PWM3	使 用 SB
PWMPER23=50000;// 写 PWM23  的周期寄存器，周期是	20ms
PWMPOL_PPOL3=1;//	极性为正
PWMCAE_CAE3=0;//	左对齐
PWME_PWME3=1;//	使 能 PWM23
}




//电机初始化
void PWM_init_motor(void){ //	电机初始化

PWME_PWME0=0;
PWME_PWME1=0;
PWMPRCLK_PCKA=2; //Clock A=40M/4=10M
PWMPOL_PPOL1=1;//	通道 1 正极性输出
PWMCLK_PCLK1=0;//	通道 1 选择 A 时钟
PWMCAE_CAE1=0;//	左对齐
PWMCTL_CON01=1;
PWMPER01=1000;// 输出频率 =10M/1000=10Khz
PWMDTY01=0;//	通道  1  占空比为	100/250
PWME_PWME1=1;//	通道 1 使能


PWME_PWME4=0;
PWME_PWME5=0;
PWMPRCLK_PCKA=1; //Clock A=40M/2=20M
PWMPOL_PPOL5=1;//	通道 5 正极性输出
PWMCLK_PCLK5=1;//	通道 5 选择 SA 时钟
PWMSCLA=1;	//ClockSB=20M/(2*1)=10M
PWMCAE_CAE5=0;//	左对齐
PWMCTL_CON45=1;
PWMPER45=1000;// 输出频率 =10M/1000=10Khz
PWMDTY45=0;//	初始通占空比	0
PWME_PWME5=1;//	通道 5 使能
}
 
void AD_Init(void)
{


ATD0CTL1=0x20;	// 选择 AD 通道为外部触发	,10 位精度 ,采样前不放电ATD0CTL2=0x40;	// 标志位自动清零，禁止外部触发	, 禁止中断
ATD0CTL3=0xA0;			// 右对齐无符号	,每次转换		4 个序列 , No FIFO, Freeze 模式下继续转ATD0CTL4=0x09;	// 采样时间为	4 个 AD 时 钟 周 期 ,PRS=9,ATDClock=40/(2*(9+1))2MHz ATD0CTL5=0x30;		// 特殊通道禁止  ,连续转换	4 个通道	,多通道转换，  起始通道为	0 转换ATD0DIEN=0x00;		// 禁止数字输入
}


/************************
///////// 检测起跑线 \\\\\\\\\\\
*****************/ void Checkstart(){


asm sei;

TIOS_IOS0=0;	//输入捕捉TSCR1=0X80;
TSCR2=0X07;


TCTL4=0X01;//	上升沿捕捉TIE=0X01;	// 允许硬件中断asm cli;
}







//拨码开关void boman(){
if(PORTA_PA0==1) {
b_rkp[0]=table_rkp0[0]; b_rkp[1]=table_rkp0[1]; b_rkp[2]=table_rkp0[2]; b_rkp[3]=table_rkp0[3]; b_rkp[4]=table_rkp0[4]; b_rkp[5]=table_rkp0[5]; b_rkp[6]=table_rkp0[6];

b_rkd[0]=table_rkd0[0]; b_rkd[1]=table_rkd0[1]; b_rkd[2]=table_rkd0[2];
 
b_rkd[3]=table_rkd0[3]; b_rkd[4]=table_rkd0[4]; b_rkd[5]=table_rkd0[5]; b_rkd[6]=table_rkd0[6];

b_mkp[0]=table_mkp0[0]; b_mkp[1]=table_mkp0[1]; b_mkp[2]=table_mkp0[2]; b_mkp[3]=table_mkp0[3]; b_mkp[4]=table_mkp0[4]; b_mkp[5]=table_mkp0[5];

b_mki[0]=table_mki0[0]; b_mki[1]=table_mki0[1]; b_mki[2]=table_mki0[2]; b_mki[3]=table_mki0[3]; b_mki[4]=table_mki0[4]; b_mki[5]=table_mki0[5];

b_mkd[0]=table_mkd0[0]; b_mkd[1]=table_mkd0[1]; b_mkd[2]=table_mkd0[2]; b_mkd[3]=table_mkd0[3]; b_mkd[4]=table_mkd0[4]; b_mkd[5]=table_mkd0[5];


s_table[0]=s_ideal0[0]; s_table[1]=s_ideal0[1]; s_table[2]=s_ideal0[2]; s_table[3]=s_ideal0[3]; s_table[4]=s_ideal0[4]; s_table[5]=s_ideal0[5];

}
if(PORTA_PA1==1) {
b_rkp[0]=table_rkp1[0]; b_rkp[1]=table_rkp1[1]; b_rkp[2]=table_rkp1[2]; b_rkp[3]=table_rkp1[3]; b_rkp[4]=table_rkp1[4]; b_rkp[5]=table_rkp1[5]; b_rkp[6]=table_rkp1[6];
 
b_rkd[0]=table_rkd1[0]; b_rkd[1]=table_rkd1[1]; b_rkd[2]=table_rkd1[2]; b_rkd[3]=table_rkd1[3]; b_rkd[4]=table_rkd1[4]; b_rkd[5]=table_rkd1[5]; b_rkd[6]=table_rkd1[6];

b_mkp[0]=table_mkp1[0]; b_mkp[1]=table_mkp1[1]; b_mkp[2]=table_mkp1[2]; b_mkp[3]=table_mkp1[3]; b_mkp[4]=table_mkp1[4]; b_mkp[5]=table_mkp1[5];

b_mki[0]=table_mki1[0]; b_mki[1]=table_mki1[1]; b_mki[2]=table_mki1[2]; b_mki[3]=table_mki1[3]; b_mki[4]=table_mki1[4]; b_mki[5]=table_mki1[5];

b_mkd[0]=table_mkd1[0]; b_mkd[1]=table_mkd1[1]; b_mkd[2]=table_mkd1[2]; b_mkd[3]=table_mkd1[3]; b_mkd[4]=table_mkd1[4]; b_mkd[5]=table_mkd1[5];

s_table[0]=s_ideal1[0]; s_table[1]=s_ideal1[1]; s_table[2]=s_ideal1[2]; s_table[3]=s_ideal1[3]; s_table[4]=s_ideal1[4]; s_table[5]=s_ideal1[5];
}
if(PORTA_PA2==1) {
b_rkp[0]=table_rkp2[0]; b_rkp[1]=table_rkp2[1]; b_rkp[2]=table_rkp2[2]; b_rkp[3]=table_rkp2[3]; b_rkp[4]=table_rkp2[4]; b_rkp[5]=table_rkp2[5]; b_rkp[6]=table_rkp2[6];
 

b_rkd[0]=table_rkd2[0]; b_rkd[1]=table_rkd2[1]; b_rkd[2]=table_rkd2[2]; b_rkd[3]=table_rkd2[3]; b_rkd[4]=table_rkd2[4]; b_rkd[5]=table_rkd2[5]; b_rkd[6]=table_rkd2[6];

b_mkp[0]=table_mkp2[0]; b_mkp[1]=table_mkp2[1]; b_mkp[2]=table_mkp2[2]; b_mkp[3]=table_mkp2[3]; b_mkp[4]=table_mkp2[4]; b_mkp[5]=table_mkp2[5];

b_mki[0]=table_mki2[0]; b_mki[1]=table_mki2[1]; b_mki[2]=table_mki2[2]; b_mki[3]=table_mki2[3]; b_mki[4]=table_mki2[4]; b_mki[5]=table_mki2[5];

b_mkd[0]=table_mkd2[0]; b_mkd[1]=table_mkd2[1]; b_mkd[2]=table_mkd2[2]; b_mkd[3]=table_mkd2[3]; b_mkd[4]=table_mkd2[4]; b_mkd[5]=table_mkd2[5];






s_table[0]=s_ideal2[0]; s_table[1]=s_ideal2[1]; s_table[2]=s_ideal2[2]; s_table[3]=s_ideal2[3]; s_table[4]=s_ideal2[4]; s_table[5]=s_ideal2[5];
}
if(PORTA_PA3==1) {
b_rkp[0]=table_rkp3[0]; b_rkp[1]=table_rkp3[1]; b_rkp[2]=table_rkp3[2]; b_rkp[3]=table_rkp3[3];
 
b_rkp[4]=table_rkp3[4]; b_rkp[5]=table_rkp3[5]; b_rkp[6]=table_rkp3[6];

b_rkd[0]=table_rkd3[0]; b_rkd[1]=table_rkd3[1]; b_rkd[2]=table_rkd3[2]; b_rkd[3]=table_rkd3[3]; b_rkd[4]=table_rkd3[4]; b_rkd[5]=table_rkd3[5]; b_rkd[6]=table_rkd3[6];

b_mkp[0]=table_mkp3[0]; b_mkp[1]=table_mkp3[1]; b_mkp[2]=table_mkp3[2]; b_mkp[3]=table_mkp3[3]; b_mkp[4]=table_mkp3[4]; b_mkp[5]=table_mkp3[5];

b_mki[0]=table_mki3[0]; b_mki[1]=table_mki3[1]; b_mki[2]=table_mki3[2]; b_mki[3]=table_mki3[3]; b_mki[4]=table_mki3[4]; b_mki[5]=table_mki3[5];

b_mkd[0]=table_mkd3[0]; b_mkd[1]=table_mkd3[1]; b_mkd[2]=table_mkd3[2]; b_mkd[3]=table_mkd3[3]; b_mkd[4]=table_mkd3[4]; b_mkd[5]=table_mkd3[5];






s_table[0]=s_ideal3[0]; s_table[1]=s_ideal3[1]; s_table[2]=s_ideal3[2]; s_table[3]=s_ideal3[3]; s_table[4]=s_ideal3[4]; s_table[5]=s_ideal3[5];
}
}
 







/**************************
//////// 赛道特征识别	///////
****************************/ void Roadjudge(void){
if(Flag_Chute==0)	// 普通弯道情况	0
{	GeneralCtn=0;
CurveCtn=0;
if(CR>=-23&&CR<=40)	// 此 时 对 应 一 个 车 轮 的 内 侧 压 线
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@
{
ChuteCtn++;
WANCtn=0;
}
if(CR<-23||CR>40)	{	//
//@@@@@@@@@@@@@@@@@@@@@@@@@@
if(CR<-85||CR>90){	// 对 应 车 轮 的 外 侧 压 线
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ 
ChuteCtn=0;
WANCtn++;
}
else { ChuteCtn=0; WANCtn=0;
}
}

if(ChuteCtn>10000)	// 300
{
Flag_Chute=1;
}
if(W ANCtn<-10000){
Flag_Chute=2;
}


}
if(Flag_Chute==1)	//长直道情况	1
{
ChuteCtn=0; GeneralCtn=0;
 
WANCtn=0;
if(CR>40)CurveCtn++;	//	60
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
if(CR<-23)CurveCtn--;	//	-60
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ if(CurveCtn>1||CurveCtn<-1)
{
if(Flag_gaosu==1)	//高速时转入大弯道情况
{
Flag_Chute=2;
}
if(Flag_gaosu==0)
Flag_Chute=0;	// 低速时转入普通弯道情况


}
}




if(Flag_Chute==2)	//大弯道情况	2
{
ChuteCtn=0;
CurveCtn=0;
WANCtn=0;	
//@@@@@@@@@@@@@@@@@@@@@@@ 
if(CR>-23&&CR<35)GeneralCtn++;
else GeneralCtn=0; if(GeneralCtn>1300)Flag_Chute=0;
//if(GeneralCtn>130)Flag_Ct=1;
//else Flag_Ct=0;
//if(k_abs(Turn_C-Turn<54)
//	{
//	Flag_Zhj=1;
//	}
// else
//	{
//	Flag_Zhj=0;
//	}
//	if(Flag_Pwm==1&&Flag_Ct==1)Flag_Chute=1;
}
}
/******************************
\\\\\\\\\\\\\\\\\\\\//	舵机控制 \\\\\\\\\\\\\\\\\
*********************************/ void rudder_ctrl(void){
 

if(Flag_Chute==0)	//普通道
{
Flag_Pwm=0; if(CR<40&&CR>-23){
rkp=b_rkp[0]; rkd=b_rkd[0];
}
else{
rkp=b_rkp[6]; rkd=b_rkp[6];
}
}


else if(Flag_Chute==1) // 长直道
{
Flag_Pwm=1;
if(Flag_gaosu==0)	//disu
{
rkp=b_rkp[1];	//5 rkd=b_rkd[1];
}
else	//gaosu
{
rkp=b_rkp[2];	//4 rkd=b_rkd[2];
}
}
else if(Flag_Chute==2)	// 大弯道
{

if(Flag_Pwm==0)	// 普通道到大弯道
{
rkp=b_rkp[3]; rkd=b_rkd[3];
}
else if(Flag_Pwm==1)	//直道	到	大弯道
{
rkp=b_rkp[4]; rkd=b_rkd[4];
}
else{
rkp=b_rkp[5]; rkd=b_rkd[5];
 
}
}


f=3800+rkp*CR+rkd*(CR-2*preCR+ppreCR); //	舵机的 PID 算式
ppreCR=preCR;// 计算之后向前推进赋值	为下次计算做准备
preCR=CR;
}
/***********************************
电机控制  \\\\\\\\\\\\\\\\\\
************************************/ void motor_ctrl1(void){ if(Flag_Chute==0)	//普通道
{
Flag_Pwm=0; mkp=b_mkp[0]; mki=b_mki[0]; mkd=b_mkd[0]; ideal_speed=s_table[0];


}


else if(Flag_Chute==1) // 长直道
{
Flag_Pwm=1;
// Flag_PIDRev=0;
mkp=b_mkp[1]; mki=b_mki[1]; mkd=b_mkd[1]; ideal_speed=s_table[1];
}
else if(Flag_Chute==2)	// 大弯道
{

if(Flag_Pwm==0)	// 普通道进大弯道
{
mkp=b_mkp[2]; mki=b_mki[2]; mkd=b_mkd[2]; ideal_speed=s_table[2];
}
else if(Flag_Pwm==1)	// 直道进大弯道
{
mkp=b_mkp[3]; mki=b_mki[3];
 




else{
 
mkd=b_mkd[3]; ideal_speed=s_table[3];
}

mkp=b_mkp[4]; mki=b_mki[4]; mkd=b_mkd[4]; ideal_speed=s_table[4];
}
Flag_Pwm=2;
 




}
}


//电机控制
void motor_ctrl2(void){
int error,m_perror,m_ierror,m_derror; int pre_error=0;
int ppre_error=0;

error=ideal_speed-Pulse_count; m_perror = error - pre_error; m_ierror=error;
m_derror=error-2*pre_error+ppre_error; ppre_error=pre_error;
pre_error=error;
speed+=mkp*m_perror + mki*m_ierror + mkd*m_derror;	//速度 PID 控制算式

if(speed>=9000)	speed=9000; if(speed<=-4000) speed=-4000;

if(speed>=0)


{
PWMDTY45=0;
PWMDTY01=(int)speed/10;
}
 
else
{



}
}
 



PWMDTY45=(int)(-speed)/10; PWMDTY01=0;
 





//主函数 //
void main(void) {

DisableInterrupts;
PWMDTY23=3800;
PLL_Init();
DDRA=0X00;//	输入
boman();
PACBInit();
RTI_init();
PWM_rudder_init();
PWM_init_motor();
AD_Init();
DDRB=0XFF;//	输出
PORTB=0X03;//1	号与 2 号灯亮
delay(5000); //4000 3s 左 右
PORTB=0xFC;//3	号与 4 号灯亮
Checkstart();

EnableInterrupts;
/* put your own code here */








for(;;) {

while(!ATD0STA T2_CCF0);	//  等待转换结束	while(ATDOSTA T2_CCF0==1)
left1=ATD0DR0;//	读取结果寄存器	left1 的 值


while(!ATD0STAT2_CCF2);	//  等待转换结束	while(ATDOSTAT2_CCF1==1)
left2=ATD0DR2;//	读取结果寄存器的值


while(!ATD0STAT2_CCF1);	//  等待转换结束	while(ATDOSTAT2_CCF2==1) right1=ATD0DR1;//	读取结果寄存器的值

while(!ATD0STAT2_CCF3);	//  等待转换结束	while(ATDOSTAT2_CCF3==1)
 
right2=ATD0DR3;// 读取结果寄存器的值

AR_LEFT=left1+left2; AR_RIGHT=right1+right2; CR=(AR_RIGHT-AR_LEFT)/10;

if(Pulse_count>65)Flag_gaosu=1; else Flag_gaosu=0;

if(AR_RIGHT<110||AR_LEFT<95)
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@	left93
{if(pref>3800)	//if(AR_RIGHT<110) f=4500;		//f=3100;
if(pref<3800)	//if(AR_LEFT<95) f=3100;		//f=4500;
mkp=b_mkp[5]; mki=b_mki[5]; mkd=b_mkd[5];
ideal_speed=s_table[5];
}
else{
Roadjudge(); // 先对道路进行判断rudder_ctrl(); // 调整舵机motor_ctrl1(); // 调整电机的	pid 参 数

} if(f>4500)f=4500; if(f<3100)f=3100; PWMDTY23=f; pref=PWMDTY23;
motor_ctrl2();



}

} /* loop forever */




#pragma CODE_SEG 	NEAR_SEG NON_BANKED

interrupt 7 void Int_TimerOverFlow(void)
{
Pulse_count= PACNT;	//脉冲计数赋值
 
PACNT = 0X0000; CRGFLG_RTIF=1;
if(ting<1100)ting++; else ting=1100;
}


interrupt VectorNumber_Vtimch0 void stop(void){ DisableInterrupts;
TFLG1_C0F=1; // 清除中断标志位

//PORTB=0X03;//1	号与 2 号灯亮

// delay(20);

// PORTB=0xFC;//3	号与 4 号灯亮

// ganhuang++;
//if(ganhuang%4==0){
// ganhuang=0;
//PORTB=0xFC;//3	号与 4 号灯亮
//ganhuang=0;
//TIE=TIE&0X0FE;
flag=1; if(ting==1100){ flag=2; PORTB=0xF6; for(i=0;i<3000;i++){

while(!ATD0STA T2_CCF0);	//  等待转换结束	while(ATDOSTA T2_CCF0==1)
left1=ATD0DR0;//	读取结果寄存器	left1 的 值


while(!ATD0STAT2_CCF2);	//  等待转换结束	while(ATDOSTAT2_CCF1==1)
left2=ATD0DR2;//	读取结果寄存器的值


while(!ATD0STAT2_CCF1);	//  等待转换结束	while(ATDOSTAT2_CCF2==1)
right1=ATD0DR1;//	读取结果寄存器的值

while(!ATD0STAT2_CCF3);	//  等待转换结束	while(ATDOSTAT2_CCF3==1) right2=ATD0DR3;// 读取结果寄存器的值

AR_LEFT=left1+left2; AR_RIGHT=right1+right2;
 
CR=(AR_RIGHT-AR_LEFT)/10;


rkp=9; rkd=0;
f=3800+rkp*CR+rkd*(CR-preCR); preCR=CR;
if(f>4500)f=4500; if(f<3100)f=3100; PWMDTY23=f; pref=PWMDTY23;
//mkp=20;
//mki=0;
//mkd=0;
//ideal_speed=0;
//motor_ctrl2();
PWMDTY01=0;
PWMDTY45=0;
delay(2);
}
}
EnableInterrupts;
}
// asm cli;
#pragma CODE_SEG DEFAULT

// 结论 3800 中间； 3100(-700) 左打死， 4500(700) 右打死
//7.11 当右边两个相加小于	40 时往左打死，此时左车轮大概偏黑线	2cm，当左边两个相加小于 100 时右打死
// 此时右轮偏	2cm
