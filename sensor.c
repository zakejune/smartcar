#include "sensor.h"
#include "car.h"


int8 error=0;      //系统误差
uint16 Kp_e=60,Kd_e=50 ;
float  Pwm_value=SERVO_MIDDLE;   //中值
float Pi,Di;
float Direction_Err[2];	//方向偏差（g_fDirectionError[0]为一对水平电感的差比和偏差）
float Final[10];				//（g_fDirectionError[1]为一对垂直电感的差比和偏差）
float DirectionErr_dot[2];//方向偏差微分（g_fDirectionError_dot[0]为一对水平电感的差比和偏差微分）
				//（g_fDirectionError_dot[1]为一对垂直电感的差比和偏差微分）
float DirectionOut = 0;	//方向控制输出
uint16 sensor[4] = { 0 };       //获取的电感值
uint8 Stop_Flag = 0;
uint16 sensorFilter[5] = { 0 };	//阶梯滤波的电感值（未使用）
uint8 Flag_Round1 = 0;		//进入环岛的标志（在环岛里为ON）
uint8 Flag_Round2 = 0;
uint8 Flag_Round3 = 0;  //目前判断是向左拐还是向右拐
uint16 max_v[4],min_v[4], AD_sensor[4], adc_sum[4], adc_valueD[4], adc_value[4][10];
float sensor_to_one[4];


uint16  AD_Avg(ADCn_Ch_e adcn_ch,ADC_nbit bit,uint8 N)
{
  uint32 temp2=0;
  uint8 i,j;

  for(i=0;i<N;i++)
  {
    temp2+=adc_once(adcn_ch, bit);
    j=N;
    while(j--);
  }
  return (uint16)(temp2/N);
}

void To_one()               //**************过渡点AD归一化
{ 
    max_v[0] = max_v[1] = max_v[2] = max_v[3] =4000;   //给每个的最大值赋初值
    min_v[0] = min_v[1] = min_v[2] = min_v[3] =0;   //给每个的最小值赋初值，此时最小值设定为7
    
  int i;
  for(i=0;i<4;i++)
  {
   sensor_to_one[i]=(float)(AD_sensor[i] - min_v[i])/(max_v[i] - min_v[i]);
   if (sensor_to_one[i]<=0.001)   
     sensor_to_one[i]=0.001;
   if (sensor_to_one[i]>1)      
     sensor_to_one[i]=1;
   sensor[i]  = (uint16)(100* sensor_to_one[i]);   //归一化0---100
  }
}

void Get_ADC(void)              //***************过渡点AD采集滤波
{
  
  int i=0,j=0,k=0,N1=10;
  uint16 Temp=0;
  for(i=0;i<N1;i++)  //每个通道存10个数据
  {
    
        adc_value[0][i]=AD_Avg(ADC1_SE10,ADC_12bit,20); //PTB4
        adc_value[1][i]=AD_Avg(ADC1_SE11,ADC_12bit,20);  //PTB5
        adc_value[2][i]=AD_Avg(ADC1_SE12,ADC_12bit,20);  //PTB6
        adc_value[3][i]=AD_Avg(ADC1_SE13,ADC_12bit,20);   //PTB7

 
  //adc_value[6][i]=AD_Avg(ADC1_SE4a,ADC_12bit,20);//PTE0      停车
   
  }
  //冒泡排序
  for(i=0;i<4;i++)
  {
    for(j=0;j<N1;j++)
    {
      for(k=0;k<N1-j;k++)
      {
        if(adc_value[i][k]>adc_value[i][k+1])     //前面比后面的大就进行交换 
        {
          Temp=adc_value[i][k+1];
          adc_value[i][k+1]=adc_value[i][k];
          adc_value[i][k]=Temp;
        }
      }
    }
  }
  //中值滤波
  for(i=0;i<4;i++)
  {
    adc_sum[i]=adc_value[i][1]+adc_value[i][2]+adc_value[i][3]+adc_value[i][4];      //选取中间8项
    adc_valueD[i]=adc_sum[i]/4;                                      //求平均值
  }
  
  for(i=0;i<4;i++)               //将数值中的个位数除掉，降低过高精度
  {
    AD_sensor[i]=(uint16)(adc_valueD[i]/10*10);
  } 
     To_one();
}



void DirectionCtrl(void)
{
   static  float DirectionErrTemp[2][5]={0};
   Get_ADC();
 //  if(sensor[0]<10 && sensor[1]<10)  Stop_Flag=1;

//电感限幅
sensor[0]=(sensor[0]<10? 10:sensor[0]);  
sensor[1]=(sensor[1]<10? 10:sensor[1]);
sensor[2]=(sensor[2]<10? 10:sensor[2]);
sensor[3]=(sensor[3]<10? 10:sensor[3]);


//水平电感偏差
	Direction_Err[0] = (float)(sensor[2]-sensor[1])/(sensor[1]+sensor[2]);//左-右水平电感的差比和作为偏差
	Direction_Err[0] = (Direction_Err[0]>= 1? 1:Direction_Err[0]);    //偏差限幅	
	Direction_Err[0] = (Direction_Err[0]<=-1?-1:Direction_Err[0]);
//水平电感偏差微分
DirectionErrTemp[0][4] = DirectionErrTemp[0][3];
	DirectionErrTemp[0][3] = DirectionErrTemp[0][2];
	DirectionErrTemp[0][2] = DirectionErrTemp[0][1];
	DirectionErrTemp[0][1] = DirectionErrTemp[0][0];
	DirectionErrTemp[0][0] = Direction_Err[0];
	DirectionErr_dot[0] = 5*(DirectionErrTemp[0][0]-DirectionErrTemp[0][3]);//水平电感的偏差微分
	DirectionErr_dot[0] = (DirectionErr_dot[0]> 0.7? 0.7:DirectionErr_dot[0]);//偏差微分限幅
	DirectionErr_dot[0] = (DirectionErr_dot[0]<-0.7?-0.7:DirectionErr_dot[0]);


//垂直电感偏差
Direction_Err[1] = (float)(sensor[0]-sensor[3])/(sensor[0]+sensor[3]);//左-右水平电感的差比和作为偏差
	Direction_Err[1] = (Direction_Err[1]>= 1? 1:Direction_Err[1]);    //偏差限幅	
	Direction_Err[1] = (Direction_Err[1]<=-1?-1:Direction_Err[1]);

//垂直电感偏差微分

DirectionErrTemp[1][4] = DirectionErrTemp[1][3];
	DirectionErrTemp[1][3] = DirectionErrTemp[1][2];
	DirectionErrTemp[1][2] = DirectionErrTemp[1][1];
	DirectionErrTemp[1][1] = DirectionErrTemp[1][0];
	DirectionErrTemp[1][0] = Direction_Err[1];
	DirectionErr_dot[1] = 5*(DirectionErrTemp[1][0]-DirectionErrTemp[1][3]);//垂直电感的偏差微分
	DirectionErr_dot[1] = (DirectionErr_dot[1]> 0.7? 0.7:DirectionErr_dot[1]);//偏差微分限幅
	DirectionErr_dot[1] = (DirectionErr_dot[1]<-0.7?-0.7:DirectionErr_dot[1]);	 

}

void island()  //环岛识别与控制
{
	DirectionCtrl();
	float island1[10];
	uint16 i,j;
  for(i=0;i<10;i++)
  {
    island1[i]=Direction_Err[0];
    j=10;
    while(j--);
  }
}


void sensor_init()
{
     adc_init(ADC1_SE10);
     adc_init(ADC1_SE11);   
     adc_init(ADC1_SE12);
     adc_init(ADC1_SE13);
}



void finaler(void)
{
   uint16 i,j;
    
  for(i=0;i<10;i++)
  {
    Final[i]=Direction_Err[0];
    j=10;
    while(j--);
  }
}
  


void PID_elec(void)
{
    finaler();   
    PID_s1 pe;
   // PID_s1.current_err=Direction_Err[0];
  
    pe.last_err=(Final[0]-Final[3]);      //差值更新
    pe.current_err = Final[0];

    Pi = Kp_e * pe.current_err;
    Di = Kd_e * (pe.current_err - pe.last_err);
//差值更新
  Di=(Di>50?50:Di);
  Pi=(Pi>50? 50:Pi);
	
	
	
	
    Pwm_value=SERVO_MIDDLE+Pi+Di ;

}

void Send_Upper(void)
{
  uchar a[10],b[10],c[10],d[10],e[10],f[10];
  
  sprintf((char*)a, "%d", AD_sensor[1]);
  sprintf((char*)b, "%d", AD_sensor[2]);
  sprintf((char*)e, "%d", AD_sensor[0]);
  sprintf((char*)f, "%d", AD_sensor[3]);
  sprintf((char*)c, "%.3f", Pi);
  sprintf((char*)d, "%.3f", Di);
  
  uart_putbuff(UART1, (uint8_t *)a, 5);
  uart_putbuff(UART1, (uint8_t *)b, 5);
  uart_putbuff(UART1, (uint8_t *)e, 5);
  uart_putbuff(UART1, (uint8_t *)f, 5);
  uart_putbuff(UART1, (uint8_t *)c, 7);
  uart_putbuff(UART1, (uint8_t *)d, 7);

  uart_putchar(UART1,0x0A);
}

