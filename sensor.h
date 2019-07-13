#ifndef _CAR_SENSOR_H_
#define _CAR_SENSOR_H_

#include "car.h"
#include  "sensor.h"

typedef struct  PID_s1
{
    float pre_err;
    float current_err;
    float last_err;

}PID_s1;

void PID_elec(void);
extern float Final_Err[10];
//uint16 AD_Avg(ADCn_Ch_e adcn_ch, ADC_nbit bit, uint8 N);
void To_one();
void Get_ADC(void);
void sensor_init();
void DirectionCtrl(void);
void finalerr(void);
void PID_elec(void);
void Send_Upper(void);

#endif
