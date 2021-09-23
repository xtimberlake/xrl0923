#ifndef __LIB_IMPED_H
#define __LIB_IMPED_H

#include "main.h"
#define ABS(x) ((x>0)? (x): (-x))

typedef struct __impedance_t
{
    float Kp;
    float Kd;

    float set;				//目标值
    float get;				//测量值
    float err;				//误差

    float d_set;				//目标值
    float d_get;				//测量值
    float d_err;				//误差


    float pout;							//p输出
    float dout;							//d输出

    float pos_out;						//本次位置式输出
    float last_pos_out;				//上次输出

    float max_err;
    float deadband;				//err < deadband return

    long unsigned int MaxOutput;				//输出限幅

}impedance_t;

void impedance_struct_init(impedance_t* impd, float Kp, float Kd, long unsigned int max, float deadband);
void abs_limit(float *a, float ABS_MAX,float offset);
float impedance_calc(impedance_t* impd, float get, float set, float d_get, float d_set);
void impedance_control(void);

#endif
