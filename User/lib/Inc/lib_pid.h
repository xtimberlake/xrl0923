#ifndef __PID_H
#define __PID_H

#include "main.h"

enum{
    LLAST	= 0,
    LAST 	= 1,
    NOW 	= 2,
    
    POSITION_PID,
    DELTA_PID,
};
typedef struct __pid_pkg
{
    float p;
    float i;
    float d;
    
    float set[3];				//鐩爣鍊�,鍖呭惈NOW锛� LAST锛� LLAST涓婁笂娆�
    float get[3];				//娴嬮噺鍊�
    float err[3];				//璇樊
	
    
    float pout;							//p杈撳嚭
    float iout;							//i杈撳嚭
    float dout;							//d杈撳嚭
    
    float pos_out;						//鏈浣嶇疆寮忚緭鍑�
    float last_pos_out;				//涓婃杈撳嚭
    float delta_u;						//鏈澧為噺鍊�
    float delta_out;					//鏈澧為噺寮忚緭鍑� = last_delta_out + delta_u
    float last_delta_out;
    
	  float max_err;
	  float deadband;				//err < deadband return
    uint32_t pid_mode;
    uint32_t MaxOutput;				//杈撳嚭闄愬箙
    uint32_t IntegralLimit;		//绉垎闄愬箙

}pid_pkg;

void PID_struct_init(
	pid_pkg* pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,
    
    float 	kp, 
    float 	ki, 
    float 	kd);

float pid_calc(pid_pkg* pid, float fdb, float ref);
    

#endif

