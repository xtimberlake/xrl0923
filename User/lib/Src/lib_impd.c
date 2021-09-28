
/*
 * lib_pid.c
 *
 *  Created on: Sep 14, 2021
 *      Author: haoyun
 */
#include <lib_impd.h>



void impedance_struct_init(impedance_t* impd, float Kp, float Kd, long unsigned int max, float deadband)
{
  impd->Kp = Kp;
  impd->Kd = Kd;
  impd->MaxOutput = max;
  impd->deadband = deadband;

}

void abs_limit(float *a, float ABS_MAX,float offset)
{
	if(*a > ABS_MAX+offset)
			*a = ABS_MAX+offset;
	if(*a < -ABS_MAX+offset)
			*a = -ABS_MAX+offset;
}

float impedance_calc(impedance_t* impd, float get, float set, float d_get, float d_set)
{
  impd->get = get;
  impd->set = set;
  impd->err = set - get;	//set - measure
  if(ABS(impd->err) < impd->deadband) return 0;
  impd->d_get = d_get;
  impd->d_set = d_set;
  impd->d_err = d_set - d_get;	//set - measure

   impd->pout = impd->Kp * impd->err;
   impd->dout = impd->Kd * impd->d_err;

   impd->pos_out = impd->pout + impd->dout;

   abs_limit(&(impd->pos_out), impd->MaxOutput,0);
   impd->last_pos_out = impd->pos_out;	//update last time

  return impd->pos_out;
}
