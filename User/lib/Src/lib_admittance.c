#include "lib_admittance.h"

void admittance_struct_init(admittance_t* admt, float Kd, float Bd, float MaxOutput, float deadband)
{
	admt->Kd = Kd;
	admt->Bd = Bd;
	admt->MaxOutput = MaxOutput;
	admt->deadband = deadband;
	admt->delte_t = 0.004;
}

float admittance_calc(admittance_t* admt, float f_ext, float x0)
{
	admt->f_ext = f_ext;
	admt->x0 = x0;
//	if(ABS(admt->f_ext) < admt->deadband)
//	{
//		ramp_t rp = {admt->ed, 0.0, 0.002, -0.3, 0.3, 0.0}; //输入数据 输出数据 速度 限幅最小值 限幅最大值 输出
//		admt->ed = ramp_calc(&rp, admt->ed, 0.0);
//		admt->xf = admt->x0;
//		admt->dot_ed = 0.0;
//		//admt->ed = 0.0;
//		return admt->ed;
//	}

	admt->dot_ed = (admt->f_ext - admt->Kd * (admt->x0 - admt->xf))/admt->Bd;
	admt->ed = admt->dot_ed * admt->delte_t + (admt->x0 - admt->xf);
	abs_limit(&(admt->ed), admt->MaxOutput, 0);
	admt->xf = admt->x0 - admt->ed;


  return admt->ed;
}

float ramp_calc(ramp_t *rp, float src, float dest)
{
	rp->src = src;
	rp->dest = dest;
	if(rp->src < rp->dest)
	{
		rp->output = rp->src + rp->rate;
	}
	else if(rp->src > rp->dest)
	{
		rp->output = rp->src - rp->rate;
	}
	abs_limit(&(rp->output), rp->max_value, 0);

	return rp->output;

}

