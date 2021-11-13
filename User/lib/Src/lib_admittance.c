#include "lib_admittance.h"

admittance_t admt_x_diff_param, admt_y_diff_param;

admittance_t admt_left_hip_diff;
admittance_t admt_left_knee_diff;
admittance_t admt_right_hip_diff;
admittance_t admt_right_knee_diff;

admittance_t admt_left_x_diff, admt_left_x_diff_2;
admittance_t admt_left_y_diff, admt_left_y_diff_2;
admittance_t admt_right_x_diff, admt_right_x_diff_2;
admittance_t admt_right_y_diff, admt_right_y_diff_2;

void admit_params_init()
{
	admt_x_diff_param.Kd = 150.0;
	admt_x_diff_param.Bd = 3.0;

	admt_y_diff_param.Kd = 150.0;
	admt_y_diff_param.Bd = 2.0;

	admittance_struct_init(&admt_left_x_diff, 150.0, 3.0, 2.0, 0.0);
	admittance_struct_init(&admt_left_y_diff, 150.0, 2.0, 30.0, 0.0);
	admittance_struct_init(&admt_right_x_diff, 150.0, 3.0, 2.0, 0.0);
	admittance_struct_init(&admt_right_y_diff, 150.0, 2.0, 30.0, 0.0);

	admittance_struct_init(&admt_left_x_diff_2, 150.0, 3.0, 2.0, 0.0);
	admittance_struct_init(&admt_left_y_diff_2, 150.0, 2.0, 30.0, 0.0);
	admittance_struct_init(&admt_right_x_diff_2, 150.0, 3.0, 2.0, 0.0);
	admittance_struct_init(&admt_right_y_diff_2, 150.0, 2.0, 30.0, 0.0);

}

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

	if(admt->init_time < 50)
	{
		admt->init_time++;
		admt->xf = admt->x0;
		admt->ed = 0;
	}
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

float admittance_calc2(admittance_t* admt, float f_ext, float x0, float dot_x0, uint32_t t_now)
{
	admt->access_time_now = t_now;
	admt->process_time_interval = admt->access_time_now - admt->access_time_last;

	admt->f_ext = f_ext;
	admt->x0 = x0;
	admt->dot_x0 = 0.1 * dot_x0;
	if(admt->init_time < 50)
	{
		admt->init_time++;
		admt->xf = admt->x0;
		admt->ed = 0;
	}
	else {

		if(admt->process_time_interval >= 10)
		{
			admt->dot_ed = (admt->f_ext - admt->Kd * (admt->x0 - admt->xf))/admt->Bd;
			admt->dot_xf = admt->dot_x0 - admt->dot_ed;
			admt->xf = admt->xf + admt->dot_xf * ((float)admt->process_time_interval/1000);
			admt->ed = admt->x0 - admt->xf;
			abs_limit(&(admt->ed), admt->MaxOutput, 0);

			admt->access_time_last = t_now;
		}

	}

	return admt->ed;
}

void admt_param_change(void)
{
	// x axis
	admt_right_x_diff.Kd = admt_x_diff_param.Kd;
	admt_left_x_diff.Kd = admt_x_diff_param.Kd;

	admt_right_x_diff.Bd = admt_x_diff_param.Bd;
	admt_left_x_diff.Bd = admt_x_diff_param.Bd;

	// y axis
	admt_right_y_diff.Kd = admt_y_diff_param.Kd;
	admt_left_y_diff.Kd = admt_y_diff_param.Kd;

	admt_right_y_diff.Bd = admt_y_diff_param.Bd;
	admt_left_y_diff.Bd = admt_y_diff_param.Bd;

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

