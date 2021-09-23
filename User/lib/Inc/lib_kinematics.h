#ifndef __LIB_KINEMATICS_H
#define __LIB_KINEMATICS_H

#include "robot_task.h"
#include "math.h"

enum
{
	HIP = 0,
	KNEE = 1
};

enum
{
	X = 0,
	Y = 1
};


float* inverse_kinematics(float L1, float L2, float x_ref, float z_ref);
float* contact_force_estimate(float theta1, float theta2, float tau1, float tau2, float a1, float a2);

#endif
