/*
 * xrl_kinematics.c
 *
 *  Created on: Sep 11, 2021
 *      Author: haoyun
 */

#include "lib_kinematics.h"

void leg_forward_kinematics(leg_side_t* l, float L1, float L2, float phi_1, float phi_2)
{
	float l_3, theta_3, theta_2;
	theta_3 = M_PI - (phi_1 + phi_2);
	l_3 = sqrt(L1*L1 + L2*L2 - 2*L1*L2*cos(theta_3));
	theta_2 = asin(L2*sin(theta_3)/l_3);

	float left_theta_target = M_PI/2 - (theta_2 - phi_1);
	l->x = l_3 * cos(left_theta_target);
	l->y = l_3 * sin(left_theta_target);
}

float* inverse_kinematics(float L1, float L2, float x_ref, float z_ref)
{

	  float left_l_3, left_theta_2, left_theta_triangle, theta_3;
	  float target_phi_1, target_phi_2;

	  left_l_3 = sqrt(x_ref*x_ref + z_ref*z_ref);
	  left_theta_2 = acos((L1*L1 + left_l_3*left_l_3 - L2*L2)/(2*L1*left_l_3));
	  left_theta_triangle = atan(x_ref/z_ref);

	  target_phi_1 = left_theta_2 - left_theta_triangle;

	  theta_3 = acos((L1*L1 + L2*L2 - left_l_3*left_l_3)/(2*L1*L2));
	  target_phi_2 = M_PI - target_phi_1 - theta_3;

	  static float target_angle[2];
	  target_angle[0] = target_phi_1;
	  target_angle[1] = target_phi_2;

	  return target_angle;

}

float* contact_force_estimate(float theta1, float theta2, float tau1, float tau2, float a1, float a2)
{
	float c1, s1, s2, s1_2, c1_2;
	c1 = cos(theta1);
	s1 = sin(theta1);
	s2 = sin(theta2);
	s1_2 = sin(theta1 - theta2);
	c1_2 = cos(theta1 - theta2);

	float J_T_INV[2][2];
	J_T_INV[0][0] = -c1_2 / (a1 * s2);
	J_T_INV[0][1] = (a1 * c1 + a2 * c1_2) / (a1 * a2 * s2);
	J_T_INV[1][0] = -s1_2 / (a1 * s2);
	J_T_INV[1][1] = (a1 * s1 + a2 * s1_2) / (a1 * a2 * s2);

	static float f[2];
	f[0] = J_T_INV[0][0] * tau1 + J_T_INV[0][1] * tau2;
	f[1] = J_T_INV[1][0] * tau1 + J_T_INV[1][1] * tau2;

	return f;

}

