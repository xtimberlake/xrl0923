/*
 * lib_planning.c
 *
 *  Created on: Sep 27, 2021
 *      Author: haoyun
 */
#include "lib_planning.h"

/*
 * Function: bezier_generate
 * Description: generate Bezier Curve
 * 				ref: https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/Bezier/bezier-construct.html
 *
 * Parameter 1: x of control point(s)
 * Parameter 2: y of control point(s)
 * Parameter 3: number of control points
 * Parameter 3: the factor in [0, 1]
 * Return : current point coordinate & velocity
 */
float* bezier_generate(float control_pts_x[], float control_pts_y[], uint16_t length, float u)
{
	static float pt[4];
	uint16_t n = length - 1; // the order of Bezier curve

	float B_n[length];
	float B_n_n[length-1];
	for (uint16_t i = 0; i < length; i++)
	{
		B_n[i] = ploy_B_u(n, i, u);
	}

	float diff_x[length-1];
	float diff_y[length-1];
	for (uint16_t i = 0; i < length-1; i++)
	{
			B_n_n[i] = n * B_n[i];
			diff_x[i] = control_pts_x[i+1] - control_pts_x[i];
			diff_y[i] = control_pts_y[i+1] - control_pts_y[i];
	}


	pt[X] = dot_c(control_pts_x, B_n, length);
	pt[Y] = dot_c(control_pts_y, B_n, length);
	pt[DX] = dot_c(diff_x, B_n_n, length-1);
	pt[DY] = dot_c(diff_y, B_n_n, length-1);


	return pt;

}

float ploy_B_u(uint16_t n, uint16_t i, float u)
{
	return combination_c(n, i) * powf(u, i) * powf((1 - u), (n-i));
}

/*
 * Function: factorial_c
 * Description: calculate (n!) in C language
 *
 * Return : n!
 */
uint16_t factorial_c(uint16_t n)
{
	uint16_t ret = 1;
	for (int16_t i = 1; i < n + 1; i++) {
		ret *= i;
	}
	return ret;
}

/*
 * Function: combination_c
 * Description: calculate C^k_n in C language
 *
 * Return : C^k_n
 */
uint16_t combination_c(uint16_t n, uint16_t k)
{
	return factorial_c(n)/(factorial_c(n-k) * factorial_c(k));
}

/*
 * Function: dot_c
 * Description: dot product of A & B
 *
 * Return : <A,B>
 */
float dot_c(float A[], float B[], uint16_t len)
{
	float ret = 0;

	for (uint16_t i  = 0; i < len; i++ ) {
		ret += A[i] * B[i];
	}
	return ret;

}

