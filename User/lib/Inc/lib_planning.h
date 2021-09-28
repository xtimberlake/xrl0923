#ifndef __LIB_PLANNING_H
#define __LIB_PLANNING_H

#include "main.h"
#include "math.h"
#include "lib_kinematics.h"


float* bezier_generate(float control_pts_x[], float control_pts_y[], uint16_t length, float u);
float ploy_B_u(uint16_t n, uint16_t i, float u);
uint16_t factorial_c(uint16_t n);
uint16_t combination_c(uint16_t n, uint16_t k);
float dot_c(float A[], float B[], uint16_t len);


#endif
