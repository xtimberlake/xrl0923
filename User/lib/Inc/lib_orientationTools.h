#ifndef LIB_ORIENTATIONTOOL_H
#define LIB_ORIENTATIONTOOL_H

#include "main.h"
#include "arm_math.h"
#include "math.h"

 float square(float a);
 void quatToRPY(
  const arm_matrix_instance_f32 * q,
  arm_matrix_instance_f32 * rpy);
 void quaternionToRotationMatrix(
		 const arm_matrix_instance_f32 * q,
		 arm_matrix_instance_f32 * rB);
#endif
