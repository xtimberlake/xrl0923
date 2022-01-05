#include "lib_orientationTools.h"

 void quatToRPY(
  const arm_matrix_instance_f32 * q,
  arm_matrix_instance_f32 * rpy)
 {

	 float as = fminf(-2.0f * (q->pData[1] * q->pData[3] - q->pData[0] * q->pData[2]), .99999);
	 rpy->pData[2] = atan2f(square(q->pData[0]) + square(q->pData[1]) - square(q->pData[2]) - square(q->pData[3]), \
			 2.0f * (q->pData[1] * q->pData[2] + q->pData[0] * q->pData[3]));
	 rpy->pData[1] = asinf(as);
	 rpy->pData[0] = atan2f(square(q->pData[0]) - square(q->pData[1]) - square(q->pData[2]) + square(q->pData[3]), \
			 2 * (q->pData[2] * q->pData[3] + q->pData[0] * q->pData[1]));

 }

 void quaternionToRotationMatrix(
		 const arm_matrix_instance_f32 * q,
		 arm_matrix_instance_f32 * rB)
 {
	  float32_t e0 = q->pData[0];
	  float32_t e1 = q->pData[1];
	  float32_t e2 = q->pData[2];
	  float32_t e3 = q->pData[3];

	  float32_t zeros[9] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	  arm_matrix_instance_f32 Z;
	  arm_mat_init_f32(&Z, 3, 3, zeros);

	  Z.pData[0] = 1 - 2 * (e2 * e2 + e3 * e3);
	  Z.pData[1] = 2 * (e1 * e2 - e0 * e3);
	  Z.pData[2] = 2 * (e1 * e3 + e0 * e2);
	  Z.pData[3] = 2 * (e1 * e2 + e0 * e3);
	  Z.pData[4] = 1 - 2 * (e1 * e1 + e3 * e3);
	  Z.pData[5] = 2 * (e2 * e3 - e0 * e1);
	  Z.pData[6] = 2 * (e1 * e3 - e0 * e2);
	  Z.pData[7] = 2 * (e2 * e3 + e0 * e1);
	  Z.pData[8] = 1 - 2 * (e2 * e2 + e3 * e3);

	  arm_mat_trans_f32(&Z, rB);
 }

 float square(float a) {
   return a * a;
 }
