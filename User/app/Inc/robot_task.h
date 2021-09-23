#ifndef __ROBOT_TASK_H
#define __ROBOT_TASK_H

#include "main.h"
#include "lib_kinematics.h"
#include "lib_walking.h"
#include "lib_impd.h"
#include "lib_admittance.h"

#define LENGTH_HIP_LINK 625.0f
#define LENGTH_KNEE_LINK 520.0f

//定义机器人状态
typedef enum
{
  INITIALIZING,
  WORKING

}robot_state_e;

typedef struct
{
  float x,y;
  float x_ref,y_ref;
  float x_cmd,y_cmd;
  float fx, fy;
  float fx_ref, fy_ref;

} leg_side_t;

typedef struct
{
  robot_state_e state, last_state; //机器人运行状态

  walkingPara_TypeDef walkingParam;


  leg_side_t leftLeg;
  leg_side_t rightLeg;

  const double *imu;

} robot_TypeDef;
robot_TypeDef robot;


void robot_task(void *argument);
void robot_sensing(void);
void robot_acting(void);
void robot_impedance_ctrller(void);
void robot_admittance_ctrller(void);
void robot_leg_admittance_ctrller(void);

//lib_kinematics.c文件的函数
extern void leg_forward_kinematics(leg_side_t* l, float L1, float L2, float phi_1, float phi_2);

#endif
