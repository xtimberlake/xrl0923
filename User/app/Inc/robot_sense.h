#ifndef __ROBOT_SENSE_H
#define __ROBOT_SENSE_H

#include "main.h"
#include "tim.h"
#include "robot_task.h"

void robot_sensing(uint32_t t);
void contact_event_estimator(void);

#endif
