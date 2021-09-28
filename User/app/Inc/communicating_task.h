#ifndef __COMMUNICATING_TASK_H
#define __COMMUNICATION_TASK_H

#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "usart.h"
#include "main.h"
#include "common_debug.h"
#include "math.h"

#define CMD_SEND_FLAG  (1 << 0)

typedef struct
{
  float           vx; // forward/back
  float           vy; // left/right
  float           vw; // rotate

} chassis_t;



uint8_t* from_float_to_uint(float f);
void communicatingTask(void *argument);
void ubuntu_receive_callback(uint8_t *rx_buff, uint16_t length);

#endif
