#include "robot_sense.h"


void robot_sensingTask(void *argument)
{
  /* USER CODE BEGIN robot_sensingTask */
  /* Infinite loop */
  for(;;)
  {
	  robot_sensing();
  }
  /* USER CODE END robot_sensingTask */
}
