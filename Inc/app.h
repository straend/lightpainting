#ifndef _APP__H_
#define _APP__H_
#include "stm32f4xx_hal.h"

typedef enum {
  APPLICATION_IDLE = 0,  
  APPLICATION_START,   
  APPLICATION_READY,
  APPLICATION_DISCONNECT,
  APPLICATION_SELECT_FILE,
  APPLICATION_DELAY,
}ApplicationTypeDef;


ApplicationTypeDef Appli_state;

#endif