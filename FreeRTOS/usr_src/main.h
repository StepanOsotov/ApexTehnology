#ifndef __MAIN_H__
#define __MAIN_H__
//--------------------------------------------------------------------------------
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//--------------------------------------------------------------------------------

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "portmacro.h"
#include "semphr.h"
#include "projdefs.h"
#include "croutine.h"
#include "event_groups.h"
#include "message_buffer.h"

//--------------------------------------------------------------------------------
#include "stm32l1xx.h"
#include "stm32l1xx_gpio.h"
#include "stm32l1xx_rcc.h"
#include "stm32l1xx_tim.h"
#include "stm32l1xx_usart.h"
#include "misc.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//--------------------------------------------------------------------------------

#include "usart2_usr.h"
#include "usart3_usr.h"
#include "init_periph.h"
#include "user_tasks.h"
#include "delay_us.h"
#include "stm32l1xx_conf.h"

//#define DWT_WORK

//--------------------------------------------------------------------------------

#endif // __MAIN_H__
