#ifndef __CDC_H__
#define __CDC_H__

#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "main.h"
#include "semphr.h"

#include "INS.h"




extern void CDC_Init(void);
extern void CDC_Receive(void);

#endif