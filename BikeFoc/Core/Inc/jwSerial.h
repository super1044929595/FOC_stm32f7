#ifndef JW_SERIAL_H
#define JW_SERIAL_H





#include "main.h"
#include "cmsis_os.h"
#include "stm32f7xx_hal.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "jwUnite.h"
#include "main.h"


typedef struct{

const char *name;

bool (*communication_Init)(void);

bool (*communication_Reset)(void);

void (*communication_callbackregister)(void);

void (*communication_power_onoff)(bool onoff);

void (*communication_interrupt_register)(void);

}Communicatopn_InitTypedef ;






bool jw_communication_Init(void);











#endif
