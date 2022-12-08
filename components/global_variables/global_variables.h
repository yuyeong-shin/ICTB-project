
#ifndef GLOBAL_VARIABLES_H
#define GLOBAL_VARIABLES_H

#include <stdio.h>
#include <stdlib.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_system.h"

SemaphoreHandle_t sem_adc;

/*
	0 : init -> white LED
	1 : normal operation -> green LED
	2 : no user data, no user data operation -> blue LED blink
	3 : user setting mode -> blue LED
	4 : battery low -> Red LED
*/
uint16_t		global_status;


// ADC 
uint32_t voltage;
void Init_global_variables(void);



#endif
