
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
uint16_t		global_status;

void Init_global_variables(void);



#endif