#pragma once

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#include "../global_variables/global_variables.h"

//#include "driver/rmt.h"
//#include "led_strip.h"

#define GPIO_INPUT_IO_1		5
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_1))
#define ESP_INTR_FLAG_DEFAULT 0

//ADC Attenuation
#define ADC_EXAMPLE_ATTEN           ADC_ATTEN_DB_11		//for 0 ~ 3100mV
//ADC Calibration
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP_FIT

u_int32_t g_timer_cnt;

void katech_esp_gpio_init(void);
void katech_esp_adc_init(void);
