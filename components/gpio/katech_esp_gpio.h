#pragma once

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

//#include "driver/rmt.h"
//#include "led_strip.h"

#define GPIO_INPUT_IO_1		5
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_1))
#define ESP_INTR_FLAG_DEFAULT 0


u_int32_t g_timer_cnt;

void katech_esp_gpio_init(void);
