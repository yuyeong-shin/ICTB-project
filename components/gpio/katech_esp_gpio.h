#pragma once

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#define GPIO_INPUT_IO_0		5
#define GPIO_INPUT_PIN_SEL	(1ULL << GPIO_INPUT_IO_0)
#define ESP_INTR_FLAG_DEFAULT 0

void katech_esp_gpio_init(void);
