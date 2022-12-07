/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "global_variables.h"

#include "id_open.h"
#include "nmea_parser.h"
#include "katech_esp_gpio.h"

#include "driver/rmt.h"
#include "led_strip.h"

// FreeRTOS SET
#define STATS_TICKS         pdMS_TO_TICKS(1000)
#define PRIO_TASK_LED		6
#define PRIO_TASK_WIFI      3
#define PRIO_TASK_NMEA		2

// GPS TIME SET
#define TIME_ZONE			(+9)		// Korea
#define YEAR_BASE			(2000)		// data in GPS starts from 2000

// MCU TIMER SET
#define ESP_TIMER_PERIOD	100			// ms

// for RGB LED SET
#define CONFIG_EXAMPLE_RMT_TX_GPIO 48
#define CONFIG_EXAMPLE_STRIP_LED_NUMBER 24

#define RMT_TX_CHANNEL RMT_CHANNEL_0

#define EXAMPLE_CHASE_SPEED_MS (1000)


static SemaphoreHandle_t sem_;
static SemaphoreHandle_t sem_led;
static SemaphoreHandle_t mut_;
	
static led_strip_t *strip;

struct UTM_parameters utm_parameters;
struct UTM_data utm_data;

static const char *TAG_gps = "gps_";

void init_rmt_led(void);
void led_strip_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b);

static void timer_callback(void *arg)
{
	g_timer_cnt++;
	if (g_timer_cnt % 10 == 1)
	{
		xSemaphoreGive(sem_);	
	}
	else if (g_timer_cnt % 10 == 3)
	{
		xSemaphoreGive(sem_led);
	}
	
	if (g_timer_cnt % 100 == 5)
	{
		xSemaphoreGive(sem_adc);
	}
	
}

static void timer_init(void)
{
	const esp_timer_create_args_t timer_args = {
		.callback = &timer_callback,
		.name = "Timer"
	};
	
	esp_timer_handle_t timer_handle;
	ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer_handle));
	
	//start timer
	ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handle, ESP_TIMER_PERIOD * 1000));
}
static void task_opendroneid_wifi(void *arg)
{
	//static int take_count = 0;
	
	while (1)
	{
		if (xSemaphoreTake(sem_, (TickType_t) 3000) == pdTRUE)
		{
			
			xSemaphoreTakeRecursive(mut_, (TickType_t) 10);
//			printf("Semaphore take count: %d, Tick: %d\r\n", take_count++, xTaskGetTickCount());
//			printf("%.7f\r\n", utm_data.latitude_d);
			ID_OpenDrone_transmit(&utm_data);
			//ID_OpenDrone_transmit_wifi(&utm_data);
			xSemaphoreGiveRecursive(mut_);
			//ID_OpenDrone_transmit_wifi(&utm_data);
			
		}
		else
		{
			printf("Semaphore don't take\r\n");
		}
		//printf("wifi_task\r\n");
		
		
		
		
		//vTaskDelay(pdMS_TO_TICKS(1000));	
	}
	
}

static void task_led_status(void *arg)
{
	//static int take_count = 0;
	
	while (1)
	{
		if (xSemaphoreTake(sem_led, (TickType_t) 3000) == pdTRUE)
		{
			
					
		}
		else
		{
			printf("Semaphore don't take\r\n");
		}
		//printf("wifi_task\r\n");
		
		
		
		
		//vTaskDelay(pdMS_TO_TICKS(1000));	
	}
	
}
static void gps_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
	gps_t *gps = NULL;
//	printf("GPS Inttt\r\n");
	
	switch (event_id) {
	case GPS_UPDATE:
		gps = (gps_t *)event_data;
		/* print information parsed from GPS statements */
/*		ESP_LOGI(TAG_gps,
			"%d/%d/%d %d:%d:%d => \r\n"
		         "\t\t\t\t\t\tlatitude   = %.05f°N\r\n"
		         "\t\t\t\t\t\tlongitude = %.05f°E\r\n"
		         "\t\t\t\t\t\taltitude   = %.02fm\r\n"
		         "\t\t\t\t\t\tspeed      = %fm/s",
			gps->date.year + YEAR_BASE,
			gps->date.month,
			gps->date.day,
			gps->tim.hour + TIME_ZONE,
			gps->tim.minute,
			gps->tim.second,
			gps->latitude,
			gps->longitude,
			gps->altitude,
			gps->speed);*/
//		utm_data.years = (int)gps->date.year + YEAR_BASE;
//		utm_data.months = (int)gps->date.month;
//		utm_data.days = (int)gps->date.day;
//		utm_data.hours = (int)gps->tim.hour + TIME_ZONE;
//		utm_data.minutes = (int)gps->tim.minute;
//		utm_data.seconds = (int)gps->tim.second;
//		utm_data.csecs = (int)gps->tim.thousand;
//		
//		utm_data.satellites = (int)gps->sats_in_use;
//		
//		utm_data.latitude_d = (double)gps->latitude;
//		utm_data.longitude_d = (double)gps->longitude;
//		utm_data.alt_agl_m = gps->altitude;
//		utm_data.heading = (int)gps->cog;
//		utm_data.speed_kn = (int)gps->speed;
		
		utm_data.years = 2022;
		utm_data.months = 10;
		utm_data.days = 6;
		utm_data.hours = 7 + TIME_ZONE;
		utm_data.minutes = 32;
		utm_data.seconds = 12;
		utm_data.csecs = 123;
		
		utm_data.satellites = 1;
		
		// test module No:01
//		utm_data.latitude_d = 37.5060638;
//		utm_data.longitude_d = 126.8726285;
		// test module No:02
//		utm_data.latitude_d = 37.5094697;
//		utm_data.longitude_d = 126.8821053;
		// test module No:03
		//utm_data.latitude_d = 37.5032568;
		//utm_data.longitude_d = 126.8743027;
		// test module No:04
		//utm_data.latitude_d = 37.5022703;
		//utm_data.longitude_d = 126.8744127;
		// test module No:05
		utm_data.latitude_d = 37.5047802;
		utm_data.longitude_d = 126.8729027;
		utm_data.alt_agl_m = 23;
		utm_data.alt_msl_m = 12;
		utm_data.base_alt_m = 30;
		utm_data.heading = 144;
		utm_data.speed_kn = 33;
			
		break;
	case GPS_UNKNOWN:
		/* print unknown statements */
		ESP_LOGW(TAG_gps, "Unknown statement:%s", (char *)event_data);
		break;
	default:
		break;
	}
}

void app_main(void)
{
	//TickType_t tick = 0;
	
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);
    
	vTaskDelay(pdMS_TO_TICKS(100));
		
	sem_ = xSemaphoreCreateBinary();
	sem_led = xSemaphoreCreateBinary();
	mut_ = xSemaphoreCreateRecursiveMutex();
	
	printf("Hello world!\n");
	
	Init_global_variables();
		
	// OpendroneID Init
	ID_OpenDrone_init();
	vTaskDelay(pdMS_TO_TICKS(100));
	
	memset(&utm_parameters, 0, sizeof(utm_parameters));
	strcpy(utm_parameters.UAS_operator, "KATECH_ODID_No:05");
	
	utm_parameters.region			= 2;
	utm_parameters.EU_category		= 2;
	
	ID_OpenDrone_param_init(&utm_parameters);
	
	memset(&utm_data, 0, sizeof(utm_data));
	vTaskDelay(pdMS_TO_TICKS(100));
	
	// for GPS data
	nmea_parser_config_t config = NMEA_PARSER_CONFIG_DEFAULT();
	nmea_parser_handle_t nmea_hdl = nmea_parser_init(&config);
	
	nmea_parser_add_handler(nmea_hdl, gps_event_handler, NULL);
	
	// for Switch Input (GPIO interrupt)
	katech_esp_gpio_init();

	// for Battery ADC 
	katech_esp_adc_init();
	// Timer Start
	timer_init();
	
    /* Print chip information */
/*    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), WiFi%s%s, ",
            CONFIG_IDF_TARGET,
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());
*/
	
	init_rmt_led();
	
	//xTaskCreatePinnedToCore(task_opendroneid_wifi, "wifi", 1024, NULL, PRIO_TASK_WIFI, NULL, 0);
	xTaskCreate(task_opendroneid_wifi, "WIFI", 4096, NULL, PRIO_TASK_WIFI, NULL);
	
	xTaskCreate(task_led_status, "LED_STATUS", 2048, NULL, PRIO_TASK_LED, NULL);
	
	// GPS Test data
	utm_data.base_latitude  = 37.5047627;
	utm_data.base_longitude = 126.8750924; //STX W Tower
	utm_data.base_alt_m     = 30.0;
	
	utm_data.base_valid = 1;
	
	while (1)
	{
		
//		tick = xTaskGetTickCount();
		
		//ID_OpenDrone_transmit(&utm_data);
//		printf("main %d\r\n", tick);
		//vTaskDelay(pdMS_TO_TICKS(1000));
//		__asm__ __volatile__("NOP");
	}
	
    

}

void init_rmt_led(void)
{
	rmt_config_t rmtconfig = RMT_DEFAULT_CONFIG_TX(CONFIG_EXAMPLE_RMT_TX_GPIO, RMT_TX_CHANNEL);
	// set counter clock to 40MHz
	rmtconfig.clk_div = 2;

	ESP_ERROR_CHECK(rmt_config(&rmtconfig));
	ESP_ERROR_CHECK(rmt_driver_install(rmtconfig.channel, 0, 0));

	// install ws2812 driver
	led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(CONFIG_EXAMPLE_STRIP_LED_NUMBER, (led_strip_dev_t)rmtconfig.channel);
	strip = led_strip_new_rmt_ws2812(&strip_config);
	if (!strip) {
		
	}
	// Clear LED strip (turn off all LEDs)
	ESP_ERROR_CHECK(strip->clear(strip, 100));
	
		
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	
	ESP_ERROR_CHECK(strip->set_pixel(strip, 0, 0, 180/20, 0/20));
	ESP_ERROR_CHECK(strip->refresh(strip, 100));
}

void led_strip_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b)
{
	h %= 360; // h -> [0,360]
	uint32_t rgb_max = v * 2.55f;
	uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;

	uint32_t i = h / 60;
	uint32_t diff = h % 60;

	// RGB adjustment amount by hue
	uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

	switch (i) {
	case 0:
		*r = rgb_max;
		*g = rgb_min + rgb_adj;
		*b = rgb_min;
		break;
	case 1:
		*r = rgb_max - rgb_adj;
		*g = rgb_max;
		*b = rgb_min;
		break;
	case 2:
		*r = rgb_min;
		*g = rgb_max;
		*b = rgb_min + rgb_adj;
		break;
	case 3:
		*r = rgb_min;
		*g = rgb_max - rgb_adj;
		*b = rgb_max;
		break;
	case 4:
		*r = rgb_min + rgb_adj;
		*g = rgb_min;
		*b = rgb_max;
		break;
	default:
		*r = rgb_max;
		*g = rgb_min;
		*b = rgb_max - rgb_adj;
		break;
	}
}