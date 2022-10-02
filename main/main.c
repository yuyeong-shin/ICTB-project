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

#include "id_open.h"
#include "nmea_parser.h"
#include "katech_esp_gpio.h"

#define STATS_TICKS         pdMS_TO_TICKS(1000)
#define PRIO_TASK_WIFI      3
#define PRIO_TASK_NMEA		2

#define TIME_ZONE			(+9)		// Korea
#define YEAR_BASE			(2000)		// data in GPS starts from 2000

#define ESP_TIMER_PERIOD	1000			// ms
static SemaphoreHandle_t sem_;
static SemaphoreHandle_t mut_;
	
struct UTM_parameters utm_parameters;
struct UTM_data utm_data;

static const char *TAG_gps = "gps_";

static void timer_callback(void *arg)
{
	g_timer_cnt++;
	xSemaphoreGive(sem_);
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

static void gps_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
	gps_t *gps = NULL;
//	printf("GPS Inttt\r\n");
	
	switch (event_id) {
	case GPS_UPDATE:
		gps = (gps_t *)event_data;
		/* print information parsed from GPS statements */
		ESP_LOGI(TAG_gps,
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
			gps->speed);
		utm_data.years = (int)gps->date.year + YEAR_BASE;
		utm_data.months = (int)gps->date.month;
		utm_data.days = (int)gps->date.day;
		utm_data.hours = (int)gps->tim.hour + TIME_ZONE;
		utm_data.minutes = (int)gps->tim.minute;
		utm_data.seconds = (int)gps->tim.second;
		utm_data.csecs = (int)gps->tim.thousand;
		
		utm_data.satellites = (int)gps->sats_in_use;
		
		utm_data.latitude_d = (double)gps->latitude;
		utm_data.longitude_d = (double)gps->longitude;
		utm_data.alt_agl_m = gps->altitude;
		utm_data.heading = (int)gps->cog;
		utm_data.speed_kn = (int)gps->speed;
			
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
	mut_ = xSemaphoreCreateRecursiveMutex();
	
	printf("Hello world!\n");
	
	// OpendroneID Init
	ID_OpenDrone_init();
	vTaskDelay(pdMS_TO_TICKS(100));
	
	memset(&utm_parameters, 0, sizeof(utm_parameters));
	strcpy(utm_parameters.UAS_operator, "FIN-OP-1234567");
	
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
    vTaskDelay(1000 / portTICK_PERIOD_MS);
	
	//xTaskCreatePinnedToCore(task_opendroneid_wifi, "wifi", 1024, NULL, PRIO_TASK_WIFI, NULL, 0);
	xTaskCreate(task_opendroneid_wifi, "WIFI", 4096, NULL, PRIO_TASK_WIFI, NULL);
	
	// GPS Test data
	utm_data.base_latitude  = 36.740751;
	utm_data.base_longitude = 127.119139;
	utm_data.base_alt_m     = 80.0;

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
