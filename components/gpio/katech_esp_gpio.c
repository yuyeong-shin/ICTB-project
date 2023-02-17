//TODO: add code for your component here
#include "katech_esp_gpio.h"
#include "esp_log.h"


//static xQueueHandle gpio_evt_queue = NULL;
static SemaphoreHandle_t gpio_isr_sem;

bool cali_enable = 0;
static int adc_raw;
static esp_adc_cal_characteristics_t adc_chars;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
	uint32_t gpio_num = (uint32_t) arg;
	
	xSemaphoreGive(gpio_isr_sem);
//	xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL); 
	//printf("gpio num %d \r\n", gpio_num);
}

static void task_switch_mode_change(void* arg)
{
	//uint32_t io_num;
	static uint32_t isr_num = 0;
	static uint32_t isr_num_mem = 0;
	static uint32_t timer_mem = 0;
	static uint8_t flag = 0;
	
	for (;;) {
//		if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
		if (xSemaphoreTake(gpio_isr_sem, (TickType_t) portMAX_DELAY) == pdTRUE) {
			//printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
			printf("GPIO ISR num %d timer_cnt %d\r\n", isr_num++, g_timer_cnt);
			printf("timer_mem %d isr_num_mem %d\r\n", timer_mem, isr_num_mem);
			
			if (flag == 0)
			{
				timer_mem = g_timer_cnt;
				isr_num_mem = isr_num;
				flag = 1;
			}
			
			if (flag == 1)
			{
				if ((g_timer_cnt - timer_mem < 10) && (isr_num - isr_num_mem > 4))
				{
					isr_num = 0;
					flag = 0;
					timer_mem = 0;
					isr_num_mem = 0;
					printf("mode change \r\n");
				}
			}
			
		}
	}
}

static void task_adc(void* arg)
{

	for (;;)
	{
		if (xSemaphoreTake(sem_adc, (TickType_t) portMAX_DELAY) == pdTRUE)
		{
			adc_raw = adc1_get_raw(ADC1_CHANNEL_7);
//			ESP_LOGI("ADC_RAW", "raw data: %d", adc_raw);
			
			if (cali_enable)
			{
				voltage = esp_adc_cal_raw_to_voltage(adc_raw, &adc_chars) * 2;
				
				if (voltage <= 3400)
				{
					global_status = 4;
				}
				ESP_LOGI("ADC_VOLT", "cali data: %d mV", voltage);
			//	ESP_LOGI("ADC_BITWIDTH", "adc bit: %d", ADC_WIDTH_BIT_DEFAULT);
			}
		}
	}
}


void katech_esp_gpio_init(void)
{
	
	//zero-initialize the config structure.
	gpio_config_t io_conf = { };
	//interrupt of rising edge
	io_conf.intr_type = GPIO_INTR_POSEDGE;
	//bit mask of the pins, use GPIO4/5 here
	io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
	//set as input mode
	io_conf.mode = GPIO_MODE_INPUT;
	//enable pull-up mode
	io_conf.pull_up_en = 1;
	
	ESP_ERROR_CHECK(gpio_config(&io_conf));
	
	//create a queue to handle gpio event from isr
//	gpio_evt_queue = xQueueCreate(5, sizeof(uint32_t));
	gpio_isr_sem = xSemaphoreCreateBinary();
	//start gpio task
	xTaskCreate(task_switch_mode_change, "switch_mode_change", 2048, NULL, 10, NULL);
	
	//install gpio isr service
	ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT));
	
	ESP_ERROR_CHECK(gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1));
	

}

void katech_esp_adc_init(void)
{
	esp_err_t ret;
	
	ret = esp_adc_cal_check_efuse(ADC_EXAMPLE_CALI_SCHEME);
	if (ret == ESP_ERR_NOT_SUPPORTED)
	{
		ESP_LOGW("ADC", "Calibration scheme not supported, skip software calibration");
	}
	else if (ret == ESP_ERR_INVALID_VERSION) {
		ESP_LOGW("ADC", "eFuse not burnt, skip software calibration");
	}
	else if (ret == ESP_OK) {
		cali_enable = true;
//		esp_adc_cal_characterize(ADC_UNIT_1, ADC_EXAMPLE_ATTEN, ADC_WIDTH_BIT_DEFAULT, 0, &adc_chars);  // max 12bit adc ADC_WIDTH_12Bit
		esp_adc_cal_characterize(ADC_UNIT_1, ADC_EXAMPLE_ATTEN, ADC_WIDTH_12Bit, 0, &adc_chars); // max 12bit adc ADC_WIDTH_12Bit
	}
	else {
		ESP_LOGE("ADC", "Invalid arg");
	}
	
	//ADC config
//	ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_DEFAULT));
	ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_12Bit));
	ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_EXAMPLE_ATTEN));
	
	//start adc task
	xTaskCreate(task_adc, "task_adc", 2048, NULL, 10, NULL);
		
}