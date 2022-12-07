//TODO: add code for your component here
#include "katech_esp_gpio.h"
#include "esp_log.h"


//static xQueueHandle gpio_evt_queue = NULL;
static SemaphoreHandle_t gpio_isr_sem;

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
