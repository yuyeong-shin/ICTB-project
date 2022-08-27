//TODO: add code for your component here
#include "katech_esp_gpio.h"
#include "esp_log.h"

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
	uint32_t gpio_num = (uint32_t) arg;
	
	printf("gpio num %d \r\n", gpio_num);
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
	
	gpio_config(&io_conf);
	
	//change gpio intrrupt type for one pin rising edge
	gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_POSEDGE);
	
	//install gpio isr service
	gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
	//hook isr handler for specific gpio pin
	gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
	
}
