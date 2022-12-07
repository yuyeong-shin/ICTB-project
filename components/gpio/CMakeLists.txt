#Created by VisualGDB. Right-click on the component in Solution Explorer to edit properties using convenient GUI.

idf_component_register(
	SRCS katech_esp_gpio.c
	INCLUDE_DIRS "."
	PRIV_INCLUDE_DIRS
		../../../../../../../../SysGCC/esp32/esp-idf/v4.4.1/components/esp_adc_cal/include
		../../../../../../../../SysGCC/esp32/esp-idf/v4.4.1/components/fatfs/test_fatfs_host/sdkconfig
		../../../../../../../../SysGCC/esp32/esp-idf/v4.4.1/components/spiffs/spiffs/afltests
		../../../../../../../../SysGCC/esp32/esp-idf/v4.4.1/components/tinyusb/tinyusb/hw/bsp/esp32s2/components/led_strip/include)
