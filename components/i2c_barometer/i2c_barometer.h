#ifndef I2C_BARO_H
#define I2C_BARO_H


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "driver/i2c.h"

#include "../global_variables/global_variables.h"

#define CONFIG_I2CDEV_NOLOCK		1

#define CONFIG_I2CDEV_TIMEOUT   1000
#define MS5611_ADDR_CSB_HIGH    0x76
#define MS5611_ADDR_CSB_LOW     0x77

#define I2C_FREQ_HZ 400000 // 400 kHz

#define CMD_CONVERT_D1 0x40
#define CMD_CONVERT_D2 0x50
#define CMD_ADC_READ   0x00
#define CMD_RESET      0x1E

#define PROM_ADDR_SENS     0xa2
#define PROM_ADDR_OFF      0xa4
#define PROM_ADDR_TCS      0xa6
#define PROM_ADDR_TCO      0xa8
#define PROM_ADDR_T_REF    0xaa
#define PROM_ADDR_TEMPSENS 0xac

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

#define CONFIG_I2C_MASTER_SCL		19
#define CONFIG_I2C_MASTER_SDA		18
#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

/**
 * I2C device descriptor
 */
typedef struct
{
	i2c_port_t port; //!< I2C port number
	i2c_config_t cfg; //!< I2C driver configuration
	uint8_t addr; //!< Unshifted address
	SemaphoreHandle_t mutex; //!< Device mutex
	uint32_t timeout_ticks; /*!< HW I2C bus timeout (stretch time), in ticks. 80MHz APB clock
	                              ticks for ESP-IDF, CPU ticks for ESP8266.
	                              When this value is 0, I2CDEV_MAX_STRETCH_TIME will be used */
} i2c_dev_t;

typedef enum {
	I2C_DEV_WRITE = 0,
	/**< Write operation */
	I2C_DEV_READ       /**< Read operation */
} i2c_dev_type_t;

/**
 * Oversampling ratio
 */
typedef enum
{
	MS5611_OSR_256 = 0x00,
	//!< 256 samples per measurement
	MS5611_OSR_512 = 0x02,
	//!< 512 samples per measurement
	MS5611_OSR_1024 = 0x04,
	//!< 1024 samples per measurement
	MS5611_OSR_2048 = 0x06,
	//!< 2048 samples per measurement
	MS5611_OSR_4096 = 0x08  //!< 4096 samples per measurement
} ms5611_osr_t;

/**
 * Configuration data
 */
typedef struct
{
	uint16_t sens; //!< C1 Pressure sensitivity                             | SENS_t1
	uint16_t off; //!< C2 Pressure offset                                  | OFF_t1
	uint16_t tcs; //!< C3 Temperature coefficient of pressure sensitivity  | TCS
	uint16_t tco; //!< C4 Temperature coefficient of pressure offset       | TCO
	uint16_t t_ref; //!< C5 Reference temperature                            | T_ref
	uint16_t tempsens; //!< C6 Temperature coefficient of the temperature       | TEMPSENSE
} ms5611_config_data_t;

/**
 * Device descriptor
 */
typedef struct
{
	i2c_dev_t i2c_dev; //!< I2C device settings
	ms5611_osr_t osr; //!< Oversampling setting
	ms5611_config_data_t config_data; //!< Device configuration, filled upon initialize
} ms5611_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev Device descriptor
 * @param addr I2C address, `MS5611_ADDR_CSB_HIGH` or `MS5611_ADDR_CSB_LOW`
 * @param port I2C port
 * @param sda_gpio GPIO pin for SDA
 * @param scl_gpio GPIO pin for SCL
 * @return `ESP_OK` on success
 */
esp_err_t ms5611_init_desc(ms5611_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t ms5611_free_desc(ms5611_t *dev);

/**
 * @brief Init MS5611-01BA03
 *
 * Reset device and read calibration data
 *
 * @param dev Device descriptor
 * @param osr Oversampling ratio
 * @return `ESP_OK` on success
 */
esp_err_t ms5611_init(ms5611_t *dev, ms5611_osr_t osr);

/**
 * @brief Measure pressure and temperature
 *
 * @param dev Device descriptor
 * @param[out] pressure Pressure, Pa
 * @param[out] temperature Temperature, degrees Celsius
 * @return `ESP_OK` on success
 */
esp_err_t ms5611_get_sensor_data(ms5611_t *dev, int32_t *pressure, float *temperature);

#endif
