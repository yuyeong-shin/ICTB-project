//TODO: add code for your component here
#include "i2c_barometer.h"
#include "esp_log.h"

static const char *TAG = "I2C_BARO";

typedef struct {
	SemaphoreHandle_t lock;
	i2c_config_t config;
	bool installed;
} i2c_port_state_t;

static i2c_port_state_t states[I2C_NUM_MAX];

#if CONFIG_I2CDEV_NOLOCK
#define SEMAPHORE_TAKE(port)
#else
#define SEMAPHORE_TAKE(port) do { \
        if (!xSemaphoreTake(states[port].lock, pdMS_TO_TICKS(CONFIG_I2CDEV_TIMEOUT))) \
        { \
            ESP_LOGE(TAG, "Could not take port mutex %d", port); \
            return ESP_ERR_TIMEOUT; \
        } \
        } while (0)
#endif
	        
#if CONFIG_I2CDEV_NOLOCK
#define SEMAPHORE_GIVE(port)
#else
#define SEMAPHORE_GIVE(port) do { \
        if (!xSemaphoreGive(states[port].lock)) \
        { \
            ESP_LOGE(TAG, "Could not give port mutex %d", port); \
            return ESP_FAIL; \
        } \
        } while (0)
#endif

esp_err_t i2cdev_init()
{
	memset(states, 0, sizeof(states));

#if !CONFIG_I2CDEV_NOLOCK
	for (int i = 0; i < I2C_NUM_MAX; i++)
	{
		states[i].lock = xSemaphoreCreateMutex();
		if (!states[i].lock)
		{
			ESP_LOGE("I2C_BARO", "Could not create port mutex %d", i);
			return ESP_FAIL;
		}
	}
#endif
	
	return ESP_OK;
}


esp_err_t i2cdev_done()
{
	for (int i = 0; i < I2C_NUM_MAX; i++)
	{
		if (!states[i].lock) continue;

		if (states[i].installed)
		{
			SEMAPHORE_TAKE(i);
			i2c_driver_delete(i);
			states[i].installed = false;
			SEMAPHORE_GIVE(i);
		}
#if !CONFIG_I2CDEV_NOLOCK
		vSemaphoreDelete(states[i].lock);
#endif
		states[i].lock = NULL;
	}
	return ESP_OK;
}


esp_err_t i2c_dev_create_mutex(i2c_dev_t *dev)
{
#if !CONFIG_I2CDEV_NOLOCK
	if (!dev) return ESP_ERR_INVALID_ARG;

	ESP_LOGV(TAG, "[0x%02x at %d] creating mutex", dev->addr, dev->port);

	dev->mutex = xSemaphoreCreateMutex();
	if (!dev->mutex)
	{
		ESP_LOGE(TAG, "[0x%02x at %d] Could not create device mutex", dev->addr, dev->port);
		return ESP_FAIL;
	}
#endif

	return ESP_OK;
}


esp_err_t i2c_dev_delete_mutex(i2c_dev_t *dev)
{
#if !CONFIG_I2CDEV_NOLOCK
	if (!dev) return ESP_ERR_INVALID_ARG;

	ESP_LOGV(TAG, "[0x%02x at %d] deleting mutex", dev->addr, dev->port);

	vSemaphoreDelete(dev->mutex);
#endif
	return ESP_OK;
}

esp_err_t i2c_dev_take_mutex(i2c_dev_t *dev)
{
#if !CONFIG_I2CDEV_NOLOCK
	if (!dev) return ESP_ERR_INVALID_ARG;

	ESP_LOGV(TAG, "[0x%02x at %d] taking mutex", dev->addr, dev->port);

	if (!xSemaphoreTake(dev->mutex, pdMS_TO_TICKS(CONFIG_I2CDEV_TIMEOUT)))
	{
		ESP_LOGE(TAG, "[0x%02x at %d] Could not take device mutex", dev->addr, dev->port);
		return ESP_ERR_TIMEOUT;
	}
#endif
	return ESP_OK;
}

esp_err_t i2c_dev_give_mutex(i2c_dev_t *dev)
{
#if !CONFIG_I2CDEV_NOLOCK
	if (!dev) return ESP_ERR_INVALID_ARG;

	ESP_LOGV(TAG, "[0x%02x at %d] giving mutex", dev->addr, dev->port);

	if (!xSemaphoreGive(dev->mutex))
	{
		ESP_LOGE(TAG, "[0x%02x at %d] Could not give device mutex", dev->addr, dev->port);
		return ESP_FAIL;
	}
#endif
	return ESP_OK;
}

static bool cfg_equal(const i2c_config_t *a, const i2c_config_t *b)
{
	return a->scl_io_num == b->scl_io_num
	    && a->sda_io_num == b->sda_io_num
#if HELPER_TARGET_IS_ESP32
	            && a->master.clk_speed == b->master.clk_speed
#elif HELPER_TARGET_IS_ESP8266
	                    && a->clk_stretch_tick == b->clk_stretch_tick
#endif
	                            && a->scl_pullup_en == b->scl_pullup_en
	                            && a->sda_pullup_en == b->sda_pullup_en;
}


static esp_err_t i2c_setup_port(const i2c_dev_t *dev)
{
	if (dev->port >= I2C_NUM_MAX) return ESP_ERR_INVALID_ARG;

	esp_err_t res;
	if (!cfg_equal(&dev->cfg, &states[dev->port].config) || !states[dev->port].installed)
	{
		ESP_LOGD(TAG, "Reconfiguring I2C driver on port %d", dev->port);
		i2c_config_t temp;
		memcpy(&temp, &dev->cfg, sizeof(i2c_config_t));
		temp.mode = I2C_MODE_MASTER;

		// Driver reinstallation
		if (states[dev->port].installed)
		{
			i2c_driver_delete(dev->port);
			states[dev->port].installed = false;
		}
#if HELPER_TARGET_IS_ESP32
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 1, 0)
		// See https://github.com/espressif/esp-idf/issues/10163
		if ((res = i2c_driver_install(dev->port, temp.mode, 0, 0, 0)) != ESP_OK)
			return res;
		if ((res = i2c_param_config(dev->port, &temp)) != ESP_OK)
			return res;
#else
		if ((res = i2c_param_config(dev->port, &temp)) != ESP_OK)
			return res;
		if ((res = i2c_driver_install(dev->port, temp.mode, 0, 0, 0)) != ESP_OK)
			return res;
#endif
#endif
#if HELPER_TARGET_IS_ESP8266
		// Clock Stretch time, depending on CPU frequency
		temp.clk_stretch_tick = dev->timeout_ticks ? dev->timeout_ticks : I2CDEV_MAX_STRETCH_TIME;
		if ((res = i2c_driver_install(dev->port, temp.mode)) != ESP_OK)
			return res;
		if ((res = i2c_param_config(dev->port, &temp)) != ESP_OK)
			return res;
#endif
		states[dev->port].installed = true;

		memcpy(&states[dev->port].config, &temp, sizeof(i2c_config_t));
		ESP_LOGD(TAG, "I2C driver successfully reconfigured on port %d", dev->port);
	}
#if HELPER_TARGET_IS_ESP32
	int t;
	if ((res = i2c_get_timeout(dev->port, &t)) != ESP_OK)
		return res;
	// Timeout cannot be 0
	uint32_t ticks = dev->timeout_ticks ? dev->timeout_ticks : I2CDEV_MAX_STRETCH_TIME;
	if ((ticks != t) && (res = i2c_set_timeout(dev->port, ticks)) != ESP_OK)
		return res;
	ESP_LOGD(TAG, "Timeout: ticks = %" PRIu32 " (%" PRIu32 " usec) on port %d", dev->timeout_ticks, dev->timeout_ticks / 80, dev->port);
#endif

	return ESP_OK;
}

esp_err_t i2c_dev_probe(const i2c_dev_t *dev, i2c_dev_type_t operation_type)
{
	if (!dev) return ESP_ERR_INVALID_ARG;

	SEMAPHORE_TAKE(dev->port);

	esp_err_t res = i2c_setup_port(dev);
	if (res == ESP_OK)
	{
		i2c_cmd_handle_t cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, dev->addr << 1 | (operation_type == I2C_DEV_READ ? 1 : 0), true);
		i2c_master_stop(cmd);

		res = i2c_master_cmd_begin(dev->port, cmd, pdMS_TO_TICKS(CONFIG_I2CDEV_TIMEOUT));

		i2c_cmd_link_delete(cmd);
	}

	SEMAPHORE_GIVE(dev->port);

	return res;
}

esp_err_t i2c_dev_read(const i2c_dev_t *dev, const void *out_data, size_t out_size, void *in_data, size_t in_size)
{
	if (!dev || !in_data || !in_size) return ESP_ERR_INVALID_ARG;

	SEMAPHORE_TAKE(dev->port);

	esp_err_t res = i2c_setup_port(dev);
	if (res == ESP_OK)
	{
		i2c_cmd_handle_t cmd = i2c_cmd_link_create();
		if (out_data && out_size)
		{
			i2c_master_start(cmd);
			i2c_master_write_byte(cmd, dev->addr << 1, true);
			i2c_master_write(cmd, (void *)out_data, out_size, true);
		}
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (dev->addr << 1) | 1, true);
		i2c_master_read(cmd, in_data, in_size, I2C_MASTER_LAST_NACK);
		i2c_master_stop(cmd);

		res = i2c_master_cmd_begin(dev->port, cmd, pdMS_TO_TICKS(CONFIG_I2CDEV_TIMEOUT));
		if (res != ESP_OK)
			ESP_LOGE(TAG, "Could not read from device [0x%02x at %d]: %d (%s)", dev->addr, dev->port, res, esp_err_to_name(res));

		i2c_cmd_link_delete(cmd);
	}

	SEMAPHORE_GIVE(dev->port);
	return res;
}

esp_err_t i2c_dev_write(const i2c_dev_t *dev, const void *out_reg, size_t out_reg_size, const void *out_data, size_t out_size)
{
	if (!dev || !out_data || !out_size) return ESP_ERR_INVALID_ARG;

	SEMAPHORE_TAKE(dev->port);

	esp_err_t res = i2c_setup_port(dev);
	if (res == ESP_OK)
	{
		i2c_cmd_handle_t cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, dev->addr << 1, true);
		if (out_reg && out_reg_size)
			i2c_master_write(cmd, (void *)out_reg, out_reg_size, true);
		i2c_master_write(cmd, (void *)out_data, out_size, true);
		i2c_master_stop(cmd);
		res = i2c_master_cmd_begin(dev->port, cmd, pdMS_TO_TICKS(CONFIG_I2CDEV_TIMEOUT));
		if (res != ESP_OK)
			ESP_LOGE(TAG, "Could not write to device [0x%02x at %d]: %d (%s)", dev->addr, dev->port, res, esp_err_to_name(res));
		i2c_cmd_link_delete(cmd);
	}

	SEMAPHORE_GIVE(dev->port);
	return res;
}

esp_err_t i2c_dev_read_reg(const i2c_dev_t *dev, uint8_t reg, void *in_data, size_t in_size)
{
	return i2c_dev_read(dev, &reg, 1, in_data, in_size);
}

esp_err_t i2c_dev_write_reg(const i2c_dev_t *dev, uint8_t reg, const void *out_data, size_t out_size)
{
	return i2c_dev_write(dev, &reg, 1, out_data, out_size);
}


/**
 * @brief Init library
 *
 * The function must be called before any other
 * functions of this library.
 *
 * @return ESP_OK on success
 */
esp_err_t i2cdev_init();

/**
 * @brief Finish work with library
 *
 * Uninstall i2c drivers.
 *
 * @return ESP_OK on success
 */
esp_err_t i2cdev_done();

/**
 * @brief Create mutex for device descriptor
 *
 * This function does nothing if option CONFIG_I2CDEV_NOLOCK is enabled.
 *
 * @param dev Device descriptor
 * @return ESP_OK on success
 */
esp_err_t i2c_dev_create_mutex(i2c_dev_t *dev);

/**
 * @brief Delete mutex for device descriptor
 *
 * This function does nothing if option CONFIG_I2CDEV_NOLOCK is enabled.
 *
 * @param dev Device descriptor
 * @return ESP_OK on success
 */
esp_err_t i2c_dev_delete_mutex(i2c_dev_t *dev);

/**
 * @brief Take device mutex
 *
 * This function does nothing if option CONFIG_I2CDEV_NOLOCK is enabled.
 *
 * @param dev Device descriptor
 * @return ESP_OK on success
 */
esp_err_t i2c_dev_take_mutex(i2c_dev_t *dev);

/**
 * @brief Give device mutex
 *
 * This function does nothing if option CONFIG_I2CDEV_NOLOCK is enabled.
 *
 * @param dev Device descriptor
 * @return ESP_OK on success
 */
esp_err_t i2c_dev_give_mutex(i2c_dev_t *dev);

/**
 * @brief Check the availability of the device
 *
 * Issue an operation of \p operation_type to the I2C device then stops.
 *
 * @param dev Device descriptor
 * @param operation_type Operation type
 * @return ESP_OK if device is available
 */
esp_err_t i2c_dev_probe(const i2c_dev_t *dev, i2c_dev_type_t operation_type);

/**
 * @brief Read from slave device
 *
 * Issue a send operation of \p out_data register address, followed by reading \p in_size bytes
 * from slave into \p in_data .
 * Function is thread-safe.
 *
 * @param dev Device descriptor
 * @param out_data Pointer to data to send if non-null
 * @param out_size Size of data to send
 * @param[out] in_data Pointer to input data buffer
 * @param in_size Number of byte to read
 * @return ESP_OK on success
 */
esp_err_t i2c_dev_read(const i2c_dev_t *dev,
	const void *out_data,
	size_t out_size,
	void *in_data,
	size_t in_size);

/**
 * @brief Write to slave device
 *
 * Write \p out_size bytes from \p out_data to slave into \p out_reg register address.
 * Function is thread-safe.
 *
 * @param dev Device descriptor
 * @param out_reg Pointer to register address to send if non-null
 * @param out_reg_size Size of register address
 * @param out_data Pointer to data to send
 * @param out_size Size of data to send
 * @return ESP_OK on success
 */
esp_err_t i2c_dev_write(const i2c_dev_t *dev,
	const void *out_reg,
	size_t out_reg_size,
	const void *out_data,
	size_t out_size);

/**
 * @brief Read from register with an 8-bit address
 *
 * Shortcut to ::i2c_dev_read().
 *
 * @param dev Device descriptor
 * @param reg Register address
 * @param[out] in_data Pointer to input data buffer
 * @param in_size Number of byte to read
 * @return ESP_OK on success
 */
esp_err_t i2c_dev_read_reg(const i2c_dev_t *dev,
	uint8_t reg,
	void *in_data,
	size_t in_size);

/**
 * @brief Write to register with an 8-bit address
 *
 * Shortcut to ::i2c_dev_write().
 *
 * @param dev Device descriptor
 * @param reg Register address
 * @param out_data Pointer to data to send
 * @param out_size Size of data to send
 * @return ESP_OK on success
 */
esp_err_t i2c_dev_write_reg(const i2c_dev_t *dev,
	uint8_t reg,
	const void *out_data,
	size_t out_size);

#define I2C_DEV_TAKE_MUTEX(dev) do { \
	        esp_err_t __ = i2c_dev_take_mutex(dev); \
	        if (__ != ESP_OK) return __;\
	    } while (0)

#define I2C_DEV_GIVE_MUTEX(dev) do { \
	            esp_err_t __ = i2c_dev_give_mutex(dev); \
	            if (__ != ESP_OK) return __;\
	        } while (0)

#define I2C_DEV_CHECK(dev, X) do { \
	                esp_err_t ___ = X; \
	                if (___ != ESP_OK) { \
	                    I2C_DEV_GIVE_MUTEX(dev); \
	                    return ___; \
	            } \
	        } while (0)

#define I2C_DEV_CHECK_LOGE(dev, X, msg, ...) do { \
	                esp_err_t ___ = X; \
	                if (___ != ESP_OK) { \
	                    I2C_DEV_GIVE_MUTEX(dev); \
	                    ESP_LOGE(TAG, msg, ## __VA_ARGS__); \
	                    return ___; \
	            } \
	        } while (0)



static inline esp_err_t send_command(ms5611_t *dev, uint8_t cmd)
{
	return i2c_dev_write(&dev->i2c_dev, NULL, 0, &cmd, 1);
}

static inline uint16_t shuffle(uint16_t val)
{
	return ((val & 0xff00) >> 8) | ((val & 0xff) << 8);
}


static inline esp_err_t read_prom(ms5611_t *dev)
{
	uint16_t tmp;

	// FIXME calculate CRC (AN502)

	CHECK(i2c_dev_read_reg(&dev->i2c_dev, PROM_ADDR_SENS, &tmp, 2));
	dev->config_data.sens = shuffle(tmp);
	CHECK(i2c_dev_read_reg(&dev->i2c_dev, PROM_ADDR_OFF, &tmp, 2));
	dev->config_data.off = shuffle(tmp);
	CHECK(i2c_dev_read_reg(&dev->i2c_dev, PROM_ADDR_TCS, &tmp, 2));
	dev->config_data.tcs = shuffle(tmp);
	CHECK(i2c_dev_read_reg(&dev->i2c_dev, PROM_ADDR_TCO, &tmp, 2));
	dev->config_data.tco = shuffle(tmp);
	CHECK(i2c_dev_read_reg(&dev->i2c_dev, PROM_ADDR_T_REF, &tmp, 2));
	dev->config_data.t_ref = shuffle(tmp);
	CHECK(i2c_dev_read_reg(&dev->i2c_dev, PROM_ADDR_TEMPSENS, &tmp, 2));
	dev->config_data.tempsens = shuffle(tmp);

	return ESP_OK;
}

static esp_err_t read_adc(ms5611_t *dev, uint32_t *result)
{
	uint8_t tmp[3];

	CHECK(i2c_dev_read_reg(&dev->i2c_dev, 0, tmp, 3));
	*result = (tmp[0] << 16) | (tmp[1] << 8) | tmp[2];

	// If we are to fast the ADC will return 0 instead of the actual result
	return *result == 0 ? ESP_ERR_INVALID_RESPONSE : ESP_OK;
}

static void wait_conversion(ms5611_t *dev)
{
	uint32_t us = 8220;
	switch (dev->osr)
	{
	case MS5611_OSR_256: us = 500; break;   // 0.5ms
	case MS5611_OSR_512: us = 1100; break;  // 1.1ms
	case MS5611_OSR_1024: us = 2100; break; // 2.1ms
	case MS5611_OSR_2048: us = 4100; break; // 4.1ms
	case MS5611_OSR_4096: us = 8220; break; // 8.22ms
	}
	ets_delay_us(us);
}

static inline esp_err_t get_raw_temperature(ms5611_t *dev, uint32_t *result)
{
	CHECK(send_command(dev, CMD_CONVERT_D2 + dev->osr));
	wait_conversion(dev);
	CHECK(read_adc(dev, result));

	return ESP_OK;
}

static inline esp_err_t get_raw_pressure(ms5611_t *dev, uint32_t *result)
{
	CHECK(send_command(dev, CMD_CONVERT_D1 + dev->osr));
	wait_conversion(dev);
	CHECK(read_adc(dev, result));

	return ESP_OK;
}

static esp_err_t ms5611_reset(ms5611_t *dev)
{
	I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
	I2C_DEV_CHECK(&dev->i2c_dev, send_command(dev, CMD_RESET));
	I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

	return ESP_OK;
}

/////////////////////////Public//////////////////////////////////////

esp_err_t ms5611_init_desc(ms5611_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
	CHECK_ARG(dev);

	if (addr != MS5611_ADDR_CSB_HIGH && addr != MS5611_ADDR_CSB_LOW)
	{
		ESP_LOGE(TAG, "Invalid I2C address 0x%02x", addr);
		return ESP_ERR_INVALID_ARG;
	}

	dev->i2c_dev.port = port;
	dev->i2c_dev.addr = addr;
	dev->i2c_dev.cfg.sda_io_num = sda_gpio;
	dev->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
	dev->i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

	return i2c_dev_create_mutex(&dev->i2c_dev);
}

esp_err_t ms5611_free_desc(ms5611_t *dev)
{
	CHECK_ARG(dev);

	return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t ms5611_init(ms5611_t *dev, ms5611_osr_t osr)
{
	CHECK_ARG(dev);

	dev->osr = osr;

	// First of all we need to reset the chip
	CHECK(ms5611_reset(dev));
	// Wait a bit for the device to reset
	vTaskDelay(pdMS_TO_TICKS(10));
	// Get the config
	CHECK(read_prom(dev));

	return ESP_OK;
}

esp_err_t ms5611_get_sensor_data(ms5611_t *dev, int32_t *pressure, float *temperature)
{
	CHECK_ARG(dev && pressure && temperature);

	// Second order temperature compensation see datasheet p8
	uint32_t raw_pressure = 0;
	uint32_t raw_temperature = 0;

	I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
	I2C_DEV_CHECK(&dev->i2c_dev, get_raw_pressure(dev, &raw_pressure));
	I2C_DEV_CHECK(&dev->i2c_dev, get_raw_temperature(dev, &raw_temperature));
	I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

	// dT = D2 - T_ref = D2 - C5 * 2^8
	int32_t dt = raw_temperature - ((int32_t)dev->config_data.t_ref << 8);
	// Actual temperature (-40...85C with 0.01 resolution)
	// TEMP = 20C +dT * TEMPSENSE =2000 + dT * C6 / 2^23
	int64_t temp = (2000 + (int64_t)dt * dev->config_data.tempsens / 8388608);
	// Offset at actual temperature
	// OFF=OFF_t1 + TCO * dT = OFF_t1(C2) * 2^16 + (C4*dT)/2^7
	int64_t off = (int64_t)((int64_t)dev->config_data.off * 65536)
	    + (((int64_t)dev->config_data.tco * dt) / 128);
	// Sensitivity at actual temperature
	// SENS=SENS_t1 + TCS *dT = SENS_t1(C1) *2^15 + (TCS(C3) *dT)/2^8
	int64_t sens = (int64_t)(((int64_t)dev->config_data.sens) * 32768)
	    + (((int64_t)dev->config_data.tcs * dt) / 256);

	// Set defaults for temp >= 2000
	int64_t t_2 = 0;
	int64_t off_2 = 0;
	int64_t sens_2 = 0;
	int64_t help = 0;
	if (temp < 2000)
	{
		// Low temperature
		t_2 = ((dt * dt) >> 31); // T2 = dT^2/2^31
		help = (temp - 2000);
		help = 5 * help * help;
		off_2 = help >> 1; // OFF_2  = 5 * (TEMP - 2000)^2/2^1
		sens_2 = help >> 2; // SENS_2 = 5 * (TEMP - 2000)^2/2^2
		if (temp < -1500)
		{
			// Very low temperature
			help = (temp + 1500);
			help = help * help;
			off_2 = off_2 + 7 * help; // OFF_2  = OFF_2 + 7 * (TEMP + 1500)^2
			sens_2 = sens_2 + ((11 * help) >> 1); // SENS_2 = SENS_2 + 7 * (TEMP + 1500)^2/2^1
		}
	}

	temp = temp - t_2;
	off = off - off_2;
	sens = sens - sens_2;

	// Temperature compensated pressure (10...1200mbar with 0.01mbar resolution
	// P = digital pressure value  * SENS - OFF = (D1 * SENS/2^21 -OFF)/2^15
	*pressure = (int32_t)(((int64_t)raw_pressure * (sens / 0x200000) - off) / 32768);
	*temperature = (float)temp / 100.0;

	return ESP_OK;
}

static void task_i2c_barometer(void* arg)
{
	
}

void katech_esp_i2c_baro_init(void)
{
	esp_err_t ret;
	
	ms5611_config_data_t ms5611_conf = { 
		.sens = 0,
		.off = 0,
		.tcs = 0,
		.tco = 0,
		.t_ref = 0,
		.tempsens = 0
	};
	
	i2c_config_t conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = I2C_MASTER_SDA_IO,
		.scl_io_num = I2C_MASTER_SCL_IO,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = I2C_MASTER_FREQ_HZ,
	};

	i2c_dev_t i2cdev_conf = { 
		.port = 0,
		.addr = 0,
		.cfg = conf,
		.mutex = 0,
		.timeout_ticks = 0,
	};
	
	ms5611_t MS5611 = { 
		.i2c_dev = i2cdev_conf,
		.osr = MS5611_OSR_1024,
		.config_data = ms5611_conf,
	};
	
	
	//start adc task
	xTaskCreate(task_i2c_barometer, "task_i2c_barometer", 2048, NULL, 10, NULL);
}