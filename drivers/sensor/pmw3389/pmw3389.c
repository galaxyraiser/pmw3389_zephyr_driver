//
// Created by jonasotto on 9/15/22.
//

#define DT_DRV_COMPAT pixart_pmw3389

#include "pmw3389.h"
#include <zephyr/pm/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/__assert.h>

struct pmw3389_config {
	struct spi_dt_spec spi;
	struct gpio_dt_spec irq_gpio;
	int resolution_cpi; // [counts/inch]
};

LOG_MODULE_REGISTER(pmw3389, LOG_LEVEL_DBG);

// Undefine to fall back to reading each register separately
#define FETCH_USING_BURST_READ

// region SPI Registers and Timing Constants
#define REG_Product_ID		    0x00
#define Expected_Product_ID	    0x47
#define REG_Motion		    0x02
#define BIT_Motion_MOT		    7
#define REG_Delta_X_L		    0x03
#define REG_Delta_X_H		    0x04
#define REG_Delta_Y_L		    0x05
#define REG_Delta_Y_H		    0x06
#define REG_Resolution_L	    0x0E
#define REG_Resolution_H	    0x0F
#define REG_Frame_Capture	    0x12
#define REG_SROM_ID				0x2A
#define REG_Power_Up_Reset	    0x3A
#define REG_Inverse_Product_ID	    0x3F
#define Expected_Inverse_Product_ID 0xB8
#define REG_Motion_Burst	    0x50
#define REG_RawData_Burst	    0x64

// Min command interval of write commands
#define T_SWW_US 180

// Min byte interval for read after write (..., data_write_1, address_read_2, ...)
#define T_SWR_US 180

// Min byte interval after read (..., data_read_1, address_2, ...)
#define T_SRW_US 20
#define T_SRR_US 20

// Min delay between address and data bytes for read
#define T_SRAD_US 160

// Min delay between address and data bytes for burst motion read
#define T_SRAD_MOTBR_US 35

// Wait such that the time between last address bit of the next read and last data bit is
// > T_SWR: Assume clock frequency is at the maximum of 2MHz -> 4us for 8 address bits
#define DELAY_WRITE_WRITE (T_SWW_US - 8)
#define DELAY_WRITE_READ  (T_SWR_US - 4)
#define DELAY_READ_WRITE  T_SRW_US
#define DELAY_READ_READ	  T_SRR_US

// endregion

// region Utility
#define xstr(a) str(a)
#define str(a) #a

// Return error code, if != 0
#define TRY(that)																			\
	{																						\
		int error = that;																	\
		const char *param = xstr(that);														\
		if (error != 0) {																	\
			LOG_ERR("Operation %s failed", param);											\
			return error;																	\
		}																					\
	}

// Release SPI and return error code, if != 0
#define TRY_R(that)																			\
	{																						\
		int error = that;																	\
		const char *param = xstr(that);														\
		if (error != 0) {																	\
			LOG_ERR("Operatin %s failed", param);											\
			spi_release_dt(spec);															\
			return error;																	\
		}																					\
	}

static bool is_bit_set(uint8_t value, uint8_t index)
{
	return (value & (1 << index)) != 0;
}

static int16_t signed_16_from_parts(uint8_t low, uint8_t high)
{
	int res = low;
	res |= high << 8;
	return (int16_t)res;
}

// endregion

// region SPI Bus

// Note: Functions with _nr suffix do not release the SPI bus (chip select)!

/*
 * Chip SPI Interface description
 *
 * Whenever NCS goes high, the entire transaction is aborted, the serial port will be reset.
 * All transactions should be framed by NCS (do not leave NCS low if not used)
 * NCS must be raised after burst-mode transaction, or to terminate burst-mode.
 *
 * Chip reads MOSI on SCLK rising edge.
 * Chip writes MISO on SCLK falling edge.
 *
 * Write Operation:
 * Pull NCS low, delay 120ns
 * 1-bit, 7-bit address, 8-bit data.
 * (delay T_SCLK_NCS_WRITE (35us), pull NCS high)
 *
 * Read Operation:
 * 0-bit, 7-bit address, T_SRAD delay, 8-bit data
 * (delay T_SCLK_NCS_READ (120ns), pull NCS high)
 */

/**
 * @note Does not release SPI bus!
 * @return a value from spi_write().
 */
static int send_byte_nr(const struct spi_dt_spec *spec, uint8_t data)
{
	uint8_t tx_data[] = {data};
	struct spi_buf tx_buffer = {.buf = tx_data, .len = sizeof(tx_data)};
	struct spi_buf_set tx_buffer_set = {.buffers = &tx_buffer, .count = 1};

	return spi_write_dt(spec, &tx_buffer_set);
}

/**
 * @note Does not release SPI bus!
 * @return a value from spi_read().
 */
// NOLINTNEXTLINE(readability-non-const-parameter)
static int receive_byte_nr(const struct spi_dt_spec *spec, uint8_t *out)
{
	struct spi_buf rx_buffer = {.buf = out, .len = 1};
	struct spi_buf_set rx_buffer_set = {.buffers = &rx_buffer, .count = 1};
	return spi_read_dt(spec, &rx_buffer_set);
}

/**
 * @param reg 7-bit register address
 * @note: Delay by DELAY_WRITE_WRITE until starting the next write, or DELAY_WRITE_READ
 *        until starting next read
 * @note Does not release SPI bus!
 * @return a value from spi_write().
 */
int write_register_nr(const struct spi_dt_spec *spec, uint8_t reg, uint8_t data)
{
	// Set first address bit to 1 to indicate write
	uint8_t tx_data[] = {reg | 0b10000000, data};
	struct spi_buf tx_buffer = {.buf = tx_data, .len = sizeof(tx_data)};
	struct spi_buf_set tx_buffer_set = {.buffers = &tx_buffer, .count = 1};

	return spi_write_dt(spec, &tx_buffer_set);
}

/**
 * @param addr 7-bit register address
 * @return Register contents
 * @note Delay by T_SRR or T_SRW until starting the next read/write
 * @note Does not release SPI bus!
 * @return a value from spi_read().
 */
int read_register_nr(const struct spi_dt_spec *spec, uint8_t addr, uint8_t *out)
{
	// Write address
	send_byte_nr(spec, addr & ~(1 << 7));
	// Wait T_SRAD
	k_busy_wait(T_SRAD_US);
	// Read Data
	return receive_byte_nr(spec, out);
}

/**
 * Read multiple registers
 */
int read_multiple(const struct spi_dt_spec *spec, const uint8_t addresses[], int nr_addresses,
		  uint8_t data_out[])
{
	for (int i = 0; i < nr_addresses; i++) {
		TRY_R(read_register_nr(spec, addresses[i], &data_out[i]));
		// Wait T_SRR if not last
		if (i != nr_addresses - 1) {
			k_busy_wait(DELAY_READ_READ);
		}
	}

	// T_SCLK_NCS_READ: omitted because small (120ns)
	TRY(spi_release_dt(spec));
	return 0;
}

/**
 * Burst Motion Read. Reads the following registers (up to specified number):
 * BYTE[00] = Motion
 * BYTE[01] = Observation
 * BYTE[02] = Delta_X_L
 * BYTE[03] = Delta_X_H
 * BYTE[04] = Delta_Y_L
 * BYTE[05] = Delta_Y_H
 * BYTE[06] = SQUAL
 * BYTE[07] = RawData_Sum
 * BYTE[08] = Maximum_RawData
 * BYTE[09] = Minimum_Rawdata
 * BYTE[10] = Shutter_Upper
 * BYTE[11] = Shutter_Lower
 * @param dev
 * @param out
 * @param nr_bytes
 */
// NOLINTNEXTLINE(readability-non-const-parameter)
int burst_read_motion(const struct spi_dt_spec *spec, uint8_t out[], int nr_bytes)
{
	//LOG_DBG("Starting burst read!");
	// 1. Write any value to Motion_Burst register
	TRY_R(write_register_nr(spec, REG_Motion_Burst, 0));

	// 2. Lower NCS
	// (is already low)

	// 3. Send Motion_Burst address (0x50).
	TRY_R(send_byte_nr(spec, REG_Motion_Burst));

	// 4. Wait for t_SRAD_MOTBR
	k_busy_wait(T_SRAD_MOTBR_US);

	// 5. Start reading SPI data continuously up to 12bytes.
	struct spi_buf rx_buffers = {.buf = out, .len = nr_bytes};
	struct spi_buf_set rx_buffer_set = {.buffers = &rx_buffers, .count = 1};
	TRY_R(spi_transceive_dt(spec, NULL, &rx_buffer_set));

	// Motion burst may be terminated by pulling NCS high for at least t_BEXIT
	TRY(spi_release_dt(spec));
	//LOG_DBG("Burst read done.");
	return 0;
}

/**
 * Fetch motion data by manually reading each register
 */
void fetch_manual(const struct spi_dt_spec *spec, int16_t *delta_x, int16_t *delta_y)
{
	// For first motion read, write any value to Motion register first (?)
	write_register_nr(spec, REG_Motion, 0x00);
	k_busy_wait(DELAY_WRITE_READ);

	uint8_t motion = 0;
	read_register_nr(spec, REG_Motion, &motion);
	if (is_bit_set(motion, BIT_Motion_MOT)) {
		// Motion has occurred!
		uint8_t results[4] = {0};
		uint8_t addresses[4] = {REG_Delta_X_L, REG_Delta_X_H, REG_Delta_Y_L, REG_Delta_Y_H};
		k_busy_wait(DELAY_READ_READ);
		read_multiple(spec, addresses, 4, results);
		*delta_x = signed_16_from_parts(results[0], results[1]);
		*delta_y = signed_16_from_parts(results[2], results[3]);
	} else {
		// No motion has occurred
		*delta_x = 0;
		*delta_y = 0;
	}

	spi_release_dt(spec);
}

/**
 * Fetch motion data using burst read
 * @param squal If != NULL, out parameter for surface quality
 * @return 0 or negative error code
 */
int fetch_burst(const struct spi_dt_spec *spec, int16_t *delta_x, int16_t *delta_y, uint8_t *squal)
{
	int len = 6;
	if (squal != NULL) {
		len = 7;
	}

	uint8_t data[12] = {0};
	TRY(burst_read_motion(spec, data, len));
	if (is_bit_set(data[0], BIT_Motion_MOT)) {
		*delta_x = signed_16_from_parts(data[2], data[3]);
		*delta_y = signed_16_from_parts(data[4], data[5]);
	} else {
		*delta_x = 0;
		*delta_y = 0;
	}

	if (squal != NULL) {
		*squal = data[6];
	}
	return 0;
}

// endregion

// region Driver Functions

int pwm3389_get_raw_data(struct device *dev, uint8_t *out)
{
	const struct pmw3389_config *config = dev->config;
	const struct spi_dt_spec *spec = &config->spi;

	TRY_R(write_register_nr(spec, 0x10, 0x00));
	TRY_R(write_register_nr(spec, REG_Frame_Capture, 0x83));
	TRY_R(write_register_nr(spec, REG_Frame_Capture, 0xC5));
	k_sleep(K_MSEC(20));
	TRY_R(send_byte_nr(spec, REG_RawData_Burst));
	k_busy_wait(T_SRAD_US);
	for (int i = 0; i < 1296; i++) {
		TRY_R(receive_byte_nr(spec, &out[i]));
		k_busy_wait(15);
	}
	TRY(spi_release_dt(spec));
	return 0;
}

// forward declaration
int pmw3389_sample_fetch(const struct device *dev, enum sensor_channel chan);
int pmw3389_channel_get(const struct device *dev, enum sensor_channel chan,
			struct sensor_value *val);

static int pmw3389_report_data(const struct device *dev) {
	struct pmw3389_data *data = dev->data;

	sensor_sample_fetch(dev);

#if CONFIG_PMW3389_REPORT_INTERVAL_MIN > 0
	int64_t now = k_uptime_get();
	// purge accumulated delta, if last sampled had not been reported on last report tick
	if (now - data->last_smp_time >= CONFIG_PMW3389_REPORT_INTERVAL_MIN) {
		//LOG_DBG("Reset dx/dy");
		data->dx = 0;
		data->dy = 0;
	}
	data->last_smp_time = now;
#endif

#if CONFIG_PMW3389_INVERT_Y
    data->delta_y = -data->delta_y;
#endif

	data->dx += data->delta_x;
	data->dy += data->delta_y;

#if CONFIG_PMW3389_REPORT_INTERVAL_MIN > 0
	if (now - data->last_rpt_time < CONFIG_PMW3389_REPORT_INTERVAL_MIN) {
		return 0;
	}
#endif

    // fetch report value
	int16_t rx = (int16_t)CLAMP(data->dx, INT16_MIN, INT16_MAX);
	int16_t ry = (int16_t)CLAMP(data->dy, INT16_MIN, INT16_MAX);
	data->dx = data->dy = 0;

    // TODO: lock free
    struct k_msgq *out_q = atomic_ptr_get(&data->out_q);
    if (out_q) {
        k_msgq_put(out_q,
                &(struct mouse_input_report){.x = rx, .y = ry, .t = k_uptime_get(), .dev = dev}, K_MSEC(10));
    }

#if CONFIG_PMW3389_REPORT_INTERVAL_MIN > 0
	data->last_rpt_time = now;
#endif

	return 0;
}

static int pmw3389_set_interrupt(const struct device *dev, const bool en) {
	const struct pmw3389_config *config = dev->config;
	const struct gpio_dt_spec *irq = &config->irq_gpio;

	int ret = gpio_pin_interrupt_configure_dt(irq, en ? GPIO_INT_LEVEL_ACTIVE : GPIO_INT_DISABLE);
	if (ret < 0) {
		LOG_ERR("can't set interrupt");
	}
	return ret;
}

static void pmw3389_gpio_callback(const struct device *gpiob, struct gpio_callback *cb,
								  uint32_t pins) {
	struct pmw3389_data *data = CONTAINER_OF(cb, struct pmw3389_data, irq_gpio_cb);
	const struct device *dev = data->dev;
	pmw3389_set_interrupt(dev, false);
	k_work_submit(&data->trigger_work);
}

static void pmw3389_work_callback(struct k_work *work) {
	struct pmw3389_data *data = CONTAINER_OF(work, struct pmw3389_data, trigger_work);
	const struct device *dev = data->dev;
	// invoke trigger handler
	if (data->handler) {
		data->handler(dev, NULL);
	}
    pmw3389_report_data(dev);
	pmw3389_set_interrupt(dev, true);
}

static int pmw3389_init_irq(const struct device *dev) {
	int err;
	struct pmw3389_data *data = dev->data;
	const struct pmw3389_config *config = dev->config;

	// check readiness of irq gpio pin
	if (!device_is_ready(config->irq_gpio.port)) {
		LOG_ERR("IRQ GPIO device not ready");
		return -ENODEV;
	}

	// init the irq pin
	err = gpio_pin_configure_dt(&config->irq_gpio, GPIO_INPUT);
	if (err) {
		LOG_ERR("Cannot configure IRQ GPIO");
		return err;
	}

	// setup and add the irq callback associated
	gpio_init_callback(&data->irq_gpio_cb, pmw3389_gpio_callback, BIT(config->irq_gpio.pin));

	err = gpio_add_callback(config->irq_gpio.port, &data->irq_gpio_cb);
	if (err) {
		LOG_ERR("Cannot add IRQ GPIO callback");
	}

	// Enable IRQ
	pmw3389_set_interrupt(dev, true);

	return err;
}

int pmw3389_set_resolution(const struct device *dev, int dpi)
{
	const struct pmw3389_config *config = dev->config;
	const struct spi_dt_spec *spec = &config->spi;

    // Set resolution
	LOG_DBG("Configuring resolution: %d cpi", dpi);
	if (dpi < 50 || dpi > 16000) {
		LOG_ERR("Resolution of %d is out of range [%d, %d]", dpi, 50, 16000);
		spi_release_dt(spec);
		return -EINVAL;
	}
	if (dpi % 50 != 0) {
		LOG_WRN("Resolution of %d is not a multiple of 50cpi!", config->resolution_cpi);
	}
	uint16_t resolution = dpi / 50;
	k_busy_wait(DELAY_WRITE_WRITE);
	TRY_R(write_register_nr(spec, REG_Resolution_H, resolution >> 8));
	k_busy_wait(DELAY_WRITE_WRITE);
	TRY_R(write_register_nr(spec, REG_Resolution_L, resolution & 0xFF));

    return 0;
}

int pmw3389_init(const struct device *dev)
{
	LOG_INF("Initializing PMW3389");

	const struct pmw3389_config *config = dev->config;
	const struct spi_dt_spec *spec = &config->spi;
	struct pmw3389_data *data = dev->data;

	data->dev = dev;
	k_work_init(&data->trigger_work, pmw3389_work_callback);

	// init irq routine
	int err = pmw3389_init_irq(dev);
	if (err) {
		return err;
	}

	// SPI: Manual CS control, Zephyr does delay between pulling NCS low and start, but does not
	//  wait after transmission. NCS is not pulled high automatically, as configured

	// 1. Apply Power
	// 2. Drive NCS high, then low (done by driver)
	// 3. Write 0x5A to Power_Up_Reset register
	LOG_DBG("Resetting device");
	TRY_R(write_register_nr(spec, REG_Power_Up_Reset, 0x5A));
	TRY(spi_release_dt(spec));
	// 4. Wait for at least 50ms.
	k_sleep(K_MSEC(50));

	// 5. Read from registers 0x02, 0x03, 0x04, 0x05 and 0x06 one time regardless of the motion
	// pin state.
	uint8_t addresses[] = {0x02, 0x03, 0x04, 0x05, 0x06};
	uint8_t dummy_read[sizeof(addresses)];
	TRY(read_multiple(spec, addresses, sizeof(addresses), dummy_read));

	// (6. Perform SROM download [Refer to 7.1 SROM Download].)
	// TODO: Find out why/if that is necessary, and which SROM to steal from where

	// 7. Write to register 0x3D with value 0x80.
	// Last op was read if SROM DL was not performed -> wait T_SRW
	k_busy_wait(DELAY_READ_WRITE);
	TRY_R(write_register_nr(spec, 0x3D, 0x80));

	k_busy_wait(DELAY_WRITE_READ);

	// 8. Read register 0x3D at 1ms interval until value 0xC0 is obtained or read up to
	//    55ms. This register read interval must be carried out at 1ms interval with timing
	//    tolerance of +/- 1%.
	LOG_DBG("Waiting for proper value in 0x3D...");
	while (true) {
		// Do not wait T_SRR since the 1ms is way longer anyway
		uint8_t result_3D = 0;
		TRY_R(read_register_nr(spec, 0x3D, &result_3D));
		LOG_DBG(" Got %#x", result_3D);
		if (result_3D == 0xC0) {
			break;
		}
		k_busy_wait(1000);
	}

	// 9. Write to register 0x3D with value 0x00.
	k_busy_wait(DELAY_READ_WRITE);
	TRY_R(write_register_nr(spec, 0x3D, 0x00));

	// 10. Write 0x20 to register 0x10
	// 0x10 is Config2, 0x20=0b00100000 enables the REST mode and sets CPI to addect both X and
	// Y
	k_busy_wait(DELAY_WRITE_WRITE);
	TRY_R(write_register_nr(spec, 0x10, 0x20));

	// 11. Load configuration for other registers.
    if ((err = pmw3389_set_resolution(dev, config->resolution_cpi)) != 0) {
        return err;
    }

	// Verify communication
	LOG_DBG("Verifying communication by reading product ID");
	k_busy_wait(DELAY_WRITE_READ);
	uint8_t received_product_id = 0;
	TRY_R(read_register_nr(spec, REG_Product_ID, &received_product_id));
	if (received_product_id != Expected_Product_ID) {
		LOG_ERR("Read product ID %#x, but expected %#x, could not verify communication "
			"with sensor!",
			received_product_id, Expected_Product_ID);
		spi_release_dt(spec);
		return -EIO;
	}
	k_busy_wait(DELAY_READ_READ);
	TRY_R(read_register_nr(spec, REG_Inverse_Product_ID, &received_product_id));
	if (received_product_id != Expected_Inverse_Product_ID) {
		LOG_ERR("Read inverse product ID %#x, but expected %#x, could not verify "
			"communication with sensor!",
			received_product_id, Expected_Inverse_Product_ID);
		spi_release_dt(spec);
		return -EIO;
	}

	uint8_t srom_id = 0;
	TRY_R(read_register_nr(spec, REG_SROM_ID, &srom_id));
	LOG_DBG("Product ID: %02x, srom_id: %02x", received_product_id, srom_id);
	TRY(spi_release_dt(spec));

	LOG_INF("Done!");
	return 0;
}

int pmw3389_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	if (chan != SENSOR_CHAN_POS_DX && chan != SENSOR_CHAN_POS_DY &&
		chan != SENSOR_CHAN_ALL) {
		return -EINVAL;
	}

	const struct pmw3389_config *config = dev->config;
	const struct spi_dt_spec *spec = &config->spi;
	struct pmw3389_data *data = dev->data;

#ifdef FETCH_USING_BURST_READ
	TRY(fetch_burst(spec, &data->delta_x, &data->delta_y, NULL));
#else
	fetch_manual(spec, &data->delta_x, &data->delta_y);
#endif
	return 0;
}

int pmw3389_channel_get(const struct device *dev, enum sensor_channel chan,
			struct sensor_value *val)
{
	const struct pmw3389_config *config = dev->config;
	const struct pmw3389_data *data = dev->data;
	int16_t counts;

	switch ((int)chan) {
	case SENSOR_CHAN_POS_DX:
		counts = data->delta_x;
		break;
	case SENSOR_CHAN_POS_DY:
		counts = data->delta_y;
		break;
	default:
		return -ENOTSUP;
	}

	val->val1 = counts;
	//TRY(sensor_value_from_double(val, (double)counts / (39.3700787 * config->resolution_cpi)));

	return 0;
}

int pmw3389_trigger_set(const struct device *dev,
			     const struct sensor_trigger *trig,
			     sensor_trigger_handler_t handler)
{
	struct pmw3389_data *data = dev->data;
    // TODO: atomic set
    data->handler = handler;
	return 0;
}

int pmw3389_attr_set(const struct device *dev, enum sensor_channel chan, enum sensor_attribute attr,
                    const struct sensor_value *val)
{
    return pmw3389_set_resolution(dev, val->val1);
}

// endregion
static const struct sensor_driver_api pmw3389_api = {
	.sample_fetch = pmw3389_sample_fetch,
	.channel_get = pmw3389_channel_get,
    .trigger_set = pmw3389_trigger_set,
    .attr_set = pmw3389_attr_set,
};

#if IS_ENABLED(CONFIG_PM_DEVICE)

static int pmw3389_pm_action(const struct device *dev, enum pm_device_action action) {
    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
        LOG_DBG("Suspend");
        return pmw3389_set_interrupt(dev, false);
    case PM_DEVICE_ACTION_RESUME:
        LOG_DBG("Resume");
        return pmw3389_set_interrupt(dev, true);
    default:
        return -ENOTSUP;
    }
}

#endif // IS_ENABLED(CONFIG_PM_DEVICE)

#define PMW3389_INIT(n)                                                                            \
	static struct pmw3389_data pmw3389_data_##n;                                               \
	static const struct pmw3389_config pmw3389_config_##n = {                                  \
		.spi = SPI_DT_SPEC_INST_GET(n,                                                     \
					    SPI_OP_MODE_MASTER | SPI_WORD_SET(8U) |                \
						    SPI_HOLD_ON_CS | SPI_MODE_CPOL |                                    \
						    SPI_MODE_CPHA,                                                      \
					    0U),                                                                    \
		.resolution_cpi = DT_INST_PROP(n, resolution),                                          \
		.irq_gpio = GPIO_DT_SPEC_INST_GET(n, irq_gpios)};                                       \
	PM_DEVICE_DT_INST_DEFINE(n, pmw3389_pm_action);                                             \
	DEVICE_DT_INST_DEFINE(n, &pmw3389_init, PM_DEVICE_DT_INST_GET(n), &pmw3389_data_##n, &pmw3389_config_##n,       \
			      POST_KERNEL, CONFIG_APPLICATION_INIT_PRIORITY, &pmw3389_api);

DT_INST_FOREACH_STATUS_OKAY(PMW3389_INIT)
