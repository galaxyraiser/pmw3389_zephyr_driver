//
// Created by jonasotto on 9/15/22.
//

#ifndef CAROLO_APP_PMW3389_H
#define CAROLO_APP_PMW3389_H

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

struct pmw3389_data {
	const struct device     *dev;

	struct gpio_callback    irq_gpio_cb;    // motion pin irq callback
	struct k_work           trigger_work;   // realtrigger job

    atomic_ptr_t out_q;

	int16_t delta_x;
	int16_t delta_y;

	int16_t dx, dy;
#if CONFIG_PMW3389_REPORT_INTERVAL_MIN > 0
	int64_t last_smp_time;
	int64_t last_rpt_time;
#endif

    // Trigger handler
    sensor_trigger_handler_t handler;
};


struct mouse_input_report
{
    int64_t t;
    int16_t x,y;
    const struct device *dev;
} __attribute__((aligned(8)));

/**
 * Read raw pixel data from sensor
 * @note Untested, may be broken
 * @param out Array of size >= 1296
 */
int pwm3389_get_raw_data(struct device *dev, uint8_t *out);

#ifdef __cplusplus
}
#endif
#endif // CAROLO_APP_PMW3389_H
