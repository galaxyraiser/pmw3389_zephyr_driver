//
// Created by jonasotto on 9/15/22.
//

#ifndef CAROLO_APP_PMW3389_H
#define CAROLO_APP_PMW3389_H

#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

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
