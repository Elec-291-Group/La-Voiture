#ifndef VL53L0X_H
#define VL53L0X_H

#include <stdint.h>

/* Initialise the VL53L0X over I2C1.
   Returns 1 on success, 0 if the device is not found or init fails.
   Call after MX_I2C1_Init(). */
int VL53L0X_Init(void);

/* Perform one single-shot ranging measurement.
   Returns distance in mm, or 0xFFFF on timeout / sensor error. */
uint16_t VL53L0X_ReadDistance(void);

#endif /* VL53L0X_H */
