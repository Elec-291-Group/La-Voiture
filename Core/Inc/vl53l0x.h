#ifndef VL53L0X_H
#define VL53L0X_H

#include <stdint.h>

int VL53L0X_Init(void);
uint16_t VL53L0X_ReadDistance(void);

#endif /* VL53L0X_H */