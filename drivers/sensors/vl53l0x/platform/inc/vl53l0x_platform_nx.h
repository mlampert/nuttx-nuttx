#ifndef __INCLUDED_vl53l0x_platform_nx_h
#define __INCLUDED_vl53l0x_platform_nx_h

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/vl53l0x.h>
#include <stdint.h>

typedef struct {
  FAR struct i2c_master_s     *i2c;
  struct i2c_config_s         i2cConfig;
  VL53L0X_profile_t init;
} VL53L0X_nx_dev_t;

int VL53L0X_nx_read(VL53L0X_nx_dev_t *dev, uint8_t index, FAR uint8_t* pdata, uint32_t count);
int VL53L0X_nx_write(VL53L0X_nx_dev_t *dev, uint8_t index, FAR const uint8_t* pdata, uint32_t count);

#endif
