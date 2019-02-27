#ifndef __INCLUDED_vl53l0x_platform_nx_h
#define __INCLUDED_vl53l0x_platform_nx_h

#include <nuttx/i2c/i2c_master.h>
#include <stdint.h>
#include <vl53l0x_types.h>


typedef struct {
  uint8_t         enable;
  FixPoint1616_t  value;
} VL53L0X_nx_limit_check_t;

typedef struct {
  VL53L0X_nx_limit_check_t  sigmaFinalRange;
  VL53L0X_nx_limit_check_t  signalRateFinalRange;
  VL53L0X_nx_limit_check_t  rangeIgnoreThreshold;
  uint32_t                  measurementTimingBudget;
  uint8_t                   vcselPeriodPreRange;
  uint8_t                   vcselPeriodFinalRange;
} VL53L0X_nx_profile_values_t;

typedef struct {
  FAR struct i2c_master_s     *i2c;
  struct i2c_config_s         i2cConfig;
  VL53L0X_nx_profile_values_t init;
} VL53L0X_nx_dev_t;

int VL53L0X_nx_read(VL53L0X_nx_dev_t *dev, uint8_t index, FAR uint8_t* pdata, uint32_t count);
int VL53L0X_nx_write(VL53L0X_nx_dev_t *dev, uint8_t index, FAR const uint8_t* pdata, uint32_t count);

#endif
