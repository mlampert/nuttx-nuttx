#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>
#include <nuttx/sensors/vl53l0x.h>

#include "stm32_i2c.h"
#include "nucleo-144.h"


#if defined (CONFIG_I2C) && defined(CONFIG_SENSORS_VL53L0X)
#define VL53L0X_I2C_DEV  1  /* I2C1 */


int stm32_vl53l0x_initialize(FAR const char *devpath)
{
  FAR struct i2c_master_s *i2c;
  int ret;

  sninfo("Initializing VL53L0X for %s\n", devpath);


  i2c = stm32_i2cbus_initialize(VL53L0X_I2C_DEV);
  if (i2c == NULL)
  {
    return -ENODEV;
  }

  ret = VL53L0X_register(devpath, i2c, VL53L0X_I2C_ADDR, VL53L0X_I2C_FREQ);
  if (ret < 0)
  {
    snerr("ERROR: Failed registering VL53L0X (%s) - %d\n", devpath, ret);
  }
  return ret;
}

#endif
