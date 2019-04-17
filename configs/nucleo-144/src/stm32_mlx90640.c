#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>
#include <nuttx/sensors/mlx90640.h>

#include "stm32_i2c.h"
#include "nucleo-144.h"


#if defined (CONFIG_I2C) && defined(CONFIG_SENSORS_MLX90640)
#define MLX90640_I2C_DEV  1  /* I2C1 */

#if 0
# define MLX90640_I2C_SPEED MLX90640_I2C_FREQ
#else
# define MLX90640_I2C_SPEED I2C_SPEED_FAST
#endif

int stm32_mlx90640_initialize(FAR const char *devpath)
{
  FAR struct i2c_master_s *i2c;
  int ret;

  sninfo("Initializing MLX90640 for %s\n", devpath);


  i2c = stm32_i2cbus_initialize(MLX90640_I2C_DEV);
  if (i2c == NULL)
  {
    return -ENODEV;
  }

  ret = MLX90640_register(devpath, i2c, MLX90640_I2C_ADDR, MLX90640_I2C_SPEED);
  if (ret < 0)
  {
    snerr("ERROR: Failed registering MLX90640 (%s) - %d\n", devpath, ret);
  }
  return ret;
}

#endif
