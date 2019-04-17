#ifndef INCLUDED_mlx90640_nx_h
#define INCLUDED_mlx90640_nx_h

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/mlx90640.h>

typedef struct {
  FAR struct i2c_master_s     *i2c;
  struct i2c_config_s         i2cConfig;
} MLX90640_nx_dev_t;

int MLX90640_I2CRead(MLX90640_nx_dev_t *dev, uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data);
int MLX90640_I2CWrite(MLX90640_nx_dev_t *dev, uint16_t writeAddress, uint16_t data);

#endif
