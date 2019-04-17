#ifndef __INCLUDE_NUTTX_SENSORS_mlx90640_h
#define __INCLUDE_NUTTX_SENSORS_mlx90640_h

#include <stdint.h>

/* NuttX uses the upper 7 bit shifted right as the address -> 0x52 / 2 = 0x29 */
#define MLX90640_I2C_ADDR    0x33
#define MLX90640_I2C_FREQ    1000000

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

struct i2c_master_s;

int MLX90640_register(FAR const char *devpath, FAR struct i2c_master_s *i2c, uint8_t addr, uint32_t speed);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif
