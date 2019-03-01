#ifndef __INCLUDE_NUTTX_SENSORS_vl53l0x_h
#define __INCLUDE_NUTTX_SENSORS_vl53l0x_h

#include <stdint.h>

/* NuttX uses the upper 7 bit shifted right as the address -> 0x52 / 2 = 0x29 */
#define VL53L0X_I2C_ADDR    0x29
#define VL53L0X_I2C_FREQ    400000


#define VL53L0X_PROFILE_DEFAULT         0x0000
#define VL53L0X_PROFILE_LONG_RANGE      0x0001
#define VL53L0X_PROFILE_HIGH_SPEED      0x0002
#define VL53L0X_PROFILE_HIGH_ACCURACY   0x0003

#define VL53L0X_PROFILE_MODE_SINGLE     0x0000
#define VL53L0X_PROFILE_MODE_CONT       0x0100

#define VL53L0X_PROFILE_MASK_PROFILE    0x00FF
#define VL53L0X_PROFILE_MASK_MODE       0xFF00

typedef struct VL53L0X_ioctl_s {
  uint32_t  reg;
  uint32_t  val;
  int       ret;
} VL53L0X_ioctl_t;

#define VL53L0X_IOCTL_READ                0x0000
#define VL53L0X_IOCTL_WRITE               0x8000
#define VL53L0X_IOCTL_CMD_MASK            0x7FFF

#define VL53L0X_IOCTL_RESET               0x0000
#define VL53L0X_IOCTL_PROFILE             0x0001

#define VL53L0X_IOCTL_LIMIT_CHECK_ENABLE  0x0100
#define VL53L0X_IOCTL_LIMIT_CHECK_VALUE   0x0101
#define VL53L0X_IOCTL_MEASURE_TIME_US     0x0103
#define VL53L0X_IOCTL_VCSEL_PULSE_PERIOD  0x0104

#define VL53L0X_IOCTL_DUMP_CONFIG         0x1000


#define VL53L0X_IOCTL_ERROR_NONE                                 0
/* Errors returned from the VL53L0X low level driver.
 * See STMicroelectronics documentation */
#define VL53L0X_IOCTL_ERROR_CALIBRATION_WARNING                 -1
#define VL53L0X_IOCTL_ERROR_MIN_CLIPPED                         -2
#define VL53L0X_IOCTL_ERROR_UNDEFINED                           -3
#define VL53L0X_IOCTL_ERROR_INVALID_PARAMS                      -4
#define VL53L0X_IOCTL_ERROR_NOT_SUPPORTED                       -5
#define VL53L0X_IOCTL_ERROR_RANGE_ERROR                         -6
#define VL53L0X_IOCTL_ERROR_TIME_OUT                            -7
#define VL53L0X_IOCTL_ERROR_MODE_NOT_SUPPORTED                  -8
#define VL53L0X_IOCTL_ERROR_BUFFER_TOO_SMALL                    -9
#define VL53L0X_IOCTL_ERROR_GPIO_NOT_EXISTING                  -10
#define VL53L0X_IOCTL_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED   -11
#define VL53L0X_IOCTL_ERROR_INTERRUPT_NOT_CLEARED              -12
#define VL53L0X_IOCTL_ERROR_CONTROL_INTERFACE                  -20
#define VL53L0X_IOCTL_ERROR_INVALID_COMMAND                    -30
#define VL53L0X_IOCTL_ERROR_DIVISION_BY_ZERO                   -40
#define VL53L0X_IOCTL_ERROR_REF_SPAD_INIT                      -50
#define VL53L0X_IOCTL_ERROR_NOT_IMPLEMENTED                    -99

/* Errors specific to the ioctl interface  */
#define VL53L0X_IOCTL_ERROR_INVALID_REQUEST                   -200
#define VL53L0X_IOCTL_ERROR_NOT_SUPPORTED                     -210
#define VL53L0X_IOCTL_ERROR_READ_NOT_SUPPORTED                -211
#define VL53L0X_IOCTL_ERROR_WRITE_NOT_SUPPORTED               -212
#define VL53L0X_IOCTL_ERROR_INVALID_PROFILE                   -220
#define VL53L0X_IOCTL_ERROR_INVALID_RANGING_MODE              -221

/* Each read results in the following structure being returned */
typedef struct VL53L0X_data_s {
	uint16_t  range;
	uint16_t  maxRange;
	float     reflectance;
	float     ambientLight;
	uint16_t  spadCount;
	uint8_t   zoneId;
	uint8_t   status;
} VL53L0X_data_t;

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

struct i2c_master_s;
int VL53L0X_register(FAR const char *devpath, FAR struct i2c_master_s *i2c, uint8_t addr, uint32_t speed);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif
