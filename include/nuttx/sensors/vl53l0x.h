#ifndef __INCLUDE_NUTTX_SENSORS_vl53l0x_h
#define __INCLUDE_NUTTX_SENSORS_vl53l0x_h

#include <stdint.h>

/* NuttX uses the upper 7 bit shifted right as the address -> 0x52 / 2 = 0x29 */
#define VL53L0X_I2C_ADDR    0x29
#define VL53L0X_I2C_FREQ    400000

/* The datasheet for the VL53L0X specifies 4 different measurement profiles
 * which can be set with the following constants.
 * \c VL53L0X_PROFILE_CURRENT and \c VL53L0X_PROFILE_CUSTOM can be used to set
 * a custom profile or retrieve the currently active values.
 * \c VL53L0X_PROFILE_INITIAL can be used to retrieve the pfofile values as
 * they were after device initialisation, before a profile was set. These values
 * are filled in if a profile has \c 0 as a value.
 * */
#define VL53L0X_PROFILE_DEFAULT         0x0000
#define VL53L0X_PROFILE_LONG_RANGE      0x0001
#define VL53L0X_PROFILE_HIGH_SPEED      0x0002
#define VL53L0X_PROFILE_HIGH_ACCURACY   0x0003
#define VL53L0X_PROFILE_CURRENT         0x0004
#define VL53L0X_PROFILE_CUSTOM          0x0004
#define VL53L0X_PROFILE_INITIAL         0x0005

#define VL53L0X_PROFILE_MODE_SINGLE     0x0000
#define VL53L0X_PROFILE_MODE_CONT       0x0001

#define VL53L0X_PROFILE_MASK_PROFILE    0x00FF
#define VL53L0X_PROFILE_MASK_MODE       0xFF00

/* According to the API the type of a VL53L0X is supposed to be 1,
 * (see ProductType in vl53l0x_def.h), however I always read 0xEE */
#define VL53L0X_PRODUCT_TYPE_VL53L0X    0xEE
#define VL53L0X_PRODUCT_TYPE_VL53L1X    2

#define VL53L0X_CHECK_ENABLE_SIGMA_FINAL_RANGE          0x01
#define VL53L0X_CHECK_ENABLE_SIGNAL_RATE_FINAL_RANGE    0x02
#define VL53L0X_CHECK_ENABLE_RANGE_IGN_THRESHOLD        0x04

/* A profile is defined by the values below. Any value \c 0
 * is replaced with the initial value for that parameter.
 * \c checkEnable holds the bitmask of the enabled checks,
 * see above for supported checks.
 * \c sigmaFinalRange, \c signalRateFinalRange and 'c rangeIgnoreThreshold
 * are fixed point 16.16 values.
 * \c measurementTimingBudget is in \c us
 * */
typedef struct VL53L0X_profile_s {
  uint8_t  checkEnable;
  uint32_t sigmaFinalRange;
  uint32_t signalRateFinalRange;
  uint32_t rangeIgnoreThreshold;
  uint32_t measurementTimingBudget;
  uint8_t  vcselPeriodPreRange;
  uint8_t  vcselPeriodFinalRange;
} VL53L0X_profile_t;

typedef struct VL53L0X_ioctl_s {
  union {
    /* Structure filled or read by \c VL53L0X_IOCTL_PROFILE command.
     * \c id must be set.
     * \c param is ignored for writing other than \c VL53L0X_PROFILE_CUSTOM
     * \c VL53L0X_PROFILE_CURRENT is only valid for reading */
    struct {
      uint8_t           id;
      uint8_t           mode;
      VL53L0X_profile_t param;
    } profile;

    /* Structure filled by \c VL53L0X_IOCTL_INFO command */
    struct {
      char name[32];
      char type[32];
      char id[32];
      uint8_t productType;
      uint8_t versionMajor;
      uint8_t versionMinor;
    } info;

    /* Structure used for the low level commands. */
    struct {
      uint32_t  reg;
      uint32_t  val;
    } limit_check;
    uint32_t timingBudget;
  };

  /* The return value of the device level API calls, see below */
  int       ret;
} VL53L0X_ioctl_t;


/* ioctl commands are either read or write, the same basic command
 * id is used and the direction is or'ed in. */
#define VL53L0X_IOCTL_READ                0x0000
#define VL53L0X_IOCTL_WRITE               0x8000
#define VL53L0X_IOCTL_CMD_MASK            0x7FFF

/* The high level ioctl commands which should be used by the application.
 * See \c VL53L0X_ioctl_t for a description of the expected argument for
 * each command. */
#define VL53L0X_IOCTL_RESET               0x0000
#define VL53L0X_IOCTL_INFO                0x0001
#define VL53L0X_IOCTL_PROFILE             0x0002

/* This is the low level API which should not be used. It is mostly untested
 * and only here in case the API needs expansion I need to do some experiments.
 * Consider it a testing interface. */
#define VL53L0X_IOCTL_LIMIT_CHECK_ENABLE  0x0100
#define VL53L0X_IOCTL_LIMIT_CHECK_VALUE   0x0101
#define VL53L0X_IOCTL_MEASURE_TIME_US     0x0103
#define VL53L0X_IOCTL_VCSEL_PULSE_PERIOD  0x0104


#define VL53L0X_IOCTL_ERROR_NONE                                 0
/* Errors returned from the VL53L0X device level driver.
 * See STMicroelectronics' documentation */
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

/* Status values returned by the device.
 * It seems the \c range value is valid unless there was a phase
 * error.
 *    0 ... clean measurement
 *   -1 ... sigma fail, too much ambient light
 *   -2 ... signal fail, level too low, lots of noise
 *   -3 ... min range fail
 *   -4 ... phase error - no object detected
 *   -5 ... hardware fail, vcsel or vhv failure
 * */
#define VL53L0X_STATUS_VALID       0
#define VL53L0X_STATUS_SIGMA      -1
#define VL53L0X_STATUS_SIGNAL     -2
#define VL53L0X_STATUS_MIN_RANGE  -3
#define VL53L0X_STATUS_PHASE      -4
#define VL53L0X_STATUS_HARDWA     -5

/* Reading from device file returns the following structure.
 *    range       ... distance in mm (only valid if \c status != -4)
 *    maxRange    ... the max distance in mm that can be measured in the current configuration
 *    reflectance ... fixpoint 16.16 value for the object's reflectance
 *    ambientLight... fixpoint 16.16 value for the ambient light level
 *    spadCount   ... no idea
 *    zoneId      ... which zone the detect object belongs to (if set up)
 *    status      ... see above
 * */
typedef struct VL53L0X_data_s {
	uint16_t  range;
	uint16_t  maxRange;
	uint32_t  reflectance;
	uint32_t  ambientLight;
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
