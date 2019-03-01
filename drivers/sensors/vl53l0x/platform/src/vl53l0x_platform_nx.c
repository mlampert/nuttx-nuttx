#include <nuttx/config.h>

#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"
#include "vl53l0x_platform_nx.h"

#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/sensors/vl53l0x.h>


#define VL53L0X_DEBUG 1

static const VL53L0X_nx_profile_values_t VL53L0X_Profile[] = {
  /* Default */
  {
    { 1, 0 },
    { 1, 0 },
    { 1, (FixPoint1616_t)(1.5*0.023*65536) },
    0,
    0, 0
  },
  /* Long Range */
  {
    { 1, (FixPoint1616_t)(60*65536) },
    { 1, (FixPoint1616_t)(65536/10) },
    { 0, 0 },
    33000,
    18, 14
  },
  /* High Speed */
  {
    { 1, (FixPoint1616_t)(32*65536) },
    { 1, (FixPoint1616_t)(25*65536/100) },
    { 0, 0 },
    30000,
    0, 0
  },
  /* High Accuracy */
  {
    { 1, (FixPoint1616_t)(18*65536) },
    { 1, (FixPoint1616_t)(25*65536/100) },
    { 0, 0 }, 
    200000,
    0, 0
  }
};

#define ERR_OUT_IF_RET(ret, label) \
  if (ret != VL53L0X_ERROR_NONE)\
  {\
    print_pal_error(label, ret);\
    goto err_out;\
  }

#define VERSION_REQUIRED_MAJOR      1
#define VERSION_REQUIRED_MINOR      0
#define VERSION_REQUIRED_BUILD      2

static void dump_parameters(FAR VL53L0X_DEV priv)
{
  int ret;
  uint8_t period;
  VL53L0X_DeviceParameters_t param;

  ret = VL53L0X_GetDeviceParameters(priv, &param);
  if (ret == VL53L0X_ERROR_NONE)
  {
    uint32_t timing;
    ret = VL53L0X_GetMeasurementTimingBudgetMicroSeconds(priv, &timing);
    syslog(LOG_INFO, "Device Parameters:\n");
    syslog(LOG_INFO, "               mode : %d.%d\n", param.DeviceMode, param.HistogramMode);
    syslog(LOG_INFO, "             timing : %d + %d  (%d)\n", param.MeasurementTimingBudgetMicroSeconds, param.InterMeasurementPeriodMilliSeconds * 1000, timing);
    syslog(LOG_INFO, "         xtalk comp : %d - %dmm %dMCps\n", param.XTalkCompensationEnable, param.XTalkCompensationRangeMilliMeter, param.XTalkCompensationRateMegaCps);
    syslog(LOG_INFO, "       range offset : %dum\n", param.RangeOffsetMicroMeters);
    for (int i=0; i< VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS; ++i) {
      syslog(LOG_INFO, "    limit check [%d] : %d/%3d  %08X\n", i, param.LimitChecksEnable[i], param.LimitChecksStatus[i], param.LimitChecksValue[i]);
    }
    syslog(LOG_INFO, "         wrap check : %d\n", param.WrapAroundCheckEnable);
    ret = VL53L0X_GetVcselPulsePeriod(priv, VL53L0X_VCSEL_PERIOD_PRE_RANGE, &period);
    syslog(LOG_INFO, "   vcsel period pre : %d\n", period);
    ret = VL53L0X_GetVcselPulsePeriod(priv, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, &period);
    syslog(LOG_INFO, " vcsel period final : %d\n", period);

    syslog(LOG_INFO, "   timing : %d - %d\n", priv->Data.CurrentParameters.MeasurementTimingBudgetMicroSeconds, priv->Data.DeviceSpecificParameters.FinalRangeVcselPulsePeriod);
  } else {
    syslog(LOG_ERR, "get device parametes failed: %d\n", ret);
  }

  syslog(LOG_INFO, " - sigmaFinalRange      : %d - %08x\n", priv->nx.init.sigmaFinalRange.enable, priv->nx.init.sigmaFinalRange.value);
  syslog(LOG_INFO, " - signalRateFinalRange : %d - %08x\n", priv->nx.init.signalRateFinalRange.enable, priv->nx.init.signalRateFinalRange.value);
  syslog(LOG_INFO, " - rangeIgnoreThreshold : %d - %08x\n", priv->nx.init.rangeIgnoreThreshold.enable, priv->nx.init.rangeIgnoreThreshold.value);
  syslog(LOG_INFO, " - timing               : %d\n", priv->nx.init.measurementTimingBudget);
  syslog(LOG_INFO, " - vcsel                : %d / %d\n", priv->nx.init.vcselPeriodPreRange, priv->nx.init.vcselPeriodFinalRange);
}

/****************************************************************************
 * Name: FOPS
 ****************************************************************************/

static int vl53l0x_open(FAR struct file *fp);
static int vl53l0x_close(FAR struct file *fp);
static ssize_t vl53l0x_read(FAR struct file *fp, FAR char *buf, size_t len);
static int vl53l0x_ioctl(FAR struct file *fp, int cmd, unsigned long arg);

static const struct file_operations vl53l0x_fops =
{
  vl53l0x_open,
  vl53l0x_close,
  vl53l0x_read,
  NULL,            /* write */
  NULL,            /* seek */
  vl53l0x_ioctl
#ifndef CONFIG_DISABLE_POLL
  , NULL           /* poll */
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL           /* unlink */
#endif
};

static void print_pal_error(const char *label, VL53L0X_Error rc)
{
  char buf[VL53L0X_MAX_STRING_LENGTH];
  VL53L0X_GetPalErrorString(rc, buf);
#if VL53L0X_DEBUG
  syslog(LOG_ERR, "%s: %i - %s\n", label, rc, buf);
#else
  snerr("%s: %i - %s\n", label, rc, buf);
#endif
}

static int vl53l0x_open(FAR struct file *fp)
{
#if 0
  FAR VL53L0X_DEV priv = (FAR VL53L0X_DEV)(fp->f_inode->i_private);
  syslog(LOG_INFO, "vl5310x_open\n");
#endif

  return OK;
}
static int vl53l0x_close(FAR struct file *fp)
{
#if 0
  FAR VL53L0X_DEV priv = (FAR VL53L0X_DEV)(fp->f_inode->i_private);
  syslog(LOG_INFO, "vl5310x_close\n");
#endif

  return OK;
}
static ssize_t vl53l0x_read(FAR struct file *fp, FAR char *buf, size_t len)
{
  int ret;
  VL53L0X_RangingMeasurementData_t measurement;
  VL53L0X_data_t data;
  FAR VL53L0X_DEV priv = (FAR VL53L0X_DEV)(fp->f_inode->i_private);
  //syslog(LOG_DEBUG, "vl5310x_read(%d)\n", len);

#if 0
  {
    syslog(LOG_INFO, "vl5310x_read - EOF\n");
    return 0; /* EOF */
  }
#endif

  ret = VL53L0X_PerformSingleRangingMeasurement(priv, &measurement);
  if (ret < 0)
  {
    print_pal_error("ranging failed", ret);
    return 0;
  }
  data.range            = measurement.RangeMilliMeter;
  data.maxRange         = measurement.RangeDMaxMilliMeter;
  data.reflectance      = measurement.SignalRateRtnMegaCps  / 65536.0f;
  data.ambientLight     = measurement.AmbientRateRtnMegaCps / 65536.0f;
	data.spadCount        = measurement.EffectiveSpadRtnCount;
  data.zoneId           = measurement.ZoneId;
	data.status           = measurement.RangeStatus;

  if (len > sizeof(data))
  {
    len = sizeof(data);
  }

  memcpy(buf, &data, len);
  fp->f_pos += len;
  return len;
}

#define VL53L0X_SETUP(ARG, CALL)            \
      ARG->ret = CALL;                      \
      if (ARG->ret != VL53L0X_ERROR_NONE)   \
      {                                     \
        return OK;                          \
      }

#define VL53L0X_SETUP_LIMIT_CHECK(ARG, PRIV, PRF, LIMIT, MEMBER)                                          \
      VL53L0X_SETUP(ARG, VL53L0X_SetLimitCheckEnable(PRIV, LIMIT, VL53L0X_Profile[PRF].MEMBER.enable));   \
      if (VL53L0X_Profile[PRF].MEMBER.enable && VL53L0X_Profile[PRF].MEMBER.value)                        \
      {                                                                                                   \
        VL53L0X_SETUP(ARG, VL53L0X_SetLimitCheckValue(PRIV, LIMIT, VL53L0X_Profile[PRF].MEMBER.value));   \
      } else {                                                                                            \
        VL53L0X_SETUP(ARG, VL53L0X_SetLimitCheckValue(PRIV, LIMIT, PRIV->nx.init.MEMBER.value));          \
      }

#define VL53L0X_SETUP_VALUE(ARG, PRIV, PRF, FUNC, MEMBER)             \
      if (VL53L0X_Profile[PRF].MEMBER)                                \
      {                                                               \
        VL53L0X_SETUP(ARG, FUNC(PRIV, VL53L0X_Profile[PRF].MEMBER));  \
      } else {                                                        \
        VL53L0X_SETUP(ARG, FUNC(PRIV, PRIV->nx.init.MEMBER));         \
      }

#define VL53L0X_SETUP_VALUE2(ARG, PRIV, PRF, FUNC, VAL, MEMBER)             \
      if (VL53L0X_Profile[PRF].MEMBER)                                      \
      {                                                                     \
        VL53L0X_SETUP(ARG, FUNC(PRIV, VAL, VL53L0X_Profile[PRF].MEMBER));   \
      } else {                                                              \
        VL53L0X_SETUP(ARG, FUNC(PRIV, VAL, PRIV->nx.init.MEMBER));          \
      }

static int VL53L0X_setup_profile(FAR VL53L0X_DEV priv, FAR VL53L0X_ioctl_t *arg)
{
  uint16_t mod = arg->val & VL53L0X_PROFILE_MASK_MODE;
  uint16_t prf = arg->val & VL53L0X_PROFILE_MASK_PROFILE;

  switch (mod)
  {
    case VL53L0X_PROFILE_MODE_SINGLE:
      VL53L0X_SETUP(arg, VL53L0X_SetDeviceMode(priv, VL53L0X_DEVICEMODE_SINGLE_RANGING));
      break;

    case VL53L0X_PROFILE_MODE_CONT:
      arg->ret = VL53L0X_IOCTL_ERROR_NOT_SUPPORTED;
      return -EINVAL;
      break;

    default:
      arg->ret = VL53L0X_IOCTL_ERROR_INVALID_RANGING_MODE;
      return -EINVAL;
  }

  switch (prf)
  {
    case VL53L0X_PROFILE_DEFAULT:
    case VL53L0X_PROFILE_LONG_RANGE:
    case VL53L0X_PROFILE_HIGH_SPEED:
    case VL53L0X_PROFILE_HIGH_ACCURACY:
      VL53L0X_SETUP_LIMIT_CHECK(arg, priv, prf, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, sigmaFinalRange);                                          \
      VL53L0X_SETUP_LIMIT_CHECK(arg, priv, prf, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, signalRateFinalRange);                                          \
      VL53L0X_SETUP_LIMIT_CHECK(arg, priv, prf, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, rangeIgnoreThreshold);                                          \
      VL53L0X_SETUP_VALUE(arg, priv, prf, VL53L0X_SetMeasurementTimingBudgetMicroSeconds, measurementTimingBudget);             \
      VL53L0X_SETUP_VALUE2(arg, priv, prf, VL53L0X_SetVcselPulsePeriod, VL53L0X_VCSEL_PERIOD_PRE_RANGE, vcselPeriodPreRange);
      VL53L0X_SETUP_VALUE2(arg, priv, prf, VL53L0X_SetVcselPulsePeriod, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, vcselPeriodFinalRange);
      break;

    default:
      arg->ret = VL53L0X_IOCTL_ERROR_INVALID_PROFILE;
      return -EINVAL;
  }

  return OK;
}

static int vl53l0x_ioctl(FAR struct file *fp, int cmd, unsigned long param)
{
  int ret = OK;
  FAR VL53L0X_DEV     priv = (FAR VL53L0X_DEV)(fp->f_inode->i_private);
  FAR VL53L0X_ioctl_t *arg = (FAR VL53L0X_ioctl_t*)param;
  bool write = (cmd & VL53L0X_IOCTL_WRITE) == VL53L0X_IOCTL_WRITE;
  uint8_t byte0, byte1;
  uint32_t uval;

  syslog(LOG_INFO, "vl5310x_ioctl(%04X)\n", cmd);

  switch (cmd & VL53L0X_IOCTL_CMD_MASK)
  {
    case VL53L0X_IOCTL_RESET:
      if (write)
      {
        arg->ret = VL53L0X_ResetDevice(priv);
        if (arg->ret != VL53L0X_ERROR_NONE)
        {
          break;
        }
        arg->ret = VL53L0X_DataInit(priv);
        if (arg->ret != VL53L0X_ERROR_NONE)
        {
          break;
        }
        arg->ret = VL53L0X_StaticInit(priv);
        if (arg->ret != VL53L0X_ERROR_NONE)
        {
          break;
        }
        arg->ret = VL53L0X_PerformRefCalibration(priv, &byte0, &byte1);
        if (arg->ret != VL53L0X_ERROR_NONE)
        {
          break;
        }
        arg->ret = VL53L0X_PerformRefSpadManagement(priv, &uval, &byte0);
        if (arg->ret != VL53L0X_ERROR_NONE)
        {
          break;
        }
        syslog(LOG_INFO, "vl5310x successfully reset\n");
      } else {
        arg->ret = VL53L0X_IOCTL_ERROR_READ_NOT_SUPPORTED;
        ret = -EINVAL;
      }
      break;

    case VL53L0X_IOCTL_PROFILE:
      if (write)
      {
        ret = VL53L0X_setup_profile(priv, arg);
      } else {
        arg->ret = VL53L0X_IOCTL_ERROR_READ_NOT_SUPPORTED;
        ret = -EINVAL;
      }
      break;

    case VL53L0X_IOCTL_LIMIT_CHECK_ENABLE:
      if (write)
      {
        arg->ret = VL53L0X_SetLimitCheckEnable(priv, arg->reg, arg->val);
      } else {
        uint8_t enabled = 0;
        arg->ret = VL53L0X_GetLimitCheckEnable(priv, arg->reg, &enabled);
        arg->val = enabled;
      }
      break;

    case VL53L0X_IOCTL_LIMIT_CHECK_VALUE:
      if (write)
      {
        arg->ret = VL53L0X_SetLimitCheckValue(priv, arg->reg, (FixPoint1616_t)arg->val);
      } else {
        FixPoint1616_t fpoint = 0;
        arg->ret = VL53L0X_GetLimitCheckValue(priv, arg->reg, &fpoint);
        arg->val = fpoint;
      }
      break;

    case VL53L0X_IOCTL_MEASURE_TIME_US:
      if (write)
      {
        arg->ret = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(priv, arg->val);
      } else {
        arg->ret = VL53L0X_GetMeasurementTimingBudgetMicroSeconds(priv, &arg->val);
      }
      break;

    case VL53L0X_IOCTL_VCSEL_PULSE_PERIOD:
      if (write)
      {
        arg->ret = VL53L0X_SetVcselPulsePeriod(priv, arg->reg, arg->val);
      } else {
        uint8_t period = 0;
        arg->ret = VL53L0X_GetVcselPulsePeriod(priv, arg->reg, &period);
        arg->val = period;
      }
      break;

    case VL53L0X_IOCTL_DUMP_CONFIG:
      dump_parameters(priv);
      break;

    default:
      arg->ret = VL53L0X_IOCTL_ERROR_INVALID_REQUEST;
      ret = -EINVAL;
      break;
  }
  return ret;
}

/****************************************************************************
 * Name: platform API
 ****************************************************************************/

int VL53L0X_nx_read(VL53L0X_nx_dev_t *dev, uint8_t index, FAR uint8_t *data, uint32_t count)
{
  int ret;
  struct i2c_msg_s msg[2];

  /* setup index transfer */
  msg[0].frequency = dev->i2cConfig.frequency;
  msg[0].addr      = dev->i2cConfig.address;
  msg[0].flags     = 0;
  msg[0].buffer    = &index;
  msg[0].length    = 1;

  /* setup data transfer */
  msg[1].frequency = dev->i2cConfig.frequency;
  msg[1].addr      = dev->i2cConfig.address;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = data;
  msg[1].length    = count;

  for (ret = -EAGAIN; ret < 0; )
  {
    ret = I2C_TRANSFER(dev->i2c, msg, 2);
    if (ret < 0 && ret != -EAGAIN)
    {
      snerr("i2c_write failed: %d\n", ret);
      return ret;
    }
  }
 
  return OK;
}

int VL53L0X_nx_write(VL53L0X_nx_dev_t *dev, uint8_t index, FAR const uint8_t *data, uint32_t count)
{
  int ret;
  struct i2c_msg_s msg[2];

  /* setup index transfer */
  msg[0].frequency = dev->i2cConfig.frequency;
  msg[0].addr      = dev->i2cConfig.address;
  msg[0].flags     = I2C_M_NOSTOP;
  msg[0].buffer    = &index;
  msg[0].length    = 1;

  /* setup data transfer */
  msg[1].frequency = dev->i2cConfig.frequency;
  msg[1].addr      = dev->i2cConfig.address;
  msg[1].flags     = I2C_M_NOSTART;
  msg[1].buffer    = (FAR uint8_t*)data;
  msg[1].length    = count;

  for (ret = -EAGAIN; ret < 0; )
  {
    ret = I2C_TRANSFER(dev->i2c, msg, 2);
    if (ret < 0 && ret != -EAGAIN)
    {
      snerr("i2c_write failed: %d\n", ret);
      return ret;
    }
  }

  //syslog(LOG_NOTICE, "VL53L0X_nx_write(%02X, %d - %02X.%02X): %d\n", index, count, data[0], data[1], ret);
  return OK;
}

static void dump_status(FAR VL53L0X_DEV priv)
{
#if 0
  int ret = 0;
  uint8_t refByte[3];
  uint16_t refWord[2];
  uint32_t count;

  ret |= VL53L0X_RdByte(priv, 0xC0, &refByte[0]);
  ret |= VL53L0X_RdByte(priv, 0xC1, &refByte[1]);
  ret |= VL53L0X_RdByte(priv, 0xC2, &refByte[2]);
  ret |= VL53L0X_RdWord(priv, 0x51, &refWord[0]);
  ret |= VL53L0X_RdWord(priv, 0x61, &refWord[1]);
  syslog(LOG_NOTICE, "ref bytes: %02x.%02x.%02x words: %04x, %04x\n", refByte[0], refByte[1], refByte[2], refWord[0], refWord[1]);


  ret = VL53L0X_GetDeviceErrorStatus(priv, &refByte[0]);
  syslog(LOG_NOTICE, "error status (%d): %d\n", ret, refByte[0]);

  ret = VL53L0X_GetProductRevision(priv, &refByte[0], &refByte[1]);
  syslog(LOG_NOTICE, "prod revision (%d): %d.%d\n", ret, refByte[0], refByte[1]);

  ret = VL53L0X_GetPalState(priv, &refByte[0]);
  syslog(LOG_NOTICE, "pal state (%d): %d\n", ret, refByte[0]);

  ret = VL53L0X_GetPowerMode(priv, &refByte[0]);
  syslog(LOG_NOTICE, "power mode (%d): %d\n", ret, refByte[0]);

  ret = VL53L0X_GetRefCalibration(priv, &refByte[0], &refByte[1]);
  syslog(LOG_NOTICE, "ref calibration (%d): %d.%d\n", ret, refByte[0], refByte[1]);

  ret = VL53L0X_GetReferenceSpads(priv, &count, &refByte[0]);
  syslog(LOG_NOTICE, "ref spads (%d): %d.%d\n", ret, count, refByte[0]);

  /*
  ret = VL53L0X_SetReferenceSpads(priv, count, refByte[0]);
  syslog(LOG_NOTICE, "set ref spads (%d): %d.%d\n", ret, count, refByte[0]);
  */

  ret = VL53L0X_GetSpadAmbientDamperThreshold(priv, &refWord[0]);
  syslog(LOG_NOTICE, "spad damper thershold (%d): %d\n", ret, refWord[0]);

  ret = VL53L0X_GetSpadAmbientDamperFactor(priv, &refWord[0]);
  syslog(LOG_NOTICE, "spad damper factor (%d): %d\n", ret, refWord[0]);

  ret = VL53L0X_SetSpadAmbientDamperFactor(priv, 1);
  syslog(LOG_NOTICE, "set spad damper factor (%d):\n", ret);
#endif
}

static void VL53L0x_setup_limit_check(FAR VL53L0X_nx_limit_check_t *limit, FAR VL53L0X_DEV priv, uint8_t idx)
{
  limit->enable = priv->Data.CurrentParameters.LimitChecksEnable[idx];
  limit->value  = priv->Data.CurrentParameters.LimitChecksValue[idx];
}

int VL53L0X_register(FAR const char *devpath, FAR struct i2c_master_s *i2c, uint8_t addr, uint32_t freq)
{
  int ret;
  VL53L0X_Version_t version;
  VL53L0X_DeviceInfo_t info;
  FAR VL53L0X_DEV priv;
  uint8_t vhvSettings, phaseCal;
  uint32_t refSpadCount;
  uint8_t isApertureSpads;
  VL53L0X_ioctl_t ioctl;

  priv = (FAR VL53L0X_DEV)kmm_malloc(sizeof(VL53L0X_Dev_t));

  if (priv == NULL)
  {
    snerr("ERROR: Failed to allocate VL53L0X_Dev_t\n");
    return -ENOMEM;
  }

  memset(priv, 0, sizeof(VL53L0X_Dev_t));
  priv->nx.i2c = i2c;
  priv->nx.i2cConfig.frequency = freq;
  priv->nx.i2cConfig.address = addr;
  priv->nx.i2cConfig.addrlen = 7;

  ret = VL53L0X_GetVersion(&version);
  if (ret != VL53L0X_ERROR_NONE)
  {
    snerr("ERROR: Failed to get API version: %d\n", ret);
    goto err_out;
  }

  if (version.major != VERSION_REQUIRED_MAJOR ||
      version.minor != VERSION_REQUIRED_MINOR ||
      version.build != VERSION_REQUIRED_BUILD)
  {
    snerr("ERROR: API version mismatch %d.%d.%d-%d (req. %d.%d.%d)\n", version.major, version.minor, version.build, version.revision, VERSION_REQUIRED_MAJOR, VERSION_REQUIRED_MINOR, VERSION_REQUIRED_BUILD);
    ret = VL53L0X_ERROR_CONTROL_INTERFACE;
    goto err_out;
  }

  ret = VL53L0X_DataInit(priv);
  print_pal_error("data init", ret);
  if (ret != VL53L0X_ERROR_NONE)
  {
    goto err_out;
  }

  dump_status(priv);

  ret = VL53L0X_GetDeviceInfo(priv, &info);
  ERR_OUT_IF_RET(ret, "Failed to get device info");

#if VL53L0X_DEBUG
  syslog(LOG_INFO, "VL53L0X:\n");
  syslog(LOG_INFO, "  name: %s\n", info.Name);
  syslog(LOG_INFO, "  type: %s\n", info.Type);
  syslog(LOG_INFO, "    id: %s\n", info.ProductId);
  syslog(LOG_INFO, "  vers: %d.%d\n", info.ProductRevisionMajor, info.ProductRevisionMinor);
#else
  sninfo("VL53L0X:\n");
  sninfo("  name: %s\n", info.Name);
  sninfo("  type: %s\n", info.Type);
  sninfo("    id: %s\n", info.ProductId);
  sninfo("  vers: %d.%d\n", info.ProductRevisionMajor, info.ProductRevisionMinor);
#endif

  ret = VL53L0X_StaticInit(priv);
  ERR_OUT_IF_RET(ret, "device initialisation");

  ret = VL53L0X_PerformRefCalibration(priv, &vhvSettings, &phaseCal);
  ERR_OUT_IF_RET(ret, "reference calibration");

  ret = VL53L0X_PerformRefSpadManagement(priv, &refSpadCount, &isApertureSpads);
  ERR_OUT_IF_RET(ret, "reference spad management");

  /* Copy the initial values so they can be restored if profiles don't overwrite them */
  VL53L0x_setup_limit_check(&priv->nx.init.sigmaFinalRange, priv, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE);
  VL53L0x_setup_limit_check(&priv->nx.init.signalRateFinalRange, priv, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE);
  VL53L0x_setup_limit_check(&priv->nx.init.rangeIgnoreThreshold, priv, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD);
  priv->nx.init.vcselPeriodPreRange     = priv->Data.DeviceSpecificParameters.PreRangeVcselPulsePeriod;
  priv->nx.init.vcselPeriodFinalRange   = priv->Data.DeviceSpecificParameters.FinalRangeVcselPulsePeriod;
  priv->nx.init.measurementTimingBudget = priv->Data.CurrentParameters.MeasurementTimingBudgetMicroSeconds;

  dump_parameters(priv);

  ioctl.val = VL53L0X_PROFILE_DEFAULT | VL53L0X_PROFILE_MODE_SINGLE;
  ret = VL53L0X_setup_profile(priv, &ioctl);
  ERR_OUT_IF_RET(ret, "ioctl");
  ERR_OUT_IF_RET(ioctl.ret, "ioctl ret");

  dump_parameters(priv);

  ret = register_driver(devpath, &vl53l0x_fops, 0666, priv);
  if (ret < 0)
  {
    snerr("ERROR: Failed to register driver: %d\n", ret);
    goto err_out;
  }
  return ret;

err_out:
  dump_status(priv);

  kmm_free(priv);
  return ret;
}
