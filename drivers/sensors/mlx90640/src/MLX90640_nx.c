#include <nuttx/config.h>

#include "MLX90640_API.h"

#include <errno.h>
#include <nuttx/kmalloc.h>
#include <nuttx/sensors/mlx90640.h>
#include <string.h>

int MLX90640_I2CRead(MLX90640_nx_dev_t *dev, uint16_t addr, uint16_t wordCount, uint16_t *data)
{
  int ret;
#if 0
  struct i2c_msg_s msg[2];
#endif
  uint8_t *dp = (uint8_t*)data;

  uint8_t buf[2];
  buf[0] = addr >> 8;
  buf[1] = addr & 0x00FF;

#if 0
  /* setup addr transfer */
  msg[0].frequency = dev->i2cConfig.frequency;
  msg[0].addr      = dev->i2cConfig.address;
  msg[0].flags     = 0;
  msg[0].buffer    = buf;
  msg[0].length    = 2;

  /* setup data transfer */
  msg[1].frequency = dev->i2cConfig.frequency;
  msg[1].addr      = dev->i2cConfig.address;
  msg[1].flags     = I2C_M_READ | I2C_M_NOACK;
  msg[1].buffer    = dp;
  msg[1].length    = 2*wordCount;
#endif

  for (ret = -EAGAIN; ret < 0; )
  {
#if 0
    ret = I2C_TRANSFER(dev->i2c, msg, 2);
#else
    ret = i2c_writeread(dev->i2c, &dev->i2cConfig, buf, 2, dp, 2*wordCount);
#endif
    if (ret < 0 && ret != -EAGAIN)
    {
      snerr("i2c_write failed: %d\n", ret);
      return ret;
    }
    if (ret == -EAGAIN) break;
  }
 
  for (int w=0; w<wordCount; ++w)
  {
    data[w] = (uint16_t)dp[2*w] << 8 | (uint16_t)dp[2*w+1];
  }

  return OK;
}

int MLX90640_I2CWrite(MLX90640_nx_dev_t *dev, uint16_t addr, uint16_t value)
{
  int ret;
#if 0
  struct i2c_msg_s msg[1];
#endif
  uint16_t verify;

  uint8_t buf[4];
  buf[0] = addr >> 8;
  buf[1] = addr & 0x00FF;
  buf[2] = value >> 8;
  buf[3] = value & 0x00FF;

#if 0
  /* setup write transfer */
  msg[0].frequency = dev->i2cConfig.frequency;
  msg[0].addr      = dev->i2cConfig.address;
  msg[0].flags     = 0;
  msg[0].buffer    = buf;
  msg[0].length    = 4;
#endif

  for (ret = -EAGAIN; ret < 0; )
  {
#if 0
    ret = I2C_TRANSFER(dev->i2c, msg, 1);
#else
    ret = i2c_write(dev->i2c, &dev->i2cConfig, buf, 4);
#endif
    if (ret < 0 && ret != -EAGAIN)
    {
      snerr("i2c_write failed: %d\n", ret);
      return ret;
    }
    if (ret == -EAGAIN) break;
  }

  MLX90640_I2CRead(dev, addr, 1, &verify);
  if (value != verify) {
    return -2;
  }

  return OK;
}

static int mlx90640_open(FAR struct file *fp);
static int mlx90640_close(FAR struct file *fp);
static ssize_t mlx90640_read(FAR struct file *fp, FAR char *buf, size_t len);
static int mlx90640_ioctl(FAR struct file *fp, int cmd, unsigned long arg);

static const struct file_operations mlx90640_fops =
{
  mlx90640_open,
  mlx90640_close,
  mlx90640_read,
  NULL,            /* write */
  NULL,            /* seek */
  mlx90640_ioctl
#ifndef CONFIG_DISABLE_POLL
  , NULL           /* poll */
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL           /* unlink */
#endif
};

static int mlx90640_open(FAR struct file *fp)
{
  int ret = OK;
  FAR MLX90640_dev_t *priv = (FAR MLX90640_dev_t*)(fp->f_inode->i_private);
  uint16_t eeBuf[832];
  int rc;

  I2C_RESET(priv->nx.i2c);

  rc = MLX90640_SetDeviceMode(priv, 0);
  syslog(LOG_INFO, "MLX90640_SetDeviceMode: %d\n", rc);
  rc = MLX90640_SetSubPageRepeat(priv, 0);
  syslog(LOG_INFO, "MLX90640_SetSubPageRepeat: %d\n", rc);
  rc = MLX90640_SetRefreshRate(priv, 0x01);
  syslog(LOG_INFO, "MLX90640_SetRefreshRate: %d\n", rc);
  rc = MLX90640_SetChessMode(priv);
  syslog(LOG_INFO, "MLX90640_SetChessMode: %d\n", rc);

  rc = MLX90640_DumpEE(priv, eeBuf);
  syslog(LOG_INFO, "MLX90640_DumpEE: %d\n", rc);
  rc = MLX90640_ExtractParameters(eeBuf, &priv->param);
  syslog(LOG_INFO, "MLX90640_ExtractParameters: %d\n", rc);

  return ret;
}

static int mlx90640_close(FAR struct file *fp)
{
  int ret = OK;
  return ret;
}

static ssize_t mlx90640_read(FAR struct file *fp, FAR char *buf, size_t len)
{
  FAR MLX90640_dev_t *priv = (FAR MLX90640_dev_t*)(fp->f_inode->i_private);
  static float  image[768];
  uint16_t      frame[834];
  float         eTa, emissivity = 1;
  int rc;

  if (0 == fp->f_pos) {
    rc = MLX90640_GetFrameData(priv, frame);
    syslog(LOG_INFO, "MLX90640_GetFrameData: %d\n", rc);
    eTa = MLX90640_GetTa(frame, &priv->param);
    MLX90640_CalculateTo(frame, &priv->param, emissivity, eTa, image);
    MLX90640_BadPixelsCorrection(priv->param.brokenPixels, image, 1, &priv->param);
    MLX90640_BadPixelsCorrection(priv->param.outlierPixels, image, 1, &priv->param);
  }

  if ((fp->f_pos + len) > sizeof(image)) {
    len = sizeof(image) - fp->f_pos;
  }

  memcpy((uint8_t*)image + fp->f_pos, buf, len);
  fp->f_pos += len;
  return len;
}

static int mlx90640_ioctl(FAR struct file *fp, int cmd, unsigned long arg)
{
  int ret = OK;
  return ret;
}

int MLX90640_register(FAR const char *devpath, FAR struct i2c_master_s *i2c, uint8_t addr, uint32_t freq)
{
  int ret;
  FAR MLX90640_dev_t *priv = 0;

  priv = (FAR MLX90640_dev_t*)kmm_malloc(sizeof(MLX90640_dev_t));
  if (priv == NULL)
  {
    snerr("ERROR: Failed to allocate MLX90640_dev_t\n");
    return -ENOMEM;
  }

  memset(priv, 0, sizeof(MLX90640_dev_t));
  priv->nx.i2c = i2c;
  priv->nx.i2cConfig.frequency = I2C_SPEED_HIGH; //freq;
  priv->nx.i2cConfig.address = addr;
  priv->nx.i2cConfig.addrlen = 7;

  ret = register_driver(devpath, &mlx90640_fops, 0666, priv);
  if (ret < 0)
  {
    snerr("ERROR: Failed to register driver: %d\n", ret);
    goto err_out;
  }

  return OK;

err_out:
  kmm_free(priv);
  return ret;
}
