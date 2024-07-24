#include "ilps22qs.h"
#include "utils.h"

#include "hw_platform.h"

#define SPIDELAY 50
#define BOOT_TIME 10

/**
 * @defgroup    Sensitivity
 * @brief       These functions convert raw-data into engineering units.
 * @{
 *
 */

float ilps22qs_from_fs1260_to_hPa(int32_t lsb)
{
  return ((float)lsb / 1048576.0f); /* 4096.0f * 256 */
}

float ilps22qs_from_fs4000_to_hPa(int32_t lsb)
{
  return ((float)lsb / 524288.0f); /* 2048.0f * 256 */
}

float ilps22qs_from_lsb_to_celsius(int16_t lsb)
{
  return ((float)lsb / 100.0f);
}

/**
 * @defgroup  Private_functions
 * @brief     Section collect all the utility functions needed by APIs.
 * @{
 *
 */

static void bytecpy(uint8_t *target, uint8_t *source)
{
  if ((target != NULL) && (source != NULL))
  {
    *target = *source;
  }
}

int32_t ilps22qs_data_get(ilps22qs_md_t *md,
                          ilps22qs_data_t *data, uint8_t w)
{
  uint8_t buff[5];
  int32_t ret;

  ret = ilps22qs_read_reg(ILPS22QS_PRESS_OUT_XL, buff, 5, w);

  /* pressure conversion */
  data->pressure.raw = (int32_t)buff[2];
  data->pressure.raw = (data->pressure.raw * 256) + (int32_t)buff[1];
  data->pressure.raw = (data->pressure.raw * 256) + (int32_t)buff[0];
  data->pressure.raw = data->pressure.raw * 256;

  switch (md->fs)
  {
  case ILPS22QS_1260hPa:
    data->pressure.hpa = ilps22qs_from_fs1260_to_hPa(data->pressure.raw);
    break;
  case ILPS22QS_4060hPa:
    data->pressure.hpa = ilps22qs_from_fs4000_to_hPa(data->pressure.raw);
    break;
  default:
    data->pressure.hpa = 0.0f;
    break;
  }

  /* temperature conversion */
  data->heat.raw = (int16_t)buff[4];
  data->heat.raw = (data->heat.raw * 256) + (int16_t)buff[3];
  data->heat.deg_c = ilps22qs_from_lsb_to_celsius(data->heat.raw);

  return ret;
}



/**
 * @brief  Get the status of all the interrupt sources.[get]
 *
 * @param  val   the status of all the interrupt sources.(ptr)
 * @retval       interface status (MANDATORY: return 0 -> no Error)
 *
 */
int32_t ilps22qs_all_sources_get(ilps22qs_all_sources_t *val, uint8_t w)
{
  ilps22qs_fifo_status2_t fifo_status2;
  ilps22qs_int_source_t int_source;
  ilps22qs_status_t status;
  int32_t ret;

  ret = ilps22qs_read_reg(ILPS22QS_STATUS, (uint8_t *)&status, 1, w);
  if (ret == 0)
  {
    ret = ilps22qs_read_reg(ILPS22QS_INT_SOURCE,
                            (uint8_t *)&int_source, 1, w);
  }
  if (ret == 0)
  {
    ret = ilps22qs_read_reg(ILPS22QS_FIFO_STATUS2,
                            (uint8_t *)&fifo_status2, 1, w);
  }

  val->drdy_pres = status.p_da;
  val->drdy_temp = status.t_da;
  val->over_pres = int_source.ph;
  val->under_pres = int_source.pl;
  val->thrsld_pres = int_source.ia;
  val->fifo_full = fifo_status2.fifo_full_ia;
  val->fifo_ovr = fifo_status2.fifo_ovr_ia;
  val->fifo_th = fifo_status2.fifo_wtm_ia;

  return ret;
}



/**
 * @brief  Sensor conversion parameters selection.[set]
 *
 * @param  val   set the sensor conversion parameters.(ptr)
 * @retval       interface status (MANDATORY: return 0 -> no Error)
 *
 */
int32_t ilps22qs_mode_set(ilps22qs_md_t *val)
{
  ilps22qs_ctrl_reg1_t ctrl_reg1;
  ilps22qs_ctrl_reg2_t ctrl_reg2;
  uint8_t reg[2];
  int32_t ret;

  memset(&ctrl_reg1, 0, sizeof(ilps22qs_ctrl_reg1_t));
  memset(&ctrl_reg2, 0, sizeof(ilps22qs_ctrl_reg2_t));

  ctrl_reg1.odr = (uint8_t)val->odr;
  ctrl_reg1.avg = (uint8_t)val->avg;
  ctrl_reg2.en_lpfp = (uint8_t)val->lpf & 0x01U;
  ctrl_reg2.lfpf_cfg = ((uint8_t)val->lpf & 0x02U) >> 2;
  ctrl_reg2.fs_mode = (uint8_t)val->fs;

  bytecpy(&reg[0], (uint8_t *)&ctrl_reg1);
  bytecpy(&reg[1], (uint8_t *)&ctrl_reg2);
  ret = ilps22qs_write_reg(ILPS22QS_CTRL_REG1, reg, 2);

  return ret;
}

/**
 * @brief  Sensor conversion parameters selection.[get]
 *
 * @param  val   get the sensor conversion parameters.(ptr)
 * @retval       interface status (MANDATORY: return 0 -> no Error)
 *
 */
int32_t ilps22qs_mode_get(ilps22qs_md_t *val, uint8_t w)
{
  ilps22qs_ctrl_reg1_t ctrl_reg1;
  ilps22qs_ctrl_reg2_t ctrl_reg2;
  uint8_t reg[2];
  int32_t ret;

  ret = ilps22qs_read_reg(ILPS22QS_CTRL_REG1, reg, 2, w);

  if (ret == 0)
  {
    bytecpy((uint8_t *)&ctrl_reg1, &reg[0]);
    bytecpy((uint8_t *)&ctrl_reg2, &reg[1]);

    switch (ctrl_reg2.fs_mode)
    {
    case ILPS22QS_1260hPa:
      val->fs = ILPS22QS_1260hPa;
      break;
    case ILPS22QS_4060hPa:
      val->fs = ILPS22QS_4060hPa;
      break;
    default:
      val->fs = ILPS22QS_1260hPa;
      break;
    }

    switch (ctrl_reg1.odr)
    {
    case ILPS22QS_ONE_SHOT:
      val->odr = ILPS22QS_ONE_SHOT;
      break;
    case ILPS22QS_1Hz:
      val->odr = ILPS22QS_1Hz;
      break;
    case ILPS22QS_4Hz:
      val->odr = ILPS22QS_4Hz;
      break;
    case ILPS22QS_10Hz:
      val->odr = ILPS22QS_10Hz;
      break;
    case ILPS22QS_25Hz:
      val->odr = ILPS22QS_25Hz;
      break;
    case ILPS22QS_50Hz:
      val->odr = ILPS22QS_50Hz;
      break;
    case ILPS22QS_75Hz:
      val->odr = ILPS22QS_75Hz;
      break;
    case ILPS22QS_100Hz:
      val->odr = ILPS22QS_100Hz;
      break;
    case ILPS22QS_200Hz:
      val->odr = ILPS22QS_200Hz;
      break;
    default:
      val->odr = ILPS22QS_ONE_SHOT;
      break;
    }

    switch (ctrl_reg1.avg)
    {
    case ILPS22QS_4_AVG:
      val->avg = ILPS22QS_4_AVG;
      break;
    case ILPS22QS_8_AVG:
      val->avg = ILPS22QS_8_AVG;
      break;
    case ILPS22QS_16_AVG:
      val->avg = ILPS22QS_16_AVG;
      break;
    case ILPS22QS_32_AVG:
      val->avg = ILPS22QS_32_AVG;
      break;
    case ILPS22QS_64_AVG:
      val->avg = ILPS22QS_64_AVG;
      break;
    case ILPS22QS_128_AVG:
      val->avg = ILPS22QS_128_AVG;
      break;
    case ILPS22QS_256_AVG:
      val->avg = ILPS22QS_256_AVG;
      break;
    case ILPS22QS_512_AVG:
      val->avg = ILPS22QS_512_AVG;
      break;
    default:
      val->avg = ILPS22QS_4_AVG;
      break;
    }

    switch ((ctrl_reg2.lfpf_cfg << 2) | ctrl_reg2.en_lpfp)
    {
    case ILPS22QS_LPF_DISABLE:
      val->lpf = ILPS22QS_LPF_DISABLE;
      break;
    case ILPS22QS_LPF_ODR_DIV_4:
      val->lpf = ILPS22QS_LPF_ODR_DIV_4;
      break;
    case ILPS22QS_LPF_ODR_DIV_9:
      val->lpf = ILPS22QS_LPF_ODR_DIV_9;
      break;
    default:
      val->lpf = ILPS22QS_LPF_DISABLE;
      break;
    }
  }
  return ret;
}



/**
 * @brief  Device "Who am I".[get]
 *
 * @param  val   ID values.(ptr)
 * @retval       interface status (MANDATORY: return 0 -> no Error)
 *
 */
int32_t ilps22qs_id_get(ilps22qs_id_t *val, uint8_t w)
{
  uint8_t reg;
  int32_t ret;

  ret = ilps22qs_read_reg(ILPS22QS_WHO_AM_I, &reg, 1, w);
  val->whoami = reg;

  return ret;
}



int32_t ilps22qs_ah_qvar_disable()
{
  uint32_t val = 0;
  int32_t ret;

  ret = ilps22qs_write_reg(ILPS22QS_ANALOGIC_HUB_DISABLE, (uint8_t *)&val, 1);

  return ret;
}



int32_t ilps22qs_reset()
{
  // enable
  *(registers_0_addr + REG_ROC_LEAK_MUX) = 1;

  ilps22qs_ctrl_reg2_t ctrl_reg2;
  ilps22qs_ctrl_reg3_t ctrl_reg3;
  uint8_t reg[2];
  int32_t ret;

  memset(&ctrl_reg2, 0, sizeof(ilps22qs_ctrl_reg2_t));
  ctrl_reg2.swreset = PROPERTY_ENABLE;

  ret = ilps22qs_write_reg(ILPS22QS_CTRL_REG2,
                           (uint8_t *)&ctrl_reg2, 1);
  return 0;
}

/**
 * @brief  Read generic device register
 *
 * @param  reg   register to read
 * @param  data  pointer to buffer that store the data read(ptr)
 * @param  len   number of consecutive register to read
 * @retval       interface status (MANDATORY: return 0 -> no Error)
 *
 */
int32_t ilps22qs_read_reg(uint8_t reg, uint8_t *bufp,
                          uint16_t len, uint8_t w)
{
  volatile uint32_t *leak_sdi = registers_0_addr + REG_ROC_LEAK_SDA;
  // enable
  *(registers_0_addr + REG_ROC_LEAK_MUX) = 1;

  *(registers_0_addr + REG_ROC_LEAK_SDIR) = 1;

  // Drop CS, ALL chips
  *(registers_0_addr + REG_ROC_LEAK_SCL) = 1;
  *(registers_0_addr + REG_ROC_LEAK_CS) = 0;

  // this is a read so set the R bit
  reg |= 0x80;
  //  printf("Bit banging %x",reg);
  // bitbang register to SDIO
  for (uint8_t m = (uint8_t)0x80; m != 0; m >>= 1)
  {
    *(registers_0_addr + REG_ROC_LEAK_SCL) = 0;
    if (reg & m)
      *(registers_0_addr + REG_ROC_LEAK_SDA) = 1;
    else
      *(registers_0_addr + REG_ROC_LEAK_SDA) = 0;

    //    printf("%x\n",reg & m);
    //     gpio_put(sdio0Pin, reg & m);
    //    gpio_put(sdio0Pin, 0);
    /* gpio_put(sdio1Pin, reg & m);     */
    delayUs(SPIDELAY);
    *(registers_0_addr + REG_ROC_LEAK_SCL) = 1;
    delayUs(SPIDELAY);
  }

  // bitbang rest to SDIO
  //    gpio_set_dir(sdio0Pin, GPIO_IN);
  //   gpio_set_dir(sdio1Pin, GPIO_IN);

  *(registers_0_addr + REG_ROC_LEAK_SDIR) = 0;

  //  sleep_us(10);
  for (uint8_t i = 0; i < len; i++)
  {
    bufp[i] = 0;
    for (uint8_t j = 8; j--;)
    {
      *(registers_0_addr + REG_ROC_LEAK_SCL) = 0;
      delayUs(SPIDELAY);
      //            bufp[i] |= (gpio_get(sdio0Pin)<<j);
      bufp[i] |= ((*(leak_sdi)) << j);
      *(registers_0_addr + REG_ROC_LEAK_SCL) = 1;
      delayUs(SPIDELAY);
    }
  }

  *(registers_0_addr + REG_ROC_LEAK_CS) = 1;

  // disable
  *(registers_0_addr + REG_ROC_LEAK_MUX) = 1;

  return 0;
}


/**
 * @brief  Write generic device register
 *
 * @param  reg   register to write
 * @param  data  pointer to data to write in register reg(ptr)
 * @param  len   number of consecutive register to write
 * @retval       interface status (MANDATORY: return 0 -> no Error)
 *
 */
int32_t ilps22qs_write_reg(uint8_t reg, uint8_t *bufp,
                           uint16_t len)
{
  // enable
  *(registers_0_addr + REG_ROC_LEAK_MUX) = 1;

  *(registers_0_addr + REG_ROC_LEAK_SDIR) = 1;

  // Drop CS, ALL chips
  *(registers_0_addr + REG_ROC_LEAK_SCL) = 1;
  *(registers_0_addr + REG_ROC_LEAK_CS) = 0;

  // bitbang register to SDIO
  for (uint8_t m = (uint8_t)0x80; m != 0; m >>= 1)
  {
    *(registers_0_addr + REG_ROC_LEAK_SCL) = 0;
    if (reg & m)
      *(registers_0_addr + REG_ROC_LEAK_SDA) = 1;
    else
      *(registers_0_addr + REG_ROC_LEAK_SDA) = 0;
    //    gpio_put(sdio0Pin, reg & m);
    /* gpio_put(sdio1Pin, reg & m);     */

    delayUs(SPIDELAY);
    *(registers_0_addr + REG_ROC_LEAK_SCL) = 1;
    delayUs(SPIDELAY);
  }

  // bitbang rest to SDIO
  for (uint8_t i = 0; i < len; i++)
  {
    for (uint8_t m = (uint8_t)0x80; m != 0; m >>= 1)
    {
      *(registers_0_addr + REG_ROC_LEAK_SCL) = 0;
      if (bufp[i] & m)
        *(registers_0_addr + REG_ROC_LEAK_SDA) = 1;
      else
        *(registers_0_addr + REG_ROC_LEAK_SDA) = 0;

      delayUs(SPIDELAY);
      *(registers_0_addr + REG_ROC_LEAK_SCL) = 1;
      delayUs(SPIDELAY);
    }
  }

  *(registers_0_addr + REG_ROC_LEAK_CS) = 1;

  // disable
  *(registers_0_addr + REG_ROC_LEAK_MUX) = 1;

  return 0;
}


/**
 * @brief  Configures the bus operating mode.[set]
 *
 * @param  val   configures the bus operating mode.(ptr)
 * @retval       interface status (MANDATORY: return 0 -> no Error)
 *
 */
int32_t ilps22qs_bus_mode_set(ilps22qs_bus_mode_t *val)
{
  //  ilps22qs_i3c_if_ctrl_add_t i3c_if_ctrl_add;
  //  ilps22qs_if_ctrl_t if_ctrl;
  int32_t ret;

  //  memset(&if_ctrl, 0, sizeof(ilps22qs_if_ctrl_t));

  uint8_t if_ctrl = 0x60;
  //  if_ctrl.i2c_i3c_dis = ((uint8_t)val->interface & 0x02U) >> 1;
  //  if_ctrl.en_spi_read = ((uint8_t)val->interface & 0x01U);
  //  printf("TEST=%x\n",if_ctrl);
  //  ret = ilps22qs_write_reg(ctx, ILPS22QS_IF_CTRL, (uint8_t *)&if_ctrl, 1);
  ret = ilps22qs_write_reg(ILPS22QS_IF_CTRL, &if_ctrl, 1);

  //  memset(&i3c_if_ctrl_add, 0, sizeof(ilps22qs_i3c_if_ctrl_add_t));

  //  i3c_if_ctrl_add.asf_on = (uint8_t)val->filter & 0x01U;
  //  ret = ilps22qs_write_reg(ctx, ILPS22QS_I3C_IF_CTRL_ADD,
  //                             (uint8_t *)&i3c_if_ctrl_add, 1);
  delayUs(20);
  uint8_t if_ctrl_add = 0xA0;
  ret = ilps22qs_write_reg(ILPS22QS_I3C_IF_CTRL_ADD, &if_ctrl_add, 1);
  return ret;
}


/**
 * @brief  Get the status of the device.[get]
 *
 * @param  val   the status of the device.(ptr)
 * @retval       interface status (MANDATORY: return 0 -> no Error)
 *
 */
int32_t ilps22qs_status_get(ilps22qs_stat_t *val, uint8_t w)
{
  ilps22qs_interrupt_cfg_t interrupt_cfg;
  ilps22qs_int_source_t int_source;
  ilps22qs_ctrl_reg2_t ctrl_reg2;
  ilps22qs_status_t status;
  int32_t ret;

  ret = ilps22qs_read_reg(ILPS22QS_CTRL_REG2,
                          (uint8_t *)&ctrl_reg2, 1, w);
  if (ret == 0)
  {
    ret = ilps22qs_read_reg(ILPS22QS_INT_SOURCE, (uint8_t *)&int_source, 1, w);
  }
  if (ret == 0)
  {
    ret = ilps22qs_read_reg(ILPS22QS_STATUS, (uint8_t *)&status, 1, w);
  }
  if (ret == 0)
  {
    ret = ilps22qs_read_reg(ILPS22QS_INTERRUPT_CFG,
                            (uint8_t *)&interrupt_cfg, 1, w);
  }
  val->sw_reset = ctrl_reg2.swreset;
  val->boot = int_source.boot_on;
  val->drdy_pres = status.p_da;
  val->drdy_temp = status.t_da;
  val->ovr_pres = status.p_or;
  val->ovr_temp = status.t_or;
  val->end_meas = ~ctrl_reg2.oneshot;
  val->ref_done = ~interrupt_cfg.autozero;

  return ret;
}



/*
 * @brief  init function for ILP device
 *
 *
 */
int32_t ilp22qs_init()
{

  ilps22qs_bus_mode_t bus_mode;
  ilps22qs_stat_t status;
  ilps22qs_id_t id;

  /* Initialize platform specific hardware */


  *(registers_0_addr + REG_ROC_LEAK_MUX) = 0;

  *(registers_0_addr + REG_ROC_LEAK_SDIR) = 1;
  *(registers_0_addr + REG_ROC_LEAK_SDA) = 0;
  *(registers_0_addr + REG_ROC_LEAK_SCL) = 0;
  *(registers_0_addr + REG_ROC_LEAK_CS) = 1;

  bus_mode.interface = ILPS22QS_SPI_3W;
  ilps22qs_bus_mode_set(&bus_mode);

  delayUs(BOOT_TIME*1000);

  //  ilps22qs_reset(dev_ctx);
  //  sleep_us(10);

  //  while(1){
  bus_mode.interface = ILPS22QS_SPI_3W;
  ilps22qs_bus_mode_set(&bus_mode);
  //        sleep_ms(1000);
  //  }
  //  printf("Interface = %x\n",bus_mode.interface);

  /* ilps22qs_id_get(dev_ctx, &id, dataPin[1]);    */
  /* if (id.whoami != ILPS22QS_ID) */
  /*   while(1); */

  /* Restore default configuration */
  /* ilps22qs_init_set(dev_ctx, ILPS22QS_RESET); */
  /* do { */
  /*   ilps22qs_status_get(dev_ctx, &status); */
  /* } while (status.sw_reset); */

  ilps22qs_reset();

  ilps22qs_bus_mode_set(&bus_mode);

  /* Disable AH/QVAR to save power consumption */
  ilps22qs_ah_qvar_disable();

  /* Set bdu and if_inc recommended for driver usage */
  //  ilps22qs_init_set(dev_ctx, ILPS22QS_DRV_RDY);

  /* Select bus interface */
  bus_mode.filter = ILPS22QS_AUTO;
  bus_mode.interface = ILPS22QS_SPI_3W;
  ilps22qs_bus_mode_set(&bus_mode);

  return 0;
}
