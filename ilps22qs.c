#include "ilps22qs.h"
#include "utils.h"

#include "hw_platform.h"

#define SPIDELAY 100
#define BOOT_TIME 10

/**
 * @defgroup    Sensitivity
 * @brief       These functions convert raw-data into engineering units.
 * @{
 *
 */

float ilps22qs_from_fs1260_to_hPa(int32_t lsb) {
    return ((float) lsb / 1048576.0f); /* 4096.0f * 256 */
}

float ilps22qs_from_fs4000_to_hPa(int32_t lsb) {
    return ((float) lsb / 524288.0f); /* 2048.0f * 256 */
}

float ilps22qs_from_lsb_to_celsius(int16_t lsb) {
    return ((float) lsb / 100.0f);
}

/**
 * @defgroup  Private_functions
 * @brief     Section collect all the utility functions needed by APIs.
 * @{
 *
 */

static void bytecpy(uint8_t *target, uint8_t *source) {
    if ((target != NULL) && (source != NULL)) {
        *target = *source;
    }
}

int32_t ilps22qs_data_get(ilps22qs_md_t *md, ilps22qs_data_t *data) {
    uint8_t buff[5];
    int32_t ret;

    ret = ilps22qs_read_reg(ILPS22QS_PRESS_OUT_XL, buff, 5);

    /* pressure conversion */
    data->pressure.raw = (int32_t) buff[2];
    data->pressure.raw = (data->pressure.raw * 256) + (int32_t) buff[1];
    data->pressure.raw = (data->pressure.raw * 256) + (int32_t) buff[0];
    data->pressure.raw = data->pressure.raw * 256;


    /* temperature conversion */
    data->heat.raw = (int16_t) buff[4];
    data->heat.raw = (data->heat.raw * 256) + (int16_t) buff[3];


    return ret;
}

/**
 * @brief  Get the status of all the interrupt sources.[get]
 *
 * @param  val   the status of all the interrupt sources.(ptr)
 * @retval       interface status (MANDATORY: return 0 -> no Error)
 *
 */
int32_t ilps22qs_all_sources_get(ilps22qs_all_sources_t *val) {
    ilps22qs_fifo_status2_t fifo_status2;
    ilps22qs_int_source_t int_source;
    ilps22qs_status_t status;
    int32_t ret;

    ret = ilps22qs_read_reg(ILPS22QS_STATUS, (uint8_t*) &status, 1);
    if (ret == 0) {
        ret = ilps22qs_read_reg(ILPS22QS_INT_SOURCE, (uint8_t*) &int_source, 1);
    }
    if (ret == 0) {
        ret = ilps22qs_read_reg(ILPS22QS_FIFO_STATUS2, (uint8_t*) &fifo_status2,
                1);
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
int32_t ilps22qs_mode_set(ilps22qs_md_t *val) {
    ilps22qs_ctrl_reg1_t ctrl_reg1;
    ilps22qs_ctrl_reg2_t ctrl_reg2;
    uint8_t reg[2];
    int32_t ret;

    memset(&ctrl_reg1, 0, sizeof(ilps22qs_ctrl_reg1_t));
    memset(&ctrl_reg2, 0, sizeof(ilps22qs_ctrl_reg2_t));

    ctrl_reg1.odr = (uint8_t) val->odr;
    ctrl_reg1.avg = (uint8_t) val->avg;
    ctrl_reg2.en_lpfp = (uint8_t) val->lpf & 0x01U;
    ctrl_reg2.lfpf_cfg = ((uint8_t) val->lpf & 0x02U) >> 2;
    ctrl_reg2.fs_mode = (uint8_t) val->fs;

    bytecpy(&reg[0], (uint8_t*) &ctrl_reg1);
    bytecpy(&reg[1], (uint8_t*) &ctrl_reg2);
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
int32_t ilps22qs_mode_get(ilps22qs_md_t *val) {
    ilps22qs_ctrl_reg1_t ctrl_reg1;
    ilps22qs_ctrl_reg2_t ctrl_reg2;
    uint8_t reg[2];
    int32_t ret;

    ret = ilps22qs_read_reg(ILPS22QS_CTRL_REG1, reg, 2);

    if (ret == 0) {
        bytecpy((uint8_t*) &ctrl_reg1, &reg[0]);
        bytecpy((uint8_t*) &ctrl_reg2, &reg[1]);

        switch (ctrl_reg2.fs_mode) {
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

        switch (ctrl_reg1.odr) {
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

        switch (ctrl_reg1.avg) {
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

        switch ((ctrl_reg2.lfpf_cfg << 2) | ctrl_reg2.en_lpfp) {
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
int32_t ilps22qs_id_get() {
    uint8_t reg;
    int32_t ret;

    ret = ilps22qs_read_reg(ILPS22QS_WHO_AM_I, &reg, 1);

    return reg;
}

int32_t ilps22qs_ah_qvar_disable() {
    uint32_t val = 0;
    int32_t ret;

    ret = ilps22qs_write_reg(ILPS22QS_ANALOGIC_HUB_DISABLE, (uint8_t*) &val, 1);

    return ret;
}

int32_t ilps22qs_reset() {
    // enable
    *(registers_0_addr + REG_ROC_LEAK_MUX) = 1;

    ilps22qs_ctrl_reg2_t ctrl_reg2;
    ilps22qs_ctrl_reg3_t ctrl_reg3;
    uint8_t reg[2];
    int32_t ret;

    memset(&ctrl_reg2, 0, sizeof(ilps22qs_ctrl_reg2_t));
    ctrl_reg2.swreset = PROPERTY_ENABLE;

    ret = ilps22qs_write_reg(ILPS22QS_CTRL_REG2, (uint8_t*) &ctrl_reg2, 1);
    return 0;
}

/**
 * @brief  Configures the bus operating mode.[set]
 *
 * @param  val   configures the bus operating mode.(ptr)
 * @retval       interface status (MANDATORY: return 0 -> no Error)
 *
 */
int32_t ilps22qs_bus_mode_set(ilps22qs_bus_mode_t *val) {
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
int32_t ilps22qs_status_get(ilps22qs_stat_t *val) {
    ilps22qs_interrupt_cfg_t interrupt_cfg;
    ilps22qs_int_source_t int_source;
    ilps22qs_ctrl_reg2_t ctrl_reg2;
    ilps22qs_status_t status;
    int32_t ret;

    ret = ilps22qs_read_reg(ILPS22QS_CTRL_REG2, (uint8_t*) &ctrl_reg2, 1);
    if (ret == 0) {
        ret = ilps22qs_read_reg(ILPS22QS_INT_SOURCE, (uint8_t*) &int_source, 1);
    }
    if (ret == 0) {
        ret = ilps22qs_read_reg(ILPS22QS_STATUS, (uint8_t*) &status, 1);
    }
    if (ret == 0) {
        ret = ilps22qs_read_reg(ILPS22QS_INTERRUPT_CFG,
                (uint8_t*) &interrupt_cfg, 1);
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
int32_t ilp22qs_init() {

    ilps22qs_bus_mode_t bus_mode;
    ilps22qs_stat_t status;
    ilps22qs_id_t id;

    /* Initialize platform specific hardware */

    // enable
    *(registers_0_addr + REG_ROC_LEAK_MUX) = 1;

    ilps22qs_md_t md;
    md.odr = ILPS22QS_4Hz;
    md.avg = ILPS22QS_16_AVG;
    md.lpf = ILPS22QS_LPF_ODR_DIV_4;
    md.fs = ILPS22QS_4060hPa;

    ilps22qs_mode_set(&md);

    /* Disable AH/QVAR to save power consumption */
    ilps22qs_ah_qvar_disable();

    // disable
    *(registers_0_addr + REG_ROC_LEAK_MUX) = 0;


    return 0;
}

int32_t ilps22qs_read_reg(uint8_t reg, uint8_t *bufp, uint16_t len) {

    // enable
    *(registers_0_addr + REG_ROC_LEAK_MUX) = 1;

    ilps22qs_i2c_start();
    ilps22qs_i2c_sendbyte(0xB8); // chip address + W
    int32_t ack = ilps22qs_i2c_ack();
    if (ack == 1) {
        ilps22qs_i2c_stop();
        return ack;
    }

    ilps22qs_i2c_sendbyte(reg);
    ack = ilps22qs_i2c_ack();
    if (ack == 1) {
        ilps22qs_i2c_stop();
        return ack;
    }
    ilps22qs_i2c_stop();

    ilps22qs_i2c_start();
    ilps22qs_i2c_sendbyte(0xB9); // chip address + R
    ack = ilps22qs_i2c_ack();
    if (ack == 1) {
        ilps22qs_i2c_stop();
        return ack;
    }

    //    bufp[0] = 0;
    //    ilps22qs_i2c_getbyte(ctx,&bufp[0]);

    for (uint8_t i = 0; i < len; i++) {
        ilps22qs_i2c_getbyte(&bufp[i]);
        if (i < len - 1)
            ilps22qs_i2c_mack();
    }

    ilps22qs_i2c_stop();

    // disable
    *(registers_0_addr + REG_ROC_LEAK_MUX) = 0;
    return ack;
}

int32_t ilps22qs_write_reg(uint8_t reg, uint8_t *bufp, uint16_t len) {

    // enable
    *(registers_0_addr + REG_ROC_LEAK_MUX) = 1;

    ilps22qs_i2c_start();
    ilps22qs_i2c_sendbyte(0xB8); // chip address + W
    int32_t ack = ilps22qs_i2c_ack();
    if (ack == 1) {
        ilps22qs_i2c_stop();
        return ack;
    }
    ilps22qs_i2c_sendbyte(reg);
    ack = ilps22qs_i2c_ack();
    if (ack == 1) {
        ilps22qs_i2c_stop();
        return ack;
    }
//    ilps22qs_i2c_stop();

//    ilps22qs_i2c_start();
    for (uint8_t i = 0; i < len; i++) {
        ilps22qs_i2c_sendbyte(bufp[i]);
        ack = ilps22qs_i2c_ack();
        if (ack == 1) {
            ilps22qs_i2c_stop();
            return ack;
        }
    }
    ilps22qs_i2c_stop();

    // disable
    *(registers_0_addr + REG_ROC_LEAK_MUX) = 0;
    return ack;

}

int32_t ilps22qs_i2c_start() {
    *(registers_0_addr + REG_ROC_LEAK_SDA) = 1;
    *(registers_0_addr + REG_ROC_LEAK_SDIR) = 1;
    delayUs(SPIDELAY);
    *(registers_0_addr + REG_ROC_LEAK_SCL) = 1;
    delayUs(SPIDELAY);
    *(registers_0_addr + REG_ROC_LEAK_SDA) = 0;
    delayUs(SPIDELAY);
    *(registers_0_addr + REG_ROC_LEAK_SCL) = 0;
    delayUs(SPIDELAY);
    return 0;
}

int32_t ilps22qs_i2c_stop() {
    *(registers_0_addr + REG_ROC_LEAK_SDA) = 0;
    *(registers_0_addr + REG_ROC_LEAK_SDIR) = 1;
    delayUs(SPIDELAY);
    *(registers_0_addr + REG_ROC_LEAK_SCL) = 1;
    delayUs(SPIDELAY);
    *(registers_0_addr + REG_ROC_LEAK_SDA) = 1;
    delayUs(SPIDELAY);
    return 0;
}

int32_t ilps22qs_i2c_ack() {
    volatile uint32_t *leak_sdi = registers_0_addr + REG_ROC_LEAK_SDA;

    //sack
    *(registers_0_addr + REG_ROC_LEAK_SDIR) = 0;
    delayUs(SPIDELAY);
    *(registers_0_addr + REG_ROC_LEAK_SCL) = 1;
    delayUs(SPIDELAY);
    int32_t ack = 0;
    if (*(leak_sdi)) {
        ack = 1;
    }
    *(registers_0_addr + REG_ROC_LEAK_SCL) = 0;
    delayUs(SPIDELAY);
    return ack;
}

int32_t ilps22qs_i2c_mack() {

//    volatile uint32_t *leak_sdi = registers_0_addr + REG_ROC_LEAK_SDA;

    //sack
    *(registers_0_addr + REG_ROC_LEAK_SDIR) = 1;
    *(registers_0_addr + REG_ROC_LEAK_SDA) = 0;
    delayUs(SPIDELAY);
    *(registers_0_addr + REG_ROC_LEAK_SCL) = 1;
    delayUs(SPIDELAY);

    *(registers_0_addr + REG_ROC_LEAK_SCL) = 0;
    delayUs(SPIDELAY);

    return 0;

}

int32_t ilps22qs_i2c_sendbyte(uint8_t data) {
    *(registers_0_addr + REG_ROC_LEAK_SCL) = 0;
    delayUs(SPIDELAY);
    *(registers_0_addr + REG_ROC_LEAK_SDIR) = 1;
    delayUs(SPIDELAY);
    for (uint8_t m = (uint8_t) 0x80; m != 0; m >>= 1) {
        *(registers_0_addr + REG_ROC_LEAK_SCL) = 0;
        delayUs(SPIDELAY);
        if (data & m)
            *(registers_0_addr + REG_ROC_LEAK_SDA) = 1;
        else
            *(registers_0_addr + REG_ROC_LEAK_SDA) = 0;
        delayUs(SPIDELAY);
        *(registers_0_addr + REG_ROC_LEAK_SCL) = 1;
        delayUs(SPIDELAY);
    }
    *(registers_0_addr + REG_ROC_LEAK_SCL) = 0;
    delayUs(SPIDELAY);
    return 0;
}

int32_t ilps22qs_i2c_getbyte(uint8_t *data) {
    volatile uint32_t *leak_sdi = registers_0_addr + REG_ROC_LEAK_SDA;

    *(registers_0_addr + REG_ROC_LEAK_SCL) = 0;
    delayUs(SPIDELAY);
    *(registers_0_addr + REG_ROC_LEAK_SDIR) = 0;
    delayUs(SPIDELAY);

    data[0] = 0;
    for (uint8_t j = 8; j--;) {
        *(registers_0_addr + REG_ROC_LEAK_SCL) = 0;
        delayUs(SPIDELAY);
        //            bufp[i] |= (gpio_get(sdio0Pin)<<j);
        data[0] |= ((*(leak_sdi)) << j);
        *(registers_0_addr + REG_ROC_LEAK_SCL) = 1;
        delayUs(SPIDELAY);
    }

    *(registers_0_addr + REG_ROC_LEAK_SCL) = 0;
    delayUs(SPIDELAY);
    return 0;
}

