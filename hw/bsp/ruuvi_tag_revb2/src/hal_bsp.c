/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#define MMGR 0

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <assert.h>
#include "os/mynewt.h"
#include "nrfx.h"
#include "flash_map/flash_map.h"
#include "hal/hal_bsp.h"
#include "hal/hal_system.h"
#include "hal/hal_flash.h"
#include "hal/hal_spi.h"
#include "hal/hal_watchdog.h"
#include "hal/hal_i2c.h"
#include "hal/hal_gpio.h"
#include "mcu/nrf52_hal.h"
#if MMGR
#include "motion_mgr/motion_mgr.h"
#endif
#if MYNEWT_VAL(UART_0) || MYNEWT_VAL(UART_1)
#include "uart/uart.h"
#endif
#if MYNEWT_VAL(UART_0)
#include "uart_hal/uart_hal.h"
#endif
#if MYNEWT_VAL(UART_1)
#include "uart_bitbang/uart_bitbang.h"
#endif
#include "bsp.h"
#if MYNEWT_VAL(ADC_0)
#include <adc_nrf52/adc_nrf52.h>
#include <nrfx_saadc.h>
#endif
#if MYNEWT_VAL(PWM_0) || MYNEWT_VAL(PWM_1) || MYNEWT_VAL(PWM_2)
#include <pwm_nrf52/pwm_nrf52.h>
#endif
#if MYNEWT_VAL(SOFT_PWM)
#include <soft_pwm/soft_pwm.h>
#endif

#if MYNEWT_VAL(BME280_ONB)
#include <bme280/bme280.h>
static struct bme280 bme280;
#endif

#include <lis2dh12/lis2dh12.h>
#if MYNEWT_VAL(LIS2DH12_ONB)
static struct lis2dh12 lis2dh12;
#endif

#if MYNEWT_VAL(UART_0)
static struct uart_dev os_bsp_uart0;
static const struct nrf52_uart_cfg os_bsp_uart0_cfg = {
    .suc_pin_tx = MYNEWT_VAL(UART_0_PIN_TX),
    .suc_pin_rx = MYNEWT_VAL(UART_0_PIN_RX),
    .suc_pin_rts = MYNEWT_VAL(UART_0_PIN_RTS),
    .suc_pin_cts = MYNEWT_VAL(UART_0_PIN_CTS),
};
#endif

#if MYNEWT_VAL(UART_1)
static struct uart_dev os_bsp_bitbang_uart1;
static const struct uart_bitbang_conf os_bsp_uart1_cfg = {
    .ubc_txpin = MYNEWT_VAL(UART_1_PIN_TX),
    .ubc_rxpin = MYNEWT_VAL(UART_1_PIN_RX),
    .ubc_cputimer_freq = MYNEWT_VAL(OS_CPUTIME_FREQ),
};
#endif

#if MYNEWT_VAL(SPI_0_MASTER)
/*
 * NOTE: Our HAL expects that the SS pin, if used, is treated as a gpio line
 * and is handled outside the SPI routines.
 */
static const struct nrf52_hal_spi_cfg os_bsp_spi0m_cfg = {
    .sck_pin      = MYNEWT_VAL(SPI_0_MASTER_PIN_SCK),
    .mosi_pin     = MYNEWT_VAL(SPI_0_MASTER_PIN_MOSI),
    .miso_pin     = MYNEWT_VAL(SPI_0_MASTER_PIN_MISO),
};

#if MYNEWT_VAL(BME280_ONB)
static const struct sensor_itf spi_0_itf_bme = {
    .si_type = SENSOR_ITF_SPI,
    .si_num = 0,
    .si_cs_pin = 3
};
#endif

#if MYNEWT_VAL(LIS2DH12_ONB)
static const struct sensor_itf spi_0_itf_lis = {
    .si_type = SENSOR_ITF_SPI,
    .si_num = 0,
    .si_cs_pin = 8,
    .si_low_pin = 2,
    .si_high_pin = 6
};
#endif
#endif

#if MYNEWT_VAL(SPI_0_SLAVE)
static const struct nrf52_hal_spi_cfg os_bsp_spi0s_cfg = {
    .sck_pin      = MYNEWT_VAL(SPI_0_SLAVE_PIN_SCK),
    .mosi_pin     = MYNEWT_VAL(SPI_0_SLAVE_PIN_MOSI),
    .miso_pin     = MYNEWT_VAL(SPI_0_SLAVE_PIN_MISO),
    .ss_pin       = MYNEWT_VAL(SPI_0_SLAVE_PIN_SS),
};
#endif

#if MYNEWT_VAL(ADC_0)
static struct adc_dev os_bsp_adc0;
static struct nrf52_adc_dev_cfg os_bsp_adc0_config = {
    .nadc_refmv     = MYNEWT_VAL(ADC_0_REFMV_0),
};
#endif


#if MYNEWT_VAL(PWM_0)
static struct pwm_dev os_bsp_pwm0;
int pwm0_idx;
#endif
#if MYNEWT_VAL(PWM_1)
static struct pwm_dev os_bsp_pwm1;
int pwm1_idx;
#endif
#if MYNEWT_VAL(PWM_2)
static struct pwm_dev os_bsp_pwm2;
int pwm2_idx;
#endif
#if MYNEWT_VAL(SOFT_PWM)
static struct pwm_dev os_bsp_spwm[MYNEWT_VAL(SOFT_PWM_DEVS)];
char* spwm_name[MYNEWT_VAL(SOFT_PWM_DEVS)];
int spwm_idx[MYNEWT_VAL(SOFT_PWM_DEVS)];
#endif

#if MYNEWT_VAL(I2C_0)
struct os_mutex g_bsp_i2c0_itf_lock;

static int
bsp_i2c0_itf_lock_init(void)
{
    return os_mutex_init(&g_bsp_i2c0_itf_lock);
}
static const struct nrf52_hal_i2c_cfg hal_i2c_cfg = {
    .scl_pin = MYNEWT_VAL(I2C_0_PIN_SCL),
    .sda_pin = MYNEWT_VAL(I2C_0_PIN_SDA),
    .i2c_frequency = MYNEWT_VAL(I2C_0_FREQ_KHZ),
};
#endif

/*
 * What memory to include in coredump.
 */
static const struct hal_bsp_mem_dump dump_cfg[] = {
    [0] = {
        .hbmd_start = &_ram_start,
        .hbmd_size = RAM_SIZE
    }
};

const struct hal_flash *
hal_bsp_flash_dev(uint8_t id)
{
    /*
     * Internal flash mapped to id 0.
     */
    if (id != 0) {
        return NULL;
    }
    return &nrf52k_flash_dev;
}

const struct hal_bsp_mem_dump *
hal_bsp_core_dump(int *area_cnt)
{
    *area_cnt = sizeof(dump_cfg) / sizeof(dump_cfg[0]);
    return dump_cfg;
}

int
hal_bsp_power_state(int state)
{
    return (0);
}

/**
 * Returns the configured priority for the given interrupt. If no priority
 * configured, return the priority passed in
 *
 * @param irq_num
 * @param pri
 *
 * @return uint32_t
 */
uint32_t
hal_bsp_get_nvic_priority(int irq_num, uint32_t pri)
{
    uint32_t cfg_pri;

    switch (irq_num) {
    /* Radio gets highest priority */
    case RADIO_IRQn:
        cfg_pri = 0;
        break;
    default:
        cfg_pri = pri;
    }
    return cfg_pri;
}

#if MMGR
motion_mgr_sensor_t bsp_motion_mgr_sensor;
#endif

/**
 * BME280 Sensor default configuration
 *
 * @return 0 on success, non-zero on failure
 */
int
config_bme280_sensor(void)
{
#if MYNEWT_VAL(BME280_ONB)
    int rc;
    struct os_dev *dev;
    struct bme280_cfg bmecfg;

    dev = (struct os_dev *) os_dev_open("bme280_0", OS_TIMEOUT_NEVER, NULL);
    assert(dev != NULL);

    memset(&bmecfg, 0, sizeof(bmecfg));

    bmecfg.bc_mode = BME280_MODE_FORCED;
    bmecfg.bc_iir = BME280_FILTER_OFF;
    bmecfg.bc_sby_dur = BME280_STANDBY_MS_0_5;
    bmecfg.bc_boc[0].boc_type = SENSOR_TYPE_RELATIVE_HUMIDITY;
    bmecfg.bc_boc[1].boc_type = SENSOR_TYPE_PRESSURE;
    bmecfg.bc_boc[2].boc_type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
    bmecfg.bc_boc[0].boc_oversample = BME280_SAMPLING_X1;
    bmecfg.bc_boc[1].boc_oversample = BME280_SAMPLING_X1;
    bmecfg.bc_boc[2].boc_oversample = BME280_SAMPLING_X1;
    bmecfg.bc_s_mask = SENSOR_TYPE_AMBIENT_TEMPERATURE|
                       SENSOR_TYPE_PRESSURE|
                       SENSOR_TYPE_RELATIVE_HUMIDITY;

    rc = bme280_config((struct bme280 *)dev, &bmecfg);
    SYSINIT_PANIC_ASSERT(rc == 0);

    os_dev_close(dev);
#endif
    return 0;
}

/**
 * LIS2Dh12 Sensor default configuration
 *
 * @return 0 on success, non-zero on failure
 */
int
config_lis2dh12_sensor(void)
{
#if MYNEWT_VAL(LIS2DH12_ONB)
    int rc;
    struct os_dev *dev;
    struct lis2dh12_cfg cfg;

    dev = (struct os_dev *) os_dev_open("lis2dh12_0", OS_TIMEOUT_NEVER, NULL);
    assert(dev != NULL);

    memset(&cfg, 0, sizeof(cfg));

    cfg.lc_s_mask = SENSOR_TYPE_ACCELEROMETER;
    cfg.lc_rate = LIS2DH12_DATA_RATE_HN_1344HZ_L_5376HZ;
    cfg.lc_fs = LIS2DH12_FS_2G;

    rc = lis2dh12_config((struct lis2dh12 *)dev, &cfg);
    SYSINIT_PANIC_ASSERT(rc == 0);

    os_dev_close(dev);
#endif
    return 0;
}

#if MMGR
struct lis2dh12 bsp_motion_sensor;
/*
const struct lis2dh12_notif_cfg lis2dh12_ncfg[] = {
    {
      .event     = SENSOR_EVENT_TYPE_SINGLE_TAP,
      .int_num   = 0,
      .notif_src = LIS2DH12_CLICK_SRC_SCLICK,
      .int_cfg   = LIS2DH12_CTRL_REG3_I1_CLICK
    },
    {
      .event     = SENSOR_EVENT_TYPE_DOUBLE_TAP,
      .int_num   = 0,
      .notif_src = LIS2DH12_CLICK_SRC_DCLICK,
      .int_cfg   = LIS2DH12_CTRL_REG3_I1_CLICK
    },
    {
      .event     = SENSOR_EVENT_TYPE_SLEEP,
      .int_num   = 1,
      .notif_src = 0,
      .int_cfg   = LIS2DH12_CTRL_REG6_I2_ACT
    },
    {
      .event     = SENSOR_EVENT_TYPE_FREE_FALL,
      .int_num   = 0,
      .notif_src = LIS2DH12_INT1_IA,
      .int_cfg   = LIS2DH12_CTRL_REG3_I1_IA2
    },
    {
      .event     = SENSOR_EVENT_TYPE_WAKEUP,
      .int_num    = 1,
      .notif_src  = 0,
      .int_cfg    = LIS2DH12_CTRL_REG6_I2_ACT
    },
    {
      .event     = SENSOR_EVENT_TYPE_SLEEP_CHANGE,
      .int_num   = 1,
      .notif_src = 0,
      .int_cfg   = LIS2DH12_CTRL_REG6_I2_ACT
    },
    {
      .event     = SENSOR_EVENT_TYPE_ORIENT_CHANGE,
      .int_num   = 0,
      .notif_src = LIS2DH12_INT1_IA,
      .int_cfg   = LIS2DH12_CTRL_REG3_I1_IA1
    },
    {
      .event     = SENSOR_EVENT_TYPE_ORIENT_X_CHANGE,
      .int_num   = 0,
      .notif_src = LIS2DH12_INT1_XH | LIS2DH12_INT1_XL,
      .int_cfg   = LIS2DH12_CTRL_REG3_I1_IA1
    },
    {
      .event     = SENSOR_EVENT_TYPE_ORIENT_Y_CHANGE,
      .int_num   = 0,
      .notif_src = LIS2DH12_INT1_YH | LIS2DH12_INT1_YL,
      .int_cfg   = LIS2DH12_CTRL_REG3_I1_IA1
    },
    {
      .event     = SENSOR_EVENT_TYPE_ORIENT_Z_CHANGE,
      .int_num   = 0,
      .notif_src = LIS2DH12_INT1_ZH | LIS2DH12_INT1_ZL,
      .int_cfg   = LIS2DH12_CTRL_REG3_I1_IA1
    }
};
*/
const struct lis2dh12_cfg bsp_motion_init_cfg =
{
    .lc_rate                      = MYNEWT_VAL(LIS2DH12_CFG_RATE),
    .lc_fs                        = MYNEWT_VAL(LIS2DH12_CFG_FS),

    .reference                    = MYNEWT_VAL(LIS2DH12_CFG_REFERENCE),

    /* Tap config */
    .tap                          =
    {
        .en_xs                    = MYNEWT_VAL(LIS2DH12_CFG_TAP_EN_XS),
        .en_ys                    = MYNEWT_VAL(LIS2DH12_CFG_TAP_EN_YS),
        .en_zs                    = MYNEWT_VAL(LIS2DH12_CFG_TAP_EN_ZS),
        .en_xd                    = MYNEWT_VAL(LIS2DH12_CFG_TAP_EN_XD),
        .en_yd                    = MYNEWT_VAL(LIS2DH12_CFG_TAP_EN_YD),
        .en_zd                    = MYNEWT_VAL(LIS2DH12_CFG_TAP_EN_ZD),
        /* ths data is 5 bits, fs = +-2g */
        .click_ths                = MYNEWT_VAL(LIS2DH12_CFG_TAP_THS),
        /* shock is maximum time data can be over threshold to register as tap
           0 = 2*1/ODR, LSB = 4*1/ODR */
        .time_limit               = MYNEWT_VAL(LIS2DH12_CFG_TAP_TIME_LIMIT),
        /* latency is time between taps in double tap, LSB = 1/ODR */
        .time_latency             = MYNEWT_VAL(LIS2DH12_CFG_TAP_TIME_LATENCY),
        /* quiet is time after tap data is just below threshold
           1/ODR */
        .time_window              = MYNEWT_VAL(LIS2DH12_CFG_TAP_TIME_WINDOW),
    },

    /* Read mode config */
    .read_mode                    =
    {
        .mode                     = MYNEWT_VAL(LIS2DH12_CFG_READ_MODE_MODE),
        .int_num                  = MYNEWT_VAL(LIS2DH12_CFG_READ_MODE_INT_NUM),
        .int_cfg                  = MYNEWT_VAL(LIS2DH12_CFG_READ_MODE_INT_CFG),
    },

    /* interrupt config */
    .int_cfg =
    {
        {
            .cfg = (MYNEWT_VAL(LIS2DH12_CFG_INT1_XLIE) << 0) |
                (MYNEWT_VAL(LIS2DH12_CFG_INT1_XHIE) << 1) |
                (MYNEWT_VAL(LIS2DH12_CFG_INT1_YLIE) << 2) |
                (MYNEWT_VAL(LIS2DH12_CFG_INT1_YHIE) << 3) |
                (MYNEWT_VAL(LIS2DH12_CFG_INT1_ZLIE) << 4) |
                (MYNEWT_VAL(LIS2DH12_CFG_INT1_ZHIE) << 5) |
                (MYNEWT_VAL(LIS2DH12_CFG_INT1_AOI_6D) << 6),
            .ths                  = MYNEWT_VAL(LIS2DH12_CFG_INT1_THS),
            .dur                  = MYNEWT_VAL(LIS2DH12_CFG_INT1_DUR),
        },
        {
            .cfg = (MYNEWT_VAL(LIS2DH12_CFG_INT2_XLIE) << 0) |
                (MYNEWT_VAL(LIS2DH12_CFG_INT2_XHIE) << 1) |
                (MYNEWT_VAL(LIS2DH12_CFG_INT2_YLIE) << 2) |
                (MYNEWT_VAL(LIS2DH12_CFG_INT2_YHIE) << 3) |
                (MYNEWT_VAL(LIS2DH12_CFG_INT2_ZLIE) << 4) |
                (MYNEWT_VAL(LIS2DH12_CFG_INT2_ZHIE) << 5) |
                (MYNEWT_VAL(LIS2DH12_CFG_INT2_AOI_6D) << 6),
            .ths                  = MYNEWT_VAL(LIS2DH12_CFG_INT2_THS),
            .dur                  = MYNEWT_VAL(LIS2DH12_CFG_INT2_DUR),
        }
    },

    /* Power mode */
    .power_mode                   = MYNEWT_VAL(LIS2DH12_CFG_POWER_MODE),

    /* fifo  config */
    .fifo_mode                    = MYNEWT_VAL(LIS2DH12_CFG_FIFO_MODE),
    .fifo_watermark               = MYNEWT_VAL(LIS2DH12_CFG_FIFO_WATERMARK),

    /* sleep/wakeup settings */
    .act_ths                      = MYNEWT_VAL(LIS2DH12_CFG_ACT_THS),
    .act_dur                      = MYNEWT_VAL(LIS2DH12_CFG_ACT_DUR),

    .latch_int1                   = MYNEWT_VAL(LIS2DH12_CFG_LATCH_INT1),
    .latch_int2                   = MYNEWT_VAL(LIS2DH12_CFG_LATCH_INT2),

    //.notif_cfg                    = (struct lis2dh12_notif_cfg *)lis2dh12_ncfg,
    //.max_num_notif                = sizeof(lis2dh12_ncfg)/sizeof(*lis2dh12_ncfg),

    /* Sensor type mask to track enabled sensors */
    .lc_s_mask                         = MYNEWT_VAL(LIS2DH12_CFG_MASK),
};

static struct sensor_itf bsp_msense_itf = {
    .si_type = SENSOR_ITF_SPI,
    .si_num  = 0,
    .si_cs_pin = 8,
    .si_ints = {
    { ACC_INT1, 1, MYNEWT_VAL(LIS2DH12_INT1_CFG_TRIGGER) },
    { ACC_INT2, 2, MYNEWT_VAL(LIS2DH12_INT2_CFG_TRIGGER) } },
    // .si_lock = &g_bsp_i2c0_itf_lock
};


// XXX:
//motion_mgr_sensor_t bsp_motion_mgr_sensor;
#define SENSOR_MOTION_INIT_FN lis2dh12_init
#define SENSOR_MOTION_INIT_DEVICE_SHELL lis2dh12_shell_init

int bsp_lis2dh12_set_power_mode(uint8_t mode)
{
    int rc = 0;
    // sensor_lock( &bsp_motion_sensor.sensor );
    // TODO: Not yet implemented
    // sensor_unlock( &bsp_motion_sensor.sensor );
    return rc;
}
#endif

static void
sensor_dev_create(void)
{
    int rc;
    (void)rc;

#if MYNEWT_VAL(BME280_ONB)
    rc = os_dev_create((struct os_dev *) &bme280, "bme280_0",
      OS_DEV_INIT_PRIMARY, 0, bme280_init, (void *)&spi_0_itf_bme);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(LIS2DH12_ONB)
    rc = os_dev_create((struct os_dev *) &lis2dh12, "lis2dh12_0",
      OS_DEV_INIT_PRIMARY, 0, lis2dh12_init, (void *)&spi_0_itf_lis);
    assert(rc == 0);
#endif

}

void
hal_bsp_init(void)
{
    int rc;
#if MYNEWT_VAL(SOFT_PWM)
    int idx;
#endif

    (void)rc;

    /* Make sure system clocks have started */
    hal_system_clock_start();

#if MYNEWT_VAL(TIMER_0)
    rc = hal_timer_init(0, NULL);
    assert(rc == 0);
#endif
#if MYNEWT_VAL(TIMER_1)
    rc = hal_timer_init(1, NULL);
    assert(rc == 0);
#endif
#if MYNEWT_VAL(TIMER_2)
    rc = hal_timer_init(2, NULL);
    assert(rc == 0);
#endif
#if MYNEWT_VAL(TIMER_3)
    rc = hal_timer_init(3, NULL);
    assert(rc == 0);
#endif
#if MYNEWT_VAL(TIMER_4)
    rc = hal_timer_init(4, NULL);
    assert(rc == 0);
#endif
#if MYNEWT_VAL(TIMER_5)
    rc = hal_timer_init(5, NULL);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(ADC_0)
    rc = os_dev_create((struct os_dev *) &os_bsp_adc0,
                       "adc0",
                       OS_DEV_INIT_KERNEL,
                       OS_DEV_INIT_PRIO_DEFAULT,
                       nrf52_adc_dev_init,
                       &os_bsp_adc0_config);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(PWM_0)
    pwm0_idx = 0;
    rc = os_dev_create((struct os_dev *) &os_bsp_pwm0,
                       "pwm0",
                       OS_DEV_INIT_KERNEL,
                       OS_DEV_INIT_PRIO_DEFAULT,
                       nrf52_pwm_dev_init,
                       &pwm0_idx);
    assert(rc == 0);
#endif
#if MYNEWT_VAL(PWM_1)
    pwm1_idx = 1;
    rc = os_dev_create((struct os_dev *) &os_bsp_pwm1,
                       "pwm1",
                       OS_DEV_INIT_KERNEL,
                       OS_DEV_INIT_PRIO_DEFAULT,
                       nrf52_pwm_dev_init,
                       &pwm1_idx);
    assert(rc == 0);
#endif
#if MYNEWT_VAL(PWM_2)
    pwm2_idx = 2;
    rc = os_dev_create((struct os_dev *) &os_bsp_pwm2,
                       "pwm2",
                       OS_DEV_INIT_KERNEL,
                       OS_DEV_INIT_PRIO_DEFAULT,
                       nrf52_pwm_dev_init,
                       &pwm2_idx);
    assert(rc == 0);
#endif
#if MYNEWT_VAL(SOFT_PWM)
    for (idx = 0; idx < MYNEWT_VAL(SOFT_PWM_DEVS); idx++)
    {
        spwm_name[idx] = "spwm0";
        spwm_name[idx][4] = '0' + idx;
        spwm_idx[idx] = idx;
        rc = os_dev_create((struct os_dev *) &os_bsp_spwm[idx],
                           spwm_name[idx],
                           OS_DEV_INIT_KERNEL,
                           OS_DEV_INIT_PRIO_DEFAULT,
                           soft_pwm_dev_init,
                           &spwm_idx[idx]);
        assert(rc == 0);
    }
#endif

#if (MYNEWT_VAL(OS_CPUTIME_TIMER_NUM) >= 0)
    rc = os_cputime_init(MYNEWT_VAL(OS_CPUTIME_FREQ));
    assert(rc == 0);
#endif

#if MYNEWT_VAL(I2C_0)
    bsp_i2c0_itf_lock_init();
    rc = hal_i2c_init(0, (void *)&hal_i2c_cfg);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(SPI_0_MASTER)
    rc = hal_spi_init(0, (void *)&os_bsp_spi0m_cfg, HAL_SPI_TYPE_MASTER);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(SPI_0_SLAVE)
    rc = hal_spi_init(0, (void *)&os_bsp_spi0s_cfg, HAL_SPI_TYPE_SLAVE);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(UART_0)
    rc = os_dev_create((struct os_dev *) &os_bsp_uart0, "uart0",
      OS_DEV_INIT_PRIMARY, 0, uart_hal_init, (void *)&os_bsp_uart0_cfg);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(UART_1)
    rc = os_dev_create((struct os_dev *) &os_bsp_bitbang_uart1, "uart1",
      OS_DEV_INIT_PRIMARY, 0, uart_bitbang_init, (void *)&os_bsp_uart1_cfg);
    assert(rc == 0);
#endif

    sensor_dev_create();
}

#if MMGR
static void bsp_init_sensors(void)
{
    int rc;
    (void)rc;

    #if MYNEWT_VAL(SENSOR_MOTION_ENABLE)
    rc = motion_mgr_sensor_create(&bsp_motion_mgr_sensor,
        (void*)&bsp_motion_sensor, "motion", SENSOR_MOTION_INIT_FN,
        (void*)&bsp_msense_itf);
    assert(rc == 0);
    #endif

    #if MYNEWT_VAL(SENSOR_MOTION_ENABLE)
    motion_mgr_init(&bsp_motion_mgr_sensor);

    rc = motion_mgr_config_sensor(&bsp_motion_mgr_sensor,
        (void*)&bsp_motion_init_cfg);
    assert(rc == 0);
    #endif

    #if MYNEWT_VAL(SENSOR_MOTION_ENABLE) && MYNEWT_VAL(SENSOR_MOTION_DRIVER_SHELL_ENABLE)
    SENSOR_MOTION_INIT_DEVICE_SHELL();
    #endif
}
#endif

void bsp_pkg_init(void)
{
#if MYNEWT_VAL(I2C_0)
    bsp_i2c0_itf_lock_init();
#endif

#if MMGR
    bsp_init_sensors();
#endif
}
