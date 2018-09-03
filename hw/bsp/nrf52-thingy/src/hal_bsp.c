
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

#include <stdint.h>
#include <stddef.h>
#include <assert.h>
#include "os/mynewt.h"
#include "nrfx.h"
#include "flash_map/flash_map.h"
#include "hal/hal_bsp.h"
#include "hal/hal_flash.h"
#include "hal/hal_system.h"
#include "mcu/nrf52_hal.h"
#include "mcu/nrf52_periph.h"
#include "bsp/bsp.h"
#include "defs/sections.h"
#if MYNEWT_VAL(SOFT_PWM)
#include "pwm/pwm.h"
#include "soft_pwm/soft_pwm.h"
#endif
#if MYNEWT_VAL(UARTBB_0)
#include "uart_bitbang/uart_bitbang.h"
#endif
#if MYNEWT_VAL(LIS2DH12_ONB)
#include "lis2dh12/lis2dh12.h"
static struct lis2dh12 lis2dh12;
#endif

#if MYNEWT_VAL(SOFT_PWM)
static struct pwm_dev os_bsp_spwm[MYNEWT_VAL(SOFT_PWM_DEVS)];
#endif

#if MYNEWT_VAL(UARTBB_0)
static const struct uart_bitbang_conf os_bsp_uartbb0_cfg = {
    .ubc_txpin = MYNEWT_VAL(UARTBB_0_PIN_TX),
    .ubc_rxpin = MYNEWT_VAL(UARTBB_0_PIN_RX),
    .ubc_cputimer_freq = MYNEWT_VAL(OS_CPUTIME_FREQ),
};
#endif

#if MYNEWT_VAL(LIS2DH12_ONB)
static struct sensor_itf i2c_0_itf_lis = {
    .si_type = SENSOR_ITF_I2C,
    .si_num  = 0,
    .si_addr = 0x19
};
#endif

/* XXX andk */
#include "hal/hal_gpio.h"
#include "lis2dh12/lis2dh12.h"
#include "motion_mgr/motion_mgr.h"
motion_mgr_sensor_t bsp_motion_mgr_sensor;
struct lis2dh12 bsp_motion_sensor;
#define SENSOR_MOTION_INIT_FN lis2dh12_init
#define SENSOR_MOTION_INIT_DEVICE_SHELL lis2dh12_shell_init

#if MYNEWT_VAL(I2C_0)
struct os_mutex g_bsp_i2c0_itf_lock;

static int
bsp_i2c0_itf_lock_init(void)
{
    return os_mutex_init(&g_bsp_i2c0_itf_lock);
}
#endif

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

    .notif_cfg                    = (struct lis2dh12_notif_cfg *)lis2dh12_ncfg,
    .max_num_notif                = sizeof(lis2dh12_ncfg)/sizeof(*lis2dh12_ncfg),

    /* Sensor type mask to track enabled sensors */
    .lc_s_mask                         = MYNEWT_VAL(LIS2DH12_CFG_MASK),
};

static struct sensor_itf bsp_msense_itf = {
    .si_type = SENSOR_ITF_I2C,
    .si_num  = 0,
    .si_addr = 0x19,
    .si_ints = {
    { ACC_INT1, 1, MYNEWT_VAL(LIS2DH12_INT1_CFG_TRIGGER) },
    { ACC_INT2, 2, MYNEWT_VAL(LIS2DH12_INT2_CFG_TRIGGER) } },
    // .si_lock = &g_bsp_i2c0_itf_lock
};

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
    if (id == 0) {
        return &nrf52k_flash_dev;
    }

    return NULL;
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
    cfg.lc_pull_up_disc = 1;

    rc = lis2dh12_config((struct lis2dh12 *)dev, &cfg);
    SYSINIT_PANIC_ASSERT(rc == 0);

    os_dev_close(dev);
#endif
    return 0;
}

static void
sensor_dev_create(void)
{
    int rc;
    (void)rc;

#if MYNEWT_VAL(LIS2DH12_ONB)
    rc = os_dev_create((struct os_dev *) &lis2dh12, "lis2dh12_0",
      OS_DEV_INIT_PRIMARY, 0, lis2dh12_init, (void *)&i2c_0_itf_lis);
    assert(rc == 0);
#endif

}

void
hal_bsp_init(void)
{
#if MYNEWT_VAL(SOFT_PWM)
    int rc;
    int idx;
    char *spwm_name;
#endif

    /* Make sure system clocks have started */
    hal_system_clock_start();

    /* Create all available nRF52840 peripherals */
    nrf52_periph_create();

#if MYNEWT_VAL(SOFT_PWM)
    for (idx = 0; idx < MYNEWT_VAL(SOFT_PWM_DEVS); idx++) {
        asprintf(&spwm_name, "spwm%d", idx);
        rc = os_dev_create(&os_bsp_spwm[idx].pwm_os_dev, spwm_name,
                           OS_DEV_INIT_KERNEL, OS_DEV_INIT_PRIO_DEFAULT,
                           soft_pwm_dev_init, UINT_TO_POINTER(idx));
        assert(rc == 0);
    }
#endif

#if MYNEWT_VAL(UARTBB_0)
    rc = os_dev_create(&os_bsp_uartbb0.ud_dev, "uartbb0",
                       OS_DEV_INIT_PRIMARY, 0, uart_bitbang_init,
                       (void *)&os_bsp_uartbb0_cfg);
    assert(rc == 0);
#endif

    sensor_dev_create();
}

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

void bsp_pkg_init(void)
{
#if MYNEWT_VAL(I2C_0)
    bsp_i2c0_itf_lock_init();
#endif

    bsp_init_sensors();
}
