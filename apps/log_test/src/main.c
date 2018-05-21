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

#include "sysinit/sysinit.h"
#include "os/os.h"
#include "log/log.h"
#include "log/log_fcb_slot1.h"
#include "console/console.h"
#include "shell/shell.h"
#include "fcb/fcb.h"
#include "cbmem/cbmem.h"
#include "nimble/ble.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"

static struct log my_log;
static struct fcb_log fcb_log;
static struct cbmem cbmem;
static uint32_t cbmem_buf[2048];
static struct log_fcb_slot1 lfs1;

static void start_advertise(void);

static int
ble_gap_event_func(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        log_printf(&my_log, LOG_MODULE_DEFAULT, LOG_LEVEL_INFO, "BLE_GAP_EVENT_CONNECT (%d)", event->connect.status);

        if (event->connect.status != 0) {
            start_advertise();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        log_printf(&my_log, LOG_MODULE_DEFAULT, LOG_LEVEL_INFO, "BLE_GAP_EVENT_DISCONNECT");

        start_advertise();
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        log_printf(&my_log, LOG_MODULE_DEFAULT, LOG_LEVEL_INFO, "BLE_GAP_EVENT_ADV_COMPLETE");

        start_advertise();
        return 0;
    }

    return 0;
}

static void
start_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    const char *name;
    uint8_t own_addr_type;
    int rc;

    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    assert(rc == 0);

    name = ble_svc_gap_device_name();

    memset(&fields, 0, sizeof fields);
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;
    rc = ble_gap_adv_set_fields(&fields);
    assert(rc == 0);

    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, ble_gap_event_func, NULL);
    assert(rc == 0);
}

static void
ble_on_sync_func(void)
{
    int rc;

    rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);

    start_advertise();
}

static int
reinit_fcb(struct fcb_log *arg)
{
    const struct flash_area *fa;
    int rc;

    rc = fcb_init(&fcb_log.fl_fcb);
    if (rc) {
        /* Need to erase full area here */
        rc = flash_area_open(FLASH_AREA_IMAGE_1, &fa);
        if (rc) {
            goto done;
        }
        flash_area_erase(fa, 0, fa->fa_size);

        rc = fcb_init(&fcb_log.fl_fcb);
    }

done:
    return rc;
}

static void
init_log(void)
{
    struct flash_area *fa_sectors;
    int fa_sectors_cnt;
    int rc;

    rc = flash_area_to_sectors(FLASH_AREA_IMAGE_1, &fa_sectors_cnt, NULL);
    if (rc || !fa_sectors_cnt) {
        goto done;
    }

    fa_sectors = malloc(sizeof(struct flash_area) * fa_sectors_cnt);
    if (!fa_sectors) {
        goto done;
    }

    rc = flash_area_to_sectors(FLASH_AREA_IMAGE_1, &fa_sectors_cnt, fa_sectors);
    if (rc || !fa_sectors_cnt) {
        goto done;
    }

    fcb_log.fl_fcb.f_sectors = fa_sectors;
    fcb_log.fl_fcb.f_sector_cnt = fa_sectors_cnt;
    fcb_log.fl_fcb.f_magic = 0xDEADBEEF;
    fcb_log.fl_fcb.f_version = g_log_info.li_version;
    fcb_log.fl_entries = 0;

    cbmem_init(&cbmem, cbmem_buf, sizeof(cbmem_buf));

    log_fcb_slot1_init(&lfs1, &fcb_log, &cbmem, reinit_fcb);

done:
    log_register("testlog", &my_log, &log_fcb_slot1_handler, &lfs1, LOG_SYSLEVEL);
}

int
main(void)
{
    int rc;

    sysinit();

    init_log();

    log_register("ble_hs", &ble_hs_log, &log_console_handler, NULL, LOG_LEVEL_INFO);

    ble_hs_cfg.sync_cb = ble_on_sync_func;

    rc = ble_svc_gap_device_name_set("log_test");
    assert(rc == 0);

    while (1) {
        os_eventq_run(os_eventq_dflt_get());
    }

    return 0;
}
