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

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "os/os_mbuf.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "bleprph.h"

static const ble_uuid128_t gatt_svr_svc_uuid =
    BLE_UUID128_INIT(0x10, 0xfe, 0xf3, 0x76, 0xea, 0x81, 0x44, 0xbc,
                     0x8c, 0x11, 0x02, 0x67, 0x2d, 0x89, 0xb8, 0x9d);

#define GATT_SVR_NOTIFY_MAX_LEN 180
#define GATT_SVR_COMMAND_MAX_LEN 256
static char gatt_svr_chr_val[GATT_SVR_NOTIFY_MAX_LEN] = "waiting for N2K data";
static uint16_t gatt_svr_chr_val_handle;
static const ble_uuid128_t gatt_svr_chr_uuid =
    BLE_UUID128_INIT(0x11, 0xfe, 0xf3, 0x76, 0xea, 0x81, 0x44, 0xbc,
                     0x8c, 0x11, 0x02, 0x67, 0x2d, 0x89, 0xb8, 0x9d);
static uint16_t gatt_svr_cmd_val_handle;
static const ble_uuid128_t gatt_svr_cmd_chr_uuid =
    BLE_UUID128_INIT(0x12, 0xfe, 0xf3, 0x76, 0xea, 0x81, 0x44, 0xbc,
                     0x8c, 0x11, 0x02, 0x67, 0x2d, 0x89, 0xb8, 0x9d);
static char gatt_svr_command_buf[GATT_SVR_COMMAND_MAX_LEN];
static size_t gatt_svr_command_len;
static bool gatt_svr_command_overflow;
static SemaphoreHandle_t gatt_svr_notify_lock;
static gatt_svr_command_handler_fn gatt_svr_command_handler;
static void *gatt_svr_command_handler_ctx;

static int
gatt_svc_access(uint16_t conn_handle, uint16_t attr_handle,
                struct ble_gatt_access_ctxt *ctxt,
                void *arg);

static void
gatt_svr_dispatch_command_line(const char *line)
{
    if ((line != NULL) && (gatt_svr_command_handler != NULL)) {
        gatt_svr_command_handler(line, gatt_svr_command_handler_ctx);
    }
}

/* App commands are framed as UTF-8 text lines terminated by '\n'. */
static int
gatt_svr_handle_command_write(struct ble_gatt_access_ctxt *ctxt)
{
    uint16_t chunk_len = OS_MBUF_PKTLEN(ctxt->om);
    char chunk[GATT_SVR_COMMAND_MAX_LEN];

    if (chunk_len >= sizeof(chunk)) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    if (os_mbuf_copydata(ctxt->om, 0, chunk_len, chunk) != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    chunk[chunk_len] = '\0';
    /* Temporary debug instrumentation for BLE command RX visibility. */
    MODLOG_DFLT(WARN,
                "\n"
                "================ BLE CMD RX ================\n"
                "STAGE: write received on command characteristic\n"
                "BYTES: %u\n"
                "CHUNK: %s\n"
                "============================================\n",
                (unsigned)chunk_len,
                chunk_len > 0u ? chunk : "(empty)");

    for (uint16_t i = 0; i < chunk_len; i++) {
        char ch = chunk[i];

        if (ch == '\r') {
            continue;
        }
        if (ch == '\n') {
            if (!gatt_svr_command_overflow && gatt_svr_command_len > 0u) {
                gatt_svr_command_buf[gatt_svr_command_len] = '\0';
                MODLOG_DFLT(WARN,
                            "\n"
                            "================ BLE CMD RX ================\n"
                            "STAGE: full command line assembled\n"
                            "RAW: %s\n"
                            "============================================\n",
                            gatt_svr_command_buf);
                gatt_svr_dispatch_command_line(gatt_svr_command_buf);
            }
            gatt_svr_command_len = 0u;
            gatt_svr_command_overflow = false;
            continue;
        }
        if (gatt_svr_command_overflow) {
            continue;
        }
        if (gatt_svr_command_len + 1u >= sizeof(gatt_svr_command_buf)) {
            MODLOG_DFLT(WARN,
                        "\n"
                        "================ BLE CMD RX ================\n"
                        "STAGE: command buffer overflow\n"
                        "ACTION: dropping current line\n"
                        "============================================\n");
            gatt_svr_command_len = 0u;
            gatt_svr_command_overflow = true;
            continue;
        }
        gatt_svr_command_buf[gatt_svr_command_len++] = ch;
    }

    return 0;
}

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        /*** Service ***/
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[])
        { {
                .uuid = &gatt_svr_chr_uuid.u,
                .access_cb = gatt_svc_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &gatt_svr_chr_val_handle,
            }, {
                /* Dedicated app -> ESP32 command input. */
                .uuid = &gatt_svr_cmd_chr_uuid.u,
                .access_cb = gatt_svc_access,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
                .val_handle = &gatt_svr_cmd_val_handle,
            }, {
                0, /* No more characteristics in this service. */
            }
        },
    },

    {
        0, /* No more services. */
    },
};

/* Access callback for the single READ + NOTIFY characteristic. */
static int
gatt_svc_access(uint16_t conn_handle, uint16_t attr_handle,
                struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;
    (void)arg;

    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            MODLOG_DFLT(INFO, "Characteristic read; conn_handle=%d attr_handle=%d\n",
                        conn_handle, attr_handle);
        } else {
            MODLOG_DFLT(INFO, "Characteristic read by NimBLE stack; attr_handle=%d\n",
                        attr_handle);
        }
        if (attr_handle == gatt_svr_chr_val_handle) {
            rc = os_mbuf_append(ctxt->om,
                                gatt_svr_chr_val,
                                strlen(gatt_svr_chr_val));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        goto unknown;

    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        if (attr_handle == gatt_svr_cmd_val_handle) {
            return gatt_svr_handle_command_write(ctxt);
        }
        goto unknown;

    default:
        goto unknown;
    }

unknown:
    /* Unknown characteristic/descriptor;
     * The NimBLE host should not have called this function;
     */
    assert(0);
    return BLE_ATT_ERR_UNLIKELY;
}

void
gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    char buf[BLE_UUID_STR_LEN];

    switch (ctxt->op) {
    case BLE_GATT_REGISTER_OP_SVC:
        MODLOG_DFLT(DEBUG, "registered service %s with handle=%d\n",
                    ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                    ctxt->svc.handle);
        break;

    case BLE_GATT_REGISTER_OP_CHR:
        MODLOG_DFLT(DEBUG, "registering characteristic %s with "
                    "def_handle=%d val_handle=%d\n",
                    ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                    ctxt->chr.def_handle,
                    ctxt->chr.val_handle);
        break;

    case BLE_GATT_REGISTER_OP_DSC:
        MODLOG_DFLT(DEBUG, "registering descriptor %s with handle=%d\n",
                    ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                    ctxt->dsc.handle);
        break;

    default:
        assert(0);
        break;
    }
}

int
gatt_svr_init(void)
{
    int rc;

    if (gatt_svr_notify_lock == NULL) {
        gatt_svr_notify_lock = xSemaphoreCreateMutex();
        if (gatt_svr_notify_lock == NULL) {
            return BLE_HS_ENOMEM;
        }
    }

    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    return 0;
}

uint16_t
gatt_svr_notify_chr_val_handle(void)
{
    return gatt_svr_chr_val_handle;
}

int
gatt_svr_set_notify_text(const char *text)
{
    int written;

    if (text == NULL) {
        return BLE_HS_EINVAL;
    }

    written = snprintf(gatt_svr_chr_val, sizeof(gatt_svr_chr_val), "%s", text);
    if (written < 0) {
        return BLE_HS_EINVAL;
    }

    if (written >= (int)sizeof(gatt_svr_chr_val)) {
        gatt_svr_chr_val[sizeof(gatt_svr_chr_val) - 1] = '\0';
    }

    return 0;
}

int
gatt_svr_chr_notify(uint16_t conn_handle)
{
    struct os_mbuf *om;
    int rc;

    if ((gatt_svr_notify_lock != NULL) &&
        (xSemaphoreTake(gatt_svr_notify_lock, portMAX_DELAY) != pdTRUE)) {
        return BLE_HS_ETIMEOUT;
    }

    om = ble_hs_mbuf_from_flat(gatt_svr_chr_val, strlen(gatt_svr_chr_val));
    if (om == NULL) {
        if (gatt_svr_notify_lock != NULL) {
            xSemaphoreGive(gatt_svr_notify_lock);
        }
        return BLE_HS_ENOMEM;
    }

    rc = ble_gatts_notify_custom(conn_handle, gatt_svr_chr_val_handle, om);
    if (rc != 0) {
        os_mbuf_free_chain(om);
    }
    if (gatt_svr_notify_lock != NULL) {
        xSemaphoreGive(gatt_svr_notify_lock);
    }
    return rc;
}

int
gatt_svr_notify_text(uint16_t conn_handle, const char *text)
{
    struct os_mbuf *om;
    int rc;

    if ((text == NULL) || (gatt_svr_notify_lock == NULL)) {
        return BLE_HS_EINVAL;
    }
    if (xSemaphoreTake(gatt_svr_notify_lock, portMAX_DELAY) != pdTRUE) {
        return BLE_HS_ETIMEOUT;
    }

    rc = gatt_svr_set_notify_text(text);
    if (rc != 0) {
        xSemaphoreGive(gatt_svr_notify_lock);
        return rc;
    }

    om = ble_hs_mbuf_from_flat(gatt_svr_chr_val, strlen(gatt_svr_chr_val));
    if (om == NULL) {
        xSemaphoreGive(gatt_svr_notify_lock);
        return BLE_HS_ENOMEM;
    }

    rc = ble_gatts_notify_custom(conn_handle, gatt_svr_chr_val_handle, om);
    if (rc != 0) {
        os_mbuf_free_chain(om);
    }
    xSemaphoreGive(gatt_svr_notify_lock);
    return rc;
}

int
gatt_svr_notify_text_chunks(uint16_t conn_handle, const char *text)
{
    size_t total_len;
    size_t offset = 0u;

    if (text == NULL) {
        return BLE_HS_EINVAL;
    }

    total_len = strlen(text);
    while (offset < total_len) {
        char chunk[GATT_SVR_NOTIFY_MAX_LEN];
        size_t chunk_len = total_len - offset;
        int rc;

        if (chunk_len >= sizeof(chunk)) {
            chunk_len = sizeof(chunk) - 1u;
        }

        memcpy(chunk, &text[offset], chunk_len);
        chunk[chunk_len] = '\0';
        rc = gatt_svr_notify_text(conn_handle, chunk);
        if (rc != 0) {
            return rc;
        }
        offset += chunk_len;
    }

    return 0;
}

void
gatt_svr_set_command_handler(gatt_svr_command_handler_fn handler, void *ctx)
{
    gatt_svr_command_handler = handler;
    gatt_svr_command_handler_ctx = ctx;
}
