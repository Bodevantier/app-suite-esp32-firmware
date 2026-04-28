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

#include "esp_log.h"
#include "nvs_flash.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/spi_slave.h"
#include "esp_attr.h"
#include "esp_bt.h"
#include "esp_err.h"
#include "esp_nimble_hci.h"
/* BLE */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "bleprph.h"
#include "n2k_raw_frame.h"
#include "spi_n2k_transport.h"

#define SPI_SLAVE_HOST   SPI2_HOST
#define PIN_NUM_MOSI     11
#define PIN_NUM_MISO     13
#define PIN_NUM_SCLK     12
#define PIN_NUM_CS       10
#define SPI_RX_BUF_SIZE  256
#define SPI_CUSTOM_TX_QUEUE_LEN 8
#define N2K_LOCAL_SOURCE 0x31u
#define N2K_PGN_ADDRESS_CLAIM 60928u
#define N2K_PGN_SUPPORTED_PGN_LIST 126464u
#define N2K_PGN_PRODUCT_INFO 126996u
#define N2K_PGN_CONFIGURATION_INFO 126998u
#define BLE_NOTIFY_TEXT_MAX_LEN 180
#define BLE_NOTIFY_BINARY_MAX_LEN 180
#define BLE_NOTIFY_BINARY_PACKET_VERSION 1u
#define BLE_NOTIFY_BINARY_PACKET_TYPE_FRAME_BATCH 1u
#define BLE_NOTIFY_BINARY_PACKET_HEADER_LEN 8u
#define BLE_NOTIFY_BINARY_FRAME_LEN 14u
#define BLE_NOTIFY_BINARY_MAX_FRAMES ((BLE_NOTIFY_BINARY_MAX_LEN - BLE_NOTIFY_BINARY_PACKET_HEADER_LEN) / BLE_NOTIFY_BINARY_FRAME_LEN)
#define BLE_NOTIFY_BINARY_QUEUE_LEN 64
#define BLE_NOTIFY_TASK_STACK_SIZE 8192
#define SPI_N2K_TASK_STACK_SIZE 8192
#define BLE_NOTIFY_TASK_IDLE_DELAY_MS 1000u
#define BLE_NOTIFY_TASK_REFRESH_POLL_MS 200u
#define BLE_DEVICE_ADV_NAME "SDolve N2K BLE"

typedef struct {
    uint8_t len;
    uint8_t data[SPI_N2K_MAX_PACKET_LEN];
} SpiQueuedPacket_t;

#if CONFIG_EXAMPLE_EXTENDED_ADV
static uint8_t ext_adv_pattern_1[] = {
    0x02, BLE_HS_ADV_TYPE_FLAGS, 0x06,
    0x03, BLE_HS_ADV_TYPE_COMP_UUIDS16, 0xab, 0xcd,
    0x03, BLE_HS_ADV_TYPE_COMP_UUIDS16, 0x18, 0x11,
    0x11, BLE_HS_ADV_TYPE_COMP_NAME, 'n', 'i', 'm', 'b', 'l', 'e', '-', 'b', 'l', 'e', 'p', 'r', 'p', 'h', '-', 'e',
};
#endif

static const char *tag = "NimBLE_BLE_PRPH";
static ble_uuid128_t advertised_service_uuid =
    BLE_UUID128_INIT(0x10, 0xfe, 0xf3, 0x76, 0xea, 0x81, 0x44, 0xbc,
                     0x8c, 0x11, 0x02, 0x67, 0x2d, 0x89, 0xb8, 0x9d);
static int bleprph_gap_event(struct ble_gap_event *event, void *arg);
#if CONFIG_EXAMPLE_RANDOM_ADDR
static uint8_t own_addr_type = BLE_OWN_ADDR_RANDOM;
#else
static uint8_t own_addr_type;
#endif

#if MYNEWT_VAL(BLE_EATT_CHAN_NUM) > 0
static uint16_t cids[MYNEWT_VAL(BLE_EATT_CHAN_NUM)];
static uint16_t bearers;
#endif
static uint16_t active_conn_handle = BLE_HS_CONN_HANDLE_NONE;

static QueueHandle_t spi_custom_tx_queue;
static QueueHandle_t ble_binary_frame_queue;
static uint16_t ble_binary_packet_sequence = 0u;
static volatile bool ble_device_list_request_pending = false;

static bool spi_enqueue_custom_packet(const uint8_t *packet, size_t packet_len);
static void ble_queue_binary_frame(const N2K_RawFrame_t *frame);
static size_t ble_build_binary_packet(uint8_t *out_buf,
                                      size_t out_buf_size,
                                      uint16_t sequence,
                                      const N2K_RawFrame_t *frames,
                                      size_t frame_count);
static bool spi_send_device_list_request(void);
static void spi_log_packet_hex(const char *prefix, const uint8_t *data, size_t len);

static bool spi_enqueue_custom_packet(const uint8_t *packet, size_t packet_len) {
    SpiQueuedPacket_t queued;

    if ((packet == NULL) || (packet_len == 0u) || (packet_len > sizeof(queued.data)) ||
        (spi_custom_tx_queue == NULL)) {
        return false;
    }

    memset(&queued, 0, sizeof(queued));
    queued.len = (uint8_t)packet_len;
    memcpy(queued.data, packet, packet_len);
    return xQueueSend(spi_custom_tx_queue, &queued, 0) == pdTRUE;
}

static void ble_queue_binary_frame(const N2K_RawFrame_t *frame) {
    if ((frame == NULL) || (ble_binary_frame_queue == NULL)) {
        return;
    }

    (void)xQueueSend(ble_binary_frame_queue, frame, 0);
}

static size_t ble_build_binary_packet(uint8_t *out_buf,
                                      size_t out_buf_size,
                                      uint16_t sequence,
                                      const N2K_RawFrame_t *frames,
                                      size_t frame_count) {
    size_t offset = 0u;

    if ((out_buf == NULL) || (frames == NULL) || (frame_count == 0u)) {
        return 0u;
    }
    if (frame_count > BLE_NOTIFY_BINARY_MAX_FRAMES) {
        return 0u;
    }
    if (out_buf_size < (BLE_NOTIFY_BINARY_PACKET_HEADER_LEN + (frame_count * BLE_NOTIFY_BINARY_FRAME_LEN))) {
        return 0u;
    }

    out_buf[offset++] = BLE_NOTIFY_BINARY_PACKET_VERSION;
    out_buf[offset++] = BLE_NOTIFY_BINARY_PACKET_TYPE_FRAME_BATCH;
    out_buf[offset++] = (uint8_t)(sequence & 0xFFu);
    out_buf[offset++] = (uint8_t)((sequence >> 8u) & 0xFFu);
    out_buf[offset++] = (uint8_t)frame_count;
    out_buf[offset++] = 0u;
    out_buf[offset++] = 0u;
    out_buf[offset++] = 0u;

    for (size_t i = 0u; i < frame_count; i++) {
        uint8_t dlc = n2k_raw_frame_clamp_dlc(frames[i].dlc);

        out_buf[offset++] = (uint8_t)(frames[i].can_id & 0xFFu);
        out_buf[offset++] = (uint8_t)((frames[i].can_id >> 8u) & 0xFFu);
        out_buf[offset++] = (uint8_t)((frames[i].can_id >> 16u) & 0xFFu);
        out_buf[offset++] = (uint8_t)((frames[i].can_id >> 24u) & 0xFFu);
        out_buf[offset++] = dlc;
        out_buf[offset++] = frames[i].flags;
        memcpy(&out_buf[offset], frames[i].data, N2K_RAW_FRAME_MAX_DATA_LEN);
        offset += N2K_RAW_FRAME_MAX_DATA_LEN;
    }

    return offset;
}

static void spi_log_packet_hex(const char *prefix, const uint8_t *data, size_t len) {
    char line[3u * SPI_N2K_MAX_PACKET_LEN + 1u];
    size_t max_bytes;
    size_t pos = 0u;

    if ((prefix == NULL) || (data == NULL)) {
        return;
    }

    max_bytes = len;
    if (max_bytes > SPI_N2K_MAX_PACKET_LEN) {
        max_bytes = SPI_N2K_MAX_PACKET_LEN;
    }

    for (size_t i = 0u; i < max_bytes; i++) {
        if ((pos + 3u) >= sizeof(line)) {
            break;
        }
        pos += (size_t)snprintf(&line[pos], sizeof(line) - pos, "%02X ", data[i]);
    }
    if (pos > 0u) {
        line[pos - 1u] = '\0';
    } else {
        line[0] = '\0';
    }

    ESP_LOGI(tag, "%s type=0x%02X len=%u data=[%s]",
             prefix,
             max_bytes >= 3u ? data[2] : 0u,
             (unsigned)len,
             line);
}

/* Send a DEVICE_LIST_REQUEST SPI packet to the STM32.
 * The STM32 will broadcast ISO Requests for AddressClaim, PGN List, and
 * ProductInfo; responses arrive as regular N2K_RX_FRAME packets. */
static bool spi_send_device_list_request(void) {
    uint8_t packet[SPI_N2K_MAX_PACKET_LEN];
    size_t packet_len = 0u;

    if (!spi_n2k_transport_build_custom_packet(SPI_N2K_PKT_TYPE_DEVICE_LIST_REQUEST,
                                                NULL,
                                                0u,
                                                packet,
                                                sizeof(packet),
                                                &packet_len)) {
        ESP_LOGE(tag, "DEVICE_LIST_REQUEST build failed");
        return false;
    }
    if (!spi_enqueue_custom_packet(packet, packet_len)) {
        ESP_LOGW(tag, "DEVICE_LIST_REQUEST queue full");
        return false;
    }
    spi_log_packet_hex("SPI TX", packet, packet_len);
    ESP_LOGI(tag, "scan_n2k: DEVICE_LIST_REQUEST sent");
    return true;
}

static void ble_handle_command_line(const char *command_line, void *ctx)
{
    (void)ctx;

    if (command_line == NULL) {
        ESP_LOGW(tag, "BLE command parse failed: empty input");
        return;
    }

    /* Accept both legacy "request_device_list" and the new "scan_n2k" command. */
    if ((strncmp(command_line, "request_device_list", 19) == 0) ||
        (strncmp(command_line, "scan_n2k", 8) == 0)) {
        ble_device_list_request_pending = true;
        ESP_LOGI(tag, "BLE command scan_n2k accepted");
    } else {
        ESP_LOGW(tag, "BLE command ignored: unsupported command '%s'", command_line);
    }
}

void ble_store_config_init(void);

static void spi_n2k_task(void *param) {
    (void)param;
    TickType_t last_stats_log = xTaskGetTickCount();
    static SpiN2kTransportParser_t parser;
    /* Two ping-pong buffer pairs so the SPI DMA can be armed with the NEXT
     * transaction before we start processing the previous one.  This ensures
     * the hardware never misses a CS pulse because we were busy elsewhere
     * (e.g. sending a BLE notification).  This is the pattern recommended by
     * the ESP-IDF spi_slave documentation:
     * https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/spi_slave.html */
    WORD_ALIGNED_ATTR static uint8_t tx_buf[2][SPI_RX_BUF_SIZE];
    WORD_ALIGNED_ATTR static uint8_t rx_buf[2][SPI_RX_BUF_SIZE];
    static spi_slave_transaction_t trans[2];
    static SpiQueuedPacket_t tx_custom;
    static SpiN2kPacket_t packet;
    static N2K_RawFrame_t frame;

    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = SPI_RX_BUF_SIZE,
    };

    spi_slave_interface_config_t slvcfg = {
        .spics_io_num = PIN_NUM_CS,
        .flags = 0,
        .queue_size = 2,   /* must be >= 2 for double-buffering */
        .mode = 0,
    };

    spi_n2k_transport_parser_init(&parser);
    ESP_ERROR_CHECK(spi_slave_initialize(SPI_SLAVE_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO));
    ESP_LOGI(tag, "SPI slave ready (MOSI=%d MISO=%d SCLK=%d CS=%d)",
             PIN_NUM_MOSI, PIN_NUM_MISO, PIN_NUM_SCLK, PIN_NUM_CS);

    /* Prime the queue: arm slot 0 so there is always one transaction waiting
     * for the master before we enter the main loop. */
    memset(rx_buf[0], 0, SPI_RX_BUF_SIZE);
    memset(tx_buf[0], 0xFF, SPI_RX_BUF_SIZE);
    memset(&trans[0], 0, sizeof(trans[0]));
    trans[0].length    = SPI_RX_BUF_SIZE * 8;
    trans[0].rx_buffer = rx_buf[0];
    trans[0].tx_buffer = tx_buf[0];
    ESP_ERROR_CHECK(spi_slave_queue_trans(SPI_SLAVE_HOST, &trans[0], portMAX_DELAY));

    int slot = 1; /* the slot we are about to arm next */

    while (1) {
        spi_slave_transaction_t *completed = NULL;

        /* ── Arm the NEXT slot before blocking for the previous result ── */
        memset(rx_buf[slot], 0, SPI_RX_BUF_SIZE);
        memset(tx_buf[slot], 0xFF, SPI_RX_BUF_SIZE);

        /* Fill the outbound tx buffer for this slot if a custom packet is
         * pending (device-list request, etc.). */
        if (ble_device_list_request_pending) {
            ble_device_list_request_pending = false;
            (void)spi_send_device_list_request();
        }
        if (xQueueReceive(spi_custom_tx_queue, &tx_custom, 0) == pdTRUE) {
            memcpy(tx_buf[slot], tx_custom.data, tx_custom.len);
            spi_log_packet_hex("SPI TX", tx_custom.data, tx_custom.len);
        }

        memset(&trans[slot], 0, sizeof(trans[slot]));
        trans[slot].length    = SPI_RX_BUF_SIZE * 8;
        trans[slot].rx_buffer = rx_buf[slot];
        trans[slot].tx_buffer = tx_buf[slot];

        /* Queue the next slot — the HW is now armed and will capture the very
         * next CS pulse regardless of how long we take below. */
        ESP_ERROR_CHECK(spi_slave_queue_trans(SPI_SLAVE_HOST, &trans[slot], portMAX_DELAY));

        /* Flip to the other slot for the next iteration. */
        slot ^= 1;

        /* ── Now block until the previously-queued transaction completes ── */
        if (spi_slave_get_trans_result(SPI_SLAVE_HOST, &completed, portMAX_DELAY) != ESP_OK) {
            continue;
        }

        int rx_len_bytes = (int)(completed->trans_len / 8);
        uint8_t *rx = (uint8_t *)completed->rx_buffer;

        for (int i = 0; i < rx_len_bytes; i++) {
            if (spi_n2k_transport_parser_consume_byte(&parser, rx[i], &packet)) {

                if (packet.pkt_type == SPI_N2K_PKT_TYPE_N2K_RX_FRAME) {
                    if (spi_n2k_transport_parse_frame_payload(&packet, &frame)) {
                        frame.flags |= N2K_RAW_FLAG_DIRECTION_RX;
                        /* Log AddressClaim / PGN-List / ProductInfo so we can
                         * confirm N2K identity responses are reaching the ESP32. */
                        {
                            uint32_t _pf  = (frame.can_id >> 16) & 0xFFu;
                            uint32_t _ps  = (frame.can_id >>  8) & 0xFFu;
                            uint32_t _dp  = (frame.can_id >> 24) & 0x01u;
                            uint32_t _pgn = (_dp << 16) | (_pf << 8) | (_pf < 240u ? 0u : _ps);
                            uint8_t  _sa  = (uint8_t)(frame.can_id & 0xFFu);
                            if (_pgn == 60928u || _pgn == 126464u || _pgn == 126996u) {
                                ESP_LOGI(tag, "N2K identity SPI RX: pgn=%lu src=%u dlc=%u",
                                         (unsigned long)_pgn, (unsigned)_sa, (unsigned)frame.dlc);
                            }
                        }
                        ble_queue_binary_frame(&frame);
                    }
                } else if (packet.pkt_type == SPI_N2K_PKT_TYPE_STATUS) {
                    if ((packet.payload_len > 0u) &&
                        (active_conn_handle != BLE_HS_CONN_HANDLE_NONE)) {
                        char text_buf[SPI_N2K_MAX_PAYLOAD_LEN + 1u];
                        size_t copy_len = (packet.payload_len < SPI_N2K_MAX_PAYLOAD_LEN)
                                          ? packet.payload_len
                                          : SPI_N2K_MAX_PAYLOAD_LEN;
                        memcpy(text_buf, packet.payload, copy_len);
                        text_buf[copy_len] = '\0';
                        gatt_svr_notify_text_chunks(active_conn_handle, text_buf);
                        ESP_LOGI(tag, "SPI STATUS->BLE len=%u", (unsigned)copy_len);
                    } else {
                        ESP_LOGI(tag, "SPI STATUS (no BLE conn) payload_len=%u",
                                 (unsigned)packet.payload_len);
                    }
                } else {
                    ESP_LOGD(tag, "SPI RX ignored pkt_type=0x%02X", (unsigned)packet.pkt_type);
                }
            }
        }

        TickType_t now = xTaskGetTickCount();
        if ((now - last_stats_log) >= pdMS_TO_TICKS(5000)) {
            ESP_LOGI(tag, "SPI stats ok=%lu bad_crc=%lu bad_len=%lu unknown_type=%lu bytes=%lu bad_sof=%lu",
                     (unsigned long)parser.stats.packets_ok,
                     (unsigned long)parser.stats.bad_crc,
                     (unsigned long)parser.stats.bad_len,
                     (unsigned long)parser.stats.unknown_type,
                     (unsigned long)parser.stats.bytes_seen,
                     (unsigned long)parser.stats.bad_sof);
            last_stats_log = now;
        }
    }
}

static void
ble_test_notify_task(void *param)
{
    uint8_t binary_packet[BLE_NOTIFY_BINARY_MAX_LEN];
    N2K_RawFrame_t binary_frames[BLE_NOTIFY_BINARY_MAX_FRAMES];
    int rc;
    (void)param;

    while (1) {
        /* If not connected, sleep briefly then re-check. */
        if ((active_conn_handle == BLE_HS_CONN_HANDLE_NONE) ||
            (ble_binary_frame_queue == NULL)) {
            vTaskDelay(pdMS_TO_TICKS(BLE_NOTIFY_TASK_IDLE_DELAY_MS));
            continue;
        }

        /* Block until at least one frame is available (or 100 ms timeout to
         * re-check the connection handle). This eliminates the artificial
         * 50 ms / 1000 ms polling delay and dispatches each frame immediately. */
        size_t binary_count = 0u;
        if (xQueueReceive(ble_binary_frame_queue,
                          &binary_frames[0],
                          pdMS_TO_TICKS(100u)) == pdTRUE) {
            binary_count = 1u;
            /* Drain any additional frames already queued (non-blocking). */
            while ((binary_count < BLE_NOTIFY_BINARY_MAX_FRAMES) &&
                   (xQueueReceive(ble_binary_frame_queue,
                                  &binary_frames[binary_count],
                                  0) == pdTRUE)) {
                binary_count++;
            }
        }

        if (binary_count > 0u) {
            size_t binary_len = ble_build_binary_packet(binary_packet,
                                                        sizeof(binary_packet),
                                                        ble_binary_packet_sequence++,
                                                        binary_frames,
                                                        binary_count);
            if (binary_len > 0u) {
                /* Retry up to 5 times on mbuf exhaustion (BLE_HS_ENOMEM = 6).
                 * A 50 ms yield lets the BLE stack free completed buffers. */
                int _attempts = 0;
                do {
                    rc = gatt_svr_notify_binary(active_conn_handle, binary_packet, (uint16_t)binary_len);
                    if (rc == BLE_HS_ENOMEM) {
                        vTaskDelay(pdMS_TO_TICKS(50));
                        _attempts++;
                    }
                } while (rc == BLE_HS_ENOMEM && _attempts < 5);
                if (rc != 0) {
                    ESP_LOGW(tag, "binary notification send failed (attempts=%d) rc=%d", _attempts + 1, rc);
                }
            }
        }
        /* No vTaskDelay — loop immediately to wait for the next frame. */
    }
}

#if NIMBLE_BLE_CONNECT
/**
 * Logs information about a connection to the console.
 */
static void
bleprph_print_conn_desc(struct ble_gap_conn_desc *desc)
{
    MODLOG_DFLT(INFO, "handle=%d our_ota_addr_type=%d our_ota_addr=",
                desc->conn_handle, desc->our_ota_addr.type);
    print_addr(desc->our_ota_addr.val);
    MODLOG_DFLT(INFO, " our_id_addr_type=%d our_id_addr=",
                desc->our_id_addr.type);
    print_addr(desc->our_id_addr.val);
    MODLOG_DFLT(INFO, " peer_ota_addr_type=%d peer_ota_addr=",
                desc->peer_ota_addr.type);
    print_addr(desc->peer_ota_addr.val);
    MODLOG_DFLT(INFO, " peer_id_addr_type=%d peer_id_addr=",
                desc->peer_id_addr.type);
    print_addr(desc->peer_id_addr.val);
    MODLOG_DFLT(INFO, " conn_itvl=%d conn_latency=%d supervision_timeout=%d "
                "encrypted=%d authenticated=%d bonded=%d\n",
                desc->conn_itvl, desc->conn_latency,
                desc->supervision_timeout,
                desc->sec_state.encrypted,
                desc->sec_state.authenticated,
                desc->sec_state.bonded);
}
#endif

#if CONFIG_EXAMPLE_EXTENDED_ADV
/**
 * Enables advertising with the following parameters:
 *     o General discoverable mode.
 *     o Undirected connectable mode.
 */
static void
ext_bleprph_advertise(void)
{
    struct ble_gap_ext_adv_params params;
    struct os_mbuf *data;
    uint8_t instance = 0;
    int rc;

    /* First check if any instance is already active */
    if(ble_gap_ext_adv_active(instance)) {
        return;
    }

    /* use defaults for non-set params */
    memset (&params, 0, sizeof(params));

    /* enable connectable advertising */
    params.connectable = 1;

    /* advertise using random addr */
    params.own_addr_type = BLE_OWN_ADDR_PUBLIC;

    params.primary_phy = BLE_HCI_LE_PHY_1M;
    params.secondary_phy = BLE_HCI_LE_PHY_2M;
    params.tx_power = 127;
    params.sid = 1;

    params.itvl_min = BLE_GAP_ADV_FAST_INTERVAL1_MIN;
    params.itvl_max = BLE_GAP_ADV_FAST_INTERVAL1_MIN;

    /* configure instance 0 */
    rc = ble_gap_ext_adv_configure(instance, &params, NULL,
                                   bleprph_gap_event, NULL);
    assert (rc == 0);

    /* in this case only scan response is allowed */

    /* get mbuf for scan rsp data */
    data = os_msys_get_pkthdr(sizeof(ext_adv_pattern_1), 0);
    assert(data);

    /* fill mbuf with scan rsp data */
    rc = os_mbuf_append(data, ext_adv_pattern_1, sizeof(ext_adv_pattern_1));
    assert(rc == 0);

    rc = ble_gap_ext_adv_set_data(instance, data);
    assert (rc == 0);

    /* start advertising */
    rc = ble_gap_ext_adv_start(instance, 0, 0);
    assert (rc == 0);
    ESP_LOGI(tag, "advertising started (extended)");
}
#else
/**
 * Enables advertising with the following parameters:
 *     o General discoverable mode.
 *     o Undirected connectable mode.
 */
static void
bleprph_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    struct ble_hs_adv_fields rsp_fields;
#if CONFIG_BT_NIMBLE_GAP_SERVICE
    const char *name;
#endif
    int rc;

    /**
     *  Set the advertisement data included in our advertisements:
     *     o Flags (indicates advertisement type and other general info).
     *     o Advertising tx power.
     *     o Device name.
     */

    memset(&fields, 0, sizeof fields);

    /* Advertise two flags:
     *     o Discoverability in forthcoming advertisement (general)
     *     o BLE-only (BR/EDR unsupported).
     */
    fields.flags = BLE_HS_ADV_F_DISC_GEN |
                   BLE_HS_ADV_F_BREDR_UNSUP;

    /* Indicate that the TX power level field should be included; have the
     * stack fill this value automatically.  This is done by assigning the
     * special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
     */
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    fields.uuids128 = &advertised_service_uuid;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;

#if CONFIG_BT_NIMBLE_GAP_SERVICE
    memset(&rsp_fields, 0, sizeof rsp_fields);

    name = ble_svc_gap_device_name();
    rsp_fields.name = (uint8_t *)name;
    rsp_fields.name_len = strlen(name);
    rsp_fields.name_is_complete = 1;
#endif

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error setting advertisement data; rc=%d\n", rc);
        return;
    }

#if CONFIG_BT_NIMBLE_GAP_SERVICE
    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error setting scan response data; rc=%d\n", rc);
        return;
    }
#endif

    /* Begin advertising. */
    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, bleprph_gap_event, NULL);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error enabling advertisement; rc=%d\n", rc);
        return;
    }
    ESP_LOGI(tag, "advertising started");
}
#endif

#if MYNEWT_VAL(BLE_POWER_CONTROL)
static void bleprph_power_control(uint16_t conn_handle)
{
    int rc;

    rc = ble_gap_read_remote_transmit_power_level(conn_handle, 0x01 );  // Attempting on LE 1M phy
    assert (rc == 0);

    rc = ble_gap_set_transmit_power_reporting_enable(conn_handle, 0x1, 0x1);
    assert (rc == 0);
}
#endif

/**
 * The nimble host executes this callback when a GAP event occurs.  The
 * application associates a GAP event callback with each connection that forms.
 * bleprph uses the same callback for all connections.
 *
 * @param event                 The type of event being signalled.
 * @param ctxt                  Various information pertaining to the event.
 * @param arg                   Application-specified argument; unused by
 *                                  bleprph.
 *
 * @return                      0 if the application successfully handled the
 *                                  event; nonzero on failure.  The semantics
 *                                  of the return code is specific to the
 *                                  particular GAP event being signalled.
 */
static int
bleprph_gap_event(struct ble_gap_event *event, void *arg)
{
#if NIMBLE_BLE_CONNECT
    struct ble_gap_conn_desc desc;
    int rc;
#endif
    (void)arg;

    switch (event->type) {

#if NIMBLE_BLE_CONNECT
    case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed. */
        MODLOG_DFLT(INFO, "connection %s; status=%d ",
                    event->connect.status == 0 ? "established" : "failed",
                    event->connect.status);
        if (event->connect.status == 0) {
            rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
            assert(rc == 0);
            bleprph_print_conn_desc(&desc);
            active_conn_handle = event->connect.conn_handle;
            ESP_LOGI(tag, "client connected; conn_handle=%d", active_conn_handle);
            /* Connection parameter update is intentionally deferred to the
             * BLE_GAP_EVENT_SUBSCRIBE handler.  Requesting a very short
             * interval (7.5-15 ms) immediately on connect races with the
             * central's concurrent MTU exchange and service discovery, which
             * can cause Android to terminate the connection (status=22 /
             * GATT_CONN_TERMINATE_LOCAL_HOST).  Waiting until the client has
             * finished GATT setup and written the CCCD avoids this race. */
        }
        MODLOG_DFLT(INFO, "\n");

        if (event->connect.status != 0) {
            /* Connection failed; resume advertising. */
#if CONFIG_EXAMPLE_EXTENDED_ADV
            ext_bleprph_advertise();
#else
            bleprph_advertise();
#endif
        }

#if MYNEWT_VAL(BLE_POWER_CONTROL)
	bleprph_power_control(event->connect.conn_handle);
#endif
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        MODLOG_DFLT(INFO, "disconnect; reason=%d ", event->disconnect.reason);
        bleprph_print_conn_desc(&event->disconnect.conn);
        MODLOG_DFLT(INFO, "\n");
        ESP_LOGI(tag, "client disconnected; reason=%d", event->disconnect.reason);
        active_conn_handle = BLE_HS_CONN_HANDLE_NONE;

        /* Connection terminated; resume advertising. */
#if CONFIG_EXAMPLE_EXTENDED_ADV
        ext_bleprph_advertise();
#else
        bleprph_advertise();
#endif
        return 0;

    case BLE_GAP_EVENT_CONN_UPDATE:
        /* The central has updated the connection parameters. */
        MODLOG_DFLT(INFO, "connection updated; status=%d ",
                    event->conn_update.status);
        rc = ble_gap_conn_find(event->conn_update.conn_handle, &desc);
        assert(rc == 0);
        bleprph_print_conn_desc(&desc);
        MODLOG_DFLT(INFO, "\n");
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        MODLOG_DFLT(INFO, "advertise complete; reason=%d",
                    event->adv_complete.reason);
#if CONFIG_EXAMPLE_EXTENDED_ADV
        ext_bleprph_advertise();
#else
        bleprph_advertise();
#endif
        return 0;

    case BLE_GAP_EVENT_ENC_CHANGE:
        /* Encryption has been enabled or disabled for this connection. */
        MODLOG_DFLT(INFO, "encryption change event; status=%d ",
                    event->enc_change.status);
        rc = ble_gap_conn_find(event->enc_change.conn_handle, &desc);
        assert(rc == 0);
        bleprph_print_conn_desc(&desc);
        MODLOG_DFLT(INFO, "\n");
        return 0;

    case BLE_GAP_EVENT_NOTIFY_TX:
        if (event->notify_tx.status != 0) {
            MODLOG_DFLT(WARN, "notify_tx failed; conn_handle=%d attr_handle=%d "
                        "status=%d is_indication=%d",
                        event->notify_tx.conn_handle,
                        event->notify_tx.attr_handle,
                        event->notify_tx.status,
                        event->notify_tx.indication);
        }
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        MODLOG_DFLT(INFO, "subscribe event; conn_handle=%d attr_handle=%d "
                    "reason=%d prevn=%d curn=%d previ=%d curi=%d\n",
                    event->subscribe.conn_handle,
                    event->subscribe.attr_handle,
                    event->subscribe.reason,
                    event->subscribe.prev_notify,
                    event->subscribe.cur_notify,
                    event->subscribe.prev_indicate,
                    event->subscribe.cur_indicate);
        ESP_LOGI(tag, "subscribe state on handle=%d: notify=%d indicate=%d",
                 event->subscribe.attr_handle,
                 event->subscribe.cur_notify,
                 event->subscribe.cur_indicate);

        /* Now that GATT setup is complete (MTU exchanged, services discovered,
         * CCCD written) it is safe to request a shorter connection interval.
         * itvl_min/max are in BLE units of 1.25 ms.
         *   24 = 30 ms  (conservative but still fast for BLE notifications)
         *   40 = 50 ms
         * Requesting this here, rather than immediately on connect, avoids a
         * race with Android's MTU + service-discovery operations that caused
         * the connection to drop with status=22. */
        if (event->subscribe.cur_notify) {
            struct ble_gap_upd_params conn_params = {
                .itvl_min            = 24,   /* 30 ms */
                .itvl_max            = 40,   /* 50 ms */
                .latency             = 0,
                .supervision_timeout = 400,  /* 4 s   */
                .min_ce_len          = 0,
                .max_ce_len          = 0,
            };
            rc = ble_gap_update_params(event->subscribe.conn_handle, &conn_params);
            if (rc != 0) {
                ESP_LOGW(tag, "conn param update (post-subscribe) failed, rc=%d", rc);
            }
        }
        return 0;

    case BLE_GAP_EVENT_MTU:
        MODLOG_DFLT(INFO, "mtu update event; conn_handle=%d cid=%d mtu=%d\n",
                    event->mtu.conn_handle,
                    event->mtu.channel_id,
                    event->mtu.value);
        return 0;

    case BLE_GAP_EVENT_REPEAT_PAIRING:
        /* We already have a bond with the peer, but it is attempting to
         * establish a new secure link.  This app sacrifices security for
         * convenience: just throw away the old bond and accept the new link.
         */

        /* Delete the old bond. */
        rc = ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
        assert(rc == 0);
        ble_store_util_delete_peer(&desc.peer_id_addr);

        /* Return BLE_GAP_REPEAT_PAIRING_RETRY to indicate that the host should
         * continue with the pairing operation.
         */
        return BLE_GAP_REPEAT_PAIRING_RETRY;

    case BLE_GAP_EVENT_AUTHORIZE:
        MODLOG_DFLT(INFO, "authorize event: conn_handle=%d attr_handle=%d is_read=%d",
                    event->authorize.conn_handle,
                    event->authorize.attr_handle,
                    event->authorize.is_read);

        /* Keep test setup permissive: allow reads/writes without extra authorize gate. */
        event->authorize.out_response = BLE_GAP_AUTHORIZE_ACCEPT;
        return 0;

#if MYNEWT_VAL(BLE_POWER_CONTROL)
    case BLE_GAP_EVENT_TRANSMIT_POWER:
        MODLOG_DFLT(INFO, "Transmit power event : status=%d conn_handle=%d reason=%d "
                           "phy=%d power_level=%x power_level_flag=%d delta=%d",
                     event->transmit_power.status,
                     event->transmit_power.conn_handle,
                     event->transmit_power.reason,
                     event->transmit_power.phy,
                     event->transmit_power.transmit_power_level,
                     event->transmit_power.transmit_power_level_flag,
                     event->transmit_power.delta);
        return 0;

    case BLE_GAP_EVENT_PATHLOSS_THRESHOLD:
        MODLOG_DFLT(INFO, "Pathloss threshold event : conn_handle=%d current path loss=%d "
                           "zone_entered =%d",
                     event->pathloss_threshold.conn_handle,
                     event->pathloss_threshold.current_path_loss,
                     event->pathloss_threshold.zone_entered);
        return 0;
#endif

#if MYNEWT_VAL(BLE_EATT_CHAN_NUM) > 0
    case BLE_GAP_EVENT_EATT:
        MODLOG_DFLT(INFO, "EATT %s : conn_handle=%d cid=%d",
                event->eatt.status ? "disconnected" : "connected",
                event->eatt.conn_handle,
                event->eatt.cid);
	if (event->eatt.status) {
		/* Abort if disconnected */
		return 0;
	}
	cids[bearers] = event->eatt.cid;
	bearers += 1;
	if (bearers != MYNEWT_VAL(BLE_EATT_CHAN_NUM)) {
		/* Wait until all EATT bearers are connected before proceeding */
		return 0;
	}
	/* Set the default bearer to use for further procedures */
	rc = ble_att_set_default_bearer_using_cid(event->eatt.conn_handle, cids[0]);
	if (rc != 0) {
		MODLOG_DFLT(INFO, "Cannot set default EATT bearer, rc = %d\n", rc);
		return rc;
	}

	return 0;
#endif

#if MYNEWT_VAL(BLE_CONN_SUBRATING)
    case BLE_GAP_EVENT_SUBRATE_CHANGE:
        MODLOG_DFLT(INFO, "Subrate change event : conn_handle=%d status=%d factor=%d",
                    event->subrate_change.conn_handle,
                    event->subrate_change.status,
                    event->subrate_change.subrate_factor);
        return 0;
#endif
#endif
    }
    return 0;
}

static void
bleprph_on_reset(int reason)
{
    MODLOG_DFLT(ERROR, "Resetting state; reason=%d\n", reason);
}

#if CONFIG_EXAMPLE_RANDOM_ADDR
static void
ble_app_set_addr(void)
{
    ble_addr_t addr;
    int rc;

    /* generate new non-resolvable private address */
    rc = ble_hs_id_gen_rnd(0, &addr);
    assert(rc == 0);

    /* set generated address */
    rc = ble_hs_id_set_rnd(addr.val);

    assert(rc == 0);
}
#endif

static void
bleprph_on_sync(void)
{
    int rc;

#if CONFIG_EXAMPLE_RANDOM_ADDR
    /* Generate a non-resolvable private address. */
    ble_app_set_addr();
#endif

    /* Make sure we have proper identity address set (public preferred) */
#if CONFIG_EXAMPLE_RANDOM_ADDR
    rc = ble_hs_util_ensure_addr(1);
#else
    rc = ble_hs_util_ensure_addr(0);
#endif
    assert(rc == 0);

    /* Figure out address to use while advertising (no privacy for now) */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error determining address type; rc=%d\n", rc);
        return;
    }

    /* Printing ADDR */
    uint8_t addr_val[6] = {0};
    rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);

    MODLOG_DFLT(INFO, "Device Address: ");
    print_addr(addr_val);
    MODLOG_DFLT(INFO, "\n");
    /* Begin advertising. */
#if CONFIG_EXAMPLE_EXTENDED_ADV
    ext_bleprph_advertise();
#else
    bleprph_advertise();
#endif
}

void bleprph_host_task(void *param)
{
    ESP_LOGI(tag, "BLE Host Task Started");
    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();

    nimble_port_freertos_deinit();
}

void
app_main(void)
{
    int rc;
    bool hci_inited = false;

    spi_custom_tx_queue = xQueueCreate(SPI_CUSTOM_TX_QUEUE_LEN, sizeof(SpiQueuedPacket_t));
    if (spi_custom_tx_queue == NULL) {
        ESP_LOGE(tag, "failed to create spi custom tx queue");
        return;
    }
    ble_binary_frame_queue = xQueueCreate(BLE_NOTIFY_BINARY_QUEUE_LEN, sizeof(N2K_RawFrame_t));
    if (ble_binary_frame_queue == NULL) {
        ESP_LOGE(tag, "failed to create binary frame queue");
        return;
    }

    /* Initialize NVS - it is used to store PHY calibration data */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Best effort: reclaim Classic BT memory for NimBLE-only application. */
    ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(tag, "Classic BT memory release skipped: %s", esp_err_to_name(ret));
    }

    ret = esp_nimble_hci_init();
    if (ret == ESP_OK) {
        hci_inited = true;
    } else {
        /* Some environments initialize controller differently; continue with host init fallback. */
        ESP_LOGW(tag, "NimBLE HCI init failed (%s), trying host init fallback", esp_err_to_name(ret));
    }

    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(tag, "Failed to init nimble %d ", ret);
        if (hci_inited) {
            esp_nimble_hci_deinit();
        }
        return;
    }
    /* Initialize the NimBLE host configuration. */
    ble_hs_cfg.reset_cb = bleprph_on_reset;
    ble_hs_cfg.sync_cb = bleprph_on_sync;
    ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    /* Keep pairing/security simple for easy phone testing. */
    ble_hs_cfg.sm_io_cap = BLE_HS_IO_NO_INPUT_OUTPUT;
    ble_hs_cfg.sm_bonding = 0;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 0;
    ble_hs_cfg.sm_our_key_dist = 0;
    ble_hs_cfg.sm_their_key_dist = 0;

#if MYNEWT_VAL(BLE_GATTS)
    rc = gatt_svr_init();
    assert(rc == 0);
    gatt_svr_set_command_handler(ble_handle_command_line, NULL);
#endif

#if CONFIG_BT_NIMBLE_GAP_SERVICE
    /* Set the default device name. */
    rc = ble_svc_gap_device_name_set(BLE_DEVICE_ADV_NAME);
    assert(rc == 0);
#endif

    /* Need to have template for store */
    ble_store_config_init();

    nimble_port_freertos_init(bleprph_host_task);

#if MYNEWT_VAL(BLE_EATT_CHAN_NUM) > 0
    bearers = 0;
    for (int i = 0; i < MYNEWT_VAL(BLE_EATT_CHAN_NUM); i++) {
        cids[i] = 0;
    }
#endif

    /* Stack kept larger to safely format and send multiline BLE text snapshots. */
    xTaskCreate(ble_test_notify_task, "ble_test_notify", BLE_NOTIFY_TASK_STACK_SIZE, NULL, 5, NULL);
    xTaskCreate(spi_n2k_task, "spi_n2k_task", SPI_N2K_TASK_STACK_SIZE, NULL, 6, NULL);
}
