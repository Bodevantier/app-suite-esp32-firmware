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
#include <time.h>
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/spi_slave.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "cJSON.h"
/* BLE */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "bleprph.h"
#include "n2k_app_model.h"
#include "n2k_can_id.h"
#include "n2k_decoder.h"
#include "n2k_raw_frame.h"
#include "n2k_request_manager.h"
#include "spi_n2k_transport.h"

#define SPI_SLAVE_HOST   SPI2_HOST
#define PIN_NUM_MOSI     11
#define PIN_NUM_MISO     13
#define PIN_NUM_SCLK     12
#define PIN_NUM_CS       10
#define SPI_RX_BUF_SIZE  256
#define N2K_TX_QUEUE_LEN 16
#define N2K_LOCAL_SOURCE 0x31u
#define N2K_PGN_ADDRESS_CLAIM 60928u
#define N2K_PGN_SUPPORTED_PGN_LIST 126464u
#define N2K_PGN_PRODUCT_INFO 126996u
#define N2K_PGN_CONFIGURATION_INFO 126998u
#define N2K_REQUEST_GAP_MS 250u
#define N2K_LOCAL_ADDRESS_CLAIM_RETRY_MS 2000u
#define N2K_GATEWAY_DEVICE_NAME "BLE Gateway"
#define N2K_GATEWAY_MODEL_NAME "ESP32 SPI N2K BLE Gateway"
#define N2K_GATEWAY_CATEGORY "gateway"
#define N2K_INITIAL_DISCOVERY_DELAY_MS 1000u
#define N2K_INITIAL_DISCOVERY_RETRY_MS 1000u
#define N2K_INFO_REQUEST_RETRY_MS 5000u
#define N2K_GLOBAL_INFO_REQUEST_RETRY_MS 1500u
#define N2K_PGN_LIST_REQUEST_RETRY_MS 10000u
#define N2K_MAX_INFO_REQUESTS 4u
#define N2K_MAX_PGN_LIST_REQUESTS 2u
#define BLE_NOTIFY_TEXT_MAX_LEN 180
#define BLE_NOTIFY_TASK_STACK_SIZE 8192
#define BLE_NOTIFY_TASK_IDLE_DELAY_MS 1000u
#define BLE_NOTIFY_TASK_REFRESH_POLL_MS 200u
#define BLE_DEVICE_LIST_REFRESH_WINDOW_MS 3000u
#define BLE_DEVICE_LIST_REFRESH_MAX_WINDOW_MS 6000u
#define N2K_LIVE_WIND_FRESH_MS 3000u
#define N2K_DEVICE_ONLINE_MS 10000u
#define N2K_DEVICE_STALE_HIDE_MS 120000u

#if CONFIG_EXAMPLE_EXTENDED_ADV
static uint8_t ext_adv_pattern_1[] = {
    0x02, BLE_HS_ADV_TYPE_FLAGS, 0x06,
    0x03, BLE_HS_ADV_TYPE_COMP_UUIDS16, 0xab, 0xcd,
    0x03, BLE_HS_ADV_TYPE_COMP_UUIDS16, 0x18, 0x11,
    0x11, BLE_HS_ADV_TYPE_COMP_NAME, 'n', 'i', 'm', 'b', 'l', 'e', '-', 'b', 'l', 'e', 'p', 'r', 'p', 'h', '-', 'e',
};
#endif

static const char *tag = "NimBLE_BLE_PRPH";
static int bleprph_gap_event(struct ble_gap_event *event, void *arg);
static bool n2k_enqueue_tx_frame(const N2K_RawFrame_t *frame, void *ctx);
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

static SemaphoreHandle_t n2k_text_lock;
static SemaphoreHandle_t n2k_model_lock;
static char latest_n2k_text[BLE_NOTIFY_TEXT_MAX_LEN] = "waiting for N2K data";
static char ble_notify_task_payload[BLE_NOTIFY_TEXT_MAX_LEN];
static uint32_t latest_n2k_seq = 0;
static QueueHandle_t n2k_tx_queue;
static N2kAppModel_t n2k_app_model;
static N2kDecoder_t n2k_decoder;
static N2kRequestManager_t n2k_request_manager;
static volatile bool ble_device_list_request_pending = false;
static bool ble_device_list_refresh_active = false;
static uint32_t ble_device_list_refresh_start_ms = 0u;
static bool n2k_initial_discovery_request_sent = false;
static uint32_t n2k_last_initial_discovery_attempt_ms = 0u;
static uint32_t n2k_last_request_ms = 0u;
static uint32_t n2k_last_global_product_request_ms = 0u;
static uint32_t n2k_last_global_configuration_request_ms = 0u;
static uint32_t n2k_last_global_pgn_list_request_ms = 0u;
static bool n2k_local_address_claim_sent = false;
static uint32_t n2k_last_local_address_claim_ms = 0u;
static struct {
    char name_buf[48];
    char model_buf[48];
    char manufacturer_buf[40];
    char last_seen_buf[32];
} ble_device_list_meta_scratch;

static bool json_string_contains_token_ci(const char *text, const char *token)
{
    size_t token_len;
    if ((text == NULL) || (token == NULL)) {
        return false;
    }
    token_len = strlen(token);
    if (token_len == 0u) {
        return false;
    }
    for (const char *p = text; *p != '\0'; ++p) {
        size_t i = 0u;
        while ((i < token_len) && (p[i] != '\0')) {
            char a = p[i];
            char b = token[i];
            if (a >= 'A' && a <= 'Z') {
                a = (char)(a - 'A' + 'a');
            }
            if (b >= 'A' && b <= 'Z') {
                b = (char)(b - 'A' + 'a');
            }
            if (a != b) {
                break;
            }
            i++;
        }
        if (i == token_len) {
            return true;
        }
    }
    return false;
}

static bool n2k_text_available(const char *text)
{
    return (text != NULL) && (text[0] != '\0');
}

static bool n2k_metadata_contains_keywords(const N2kDeviceEntry_t *entry,
                                           const char *const *keywords,
                                           size_t keyword_count,
                                           const char **matched_field)
{
    if ((entry == NULL) || (keywords == NULL)) {
        return false;
    }

    for (size_t i = 0; i < keyword_count; i++) {
        const char *keyword = keywords[i];
        if (json_string_contains_token_ci(entry->installation_desc1, keyword)) {
            if (matched_field != NULL) {
                *matched_field = "installation_desc1";
            }
            return true;
        }
        if (json_string_contains_token_ci(entry->installation_desc2, keyword)) {
            if (matched_field != NULL) {
                *matched_field = "installation_desc2";
            }
            return true;
        }
        if (json_string_contains_token_ci(entry->model, keyword)) {
            if (matched_field != NULL) {
                *matched_field = "model";
            }
            return true;
        }
        if (json_string_contains_token_ci(entry->model_id, keyword)) {
            if (matched_field != NULL) {
                *matched_field = "model_id";
            }
            return true;
        }
        if (json_string_contains_token_ci(entry->software_version, keyword)) {
            if (matched_field != NULL) {
                *matched_field = "software_version";
            }
            return true;
        }
    }

    return false;
}

static const char *n2k_guess_category(const N2kDeviceEntry_t *entry, const char **selected_source)
{
    static const char *const wind_keywords[] = {"wind", "anemometer"};
    static const char *const tank_keywords[] = {"tank", "fluid", "level"};
    static const char *const battery_keywords[] = {"battery", "charger", "alternator", "voltage", "current"};
    static const char *const temperature_keywords[] = {"temperature", "temp", "thermo"};
    static const char *const gps_keywords[] = {"gps", "gnss", "position", "ais"};
    static const char *const logger_keywords[] = {"logger", "logging", "record", "history", "log"};
    static const char *const generic_keywords[] = {"gateway", "bridge", "adapter", "interface", "display", "monitor", "controller", "node"};
    const char *matched_field = NULL;

    if (entry == NULL) {
        if (selected_source != NULL) {
            *selected_source = "none";
        }
        return "unknown";
    }

    if (n2k_metadata_contains_keywords(entry, wind_keywords, sizeof(wind_keywords) / sizeof(wind_keywords[0]), &matched_field)) {
        if (selected_source != NULL) {
            *selected_source = matched_field;
        }
        return "wind";
    }
    if (n2k_metadata_contains_keywords(entry, tank_keywords, sizeof(tank_keywords) / sizeof(tank_keywords[0]), &matched_field)) {
        if (selected_source != NULL) {
            *selected_source = matched_field;
        }
        return "tank";
    }
    if (n2k_metadata_contains_keywords(entry, battery_keywords, sizeof(battery_keywords) / sizeof(battery_keywords[0]), &matched_field)) {
        if (selected_source != NULL) {
            *selected_source = matched_field;
        }
        return "battery";
    }
    if (n2k_metadata_contains_keywords(entry, temperature_keywords, sizeof(temperature_keywords) / sizeof(temperature_keywords[0]), &matched_field)) {
        if (selected_source != NULL) {
            *selected_source = matched_field;
        }
        return "temperature";
    }
    if (n2k_metadata_contains_keywords(entry, gps_keywords, sizeof(gps_keywords) / sizeof(gps_keywords[0]), &matched_field)) {
        if (selected_source != NULL) {
            *selected_source = matched_field;
        }
        return "gps";
    }
    if (n2k_metadata_contains_keywords(entry, logger_keywords, sizeof(logger_keywords) / sizeof(logger_keywords[0]), &matched_field)) {
        if (selected_source != NULL) {
            *selected_source = matched_field;
        }
        return "logger";
    }
    if (n2k_metadata_contains_keywords(entry, generic_keywords, sizeof(generic_keywords) / sizeof(generic_keywords[0]), &matched_field)) {
        if (selected_source != NULL) {
            *selected_source = matched_field;
        }
        return "generic";
    }
    if (selected_source != NULL) {
        *selected_source = "no metadata match";
    }
    return "unknown";
}

static const char *n2k_best_device_name(const N2kDeviceEntry_t *entry, char *fallback_buf, size_t fallback_buf_len, const char **selected_source)
{
    if (entry == NULL) {
        if (selected_source != NULL) {
            *selected_source = "none";
        }
        return "Unknown device";
    }
    if (n2k_text_available(entry->installation_desc1)) {
        if (selected_source != NULL) {
            *selected_source = "installation_desc1";
        }
        return entry->installation_desc1;
    }
    if (n2k_text_available(entry->installation_desc2)) {
        if (selected_source != NULL) {
            *selected_source = "installation_desc2";
        }
        return entry->installation_desc2;
    }
    if (n2k_text_available(entry->model_id)) {
        if (selected_source != NULL) {
            *selected_source = "model_id";
        }
        return entry->model_id;
    }
    if (n2k_text_available(entry->model)) {
        if (selected_source != NULL) {
            *selected_source = "model";
        }
        return entry->model;
    }
    if ((fallback_buf != NULL) && (fallback_buf_len > 0u) && entry->source <= 253u) {
        snprintf(fallback_buf, fallback_buf_len, "N2K node src %u", (unsigned)entry->source);
        if (selected_source != NULL) {
            *selected_source = "source fallback";
        }
        return fallback_buf;
    }
    if (selected_source != NULL) {
        *selected_source = "unknown fallback";
    }
    return "Unknown device";
}

static const char *n2k_best_model(const N2kDeviceEntry_t *entry, char *fallback_buf, size_t fallback_buf_len, const char **selected_source)
{
    if ((entry != NULL) && n2k_text_available(entry->model_id)) {
        if (selected_source != NULL) {
            *selected_source = "model_id";
        }
        return entry->model_id;
    }
    if ((entry != NULL) && n2k_text_available(entry->model)) {
        if (selected_source != NULL) {
            *selected_source = "model";
        }
        return entry->model;
    }
    if ((entry != NULL) && n2k_text_available(entry->software_version)) {
        if ((fallback_buf != NULL) && (fallback_buf_len > 0u)) {
            snprintf(fallback_buf, fallback_buf_len, "SW %s", entry->software_version);
            if (selected_source != NULL) {
                *selected_source = "software_version";
            }
            return fallback_buf;
        }
        if (selected_source != NULL) {
            *selected_source = "software_version";
        }
        return entry->software_version;
    }
    if ((entry != NULL) && (entry->product_code != 0u) &&
        (fallback_buf != NULL) && (fallback_buf_len > 0u)) {
        snprintf(fallback_buf, fallback_buf_len, "Product code %u", (unsigned)entry->product_code);
        if (selected_source != NULL) {
            *selected_source = "product_code";
        }
        return fallback_buf;
    }
    if (selected_source != NULL) {
        *selected_source = "unknown fallback";
    }
    return "Unknown model";
}

static const char *n2k_best_manufacturer(const N2kDeviceEntry_t *entry, char *buf, size_t buf_len, const char **selected_source)
{
    if ((entry != NULL) && n2k_text_available(entry->manufacturer_text)) {
        if (selected_source != NULL) {
            *selected_source = "manufacturer_text";
        }
        return entry->manufacturer_text;
    }
    if ((buf == NULL) || (buf_len == 0u)) {
        if (selected_source != NULL) {
            *selected_source = "invalid buffer";
        }
        return "Unknown manufacturer";
    }
    if ((entry != NULL) && entry->manufacturer_code != 0u) {
        snprintf(buf, buf_len, "Manufacturer code %u", (unsigned)entry->manufacturer_code);
        if (selected_source != NULL) {
            *selected_source = "manufacturer_code";
        }
        return buf;
    }
    snprintf(buf, buf_len, "Unknown manufacturer");
    if (selected_source != NULL) {
        *selected_source = "unknown fallback";
    }
    return buf;
}

static bool n2k_format_last_seen_utc(const N2kDeviceEntry_t *entry, char *buf, size_t buf_len)
{
    time_t now_epoch;
    uint32_t now_ms;
    uint32_t age_ms;
    time_t seen_epoch;
    struct tm seen_tm;

    if ((entry == NULL) || (buf == NULL) || (buf_len == 0u) || (entry->last_seen_ms == 0u)) {
        return false;
    }

    now_epoch = time(NULL);
    if (now_epoch < 1700000000) {
        return false;
    }

    now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    age_ms = (now_ms >= entry->last_seen_ms) ? (now_ms - entry->last_seen_ms) : 0u;
    seen_epoch = now_epoch - (time_t)(age_ms / 1000u);
    if (gmtime_r(&seen_epoch, &seen_tm) == NULL) {
        return false;
    }

    return strftime(buf, buf_len, "%Y-%m-%dT%H:%M:%SZ", &seen_tm) > 0u;
}

static bool n2k_get_live_wind_source(uint8_t *source_out)
{
    bool valid = false;
    uint32_t now_ms;

    if ((source_out == NULL) || (n2k_model_lock == NULL)) {
        return false;
    }
    if (xSemaphoreTake(n2k_model_lock, portMAX_DELAY) != pdTRUE) {
        return false;
    }

    now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    if (n2k_app_model.wind.valid &&
        (n2k_app_model.wind.updated_ms != 0u) &&
        ((uint32_t)(now_ms - n2k_app_model.wind.updated_ms) <= N2K_LIVE_WIND_FRESH_MS)) {
        *source_out = n2k_app_model.wind.source;
        valid = true;
    }

    xSemaphoreGive(n2k_model_lock);
    return valid;
}

static bool n2k_is_remote_entry(const N2kDeviceEntry_t *entry)
{
    return (entry != NULL) && entry->used && (entry->source != N2K_LOCAL_SOURCE);
}

static bool n2k_entry_old_enough_for_request(const N2kDeviceEntry_t *entry, uint32_t now_ms)
{
    if ((entry == NULL) || (entry->first_seen_ms == 0u)) {
        return false;
    }
    return (uint32_t)(now_ms - entry->first_seen_ms) >= N2K_INITIAL_DISCOVERY_DELAY_MS;
}

static bool n2k_entry_seen_within_window(const N2kDeviceEntry_t *entry, uint32_t now_ms, uint32_t window_ms)
{
    if ((entry == NULL) || (entry->last_seen_ms == 0u)) {
        return false;
    }
    return (uint32_t)(now_ms - entry->last_seen_ms) <= window_ms;
}

static bool n2k_request_retry_due(uint32_t now_ms, uint32_t last_request_ms, uint32_t retry_ms)
{
    return (last_request_ms == 0u) || ((uint32_t)(now_ms - last_request_ms) >= retry_ms);
}

static bool n2k_request_gap_elapsed(uint32_t now_ms)
{
    return (n2k_last_request_ms == 0u) || ((uint32_t)(now_ms - n2k_last_request_ms) >= N2K_REQUEST_GAP_MS);
}

static bool n2k_entry_has_useful_metadata(const N2kDeviceEntry_t *entry)
{
    return (entry != NULL) &&
           entry->has_address_claim &&
           entry->has_product_info &&
           entry->has_configuration_info;
}

static bool n2k_entry_needs_pgn_list_enrichment(const N2kDeviceEntry_t *entry)
{
    return (entry != NULL) &&
           (!entry->has_tx_pgn_list || !entry->has_rx_pgn_list);
}

static void n2k_add_supported_pgns_json(cJSON *supported_pgns, const N2kDeviceEntry_t *entry)
{
    if ((supported_pgns == NULL) || (entry == NULL)) {
        return;
    }

    for (uint8_t i = 0u; i < entry->supported_pgn_count; i++) {
        cJSON_AddItemToArray(supported_pgns, cJSON_CreateNumber((double)entry->supported_pgns[i]));
    }
}

static void n2k_collect_refresh_status(size_t *tracked_devices,
                                       size_t *useful_incomplete_devices,
                                       size_t *pgn_enrichment_pending_devices)
{
    size_t tracked = 0u;
    size_t useful_incomplete = 0u;
    size_t pgn_pending = 0u;

    if ((n2k_model_lock == NULL) || (xSemaphoreTake(n2k_model_lock, portMAX_DELAY) != pdTRUE)) {
        if (tracked_devices != NULL) {
            *tracked_devices = 0u;
        }
        if (useful_incomplete_devices != NULL) {
            *useful_incomplete_devices = 0u;
        }
        if (pgn_enrichment_pending_devices != NULL) {
            *pgn_enrichment_pending_devices = 0u;
        }
        return;
    }

    for (size_t i = 0; i < N2K_DEVICE_MANAGER_MAX_DEVICES; i++) {
        const N2kDeviceEntry_t *entry = &n2k_app_model.devices.entries[i];
        if (!n2k_is_remote_entry(entry)) {
            continue;
        }

        tracked++;
        if (!n2k_entry_has_useful_metadata(entry)) {
            useful_incomplete++;
        }
        if (n2k_entry_needs_pgn_list_enrichment(entry)) {
            pgn_pending++;
        }
    }

    xSemaphoreGive(n2k_model_lock);

    if (tracked_devices != NULL) {
        *tracked_devices = tracked;
    }
    if (useful_incomplete_devices != NULL) {
        *useful_incomplete_devices = useful_incomplete;
    }
    if (pgn_enrichment_pending_devices != NULL) {
        *pgn_enrichment_pending_devices = pgn_pending;
    }
}
static bool n2k_send_request(uint8_t destination,
                             uint32_t requested_pgn,
                             uint32_t now_ms)
{
    bool result = n2k_request_manager_send_request(&n2k_request_manager, destination, requested_pgn, now_ms);

    if (result) {
        n2k_last_request_ms = now_ms;
    }
    return result;
}

static uint64_t n2k_local_name_value(void)
{
    const uint64_t unique_number = 0x12345u;
    const uint64_t manufacturer_code = 2046u;
    const uint64_t device_instance_lower = 0u;
    const uint64_t device_instance_upper = 0u;
    const uint64_t device_function = 130u;
    const uint64_t device_class = 85u;
    const uint64_t system_instance = 0u;
    const uint64_t industry_group = 4u;
    const uint64_t arbitrary_address_capable = 1u;

    return (unique_number & 0x1FFFFFu)
           | ((manufacturer_code & 0x7FFu) << 21u)
           | ((device_instance_lower & 0x07u) << 32u)
           | ((device_instance_upper & 0x1Fu) << 35u)
           | ((device_function & 0xFFu) << 40u)
           | ((device_class & 0x7Fu) << 49u)
           | ((system_instance & 0x0Fu) << 56u)
           | ((industry_group & 0x07u) << 60u)
           | ((arbitrary_address_capable & 0x01u) << 63u);
}

static bool n2k_send_local_address_claim(uint32_t now_ms, const char *reason)
{
    N2K_RawFrame_t frame;
    uint32_t can_id = 0u;
    uint64_t name = n2k_local_name_value();
    bool result;

    if (!n2k_can_id_build_29bit(6u, N2K_PGN_ADDRESS_CLAIM, N2K_LOCAL_SOURCE, 0xFFu, &can_id)) {
        return false;
    }

    n2k_raw_frame_clear(&frame);
    frame.timestamp_ms = now_ms;
    frame.can_id = can_id;
    frame.dlc = 8u;
    frame.flags = N2K_RAW_FLAG_EXTENDED_ID;
    for (uint8_t i = 0u; i < 8u; i++) {
        frame.data[i] = (uint8_t)((name >> (8u * i)) & 0xFFu);
    }

    result = n2k_enqueue_tx_frame(&frame, n2k_tx_queue);
    if (result) {
        n2k_last_request_ms = now_ms;
        n2k_last_local_address_claim_ms = now_ms;
        n2k_local_address_claim_sent = true;
    }
    return result;
}

static bool n2k_ensure_local_address_claim(uint32_t now_ms, const char *reason)
{
    if (!n2k_request_gap_elapsed(now_ms)) {
        return false;
    }

    if (!n2k_local_address_claim_sent ||
        n2k_request_retry_due(now_ms, n2k_last_local_address_claim_ms, N2K_LOCAL_ADDRESS_CLAIM_RETRY_MS)) {
        return n2k_send_local_address_claim(now_ms, reason);
    }

    return false;
}

static bool n2k_request_initial_discovery_if_needed(uint32_t now_ms)
{
    if (n2k_ensure_local_address_claim(now_ms, "announce local source before discovery")) {
        return true;
    }

    if (n2k_initial_discovery_request_sent || !n2k_request_gap_elapsed(now_ms)) {
        return false;
    }
    if (now_ms < N2K_INITIAL_DISCOVERY_DELAY_MS) {
        return false;
    }
    if ((n2k_last_initial_discovery_attempt_ms != 0u) &&
        ((uint32_t)(now_ms - n2k_last_initial_discovery_attempt_ms) < N2K_INITIAL_DISCOVERY_RETRY_MS)) {
        return false;
    }

    n2k_last_initial_discovery_attempt_ms = now_ms;
    if (n2k_send_request(0xffu,
                         N2K_PGN_ADDRESS_CLAIM,
                         now_ms)) {
        n2k_initial_discovery_request_sent = true;
        return true;
    }

    return false;
}

static bool n2k_request_next_pending_metadata(uint32_t now_ms)
{
    bool product_pending = false;
    bool config_pending = false;
    bool pgn_list_pending = false;

    if (n2k_ensure_local_address_claim(now_ms, "announce local source before metadata request")) {
        return true;
    }

    if ((n2k_model_lock == NULL) || !n2k_request_gap_elapsed(now_ms)) {
        return false;
    }
    if (xSemaphoreTake(n2k_model_lock, portMAX_DELAY) != pdTRUE) {
        return false;
    }

    for (size_t i = 0; i < N2K_DEVICE_MANAGER_MAX_DEVICES; i++) {
        N2kDeviceEntry_t *entry = &n2k_app_model.devices.entries[i];
        if (!n2k_is_remote_entry(entry) || !n2k_entry_old_enough_for_request(entry, now_ms)) {
            continue;
        }
        if (!entry->has_address_claim &&
            (entry->address_claim_request_count < N2K_MAX_INFO_REQUESTS) &&
            n2k_request_retry_due(now_ms, entry->last_address_claim_request_ms, N2K_INFO_REQUEST_RETRY_MS)) {
            if (n2k_send_request(entry->source,
                                 N2K_PGN_ADDRESS_CLAIM,
                                 now_ms)) {
                entry->last_address_claim_request_ms = now_ms;
                entry->address_claim_request_count++;
                xSemaphoreGive(n2k_model_lock);
                return true;
            }
        }
    }

    for (size_t i = 0; i < N2K_DEVICE_MANAGER_MAX_DEVICES; i++) {
        N2kDeviceEntry_t *entry = &n2k_app_model.devices.entries[i];
        if (!n2k_is_remote_entry(entry) || !n2k_entry_old_enough_for_request(entry, now_ms)) {
            continue;
        }
        if (!entry->has_product_info && (entry->product_request_count < N2K_MAX_INFO_REQUESTS)) {
            if (n2k_request_retry_due(now_ms, entry->last_product_request_ms, N2K_INFO_REQUEST_RETRY_MS)) {
                if (n2k_send_request(entry->source,
                                     N2K_PGN_PRODUCT_INFO,
                                     now_ms)) {
                    entry->last_product_request_ms = now_ms;
                    entry->product_request_count++;
                    xSemaphoreGive(n2k_model_lock);
                    return true;
                }
            }
            product_pending = true;
        }
    }
    if (product_pending) {
        if (n2k_request_retry_due(now_ms,
                                  n2k_last_global_product_request_ms,
                                  N2K_GLOBAL_INFO_REQUEST_RETRY_MS) &&
            n2k_send_request(0xffu,
                             N2K_PGN_PRODUCT_INFO,
                             now_ms)) {
            n2k_last_global_product_request_ms = now_ms;
            xSemaphoreGive(n2k_model_lock);
            return true;
        }
        xSemaphoreGive(n2k_model_lock);
        return false;
    }

    for (size_t i = 0; i < N2K_DEVICE_MANAGER_MAX_DEVICES; i++) {
        N2kDeviceEntry_t *entry = &n2k_app_model.devices.entries[i];
        if (!n2k_is_remote_entry(entry) || !n2k_entry_old_enough_for_request(entry, now_ms)) {
            continue;
        }
        if (!entry->has_configuration_info && (entry->configuration_request_count < N2K_MAX_INFO_REQUESTS)) {
            if (n2k_request_retry_due(now_ms, entry->last_configuration_request_ms, N2K_INFO_REQUEST_RETRY_MS)) {
                if (n2k_send_request(entry->source,
                                     N2K_PGN_CONFIGURATION_INFO,
                                     now_ms)) {
                    entry->last_configuration_request_ms = now_ms;
                    entry->configuration_request_count++;
                    xSemaphoreGive(n2k_model_lock);
                    return true;
                }
            }
            config_pending = true;
        }
    }
    if (config_pending) {
        if (n2k_request_retry_due(now_ms,
                                  n2k_last_global_configuration_request_ms,
                                  N2K_GLOBAL_INFO_REQUEST_RETRY_MS) &&
            n2k_send_request(0xffu,
                             N2K_PGN_CONFIGURATION_INFO,
                             now_ms)) {
            n2k_last_global_configuration_request_ms = now_ms;
            xSemaphoreGive(n2k_model_lock);
            return true;
        }
        xSemaphoreGive(n2k_model_lock);
        return false;
    }

    for (size_t i = 0; i < N2K_DEVICE_MANAGER_MAX_DEVICES; i++) {
        N2kDeviceEntry_t *entry = &n2k_app_model.devices.entries[i];
        if (!n2k_is_remote_entry(entry) || !n2k_entry_old_enough_for_request(entry, now_ms)) {
            continue;
        }
        if ((!entry->has_tx_pgn_list || !entry->has_rx_pgn_list) &&
            (entry->pgn_list_request_count < N2K_MAX_PGN_LIST_REQUESTS) &&
            n2k_request_retry_due(now_ms, entry->last_pgn_list_request_ms, N2K_PGN_LIST_REQUEST_RETRY_MS)) {
            if (n2k_send_request(entry->source,
                                 N2K_PGN_SUPPORTED_PGN_LIST,
                                 now_ms)) {
                entry->last_pgn_list_request_ms = now_ms;
                entry->pgn_list_request_count++;
                xSemaphoreGive(n2k_model_lock);
                return true;
            }
        }
        if (!entry->has_tx_pgn_list || !entry->has_rx_pgn_list) {
            pgn_list_pending = true;
        }
    }

    if (pgn_list_pending &&
        n2k_request_retry_due(now_ms,
                              n2k_last_global_pgn_list_request_ms,
                              N2K_PGN_LIST_REQUEST_RETRY_MS) &&
        n2k_send_request(0xffu,
                         N2K_PGN_SUPPORTED_PGN_LIST,
                         now_ms)) {
        n2k_last_global_pgn_list_request_ms = now_ms;
        xSemaphoreGive(n2k_model_lock);
        return true;
    }

    xSemaphoreGive(n2k_model_lock);
    return false;
}

static size_t n2k_copy_device_snapshot(N2kDeviceEntry_t *dst, size_t max_count)
{
    size_t copied = 0u;
    uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

    if ((dst == NULL) || (max_count == 0u) || (n2k_model_lock == NULL)) {
        return 0u;
    }
    if (xSemaphoreTake(n2k_model_lock, portMAX_DELAY) != pdTRUE) {
        return 0u;
    }

    {
        const N2kDeviceEntry_t *entries = n2k_device_manager_entries(&n2k_app_model.devices);
        if (entries != NULL) {
            for (size_t i = 0; i < N2K_DEVICE_MANAGER_MAX_DEVICES && copied < max_count; i++) {
                if (!n2k_is_remote_entry(&entries[i])) {
                    continue;
                }
                if (!n2k_entry_seen_within_window(&entries[i], now_ms, N2K_DEVICE_STALE_HIDE_MS)) {
                    continue;
                }
                dst[copied++] = entries[i];
            }
        }
    }

    xSemaphoreGive(n2k_model_lock);
    return copied;
}

static void ble_request_missing_device_metadata(uint32_t *useful_requests_needed,
                                                uint32_t *optional_pgn_requests_needed)
{
    uint32_t useful_needed = 0u;
    uint32_t optional_needed = 0u;

    if (useful_requests_needed != NULL) {
        *useful_requests_needed = 0u;
    }
    if (optional_pgn_requests_needed != NULL) {
        *optional_pgn_requests_needed = 0u;
    }
    n2k_last_global_product_request_ms = 0u;
    n2k_last_global_configuration_request_ms = 0u;
    n2k_last_global_pgn_list_request_ms = 0u;
    n2k_initial_discovery_request_sent = false;
    n2k_last_initial_discovery_attempt_ms = 0u;

    if ((n2k_model_lock == NULL) || (xSemaphoreTake(n2k_model_lock, portMAX_DELAY) != pdTRUE)) {
        return;
    }

    {
        N2kDeviceEntry_t *entries = n2k_app_model.devices.entries;
        if (entries != NULL) {
            for (size_t i = 0; i < N2K_DEVICE_MANAGER_MAX_DEVICES; i++) {
                if (!n2k_is_remote_entry(&entries[i])) {
                    continue;
                }
                if (!entries[i].has_address_claim) {
                    useful_needed++;
                    entries[i].last_address_claim_request_ms = 0u;
                    entries[i].address_claim_request_count = 0u;
                }
                if (!entries[i].has_product_info) {
                    useful_needed++;
                    entries[i].last_product_request_ms = 0u;
                    entries[i].product_request_count = 0u;
                }
                if (!entries[i].has_configuration_info) {
                    useful_needed++;
                    entries[i].last_configuration_request_ms = 0u;
                    entries[i].configuration_request_count = 0u;
                }
                if (!entries[i].has_tx_pgn_list || !entries[i].has_rx_pgn_list) {
                    optional_needed++;
                    entries[i].last_pgn_list_request_ms = 0u;
                    entries[i].pgn_list_request_count = 0u;
                }
            }
        }
    }

    xSemaphoreGive(n2k_model_lock);
    if (useful_requests_needed != NULL) {
        *useful_requests_needed = useful_needed;
    }
    if (optional_pgn_requests_needed != NULL) {
        *optional_pgn_requests_needed = optional_needed;
    }
}

static void ble_add_gateway_device_json(cJSON *devices)
{
    cJSON *device;
    cJSON *supported_pgns;

    if (devices == NULL) {
        return;
    }

    device = cJSON_CreateObject();
    supported_pgns = cJSON_CreateArray();
    if ((device == NULL) || (supported_pgns == NULL)) {
        cJSON_Delete(device);
        cJSON_Delete(supported_pgns);
        return;
    }

    cJSON_AddNumberToObject(device, "src", N2K_LOCAL_SOURCE);
    cJSON_AddStringToObject(device, "name", N2K_GATEWAY_DEVICE_NAME);
    cJSON_AddStringToObject(device, "model", N2K_GATEWAY_MODEL_NAME);
    cJSON_AddStringToObject(device, "manufacturer", "Manufacturer code 2046");
    cJSON_AddStringToObject(device, "category", N2K_GATEWAY_CATEGORY);
    cJSON_AddBoolToObject(device, "online", true);
    cJSON_AddNullToObject(device, "lastSeen");
    cJSON_AddBoolToObject(device, "hasAddressClaim", n2k_local_address_claim_sent);
    cJSON_AddBoolToObject(device, "hasProductInfo", false);
    cJSON_AddBoolToObject(device, "hasConfigurationInfo", false);
    cJSON_AddBoolToObject(device, "hasTxPgnList", false);
    cJSON_AddBoolToObject(device, "hasRxPgnList", false);
    cJSON_AddBoolToObject(device, "isGateway", true);
    cJSON_AddItemToObject(device, "supportedPgns", supported_pgns);
    cJSON_AddItemToArray(devices, device);
}

/* Large JSON responses are split across notify packets; the app should reassemble bytes until '\n'. */
static int ble_send_device_list_response(uint16_t conn_handle)
{
    size_t snapshot_bytes = sizeof(N2kDeviceEntry_t) * N2K_DEVICE_MANAGER_MAX_DEVICES;
    N2kDeviceEntry_t *snapshot = NULL;
    size_t device_count = 0u;
    uint8_t live_wind_source = 0xFFu;
    bool live_wind_source_valid = false;
    cJSON *root = NULL;
    cJSON *devices = NULL;
    char *json_text;
    char *framed_text;
    int rc;

    snapshot = (N2kDeviceEntry_t *)malloc(snapshot_bytes);
    if (snapshot == NULL) {
        ESP_LOGE(tag, "BLE device_list snapshot allocation failed: bytes=%u", (unsigned)snapshot_bytes);
        return BLE_HS_ENOMEM;
    }

    device_count = n2k_copy_device_snapshot(snapshot, N2K_DEVICE_MANAGER_MAX_DEVICES);
    root = cJSON_CreateObject();
    devices = cJSON_CreateArray();

    if ((root == NULL) || (devices == NULL)) {
        cJSON_Delete(root);
        cJSON_Delete(devices);
        ESP_LOGE(tag, "BLE device_list build failed: out of memory");
        free(snapshot);
        return BLE_HS_ENOMEM;
    }

    cJSON_AddStringToObject(root, "type", "device_list");
    cJSON_AddItemToObject(root, "devices", devices);
    ble_add_gateway_device_json(devices);
    live_wind_source_valid = n2k_get_live_wind_source(&live_wind_source);

    for (size_t i = 0; i < device_count; i++) {
        cJSON *device = cJSON_CreateObject();
        cJSON *supported_pgns = cJSON_CreateArray();
        const char *name_source = NULL;
        const char *model_source = NULL;
        const char *manufacturer_source = NULL;
        const char *category_source = NULL;
        const char *name;
        const char *model;
        const char *manufacturer_text;
        const char *category;
        bool has_last_seen;
        bool is_online;

        if ((device == NULL) || (supported_pgns == NULL)) {
            cJSON_Delete(root);
            ESP_LOGE(tag, "BLE device_list build failed while creating device JSON");
            free(snapshot);
            return BLE_HS_ENOMEM;
        }

        name = n2k_best_device_name(&snapshot[i],
                                    ble_device_list_meta_scratch.name_buf,
                                    sizeof(ble_device_list_meta_scratch.name_buf),
                                    &name_source);
        model = n2k_best_model(&snapshot[i],
                               ble_device_list_meta_scratch.model_buf,
                               sizeof(ble_device_list_meta_scratch.model_buf),
                               &model_source);
        manufacturer_text = n2k_best_manufacturer(&snapshot[i],
                                                  ble_device_list_meta_scratch.manufacturer_buf,
                                                  sizeof(ble_device_list_meta_scratch.manufacturer_buf),
                                                  &manufacturer_source);
        category = n2k_guess_category(&snapshot[i], &category_source);
        has_last_seen = n2k_format_last_seen_utc(&snapshot[i],
                                                 ble_device_list_meta_scratch.last_seen_buf,
                                                 sizeof(ble_device_list_meta_scratch.last_seen_buf));
        is_online = n2k_entry_seen_within_window(&snapshot[i],
                             (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS),
                             N2K_DEVICE_ONLINE_MS);
        cJSON_AddNumberToObject(device, "src", snapshot[i].source);
        cJSON_AddStringToObject(device, "name", name);
        cJSON_AddStringToObject(device, "model", model);
        cJSON_AddStringToObject(device, "manufacturer", manufacturer_text);
        cJSON_AddStringToObject(device, "category", category);
        cJSON_AddBoolToObject(device, "online", is_online);
        if (has_last_seen) {
            cJSON_AddStringToObject(device, "lastSeen", ble_device_list_meta_scratch.last_seen_buf);
        } else {
            cJSON_AddNullToObject(device, "lastSeen");
        }
        cJSON_AddBoolToObject(device, "hasAddressClaim", snapshot[i].has_address_claim);
        cJSON_AddBoolToObject(device, "hasProductInfo", snapshot[i].has_product_info);
        cJSON_AddBoolToObject(device, "hasConfigurationInfo", snapshot[i].has_configuration_info);
        cJSON_AddBoolToObject(device, "hasTxPgnList", snapshot[i].has_tx_pgn_list);
        cJSON_AddBoolToObject(device, "hasRxPgnList", snapshot[i].has_rx_pgn_list);
        cJSON_AddBoolToObject(device,
                              "hasLiveWindData",
                              live_wind_source_valid && (snapshot[i].source == live_wind_source));
        if (n2k_text_available(snapshot[i].serial_code)) {
            cJSON_AddStringToObject(device, "serialNumber", snapshot[i].serial_code);
        }
        if (n2k_text_available(snapshot[i].software_version)) {
            cJSON_AddStringToObject(device, "softwareVersion", snapshot[i].software_version);
        }
        if (n2k_text_available(snapshot[i].model)) {
            cJSON_AddStringToObject(device, "modelVersion", snapshot[i].model);
        }
        if (n2k_text_available(snapshot[i].manufacturer_text)) {
            cJSON_AddStringToObject(device, "manufacturerText", snapshot[i].manufacturer_text);
        }
        if (n2k_text_available(snapshot[i].installation_desc1)) {
            cJSON_AddStringToObject(device, "installationDescription1", snapshot[i].installation_desc1);
        }
        if (n2k_text_available(snapshot[i].installation_desc2)) {
            cJSON_AddStringToObject(device, "installationDescription2", snapshot[i].installation_desc2);
        }
        n2k_add_supported_pgns_json(supported_pgns, &snapshot[i]);
        cJSON_AddItemToObject(device, "supportedPgns", supported_pgns);
        cJSON_AddItemToArray(devices, device);
    }

    json_text = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (json_text == NULL) {
        ESP_LOGE(tag, "BLE device_list serialization failed");
        free(snapshot);
        return BLE_HS_ENOMEM;
    }

    framed_text = (char *)malloc(strlen(json_text) + 2u);
    if (framed_text == NULL) {
        cJSON_free(json_text);
        ESP_LOGE(tag, "BLE device_list framing failed");
        free(snapshot);
        return BLE_HS_ENOMEM;
    }

    snprintf(framed_text, strlen(json_text) + 2u, "%s\n", json_text);
    ESP_LOGI(tag, "BLE sending device_list: devices=%u json_len=%u",
             (unsigned)device_count,
             (unsigned)strlen(json_text));
    rc = gatt_svr_notify_text_chunks(conn_handle, framed_text);
    ESP_LOGI(tag, "BLE device_list send %s, rc=%d",
             rc == 0 ? "succeeded" : "failed",
             rc);

    free(snapshot);
    free(framed_text);
    cJSON_free(json_text);
    return rc;
}

static void ble_handle_command_line(const char *command_line, void *ctx)
{
    cJSON *root;
    cJSON *command;
    const char *parsed_command = NULL;
    (void)ctx;

    if (command_line == NULL) {
        ESP_LOGW(tag, "BLE command parse failed: empty input");
        return;
    }

    root = cJSON_Parse(command_line);
    if (root == NULL) {
        ESP_LOGW(tag, "BLE command parse failed: invalid JSON");
        return;
    }

    command = cJSON_GetObjectItemCaseSensitive(root, "command");
    if (!cJSON_IsString(command) || (command->valuestring == NULL)) {
        ESP_LOGW(tag, "BLE command parse failed: missing command");
        cJSON_Delete(root);
        return;
    }

    parsed_command = command->valuestring;
    if (strcmp(parsed_command, "request_device_list") == 0) {
        ble_device_list_request_pending = true;
    } else {
        ESP_LOGW(tag, "BLE command ignored: unsupported command '%s'", parsed_command);
    }

    cJSON_Delete(root);
}

void ble_store_config_init(void);

static void n2k_set_latest_text(const char *text)
{
    if (text == NULL || n2k_text_lock == NULL) {
        return;
    }

    if (xSemaphoreTake(n2k_text_lock, portMAX_DELAY) == pdTRUE) {
        snprintf(latest_n2k_text, sizeof(latest_n2k_text), "%s", text);
        latest_n2k_seq++;
        xSemaphoreGive(n2k_text_lock);
    }
}

static bool n2k_enqueue_tx_frame(const N2K_RawFrame_t *frame, void *ctx) {
    QueueHandle_t queue = (QueueHandle_t)ctx;
    if ((queue == NULL) || (frame == NULL)) {
        return false;
    }
    return xQueueSend(queue, frame, 0) == pdTRUE;
}

static void n2k_update_latest_text_from_model(void) {
    char line[180];
    char wind_text[40];
    char heading_text[24];
    char gps_text[40];
    char battery_text[40];
    size_t device_count = n2k_device_manager_count(&n2k_app_model.devices);
    const char *model_name = "n/a";
    const N2kDeviceEntry_t *entries = n2k_device_manager_entries(&n2k_app_model.devices);

    if (entries != NULL) {
        for (size_t i = 0; i < N2K_DEVICE_MANAGER_MAX_DEVICES; i++) {
            if (entries[i].used && entries[i].model[0] != '\0') {
                model_name = entries[i].model;
                break;
            }
        }
    }

    if (n2k_app_model.wind.valid) {
        snprintf(wind_text, sizeof(wind_text), "spd=%.2f,ang=%.1f,ref=%u",
                 n2k_app_model.wind.speed_mps,
                 n2k_app_model.wind.angle_deg,
                 (unsigned)n2k_app_model.wind.reference);
    } else {
        snprintf(wind_text, sizeof(wind_text), "-");
    }

    if (n2k_app_model.heading.valid) {
        snprintf(heading_text, sizeof(heading_text), "%.1fdeg",
                 n2k_app_model.heading.heading_deg);
    } else {
        snprintf(heading_text, sizeof(heading_text), "-");
    }

    if (n2k_app_model.position.valid) {
        snprintf(gps_text, sizeof(gps_text), "%.5f,%.5f",
                 n2k_app_model.position.latitude_deg,
                 n2k_app_model.position.longitude_deg);
    } else {
        snprintf(gps_text, sizeof(gps_text), "-");
    }

    if (n2k_app_model.battery.valid) {
        snprintf(battery_text, sizeof(battery_text), "%.2fV/%.2fA/%.1fC",
                 n2k_app_model.battery.voltage_v,
                 n2k_app_model.battery.current_a,
                 n2k_app_model.battery.temperature_c);
    } else {
        snprintf(battery_text, sizeof(battery_text), "-");
    }

    snprintf(line, sizeof(line),
             "dev:%u wind:%.24s hdg:%.12s gps:%.24s bat:%.24s unknown:%lu model:%.20s",
             (unsigned)device_count,
             wind_text,
             heading_text,
             gps_text,
             battery_text,
             (unsigned long)n2k_app_model.unknown_pgn_count,
             model_name);
    n2k_set_latest_text(line);
}

static void spi_n2k_task(void *param) {
    (void)param;
    SpiN2kTransportParser_t parser;
    TickType_t last_stats_log = xTaskGetTickCount();
    uint8_t tx_buf[SPI_RX_BUF_SIZE];
    WORD_ALIGNED_ATTR static uint8_t rx_buf[SPI_RX_BUF_SIZE];
    N2K_RawFrame_t tx_frame;

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
        .queue_size = 1,
        .mode = 0,
    };

    spi_n2k_transport_parser_init(&parser);
    ESP_ERROR_CHECK(spi_slave_initialize(SPI_SLAVE_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO));
    ESP_LOGI(tag, "SPI slave ready (MOSI=%d MISO=%d SCLK=%d CS=%d)",
             PIN_NUM_MOSI, PIN_NUM_MISO, PIN_NUM_SCLK, PIN_NUM_CS);

    while (1) {
        spi_slave_transaction_t trans;
        memset(rx_buf, 0, sizeof(rx_buf));
        memset(tx_buf, 0xFF, sizeof(tx_buf));

        if (xQueueReceive(n2k_tx_queue, &tx_frame, 0) == pdTRUE) {
            size_t packet_len = 0u;
            (void)spi_n2k_transport_build_frame_packet(SPI_N2K_PKT_TYPE_N2K_TX_FRAME, &tx_frame, tx_buf, sizeof(tx_buf), &packet_len);
        }

        memset(&trans, 0, sizeof(trans));
        trans.length = SPI_RX_BUF_SIZE * 8;
        trans.rx_buffer = rx_buf;
        trans.tx_buffer = tx_buf;

        if (spi_slave_transmit(SPI_SLAVE_HOST, &trans, portMAX_DELAY) != ESP_OK) {
            continue;
        }

        int rx_len_bytes = trans.trans_len / 8;
        for (int i = 0; i < rx_len_bytes; i++) {
            SpiN2kPacket_t packet;
            if (spi_n2k_transport_parser_consume_byte(&parser, rx_buf[i], &packet)) {
                if (packet.pkt_type == SPI_N2K_PKT_TYPE_N2K_RX_FRAME) {
                    N2K_RawFrame_t frame;
                    if (spi_n2k_transport_parse_frame_payload(&packet, &frame)) {
                        frame.flags |= N2K_RAW_FLAG_DIRECTION_RX;
                        if ((n2k_model_lock != NULL) &&
                            (xSemaphoreTake(n2k_model_lock, portMAX_DELAY) == pdTRUE)) {
                            n2k_decoder_process_frame(&n2k_decoder, &frame);
                            n2k_update_latest_text_from_model();
                            xSemaphoreGive(n2k_model_lock);
                        }
                    }
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

static void n2k_request_task(void *param) {
    (void)param;
    while (1) {
        uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        bool refresh_needed = ble_device_list_request_pending || ble_device_list_refresh_active;

        if (!refresh_needed) {
            vTaskDelay(pdMS_TO_TICKS(BLE_NOTIFY_TASK_IDLE_DELAY_MS));
            continue;
        }

        if (!n2k_request_initial_discovery_if_needed(now_ms)) {
            (void)n2k_request_next_pending_metadata(now_ms);
        }

        vTaskDelay(pdMS_TO_TICKS(BLE_NOTIFY_TASK_REFRESH_POLL_MS));
    }
}

static void
ble_test_notify_task(void *param)
{
    uint32_t last_sent_seq = 0;
    int rc;
    (void)param;

    while (1) {
        TickType_t loop_delay = pdMS_TO_TICKS(BLE_NOTIFY_TASK_IDLE_DELAY_MS);
        if (active_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            bool has_new = false;
            uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

            if (ble_device_list_request_pending && !ble_device_list_refresh_active) {
                uint32_t useful_requests_needed = 0u;
                uint32_t optional_pgn_requests_needed = 0u;
                ble_device_list_request_pending = false;
                ble_request_missing_device_metadata(&useful_requests_needed, &optional_pgn_requests_needed);
                if (n2k_request_gap_elapsed(now_ms) &&
                    n2k_send_request(0xffu,
                                     N2K_PGN_ADDRESS_CLAIM,
                                     now_ms)) {
                }

                ble_device_list_refresh_active = true;
                ble_device_list_refresh_start_ms = now_ms;
            }

            if (ble_device_list_refresh_active) {
                size_t useful_incomplete_devices = 0u;
                loop_delay = pdMS_TO_TICKS(BLE_NOTIFY_TASK_REFRESH_POLL_MS);
                n2k_collect_refresh_status(NULL,
                                           &useful_incomplete_devices,
                                           NULL);
                if ((uint32_t)(now_ms - ble_device_list_refresh_start_ms) >= BLE_DEVICE_LIST_REFRESH_WINDOW_MS) {
                    if ((useful_incomplete_devices > 0u) &&
                        ((uint32_t)(now_ms - ble_device_list_refresh_start_ms) < BLE_DEVICE_LIST_REFRESH_MAX_WINDOW_MS)) {
                    } else {
                        rc = ble_send_device_list_response(active_conn_handle);
                        ble_device_list_refresh_active = false;
                        ble_device_list_refresh_start_ms = 0u;
                        (void)rc;
                    }
                }
            }

            if (xSemaphoreTake(n2k_text_lock, pdMS_TO_TICKS(50)) == pdTRUE) {
                if (latest_n2k_seq != last_sent_seq) {
                    snprintf(ble_notify_task_payload, sizeof(ble_notify_task_payload), "%s", latest_n2k_text);
                    last_sent_seq = latest_n2k_seq;
                    has_new = true;
                }
                xSemaphoreGive(n2k_text_lock);
            }

            if (has_new && !ble_device_list_request_pending && !ble_device_list_refresh_active) {
                rc = gatt_svr_notify_text(active_conn_handle, ble_notify_task_payload);

                if (rc == 0) {
                    ESP_LOGI(tag, "notification sent: %s", ble_notify_task_payload);
                } else {
                    ESP_LOGW(tag, "notification send failed, rc=%d", rc);
                }
            }
        } else if (ble_device_list_request_pending || ble_device_list_refresh_active) {
            ble_device_list_request_pending = false;
            ble_device_list_refresh_active = false;
            ble_device_list_refresh_start_ms = 0u;
        }

        vTaskDelay(loop_delay);
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

#if CONFIG_BT_NIMBLE_GAP_SERVICE
    name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;
#endif

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error setting advertisement data; rc=%d\n", rc);
        return;
    }

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
        MODLOG_DFLT(INFO, "notify_tx event; conn_handle=%d attr_handle=%d "
                    "status=%d is_indication=%d",
                    event->notify_tx.conn_handle,
                    event->notify_tx.attr_handle,
                    event->notify_tx.status,
                    event->notify_tx.indication);
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

    n2k_text_lock = xSemaphoreCreateMutex();
    if (n2k_text_lock == NULL) {
        ESP_LOGE(tag, "failed to create n2k mutex");
        return;
    }
    n2k_model_lock = xSemaphoreCreateMutex();
    if (n2k_model_lock == NULL) {
        ESP_LOGE(tag, "failed to create n2k model mutex");
        return;
    }
    n2k_tx_queue = xQueueCreate(N2K_TX_QUEUE_LEN, sizeof(N2K_RawFrame_t));
    if (n2k_tx_queue == NULL) {
        ESP_LOGE(tag, "failed to create n2k tx queue");
        return;
    }
    n2k_app_model_init(&n2k_app_model);
    n2k_decoder_init(&n2k_decoder, &n2k_app_model);
    n2k_request_manager_init(&n2k_request_manager, N2K_LOCAL_SOURCE, n2k_enqueue_tx_frame, n2k_tx_queue);

    /* Initialize NVS - it is used to store PHY calibration data */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(tag, "Failed to init nimble %d ", ret);
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
    rc = ble_svc_gap_device_name_set("BLE Test");
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

    /* Increased because device_list JSON + BLE notify path exceeded previous stack headroom. */
    xTaskCreate(ble_test_notify_task, "ble_test_notify", BLE_NOTIFY_TASK_STACK_SIZE, NULL, 5, NULL);
    xTaskCreate(spi_n2k_task, "spi_n2k_task", 4096, NULL, 6, NULL);
    xTaskCreate(n2k_request_task, "n2k_request_task", 3072, NULL, 5, NULL);
}
