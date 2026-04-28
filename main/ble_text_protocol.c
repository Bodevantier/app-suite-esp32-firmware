#include "ble_text_protocol.h"

#include <stdio.h>
#include <string.h>

#include "bleprph.h"
#include "esp_log.h"

static const char * const ble_proto_tag = "BLE_PROTO";

#define BLE_TEXT_LINE_MAX_LEN 512

static bool safe_starts_with(const char *text, const char *prefix) {
    size_t len;

    if ((text == NULL) || (prefix == NULL)) {
        return false;
    }
    len = strlen(prefix);
    if (strlen(text) < len) {
        return false;
    }
    return strncmp(text, prefix, len) == 0;
}

static int notify_line(uint16_t conn_handle, const char *line) {
    char out[BLE_TEXT_LINE_MAX_LEN + 2u];

    if (line == NULL) {
        return -1;
    }

    snprintf(out, sizeof(out), "%s\n", line);
    return gatt_svr_notify_text_chunks(conn_handle, out);
}

static const char *safe_text(const char *value) {
    if ((value == NULL) || (value[0] == '\0')) {
        return "-";
    }

    return value;
}

static void format_name_hex(uint64_t name, char *buffer, size_t buffer_size) {
    unsigned long upper;
    unsigned long lower;

    if ((buffer == NULL) || (buffer_size == 0u)) {
        return;
    }

    upper = (unsigned long)(name >> 32u);
    lower = (unsigned long)(name & 0xffffffffUL);
    snprintf(buffer, buffer_size, "%08lX%08lX", upper, lower);
}

bool ble_text_command_is_request_device_list(const char *command_line) {
    if (command_line == NULL) {
        return false;
    }

    if (strcmp(command_line, "request_device_list") == 0) {
        return true;
    }
    if (safe_starts_with(command_line, "cmd:request_device_list")) {
        return true;
    }
    return false;
}

int ble_text_send_snapshot(uint16_t conn_handle,
                           const N2kAppModel_t *model,
                           uint8_t local_source,
                           uint32_t now_ms,
                           uint32_t online_window_ms,
                           uint32_t stale_hide_ms) {
    char line[BLE_TEXT_LINE_MAX_LEN + 1u];
    int rc;
    int lines_sent = 0;
    int lines_failed = 0;

    if (model == NULL) {
        return -1;
    }

    ESP_LOGI(ble_proto_tag, "[SNAP START] devices=%u complete=%d in_progress=%d",
             (unsigned)model->devices.count,
             model->device_list.complete ? 1 : 0,
             model->device_list.in_progress ? 1 : 0);

    snprintf(line,
             sizeof(line),
             "device_list snapshot id=%u complete=%u expected=%u received=%u malformed=%lu completed=%lu dropped=%lu unknown=%lu parse=%lu",
             (unsigned)model->device_list.request_id,
             model->device_list.complete ? 1u : 0u,
             (unsigned)model->device_list.expected_count,
             (unsigned)model->device_list.received_records,
             (unsigned long)model->device_list_stats.malformed_device_list_messages,
             (unsigned long)model->device_list_stats.completed_snapshots,
             (unsigned long)model->device_list_stats.dropped_snapshots,
             (unsigned long)model->device_list_stats.unknown_packet_types,
             (unsigned long)model->device_list_stats.spi_parse_errors);
    ESP_LOGI(ble_proto_tag, "[BLE TX] %s", line);
    rc = notify_line(conn_handle, line);
    if (rc != 0) {
        ESP_LOGE(ble_proto_tag, "[SNAP] header notify FAILED rc=%d", rc);
        return rc;
    }
    lines_sent++;

    snprintf(line,
             sizeof(line),
             "device src=%u gateway=1 online=1 hasAddressClaim=1 name=SDolve NMEA2000 Bluetooth model=ESP32_SPI_N2K_BLE manufacturer=SDolve category=gateway",
             (unsigned)local_source);
    ESP_LOGI(ble_proto_tag, "[BLE TX] %s", line);
    rc = notify_line(conn_handle, line);
    if (rc != 0) {
        ESP_LOGE(ble_proto_tag, "[SNAP] gateway line notify FAILED rc=%d", rc);
        return rc;
    }
    lines_sent++;

    if (model->devices.entries == NULL) {
        return 0;
    }
    for (size_t i = 0; i < model->devices.count; i++) {
        const N2kDeviceEntry_t *entry = &model->devices.entries[i];
        uint32_t age_ms;
        bool visible;
        bool online;
        char name_hex[17];
        const char *display_name;
        const char *display_model;
        const char *display_manufacturer;

        if (!entry->used || (entry->source == local_source)) {
            continue;
        }
        if (entry->last_seen_ms == 0u) {
            continue;
        }

        age_ms = now_ms - entry->last_seen_ms;
        visible = age_ms <= stale_hide_ms;
        if (!visible) {
            continue;
        }
        online = age_ms <= online_window_ms;
        format_name_hex(entry->name, name_hex, sizeof(name_hex));
        display_name = entry->model_id[0] != '\0' ? entry->model_id : name_hex;
        display_model = safe_text(entry->model);
        display_manufacturer = entry->manufacturer_text[0] != '\0'
            ? entry->manufacturer_text
            : "-";

        snprintf(line,
                 sizeof(line),
                 "device src=%u online=%u hasAddressClaim=%u hasProductInfo=%u hasConfigurationInfo=%u product=%u mfg=%u name=%s model=%s manufacturer=%s serial=%s sw=%s install1=%s install2=%s tx=%u rx=%u deviceClass=%u deviceFunction=%u nameValue=%s",
                 (unsigned)entry->source,
                 online ? 1u : 0u,
                 entry->has_address_claim ? 1u : 0u,
                 entry->has_product_info ? 1u : 0u,
                 entry->has_configuration_info ? 1u : 0u,
                 (unsigned)entry->product_code,
                 (unsigned)entry->manufacturer_code,
                 display_name,
                 display_model,
                 display_manufacturer,
                 safe_text(entry->serial_code),
                 safe_text(entry->software_version),
                 safe_text(entry->installation_desc1),
                 safe_text(entry->installation_desc2),
                 entry->has_tx_pgn_list ? 1u : 0u,
                 entry->has_rx_pgn_list ? 1u : 0u,
                 (unsigned)entry->device_class,
                 (unsigned)entry->device_function,
                 name_hex);

        ESP_LOGI(ble_proto_tag, "[BLE TX] %s", line);
        rc = notify_line(conn_handle, line);
        if (rc != 0) {
            ESP_LOGE(ble_proto_tag, "[SNAP] device[%u] src=%u notify FAILED rc=%d (sent=%d failed=%d)",
                     (unsigned)i, (unsigned)entry->source, rc, lines_sent, lines_failed + 1);
            lines_failed++;
            /* Continue sending remaining devices rather than aborting the whole
               snapshot.  A partial list is better than nothing. */
            continue;
        }
        lines_sent++;
    }

    ESP_LOGI(ble_proto_tag, "[SNAP END] lines_sent=%d lines_failed=%d", lines_sent, lines_failed);
    return lines_failed > 0 ? -1 : 0;
}
