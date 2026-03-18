#include "n2k_device_manager.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"

static const char *TAG = "n2k_device_mgr";

#define N2K_PRODUCT_INFO_FIXED_TEXT_LEN 32u
#define N2K_PGN_LIST_TYPE_TRANSMIT 0u
#define N2K_PGN_LIST_TYPE_RECEIVE 1u

static void format_name_hex(uint64_t name, char *buffer, size_t buffer_size) {
    uint32_t upper = (uint32_t)(name >> 32u);
    uint32_t lower = (uint32_t)(name & 0xffffffffu);

    if ((buffer == NULL) || (buffer_size == 0u)) {
        return;
    }

    snprintf(buffer, buffer_size, "%08" PRIX32 "%08" PRIX32, upper, lower);
}

static uint16_t read_u16_le(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8u);
}

static uint32_t read_u24_le(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8u) | ((uint32_t)p[2] << 16u);
}

static uint64_t read_u64_le(const uint8_t *p) {
    uint64_t v = 0;
    for (uint8_t i = 0; i < 8u; i++) {
        v |= ((uint64_t)p[i] << (8u * i));
    }
    return v;
}

static void copy_ascii_field(char *dst, size_t dst_sz, const uint8_t *src, uint8_t src_len) {
    size_t n;
    if ((dst == NULL) || (dst_sz == 0u)) {
        return;
    }
    if ((src == NULL) || (src_len == 0u)) {
        dst[0] = '\0';
        return;
    }
    n = (size_t)src_len;
    if (n >= dst_sz) {
        n = dst_sz - 1u;
    }
    memcpy(dst, src, n);
    dst[n] = '\0';
}

static void copy_ascii_fixed_field(char *dst, size_t dst_sz, const uint8_t *src, size_t src_len) {
    size_t start = 0u;
    size_t end = src_len;

    if ((dst == NULL) || (dst_sz == 0u)) {
        return;
    }
    if ((src == NULL) || (src_len == 0u)) {
        dst[0] = '\0';
        return;
    }

    while ((start < end) &&
           ((src[start] == 0x00u) || (src[start] == 0xFFu) || (src[start] == ' '))) {
        start++;
    }
    while ((end > start) &&
           ((src[end - 1u] == 0x00u) || (src[end - 1u] == 0xFFu) || (src[end - 1u] == ' '))) {
        end--;
    }

    copy_ascii_field(dst, dst_sz, &src[start], (uint8_t)(end - start));
}

static bool parse_string_lau(const uint8_t *data, size_t len, size_t *index, char *dst, size_t dst_sz) {
    uint8_t field_len;
    size_t available;

    if ((data == NULL) || (index == NULL) || (dst == NULL) || (dst_sz == 0u) || (*index >= len)) {
        return false;
    }

    field_len = data[*index];
    (*index)++;
    if (field_len == 0u || field_len == 0xFFu) {
        dst[0] = '\0';
        return true;
    }

    available = len - *index;
    if ((size_t)field_len > available) {
        dst[0] = '\0';
        *index = len;
        return false;
    }

    if (field_len <= 1u) {
        dst[0] = '\0';
        *index += field_len;
        return true;
    }

    copy_ascii_field(dst, dst_sz, &data[*index + 1u], (uint8_t)(field_len - 1u));
    *index += field_len;
    return true;
}

static void mark_entry_discovered(N2kDeviceEntry_t *entry, uint8_t source, uint32_t now_ms) {
    if (entry == NULL) {
        return;
    }

    if (entry->first_seen_ms == 0u) {
        entry->first_seen_ms = now_ms;
        ESP_LOGI(TAG,
                 "[DISCOVERED] tick=%" PRIu32 " src=%u",
                 now_ms,
                 (unsigned)source);
    }
}

static void clear_device_dynamic_metadata(N2kDeviceEntry_t *entry) {
    if (entry == NULL) {
        return;
    }

    entry->n2k_version = 0u;
    entry->product_code = 0u;
    entry->has_product_info = false;
    entry->has_configuration_info = false;
    entry->has_tx_pgn_list = false;
    entry->has_rx_pgn_list = false;
    entry->certification_level = 0u;
    entry->load_equivalency = 0u;
    entry->product_request_count = 0u;
    entry->configuration_request_count = 0u;
    entry->pgn_list_request_count = 0u;
    entry->supported_pgn_count = 0u;
    entry->last_product_request_ms = 0u;
    entry->last_configuration_request_ms = 0u;
    entry->last_pgn_list_request_ms = 0u;
    memset(entry->supported_pgns, 0, sizeof(entry->supported_pgns));
}

static void add_supported_pgn(N2kDeviceEntry_t *entry, uint32_t pgn) {
    if (entry == NULL) {
        return;
    }

    for (uint8_t i = 0u; i < entry->supported_pgn_count; i++) {
        if (entry->supported_pgns[i] == pgn) {
            return;
        }
    }

    if (entry->supported_pgn_count >= N2K_DEVICE_SUPPORTED_PGNS_MAX) {
        return;
    }

    entry->supported_pgns[entry->supported_pgn_count++] = pgn;
}

static N2kDeviceEntry_t *find_or_create_entry(N2kDeviceManager_t *manager, uint8_t source) {
    N2kDeviceEntry_t *free_slot = NULL;

    for (size_t i = 0; i < N2K_DEVICE_MANAGER_MAX_DEVICES; i++) {
        N2kDeviceEntry_t *entry = &manager->entries[i];
        if (entry->used && entry->source == source) {
            return entry;
        }
        if (!entry->used && free_slot == NULL) {
            free_slot = entry;
        }
    }

    if (free_slot == NULL) {
        manager->drop_count++;
        return NULL;
    }

    memset(free_slot, 0, sizeof(*free_slot));
    free_slot->used = true;
    free_slot->source = source;
    return free_slot;
}

void n2k_device_manager_init(N2kDeviceManager_t *manager) {
    if (manager == NULL) {
        return;
    }
    memset(manager, 0, sizeof(*manager));
}

void n2k_device_manager_update_last_seen(N2kDeviceManager_t *manager, uint8_t source, uint32_t now_ms) {
    N2kDeviceEntry_t *entry;
    if (manager == NULL) {
        return;
    }
    entry = find_or_create_entry(manager, source);
    if (entry == NULL) {
        return;
    }
    mark_entry_discovered(entry, source, now_ms);
    entry->last_seen_ms = now_ms;
}

void n2k_device_manager_handle_address_claim(N2kDeviceManager_t *manager, uint8_t source, uint32_t now_ms, const uint8_t *data, size_t len) {
    N2kDeviceEntry_t *entry;
    uint64_t name;
    uint64_t previous_name;
    bool had_address_claim;
    char old_name_hex[17];
    char new_name_hex[17];

    if ((manager == NULL) || (data == NULL) || (len < 8u)) {
        return;
    }
    entry = find_or_create_entry(manager, source);
    if (entry == NULL) {
        return;
    }

    mark_entry_discovered(entry, source, now_ms);

    name = read_u64_le(data);
    previous_name = entry->name;
    had_address_claim = entry->has_address_claim;
    if (had_address_claim && (previous_name != 0u) && (previous_name != name)) {
        format_name_hex(previous_name, old_name_hex, sizeof(old_name_hex));
        format_name_hex(name, new_name_hex, sizeof(new_name_hex));
        ESP_LOGW(TAG,
                 "[ADDR CLAIM] tick=%" PRIu32 " src=%u name changed old=%s new=%s",
                 now_ms,
                 (unsigned)source,
                 old_name_hex,
                 new_name_hex);
        clear_device_dynamic_metadata(entry);
    }

    entry->source = source;
    entry->name = name;
    entry->unique_number = (uint32_t)(name & 0x1FFFFFu);
    entry->manufacturer_code = (uint16_t)((name >> 21u) & 0x7FFu);
    entry->device_instance_lower = (uint8_t)((name >> 32u) & 0x07u);
    entry->device_instance_upper = (uint8_t)((name >> 35u) & 0x1Fu);
    entry->device_function = (uint8_t)((name >> 40u) & 0xFFu);
    entry->device_class = (uint8_t)((name >> 49u) & 0x7Fu);
    entry->system_instance = (uint8_t)((name >> 56u) & 0x0Fu);
    entry->has_address_claim = true;
    entry->last_address_claim_request_ms = 0u;
    entry->address_claim_request_count = 0u;
    entry->last_seen_ms = now_ms;

    format_name_hex(name, new_name_hex, sizeof(new_name_hex));
    ESP_LOGI(TAG,
             "[ADDR CLAIM] tick=%" PRIu32 " src=%u name=%s unique=%" PRIu32 " manufacturer=%u instance=%u function=%u class=%u system=%u",
             now_ms,
             (unsigned)source,
             new_name_hex,
             entry->unique_number,
             (unsigned)entry->manufacturer_code,
             (unsigned)((entry->device_instance_upper << 3u) | entry->device_instance_lower),
             (unsigned)entry->device_function,
             (unsigned)entry->device_class,
             (unsigned)entry->system_instance);
}

void n2k_device_manager_handle_product_info(N2kDeviceManager_t *manager, uint8_t source, uint32_t now_ms, const uint8_t *data, size_t len) {
    N2kDeviceEntry_t *entry;

    if ((manager == NULL) || (data == NULL) || (len < 4u)) {
        return;
    }
    entry = find_or_create_entry(manager, source);
    if (entry == NULL) {
        return;
    }

    mark_entry_discovered(entry, source, now_ms);

    entry->n2k_version = read_u16_le(&data[0]);
    entry->product_code = read_u16_le(&data[2]);
    if (len >= 4u + N2K_PRODUCT_INFO_FIXED_TEXT_LEN) {
        copy_ascii_fixed_field(entry->model_id, sizeof(entry->model_id), &data[4], N2K_PRODUCT_INFO_FIXED_TEXT_LEN);
    }
    if (len >= 36u + N2K_PRODUCT_INFO_FIXED_TEXT_LEN) {
        copy_ascii_fixed_field(entry->software_version, sizeof(entry->software_version), &data[36], N2K_PRODUCT_INFO_FIXED_TEXT_LEN);
    }
    if (len >= 68u + N2K_PRODUCT_INFO_FIXED_TEXT_LEN) {
        copy_ascii_fixed_field(entry->model, sizeof(entry->model), &data[68], N2K_PRODUCT_INFO_FIXED_TEXT_LEN);
    }
    if (len >= 100u + N2K_PRODUCT_INFO_FIXED_TEXT_LEN) {
        copy_ascii_fixed_field(entry->serial_code, sizeof(entry->serial_code), &data[100], N2K_PRODUCT_INFO_FIXED_TEXT_LEN);
    }
    if (len >= 133u) {
        entry->certification_level = data[132];
    }
    if (len >= 134u) {
        entry->load_equivalency = data[133];
    }

    entry->has_product_info = true;
    entry->last_product_request_ms = 0u;
    entry->product_request_count = 0u;
    entry->last_seen_ms = now_ms;

    ESP_LOGI(TAG,
             "[PRODUCT] tick=%" PRIu32 " src=%u version=%u product=%u model=%s sw=%s modelVersion=%s serial=%s cert=%u len=%u",
             now_ms,
             (unsigned)source,
             (unsigned)entry->n2k_version,
             (unsigned)entry->product_code,
             entry->model_id[0] != '\0' ? entry->model_id : "-",
             entry->software_version[0] != '\0' ? entry->software_version : "-",
             entry->model[0] != '\0' ? entry->model : "-",
             entry->serial_code[0] != '\0' ? entry->serial_code : "-",
             (unsigned)entry->certification_level,
             (unsigned)entry->load_equivalency);
}

void n2k_device_manager_handle_configuration_info(N2kDeviceManager_t *manager, uint8_t source, uint32_t now_ms, const uint8_t *data, size_t len) {
    N2kDeviceEntry_t *entry;
    size_t index = 0u;

    if ((manager == NULL) || (data == NULL) || (len == 0u)) {
        return;
    }
    entry = find_or_create_entry(manager, source);
    if (entry == NULL) {
        return;
    }

    mark_entry_discovered(entry, source, now_ms);

    /* PGN 126998 is ordered as installation 1, installation 2, manufacturer text. */
    (void)parse_string_lau(data, len, &index, entry->installation_desc1, sizeof(entry->installation_desc1));
    (void)parse_string_lau(data, len, &index, entry->installation_desc2, sizeof(entry->installation_desc2));
    (void)parse_string_lau(data, len, &index, entry->manufacturer_text, sizeof(entry->manufacturer_text));

    entry->has_configuration_info = true;
    entry->last_configuration_request_ms = 0u;
    entry->configuration_request_count = 0u;
    entry->last_seen_ms = now_ms;

    ESP_LOGI(TAG,
             "[CONFIG] tick=%" PRIu32 " src=%u manufacturerText=%s install1=%s install2=%s",
             now_ms,
             (unsigned)source,
             entry->manufacturer_text[0] != '\0' ? entry->manufacturer_text : "-",
             entry->installation_desc1[0] != '\0' ? entry->installation_desc1 : "-",
             entry->installation_desc2[0] != '\0' ? entry->installation_desc2 : "-");
}

void n2k_device_manager_handle_supported_pgn_list(N2kDeviceManager_t *manager, uint8_t source, uint32_t now_ms, const uint8_t *data, size_t len) {
    N2kDeviceEntry_t *entry;
    uint8_t list_type;
    size_t pgn_count;
    char preview[128];
    size_t preview_used = 0u;

    if ((manager == NULL) || (data == NULL) || (len < 1u)) {
        return;
    }

    entry = find_or_create_entry(manager, source);
    if (entry == NULL) {
        return;
    }

    mark_entry_discovered(entry, source, now_ms);

    list_type = data[0];
    pgn_count = (len - 1u) / 3u;
    preview[0] = '\0';

    for (size_t i = 0u; i < pgn_count; i++) {
        uint32_t pgn = read_u24_le(&data[1u + (i * 3u)]);
        if ((preview_used + 1u) < sizeof(preview) && i < 6u) {
            int written = snprintf(preview + preview_used,
                                   sizeof(preview) - preview_used,
                                   "%s%" PRIu32,
                                   preview_used > 0u ? "," : "",
                                   pgn);
            if ((written > 0) && ((size_t)written < (sizeof(preview) - preview_used))) {
                preview_used += (size_t)written;
            }
        }
        add_supported_pgn(entry, pgn);
    }

    if (list_type == N2K_PGN_LIST_TYPE_TRANSMIT) {
        entry->has_tx_pgn_list = true;
    } else if (list_type == N2K_PGN_LIST_TYPE_RECEIVE) {
        entry->has_rx_pgn_list = true;
    }
    entry->last_seen_ms = now_ms;

    ESP_LOGI(TAG,
             "[PGN LIST] tick=%" PRIu32 " src=%u type=%s count=%u stored=%u sample=%s%s",
             now_ms,
             (unsigned)source,
             (list_type == N2K_PGN_LIST_TYPE_TRANSMIT) ? "tx" :
             (list_type == N2K_PGN_LIST_TYPE_RECEIVE) ? "rx" : "unknown",
             (unsigned)pgn_count,
             (unsigned)entry->supported_pgn_count,
             preview[0] != '\0' ? preview : "-",
             pgn_count > 6u ? ",..." : "");
}

size_t n2k_device_manager_count(const N2kDeviceManager_t *manager) {
    size_t count = 0u;
    if (manager == NULL) {
        return 0u;
    }
    for (size_t i = 0; i < N2K_DEVICE_MANAGER_MAX_DEVICES; i++) {
        if (manager->entries[i].used) {
            count++;
        }
    }
    return count;
}

const N2kDeviceEntry_t *n2k_device_manager_entries(const N2kDeviceManager_t *manager) {
    return (manager == NULL) ? NULL : manager->entries;
}
