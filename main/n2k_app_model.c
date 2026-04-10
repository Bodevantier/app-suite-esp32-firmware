#include "n2k_app_model.h"

#include <stdlib.h>
#include <string.h>

static uint32_t n2k_snapshot_last_seen_to_local(const N2kAppModel_t *model,
                                                uint32_t remote_last_seen_ms,
                                                uint32_t now_ms) {
    uint32_t snapshot_time_ms;
    uint32_t age_ms;

    if ((model == NULL) || (remote_last_seen_ms == 0u)) {
        return now_ms;
    }

    snapshot_time_ms = model->device_list.snapshot_time_ms;
    if ((snapshot_time_ms == 0u) || (remote_last_seen_ms >= snapshot_time_ms)) {
        return now_ms;
    }

    age_ms = snapshot_time_ms - remote_last_seen_ms;
    if (age_ms >= now_ms) {
        return 0u;
    }

    return now_ms - age_ms;
}

static int n2k_device_index_from_source(const N2kDeviceManager_t *manager, uint8_t source) {
    for (size_t i = 0; i < manager->count; i++) {
        if (manager->entries[i].source == source) {
            return (int)i;
        }
    }
    /* Not found - caller should append at index == count */
    return (int)manager->count;
}

/* Grow the entries array by 4 slots (up to N2K_DEVICE_MANAGER_MAX_DEVICES). */
static bool n2k_device_manager_grow(N2kDeviceManager_t *manager) {
    uint8_t new_cap;
    N2kDeviceEntry_t *new_entries;

    if (manager->capacity >= N2K_DEVICE_MANAGER_MAX_DEVICES) {
        return false;
    }
    new_cap = (uint8_t)(manager->capacity + 4u);
    if (new_cap > N2K_DEVICE_MANAGER_MAX_DEVICES) {
        new_cap = N2K_DEVICE_MANAGER_MAX_DEVICES;
    }
    new_entries = realloc(manager->entries, (size_t)new_cap * sizeof(N2kDeviceEntry_t));
    if (new_entries == NULL) {
        return false;
    }
    memset(&new_entries[manager->capacity], 0,
           (size_t)(new_cap - manager->capacity) * sizeof(N2kDeviceEntry_t));
    manager->entries = new_entries;
    manager->capacity = new_cap;
    return true;
}

void n2k_device_manager_init(N2kDeviceManager_t *manager) {
    if (manager == NULL) {
        return;
    }
    free(manager->entries);
    manager->entries = NULL;
    manager->count = 0u;
    manager->capacity = 0u;
}

const N2kDeviceEntry_t *n2k_device_manager_entries(const N2kDeviceManager_t *manager) {
    if (manager == NULL) {
        return NULL;
    }
    return manager->entries;
}

void n2k_app_model_init(N2kAppModel_t *model) {
    if (model == NULL) {
        return;
    }
    memset(model, 0, sizeof(*model));
    n2k_device_manager_init(&model->devices);
}

void n2k_app_model_store_raw(N2kAppModel_t *model, const N2K_RawFrame_t *frame) {
    if ((model == NULL) || (frame == NULL)) {
        return;
    }
    model->raw_ring[model->raw_write_index] = *frame;
    model->raw_write_index = (model->raw_write_index + 1u) % N2K_APP_RAW_RING_SIZE;
}

void n2k_app_model_note_spi_parser_stats(N2kAppModel_t *model,
                                         uint32_t bad_sof,
                                         uint32_t bad_len,
                                         uint32_t bad_crc,
                                         uint32_t unknown_type) {
    (void)bad_sof;
    if (model == NULL) {
        return;
    }
    model->device_list_stats.spi_parse_errors = bad_len + bad_crc;
    model->device_list_stats.unknown_packet_types = unknown_type;
}

bool n2k_app_model_device_list_begin(N2kAppModel_t *model,
                                     uint16_t request_id,
                                     uint32_t snapshot_time_ms,
                                     uint8_t own_source,
                                     uint8_t expected_count,
                                     uint32_t now_ms) {
    if (model == NULL) {
        return false;
    }

    if (model->device_list.in_progress && (model->device_list.request_id != request_id)) {
        model->device_list_stats.dropped_snapshots++;
    }

    model->device_list.in_progress = true;
    model->device_list.complete = false;
    model->device_list.request_id = request_id;
    model->device_list.snapshot_time_ms = snapshot_time_ms;
    model->device_list.own_source = own_source;
    model->device_list.expected_count = expected_count;
    model->device_list.end_count = 0u;
    model->device_list.received_records = 0u;
    model->device_list.started_ms = now_ms;
    model->device_list.last_update_ms = now_ms;

    /* Reset without freeing so the allocation can be reused for the next snapshot. */
    model->devices.count = 0u;
    return true;
}

bool n2k_app_model_device_list_add_device(N2kAppModel_t *model,
                                          uint16_t request_id,
                                          uint8_t record_index,
                                          uint8_t total_records,
                                          const N2kDeviceEntry_t *parsed,
                                          uint32_t now_ms) {
    N2kDeviceEntry_t entry;
    int index;

    (void)record_index;
    if ((model == NULL) || (parsed == NULL)) {
        return false;
    }
    if (!model->device_list.in_progress || (model->device_list.request_id != request_id)) {
        model->device_list_stats.malformed_device_list_messages++;
        return false;
    }

    index = n2k_device_index_from_source(&model->devices, parsed->source);
    if ((size_t)index == model->devices.count) {
        /* New device - need to append a slot. */
        if (model->devices.count >= N2K_DEVICE_MANAGER_MAX_DEVICES) {
            model->device_list_stats.malformed_device_list_messages++;
            return false;
        }
        if (model->devices.count >= model->devices.capacity) {
            if (!n2k_device_manager_grow(&model->devices)) {
                model->device_list_stats.malformed_device_list_messages++;
                return false;
            }
        }
    }

    entry = *parsed;
    entry.used = true;
    entry.source = parsed->source;
    entry.last_seen_ms = n2k_snapshot_last_seen_to_local(model,
                                                         parsed->last_seen_ms,
                                                         now_ms);
    if (((size_t)index < model->devices.count) && (model->devices.entries[index].first_seen_ms != 0u)) {
        entry.first_seen_ms = model->devices.entries[index].first_seen_ms;
    } else {
        entry.first_seen_ms = now_ms;
    }

    model->devices.entries[index] = entry;
    if ((size_t)index == model->devices.count) {
        model->devices.count++;
    }
    model->device_list.last_update_ms = now_ms;

    if ((uint8_t)(record_index + 1u) > model->device_list.received_records) {
        model->device_list.received_records = (uint8_t)(record_index + 1u);
    }
    if (total_records > model->device_list.expected_count) {
        model->device_list.expected_count = total_records;
    }
    return true;
}

bool n2k_app_model_device_list_end(N2kAppModel_t *model,
                                   uint16_t request_id,
                                   uint8_t end_count,
                                   uint32_t now_ms,
                                   uint8_t *out_final_count) {
    uint8_t count = 0u;

    if (model == NULL) {
        return false;
    }
    if (!model->device_list.in_progress || (model->device_list.request_id != request_id)) {
        model->device_list_stats.malformed_device_list_messages++;
        return false;
    }

    model->device_list.in_progress = false;
    model->device_list.complete = true;
    model->device_list.end_count = end_count;
    model->device_list.last_update_ms = now_ms;

    for (size_t i = 0; i < model->devices.count; i++) {
        count++;
    }

    if (out_final_count != NULL) {
        *out_final_count = count;
    }
    model->device_list_stats.completed_snapshots++;
    return true;
}

bool n2k_app_model_device_list_timeout_check(N2kAppModel_t *model,
                                             uint32_t now_ms,
                                             uint32_t timeout_ms,
                                             uint16_t *out_dropped_request_id) {
    uint32_t elapsed;

    if (model == NULL) {
        return false;
    }
    if (!model->device_list.in_progress) {
        return false;
    }

    elapsed = now_ms - model->device_list.last_update_ms;
    if (elapsed < timeout_ms) {
        return false;
    }

    if (out_dropped_request_id != NULL) {
        *out_dropped_request_id = model->device_list.request_id;
    }

    model->device_list.in_progress = false;
    model->device_list.complete = false;
    model->device_list_stats.dropped_snapshots++;
    return true;
}

void n2k_app_model_mark_device_list_malformed(N2kAppModel_t *model) {
    if (model == NULL) {
        return;
    }
    model->device_list_stats.malformed_device_list_messages++;
}
