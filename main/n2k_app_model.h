#ifndef N2K_APP_MODEL_H
#define N2K_APP_MODEL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "n2k_raw_frame.h"

#define N2K_APP_RAW_RING_SIZE 32u
/* Hard cap — a NMEA2000 bus cannot have more than 254 addresses, but in practice
   you'll never have more than a handful of devices. Memory is allocated on the
   heap and grows only as devices are actually discovered. */
#define N2K_DEVICE_MANAGER_MAX_DEVICES 32u
#define N2K_DEVICE_TEXT_MAX_LEN 72u
#define N2K_DEVICE_PGN_LIST_MAX 64u

#define N2K_DEVICE_FLAG_ONLINE 0x0001u
#define N2K_DEVICE_FLAG_HAS_ADDRESS_CLAIM 0x0002u
#define N2K_DEVICE_FLAG_HAS_PRODUCT_INFO 0x0004u
#define N2K_DEVICE_FLAG_HAS_CONFIGURATION_INFO 0x0008u
#define N2K_DEVICE_FLAG_HAS_TX_PGN_LIST 0x0010u
#define N2K_DEVICE_FLAG_HAS_RX_PGN_LIST 0x0020u

typedef struct {
    bool valid;
    uint8_t source;
    float speed_mps;
    float angle_deg;
    uint8_t reference;
    uint32_t updated_ms;
} N2kWindData_t;

typedef struct {
    bool valid;
    float heading_deg;
    uint32_t updated_ms;
} N2kHeadingData_t;

typedef struct {
    bool valid;
    double latitude_deg;
    double longitude_deg;
    uint32_t updated_ms;
} N2kPositionData_t;

typedef struct {
    bool valid;
    uint8_t instance;
    float voltage_v;
    float current_a;
    float temperature_c;
    uint32_t updated_ms;
} N2kBatteryData_t;

typedef struct {
    bool used;
    uint8_t source;
    uint16_t flags;
    uint64_t name;
    uint32_t first_seen_ms;
    uint32_t last_seen_ms;
    uint32_t unique_number;
    uint16_t manufacturer_code;
    uint16_t product_code;
    uint8_t device_function;
    uint8_t device_class;
    uint8_t device_instance;
    uint8_t system_instance;
    uint8_t industry_group;

    bool online;
    bool has_address_claim;
    bool has_product_info;
    bool has_configuration_info;
    bool has_tx_pgn_list;
    bool has_rx_pgn_list;

    char model_id[N2K_DEVICE_TEXT_MAX_LEN];
    char software_version[N2K_DEVICE_TEXT_MAX_LEN];
    char model[N2K_DEVICE_TEXT_MAX_LEN];
    char serial_code[N2K_DEVICE_TEXT_MAX_LEN];
    char manufacturer_text[N2K_DEVICE_TEXT_MAX_LEN];
    char installation_desc1[N2K_DEVICE_TEXT_MAX_LEN];
    char installation_desc2[N2K_DEVICE_TEXT_MAX_LEN];

    uint32_t tx_pgn_list[N2K_DEVICE_PGN_LIST_MAX];
    uint8_t tx_pgn_count;
    uint32_t rx_pgn_list[N2K_DEVICE_PGN_LIST_MAX];
    uint8_t rx_pgn_count;

    /* Compatibility field used by BLE/device-list summary output. */
    uint32_t supported_pgns[N2K_DEVICE_PGN_LIST_MAX];
    uint8_t supported_pgn_count;
} N2kDeviceEntry_t;

typedef struct {
    N2kDeviceEntry_t *entries; /* heap-allocated; grows as devices are discovered */
    uint8_t count;             /* number of valid entries currently stored */
    uint8_t capacity;          /* number of slots allocated */
} N2kDeviceManager_t;

typedef struct {
    bool in_progress;
    bool complete;
    uint16_t request_id;
    uint32_t snapshot_time_ms;
    uint8_t own_source;
    uint8_t expected_count;
    uint8_t end_count;
    uint8_t received_records;
    uint32_t started_ms;
    uint32_t last_update_ms;
} N2kDeviceListSnapshot_t;

typedef struct {
    uint32_t spi_parse_errors;
    uint32_t unknown_packet_types;
    uint32_t malformed_device_list_messages;
    uint32_t completed_snapshots;
    uint32_t dropped_snapshots;
} N2kDeviceListStats_t;

typedef struct {
    N2kWindData_t wind;
    N2kHeadingData_t heading;
    N2kPositionData_t position;
    N2kBatteryData_t battery;
    N2kDeviceManager_t devices;
    N2kDeviceListSnapshot_t device_list;
    N2kDeviceListStats_t device_list_stats;
    N2K_RawFrame_t raw_ring[N2K_APP_RAW_RING_SIZE];
    size_t raw_write_index;
    uint32_t unknown_pgn_count;
} N2kAppModel_t;

void n2k_device_manager_init(N2kDeviceManager_t *manager);
const N2kDeviceEntry_t *n2k_device_manager_entries(const N2kDeviceManager_t *manager);

void n2k_app_model_init(N2kAppModel_t *model);
void n2k_app_model_store_raw(N2kAppModel_t *model, const N2K_RawFrame_t *frame);

void n2k_app_model_note_spi_parser_stats(N2kAppModel_t *model,
                                         uint32_t bad_sof,
                                         uint32_t bad_len,
                                         uint32_t bad_crc,
                                         uint32_t unknown_type);

bool n2k_app_model_device_list_begin(N2kAppModel_t *model,
                                     uint16_t request_id,
                                     uint32_t snapshot_time_ms,
                                     uint8_t own_source,
                                     uint8_t expected_count,
                                     uint32_t now_ms);

bool n2k_app_model_device_list_add_device(N2kAppModel_t *model,
                                          uint16_t request_id,
                                          uint8_t record_index,
                                          uint8_t total_records,
                                          const N2kDeviceEntry_t *parsed,
                                          uint32_t now_ms);

bool n2k_app_model_device_list_end(N2kAppModel_t *model,
                                   uint16_t request_id,
                                   uint8_t end_count,
                                   uint32_t now_ms,
                                   uint8_t *out_final_count);

bool n2k_app_model_device_list_timeout_check(N2kAppModel_t *model,
                                             uint32_t now_ms,
                                             uint32_t timeout_ms,
                                             uint16_t *out_dropped_request_id);

void n2k_app_model_mark_device_list_malformed(N2kAppModel_t *model);

#endif
