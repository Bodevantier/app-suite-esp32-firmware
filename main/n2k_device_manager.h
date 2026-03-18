#ifndef N2K_DEVICE_MANAGER_H
#define N2K_DEVICE_MANAGER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define N2K_DEVICE_MANAGER_MAX_DEVICES 32u
#define N2K_DEVICE_MODEL_MAX_LEN 33u
#define N2K_DEVICE_SW_VER_MAX_LEN 33u
#define N2K_DEVICE_MODEL_ID_MAX_LEN 33u
#define N2K_DEVICE_SERIAL_MAX_LEN 33u
#define N2K_DEVICE_CONFIG_MAX_LEN 71u
#define N2K_DEVICE_SUPPORTED_PGNS_MAX 96u

typedef struct {
    bool used;
    uint8_t source;
    uint64_t name;
    uint32_t unique_number;
    uint16_t manufacturer_code;
    uint8_t device_instance_lower;
    uint8_t device_instance_upper;
    uint8_t device_function;
    uint8_t device_class;
    uint8_t system_instance;
    uint32_t n2k_version;
    uint16_t product_code;
    bool has_address_claim;
    bool has_product_info;
    bool has_configuration_info;
    bool has_tx_pgn_list;
    bool has_rx_pgn_list;
    char model_id[N2K_DEVICE_MODEL_ID_MAX_LEN];
    char model[N2K_DEVICE_MODEL_MAX_LEN];
    char software_version[N2K_DEVICE_SW_VER_MAX_LEN];
    char serial_code[N2K_DEVICE_SERIAL_MAX_LEN];
    char installation_desc1[N2K_DEVICE_CONFIG_MAX_LEN];
    char installation_desc2[N2K_DEVICE_CONFIG_MAX_LEN];
    char manufacturer_text[N2K_DEVICE_CONFIG_MAX_LEN];
    uint8_t certification_level;
    uint8_t load_equivalency;
    uint8_t address_claim_request_count;
    uint8_t product_request_count;
    uint8_t configuration_request_count;
    uint8_t pgn_list_request_count;
    uint8_t supported_pgn_count;
    uint32_t supported_pgns[N2K_DEVICE_SUPPORTED_PGNS_MAX];
    uint32_t first_seen_ms;
    uint32_t last_seen_ms;
    uint32_t last_address_claim_request_ms;
    uint32_t last_product_request_ms;
    uint32_t last_configuration_request_ms;
    uint32_t last_pgn_list_request_ms;
} N2kDeviceEntry_t;

typedef struct {
    N2kDeviceEntry_t entries[N2K_DEVICE_MANAGER_MAX_DEVICES];
    uint32_t drop_count;
} N2kDeviceManager_t;

void n2k_device_manager_init(N2kDeviceManager_t *manager);
void n2k_device_manager_update_last_seen(N2kDeviceManager_t *manager, uint8_t source, uint32_t now_ms);
void n2k_device_manager_handle_address_claim(N2kDeviceManager_t *manager, uint8_t source, uint32_t now_ms, const uint8_t *data, size_t len);
void n2k_device_manager_handle_product_info(N2kDeviceManager_t *manager, uint8_t source, uint32_t now_ms, const uint8_t *data, size_t len);
void n2k_device_manager_handle_configuration_info(N2kDeviceManager_t *manager, uint8_t source, uint32_t now_ms, const uint8_t *data, size_t len);
void n2k_device_manager_handle_supported_pgn_list(N2kDeviceManager_t *manager, uint8_t source, uint32_t now_ms, const uint8_t *data, size_t len);
size_t n2k_device_manager_count(const N2kDeviceManager_t *manager);
const N2kDeviceEntry_t *n2k_device_manager_entries(const N2kDeviceManager_t *manager);

#endif
