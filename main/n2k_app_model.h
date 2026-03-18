#ifndef N2K_APP_MODEL_H
#define N2K_APP_MODEL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "n2k_device_manager.h"
#include "n2k_raw_frame.h"

#define N2K_APP_RAW_RING_SIZE 32u

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
    N2kWindData_t wind;
    N2kHeadingData_t heading;
    N2kPositionData_t position;
    N2kBatteryData_t battery;
    N2kDeviceManager_t devices;
    N2K_RawFrame_t raw_ring[N2K_APP_RAW_RING_SIZE];
    size_t raw_write_index;
    uint32_t unknown_pgn_count;
} N2kAppModel_t;

void n2k_app_model_init(N2kAppModel_t *model);
void n2k_app_model_store_raw(N2kAppModel_t *model, const N2K_RawFrame_t *frame);

#endif
