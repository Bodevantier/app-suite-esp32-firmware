#ifndef BLE_TEXT_PROTOCOL_H
#define BLE_TEXT_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#include "n2k_app_model.h"

#ifdef __cplusplus
extern "C" {
#endif

bool ble_text_command_is_request_device_list(const char *command_line);

int ble_text_send_snapshot(uint16_t conn_handle,
                           const N2kAppModel_t *model,
                           uint8_t local_source,
                           uint32_t now_ms,
                           uint32_t online_window_ms,
                           uint32_t stale_hide_ms);

#ifdef __cplusplus
}
#endif

#endif
