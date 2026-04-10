#ifndef DEVICE_LIST_BRIDGE_H
#define DEVICE_LIST_BRIDGE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "n2k_app_model.h"
#include "spi_n2k_transport.h"

typedef struct {
    uint16_t next_request_id;
    uint16_t pending_request_id;
    bool pending;
    uint32_t last_request_ms;
} DeviceListRequestState_t;

typedef enum {
    DEVICE_LIST_BRIDGE_EVENT_NONE = 0,
    DEVICE_LIST_BRIDGE_EVENT_BEGIN,
    DEVICE_LIST_BRIDGE_EVENT_DEVICE,
    DEVICE_LIST_BRIDGE_EVENT_COMPLETE,
} DeviceListBridgeEventType_t;

typedef struct {
    DeviceListBridgeEventType_t type;
    uint16_t request_id;
    uint8_t expected_count;
    uint8_t record_index;
    uint8_t total_records;
    uint8_t source;
    uint8_t end_count;
} DeviceListBridgeEvent_t;

typedef struct {
    uint8_t message_type;
    uint16_t request_id;
    uint32_t snapshot_time_ms;
} DeviceListPacketHeader_t;

void device_list_request_state_init(DeviceListRequestState_t *state);

bool device_list_build_request_payload(DeviceListRequestState_t *state,
                                       uint32_t now_ms,
                                       uint8_t protocol_version,
                                       uint8_t src,
                                       uint8_t dst,
                                       uint32_t pgn,
                                       uint8_t *out_payload,
                                       size_t out_payload_len,
                                       uint16_t *out_request_id);

bool device_list_parse_packet_event(DeviceListRequestState_t *state,
                                    uint32_t now_ms,
                                    const SpiN2kPacket_t *packet,
                                    uint8_t protocol_version,
                                    DeviceListBridgeEvent_t *out_event);

bool device_list_parse_packet_header(const SpiN2kPacket_t *packet,
                                     uint8_t protocol_version,
                                     DeviceListPacketHeader_t *out_header);

bool device_list_parse_snapshot_begin(const SpiN2kPacket_t *packet,
                                      uint8_t protocol_version,
                                      DeviceListPacketHeader_t *out_header,
                                      uint8_t *out_own_source,
                                      uint8_t *out_expected_count);

bool device_list_parse_snapshot_device(const SpiN2kPacket_t *packet,
                                       uint8_t protocol_version,
                                       DeviceListPacketHeader_t *out_header,
                                       uint8_t *out_record_index,
                                       uint8_t *out_total_records,
                                       N2kDeviceEntry_t *out_entry);

bool device_list_parse_snapshot_end(const SpiN2kPacket_t *packet,
                                    uint8_t protocol_version,
                                    DeviceListPacketHeader_t *out_header,
                                    uint8_t *out_end_count);

bool device_list_take_timeout(DeviceListRequestState_t *state,
                              uint32_t now_ms,
                              uint32_t timeout_ms,
                              uint16_t *out_request_id);

#endif
