#include "device_list_bridge.h"

#include <string.h>

#define DEVICE_LIST_MSG_BEGIN 1u
#define DEVICE_LIST_MSG_DEVICE 2u
#define DEVICE_LIST_MSG_END 3u

#define DEVICE_LIST_FIELD_MODEL_ID 1u
#define DEVICE_LIST_FIELD_SOFTWARE_VERSION 2u
#define DEVICE_LIST_FIELD_MODEL_VERSION 3u
#define DEVICE_LIST_FIELD_SERIAL_CODE 4u
#define DEVICE_LIST_FIELD_MANUFACTURER_TEXT 5u
#define DEVICE_LIST_FIELD_INSTALLATION_1 6u
#define DEVICE_LIST_FIELD_INSTALLATION_2 7u
#define DEVICE_LIST_FIELD_TX_PGN_LIST 8u
#define DEVICE_LIST_FIELD_RX_PGN_LIST 9u

#define DEVICE_LIST_HEADER_LEN 8u
#define DEVICE_LIST_BEGIN_LEN 10u
#define DEVICE_LIST_DEVICE_FIXED_LEN 39u
#define DEVICE_LIST_END_LEN 9u

static uint16_t read_u16_le(const uint8_t *src) {
    return (uint16_t)((uint16_t)src[0] | ((uint16_t)src[1] << 8u));
}

static uint32_t read_u32_le(const uint8_t *src) {
    return ((uint32_t)src[0]) |
           ((uint32_t)src[1] << 8u) |
           ((uint32_t)src[2] << 16u) |
           ((uint32_t)src[3] << 24u);
}

static uint64_t read_u64_le(const uint8_t *src) {
    uint64_t value = 0u;

    for (uint8_t i = 0u; i < 8u; i++) {
        value |= ((uint64_t)src[i]) << (8u * i);
    }

    return value;
}

static void copy_text_field(char *dst, size_t dst_len, const uint8_t *src, uint16_t src_len) {
    size_t copy_len;

    if ((dst == NULL) || (dst_len == 0u)) {
        return;
    }

    copy_len = src_len;
    if (copy_len >= dst_len) {
        copy_len = dst_len - 1u;
    }

    if ((src != NULL) && (copy_len > 0u)) {
        memcpy(dst, src, copy_len);
    }
    dst[copy_len] = '\0';
}

static uint8_t copy_pgn_list(uint32_t *dst, const uint8_t *src, uint16_t src_len) {
    uint8_t count;

    if ((dst == NULL) || (src == NULL) || (src_len < 4u)) {
        return 0u;
    }

    count = (uint8_t)(src_len / 4u);
    if (count > N2K_DEVICE_PGN_LIST_MAX) {
        count = N2K_DEVICE_PGN_LIST_MAX;
    }

    for (uint8_t i = 0u; i < count; i++) {
        dst[i] = read_u32_le(&src[(size_t)i * 4u]);
    }

    return count;
}

bool device_list_parse_packet_header(const SpiN2kPacket_t *packet,
                                     uint8_t protocol_version,
                                     DeviceListPacketHeader_t *out_header) {
    if ((packet == NULL) || (out_header == NULL)) {
        return false;
    }
    if (packet->payload_len < DEVICE_LIST_HEADER_LEN) {
        return false;
    }
    if (packet->payload[0] != protocol_version) {
        return false;
    }

    out_header->message_type = packet->payload[1];
    out_header->request_id = read_u16_le(&packet->payload[2]);
    out_header->snapshot_time_ms = read_u32_le(&packet->payload[4]);
    return true;
}

bool device_list_parse_snapshot_begin(const SpiN2kPacket_t *packet,
                                      uint8_t protocol_version,
                                      DeviceListPacketHeader_t *out_header,
                                      uint8_t *out_own_source,
                                      uint8_t *out_expected_count) {
    DeviceListPacketHeader_t header;

    if ((packet == NULL) || (out_own_source == NULL) || (out_expected_count == NULL)) {
        return false;
    }
    if (!device_list_parse_packet_header(packet, protocol_version, &header)) {
        return false;
    }
    if ((header.message_type != DEVICE_LIST_MSG_BEGIN) ||
        (packet->payload_len < DEVICE_LIST_BEGIN_LEN)) {
        return false;
    }

    if (out_header != NULL) {
        *out_header = header;
    }
    *out_own_source = packet->payload[8];
    *out_expected_count = packet->payload[9];
    return true;
}

bool device_list_parse_snapshot_device(const SpiN2kPacket_t *packet,
                                       uint8_t protocol_version,
                                       DeviceListPacketHeader_t *out_header,
                                       uint8_t *out_record_index,
                                       uint8_t *out_total_records,
                                       N2kDeviceEntry_t *out_entry) {
    DeviceListPacketHeader_t header;
    uint16_t offset;
    uint8_t field_count;

    if ((packet == NULL) || (out_record_index == NULL) ||
        (out_total_records == NULL) || (out_entry == NULL)) {
        return false;
    }
    if (!device_list_parse_packet_header(packet, protocol_version, &header)) {
        return false;
    }
    if ((header.message_type != DEVICE_LIST_MSG_DEVICE) ||
        (packet->payload_len < DEVICE_LIST_DEVICE_FIXED_LEN)) {
        return false;
    }

    memset(out_entry, 0, sizeof(*out_entry));
    out_entry->used = true;
    out_entry->source = packet->payload[10];
    out_entry->flags = read_u16_le(&packet->payload[11]);
    out_entry->name = read_u64_le(&packet->payload[13]);
    out_entry->last_seen_ms = read_u32_le(&packet->payload[21]);
    out_entry->unique_number = read_u32_le(&packet->payload[25]);
    out_entry->manufacturer_code = read_u16_le(&packet->payload[29]);
    out_entry->product_code = read_u16_le(&packet->payload[31]);
    out_entry->device_function = packet->payload[33];
    out_entry->device_class = packet->payload[34];
    out_entry->device_instance = packet->payload[35];
    out_entry->system_instance = packet->payload[36];
    out_entry->industry_group = packet->payload[37];
    out_entry->online = (out_entry->flags & N2K_DEVICE_FLAG_ONLINE) != 0u;
    out_entry->has_address_claim = (out_entry->flags & N2K_DEVICE_FLAG_HAS_ADDRESS_CLAIM) != 0u;
    out_entry->has_product_info = (out_entry->flags & N2K_DEVICE_FLAG_HAS_PRODUCT_INFO) != 0u;
    out_entry->has_configuration_info = (out_entry->flags & N2K_DEVICE_FLAG_HAS_CONFIGURATION_INFO) != 0u;
    out_entry->has_tx_pgn_list = (out_entry->flags & N2K_DEVICE_FLAG_HAS_TX_PGN_LIST) != 0u;
    out_entry->has_rx_pgn_list = (out_entry->flags & N2K_DEVICE_FLAG_HAS_RX_PGN_LIST) != 0u;

    field_count = packet->payload[38];
    offset = DEVICE_LIST_DEVICE_FIXED_LEN;
    for (uint8_t i = 0u; i < field_count; i++) {
        uint8_t field_type;
        uint16_t field_len;

        if ((uint16_t)(offset + 3u) > packet->payload_len) {
            return false;
        }

        field_type = packet->payload[offset++];
        field_len = read_u16_le(&packet->payload[offset]);
        offset = (uint16_t)(offset + 2u);
        if ((uint16_t)(offset + field_len) > packet->payload_len) {
            return false;
        }

        switch (field_type) {
            case DEVICE_LIST_FIELD_MODEL_ID:
                copy_text_field(out_entry->model_id,
                                sizeof(out_entry->model_id),
                                &packet->payload[offset],
                                field_len);
                break;

            case DEVICE_LIST_FIELD_SOFTWARE_VERSION:
                copy_text_field(out_entry->software_version,
                                sizeof(out_entry->software_version),
                                &packet->payload[offset],
                                field_len);
                break;

            case DEVICE_LIST_FIELD_MODEL_VERSION:
                copy_text_field(out_entry->model,
                                sizeof(out_entry->model),
                                &packet->payload[offset],
                                field_len);
                break;

            case DEVICE_LIST_FIELD_SERIAL_CODE:
                copy_text_field(out_entry->serial_code,
                                sizeof(out_entry->serial_code),
                                &packet->payload[offset],
                                field_len);
                break;

            case DEVICE_LIST_FIELD_MANUFACTURER_TEXT:
                copy_text_field(out_entry->manufacturer_text,
                                sizeof(out_entry->manufacturer_text),
                                &packet->payload[offset],
                                field_len);
                break;

            case DEVICE_LIST_FIELD_INSTALLATION_1:
                copy_text_field(out_entry->installation_desc1,
                                sizeof(out_entry->installation_desc1),
                                &packet->payload[offset],
                                field_len);
                break;

            case DEVICE_LIST_FIELD_INSTALLATION_2:
                copy_text_field(out_entry->installation_desc2,
                                sizeof(out_entry->installation_desc2),
                                &packet->payload[offset],
                                field_len);
                break;

            case DEVICE_LIST_FIELD_TX_PGN_LIST:
                out_entry->tx_pgn_count = copy_pgn_list(out_entry->tx_pgn_list,
                                                        &packet->payload[offset],
                                                        field_len);
                break;

            case DEVICE_LIST_FIELD_RX_PGN_LIST:
                out_entry->rx_pgn_count = copy_pgn_list(out_entry->rx_pgn_list,
                                                        &packet->payload[offset],
                                                        field_len);
                break;

            default:
                break;
        }

        offset = (uint16_t)(offset + field_len);
    }

    if (out_header != NULL) {
        *out_header = header;
    }
    *out_record_index = packet->payload[8];
    *out_total_records = packet->payload[9];
    return true;
}

bool device_list_parse_snapshot_end(const SpiN2kPacket_t *packet,
                                    uint8_t protocol_version,
                                    DeviceListPacketHeader_t *out_header,
                                    uint8_t *out_end_count) {
    DeviceListPacketHeader_t header;

    if ((packet == NULL) || (out_end_count == NULL)) {
        return false;
    }
    if (!device_list_parse_packet_header(packet, protocol_version, &header)) {
        return false;
    }
    if ((header.message_type != DEVICE_LIST_MSG_END) ||
        (packet->payload_len < DEVICE_LIST_END_LEN)) {
        return false;
    }

    if (out_header != NULL) {
        *out_header = header;
    }
    *out_end_count = packet->payload[8];
    return true;
}

void device_list_request_state_init(DeviceListRequestState_t *state) {
    if (state == NULL) {
        return;
    }
    state->next_request_id = 1u;
    state->pending_request_id = 0u;
    state->pending = false;
    state->last_request_ms = 0u;
}

bool device_list_build_request_payload(DeviceListRequestState_t *state,
                                       uint32_t now_ms,
                                       uint8_t protocol_version,
                                       uint8_t src,
                                       uint8_t dst,
                                       uint32_t pgn,
                                       uint8_t *out_payload,
                                       size_t out_payload_len,
                                       uint16_t *out_request_id) {
    uint16_t request_id;

    if ((state == NULL) || (out_payload == NULL) || (out_payload_len < 9u) || (out_request_id == NULL)) {
        return false;
    }

    request_id = state->next_request_id;
    if (request_id == 0u) {
        request_id = 1u;
    }

    out_payload[0] = protocol_version;
    out_payload[1] = (uint8_t)(request_id & 0xFFu);
    out_payload[2] = (uint8_t)((request_id >> 8u) & 0xFFu);
    out_payload[3] = src;
    out_payload[4] = dst;
    out_payload[5] = (uint8_t)(pgn & 0xFFu);
    out_payload[6] = (uint8_t)((pgn >> 8u) & 0xFFu);
    out_payload[7] = (uint8_t)((pgn >> 16u) & 0xFFu);
    out_payload[8] = 0u;

    state->pending = true;
    state->pending_request_id = request_id;
    state->last_request_ms = now_ms;
    state->next_request_id = (uint16_t)(request_id + 1u);
    if (state->next_request_id == 0u) {
        state->next_request_id = 1u;
    }

    *out_request_id = request_id;
    return true;
}

bool device_list_parse_packet_event(DeviceListRequestState_t *state,
                                    uint32_t now_ms,
                                    const SpiN2kPacket_t *packet,
                                    uint8_t protocol_version,
                                    DeviceListBridgeEvent_t *out_event) {
    DeviceListPacketHeader_t header;
    uint8_t msg_type;
    uint16_t request_id;

    if ((state == NULL) || (packet == NULL) || (out_event == NULL)) {
        return false;
    }

    out_event->type = DEVICE_LIST_BRIDGE_EVENT_NONE;
    out_event->request_id = 0u;
    out_event->expected_count = 0u;
    out_event->record_index = 0u;
    out_event->total_records = 0u;
    out_event->source = 0u;
    out_event->end_count = 0u;

    if (!device_list_parse_packet_header(packet, protocol_version, &header)) {
        return false;
    }

    msg_type = header.message_type;
    request_id = header.request_id;
    out_event->request_id = request_id;

    if (msg_type == DEVICE_LIST_MSG_BEGIN) {
        if (packet->payload_len < DEVICE_LIST_BEGIN_LEN) {
            return false;
        }
        out_event->type = DEVICE_LIST_BRIDGE_EVENT_BEGIN;
        out_event->expected_count = packet->payload[9];
        state->pending = true;
        state->pending_request_id = request_id;
        state->last_request_ms = now_ms;
        return true;
    }

    if (msg_type == DEVICE_LIST_MSG_DEVICE) {
        if (packet->payload_len < 11u) {
            return false;
        }
        out_event->type = DEVICE_LIST_BRIDGE_EVENT_DEVICE;
        out_event->record_index = packet->payload[8];
        out_event->total_records = packet->payload[9];
        out_event->source = packet->payload[10];
        state->last_request_ms = now_ms;
        return true;
    }

    if (msg_type == DEVICE_LIST_MSG_END) {
        if (packet->payload_len < DEVICE_LIST_END_LEN) {
            return false;
        }
        out_event->type = DEVICE_LIST_BRIDGE_EVENT_COMPLETE;
        out_event->end_count = packet->payload[8];
        state->pending = false;
        return true;
    }

    return false;
}

bool device_list_take_timeout(DeviceListRequestState_t *state,
                              uint32_t now_ms,
                              uint32_t timeout_ms,
                              uint16_t *out_request_id) {
    if (state == NULL) {
        return false;
    }
    if (!state->pending) {
        return false;
    }
    if ((now_ms - state->last_request_ms) < timeout_ms) {
        return false;
    }

    if (out_request_id != NULL) {
        *out_request_id = state->pending_request_id;
    }
    state->pending = false;
    return true;
}
