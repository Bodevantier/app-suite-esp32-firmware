#include <assert.h>
#include <string.h>

#include "device_list_bridge.h"

static void write_u16_le(uint8_t *dst, uint16_t value) {
    dst[0] = (uint8_t)(value & 0xFFu);
    dst[1] = (uint8_t)((value >> 8u) & 0xFFu);
}

static void write_u32_le(uint8_t *dst, uint32_t value) {
    dst[0] = (uint8_t)(value & 0xFFu);
    dst[1] = (uint8_t)((value >> 8u) & 0xFFu);
    dst[2] = (uint8_t)((value >> 16u) & 0xFFu);
    dst[3] = (uint8_t)((value >> 24u) & 0xFFu);
}

static void write_u64_le(uint8_t *dst, uint64_t value) {
    for (uint8_t i = 0u; i < 8u; i++) {
        dst[i] = (uint8_t)((value >> (8u * i)) & 0xFFu);
    }
}

static void append_text_tlv(uint8_t *payload,
                            size_t *offset,
                            uint8_t type,
                            const char *text) {
    const size_t len = strlen(text);

    payload[(*offset)++] = type;
    write_u16_le(&payload[*offset], (uint16_t)len);
    *offset += 2u;
    memcpy(&payload[*offset], text, len);
    *offset += len;
}

static void append_pgn_tlv(uint8_t *payload,
                           size_t *offset,
                           uint8_t type,
                           const uint32_t *pgns,
                           uint8_t count) {
    payload[(*offset)++] = type;
    write_u16_le(&payload[*offset], (uint16_t)(count * 4u));
    *offset += 2u;

    for (uint8_t i = 0u; i < count; i++) {
        write_u32_le(&payload[*offset], pgns[i]);
        *offset += 4u;
    }
}

static SpiN2kPacket_t build_packet(const uint8_t *payload, uint8_t payload_len) {
    SpiN2kPacket_t packet;

    memset(&packet, 0, sizeof(packet));
    packet.pkt_type = SPI_N2K_PKT_TYPE_DEVICE_LIST;
    packet.payload_len = payload_len;
    memcpy(packet.payload, payload, payload_len);
    return packet;
}

static void test_request_payload_vector(void) {
    DeviceListRequestState_t state;
    uint8_t payload[9];
    uint16_t request_id = 0u;

    device_list_request_state_init(&state);
    assert(device_list_build_request_payload(&state,
                                             1000u,
                                             1u,
                                             15u,
                                             255u,
                                             60928u,
                                             payload,
                                             sizeof(payload),
                                             &request_id));
    assert(request_id == 1u);
    assert(payload[0] == 1u);
    assert(payload[1] == 1u);
    assert(payload[2] == 0u);
    assert(payload[3] == 15u);
    assert(payload[4] == 255u);
}

static void test_device_packet_vector(void) {
    static const uint32_t tx_pgns[] = {126996u, 126998u};
    DeviceListPacketHeader_t header;
    N2kDeviceEntry_t entry;
    uint8_t payload[128];
    uint8_t record_index = 0u;
    uint8_t total_records = 0u;
    size_t offset = 39u;
    SpiN2kPacket_t packet;

    memset(payload, 0, sizeof(payload));
    payload[0] = 1u;
    payload[1] = 2u;
    write_u16_le(&payload[2], 12u);
    write_u32_le(&payload[4], 5000u);
    payload[8] = 0u;
    payload[9] = 1u;
    payload[10] = 45u;
    write_u16_le(&payload[11], 0x003Fu);
    write_u64_le(&payload[13], 0x1122334455667788ULL);
    write_u32_le(&payload[21], 4900u);
    write_u32_le(&payload[25], 0x00054321u);
    write_u16_le(&payload[29], 275u);
    write_u16_le(&payload[31], 2001u);
    payload[33] = 130u;
    payload[34] = 25u;
    payload[35] = 1u;
    payload[36] = 0u;
    payload[37] = 4u;
    append_text_tlv(payload, &offset, 1u, "WS310");
    append_text_tlv(payload, &offset, 5u, "Airmar");
    append_text_tlv(payload, &offset, 6u, "Masthead Wind");
    append_pgn_tlv(payload, &offset, 8u, tx_pgns, 2u);
    payload[38] = 4u;

    packet = build_packet(payload, (uint8_t)offset);
    assert(device_list_parse_snapshot_device(&packet,
                                             1u,
                                             &header,
                                             &record_index,
                                             &total_records,
                                             &entry));
    assert(header.request_id == 12u);
    assert(record_index == 0u);
    assert(total_records == 1u);
    assert(entry.source == 45u);
    assert(entry.online);
    assert(entry.has_address_claim);
    assert(entry.has_product_info);
    assert(strcmp(entry.model_id, "WS310") == 0);
    assert(strcmp(entry.manufacturer_text, "Airmar") == 0);
    assert(strcmp(entry.installation_desc1, "Masthead Wind") == 0);
    assert(entry.tx_pgn_count == 2u);
    assert(entry.tx_pgn_list[0] == 126996u);
    assert(entry.tx_pgn_list[1] == 126998u);
}

static void test_begin_end_vectors(void) {
    DeviceListPacketHeader_t header;
    uint8_t begin_payload[] = {1u, 1u, 0x0Cu, 0x00u, 0x88u, 0x13u, 0x00u, 0x00u, 49u, 3u};
    uint8_t end_payload[] = {1u, 3u, 0x0Cu, 0x00u, 0x88u, 0x13u, 0x00u, 0x00u, 3u};
    SpiN2kPacket_t begin_packet = build_packet(begin_payload, sizeof(begin_payload));
    SpiN2kPacket_t end_packet = build_packet(end_payload, sizeof(end_payload));
    uint8_t own_source = 0u;
    uint8_t expected_count = 0u;
    uint8_t end_count = 0u;

    assert(device_list_parse_snapshot_begin(&begin_packet,
                                            1u,
                                            &header,
                                            &own_source,
                                            &expected_count));
    assert(header.request_id == 12u);
    assert(own_source == 49u);
    assert(expected_count == 3u);

    assert(device_list_parse_snapshot_end(&end_packet,
                                          1u,
                                          &header,
                                          &end_count));
    assert(end_count == 3u);
}

void device_list_bridge_run_self_tests(void) {
    test_request_payload_vector();
    test_device_packet_vector();
    test_begin_end_vectors();
}