#include "spi_n2k_transport.h"

#include <string.h>
#include "esp_log.h"

static const char *TAG = "SPI_XPORT";

static uint8_t spi_n2k_crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0u;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
    }
    return crc;
}

static bool spi_n2k_is_known_packet_type(uint8_t pkt_type) {
    return (pkt_type == SPI_N2K_PKT_TYPE_N2K_RX_FRAME) ||
           (pkt_type == SPI_N2K_PKT_TYPE_N2K_TX_FRAME) ||
           (pkt_type == SPI_N2K_PKT_TYPE_STATUS) ||
           (pkt_type == SPI_N2K_PKT_TYPE_BOAT_STATE) ||
           (pkt_type == SPI_N2K_PKT_TYPE_DEVICE_LIST) ||
           (pkt_type == SPI_N2K_PKT_TYPE_DEVICE_LIST_REQUEST);
}

static void spi_n2k_write_u32_le(uint8_t *dst, uint32_t value) {
    dst[0] = (uint8_t)(value & 0xFFu);
    dst[1] = (uint8_t)((value >> 8u) & 0xFFu);
    dst[2] = (uint8_t)((value >> 16u) & 0xFFu);
    dst[3] = (uint8_t)((value >> 24u) & 0xFFu);
}

static uint32_t spi_n2k_read_u32_le(const uint8_t *src) {
    return ((uint32_t)src[0]) |
           ((uint32_t)src[1] << 8u) |
           ((uint32_t)src[2] << 16u) |
           ((uint32_t)src[3] << 24u);
}

static void spi_n2k_parser_reset(SpiN2kTransportParser_t *parser) {
    parser->state = SPI_N2K_PARSE_WAIT_SOF1;
    parser->index = 0u;
    parser->expected_len = 0u;
}

void spi_n2k_transport_parser_init(SpiN2kTransportParser_t *parser) {
    if (parser == NULL) {
        return;
    }
    memset(parser, 0, sizeof(*parser));
    spi_n2k_parser_reset(parser);
}

bool spi_n2k_transport_parser_consume_byte(SpiN2kTransportParser_t *parser, uint8_t byte, SpiN2kPacket_t *out_packet) {
    uint8_t crc;

    if ((parser == NULL) || (out_packet == NULL)) {
        return false;
    }
    parser->stats.bytes_seen++;

    switch (parser->state) {
        case SPI_N2K_PARSE_WAIT_SOF1:
            if (byte == SPI_N2K_SOF1) {
                parser->buf[0] = byte;
                parser->index = 1u;
                parser->state = SPI_N2K_PARSE_WAIT_SOF2;
            } else {
                parser->stats.bad_sof++;
            }
            return false;

        case SPI_N2K_PARSE_WAIT_SOF2:
            if (byte == SPI_N2K_SOF2) {
                parser->buf[parser->index++] = byte;
                parser->state = SPI_N2K_PARSE_READ_TYPE;
            } else if (byte == SPI_N2K_SOF1) {
                parser->buf[0] = byte;
                parser->index = 1u;
            } else {
                parser->stats.bad_sof++;
                spi_n2k_parser_reset(parser);
            }
            return false;

        case SPI_N2K_PARSE_READ_TYPE:
            parser->buf[parser->index++] = byte;
            parser->state = SPI_N2K_PARSE_READ_LEN;
            return false;

        case SPI_N2K_PARSE_READ_LEN:
            parser->buf[parser->index++] = byte;
            if (byte > SPI_N2K_MAX_PAYLOAD_LEN) {
                ESP_LOGW(TAG, "SPI bad_len type=0x%02X len=%u", parser->buf[2], (unsigned)byte);
                parser->stats.bad_len++;
                spi_n2k_parser_reset(parser);
                return false;
            }
            parser->expected_len = 2u + 1u + 1u + (size_t)byte + 1u;
            if (parser->expected_len > SPI_N2K_MAX_PACKET_LEN) {
                parser->stats.bad_len++;
                spi_n2k_parser_reset(parser);
                return false;
            }
            parser->state = SPI_N2K_PARSE_READ_PAYLOAD_CRC;
            return false;

        case SPI_N2K_PARSE_READ_PAYLOAD_CRC:
            parser->buf[parser->index++] = byte;
            if (parser->index < parser->expected_len) {
                return false;
            }
            crc = spi_n2k_crc8(&parser->buf[2], 2u + parser->buf[3]);
            if (crc != parser->buf[parser->expected_len - 1u]) {
                ESP_LOGW(TAG, "SPI bad_crc type=0x%02X len=%u expected=0x%02X got=0x%02X",
                         parser->buf[2], (unsigned)parser->buf[3],
                         crc, parser->buf[parser->expected_len - 1u]);
                parser->stats.bad_crc++;
                spi_n2k_parser_reset(parser);
                return false;
            }

            out_packet->pkt_type = parser->buf[2];
            out_packet->payload_len = parser->buf[3];
            if (out_packet->payload_len > 0u) {
                memcpy(out_packet->payload, &parser->buf[4], out_packet->payload_len);
            }

            if (!spi_n2k_is_known_packet_type(out_packet->pkt_type)) {
                ESP_LOGW(TAG, "SPI unknown_type=0x%02X len=%u", out_packet->pkt_type, (unsigned)out_packet->payload_len);
                parser->stats.unknown_type++;
            }
            if (out_packet->pkt_type == SPI_N2K_PKT_TYPE_DEVICE_LIST ||
                out_packet->pkt_type == SPI_N2K_PKT_TYPE_DEVICE_LIST_REQUEST) {
                ESP_LOGI(TAG, "SPI parsed type=0x%02X len=%u ok=%lu",
                         out_packet->pkt_type, (unsigned)out_packet->payload_len,
                         (unsigned long)parser->stats.packets_ok + 1u);
            }
            parser->stats.packets_ok++;
            spi_n2k_parser_reset(parser);
            return true;
    }

    spi_n2k_parser_reset(parser);
    return false;
}

bool spi_n2k_transport_build_frame_packet(uint8_t pkt_type, const N2K_RawFrame_t *frame, uint8_t *out_buf, size_t out_buf_size, size_t *out_len) {
    size_t i = 0u;
    uint8_t dlc;

    if ((frame == NULL) || (out_buf == NULL) || (out_len == NULL)) {
        return false;
    }
    if (out_buf_size < (2u + 1u + 1u + SPI_N2K_FRAME_PAYLOAD_LEN + 1u)) {
        return false;
    }
    if ((pkt_type != SPI_N2K_PKT_TYPE_N2K_RX_FRAME) &&
        (pkt_type != SPI_N2K_PKT_TYPE_N2K_TX_FRAME)) {
        return false;
    }

    dlc = n2k_raw_frame_clamp_dlc(frame->dlc);
    out_buf[i++] = SPI_N2K_SOF1;
    out_buf[i++] = SPI_N2K_SOF2;
    out_buf[i++] = pkt_type;
    out_buf[i++] = SPI_N2K_FRAME_PAYLOAD_LEN;

    spi_n2k_write_u32_le(&out_buf[i], frame->timestamp_ms);
    i += 4u;
    spi_n2k_write_u32_le(&out_buf[i], frame->can_id);
    i += 4u;
    out_buf[i++] = dlc;
    out_buf[i++] = frame->flags;
    memcpy(&out_buf[i], frame->data, N2K_RAW_FRAME_MAX_DATA_LEN);
    i += N2K_RAW_FRAME_MAX_DATA_LEN;
    out_buf[i++] = spi_n2k_crc8(&out_buf[2], 2u + SPI_N2K_FRAME_PAYLOAD_LEN);

    *out_len = i;
    return true;
}

bool spi_n2k_transport_build_custom_packet(uint8_t pkt_type,
                                           const uint8_t *payload,
                                           uint8_t payload_len,
                                           uint8_t *out_buf,
                                           size_t out_buf_size,
                                           size_t *out_len) {
    size_t i = 0u;

    if ((out_buf == NULL) || (out_len == NULL)) {
        return false;
    }
    if ((payload_len > 0u) && (payload == NULL)) {
        return false;
    }
    if (payload_len > SPI_N2K_MAX_PAYLOAD_LEN) {
        return false;
    }
    if (out_buf_size < (2u + 1u + 1u + payload_len + 1u)) {
        return false;
    }

    out_buf[i++] = SPI_N2K_SOF1;
    out_buf[i++] = SPI_N2K_SOF2;
    out_buf[i++] = pkt_type;
    out_buf[i++] = payload_len;
    if (payload_len > 0u) {
        memcpy(&out_buf[i], payload, payload_len);
        i += payload_len;
    }
    out_buf[i++] = spi_n2k_crc8(&out_buf[2], 2u + payload_len);
    *out_len = i;
    return true;
}

bool spi_n2k_transport_parse_frame_payload(const SpiN2kPacket_t *packet, N2K_RawFrame_t *out_frame) {
    if ((packet == NULL) || (out_frame == NULL)) {
        return false;
    }
    if (packet->payload_len != SPI_N2K_FRAME_PAYLOAD_LEN) {
        return false;
    }
    if ((packet->pkt_type != SPI_N2K_PKT_TYPE_N2K_RX_FRAME) &&
        (packet->pkt_type != SPI_N2K_PKT_TYPE_N2K_TX_FRAME)) {
        return false;
    }

    out_frame->timestamp_ms = spi_n2k_read_u32_le(&packet->payload[0]);
    out_frame->can_id = spi_n2k_read_u32_le(&packet->payload[4]);
    out_frame->dlc = n2k_raw_frame_clamp_dlc(packet->payload[8]);
    out_frame->flags = packet->payload[9];
    memcpy(out_frame->data, &packet->payload[10], N2K_RAW_FRAME_MAX_DATA_LEN);
    return true;
}
