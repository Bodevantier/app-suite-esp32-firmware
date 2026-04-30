#ifndef SPI_N2K_TRANSPORT_H
#define SPI_N2K_TRANSPORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "n2k_raw_frame.h"

#define SPI_N2K_SOF1 0xA5u
#define SPI_N2K_SOF2 0x5Au

#define SPI_N2K_PKT_TYPE_N2K_RX_FRAME 0x01u
#define SPI_N2K_PKT_TYPE_N2K_TX_FRAME 0x02u
#define SPI_N2K_PKT_TYPE_STATUS 0x03u
#define SPI_N2K_PKT_TYPE_DEVICE_LIST 0x10u
#define SPI_N2K_PKT_TYPE_DEVICE_LIST_REQUEST 0x11u
#define SPI_N2K_PKT_TYPE_DEVICE_FORGET 0x12u

#define SPI_N2K_FRAME_PAYLOAD_LEN 18u
#define SPI_N2K_MAX_PAYLOAD_LEN 250u
#define SPI_N2K_MAX_PACKET_LEN (2u + 1u + 1u + SPI_N2K_MAX_PAYLOAD_LEN + 1u)

typedef struct {
    uint8_t pkt_type;
    uint8_t payload_len;
    uint8_t payload[SPI_N2K_MAX_PAYLOAD_LEN];
} SpiN2kPacket_t;

typedef struct {
    uint32_t packets_ok;
    uint32_t bytes_seen;
    uint32_t bad_sof;
    uint32_t bad_len;
    uint32_t bad_crc;
    uint32_t unknown_type;
} SpiN2kTransportStats_t;

typedef enum {
    SPI_N2K_PARSE_WAIT_SOF1 = 0,
    SPI_N2K_PARSE_WAIT_SOF2,
    SPI_N2K_PARSE_READ_TYPE,
    SPI_N2K_PARSE_READ_LEN,
    SPI_N2K_PARSE_READ_PAYLOAD_CRC
} SpiN2kParseState_t;

typedef struct {
    SpiN2kParseState_t state;
    uint8_t buf[SPI_N2K_MAX_PACKET_LEN];
    size_t index;
    size_t expected_len;
    SpiN2kTransportStats_t stats;
} SpiN2kTransportParser_t;

void spi_n2k_transport_parser_init(SpiN2kTransportParser_t *parser);
bool spi_n2k_transport_parser_consume_byte(SpiN2kTransportParser_t *parser, uint8_t byte, SpiN2kPacket_t *out_packet);

bool spi_n2k_transport_build_frame_packet(uint8_t pkt_type, const N2K_RawFrame_t *frame, uint8_t *out_buf, size_t out_buf_size, size_t *out_len);
bool spi_n2k_transport_build_custom_packet(uint8_t pkt_type,
                                           const uint8_t *payload,
                                           uint8_t payload_len,
                                           uint8_t *out_buf,
                                           size_t out_buf_size,
                                           size_t *out_len);
bool spi_n2k_transport_parse_frame_payload(const SpiN2kPacket_t *packet, N2K_RawFrame_t *out_frame);

#endif
