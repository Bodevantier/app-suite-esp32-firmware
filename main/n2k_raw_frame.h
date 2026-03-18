#ifndef N2K_RAW_FRAME_H
#define N2K_RAW_FRAME_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define N2K_RAW_FRAME_MAX_DATA_LEN 8u

#define N2K_RAW_FLAG_EXTENDED_ID 0x01u
#define N2K_RAW_FLAG_RTR 0x02u
#define N2K_RAW_FLAG_DIRECTION_RX 0x04u

typedef struct {
    uint32_t timestamp_ms;
    uint32_t can_id;
    uint8_t dlc;
    uint8_t data[N2K_RAW_FRAME_MAX_DATA_LEN];
    uint8_t flags;
} N2K_RawFrame_t;

void n2k_raw_frame_clear(N2K_RawFrame_t *frame);
uint8_t n2k_raw_frame_clamp_dlc(uint8_t dlc);
bool n2k_raw_frame_is_extended(const N2K_RawFrame_t *frame);
void n2k_raw_frame_payload_hex(const N2K_RawFrame_t *frame, char *out, size_t out_sz);

#endif
