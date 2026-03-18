#ifndef N2K_DECODER_H
#define N2K_DECODER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "n2k_app_model.h"
#include "n2k_can_id.h"
#include "n2k_raw_frame.h"

#define N2K_FAST_PACKET_MAX_SESSIONS 8u
#define N2K_FAST_PACKET_MAX_LEN 223u

typedef struct {
    bool complete;
    uint8_t source;
    uint32_t pgn;
    uint8_t data[N2K_FAST_PACKET_MAX_LEN];
    uint16_t len;
} N2kFastPacketMessage_t;

typedef struct {
    bool used;
    uint8_t source;
    uint32_t pgn;
    uint8_t sequence_id;
    uint8_t next_frame_counter;
    uint16_t total_len;
    uint16_t received_len;
    uint32_t last_update_ms;
    uint8_t data[N2K_FAST_PACKET_MAX_LEN];
} N2kFastPacketSession_t;

typedef struct {
    uint32_t frames_total;
    uint32_t frames_extended;
    uint32_t frames_ignored;
    uint32_t fast_packet_completed;
    uint32_t fast_packet_dropped;
} N2kDecoderStats_t;

typedef struct {
    N2kAppModel_t *app_model;
    N2kFastPacketSession_t fp_sessions[N2K_FAST_PACKET_MAX_SESSIONS];
    N2kDecoderStats_t stats;
} N2kDecoder_t;

void n2k_decoder_init(N2kDecoder_t *decoder, N2kAppModel_t *model);
void n2k_decoder_process_frame(N2kDecoder_t *decoder, const N2K_RawFrame_t *frame);

#endif
