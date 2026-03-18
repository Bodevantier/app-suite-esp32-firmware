#ifndef N2K_REQUEST_MANAGER_H
#define N2K_REQUEST_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

#include "n2k_raw_frame.h"

typedef bool (*N2kRequestTxFn)(const N2K_RawFrame_t *frame, void *ctx);

typedef struct {
    N2kRequestTxFn tx_fn;
    void *tx_ctx;
    uint8_t local_source;
} N2kRequestManager_t;

void n2k_request_manager_init(N2kRequestManager_t *manager, uint8_t local_source, N2kRequestTxFn tx_fn, void *tx_ctx);
bool n2k_request_manager_build_pgn_59904(uint8_t local_source, uint8_t destination, uint32_t requested_pgn, N2K_RawFrame_t *out_frame);
bool n2k_request_manager_send_request(N2kRequestManager_t *manager, uint8_t destination, uint32_t requested_pgn, uint32_t now_ms);

#endif
