#include "n2k_request_manager.h"

#include <string.h>

#include "n2k_can_id.h"

#define N2K_PGN_ISO_REQUEST 59904u

void n2k_request_manager_init(N2kRequestManager_t *manager, uint8_t local_source, N2kRequestTxFn tx_fn, void *tx_ctx) {
    if (manager == NULL) {
        return;
    }
    memset(manager, 0, sizeof(*manager));
    manager->local_source = local_source;
    manager->tx_fn = tx_fn;
    manager->tx_ctx = tx_ctx;
}

bool n2k_request_manager_build_pgn_59904(uint8_t local_source, uint8_t destination, uint32_t requested_pgn, N2K_RawFrame_t *out_frame) {
    uint32_t can_id;

    if (out_frame == NULL) {
        return false;
    }
    if (!n2k_can_id_build_29bit(6u, N2K_PGN_ISO_REQUEST, local_source, destination, &can_id)) {
        return false;
    }

    n2k_raw_frame_clear(out_frame);
    out_frame->can_id = can_id;
    out_frame->dlc = 3u;
    out_frame->flags = N2K_RAW_FLAG_EXTENDED_ID;
    out_frame->data[0] = (uint8_t)(requested_pgn & 0xFFu);
    out_frame->data[1] = (uint8_t)((requested_pgn >> 8u) & 0xFFu);
    out_frame->data[2] = (uint8_t)((requested_pgn >> 16u) & 0xFFu);
    out_frame->data[3] = 0xFFu;
    out_frame->data[4] = 0xFFu;
    out_frame->data[5] = 0xFFu;
    out_frame->data[6] = 0xFFu;
    out_frame->data[7] = 0xFFu;
    return true;
}

bool n2k_request_manager_send_request(N2kRequestManager_t *manager, uint8_t destination, uint32_t requested_pgn, uint32_t now_ms) {
    N2K_RawFrame_t frame;

    if ((manager == NULL) || (manager->tx_fn == NULL)) {
        return false;
    }
    if (!n2k_request_manager_build_pgn_59904(manager->local_source, destination, requested_pgn, &frame)) {
        return false;
    }
    frame.timestamp_ms = now_ms;
    return manager->tx_fn(&frame, manager->tx_ctx);
}
