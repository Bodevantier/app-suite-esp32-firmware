#include "n2k_raw_frame.h"

#include <stdio.h>
#include <string.h>

void n2k_raw_frame_clear(N2K_RawFrame_t *frame) {
    if (frame == NULL) {
        return;
    }
    memset(frame, 0, sizeof(*frame));
}

uint8_t n2k_raw_frame_clamp_dlc(uint8_t dlc) {
    return (dlc > N2K_RAW_FRAME_MAX_DATA_LEN) ? N2K_RAW_FRAME_MAX_DATA_LEN : dlc;
}

bool n2k_raw_frame_is_extended(const N2K_RawFrame_t *frame) {
    return (frame != NULL) && ((frame->flags & N2K_RAW_FLAG_EXTENDED_ID) != 0u);
}

void n2k_raw_frame_payload_hex(const N2K_RawFrame_t *frame, char *out, size_t out_sz) {
    size_t used = 0;
    uint8_t dlc;

    if ((frame == NULL) || (out == NULL) || (out_sz == 0u)) {
        return;
    }

    out[0] = '\0';
    dlc = n2k_raw_frame_clamp_dlc(frame->dlc);
    for (uint8_t i = 0; i < dlc; i++) {
        int n = snprintf(out + used, out_sz - used, "%02X%s", frame->data[i], (i + 1u < dlc) ? " " : "");
        if (n < 0) {
            return;
        }
        used += (size_t)n;
        if (used >= out_sz) {
            out[out_sz - 1u] = '\0';
            return;
        }
    }
}
