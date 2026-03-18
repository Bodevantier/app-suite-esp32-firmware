#include "n2k_can_id.h"

#include <stddef.h>

bool n2k_can_id_parse_29bit(uint32_t can_id, N2K_CanIdInfo_t *out_info) {
    uint8_t pf;
    uint8_t ps;
    uint8_t dp;

    if (out_info == NULL) {
        return false;
    }

    can_id &= 0x1FFFFFFFu;
    out_info->valid = true;
    out_info->priority = (uint8_t)((can_id >> 26u) & 0x07u);
    dp = (uint8_t)((can_id >> 24u) & 0x01u);
    pf = (uint8_t)((can_id >> 16u) & 0xFFu);
    ps = (uint8_t)((can_id >> 8u) & 0xFFu);
    out_info->source = (uint8_t)(can_id & 0xFFu);

    if (pf < 240u) {
        out_info->pgn = ((uint32_t)dp << 16u) | ((uint32_t)pf << 8u);
        out_info->destination_valid = true;
        out_info->destination = ps;
    } else {
        out_info->pgn = ((uint32_t)dp << 16u) | ((uint32_t)pf << 8u) | (uint32_t)ps;
        out_info->destination_valid = false;
        out_info->destination = 0xFFu;
    }

    return true;
}

bool n2k_can_id_build_29bit(uint8_t priority, uint32_t pgn, uint8_t source, uint8_t destination, uint32_t *out_can_id) {
    uint32_t can_id;
    uint8_t dp;
    uint8_t pf;
    uint8_t ps;

    if (out_can_id == NULL) {
        return false;
    }
    if (priority > 7u || pgn > 0x3FFFFu) {
        return false;
    }

    dp = (uint8_t)((pgn >> 16u) & 0x01u);
    pf = (uint8_t)((pgn >> 8u) & 0xFFu);

    if (pf < 240u) {
        ps = destination;
    } else {
        ps = (uint8_t)(pgn & 0xFFu);
    }

    can_id = ((uint32_t)(priority & 0x07u) << 26u)
           | ((uint32_t)dp << 24u)
           | ((uint32_t)pf << 16u)
           | ((uint32_t)ps << 8u)
           | (uint32_t)source;

    *out_can_id = can_id & 0x1FFFFFFFu;
    return true;
}
