#ifndef N2K_CAN_ID_H
#define N2K_CAN_ID_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool valid;
    uint8_t priority;
    uint32_t pgn;
    uint8_t source;
    bool destination_valid;
    uint8_t destination;
} N2K_CanIdInfo_t;

bool n2k_can_id_parse_29bit(uint32_t can_id, N2K_CanIdInfo_t *out_info);
bool n2k_can_id_build_29bit(uint8_t priority, uint32_t pgn, uint8_t source, uint8_t destination, uint32_t *out_can_id);

#endif
