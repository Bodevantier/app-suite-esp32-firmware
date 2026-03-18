#include "n2k_decoder.h"

#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "n2k_decoder"

#define N2K_PGN_ADDRESS_CLAIM 60928u
#define N2K_PGN_PRODUCT_INFO 126996u
#define N2K_PGN_CONFIGURATION_INFO 126998u
#define N2K_PGN_SUPPORTED_PGN_LIST 126464u
#define N2K_PGN_WIND 130306u
#define N2K_PGN_HEADING 127250u
#define N2K_PGN_POSITION_RAPID 129025u
#define N2K_PGN_BATTERY_STATUS 127508u
#define N2K_FAST_PACKET_SESSION_TIMEOUT_MS 2000u

static void decode_single_frame_pgn(N2kDecoder_t *decoder, const N2K_CanIdInfo_t *id_info, const N2K_RawFrame_t *frame);
static bool process_fast_packet(N2kDecoder_t *decoder, const N2K_CanIdInfo_t *id_info, const N2K_RawFrame_t *frame, uint32_t now_ms, N2kFastPacketMessage_t *out_message);

void n2k_decoder_init(N2kDecoder_t *decoder, N2kAppModel_t *model) {
    if (decoder == NULL) {
        return;
    }
    memset(decoder, 0, sizeof(*decoder));
    decoder->app_model = model;
}

void n2k_decoder_process_frame(N2kDecoder_t *decoder, const N2K_RawFrame_t *frame) {
    N2K_CanIdInfo_t id_info;
    char hex_payload[3u * N2K_RAW_FRAME_MAX_DATA_LEN] = {0};
    N2kFastPacketMessage_t fp_message = {0};
    bool known_pgn = false;
    uint32_t now_ms = 0u;

    if ((decoder == NULL) || (decoder->app_model == NULL) || (frame == NULL)) {
        return;
    }

    decoder->stats.frames_total++;
    now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    n2k_app_model_store_raw(decoder->app_model, frame);
    n2k_raw_frame_payload_hex(frame, hex_payload, sizeof(hex_payload));

    if (!n2k_raw_frame_is_extended(frame)) {
        decoder->stats.frames_ignored++;
        ESP_LOGW(TAG, "non-extended frame can_id=0x%08lX dlc=%u payload=%s", (unsigned long)frame->can_id, frame->dlc, hex_payload);
        return;
    }

    decoder->stats.frames_extended++;
    if (!n2k_can_id_parse_29bit(frame->can_id, &id_info)) {
        decoder->stats.frames_ignored++;
        return;
    }
    n2k_device_manager_update_last_seen(&decoder->app_model->devices, id_info.source, now_ms);

    if (id_info.pgn == N2K_PGN_PRODUCT_INFO || id_info.pgn == N2K_PGN_CONFIGURATION_INFO) {
        if (process_fast_packet(decoder, &id_info, frame, now_ms, &fp_message) && fp_message.complete) {
            decoder->stats.fast_packet_completed++;
            if (fp_message.pgn == N2K_PGN_PRODUCT_INFO) {
                n2k_device_manager_handle_product_info(&decoder->app_model->devices, fp_message.source, now_ms, fp_message.data, fp_message.len);
            } else if (fp_message.pgn == N2K_PGN_CONFIGURATION_INFO) {
                n2k_device_manager_handle_configuration_info(&decoder->app_model->devices, fp_message.source, now_ms, fp_message.data, fp_message.len);
            }
            known_pgn = true;
        }
    } else if (id_info.pgn == N2K_PGN_SUPPORTED_PGN_LIST) {
        uint8_t dlc = n2k_raw_frame_clamp_dlc(frame->dlc);

        if ((dlc < 8u) && (dlc >= 1u) && (frame->data[0] <= 1u)) {
            n2k_device_manager_handle_supported_pgn_list(&decoder->app_model->devices,
                                                         id_info.source,
                                                         now_ms,
                                                         frame->data,
                                                         dlc);
            known_pgn = true;
        } else if (process_fast_packet(decoder, &id_info, frame, now_ms, &fp_message) && fp_message.complete) {
            decoder->stats.fast_packet_completed++;
            n2k_device_manager_handle_supported_pgn_list(&decoder->app_model->devices,
                                                         fp_message.source,
                                                         now_ms,
                                                         fp_message.data,
                                                         fp_message.len);
            known_pgn = true;
        }
    } else {
        decode_single_frame_pgn(decoder, &id_info, frame);
        known_pgn = (id_info.pgn == N2K_PGN_ADDRESS_CLAIM ||
                     id_info.pgn == N2K_PGN_SUPPORTED_PGN_LIST ||
                     id_info.pgn == N2K_PGN_WIND ||
                     id_info.pgn == N2K_PGN_HEADING ||
                     id_info.pgn == N2K_PGN_POSITION_RAPID ||
                     id_info.pgn == N2K_PGN_BATTERY_STATUS);
    }

    if (!known_pgn) {
        decoder->app_model->unknown_pgn_count++;
    }
}

static void decode_single_frame_pgn(N2kDecoder_t *decoder, const N2K_CanIdInfo_t *id_info, const N2K_RawFrame_t *frame) {
    uint8_t dlc = n2k_raw_frame_clamp_dlc(frame->dlc);

    if (id_info->pgn == N2K_PGN_ADDRESS_CLAIM) {
        if (dlc >= 8u) {
            n2k_device_manager_handle_address_claim(&decoder->app_model->devices, id_info->source, (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS), frame->data, dlc);
        }
        return;
    }

    if (id_info->pgn == N2K_PGN_WIND && dlc >= 6u) {
        uint16_t speed_raw = (uint16_t)frame->data[1] | ((uint16_t)frame->data[2] << 8u);
        uint16_t angle_raw = (uint16_t)frame->data[3] | ((uint16_t)frame->data[4] << 8u);
        decoder->app_model->wind.valid = true;
        decoder->app_model->wind.source = id_info->source;
        decoder->app_model->wind.speed_mps = (float)speed_raw * 0.01f;
        decoder->app_model->wind.angle_deg = (float)angle_raw * 0.0001f * 57.2957795f;
        decoder->app_model->wind.reference = frame->data[5];
        decoder->app_model->wind.updated_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        return;
    }

    if (id_info->pgn == N2K_PGN_HEADING && dlc >= 3u) {
        uint16_t heading_raw = (uint16_t)frame->data[1] | ((uint16_t)frame->data[2] << 8u);
        decoder->app_model->heading.valid = true;
        decoder->app_model->heading.heading_deg = (float)heading_raw * 0.0001f * 57.2957795f;
        decoder->app_model->heading.updated_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        return;
    }

    if (id_info->pgn == N2K_PGN_POSITION_RAPID && dlc >= 8u) {
        int32_t lat_raw = (int32_t)((uint32_t)frame->data[0]
                                  | ((uint32_t)frame->data[1] << 8u)
                                  | ((uint32_t)frame->data[2] << 16u)
                                  | ((uint32_t)frame->data[3] << 24u));
        int32_t lon_raw = (int32_t)((uint32_t)frame->data[4]
                                  | ((uint32_t)frame->data[5] << 8u)
                                  | ((uint32_t)frame->data[6] << 16u)
                                  | ((uint32_t)frame->data[7] << 24u));
        decoder->app_model->position.valid = true;
        decoder->app_model->position.latitude_deg = (double)lat_raw / 10000000.0;
        decoder->app_model->position.longitude_deg = (double)lon_raw / 10000000.0;
        decoder->app_model->position.updated_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        return;
    }

    if (id_info->pgn == N2K_PGN_BATTERY_STATUS && dlc >= 7u) {
        uint16_t v_raw = (uint16_t)frame->data[1] | ((uint16_t)frame->data[2] << 8u);
        int16_t i_raw = (int16_t)((uint16_t)frame->data[3] | ((uint16_t)frame->data[4] << 8u));
        uint16_t t_raw = (uint16_t)frame->data[5] | ((uint16_t)frame->data[6] << 8u);
        decoder->app_model->battery.valid = true;
        decoder->app_model->battery.instance = frame->data[0];
        decoder->app_model->battery.voltage_v = (float)v_raw * 0.01f;
        decoder->app_model->battery.current_a = (float)i_raw * 0.01f;
        decoder->app_model->battery.temperature_c = ((float)t_raw * 0.01f) - 273.15f;
        decoder->app_model->battery.updated_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        return;
    }
}

static N2kFastPacketSession_t *find_fast_packet_session(N2kDecoder_t *decoder, uint8_t source, uint32_t pgn, uint8_t sequence_id, uint32_t now_ms) {
    N2kFastPacketSession_t *free_slot = NULL;

    for (size_t i = 0; i < N2K_FAST_PACKET_MAX_SESSIONS; i++) {
        N2kFastPacketSession_t *session = &decoder->fp_sessions[i];
        if (session->used &&
            session->source == source &&
            session->pgn == pgn &&
            session->sequence_id == sequence_id) {
            return session;
        }
        if (!session->used && free_slot == NULL) {
            free_slot = session;
        }
        if (session->used && (now_ms - session->last_update_ms > N2K_FAST_PACKET_SESSION_TIMEOUT_MS)) {
            session->used = false;
            decoder->stats.fast_packet_dropped++;
            if (free_slot == NULL) {
                free_slot = session;
            }
        }
    }

    return free_slot;
}

static bool process_fast_packet(N2kDecoder_t *decoder, const N2K_CanIdInfo_t *id_info, const N2K_RawFrame_t *frame, uint32_t now_ms, N2kFastPacketMessage_t *out_message) {
    uint8_t dlc = n2k_raw_frame_clamp_dlc(frame->dlc);
    uint8_t frame_counter;
    uint8_t sequence_id;
    N2kFastPacketSession_t *session;

    if (dlc < 2u) {
        return false;
    }
    sequence_id = (uint8_t)(frame->data[0] >> 5u);
    frame_counter = (uint8_t)(frame->data[0] & 0x1Fu);

    session = find_fast_packet_session(decoder, id_info->source, id_info->pgn, sequence_id, now_ms);
    if (session == NULL) {
        decoder->stats.fast_packet_dropped++;
        return false;
    }

    if (frame_counter == 0u) {
        uint16_t total_len = frame->data[1];
        uint16_t payload_bytes = (dlc > 2u) ? (uint16_t)(dlc - 2u) : 0u;
        if (total_len == 0u || total_len > N2K_FAST_PACKET_MAX_LEN) {
            session->used = false;
            decoder->stats.fast_packet_dropped++;
            return false;
        }
        memset(session, 0, sizeof(*session));
        session->used = true;
        session->source = id_info->source;
        session->pgn = id_info->pgn;
        session->sequence_id = sequence_id;
        session->total_len = total_len;
        session->next_frame_counter = 1u;
        session->last_update_ms = now_ms;

        if (payload_bytes > total_len) {
            payload_bytes = total_len;
        }
        if (payload_bytes > 0u) {
            memcpy(session->data, &frame->data[2], payload_bytes);
            session->received_len = payload_bytes;
        }
    } else {
        uint16_t payload_bytes = (dlc > 1u) ? (uint16_t)(dlc - 1u) : 0u;

        if (!session->used || frame_counter != session->next_frame_counter) {
            if (session->used) {
                session->used = false;
                decoder->stats.fast_packet_dropped++;
            }
            return false;
        }
        if (session->received_len >= session->total_len) {
            session->used = false;
            return false;
        }

        if (payload_bytes > (session->total_len - session->received_len)) {
            payload_bytes = session->total_len - session->received_len;
        }
        memcpy(&session->data[session->received_len], &frame->data[1], payload_bytes);
        session->received_len += payload_bytes;
        session->next_frame_counter = (uint8_t)(session->next_frame_counter + 1u);
        session->last_update_ms = now_ms;
    }

    if (session->used && session->received_len >= session->total_len) {
        out_message->complete = true;
        out_message->source = session->source;
        out_message->pgn = session->pgn;
        out_message->len = session->total_len;
        memcpy(out_message->data, session->data, session->total_len);
        session->used = false;
        return true;
    }

    return false;
}
