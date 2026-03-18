#include "n2k_app_model.h"

#include <string.h>

void n2k_app_model_init(N2kAppModel_t *model) {
    if (model == NULL) {
        return;
    }
    memset(model, 0, sizeof(*model));
    n2k_device_manager_init(&model->devices);
}

void n2k_app_model_store_raw(N2kAppModel_t *model, const N2K_RawFrame_t *frame) {
    if ((model == NULL) || (frame == NULL)) {
        return;
    }
    model->raw_ring[model->raw_write_index] = *frame;
    model->raw_write_index = (model->raw_write_index + 1u) % N2K_APP_RAW_RING_SIZE;
}
