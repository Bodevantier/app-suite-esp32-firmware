/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#ifndef H_BLEPRPH_
#define H_BLEPRPH_

#include <stdbool.h>
#include "nimble/ble.h"
#include "modlog/modlog.h"
#include "esp_peripheral.h"
#ifdef __cplusplus
extern "C" {
#endif

struct ble_hs_cfg;
struct ble_gatt_register_ctxt;
typedef void (*gatt_svr_command_handler_fn)(const char *command_line, void *ctx);

void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg);
int gatt_svr_init(void);
uint16_t gatt_svr_notify_chr_val_handle(void);
int gatt_svr_set_notify_text(const char *text);
int gatt_svr_chr_notify(uint16_t conn_handle);
int gatt_svr_notify_text(uint16_t conn_handle, const char *text);
int gatt_svr_notify_text_chunks(uint16_t conn_handle, const char *text);
void gatt_svr_set_command_handler(gatt_svr_command_handler_fn handler, void *ctx);

#ifdef __cplusplus
}
#endif

#endif
