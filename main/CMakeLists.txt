set(srcs "main.c"
         "gatt_svr.c"
         "device_list_bridge.c"
         "device_list_bridge_test.c"
         "n2k_raw_frame.c"
         "spi_n2k_transport.c"
         "n2k_app_model.c")

idf_component_register(SRCS "device_list_bridge.c" "ble_text_protocol.c" "${srcs}"
                       INCLUDE_DIRS "."
                       REQUIRES nvs_flash bt
                       PRIV_REQUIRES esp_driver_spi)
