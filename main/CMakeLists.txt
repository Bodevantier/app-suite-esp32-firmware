set(srcs "main.c"
         "gatt_svr.c"
         "n2k_raw_frame.c"
         "n2k_can_id.c"
         "spi_n2k_transport.c"
         "n2k_device_manager.c"
         "n2k_request_manager.c"
         "n2k_app_model.c"
         "n2k_decoder.c")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "."
                       REQUIRES nvs_flash bt json
                       PRIV_REQUIRES esp_driver_spi)
