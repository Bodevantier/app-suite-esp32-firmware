set(srcs "main.c"
         "gatt_svr.c"
         "n2k_raw_frame.c"
         "spi_n2k_transport.c")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "."
                       REQUIRES nvs_flash bt
                       PRIV_REQUIRES esp_driver_spi)
