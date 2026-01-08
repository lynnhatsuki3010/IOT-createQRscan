idf_component_register(
    SRCS "oneshot_read_main.c"
    INCLUDE_DIRS "."
    REQUIRES 
        driver
        protocol_examples_common
        mqtt
        esp_wifi
        nvs_flash
        esp_netif
        esp_event
        esp_adc
        wifi_provisioning 
        protocomm 
        bt
        espressif__qrcode

)
