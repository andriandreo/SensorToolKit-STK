idf_component_register(SRCS "esp32.c" "ESP32Port.c"
                       INCLUDE_DIRS "."
                       REQUIRES ad5940lib driver freertos esp_wifi mqtt nvs_flash
)