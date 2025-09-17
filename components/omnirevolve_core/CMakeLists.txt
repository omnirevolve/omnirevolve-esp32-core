idf_component_register(
    SRCS
        "src/esp32_core.c"
        "src/drivers/keypad.c"
        "src/drivers/oled_display.c"
        "src/link/stm_link.c"
        "src/link/uart_link.c"
    INCLUDE_DIRS "include"
    REQUIRES
        driver
        freertos
        spi_flash
        esp_wifi
        esp_netif
        u8g2
        omnirevolve_protocol
)
