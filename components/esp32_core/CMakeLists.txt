idf_component_register(
    SRCS
        "src/esp32_core.c"
        "src/core/state.c"
        "src/tasks/plotter_control.c"
        "src/drivers/oled_display.c"
        "src/drivers/keypad.c"
        "src/link/stm_link.c"
        "src/link/uart_proto.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        driver freertos esp_wifi esp_netif
        u8g2
        omnirevolve_protocol
)
