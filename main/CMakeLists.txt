idf_component_register(
    SRCS
        "main.c"
        "../lib/esp32_to_stm32.c"
        "../lib/ring_buffer.c"
        "../lib/spi_feeder.c"
    INCLUDE_DIRS
        "."
        "../lib"
        "shared"         # т.к. хедеры лежат в main/shared (symlink)
    REQUIRES
        driver
        u8g2
)