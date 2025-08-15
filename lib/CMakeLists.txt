# Component: plotter_esp32/lib

set(srcs
    ring_buffer.c
    consumer_stub.c
    microros_bridge.c
)

idf_component_register(
    SRCS        ${srcs}
    INCLUDE_DIRS .
    REQUIRES    freertos esp_common micro_ros_espidf_component
)
