# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# Подключаем внешние компоненты и нашу "lib" как отдельный компонент
# components/micro_ros_espidf_component — micro-ROS transport/infra
# components/u8g2                   — уже есть в проекте, держим явным образом
# lib/                              — наш компонент с ring_buffer, consumer_stub, microros_bridge, plotter_config.h
set(EXTRA_COMPONENT_DIRS
    ${CMAKE_CURRENT_LIST_DIR}/components/micro_ros_espidf_component
    ${CMAKE_CURRENT_LIST_DIR}/components/u8g2
    ${CMAKE_CURRENT_LIST_DIR}/lib
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Оставляем имя проекта как есть, чтобы не затрагивать sdkconfig и артефакты сборки
project(oled_test)
