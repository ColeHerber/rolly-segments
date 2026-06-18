cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(pico_uart_bridge C CXX ASM)
set(CMAKE_C_STANDARD 11)

pico_sdk_init()

add_executable(pico_uart_bridge main.c)

target_link_libraries(pico_uart_bridge
    pico_stdlib
    hardware_uart
)

pico_enable_stdio_usb(pico_uart_bridge 1)
pico_enable_stdio_uart(pico_uart_bridge 0)

pico_add_extra_outputs(pico_uart_bridge)
