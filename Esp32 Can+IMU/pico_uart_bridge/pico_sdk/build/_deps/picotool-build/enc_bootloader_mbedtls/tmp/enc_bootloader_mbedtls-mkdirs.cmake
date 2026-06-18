# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/Users/cole/Desktop/Esp32 Can+IMU/pico_uart_bridge/pico_sdk/build/_deps/picotool-src/enc_bootloader")
  file(MAKE_DIRECTORY "/Users/cole/Desktop/Esp32 Can+IMU/pico_uart_bridge/pico_sdk/build/_deps/picotool-src/enc_bootloader")
endif()
file(MAKE_DIRECTORY
  "/Users/cole/Desktop/Esp32 Can+IMU/pico_uart_bridge/pico_sdk/build/_deps/picotool-build/enc_bootloader_mbedtls"
  "/Users/cole/Desktop/Esp32 Can+IMU/pico_uart_bridge/pico_sdk/build/_deps/picotool-build/enc_bootloader_mbedtls"
  "/Users/cole/Desktop/Esp32 Can+IMU/pico_uart_bridge/pico_sdk/build/_deps/picotool-build/enc_bootloader_mbedtls/tmp"
  "/Users/cole/Desktop/Esp32 Can+IMU/pico_uart_bridge/pico_sdk/build/_deps/picotool-build/enc_bootloader_mbedtls/src/enc_bootloader_mbedtls-stamp"
  "/Users/cole/Desktop/Esp32 Can+IMU/pico_uart_bridge/pico_sdk/build/_deps/picotool-build/enc_bootloader_mbedtls/src"
  "/Users/cole/Desktop/Esp32 Can+IMU/pico_uart_bridge/pico_sdk/build/_deps/picotool-build/enc_bootloader_mbedtls/src/enc_bootloader_mbedtls-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/cole/Desktop/Esp32 Can+IMU/pico_uart_bridge/pico_sdk/build/_deps/picotool-build/enc_bootloader_mbedtls/src/enc_bootloader_mbedtls-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/cole/Desktop/Esp32 Can+IMU/pico_uart_bridge/pico_sdk/build/_deps/picotool-build/enc_bootloader_mbedtls/src/enc_bootloader_mbedtls-stamp${cfgdir}") # cfgdir has leading slash
endif()
