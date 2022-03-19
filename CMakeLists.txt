cmake_minimum_required(VERSION 3.13...3.23)

# initialize the SDK based on PICO_SDK_PATH
include(pico_sdk_import.cmake)

project(RP2040_GBS C CXX)

# Initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(RP2040_GBS src/main.c)
target_include_directories(RP2040_GBS PRIVATE inc)
target_link_libraries(RP2040_GBS
        pico_stdlib pico_stdio pico_bootrom pico_multicore
        hardware_clocks hardware_pio hardware_vreg hardware_pio
        hardware_sync hardware_pll hardware_spi hardware_irq
        pico_binary_info)
target_compile_definitions(RP2040_GBS PRIVATE
        PARAM_ASSERTIONS_DISABLE_ALL=1
        PICO_ENTER_USB_BOOT_ON_EXIT=1
        PICO_STDIO_ENABLE_CRLF_SUPPORT=0
        PICO_STDIO_DEFAULT_CRLF=0
        PICO_PRINTF_SUPPORT_FLOAT=0
        PICO_PRINTF_SUPPORT_EXPONENTIAL=0
        PICO_PRINTF_SUPPORT_LONG_LONG=0
        PICO_PRINTF_SUPPORT_PTRDIFF_T=0)

#pico_set_binary_type(manager copy_to_ram)
#pico_set_binary_type(manager no_flash)
pico_enable_stdio_usb(RP2040_GBS 1)
pico_enable_stdio_uart(RP2040_GBS 0)
pico_add_bin_output(RP2040_GBS)
pico_add_uf2_output(RP2040_GBS)