cmake_minimum_required(VERSION 3.12)
set(PICO_BOARD "pico_w")# Pull in SDK (must be before project)
# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

include(pico_extras_import_optional.cmake)

project(pico_examples C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

set(PICO_EXAMPLES_PATH ${PROJECT_SOURCE_DIR})

# Initialize the SDK
pico_sdk_init()

include(example_auto_set_url.cmake)

add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        -Wno-maybe-uninitialized
        )

        add_executable(pio_ws2812)

        file(MAKE_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/generated)
        
        # generate the header file into the source tree as it is included in the RP2040 datasheet
        pico_generate_pio_header(pio_ws2812 ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/generated)
        
        target_sources(pio_ws2812 PRIVATE ws2812.c)
        
        add_library(kiss_fftr kiss_fftr.c)
        add_library(kiss_fft kiss_fft.c)

        target_link_libraries(kiss_fftr pico_stdlib kiss_fft)
        
        target_link_libraries(pio_ws2812 PRIVATE pico_stdlib pico_btstack_ble pico_btstack_cyw43 pico_cyw43_arch_none pico_multicore hardware_pio hardware_adc hardware_dma hardware_pwm kiss_fftr)
        target_include_directories(pio_ws2812 PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        )
        pico_enable_stdio_usb(pio_ws2812 1)
        pico_enable_stdio_uart(pio_ws2812 1)
        pico_add_extra_outputs(pio_ws2812)
        pico_btstack_make_gatt_header(pio_ws2812 PRIVATE "${CMAKE_CURRENT_LIST_DIR}/mygatt.gatt")
        # add url via pico_set_program_url
        example_auto_set_url(pio_ws2812)
        
        add_executable(pio_ws2812_parallel)
        
        pico_generate_pio_header(pio_ws2812_parallel ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/generated)
        
        target_sources(pio_ws2812_parallel PRIVATE ws2812_parallel.c)
        
        target_compile_definitions(pio_ws2812_parallel PRIVATE
                PIN_DBG1=3)
        
        target_link_libraries(pio_ws2812_parallel PRIVATE pico_stdlib hardware_pio hardware_dma)
        pico_add_extra_outputs(pio_ws2812_parallel)
        
        # add url via pico_set_program_url
        example_auto_set_url(pio_ws2812_parallel)
        
        # Additionally generate python and hex pioasm outputs for inclusion in the RP2040 datasheet
        add_custom_target(pio_ws2812_datasheet DEPENDS ${CMAKE_CURRENT_LIST_DIR}/generated/ws2812.py)
        add_custom_command(OUTPUT ${CMAKE_CURRENT_LIST_DIR}/generated/ws2812.py
                DEPENDS ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio
                COMMAND Pioasm -o python ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio ${CMAKE_CURRENT_LIST_DIR}/generated/ws2812.py
                VERBATIM)
        add_dependencies(pio_ws2812 pio_ws2812_datasheet)

        cmake_minimum_required(VERSION 3.12)
