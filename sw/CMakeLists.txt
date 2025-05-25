cmake_minimum_required(VERSION 3.13)
include(pico-sdk/pico_sdk_init.cmake)

project(pico_gg_hd)

pico_sdk_init()

add_executable(sw_pico_gg_hd 
	${CMAKE_CURRENT_LIST_DIR}/src/main.c
)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/src/libdvi)

pico_generate_pio_header(sw_pico_gg_hd ${CMAKE_CURRENT_LIST_DIR}/src/gg_capture.pio)
pico_generate_pio_header(sw_pico_gg_hd ${CMAKE_CURRENT_LIST_DIR}/src/lcd_send.pio)

target_link_libraries(sw_pico_gg_hd pico_stdlib pico_time 
hardware_pio hardware_sync hardware_dma hardware_clocks pico_multicore 
pico_time hardware_adc hardware_pwm hardware_interp libdvi)

pico_add_extra_outputs(sw_pico_gg_hd)

pico_enable_stdio_usb(sw_pico_gg_hd 1)
pico_enable_stdio_uart(sw_pico_gg_hd 0)
