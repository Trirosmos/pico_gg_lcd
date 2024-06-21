cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(pico_gg_hd)

pico_sdk_init()

add_executable(sw_pico_gg_hd 
	../src/main.c
)

pico_set_binary_type(sw_pico_gg_hd gg)

pico_generate_pio_header(sw_pico_gg_hd ../src/gg_capture.pio)
pico_generate_pio_header(sw_pico_gg_hd ../src/lcd_send.pio)

target_link_libraries(sw_pico_gg_hd pico_stdlib pico_time hardware_pio hardware_sync hardware_dma hardware_clocks pico_multicore pico_time hardware_adc hardware_pwm)

pico_add_extra_outputs(sw_pico_gg_hd)

pico_enable_stdio_usb(sw_pico_gg_hd 1)
pico_enable_stdio_uart(sw_pico_gg_hd 0)
