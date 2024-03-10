cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(rp2040-board C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

include_directories(
	lib/pico-ssd1306
)

# display-demo app
add_executable(display-demo
	src/display-demo.c
	lib/pico-ssd1306/ssd1306.c
)

pico_enable_stdio_usb(display-demo 1)
pico_enable_stdio_uart(display-demo 0)

pico_add_extra_outputs(display-demo)

target_link_libraries(display-demo pico_stdlib hardware_i2c)
