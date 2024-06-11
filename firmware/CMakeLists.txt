cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(rp2040-board C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

include_directories(
	lib/pico-ssd1306
)

# dmx controller app
add_executable(dmx-controller
	src/dmx-controller/dmx-controller.c
	lib/pico-ssd1306/ssd1306.c
)

pico_generate_pio_header(dmx-controller ${CMAKE_CURRENT_LIST_DIR}/src/dmx-controller/quadrature_encoder.pio)
pico_generate_pio_header(dmx-controller ${CMAKE_CURRENT_LIST_DIR}/src/dmx-controller/dmx512.pio)

pico_enable_stdio_usb(dmx-controller 1)
pico_enable_stdio_uart(dmx-controller 0)

pico_add_extra_outputs(dmx-controller)

target_link_libraries(dmx-controller
	pico_stdlib
	hardware_i2c
	hardware_pwm
	hardware_pio
	hardware_uart
	hardware_dma
)

# midi joystick
add_executable(midi-joystick
	src/midi-joystick/midi-joystick.c
	src/midi-joystick/usb_descriptors.c
)

target_include_directories(midi-joystick PUBLIC
	src/midi-joystick
)

pico_enable_stdio_usb(midi-joystick 1)
pico_enable_stdio_uart(midi-joystick 0)

pico_add_extra_outputs(midi-joystick)

target_link_libraries(midi-joystick
	pico_stdlib
	pico_unique_id
	hardware_adc
	tinyusb_device
	tinyusb_board
)
