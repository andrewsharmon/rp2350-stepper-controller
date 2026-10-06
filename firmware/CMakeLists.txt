cmake_minimum_required(VERSION 3.13)

# Default to the custom stack board; override with -DPICO_BOARD=pico2 for
# bring-up on a Pico 2 (RP2350A: motors 1-5 on pins, see src/board_pins.h).
set(PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_LIST_DIR}/boards)
set(PICO_BOARD rp2354b_stepper CACHE STRING "Board type")
set(PICO_PLATFORM rp2350-arm-s CACHE STRING "Platform")

include(pico_sdk_import.cmake)

project(stepper C CXX ASM)
set(CMAKE_C_STANDARD 11)

pico_sdk_init()

add_executable(stepper
    src/main.c
    src/hbridge.c
    src/hbridge_encode.c
    src/microstep.c
    src/sine_lut.c
    src/ladder.c
    src/adc_monitor.c
    src/led.c
    src/motion.c
    src/group.c
    src/control.c
    src/console.c
)

pico_generate_pio_header(stepper ${CMAKE_CURRENT_LIST_DIR}/src/hbridge_pwm.pio)
pico_generate_pio_header(stepper ${CMAKE_CURRENT_LIST_DIR}/src/ws2812.pio)

set(NUM_MOTORS 10 CACHE STRING "Number of motors driven (1-10)")
target_compile_definitions(stepper PRIVATE NUM_MOTORS=${NUM_MOTORS})
option(LADDER_BYPASS "Ignore the e-stop/button ladder (bench only, no ladder wired)" OFF)
if(LADDER_BYPASS)
    target_compile_definitions(stepper PRIVATE LADDER_BYPASS)
endif()
option(BOOT_TRACE "Wait 3 s at boot and print start-up progress over USB" OFF)
if(BOOT_TRACE)
    target_compile_definitions(stepper PRIVATE BOOT_TRACE)
endif()

target_link_libraries(stepper
    pico_stdlib
    pico_multicore
    pico_util
    hardware_pio
    hardware_clocks
    hardware_dma
    hardware_watchdog
    hardware_adc
)

pico_enable_stdio_usb(stepper 1)
pico_enable_stdio_uart(stepper 0)
pico_add_extra_outputs(stepper)
