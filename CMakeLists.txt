# STM32F4 code from https://github.com/plusk01/stm32-libopencm3-cmake-blink

cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(p78_nrf24 C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# ------------------- PICO -------------------

pico_sdk_init()

add_library(p78_nrf24_rpipico nrf24l01.c)
target_sources(p78_nrf24_rpipico PRIVATE "nrf24l01.c")
target_link_libraries(p78_nrf24_rpipico pico_stdlib hardware_gpio hardware_spi)
target_compile_definitions(p78_nrf24_rpipico PRIVATE RPI_PICO)

# ------------------- STM32F429 -------------------

# Change if needed
set(libopencm3_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libopencm3)

# LibOpenCM3 build
add_custom_target(libopencm3 make TARGETS=stm32/f4 WORKING_DIRECTORY ${libopencm3_SOURCE_DIR})
add_library(stm32f429 STATIC IMPORTED)
set_property(TARGET stm32f429 PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${libopencm3_SOURCE_DIR}/include)
set_property(TARGET stm32f429 PROPERTY IMPORTED_LOCATION ${libopencm3_SOURCE_DIR}/lib/libopencm3_stm32f4.a)
add_dependencies(stm32f429 libopencm3)
target_link_directories(stm32f429 INTERFACE ${libopencm3_SOURCE_DIR}/lib)

target_compile_definitions(stm32f429 INTERFACE -DSTM32F4)

set(COMPILE_OPTIONS 
  --static
  -nostartfiles
  -fno-common
  -mcpu=cortex-m4
  -mthumb
  -mfloat-abi=hard
  -mfpu=fpv4-sp-d16
)
target_compile_options(stm32f429 INTERFACE ${COMPILE_OPTIONS})
target_link_options(stm32f429 INTERFACE ${COMPILE_OPTIONS})

add_library(p78_nrf24_stm32f4 nrf24l01.c)
target_include_directories(p78_nrf24_stm32f4 PRIVATE ${libopencm3_SOURCE_DIR}/include)
target_sources(p78_nrf24_stm32f4 PRIVATE "nrf24l01.h" "nrf24l01.c")
target_compile_definitions(p78_nrf24_stm32f4 PRIVATE STM32F4)
target_compile_options(p78_nrf24_stm32f4 PRIVATE ${COMPILE_OPTIONS})
target_link_directories(p78_nrf24_stm32f4 PRIVATE stm32f429)