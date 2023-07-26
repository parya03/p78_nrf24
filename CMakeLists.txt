include(../pico_sdk_import.cmake)
add_library(p78_nrf24 STATIC nrf24l01.c)
target_sources(p78_nrf24 
               "nrf24l01.h"
               "nrf24l01.c"
               )
target_link_libraries(p78_nrf24 LINK_PUBLIC
                      pico_stdlib
                      hardware_gpio
                      hardware_spi
                      )