idf_component_register(
    SRCS
        "nrf24/nrf24.c"
        "cc1101/cc1101.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        freertos
)

target_compile_definitions(${COMPONENT_LIB} PUBLIC NRF24_USE_FREERTOS=1)
