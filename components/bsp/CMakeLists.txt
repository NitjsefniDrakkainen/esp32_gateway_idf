idf_component_register(
    SRCS
        "bsp_gateway.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        drivers common freertos driver
)
