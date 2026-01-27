idf_component_register(
    SRCS
        "mqtt/swc_mqtt.c"
        "protocol/swc_protocol.c"
        "endpoint/swc_endpoint.c"
        "webserver/swc_webserver.c"
        "config/swc_config.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        device_service bsp
)
