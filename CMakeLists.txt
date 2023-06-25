idf_component_register(
        SRCS "controller.c"
        INCLUDE_DIRS include
        REQUIRES "bus-i2c" "keyboard"
)
