idf_component_register(SRCS "ad5940.c"
                       INCLUDE_DIRS "."
                       REQUIRES ESP32
)

# Disable warnings as errors for this vendor component only
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-error=maybe-uninitialized
    -Wno-error=overflow
    -Wno-error=unused-variable
    -Wno-error=unused-function
)