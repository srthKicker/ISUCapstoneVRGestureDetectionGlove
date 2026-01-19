file(GLOB_RECURSE BHI360_SRC
    "*.c"
)

idf_component_register(
    SRCS ${BHI360_SRC}
    INCLUDE_DIRS "." "firmware/bhi360"
    PRIV_INCLUDE_DIRS "firmware" "examples"
)
