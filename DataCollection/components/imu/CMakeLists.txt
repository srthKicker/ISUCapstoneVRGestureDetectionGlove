idf_component_register(
    SRCS
        "imu_mpu6050.c"
        "imu_mpu9250.c"
    INCLUDE_DIRS "."
    REQUIRES driver
)
