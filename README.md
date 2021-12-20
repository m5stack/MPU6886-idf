# MPU6886 ESP-IDF (FreeRTOS/C) Library

## Overview
This is a ESP-IDF native driver for the MPU6886 6-axis IMU over I2C library. This driver requires ROPG's [I2C Manager for ESP32](https://github.com/ropg/i2c_manager) library as an included component in order to perform thread-safe operations over the I2C bus. This library does not support using the ESP-IDF I2C drivers directly.

## Related Link
[Document](https://docs.m5stack.com/en/unit/imu)
[Datasheet](https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/docs/datasheet/core/MPU-6886-000193%2Bv1.1_GHIC_en.pdf)

## License
MIT