/*
 * Copyright (c) 2020 M5Stack <https://www.m5stack.com>
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file mpu6886.h
 * @defgroup mpu6886 mpu6886
 * @brief Functions for the MPU6886 inertial measurement unit (IMU).
 * @{
 *
 * ESP-IDF driver for the MPU6886 on I2C using @ropg — Rop Gonggrijp's I2C Manager
 * https://github.com/ropg/i2c_manager
 * 
 * Copyright (c) 2019 M5Stack <https://www.m5stack.com>
 * Copyright (c) 2021 Rashed Talukder <https://github.com/rashedtalukder>
 * 
 * MIT Licensed as described in the file LICENSE 
 * }
 */

#ifndef __MPU6886_H__
#define __MPU6886_H__

#include <stdint.h>
#include <esp_err.h>
#include "driver/i2c.h"

#include "i2c_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MPU6886_ADDRESS                 0x68 
#define MPU6886_WHOAMI                  0x75
#define MPU6886_ACCEL_INTEL_CTRL        0x69
#define MPU6886_SMPLRT_DIV              0x19
#define MPU6886_INT_PIN_CFG             0x37
#define MPU6886_INT_ENABLE              0x38
#define MPU6886_ACCEL_XOUT_H            0x3B
#define MPU6886_ACCEL_XOUT_L            0x3C
#define MPU6886_ACCEL_YOUT_H            0x3D
#define MPU6886_ACCEL_YOUT_L            0x3E
#define MPU6886_ACCEL_ZOUT_H            0x3F
#define MPU6886_ACCEL_ZOUT_L            0x40

#define MPU6886_TEMP_OUT_H              0x41
#define MPU6886_TEMP_OUT_L              0x42

#define MPU6886_GYRO_XOUT_H             0x43
#define MPU6886_GYRO_XOUT_L             0x44
#define MPU6886_GYRO_YOUT_H             0x45
#define MPU6886_GYRO_YOUT_L             0x46
#define MPU6886_GYRO_ZOUT_H             0x47
#define MPU6886_GYRO_ZOUT_L             0x48

#define MPU6886_USER_CTRL               0x6A
#define MPU6886_PWR_MGMT_1              0x6B
#define MPU6886_PWR_MGMT_2              0x6C
#define MPU6886_CONFIG                  0x1A
#define MPU6886_GYRO_CONFIG             0x1B
#define MPU6886_ACCEL_CONFIG            0x1C
#define MPU6886_ACCEL_CONFIG2           0x1D
#define MPU6886_FIFO_EN                 0x23

#define MPU6886_ADC_ACCEL_NUM_BYTES     6
#define MPU6886_ADC_GYRO_NUM_BYTES      6
#define MPU6886_ADC_TEMP_NUM_BYTES      2
#define MPU6886_FSR_ACCEL_NUM_BYTES     1
#define MPU6886_FSR_GYRO_NUM_BYTES      1

/**
 * @brief List of possible accelerometer scalars in Gs.
 */
/* @[declare_mpu6886_acc_scale_t] */
typedef enum {
    MPU6886_AFS_2G = 0,
    MPU6886_AFS_4G,
    MPU6886_AFS_8G,
    MPU6886_AFS_16G
} acc_scale_t;
/* @[declare_mpu6886_acc_scale_t] */

/**
 * @brief List of possible gyroscope scalars in degrees per second.
 */
/* @[declare_mpu6886_gyro_scale_t] */
typedef enum {
    MPU6886_GFS_250DPS = 0,
    MPU6886_GFS_500DPS,
    MPU6886_GFS_1000DPS,
    MPU6886_GFS_2000DPS
} gyro_scale_t;
/* @[declare_mpu6886_gyro_scale_t] */

/**
 * @brief Initializes the MPU6886 over I2C.
 * 
 * @note The Core2ForAWS_Init() calls this function
 * when the hardware feature is enabled.
 *
 * @return 0 if successful, -1 otherwise.
 */
/* @[declare_mpu6886_init] */
esp_err_t mpu6886_init( i2c_port_t *mpu6886_i2c_port );
/* @[declare_mpu6886_init] */

/**
 * @brief Retrieves the accelerometer measurements from the 
 * 16-bit ADC on the MPU6886.
 * 
 * @param[out] ax The 16-bit accelerometer measurement in the X direction.
 * @param[out] ay The 16-bit accelerometer measurement in the Y direction.
 * @param[out] az The 16-bit accelerometer measurement in the Z direction.
 */
/* @[declare_mpu6886_getacceladc] */
esp_err_t mpu6886_adc_accel_get( int16_t *ax, int16_t *ay, int16_t *az );
/* @[declare_mpu6886_getacceladc] */

/**
 * @brief Retrieves the gyroscope measurements from the 16-bit ADC on 
 * the MPU6886.
 * 
 * @param[out] gx The 16-bit accelerometer measurement in the X direction.
 * @param[out] gy The 16-bit accelerometer measurement in the Y direction.
 * @param[out] gz The 16-bit accelerometer measurement in the Z direction.
 */
/* @[declare_mpu6886_getgyroadc] */
esp_err_t mpu6886_adc_gyro_get( int16_t *gx, int16_t *gy, int16_t *gz );
/* @[declare_mpu6886_getgyroadc] */

/**
 * @brief Retrieves the internal temperature measurement from the 16-bit 
 * ADC on the MPU6886.
 * 
 * @param[out] t Temperature of the MPU6886 passed through the 16-bit ADC.
 */
/* @[declare_mpu6886_gettempadc] */
esp_err_t mpu6886_adc_temp_get( int16_t *t );
/* @[declare_mpu6886_gettempadc] */

/**
 * @brief Retrieves the resolution of the gyroscope measurements on the 
 * MPU6886.
 * 
 * @param[in] scale The degrees per second scale of measurement.
 * 
 * @return The gyroscope resolution.
 */
/* @[declare_mpu6886_getgyrores] */
esp_err_t mpu6886_gyro_res_get( gyro_scale_t scale, float *resolution );
/* @[declare_mpu6886_getgyrores] */

/**
 * @brief Retrieves the resolution of the accelerometer measurements on 
 * the MPU6886.
 * 
 * @param[in] scale The G's per second scale of measurement.
 * 
 * @return The accelerometer resolution.
 */
/* @[declare_mpu6886_getaccres] */
esp_err_t mpu6886_accel_res_get( acc_scale_t scale, float *resolution );
/* @[declare_mpu6886_getaccres] */

/**
 * @brief Sets the full-scale range of the gyroscope on the MPU6886.
 * 
 * @param[in] scale The degrees per second scale of measurement.
 */
/* @[declare_mpu6886_setgyrofsr] */
esp_err_t mpu6886_fsr_gyro_set( gyro_scale_t scale );
/* @[declare_mpu6886_setgyrofsr] */

/**
 * @brief Sets the full-scale range of the accelerometer on the 
 * MPU6886.
 * 
 * @param[in] scale The G's per second scale of measurement.
 */
/* @[declare_mpu6886_setaccelfsr] */
esp_err_t mpu6886_fsr_accel_set( acc_scale_t scale );
/* @[declare_mpu6886_setaccelfsr] */

/**
 * @brief Retrieves the accelerometer measurements from the MPU6886.
 * 
 * **Example:**
 * 
 * Get the accelerometer values for the X, Y, and Z direction.
 * @code{c}
 *  float accel_x = 0.00;
 *  float accel_y = 0.00;
 *  float accel_z = 0.00;
 *  MPU6886_GetAccelData(&accel_x, &accel_y, &accel_z);
 * @endcode
 * 
 * @param[out] ax The 16-bit accelerometer measurement in the X direction.
 * @param[out] ay The 16-bit accelerometer measurement in the Y direction.
 * @param[out] az The 16-bit accelerometer measurement in the Z direction.
 */
/* @[declare_mpu6886_getacceldata] */
esp_err_t mpu6886_accel_data_get( float *ax, float *ay, float *az );
/* @[declare_mpu6886_getacceldata] */

/**
 * @brief Retrieves the gyroscope measurements from the MPU6886.
 * 
 * **Example:**
 * 
 * Get the gyroscope values for the Roll, Yaw, and Pitch direction.
 * @code{c}
 *  float g_roll = 0.00;
 *  float g_yaw = 0.00;
 *  float g_pitch = 0.00;
 *  MPU6886_GetGyroData(&g_roll, &g_yaw, &g_pitch);
 * @endcode
 * 
 * @param[out] gr The 16-bit gyroscope roll measurement.
 * @param[out] gy The 16-bit gyroscope yaw measurement.
 * @param[out] gp The 16-bit gyroscope pitch measurement.
 */
/* @[declare_mpu6886_getgyrodata] */
esp_err_t mpu6886_gyro_data_get( float *gr, float *gy, float *gp );
/* @[declare_mpu6886_getgyrodata] */

/**
 * @brief Retrieves the internal temperature measurement from the MPU6886.
 * 
 * **Example:**
 * 
 * Get the temperature reading of the MPU6886 temperature sensor.
 * @code{c}
 *  float temperature;
 *  MPU6886_GetTempData(&temperature);
 * @endcode
 * 
 * @param[out] t Temperature of the MPU6886.
 */
/* @[declare_mpu6886_gettempdata] */
esp_err_t mpu6886_temp_data_get( float *t );
/* @[declare_mpu6886_gettempdata] */

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __MPU6886_H__ */