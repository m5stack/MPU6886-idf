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

#include <freertos/FreeRTOS.h>
#include <esp_log.h>
#include "i2c_manager.h"
#include "mpu6886.h"

static i2c_port_t i2c_port;
static gyro_scale_t gyro_scale = MPU6886_GFS_2000DPS;
static acc_scale_t acc_scale = MPU6886_AFS_8G;
static float acc_res, gyro_res;

static const char *TAG = "MPU6886";

static esp_err_t mpu6886_i2c_init( i2c_port_t *port )
{
    i2c_port = *port;
    return i2c_manager_init( i2c_port );
}

static esp_err_t read_reg( uint8_t reg, uint8_t number_bytes, uint8_t *read_buffer )
{
    esp_err_t res;
    if ( ( res = i2c_manager_read( i2c_port, MPU6886_ADDRESS, reg, read_buffer, number_bytes ) ) != ESP_OK )
    {
        ESP_LOGE( TAG, "\tCould not read from register 0x%02x", reg );
        return res;
    }

    return ESP_OK;
}

static esp_err_t write_reg( uint8_t reg, uint8_t number_bytes, const uint8_t write_buffer )
{
    esp_err_t res;
    if ( ( res = i2c_manager_write( i2c_port, MPU6886_ADDRESS, reg, &write_buffer, number_bytes ) ) != ESP_OK )
    {
        ESP_LOGE( TAG, "\tCould not write 0x%04x to register 0x%02x", write_buffer, reg );
        return res;
    }

    return ESP_OK;
}

esp_err_t mpu6886_init( i2c_port_t *mpu6886_i2c_port )
{
    unsigned char tempdata[ 1 ];
    unsigned char regdata;
    esp_err_t err = mpu6886_i2c_init( mpu6886_i2c_port );

    if( err != ESP_OK)
    {
        return err;
    }

    err |= read_reg( MPU6886_WHOAMI, 1, tempdata );
    if ( tempdata[ 0 ] != 0x19 )
    {
        return -1;
    }
    vTaskDelay( 1 );

    regdata = 0x00;
    err |= write_reg( MPU6886_PWR_MGMT_1, 1, regdata );
    vTaskDelay( 10 );
    
    regdata = (0x01 << 7);
    err |= write_reg( MPU6886_PWR_MGMT_1, 1, regdata );
    vTaskDelay( 10 );
    
    regdata = (0x01 << 0);
    err |= write_reg( MPU6886_PWR_MGMT_1, 1, regdata );
    vTaskDelay( 10 );

    regdata = 0x10;
    err |= write_reg( MPU6886_ACCEL_CONFIG, 1, regdata );
    vTaskDelay( 1 );

    regdata = 0x18;
    err |= write_reg( MPU6886_GYRO_CONFIG, 1, regdata );
    vTaskDelay( 1 );

    regdata = 0x01;
    err |= write_reg( MPU6886_CONFIG, 1, regdata );
    vTaskDelay( 1 );

    regdata = 0x05;
    err |= write_reg( MPU6886_SMPLRT_DIV, 1, regdata );
    vTaskDelay( 1 );

    regdata = 0x00;    
    err |= write_reg( MPU6886_INT_ENABLE, 1, regdata );
    vTaskDelay( 1 );

    regdata = 0x00;
    err |= write_reg( MPU6886_ACCEL_CONFIG2, 1, regdata );
    vTaskDelay( 1 );

    regdata = 0x00;
    err |= write_reg( MPU6886_USER_CTRL, 1, regdata );
    vTaskDelay( 1 );

    regdata = 0x00;
    err |= write_reg( MPU6886_FIFO_EN, 1, regdata );
    vTaskDelay( 1 );

    regdata = 0x22;
    err |= write_reg( MPU6886_INT_PIN_CFG, 1, regdata );
    vTaskDelay( 1 );

    regdata = 0x01;
    err |= write_reg( MPU6886_INT_ENABLE, 1, regdata );
    vTaskDelay(100);
    
    mpu6886_gyro_res_get(gyro_scale, &gyro_res);
    mpu6886_accel_res_get(acc_scale, &acc_res);
    
    return err;
}

esp_err_t mpu6886_adc_accel_get( int16_t *ax, int16_t *ay, int16_t *az )
{   
    uint8_t buf[ MPU6886_ADC_ACCEL_NUM_BYTES ];
    
    read_reg( MPU6886_ACCEL_XOUT_H, MPU6886_ADC_ACCEL_NUM_BYTES, buf );
    

    *ax = ( ( int16_t )buf[ 0 ] << 8 ) | buf[ 1 ];
    *ay = ( ( int16_t )buf[ 2 ] << 8 ) | buf[ 3 ];
    *az = ( ( int16_t )buf[ 4 ] << 8 ) | buf[ 5 ];
    
    return ESP_OK;
}

esp_err_t mpu6886_adc_gyro_get( int16_t *gr, int16_t *gy, int16_t *gp )
{
    uint8_t buf[ MPU6886_ADC_GYRO_NUM_BYTES ];
    
    read_reg( MPU6886_GYRO_XOUT_H, MPU6886_ADC_GYRO_NUM_BYTES, buf );
    

    *gr = ( ( uint16_t )buf[ 0 ] << 8 ) | buf[ 1 ];
    *gy = ( ( uint16_t )buf[ 2 ] << 8 ) | buf[ 3 ];
    *gp = ( ( uint16_t )buf[ 4 ] << 8 ) | buf[ 5 ];

    return ESP_OK;
}

esp_err_t mpu6886_adc_temp_get( int16_t *t )
{
    uint8_t buf[ MPU6886_ADC_TEMP_NUM_BYTES ];
    
    read_reg( MPU6886_TEMP_OUT_H, MPU6886_ADC_TEMP_NUM_BYTES, buf );
    
    *t = ( ( uint16_t )buf[ 0 ] << 8 ) | buf[ 1 ];

    return ESP_OK;
}

esp_err_t mpu6886_gyro_res_get( gyro_scale_t scale, float *resolution )
{
    float gyro_res = 0.0;
    
    switch( scale )
    {
        case MPU6886_GFS_250DPS:
            gyro_res = 250.0 / 32768.0;
            break;
        case MPU6886_GFS_500DPS:
            gyro_res = 500.0 / 32768.0;
            break;
        case MPU6886_GFS_1000DPS:
            gyro_res = 1000.0 / 32768.0;
            break;
        case MPU6886_GFS_2000DPS:
        default:
            gyro_res = 2000.0 / 32768.0;
            break;
    }
    
    *resolution = gyro_res;

    return ESP_OK;
}

esp_err_t mpu6886_accel_res_get( acc_scale_t scale, float *resolution )
{
    float accel_res = 0.0;
    switch( scale )
    {
        // Possible accelerometer scales (and their register bit settings) are:
        // 2 Gs (00), 4 Gs (01), 8 Gs ( 10 ), and 16 Gs  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
        case MPU6886_AFS_2G:
            accel_res = 2.0 / 32768.0;
            break;
        case MPU6886_AFS_4G:
            accel_res = 4.0 / 32768.0;
            break;
        case MPU6886_AFS_8G:
            accel_res = 8.0 / 32768.0;
            break;
        case MPU6886_AFS_16G:
        default:
            accel_res = 16.0 / 32768.0;
            break;
    }
    *resolution = accel_res;
    
    return ESP_OK;
}

esp_err_t mpu6886_fsr_gyro_set( gyro_scale_t scale )
{
    unsigned char regdata;
    regdata = ( scale << 3 );
    
    write_reg( MPU6886_GYRO_CONFIG, MPU6886_FSR_GYRO_NUM_BYTES, regdata );
    
    gyro_scale = scale;
    mpu6886_gyro_res_get( scale, &gyro_res );

    return ESP_OK;
}

esp_err_t mpu6886_fsr_accel_set( acc_scale_t scale )
{
    unsigned char regdata;
    regdata = ( scale << 3 );
    
    write_reg( MPU6886_ACCEL_CONFIG, MPU6886_FSR_ACCEL_NUM_BYTES, regdata );
    
    acc_scale = scale;
    mpu6886_accel_res_get( scale, &acc_res );
    
    return ESP_OK; 
}

esp_err_t mpu6886_accel_data_get( float *ax, float *ay, float *az )
{
    int16_t accX = 0;
    int16_t accY = 0;
    int16_t accZ = 0;
    esp_err_t err = mpu6886_adc_accel_get( &accX, &accY, &accZ );

    if( err == ESP_OK )
    {
        *ax = ( float )accX * acc_res;
        *ay = ( float )accY * acc_res;
        *az = ( float )accZ * acc_res;
    }

    return err;
}

esp_err_t mpu6886_gyro_data_get( float *gr, float *gy, float *gp )
{
    int16_t gyroR = 0;
    int16_t gyroP = 0;
    int16_t gyroY = 0;
    esp_err_t err = mpu6886_adc_gyro_get( &gyroR, &gyroY, &gyroP );
    if( err == ESP_OK )
    {
        *gr = ( float )gyroR * gyro_res;
        *gp = ( float )gyroP * gyro_res;
        *gy = ( float )gyroY * gyro_res;
    }
    
    return err;
}

esp_err_t mpu6886_temp_data_get( float *t )
{
    int16_t temp = 0;
    esp_err_t err = mpu6886_adc_temp_get( &temp );
    if( err == ESP_OK )
        *t = ( float )temp / 326.8 + 25.0;

    return err;
}
