/**
  ******************************************************************************
  * @file           : mpu6500.c
  * @brief          : MPU6500/MPU6050 I2C Driver Implementation
  * @author         : Wassim Kessaissia
  * @date           : December 2025
  ******************************************************************************
  */

#include "mpu6500.h"
#include <stdio.h>

/* External I2C handle  */
extern I2C_HandleTypeDef hi2c1;
#define PI 3.14159265358979323846
/* ==================== GLOBAL VARIABLES ==================== */

// Calibration offsets
int16_t accel_x_offset = 0;
int16_t accel_y_offset = 0;
int16_t accel_z_offset = 0;
int16_t gyro_x_offset = 0;
int16_t gyro_y_offset = 0;
int16_t gyro_z_offset = 0;

/* ==================== FUNCTION IMPLEMENTATIONS ==================== */

/**
 * @brief Scan I2C bus for MPU6500/MPU6050
 * @return I2C address if found, 0 if not found
 */
uint8_t MPU6500_Scan(void) {
    uint8_t address = 0;

    printf("Scanning I2C bus...\n");

    for (uint8_t i = 0; i < 128; i++) {
        HAL_StatusTypeDef result = HAL_I2C_IsDeviceReady(&hi2c1, i << 1, 2, 100);

        if (result == HAL_OK) {
            address = i;
            printf("Device found at address: 0x%02X\n", address);
            break;
        }
    }

    if (address == 0) {
        printf("No I2C device found!\n");
    }

    return address;
}

/**
 * @brief Initialize MPU6500 (wake from sleep mode)
 * @return 0 on success, 1 on failure
 */
uint8_t MPU6500_Init(void) {
    uint8_t data = 0x00;  // Clear SLEEP bit to wake up sensor
    HAL_StatusTypeDef status;

    // Write to PWR_MGMT_1 register to wake up sensor
    status = HAL_I2C_Mem_Write(&hi2c1, MPU6500_ADDR,REG_PWR_MGMT_1 , 1 , &data ,  1 , 100);

    if (status == HAL_OK) {
        printf("MPU6500 initialized successfully!\n");
        return 0;
    } else {
        printf("MPU6500 initialization failed!\n");
        return 1;
    }
}

/**
 * @brief Read WHO_AM_I register to identify device
 * @return Device ID (0x68 for MPU6050, 0x70 for MPU6500)
 */
uint8_t MPU6500_WhoAmI(void) {
    uint8_t device_id = 0;
    HAL_StatusTypeDef status;

    // Read WHO_AM_I register
    status = HAL_I2C_Mem_Read(&hi2c1,
                              MPU6500_ADDR,
                              REG_WHO_AM_I,
                              1,
                              &device_id,
                              1,
                              100);

    if (status == HAL_OK) {
        printf("WHO_AM_I: 0x%02X ", device_id);

        if (device_id == MPU6050_WHO_AM_I) {
            printf("(MPU6050 detected)\n");
        } else if (device_id == MPU6500_WHO_AM_I) {
            printf("(MPU6500 detected)\n");
        } else {
            printf("(Unknown device)\n");
        }
    } else {
        printf("Failed to read WHO_AM_I register!\n");
    }

    return device_id;
}

/**
 * @brief Read all sensor data (accel + gyro) in one I2C transaction
 * @param data Pointer to MPU6500_Data structure to store results
 */
void MPU6500_ReadAll(MPU6500_Data *data) {
    uint8_t buffer[14];

    // Read 14 bytes starting from ACCEL_XOUT_H
    // Registers: 0x3B to 0x48
    // [0-1]:   Accel X
    // [2-3]:   Accel Y
    // [4-5]:   Accel Z
    // [6-7]:   Temperature (skipped)
    // [8-9]:   Gyro X
    // [10-11]: Gyro Y
    // [12-13]: Gyro Z

    HAL_I2C_Mem_Read(&hi2c1, MPU6500_ADDR, REG_ACCEL_XOUT_H, 1, buffer, 14, 100);

    // Combine high and low bytes (big-endian format)
    data->accel_x = (int16_t)((buffer[0] << 8) | buffer[1]);
    data->accel_y = (int16_t)((buffer[2] << 8) | buffer[3]);
    data->accel_z = (int16_t)((buffer[4] << 8) | buffer[5]);
    // Skip temperature: buffer[6] and buffer[7]
    data->gyro_x = (int16_t)((buffer[8] << 8) | buffer[9]);
    data->gyro_y = (int16_t)((buffer[10] << 8) | buffer[11]);
    data->gyro_z = (int16_t)((buffer[12] << 8) | buffer[13]);
}

/**
 * @brief Calibrate sensor offsets
 * @note Sensor must be completely stationary during calibration
 * @note Takes 1000 samples over ~20 seconds
 */
void MPU6500_Calibrate(void) {
    MPU6500_Data data;
    int32_t accel_x_sum = 0;
    int32_t accel_y_sum = 0;
    int32_t accel_z_sum = 0;
    int32_t gyro_x_sum = 0;
    int32_t gyro_y_sum = 0;
    int32_t gyro_z_sum = 0;

    printf("\n=== CALIBRATION ===\n");
    printf("Keep sensor COMPLETELY STILL!\n");
    printf("Collecting 1000 samples...\n");

    // Collect 1000 samples
    for (uint16_t i = 0; i < 1000; i++) {
        MPU6500_ReadAll(&data);

        accel_x_sum += data.accel_x;
        accel_y_sum += data.accel_y;
        accel_z_sum += data.accel_z;
        gyro_x_sum += data.gyro_x;
        gyro_y_sum += data.gyro_y;
        gyro_z_sum += data.gyro_z;

        HAL_Delay(2);  // Small delay between readings

        // Progress indicator every 100 samples
        if ((i + 1) % 100 == 0) {
            printf("Progress: %d/1000\n", i + 1);
        }
    }

    // Calculate averages
    accel_x_offset = accel_x_sum / 1000;
    accel_y_offset = accel_y_sum / 1000;
    gyro_x_offset = gyro_x_sum / 1000;
    gyro_y_offset = gyro_y_sum / 1000;
    gyro_z_offset = gyro_z_sum / 1000;

    // Special case for Z-axis: subtract gravity (16384 = 1g)
    accel_z_offset = (accel_z_sum / 1000) - GRAVITY_ACCEL;

    printf("\nCalibration complete!\n");
    printf("Offsets:\n");
    printf("  Accel: X=%d, Y=%d, Z=%d\n",
           accel_x_offset, accel_y_offset, accel_z_offset);
    printf("  Gyro:  X=%d, Y=%d, Z=%d\n\n",
           gyro_x_offset, gyro_y_offset, gyro_z_offset);
}

/**
 * @brief Get calibrated accelerometer data
 */
void MPU6500_GetCalibratedAccel(MPU6500_Data *data, int16_t *x, int16_t *y, int16_t *z) {
    *x = data->accel_x - accel_x_offset;
    *y = data->accel_y - accel_y_offset;
    *z = data->accel_z - accel_z_offset;
}

/**
 * @brief Get calibrated gyroscope data
 */
void MPU6500_GetCalibratedGyro(MPU6500_Data *data, int16_t *x, int16_t *y, int16_t *z) {
    *x = data->gyro_x - gyro_x_offset;
    *y = data->gyro_y - gyro_y_offset;
    *z = data->gyro_z - gyro_z_offset;
}
float MPU6500_GetRoll(MPU6500_Data *data) {
    int16_t acc_y = data->accel_y - accel_y_offset;
    int16_t acc_z = data->accel_z - accel_z_offset;

    float roll = atan2(acc_y, acc_z) * 180.0 / PI;

    return roll;
}
