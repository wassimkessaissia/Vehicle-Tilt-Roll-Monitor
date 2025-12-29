/**
  ******************************************************************************
  * @file           : mpu6500.h
  * @brief          : MPU6500/MPU6050 I2C Driver Header
  * @author         : Wassim Kessaissia
  * @date           : December 2025
  ******************************************************************************
  * @description
  * This library provides functions to interface with MPU6500/MPU6050 IMU
  * sensors via I2C communication. Supports initialization, calibration,
  * and raw data reading.
  *
  * Compatible with:
  * - MPU6500 (WHO_AM_I = 0x70)
  * - MPU6050 (WHO_AM_I = 0x68)
  ******************************************************************************
  */

#ifndef MPU6500_H // in case of many files includes mpu6500.h it will only be defined once
#define MPU6500_H

#include "main.h"
#include <stdint.h>

/* ==================== DEFINES ==================== */

// I2C Device Address (7-bit address shifted left)
#define MPU6500_ADDR        (0x68 << 1)

// Register Addresses
#define REG_WHO_AM_I        0x75    // Device ID register
#define REG_PWR_MGMT_1      0x6B    // Power management register
#define REG_ACCEL_XOUT_H    0x3B    // Accelerometer data start register

// WHO_AM_I Values
#define MPU6050_WHO_AM_I    0x68    // MPU6050 device ID
#define MPU6500_WHO_AM_I    0x70    // MPU6500 device ID

// Sensitivity Constants (for ±2g and ±250°/s)
#define ACCEL_SENSITIVITY   16384.0f    //   16384 raw units = 1g (gravity)( to convert raw to real)
#define GYRO_SENSITIVITY    131.0f      //  131 raw units = 1°/s (rotation speed)( to convert raw to real)
#define GRAVITY_ACCEL       16384       // 1g in raw units (for callibration)

/* ==================== STRUCTURES ==================== */

/**
 * @brief Structure to hold raw MPU6500 sensor data
 */
typedef struct {
    int16_t accel_x;    // Raw accelerometer X-axis
    int16_t accel_y;    // Raw accelerometer Y-axis
    int16_t accel_z;    // Raw accelerometer Z-axis
    int16_t gyro_x;     // Raw gyroscope X-axis
    int16_t gyro_y;     // Raw gyroscope Y-axis
    int16_t gyro_z;     // Raw gyroscope Z-axis
} MPU6500_Data;

/* ==================== GLOBAL VARIABLES ==================== */

// Calibration offsets (defined in mpu6500.c)
extern int16_t accel_x_offset;
extern int16_t accel_y_offset;
extern int16_t accel_z_offset;
extern int16_t gyro_x_offset;
extern int16_t gyro_y_offset;
extern int16_t gyro_z_offset;

/* ==================== FUNCTION PROTOTYPES ==================== */

/**
 * @brief Scan I2C bus for MPU6500/MPU6050
 * @return I2C address if found (0x68), 0 if not found
 */
uint8_t MPU6500_Scan(void);

/**
 * @brief Initialize MPU6500 (wake from sleep mode)
 * @return 0 on success, 1 on failure
 */
uint8_t MPU6500_Init(void);

/**
 * @brief Read WHO_AM_I register to identify device
 * @return Device ID (0x68 for MPU6050, 0x70 for MPU6500)
 */
uint8_t MPU6500_WhoAmI(void);

/**
 * @brief Read all sensor data (accel + gyro) in one transaction
 * @param data Pointer to MPU6500_Data structure to store results
 */
void MPU6500_ReadAll(MPU6500_Data *data);

/**
 * @brief Calibrate sensor offsets (must be stationary during calibration)
 * @note Takes 1000 samples over ~20 seconds
 * @note Automatically calculates and stores offsets in global variables
 */
void MPU6500_Calibrate(void);

/**
 * @brief Get calibrated accelerometer data
 * @param data Pointer to raw sensor data
 * @param x Pointer to store calibrated X-axis value
 * @param y Pointer to store calibrated Y-axis value
 * @param z Pointer to store calibrated Z-axis value
 */
void MPU6500_GetCalibratedAccel(MPU6500_Data *data, int16_t *x, int16_t *y, int16_t *z);

/**
 * @brief Get calibrated gyroscope data
 * @param data Pointer to raw sensor data
 * @param x Pointer to store calibrated X-axis value
 * @param y Pointer to store calibrated Y-axis value
 * @param z Pointer to store calibrated Z-axis value
 */
void MPU6500_GetCalibratedGyro(MPU6500_Data *data, int16_t *x, int16_t *y, int16_t *z);
float MPU6500_GetRoll(MPU6500_Data *data) ;

#endif /* MPU6500_H */
