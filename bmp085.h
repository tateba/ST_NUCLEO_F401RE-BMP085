 
/**
 *
 * @file    bmp085.h
 *
 * @brief   BMP085 Digital pressure sensor interface module header file.
 *
 * @author  Theodore Ateba
 *
 * @versio  1.0
 *
 * @date    23 June 2016
 *
 * @update  09 July 2016
 *
 */
#ifndef _BMP085_H_
#define _BMP085_H_

/*===========================================================================*/
/* Include files.					                                                   */
/*===========================================================================*/
#include "i2c.h"

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   BMP085 I2C interface switch.
 * @details If set to @p TRUE the support for I2C is included.
 * @note    The default is @p TRUE.
 */
#if !defined(BMP085_USE_I2C) || defined(__DOXYGEN__)
#define BMP085_USE_I2C                    TRUE
#endif
 
/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if BMP085_USE_I2C && !HAL_USE_I2C
#error "BMP085_USE_I2C requires HAL_USE_I2C"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*
 * BMP085 calibration data coefficients structure
 */
typedef struct bmp085_eprom{
  int16_t ac1;
  int16_t ac2;
  int16_t ac3;
  uint16_t ac4;
  uint16_t ac5;
  uint16_t ac6;
  int16_t b1;
  int16_t b2;
  int16_t mb;
  int16_t mc;
  int16_t md;
  int32_t b5;
} bmp085_calib_data_t;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*
 * Control and register and I2C adresse of the BMP085
 */
#define BMP085_ADDR                       ((uint8_t)0x77)
#define BMP085_CR                         ((uint8_t)0xF4)

/*
 * Control Register value
 */
#define BMP085_MODE_TEMP                  ((uint8_t)0x2E)
#define BMP085_MODE_PR0                   ((uint8_t)0x34)
#define BMP085_MODE_PR1                   ((uint8_t)0x74)
#define BMP085_MODE_PR2                   ((uint8_t)0xB4)
#define	BMP085_MODE_PR3                   ((uint8_t)0xF4)

/*
 * Data register adresses
 */
#define BMP085_DATA                       ((uint8_t)0xF6)
#define BMP085_DATA_MSB                   ((uint8_t)0xF6)	
#define BMP085_DATA_LSB                   ((uint8_t)0xF7)
#define BMP085_DATA_XLSB                  ((uint8_t)0xF8)

/*
 * Calibration register Adresses
 */
#define BMP085_CALIBRATION_DATA_AC1_MSB   ((uint8_t)0xAA)
#define BMP085_CALIBRATION_DATA_AC1_LSB   ((uint8_t)0xAB)
#define BMP085_CALIBRATION_DATA_AC2_MSB   ((uint8_t)0xAC)
#define BMP085_CALIBRATION_DATA_AC2_LSB   ((uint8_t)0xAD)
#define BMP085_CALIBRATION_DATA_AC3_MSB   ((uint8_t)0xAE)
#define BMP085_CALIBRATION_DATA_AC3_LSB   ((uint8_t)0xAF)
#define BMP085_CALIBRATION_DATA_AC4_MSB   ((uint8_t)0xB0)
#define BMP085_CALIBRATION_DATA_AC4_LSB   ((uint8_t)0xB1)
#define BMP085_CALIBRATION_DATA_AC5_MSB   ((uint8_t)0xB2)
#define BMP085_CALIBRATION_DATA_AC5_LSB   ((uint8_t)0xB3)
#define BMP085_CALIBRATION_DATA_AC6_MSB   ((uint8_t)0xB4)
#define BMP085_CALIBRATION_DATA_AC6_LSB   ((uint8_t)0xB5)
#define	BMP085_CALIBRATION_DATA_B1_MSB    ((uint8_t)0xB6)
#define BMP085_CALIBRATION_DATA_B1_LSB    ((uint8_t)0xB7)
#define BMP085_CALIBRATION_DATA_B2_MSB    ((uint8_t)0xB8)
#define	BMP085_CALIBRATION_DATA_B2_LSB    ((uint8_t)0xB9)
#define BMP085_CALIBRATION_DATA_MB_MSB    ((uint8_t)0xBA)
#define BMP085_CALIBRATION_DATA_MB_LSB    ((uint8_t)0xBB)
#define BMP085_CALIBRATION_DATA_MC_MSB    ((uint8_t)0xBC)
#define BMP085_CALIBRATION_DATA_MC_LSB    ((uint8_t)0xBD)
#define BMP085_CALIBRATION_DATA_MD_MSB    ((uint8_t)0xBE)
#define BMP085_CALIBRATION_DATA_MD_LSB    ((uint8_t)0xBF)

/*
 * BMP085 oversampling mode
 */
#define BMP085_OVERSAMPLING_MODE_ULP      ((uint8_t)0x00)
#define BMP085_OVERSAMPLING_MODE_S        ((uint8_t)0x01)
#define BMP085_OVERSAMPLING_MODE_HR       ((uint8_t)0x02)
#define BMP085_OVERSAMPLING_MODE_UHR      ((uint8_t)0x03)

#define BMP085_ULTRA_LOW_POWER            ((uint8_t)0x00)
#define BMP085_STANDARD                   ((uint8_t)0x01)
#define BMP085_HIGH_RESOLUTION            ((uint8_t)0x02)
#define BMP085_ULTRA_HIGH_RESOLUTION      ((uint8_t)0x03)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

/*===========================================================================*/
/* Functions.                                                                */
/*===========================================================================*/

/**
 * @fn    msg_t bmp085GetCalibrationData(I2CDriver *i2cp, uint8_t bmp085Addr)
 * @brief Read BMP085 calibration data.
 *
 * @param[in] i2cp        Pointer to the I2C device driver.
 * @param[in] bmp085Addr  BMPO85 Digital pressure sensor address
 * @return    msg         The result of the calibration data reading operation.
 */
msg_t bmp085GetCalibrationData(I2CDriver *i2cp, uint8_t bmp085Addr);

/**
 * @fn      smg_t bmp085ReadTemp(I2CDriver *i2cp, uint8_t bmp085Addr,
 *                    float *temp)
 * @brief   Read temperature from the digital pressure sensor.
 *
 * @param[in] i2cp        I2C interface pointer.
 * @param[in] bmp085Addr  The BMP085 digital pressure sensor address.
 * @param[in] temp        Pointer to the temperature variable.
 * @return    msg         The result of the temperature reading operation.
 */
msg_t bmp085ReadTemp(I2CDriver *i2cp, uint8_t bmp085Addr, float *temp);

/**
 * @fn      msg_t bmp085ReadPress(I2CDriver *i2cp, uint8_t bmp085Addr,
 *                    uint8_t oss, float *press)
 * @brief   Read pressure with I2C interface of the digital pressure sensor.
 *
 * @param[in] i2cp        I2C interface pointer.
 * @param[in] bmp085Addr  BMP085 digital pressure sensor address.
 * @param[in] oss         Oversampling setting parameter.
 * @param[in] press       Pointer to the pressure variable.
 * @return    pressure    Result of the pressure reading operation..
 */
msg_t bmp085ReadPress(I2CDriver *i2cp, uint8_t bmp085Addr, uint8_t oss,
    float *press);

#endif // _BMP085_H_