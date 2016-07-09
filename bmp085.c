/**
 *
 * @file    bmp085.c
 *
 * @brief   BMP085 Digital pressure sensor interface module code.
 *
 * @author  Theodore Ateba
 *
 * @versio  1.0
 *
 * @date    23 June 2016
 *
 * @update  09 July 2016
 *
 *
 * TODO: 
 *        - Calculate the altitude
 */

/*===========================================================================*/
/* Include file                                                              */
/*===========================================================================*/
#include "bmp085.h"

/*===========================================================================*/
/* Local varaibles and type                                                  */
/*===========================================================================*/
static bmp085_calib_data_t bmp085_calib_data;

/*===========================================================================*/
/* Functions                                                                 */
/*===========================================================================*/

/**
 * @fn      static uint8_t getPressureConversionTime(uint8_t oss)
 * @brief   Return the pressure coversion time need by the sensor during 
 *          pressure conversion operation.
 *
 * @param[in] oss   Oversampling setting parameter.
 * @return    time  Time need by the sensor for conversion in (ms)
 * @retval    5     If the BMP085 mode is set to Ultra Low Power.
 * @retval    8     If the BMP085 mode is set standard.
 * @retval    14    If the BMP085 mode is set to High Resolution.
 * @retval    26    If the BMP085 mode is set to Utltra High Resolution.
 */
static uint8_t getPressureConversionTime(uint8_t oss){
  uint8_t time;
  
  switch(oss){
    case BMP085_ULTRA_LOW_POWER:
      time = 5; // Conversion time 4.5 ms in the datasheet.
      break;
    
    case BMP085_STANDARD:
      time = 8; // 7.5 ms in the datasheet.
      break;
    
    case BMP085_HIGH_RESOLUTION:
      time = 14; // 13.5 ms in the datasheet.
      break;
    
    case BMP085_ULTRA_HIGH_RESOLUTION:
      time = 26; // 25.5 ms in the datasheet.
      break;
    
    default:
      time = 26;
      break;
  }
  
  return time;
}

/**
 * @fn    msg_t bmp085GetCalibrationData(I2CDriver *i2cp, uint8_t bmp085Addr)
 * @brief Read BMP085 calibration data.
 *
 * @param[in] i2cp        Pointer to the I2C device driver.
 * @param[in] bmp085Addr  BMPO85 Digital pressure sensor address
 * @return    msg         The result of the calibration data reading operation.
 */
msg_t bmp085GetCalibrationData(I2CDriver *i2cp, uint8_t bmp085Addr){
  uint8_t txbuf;
  uint8_t rxbuf[22];
  msg_t   msg;
  
  txbuf = BMP085_CALIBRATION_DATA_AC1_MSB;
  msg = i2cReadRegisters(i2cp, bmp085Addr, &txbuf, rxbuf, 22);

  if(msg == MSG_OK){
    bmp085_calib_data.ac1 = ((rxbuf[0]  << 8) | rxbuf[1]);
    bmp085_calib_data.ac2 = ((rxbuf[2]  << 8) | rxbuf[3]);
    bmp085_calib_data.ac3 = ((rxbuf[4]  << 8) | rxbuf[5]);
    bmp085_calib_data.ac4 = ((rxbuf[6]  << 8) | rxbuf[7]);
    bmp085_calib_data.ac5 = ((rxbuf[8]  << 8) | rxbuf[9]);
    bmp085_calib_data.ac6 = ((rxbuf[10] << 8) | rxbuf[11]);
    bmp085_calib_data.b1  = ((rxbuf[12] << 8) | rxbuf[13]);
    bmp085_calib_data.b2  = ((rxbuf[14] << 8) | rxbuf[15]);
    bmp085_calib_data.mb  = ((rxbuf[16] << 8) | rxbuf[17]);
    bmp085_calib_data.mc  = ((rxbuf[18] << 8) | rxbuf[19]);
    bmp085_calib_data.md  = ((rxbuf[20] << 8) | rxbuf[21]);

    return msg;
  }
  else
    return msg;
}

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
msg_t bmp085ReadTemp(I2CDriver *i2cp, uint8_t bmp085Addr, float *temp){
  int32_t utemp;
  int32_t x1,x2;
  int32_t temperature = 0;
  uint8_t txbuf[2];
  uint8_t rxbuf[2];
  msg_t   msg;
  
  txbuf[0] = BMP085_CR;
  txbuf[1] = BMP085_MODE_TEMP;
  msg = i2cWriteRegisters(i2cp, bmp085Addr, txbuf, 2);

  if(msg != MSG_OK)
    return msg;
  
  chThdSleepMilliseconds(5);
  
  txbuf[0] = BMP085_DATA;
  msg = i2cReadRegisters(i2cp, bmp085Addr, txbuf, rxbuf, 2);

  if(msg == MSG_OK){
  
    // Building value
    utemp = (int32_t)((rxbuf[0] << 8) | rxbuf[1]);
  
    // Converting value
    x1 = ((utemp - bmp085_calib_data.ac6) * bmp085_calib_data.ac5) >> 15;
    x2 = (bmp085_calib_data.mc << 11) / (x1 + bmp085_calib_data.md);
    bmp085_calib_data.b5 = x1 + x2;
    temperature = (bmp085_calib_data.b5 + 8) >> 4;

    *temp = (float)(temperature * 0.1);

    return msg;
  }
  else
    return msg;
}

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
    float *press){
  int32_t   upress;
  int32_t   x1,x2,x3;
  int32_t   b3,b6;
  uint32_t  b4,b7;
  int32_t   pressure = 0;
  uint8_t   txbuf[2];
  uint8_t   rxbuf[3];
  msg_t     msg;
  
  txbuf[0] = BMP085_CR;
  
  if(oss == 0)
    txbuf[1] = BMP085_MODE_PR0 + (oss << 6);
  else if(oss == 1)
    txbuf[1] = BMP085_MODE_PR1 + (oss << 6);
  else if(oss == 2)
    txbuf[1] = BMP085_MODE_PR2 + (oss << 6);
  else
    txbuf[1] = BMP085_MODE_PR3 + (oss << 6);
  
  msg = i2cWriteRegisters(i2cp, bmp085Addr, txbuf, 2);
  
  if(msg != MSG_OK)
    return msg;
  
  // Waiting for conversion to end 
  chThdSleepMilliseconds(getPressureConversionTime(oss));
  
  txbuf[0] = BMP085_DATA;

  msg = i2cReadRegisters(i2cp, bmp085Addr, txbuf, rxbuf, 3);
  
  if(msg == MSG_OK){
    // Building value
    upress = (int32_t)((rxbuf[0] << 16)|(rxbuf[1] << 8)|rxbuf[2]);
    upress = upress >> (8-oss);
  
    // Converting value
    b6 = bmp085_calib_data.b5 - 4000;
    x1 = (bmp085_calib_data.b2 * ((b6 * b6) >> 12)) >> 11;
    x2 = (bmp085_calib_data.ac2 * b6) >> 11;
    x3 = x1 + x2;
    b3 = ((((int32_t)bmp085_calib_data.ac1 * 4 + x3) << oss) + 2) >> 2;
    x1 = ((bmp085_calib_data.ac3)*b6) >> 13;
    x2 = (bmp085_calib_data.b1 * (b6*b6 >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = bmp085_calib_data.ac4 * (uint32_t)(x3 + 32768) >> 15;
    b7 = ((uint32_t)upress - b3)*(50000 >> oss);
  
    if (b7 < 0x80000000)
      pressure = (b7*2)/b4;
    else
      pressure = (b7/b4)*2;
  
    x1 = (pressure >> 8)*(pressure >> 8);
    x1 = (x1*3038) >> 16;
    x2 = (-7357*pressure) >> 16;
    pressure = pressure + ((x1 + x2 + 3791) >> 4);
  
    *press = (float)(pressure * 0.01);
    
    return msg;
  }
  else
    return msg;
}
