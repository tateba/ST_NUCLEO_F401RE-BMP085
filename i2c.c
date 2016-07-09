/**
 *
 * @file    i2c.c
 *
 * @brief   I2C driver
 *
 * @author  Theodore Ateba
 *
 * @date    05 Jully 2016
 *
 * @update  05 Jully 2016
 *
 */

/*===========================================================================*/
/* Include Libraries                                                         */
/*===========================================================================*/
#include "i2c.h"

/*===========================================================================*/
/* Driver Functions                                                          */ 
/*===========================================================================*/

/**
 * @fn    msg_t i2cReadRegister(I2CDriver *i2cp, uint8_t sad,
 *            uint8_t *reg, uint8_t *rxbuf)
 * @brief Read a register of the LM75A sensor
 *
 * @param[in] i2cp    Pointer to the i2c interface.
 * @param[in] sad     Slave address without R/W bit.
 * @param[in] reg     Register address to read.
 * @param[in] rxbuf   Buffer for the data read from the i2c bus.
 * @return            The result of the reading operation.
 */
msg_t i2cReadRegister(I2CDriver *i2cp, uint8_t sad,
    uint8_t *reg, uint8_t *rxbuf){
  msg_t msg;

  i2cAcquireBus(i2cp);
  msg = i2cMasterTransmitTimeout(i2cp, sad, reg, 1, rxbuf, 1, MS2ST(4));
  i2cReleaseBus(i2cp);

  return msg;
}

/**
 * @fn    msg_t i2cReadRegisters(I2CDriver *i2cp, uint8_t sad,
 *            uint8_t *reg, uint8_t *rxbuf, uint8_t lenght)
 * @brief Read a register of the LM75A sensor
 *
 * @param[in] i2cp    Pointer to the i2c interface.
 * @param[in] sad     Slave address without R/W bit.
 * @param[in] reg     First register address to read.
 * @param[in] rxbuf   Buffer to store the data receive by i2c bus.
 * @param[in] lenght  Size of data to read
 * @retuen            The result of the reading operation.
 */
msg_t i2cReadRegisters(I2CDriver *i2cp, uint8_t sad,
    uint8_t *reg, uint8_t *rxbuf, uint8_t lenght){
  msg_t msg;

  i2cAcquireBus(i2cp);
  msg = i2cMasterTransmitTimeout(i2cp, sad, reg, 1, rxbuf, lenght, MS2ST(4));
  i2cReleaseBus(i2cp);

  return msg;
}

/**
 * @fn    msg_t i2cWriteRegisters( I2CDriver *i2cp, uint8_t sad,
 *            uint8_t *txbuf, uint8_t lenght)
 * @brief Write a register of the LM75A sensor
 *
 * @param[in] i2cp    Pointer to the i2c interface.
 * @param[in] sad     Slave address without R/W bit.
 * @param[in] txbuf   Data to write to the sensor register.
 *                    txbuf[0] is the first register to write.
 * @param[in] lenght  Size of data to write
 * @retuen            The result of the reading operation.
 */
msg_t i2cWriteRegisters(I2CDriver *i2cp, uint8_t sad,
    uint8_t *txbuf, uint8_t lenght){
  msg_t msg;

  i2cAcquireBus(i2cp);
  msg = i2cMasterTransmitTimeout(i2cp, sad, txbuf, lenght, NULL, 0, MS2ST(4));
  i2cReleaseBus(i2cp);

  return msg;
}
