/**
 *
 * @file	  main.c
 *
 * @brief	  Use the BMP085 library to mesure the temperature and the pressure 
 *			    with the Bosch Digital pressure sensor.
 *
 * @author	Theodore Ateba
 *
 * @version 1.0
 *
 * @date 	  14  May 2016
 *
 * @update  10 July 2016
 *
 */

/*===========================================================================*/
/* Include Files                                                             */
/*===========================================================================*/
#include <stdlib.h>
#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "bmp085.h"

/*===========================================================================*/
/* Variables drivers configurations                                          */
/*===========================================================================*/
float   pressure      = 0;
float   temperature   = 0;

BaseSequentialStream* chp = (BaseSequentialStream*) &SD2;

/**
 * @brief I2C Configuration structure
 */
static const I2CConfig i2cConfig = {
  OPMODE_I2C,         /**< I2C Operation mode.                               */
  400000,             /**< I2C Clock speed.                                  */
  FAST_DUTY_CYCLE_2,  /**< I2C Duty cycle mode.                              */
};

/*===========================================================================*/
/* Functions and thread functions.                                           */
/*===========================================================================*/

// Alife Thread, Blink the LED
static THD_WORKING_AREA(waLedGreenThread, 128);
static THD_FUNCTION(BlinkThread, arg){
  (void)arg;
  chRegSetThreadName("Led-Green-Binker");
  while(TRUE){
    palTogglePad(GPIOA, GPIOA_LED_GREEN);
    chThdSleepMilliseconds(1000);
  }
}


/**
 * @fn    int main(void)
 * @brief Application entry point.
 */
int main(void){
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();
  
  /* Start Serial driver */
  sdStart(&SD2, NULL);
  
  /* Configure the I2C Driver and i2C Pins */
  i2cStart(&I2CD1, &i2cConfig);	
  palSetPadMode(GPIOB, 8, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN); // SCL
  palSetPadMode(GPIOB, 9, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN); // SDA
  
  /* Read the BMP085 coefficients */
  bmp085GetCalibrationData(&I2CD1, BMP085_ADDR);
  
  /* Create the thread for the LED. */
  chThdCreateStatic(waLedGreenThread, sizeof(waLedGreenThread), LOWPRIO, BlinkThread, NULL);
  
  while (true){
    bmp085ReadTemp(&I2CD1, BMP085_ADDR, &temperature);
    
    /*
     * Possible Mode values for the pressure measurement:
     *	- BMP085_ULTRA_LOW_POWER.
     *	- BMP085_STANDARD.
     *	- BMP085_HIGH_RESOLUTION.
     *	- BMP085_ULTRA_HIGH_RESOLUTION.
     */
    
    bmp085ReadPress(&I2CD1, BMP085_ADDR, BMP085_ULTRA_HIGH_RESOLUTION, &pressure);
    chprintf(chp, "\n\r BMP085 measurement:");
    chprintf(chp, "\n\r  --> Temperature = %.3f °c.",temperature);
    chprintf(chp, "\n\r  --> Pressure = %.3f hPa.\n\r", pressure);
    chThdSleepMilliseconds(1000);
    chprintf(chp, "\033[2J\033[1;1H");
  }
  
  return 0;
}
