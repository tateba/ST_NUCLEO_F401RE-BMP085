
/**
 *
 * @file	  main.c
 *
 * @brief	  BMP085 device driver demo program.
 *
 * @author	Theodore Ateba, tf.ateba@gmail.com
 *
 * @date 	  14  May 2016
 *
 */

/*==========================================================================*/
/* Include Files.                                                           */
/*==========================================================================*/

/* Standard files. */
#include <stdlib.h>
#include "string.h"

/* ChibiOS files. */
#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "shell.h"

/* Driver files. */
#include "bmp085.h"

/*==========================================================================*/
/* Application variables.                                                   */
/*==========================================================================*/

float press       = 0;  /**< Pressure.              */
float temperature = 0;  /**< Temperature.           */
float altitude    = 0;  /**< Altitude.              */
float seapress    = 0;  /**< Pressure at sea level. */

BaseSequentialStream* chp = (BaseSequentialStream*) &SD2;

/**
 * @brief I2C Configuration structure
 */
static const I2CConfig i2cConfig = {
  OPMODE_I2C,         /**< I2C Operation mode.                              */
  400000,             /**< I2C Clock speed.                                 */
  FAST_DUTY_CYCLE_2,  /**< I2C Duty cycle mode.                             */
};

/*==========================================================================*/
/* Command line related.                                                    */
/*==========================================================================*/

/* Enable use of special ANSI escape sequences */
#define CHPRINTF_USE_ANSI_CODE      TRUE
#define SHELL_WA_SIZE               THD_WORKING_AREA_SIZE(2048)

static void cmd_read(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)argv;
  if (argc != 1) {
    chprintf(chp, "Usage: read [baro|thermo|alti]\r\n");
    return;
  }

  while (chnGetTimeout((BaseChannel *)chp, 150) == Q_TIMEOUT) {
    if (!strcmp (argv[0], "baro")) {
#if CHPRINTF_USE_ANSI_CODE
      chprintf(chp, "\033[2J\033[1;1H");
#endif
      bmp085ReadPress(&I2CD1, BMP085_ADDR, BMP085_ULTRA_HIGH_RESOLUTION, &press);
      chprintf(chp, "BMP085 Barometer data ....\r\n");
      chprintf(chp, "Pressure: %.2f hPa\r\n", press);
    }
    else if (!strcmp (argv[0], "thermo")) {
#if CHPRINTF_USE_ANSI_CODE
      chprintf(chp, "\033[2J\033[1;1H");
#endif
      bmp085ReadTemp(&I2CD1, BMP085_ADDR, &temperature);
      chprintf(chp, "BMP085 Thermometer data...\r\n");
      chprintf(chp, "Temperature: %.2f °C\r\n", temperature);
    }
    else if (!strcmp (argv[0], "alti")) {
#if CHPRINTF_USE_ANSI_CODE
      chprintf(chp, "\033[2J\033[1;1H");
#endif
      bmp085GetAltitude(&I2CD1, BMP085_ADDR, &altitude);
      chprintf(chp, "BMP085 Altimeter data...\r\n");
      chprintf(chp, "Altitude: %.2f m\r\n", altitude);
    }
    else if (!strcmp (argv[0], "all")) {
#if CHPRINTF_USE_ANSI_CODE
      chprintf(chp, "\033[2J\033[1;1H");
#endif
      bmp085ReadPress(&I2CD1, BMP085_ADDR, BMP085_ULTRA_HIGH_RESOLUTION, &press);
      bmp085ReadTemp(&I2CD1, BMP085_ADDR, &temperature);
      bmp085GetAltitude(&I2CD1, BMP085_ADDR, &altitude);
      chprintf(chp, "BMP085 sensor all data ....\r\n");
      chprintf(chp, "Pressure: %.2f hPa\r\n", press);
      chprintf(chp, "Temperature: %.2f °C\r\n", temperature);
      chprintf(chp, "Altitude: %.2f m\r\n", altitude);
      chThdSleepMilliseconds(10);
    }
    else {
      chprintf(chp, "Usage: read [baro|thermo|alti|all] \r\n");
      return;
    }
  }
  chprintf(chp, "Stopped\r\n");
}

static const ShellCommand commands[] = {
  {"read", cmd_read},
  {NULL, NULL}
};

static const ShellConfig shell_cfg1 = {
  (BaseSequentialStream *)&SD2,
  commands
};

/*==========================================================================*/
/* Functions and thread functions.                                          */
/*==========================================================================*/

/**
 * @brief   Alife Thread, Blink the LED.
 */
static THD_WORKING_AREA(waLedThread, 128);
static THD_FUNCTION(LedThread, arg) {

  (void)arg;
  chRegSetThreadName("led");

  while (TRUE) {
    palTogglePad(GPIOA, GPIOA_LED_GREEN);
    chThdSleepMilliseconds(1000);
  }
}

/**
 * @brief   Application entry point.
 */
int main(void) {
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

  /* Configure the I2C Driver and i2C Pins, SCL then SDA */
  i2cStart(&I2CD1, &i2cConfig);	
  palSetPadMode(GPIOB, 8, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);
  palSetPadMode(GPIOB, 9, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);

  /* Read the BMP085 coefficients */
  bmp085GetCalibrationData(&I2CD1, BMP085_ADDR);

  /* Create the thread for the LED. */
  chThdCreateStatic(waLedThread, sizeof(waLedThread), LOWPRIO, LedThread, NULL);

  /*
   * Shell manager initialization.
   */
  shellInit();

  while(TRUE) {

    thread_t *shelltp = chThdCreateFromHeap(NULL, SHELL_WA_SIZE,
                                            "shell", NORMALPRIO + 1,
                                            shellThread, (void *)&shell_cfg1);
    chThdWait(shelltp);                  /* Waiting termination.            */
    chThdSleepMilliseconds(1000);
  }

  return 0;
}

