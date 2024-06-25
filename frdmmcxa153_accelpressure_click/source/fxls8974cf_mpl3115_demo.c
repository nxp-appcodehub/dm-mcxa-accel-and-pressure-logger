/*
 * Copyright 2024 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file fxls8974cf_mpl3115_demo.c
 * @brief The fxls8974cf_mpl3115_demo.c file implements the ISSDK FXLS8974 & MPL3115 sensor driver
 *        example demonstration with RGB LED status based on FXLS8974CF X, Y, Z +/-1g values.
 */

/*  SDK Includes */
#include <math.h>
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_debug_console.h"

/* CMSIS Includes */
#include "Driver_I2C.h"

/* ISSDK Includes */
#include "issdk_hal.h"
#include "fxls8974_drv.h"
#include "mpl3115_drv.h"
#include "systick_utils.h"
#include "fsl_lptmr.h"
#include "fsl_clock.h"
#include "fsl_reset.h"
#include <stdbool.h>
#include "string.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define PRESSURE_MODE          (1)
#define ALTITUDE_MODE          (2)
#define FXLS8974_DATA_SIZE     (6)            /* Accel X, Y, Z data size. */
#define MPL3115_DATA_SIZE      (5)            /* 3 byte Pressure/Altitude and 2 byte Temperature. */
#define ACCEL_2G_SENS          (0.000976)     /* Sensitivity factor for 2G FS */
#define ACCEL_4G_SENS          (0.001953)     /* Sensitivity factor for 4G FS */
#define ACCEL_8G_SENS          (0.003906)     /* Sensitivity factor for 8G FS */
#define ACCEL_16G_SENS         (0.007813)     /* Sensitivity factor for 16G FS */
//#define MPL3115_MODE           PRESSURE_MODE  /* Un-comment to choose PRESSURE MODE */
#define MPL3115_MODE           ALTITUDE_MODE  /* Un-comment to choose ALTITUDE MODE */

#define DEMO_LPTMR_BASE    LPTMR0
#define LPTMR_USEC_COUNT   1000000
#define DEMO_LPTMR_IRQn    LPTMR0_IRQn
#define LPTMR_LED_HANDLER  LPTMR0_IRQHandler
#define LPTMR_SOURCE_CLOCK (16000U)
#define LED_INIT()         LED_RED_INIT(LOGIC_LED_OFF)
#define LED_TOGGLE()       LED_RED_TOGGLE()

#define ACCEL_THRESHOLD_zHIGH 1.18f  //must account for solder down offset
#define ACCEL_THRESHOLD_zLOW 0.82f  //must account for solder down offset
#define ACCEL_THRESHOLD_xy 0.01f //setting this arbitrarily low doesn't seem to help

#define AVE_BUFFER_LENGHT 20U   /* Number of samples used for Mov. Average. */

/*******************************************************************************
 * Constants
 ******************************************************************************/
/*! @brief Register settings for Normal (non buffered) mode. */
const registerwritelist_t cFxls8974ConfigNormal[] = {
    /* Set Full-scale range as 2G. */
    {FXLS8974_SENS_CONFIG1, FXLS8974_SENS_CONFIG1_FSR_2G, FXLS8974_SENS_CONFIG1_FSR_MASK},
    /* Set Wake Mode ODR Rate as 6.25Hz. */
    {FXLS8974_SENS_CONFIG3, FXLS8974_SENS_CONFIG3_WAKE_ODR_6_25HZ, FXLS8974_SENS_CONFIG3_WAKE_ODR_MASK},
    __END_WRITE_DATA__};

const registerwritelist_t cMpl3115ConfigPressuree[] = {
    /* Enable Data Ready and Event flags for Pressure, Temperature or either. */
    {MPL3115_PT_DATA_CFG,
     MPL3115_PT_DATA_CFG_TDEFE_ENABLED | MPL3115_PT_DATA_CFG_PDEFE_ENABLED | MPL3115_PT_DATA_CFG_DREM_ENABLED,
     MPL3115_PT_DATA_CFG_TDEFE_MASK | MPL3115_PT_DATA_CFG_PDEFE_MASK | MPL3115_PT_DATA_CFG_DREM_MASK},
    /* Set Over Sampling Ratio to 128. */
    {MPL3115_CTRL_REG1, MPL3115_CTRL_REG1_OS_OSR_2, MPL3115_CTRL_REG1_OS_MASK},
    /* Set Auto acquisition time step. */
    {MPL3115_CTRL_REG2, 0, MPL3115_CTRL_REG2_ST_MASK},
    __END_WRITE_DATA__};

/*! @brief Register settings for Altitude readings in Normal (non buffered) mode. */
const registerwritelist_t cMpl3115ConfigAltitude[] = {
    /* Enable Data Ready and Event flags for Altitude, Temperature or either. */
    {MPL3115_PT_DATA_CFG,
     MPL3115_PT_DATA_CFG_TDEFE_ENABLED | MPL3115_PT_DATA_CFG_PDEFE_ENABLED | MPL3115_PT_DATA_CFG_DREM_ENABLED,
     MPL3115_PT_DATA_CFG_TDEFE_MASK | MPL3115_PT_DATA_CFG_PDEFE_MASK | MPL3115_PT_DATA_CFG_DREM_MASK},
    /* Enable Altitude output and set Over Sampling Ratio to 128. */
    {MPL3115_CTRL_REG1, MPL3115_CTRL_REG1_ALT_ALT | MPL3115_CTRL_REG1_OS_OSR_2,
     MPL3115_CTRL_REG1_ALT_MASK | MPL3115_CTRL_REG1_OS_MASK},
    /* Set Auto acquisition time step. */
    {MPL3115_CTRL_REG2, 0, MPL3115_CTRL_REG2_ST_MASK},
    __END_WRITE_DATA__};

/*! @brief Address of DATA Ready Status Register. */
const registerreadlist_t cFxls8974DRDYEvent[] = {{.readFrom = FXLS8974_INT_STATUS, .numBytes = 1}, __END_READ_DATA__};

/*! @brief Address of Raw Accel Data in Normal Mode. */
const registerreadlist_t cFxls8974OutputNormal[] = {{.readFrom = FXLS8974_OUT_X_LSB, .numBytes = FXLS8974_DATA_SIZE},
                                                    __END_READ_DATA__};

/*! @brief Address and size of Raw Pressure+Temperature Data in Normal Mode. */
const registerreadlist_t cMpl3115OutputNormal[] = {{.readFrom = MPL3115_OUT_P_MSB, .numBytes = MPL3115_DATA_SIZE},
                                                   __END_READ_DATA__};

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile uint32_t x_count = 0U;
volatile uint32_t y_count = 0U;
volatile uint32_t z_count = 0U;
uint8_t blinkz_hz; //accel_z in g
uint8_t blinky_hz; //accel_y in g
uint8_t blinkx_hz; //accel_x in g
uint16_t blinkx_ms;
uint16_t blinky_ms;
uint16_t blinkz_ms;
GENERIC_DRIVER_GPIO *gpioDriver = &Driver_GPIO_KSDK;

//Moving average
uint8_t array_idx = 0;
float DATA_ARRAY [AVE_BUFFER_LENGHT] = {0};
float TEMP_ARRAY [AVE_BUFFER_LENGHT] = {0};


/*******************************************************************************
 * Code
 ******************************************************************************/
float Moving_Average(float* ARRAY);

/* LPTMR configure to interrupt every 1 ms */
void LPTMR_LED_HANDLER(void)
{
    LPTMR_ClearStatusFlags(DEMO_LPTMR_BASE, kLPTMR_TimerCompareFlag);
    x_count++;
    y_count++;
    z_count++;

    /* Z axis blink feature */
    if (blinkz_hz !=0){
		if ((z_count >= blinkz_ms/2)){
			gpioDriver->toggle_pin(&GREEN_LED);
			z_count = 0;
		}
    } else {
    	z_count =0;
    	gpioDriver->set_pin(&GREEN_LED);
    }

    /* Y axis blink feature */
    if ((blinky_hz !=0)){
		if ((y_count >= blinky_ms/2)){
			gpioDriver->toggle_pin(&BLUE_LED);
			y_count = 0;
		}
    } else {
    	y_count = 0;
    	gpioDriver->set_pin(&BLUE_LED);
    }

    /* X axis blink feature */
    if ((blinkx_hz !=0)){
		if ((x_count >= blinkx_ms/2)){
			gpioDriver->toggle_pin(&RED_LED);
			x_count = 0;
		}
    } else {
    	x_count = 0;
    	gpioDriver->set_pin(&RED_LED);
    }
    /*
     * Workaround for TWR-KV58: because write buffer is enabled, adding
     * memory barrier instructions to make sure clearing interrupt flag completed
     * before go out ISR
     */
    __DSB();
    __ISB();
}

/*!
 * @brief Main function
 */
int main(void)
{
    int32_t status;
    uint8_t whoami;
    uint8_t dataReady;
    uint8_t accelData[FXLS8974_DATA_SIZE];
    uint8_t mpl3115Data[MPL3115_DATA_SIZE];
    fxls8974_acceldata_t rawAccel;
#if MPL3115_MODE == PRESSURE_MODE
    mpl3115_pressuredata_t rawPressure;
    uint32_t pressureInPascals;
#else
    mpl3115_altitudedata_t rawAltitude;
    float altitudeInMeters;
#endif
    float tempInDegrees;
    float accel[3];

    ARM_DRIVER_I2C *I2Cdrv = &I2C_S_DRIVER; // Now using the shield.h value!!!
    fxls8974_i2c_sensorhandle_t fxls8974Driver;
    mpl3115_i2c_sensorhandle_t mpl3115Driver;

    // LPTMR
    uint32_t currentCounter = 0U;
    lptmr_config_t lptmrConfig;
    /* Board pin, clock, debug console init */
    CLOCK_EnableClock(kCLOCK_GateGPIO3);   //TODO REVIEW

    /* attach FRO 12M to LPUART0 (debug console) */
    CLOCK_SetClockDiv(kCLOCK_DivLPTMR0, 1u);
    CLOCK_AttachClk(kFRO12M_to_LPTMR0);

    CLOCK_SetupFRO16KClocking(kCLKE_16K_SYSTEM | kCLKE_16K_COREMAIN);

    /* Release peripheral RESET */
    RESET_PeripheralReset(kPORT3_RST_SHIFT_RSTn);
    RESET_PeripheralReset(kLPI2C0_RST_SHIFT_RSTn);

    /* Attach peripheral clock */
    CLOCK_SetClockDiv(kCLOCK_DivLPI2C0, 1u);
    CLOCK_AttachClk(kFRO12M_to_LPI2C0);

    /*! Initialize the MCU hardware. */
    BOARD_InitPins();
    BOARD_InitBootClocks();
    BOARD_SystickEnable();
    BOARD_InitDebugConsole();

    PRINTF("\r\n ISSDK FXLS8974 & MPL3115 sensor driver example demonstration\r\n");

    //LPTMR
    /* Configure LPTMR */
    /*
     * lptmrConfig.timerMode = kLPTMR_TimerModeTimeCounter;
     * lptmrConfig.pinSelect = kLPTMR_PinSelectInput_0;
     * lptmrConfig.pinPolarity = kLPTMR_PinPolarityActiveHigh;
     * lptmrConfig.enableFreeRunning = false;
     * lptmrConfig.bypassPrescaler = true;
     * lptmrConfig.prescalerClockSource = kLPTMR_PrescalerClock_1;
     * lptmrConfig.value = kLPTMR_Prescale_Glitch_0;
     */
    LPTMR_GetDefaultConfig(&lptmrConfig);

    /* Initialize the LPTMR */
    LPTMR_Init(DEMO_LPTMR_BASE, &lptmrConfig);

    /*
     * Set timer period.
     * Note : the parameter "ticks" of LPTMR_SetTimerPeriod should be equal or greater than 1.
     */
    LPTMR_SetTimerPeriod(DEMO_LPTMR_BASE, 16);  //16.384khz / 1000

    /* Enable timer interrupt */
    LPTMR_EnableInterrupts(DEMO_LPTMR_BASE, kLPTMR_TimerInterruptEnable);

    /* Enable at the NVIC */
    EnableIRQ(DEMO_LPTMR_IRQn);
    PRINTF("Low Power Timer Example\r\n");
    LPTMR_StartTimer(DEMO_LPTMR_BASE);

    /*! Initialize the I2C driver. */
    status = I2Cdrv->Initialize(I2C_S_SIGNAL_EVENT);
    if (ARM_DRIVER_OK != status)
    {
        PRINTF("\r\n I2C Initialization Failed\r\n");
        return -1;
    }

    /*! Set the I2C Power mode. */
    status = I2Cdrv->PowerControl(ARM_POWER_FULL);
    if (ARM_DRIVER_OK != status)
    {
        PRINTF("\r\n I2C Power Mode setting Failed\r\n");
        return -1;
    }

    /*! Set the I2C bus speed. */
    status = I2Cdrv->Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST);
    if (ARM_DRIVER_OK != status)
    {
        PRINTF("\r\n I2C Control Mode setting Failed\r\n");
        return -1;
    }

    /*! Initialize RGB LED pin used by FRDM board */
    gpioDriver->pin_init(&GREEN_LED, GPIO_DIRECTION_OUT, NULL, NULL, NULL);
    gpioDriver->pin_init(&RED_LED, GPIO_DIRECTION_OUT, NULL, NULL, NULL);
    gpioDriver->pin_init(&BLUE_LED, GPIO_DIRECTION_OUT, NULL, NULL, NULL);

    /*! Initialize FXLS8974 sensor driver. */
    status = FXLS8974_I2C_Initialize(&fxls8974Driver, &I2C_S_DRIVER, I2C_S_DEVICE_INDEX, FXLS8974_I2C_ADDR,
                                     &whoami);
    if (ARM_DRIVER_OK != status)
    {
        PRINTF("\r\n Sensor Initialization Failed\r\n");
        return -1;
    }
    if ((FXLS8964_WHOAMI_VALUE == whoami) || (FXLS8967_WHOAMI_VALUE == whoami))
    {
    	PRINTF("\r\n Successfully Initialized Gemini with WHO_AM_I = 0x%X\r\n", whoami);
    }
    else if ((FXLS8974_WHOAMI_VALUE == whoami) || (FXLS8968_WHOAMI_VALUE == whoami))
    {
    	PRINTF("\r\n Successfully Initialized Timandra with WHO_AM_I = 0x%X\r\n", whoami);
    }
    else if ((FXLS8971_WHOAMI_VALUE == whoami) || (FXLS8961_WHOAMI_VALUE == whoami))
    {
    	PRINTF("\r\n Successfully Initialized Chiron with WHO_AM_I = 0x%X\r\n", whoami);
    }
    else if (FXLS8962_WHOAMI_VALUE == whoami)
    {
    	PRINTF("\r\n Successfully Initialized Newstein with WHO_AM_I = 0x%X\r\n", whoami);
    }
    else
    {
    	PRINTF("\r\n Bad WHO_AM_I = 0x%X\r\n", whoami);
        return -1;
    }

    /*! Initialize MPL3115 sensor driver. */
    status = MPL3115_I2C_Initialize(&mpl3115Driver, &I2C_S_DRIVER, I2C_S_DEVICE_INDEX, MPL3115_I2C_ADDR,
                                    MPL3115_WHOAMI_VALUE);
    if (SENSOR_ERROR_NONE != status)
    {
        PRINTF("\r\n Sensor Initialization Failed\r\n");
        return -1;
    }
    PRINTF("\r\n Successfully Initialized MPL3115 Sensor\r\n");

    /*! Configure the FXLS8974 sensor. */
    status = FXLS8974_I2C_Configure(&fxls8974Driver, cFxls8974ConfigNormal);
    if (ARM_DRIVER_OK != status)
    {
        PRINTF("\r\n FXLS8974 Sensor Configuration Failed, Err = %d\r\n", status);
        return -1;
    }
    PRINTF("\r\n Successfully Applied FXLS8974 Sensor Configuration\r\n");

#if MPL3115_MODE == PRESSURE_MODE
    /*! Configure the MPL3115 sensor. */
    status = MPL3115_I2C_Configure(&mpl3115Driver, cMpl3115ConfigPressuree);
    if (SENSOR_ERROR_NONE != status)
    {
        PRINTF("\r\nMPL3115 configuration failed...\r\n");
        return -1;
    }
    PRINTF("\r\n Successfully Applied MPL3115 Sensor Configuration for Pressure mode\r\n");
#else
    /*! Configure the MPL3115 sensor. */
    status = MPL3115_I2C_Configure(&mpl3115Driver, cMpl3115ConfigAltitude);
    if (SENSOR_ERROR_NONE != status)
    {
        PRINTF("\r\nMPL3115 configuration failed...\r\n");
        return -1;
    }
    PRINTF("\r\n Successfully Applied MPL3115 Sensor Configuration for Altimeter mode\r\n");

#endif

    /* Reset LED Status*/
	gpioDriver->set_pin(&RED_LED);
	gpioDriver->set_pin(&BLUE_LED);
	gpioDriver->set_pin(&GREEN_LED);

	bool ALTITUDE_COMPENSATION = false;
	float ALTITUDE_FACTOR_CORRECTION = 0;

	memset(DATA_ARRAY, 0x00, sizeof(DATA_ARRAY));
	memset(TEMP_ARRAY, 0x00, sizeof(TEMP_ARRAY));
    for (;;) /* Forever loop */
    {
        /*! Wait for data ready from the FXLS8974. */
        status = FXLS8974_I2C_ReadData(&fxls8974Driver, cFxls8974DRDYEvent, &dataReady);
        if (0 == (dataReady & FXLS8974_INT_STATUS_SRC_DRDY_MASK))
        {
            continue;
        }

        /*! Read new raw sensor data from the FXLS8974. */
        status = FXLS8974_I2C_ReadData(&fxls8974Driver, cFxls8974OutputNormal, accelData);
        if (ARM_DRIVER_OK != status)
        {
            PRINTF("\r\n Read Failed. \r\n");
            return -1;
        }

        BOARD_DELAY_ms(10);
        /*! Read new raw sensor data from the MPL3115. */
        status = MPL3115_I2C_ReadData(&mpl3115Driver, cMpl3115OutputNormal, mpl3115Data);
        if (ARM_DRIVER_OK != status)
        {
            PRINTF("\r\n Read Failed. \r\n");
            return -1;
        }

        /*! Convert the raw sensor data to signed 16-bit container for display to the debug port. */
        rawAccel.accel[0] = ((int16_t)accelData[1] << 8) | accelData[0];
        rawAccel.accel[1] = ((int16_t)accelData[3] << 8) | accelData[2];
        rawAccel.accel[2] = ((int16_t)accelData[5] << 8) | accelData[4];

        /*! Convert raw values to Gs */
        accel[0] = (float) (rawAccel.accel[0] * ACCEL_2G_SENS);
        accel[1] = (float) (rawAccel.accel[1] * ACCEL_2G_SENS);
        accel[2] = (float) (rawAccel.accel[2] * ACCEL_2G_SENS);

#if MPL3115_MODE == PRESSURE_MODE
        /*! Process the pressure samples and convert the raw values to Pa. */
        //Pressure is represented in Q18.2 fixed-point format
        rawPressure.pressure = (mpl3115Data[0] << 10) + (mpl3115Data[1] << 2) + ((mpl3115Data[2] & 0xC0) >> 6);
        pressureInPascals = rawPressure.pressure; /* pressure in Pascals (decimal point precision) */
        if (mpl3115Data[2] & 0x20) {pressureInPascals += 0.500;}
        if (mpl3115Data[2] & 0x10) {pressureInPascals += 0.250;}

    	/* Temperature data*/
        rawPressure.temperature = (int16_t)((mpl3115Data[3]) << 8) | (mpl3115Data[4]);
        tempInDegrees = rawPressure.temperature / MPL3115_TEMPERATURE_CONV_FACTOR;

	    /* MOVING_AVERAGE computation for magnitude indicator on display */
	    DATA_ARRAY[array_idx] = pressureInPascals;
	    pressureInPascals = Moving_Average(DATA_ARRAY);

#else
        /*! Process the sample and convert the raw sensor data. */
        //Altitude is represented in Q16.4 fixed-point format
        rawAltitude.altitude = (int16_t)((mpl3115Data[0]) << 8) + (mpl3115Data[1]);
        altitudeInMeters = rawAltitude.altitude; /* altitude in meters (decimal point precision) */
        if (mpl3115Data[2] & 0x80) {altitudeInMeters += 0.5000;}
        if (mpl3115Data[2] & 0x40) {altitudeInMeters += 0.2500;}
        if (mpl3115Data[2] & 0x20) {altitudeInMeters += 0.1250;}
    	if (mpl3115Data[2] & 0x10) {altitudeInMeters += 0.0625;}

    	/* Temperature data*/
        rawAltitude.temperature = (int16_t)((mpl3115Data[3]) << 8) | (mpl3115Data[4]);
        tempInDegrees = rawAltitude.temperature / MPL3115_TEMPERATURE_CONV_FACTOR;

        /* Compensation for ALTITUDE: only in ALT MODE. */
	    if (!ALTITUDE_COMPENSATION){
	    	ALTITUDE_COMPENSATION = true; /* Enters only once at start of the loop. */

	    	/* Compensation for altimeter applied to sensor. */
	    	ALTITUDE_FACTOR_CORRECTION = -altitudeInMeters;
	    }

	    /* MOVING_AVERAGE computation for magnitude indicator on display */
	    DATA_ARRAY[array_idx] = altitudeInMeters;
	    altitudeInMeters = Moving_Average(DATA_ARRAY);

	    /*Altitude compensation */
	    altitudeInMeters += ALTITUDE_FACTOR_CORRECTION;
#endif

        /*! Process the Temperature samples and convert the raw values to degC. */
    	//Temperature is represented in Q8.4 fixed-point format
        tempInDegrees = (int16_t)(mpl3115Data[3]);
    	if (mpl3115Data[4] & 0x80) {tempInDegrees += 0.5000;}
    	if (mpl3115Data[4] & 0x40) {tempInDegrees += 0.2500;}
    	if (mpl3115Data[4] & 0x20) {tempInDegrees += 0.1250;}
    	if (mpl3115Data[4] & 0x10) {tempInDegrees += 0.0625;}

		//Moving average for TEMPERATURE sensor data
		TEMP_ARRAY[array_idx] = tempInDegrees;
		tempInDegrees = Moving_Average(TEMP_ARRAY); /* Both PRESSURE and ALTITUDE modes */

		array_idx++;
		if (array_idx == AVE_BUFFER_LENGHT) /* Restart pointer to arrays for mov.average */
			array_idx = 0;

    	 /* Status LEDs*/
		// Implement blinking frequency inside this statements.
		if (fabsf(accel[2]) > ACCEL_THRESHOLD_zHIGH){
			blinkz_hz = 20 * fabsf(accel[2]) -11;  //accel_z in g
			blinkz_ms = 1000 / blinkz_hz;
		}
		else {
			blinkz_hz = 0; //disables the LED toggle functionality.
		}

		if (fabsf(accel[1]) > ACCEL_THRESHOLD_xy){
			blinky_hz = 20 *fabsf(accel[1]);  //accel_y in g
			blinky_ms = 1000 / blinky_hz;
		}
		else {
			blinky_hz = 0; //disables the LED toggle functionality.
		}

		if (fabsf(accel[0]) > ACCEL_THRESHOLD_xy){
			blinkx_hz = 20 *fabsf(accel[0]);  //accel_x in g
			blinkx_ms = 1000 / blinkx_hz;
		}
		else {
			blinkx_hz = 0; //disables the LED toggle functionality.
		}

        /* NOTE: PRINTF is relatively expensive in terms of CPU time, specially when used with-in execution loop. */
		if (rawAccel.accel[0] < 0.0)
			PRINTF("\r\n Accel X\t= -%2.3f g\r\n", fabsf(accel[0]));
		else
			PRINTF("\r\n Accel X\t= %2.3f g\r\n", accel[0]);

		if (rawAccel.accel[1] < 0.0)
			PRINTF("\r\n Accel Y\t= -%2.3f g\r\n", fabsf(accel[1]));
		else
			PRINTF("\r\n Accel Y\t= %2.3f g\r\n", accel[1]);

		if (rawAccel.accel[2] < 0.0)
			PRINTF("\r\n Accel Z\t= -%2.3f g\r\n", fabsf(accel[2]));
		else
			PRINTF("\r\n Accel Z\t= %2.3f g\r\n", accel[2]);

#if MPL3115_MODE == PRESSURE_MODE
        PRINTF("\r\n Pressure\t= %d Pa\r\n", pressureInPascals);
#else
        PRINTF("\r\n Altitude\t= %.4f Meters\r\n", altitudeInMeters);
#endif
        PRINTF("\r\n Temperature\t= %.4f degC\r\n\r\n", tempInDegrees);

    }
}

float Moving_Average(float* ARRAY)
{
  /* Moving average for Sensor data */
  float cum = 0;

  for (uint8_t i=0; i< AVE_BUFFER_LENGHT; i++){
    cum += ARRAY[i];
  }

  return cum / AVE_BUFFER_LENGHT;
}
