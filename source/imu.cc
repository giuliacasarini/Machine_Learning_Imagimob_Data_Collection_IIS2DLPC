/******************************************************************************
* File Name:   imu.c
*
* Description: This file implements the interface with the motion sensor, as
*              a timer to feed the pre-processor at 50Hz.
*
* Related Document: See README.md
*
*
*******************************************************************************/
#include "imu.h"



#include "cyhal.h"
#include "cybsp.h"

#include "config.h"


/*******************************************************************************
* Macros
*******************************************************************************/
#ifdef CY_BMX_160_IMU_SPI
    #define IMU_SPI_FREQUENCY 10000000
#endif

#ifdef CY_BMI_160_IMU_SPI
    #define IMU_SPI_FREQUENCY 10000000
#endif

#ifdef CY_BMI_160_IMU_I2C
    #define IMU_I2C_MASTER_DEFAULT_ADDRESS  0
    #define IMU_I2C_FREQUENCY               1000000
#endif

#define IMU_SCAN_RATE       50
#define IMU_TIMER_FREQUENCY 100000
#define IMU_TIMER_PERIOD (IMU_TIMER_FREQUENCY/IMU_SCAN_RATE)
#define IMU_TIMER_PRIORITY  3
/*******************************************************************************
* Global Variables
*******************************************************************************/

//RV  pointer to driver object
IIS2DLPCSensor * psensor;

/* Global timer used for getting data */
cyhal_timer_t imu_timer;

float imu_data[IMU_AXIS];
/*******************************************************************************
* Local Function Prototypes
*******************************************************************************/
void imu_interrupt_handler(void* callback_arg, cyhal_timer_event_t event);
cy_rslt_t imu_timer_init(void);

/*******************************************************************************
* Function Name: imu_init
********************************************************************************
* Summary:
*    A function used to initialize the IMU based on the shield selected in the
*    makefile. Starts a timer that triggers an interrupt at 50Hz.
*
* Parameters:
*   None
*
* Return:
*     The status of the initialization.
*
*
*******************************************************************************/

/* I2C bus frequency */
#define I2C_FREQ                (400000UL)
cyhal_i2c_t mI2C;

cy_rslt_t imu_init(void)
{
    cy_rslt_t result;
    uint8 ris;


    cyhal_i2c_cfg_t mI2C_cfg;

    mI2C_cfg.is_slave = false;
    mI2C_cfg.address = 0;
    mI2C_cfg.frequencyhal_hz = I2C_FREQ;

     /* Init I2C master */
     result = cyhal_i2c_init(&mI2C, CYBSP_I2C_SDA, CYBSP_I2C_SCL, NULL);
     if (CY_RSLT_SUCCESS != result)
         {
             return result;
         }

     /* Configure I2C Master */
     result = cyhal_i2c_configure(&mI2C, &mI2C_cfg);
     if (CY_RSLT_SUCCESS != result)
         {
             return result;
         }


     psensor = new IIS2DLPCSensor(&mI2C,0x19);

    psensor->begin();
   	psensor->Enable();

   	// TODO: set sample rate e sample range
   	//if (!ris)
   	//	return -1;


    imu_flag = false;

    /* Timer for data collection */
    result = imu_timer_init();

    return result;

}


/*******************************************************************************
* Function Name: imu_timer_init
********************************************************************************
* Summary:
*   Sets up an interrupt that triggers at the desired frequency.
*
* Returns:
*   The status of the initialization.
*
*
*******************************************************************************/
cy_rslt_t imu_timer_init(void)
{
    cy_rslt_t rslt;
    const cyhal_timer_cfg_t timer_cfg =
    {
		.is_continuous = true,              /* Run the timer indefinitely */
		.direction = CYHAL_TIMER_DIR_UP,    /* Timer counts up */
		.is_compare = false,                /* Don't use compare mode */
		.period = IMU_TIMER_PERIOD,      /* Defines the timer period */
		.compare_value = 0,                 /* Timer compare value, not used */
		.value = 0                          /* Initial value of counter */
    };



    /* Initialize the timer object. Does not use pin output ('pin' is NC) and
     * does not use a pre-configured clock source ('clk' is NULL). */
    rslt = cyhal_timer_init(&imu_timer, NC, NULL);
    if (CY_RSLT_SUCCESS != rslt)
    {
        return rslt;
    }

    /* Apply timer configuration such as period, count direction, run mode, etc. */
    rslt = cyhal_timer_configure(&imu_timer, &timer_cfg);
    if (CY_RSLT_SUCCESS != rslt)
    {
        return rslt;
    }

    /* Set the frequency of timer to 100KHz */
    rslt = cyhal_timer_set_frequency(&imu_timer, IMU_TIMER_FREQUENCY);
    if (CY_RSLT_SUCCESS != rslt)
    {
        return rslt;
    }

    /* Assign the ISR to execute on timer interrupt */
    cyhal_timer_register_callback(&imu_timer, imu_interrupt_handler, NULL);
    /* Set the event on which timer interrupt occurs and enable it */
    cyhal_timer_enable_event(&imu_timer, CYHAL_TIMER_IRQ_TERMINAL_COUNT, IMU_TIMER_PRIORITY, true);
    /* Start the timer with the configured settings */
    rslt = cyhal_timer_start(&imu_timer);
    if (CY_RSLT_SUCCESS != rslt)
    {
        return rslt;
    }

    return CY_RSLT_SUCCESS;
}

/*******************************************************************************
* Function Name: imu_interrupt_handler
********************************************************************************
* Summary:
*   Interrupt handler for timer. Interrupt handler will get called at 50Hz and
*   sets a flag that can be checked in main.
*
* Parameters:
*     callback_arg: not used
*     event: not used
*
*
*******************************************************************************/
void imu_interrupt_handler(void *callback_arg, cyhal_timer_event_t event)
{
    (void) callback_arg;
    (void) event;

    imu_flag = true;
}

/*******************************************************************************
* Function Name: imu_get_data
********************************************************************************
* Summary:
*   Reads accelerometer data from the IMU and stores it in a buffer.
*
* Parameters:
*     imu_data: Stores IMU accelerometer data
*
*
*******************************************************************************/
void imu_get_data(float *imu_data)
{
    /* Read data from IMU sensor */
    cy_rslt_t result;
    int32_t acceleration[3];
    psensor->GetAxes(acceleration);

    imu_data[0] = ((float)acceleration[0]) / (float)0x1000;
    imu_data[1] = ((float)acceleration[1]) / (float)0x1000;
    imu_data[2] = ((float)acceleration[2]) / (float)0x1000;

}
