/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Imagimob Data Collection Example
*              for ModusToolbox.
*
* Related Document: See README.md 
*
*
*******************************************************************************
* Copyright 2021-2022, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#include "cyhal.h"
#include "cybsp.h"
#include "stdlib.h"

#include "imu.h"
#include "audio.h"
#include "config.h"
#include "streaming.h"

/*******************************************************************************
* Global Variables
********************************************************************************/
volatile bool pdm_pcm_flag;
volatile bool imu_flag;
volatile bool send_data = false;

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void gpio_interrupt_handler(void* handler_arg, cyhal_gpio_event_t event);

cyhal_gpio_callback_data_t cb_data =
{
    .callback     = gpio_interrupt_handler,
    .callback_arg = NULL
};
/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  This is the main function. It sets up either the PDM or IMU based on the
*  config.h file. main continuously checks flags, signaling that data is ready
*  to be streamed over UART or USB and initiates the transfer.
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Setup button to press, to initiate data transfer */
    cyhal_gpio_init(CYBSP_USER_BTN1, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, 0);
    cyhal_gpio_register_callback(CYBSP_USER_BTN1, &cb_data);
    cyhal_gpio_enable_event(CYBSP_USER_BTN1, CYHAL_GPIO_IRQ_FALL, 3, true);

    /* Initialize the streaming interface */
    mtb_data_streaming_interface_t  stream;
    streaming_init(&stream);

    /* Initialize IMU transmit buffers */
    uint8_t transmit_imu[4 * IMU_AXIS] = {0};
    float *imu_raw_data = (float*) transmit_imu;

    /* Start the imu and timer */
    result = imu_init();

    /* Initialization failed */
    if(CY_RSLT_SUCCESS != result)
    {
        /* Reset the system on sensor fail */
        NVIC_SystemReset();
    }

    /* Wait until the kit button is pressed */
    while(false == send_data);

    for(;;)
    {
    	if (send_data){
			/* Transmit IMU data or PDM data based on config.h */
			if(true == imu_flag)
			{
				imu_flag = false;
				/* Store IMU data */
				imu_get_data(imu_raw_data);

				/* Transmit data over UART */
				mtb_data_streaming_send(&stream, transmit_imu, sizeof(transmit_imu), NULL);
			}
    	}

    }
}

/*******************************************************************************
* Function Name: gpio_interrupt_handler
********************************************************************************
* Summary:
*  When the on kit button is pressed, this sets a flag and exits.
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
void gpio_interrupt_handler(void* handler_arg, cyhal_gpio_event_t event)
{
    (void) handler_arg;
    (void) event;

    /* toggle flag */
    send_data = !send_data;
}

/* [] END OF FILE */
