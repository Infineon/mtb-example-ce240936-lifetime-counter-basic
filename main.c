/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for PSOC4 wake up from DEEPSLEEP using
*              Lifetime Counter example for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2025, Cypress Semiconductor Corporation (an Infineon company) or
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

/******************************************************************************
* Header Files
*******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include "cycfg_peripherals.h"

/*******************************************************************************
* Macros
********************************************************************************/
/* Just wait milliseconds to blink LED */
#define LED_BLINK_EXTENTION (10u)

/*******************************************************************************
* Global Variables
********************************************************************************/
static const cy_stc_sysint_t interruptConfig =
{
    .intrSrc = srss_interrupt_srss_IRQn,
    .intrPriority = 3UL
};

/*******************************************************************************
* Function Prototypes
********************************************************************************/
static void ltcInterruptHandler(void);
static void resetLtc(void);

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  1. Initializes BSP and Lifetime Counter is initialized in the function with
*     the values configured by Device Configurator.
*  2. Initialize Lifetime Counter interrupt.
*  3. In the for loop,
*     3.1. In ACTIVE mode, LED6 turns on several milliseconds.
*     3.2. Put into DEEPSLEEP mode, LED6 turns off.
*
* Parameters:
*  void
*
* Return:
*  int
*
********************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize interrupt and interrupt handler */
    cy_rslt_t rslt = Cy_SysInt_Init(&interruptConfig, ltcInterruptHandler);

    if (rslt != CY_SYSINT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable NVIC */
    NVIC_ClearPendingIRQ(interruptConfig.intrSrc);
    NVIC_EnableIRQ(interruptConfig.intrSrc);

    for (;;)
    {
        /* In ACTIVE mode: LED6-ON */
        Cy_GPIO_Write(CYBSP_LED6_PORT, CYBSP_LED6_PIN, true);

        /* Keep ACTIVE mode */
        Cy_SysLib_Delay(LED_BLINK_EXTENTION);

        /* In DEEPSLEEP mode: LED6-OFF */
        Cy_GPIO_Write(CYBSP_LED6_PORT, CYBSP_LED6_PIN, false);

        /* Put into DEEPSLEEP */
        Cy_SysPm_CpuEnterDeepSleep();
    }
}

/*******************************************************************************
* Function Name: ltcInterruptHandler
********************************************************************************
* Summary:
*  Clear interrupt source and reset lifetime Counter.
*
* Parameters:
*  void
*
* Return:
*  void
*
********************************************************************************/
static void ltcInterruptHandler(void)
{
    /* Get Lifetime Counter interrupt status */
    bool intrStatus = Cy_LTC_GetInterruptStatusMasked();

    if (true == intrStatus)
    {
        /* Clear handled Lifetime Counter interrupt source */
        Cy_LTC_ClearInterrupt();

        /* Reset Lifetime counter */
        resetLtc();
    }
}

/*******************************************************************************
* Function Name: resetLtc
********************************************************************************
* Summary:
*  Reset Lifetime Counter using the generated code by Device Configurator.
*  Lifetime Counter configuration can be changed on Device Configurator.
*
* Parameters:
*  void
*
* Return:
*  void
*
********************************************************************************/
static void resetLtc(void)
{
    /* Call Lifetime Counter init function generated by Device Configurator */
    init_cycfg_peripherals();
}

/* [] END OF FILE */
