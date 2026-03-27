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
* (c) 2025, Infineon Technologies AG, or an affiliate of Infineon
* Technologies AG. All rights reserved.
* This software, associated documentation and materials ("Software") is
* owned by Infineon Technologies AG or one of its affiliates ("Infineon")
* and is protected by and subject to worldwide patent protection, worldwide
* copyright laws, and international treaty provisions. Therefore, you may use
* this Software only as provided in the license agreement accompanying the
* software package from which you obtained this Software. If no license
* agreement applies, then any use, reproduction, modification, translation, or
* compilation of this Software is prohibited without the express written
* permission of Infineon.
* 
* Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
* IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
* INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
* THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
* SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
* Infineon reserves the right to make changes to the Software without notice.
* You are responsible for properly designing, programming, and testing the
* functionality and safety of your intended application of the Software, as
* well as complying with any legal requirements related to its use. Infineon
* does not guarantee that the Software will be free from intrusion, data theft
* or loss, or other breaches ("Security Breaches"), and Infineon shall have
* no liability arising out of any Security Breaches. Unless otherwise
* explicitly approved by Infineon, the Software may not be used in any
* application where a failure of the Product or any consequences of the use
* thereof can reasonably be expected to result in personal injury.
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
