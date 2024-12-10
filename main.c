/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the DeepSleep/Hibernate wakeup using
*              a low-power comparator for ModusToolbox.
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
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

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cybsp.h"
#include "cy_pdl.h"
#include "mtb_hal.h"
#include "cy_retarget_io.h"


/*******************************************************************************
* Macros
********************************************************************************/
/* Enables code example to wakeup CPU from Hibernate mode:
 * 0U - wakeup from DeepSleep mode (default).
 * 1U - wakeup from Hibernate mode.
 */
#define WAKEUP_FROM_HIBERNATE_ENABLED               (0U)

/* Power mode string definition for printf */
#if (WAKEUP_FROM_HIBERNATE_ENABLED == 1U)
#define POWER_MODE_STRING                           "Hibernate mode"
#else
#define POWER_MODE_STRING                           "DeepSleep mode"
#endif

/*******************************************************************************
* Global Variables
********************************************************************************/
/* For the Retarget-IO (Debug UART) usage */
static cy_stc_scb_uart_context_t  DEBUG_UART_context;   /* Debug UART context */
static mtb_hal_uart_t DEBUG_UART_hal_obj;               /* Debug UART HAL object */

/* User low-power comparator context */
cy_stc_lpcomp_context_t user_lpcomp_context;

#if (WAKEUP_FROM_HIBERNATE_ENABLED != 1U)
/* User low-power comparator interrupt configuration structure */
cy_stc_sysint_t user_lpcomp_intr_config =
{
    .intrSrc = USER_LPCOMP_IRQ,
    .intrPriority = 0U,
};
#endif

/*******************************************************************************
* Function Prototypes
********************************************************************************/
#if (WAKEUP_FROM_HIBERNATE_ENABLED != 1U)
/* User low-power comparator interrupt handler */
void user_lpcomp_intr_handler(void);
#endif


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CPU.
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
    result = cybsp_init();
    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Check the IO status. If current status is frozen, unfreeze the system. */
    if (Cy_SysPm_GetIoFreezeStatus())
    {
        /* Unfreeze the system */
        Cy_SysPm_IoUnfreeze();
    }

    /* Initialize the debug UART */
    result = Cy_SCB_UART_Init(DEBUG_UART_HW, &DEBUG_UART_config, &DEBUG_UART_context);
    /* UART init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
    Cy_SCB_UART_Enable(DEBUG_UART_HW);

    /* Setup the HAL UART */
    result = mtb_hal_uart_setup(&DEBUG_UART_hal_obj, &DEBUG_UART_hal_config, &DEBUG_UART_context, NULL);
    /* HAL UART init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init(&DEBUG_UART_hal_obj);
    /* retarget-io init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize low-power comparator */
    if (CY_LPCOMP_SUCCESS != Cy_LPComp_Init_Ext(USER_LPCOMP_HW, USER_LPCOMP_CHANNEL, 
                                        &USER_LPCOMP_config, &user_lpcomp_context))
    {
        CY_ASSERT(0);
    }

#if (WAKEUP_FROM_HIBERNATE_ENABLED != 1U)
    /* Configure LPCOMP interrupt */
    Cy_LPComp_SetInterruptMask(USER_LPCOMP_HW, CY_LPCOMP_COMP1);
    Cy_SysInt_Init(&user_lpcomp_intr_config, user_lpcomp_intr_handler);
    NVIC_EnableIRQ(user_lpcomp_intr_config.intrSrc);
#endif

    /* Enable LPCOMP */
    Cy_LPComp_Enable_Ext(USER_LPCOMP_HW, USER_LPCOMP_CHANNEL, &user_lpcomp_context);

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");
    printf("********************************************************************************\r\n");
    printf("Wakeup from %s using a low-power comparator\r\n", POWER_MODE_STRING);
    printf("********************************************************************************\r\n\r\n");

    /* Check the reset reason */
    if(CY_SYSLIB_RESET_HIB_WAKEUP == (Cy_SysLib_GetResetReason() & CY_SYSLIB_RESET_HIB_WAKEUP))
    {
        /* The reset has occurred on a wakeup from Hibernate power mode */
        printf("Wakeup from the Hibernate mode\r\n\r\n");
    }

    /* Enable global interrupts */
    __enable_irq();

    for (;;)
    {
        /* If the comparison result is high, toggles user LED every 500ms */
        if(0 != Cy_LPComp_GetCompare(USER_LPCOMP_HW, USER_LPCOMP_CHANNEL))
        {
            /* Toggle User LED1 every 500ms */
            Cy_GPIO_Inv(CYBSP_USER_LED_PORT, CYBSP_USER_LED_NUM);
            Cy_SysLib_Delay(500u);
            printf("In CPU Active mode, blinking user LED at 500ms\r\n\r\n");
        }
        else /* If the comparison result is low, goes to the DeepSleep/Hibernate mode */
        {
            /* Turn on user LED for 2 seconds to indicate the MCU entering DeepSleep/Hibernate mode. */
            printf("Turn on the USER LED for 2 seconds, then enter %s\r\n\r\n", POWER_MODE_STRING);
            Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_NUM, CYBSP_LED_STATE_ON);
            Cy_SysLib_Delay(2000u);
            Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_NUM, CYBSP_LED_STATE_OFF);

#if (WAKEUP_FROM_HIBERNATE_ENABLED == 1U)
            /*Release the UART interface allowing it to be used for other purposes*/
            cy_retarget_io_deinit();

            /* Set the low-power comparator as a wake-up source from Hibernate and jump into Hibernate */
            Cy_SysPm_SetHibernateWakeupSource(CY_SYSPM_HIBERNATE_LPCOMP1_HIGH);
            if(CY_SYSPM_SUCCESS != Cy_SysPm_SystemEnterHibernate())
            {
                printf("The CPU did not enter Hibernate mode\r\n\r\n");
                CY_ASSERT(0);
            }
#else /* DeepSleep mode */
            /* Wait for UART traffic to stop */
            while (cy_retarget_io_is_tx_active());

            /* Sets CPU to the Deep Sleep mode */
            if(CY_SYSPM_SUCCESS != Cy_SysPm_CpuEnterDeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT))
            {
                printf("The CPU did not enter DeepSleep mode\r\n\r\n");
                CY_ASSERT(0);
            }
            printf("Wake up from DeepSleep mode\r\n\r\n");
#endif
        }
    }
}

#if (WAKEUP_FROM_HIBERNATE_ENABLED != 1U)
/*******************************************************************************
* Function Name: user_lpcomp_intr_handler
********************************************************************************
* Summary:
* This function is the user low-power comparator interrupt handler.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void user_lpcomp_intr_handler(void)
{
    uint32_t intrStatus = Cy_LPComp_GetInterruptStatusMasked(USER_LPCOMP_HW);
    /* Clear interrupt */
    Cy_LPComp_ClearInterrupt(USER_LPCOMP_HW, intrStatus);
}
#endif

/* [] END OF FILE */
