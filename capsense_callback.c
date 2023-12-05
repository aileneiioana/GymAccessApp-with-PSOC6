/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
/* Header file includes */ 
#include "ipc/cy_ipc_drv.h"
#include "ipc/cy_ipc_pipe.h"
#include "ipc/cy_ipc_sema.h"
#include "cy_ipc_config.h"
#include "CapSense_Sensing.h"
#include "cyapicallbacks.h"
#include "task_touch.h"

/*******************************************************************************
* Function Name: void CapSense_EntryCallback(void)
********************************************************************************
* Summary:
*  Used at the beginning of the CapSense interrupt handler to perform additional 
*  application specific actions.
*
* Parameters:
*  NULL
*
* Return:
*  void
*
*******************************************************************************/
void CapSense_EntryCallback()
   {
   /* Not used in this project */
   }

/*******************************************************************************
* Function Name: void CapSense_ExitCallback(void)
********************************************************************************
* Summary:
*  Used at the end of the CapSense interrupt handler to perform additional 
*  application-specific actions.
*
* Parameters:
*  NULL
*
* Return:
*  void
*
*******************************************************************************/
void CapSense_ExitCallback()
{
#if(CY_CPU_CORTEX_M4)
    
   /* Do this only when CapSense isn't busy with an ongoing scan and CapSense block is initialized*/
    if(CapSense_IsBusy()==CapSense_NOT_BUSY && Capsense_initialized)
    {
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
        /* Notify the task that Scan is completed. */     
        vTaskNotifyGiveFromISR(touchTaskHandle, &xHigherPriorityTaskWoken);
        /**
        * If xHigherPriorityTaskWoken is now set to pdTRUE then a context switch
        *    should be performed to ensure the interrupt returns directly to the highest
        *    priority task.  The macro used for this purpose is dependent on the port in
        *    use and may be called portEND_SWITCHING_ISR().
        **/
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }           
#endif
}
/* [] END OF FILE */
