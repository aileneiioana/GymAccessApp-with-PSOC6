/******************************************************************************
* File Name: main_cm4.c
*
* Version: 1.00
*
* Description: The project includes CapSense configuration implementing two 
* buttons and a five-element slider, FreeRTOS configuration on Cortex-M4 and
* FreeRTOS task implementation for CapSense and LED interface. In addition, 
* the project defines and implements the CapSense exit/entry callbacks through
* the “cyapicallbacks.h” file. The CapSense task initializes the CapSense block,
* scans the widgets periodically, processes the widgets after each scan and 
* notifies the LED task to display the output. The LED task controls the RGB LED.
* The slider value controls the intensity and the buttons control ON/OFF of the
* LEDs. The CapSense entry/exit callbacks provide the CapSense task resume/wakeup
* functionality on CapSense scan completion. 
*
* Related Document: 
*
* Hardware Dependency: CY8CKIT-062-BLE PSoC 6 BLE Pioneer Kit
*                     
*
*******************************************************************************
* Copyright (2018), Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* (“Software”), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries (“Cypress”) and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software (“EULA”).
*
* If no EULA applies, Cypress hereby grants you a personal, nonexclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress’s integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, 
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED 
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress 
* reserves the right to make changes to the Software without notice. Cypress 
* does not assume any liability arising out of the application or use of the 
* Software or any product or circuit described in the Software. Cypress does 
* not authorize its products for use in any products where a malfunction or 
* failure of the Cypress product may reasonably be expected to result in 
* significant property damage, injury or death (“High Risk Product”). By 
* including Cypress’s product in a High Risk Product, the manufacturer of such 
* system or application assumes all risk of such use and in doing so agrees to 
* indemnify Cypress against all liability.
*******************************************************************************/
/******************************************************************************
* This code example demonstrates the capabilities of the PSoC 6 BLE to 
* communicate bi-directionally with a BLE Central device over custom services, 
* while performing CapSense touch sensing and updating GUI elements such as an 
* RGB LED and an E-INK display. The CapSense custom service allows notifications  
* to be sent to the central device when notifications are enabled. On the other  
* hand, the RGB LED custom service allows read and write of attributes under the 
* RGB characteristics.
*
* This project utilizes CapSense component to read touch information from a
* slider two buttons button and then report this to central device over BLE.  
* On the other hand, the control values sent to PSoC 6 BLE is converted to  
* respective color and intensity values and displayed using the on-board  
* RGB LED. The BLE central device can be CySmart mobile app or CySmart BLE 
* Host Emulation PC tool. 
*
* This code example uses FreeRTOS. For documentation and API references of 
* FreeRTOS, visit : https://www.freertos.org 
*
*******************************************************************************/
/* Header file includes */
/* Header file includes */ 
#include "project.h"
#include "FreeRTOS.h"  
#include "task_touch.h"
#include "task_rgb.h"

#include "task.h"
#include "semphr.h"
#include <stdio.h>
#include <limits.h>

#include "queue.h"
#include "timers.h"
#include "uart_debug.h"
/* Header file includes */
#include "task_touch.h"
#include "uart_debug.h"
#include "task_rgb.h"

int counter=0;

#define LED_ON  0
#define LED_OFF 1

// This is used to lock and unlock the BLE Task
SemaphoreHandle_t bleSemaphore;
SemaphoreHandle_t counterSemaphore;

/* Handle for the Queue that contains RGB LED data */   
QueueHandle_t rgbLedDataQ;
/* Stack sizes of user tasks in this project */
#define TASK_RGB_STACK_SIZE      (configMINIMAL_STACK_SIZE)
#define TASK_TOUCH_STACK_SIZE    (configMINIMAL_STACK_SIZE)
    
/* Priorities of user tasks in this project - spaced at intervals of 5 for 
the ease of further modification and addition of new tasks. 
Larger number indicates higher priority. */
#define TASK_RGB_PRIORITY       1
#define TASK_TOUCH_PRIORITY     5
    
 /* Maximum number of messages that can be queued */
#define RGB_LED_QUEUE_LEN    (1) 

/* Variable to monitor CapSense block initialization*/ 
bool Capsense_initialized=false;


/* Queue handle for debug message Queue */       
QueueHandle_t debugMessageQ;

/*******************************************************************************
* Function Name: void Task_Debug(void *pvParameters)
********************************************************************************
* Summary:
*  Task that prints a debug message via UART STDIO
*
* Parameters:
*  void *pvParameters : Task parameter defined during task creation (unused)                            
*
* Return:
*  void
*
*******************************************************************************/    
void Task_Debug(void *pvParameters)
{
    /* Variable that stores the data to be printed */
    debug_print_data_t dataToPrint;
    
   /* Variable used to store the return values of RTOS APIs */
    BaseType_t rtosApiResult;
    
    /* Remove warning for unused parameter */
    (void) pvParameters;
    
    /* Repeatedly running part of the task */    
    for(;;)
    {
        /* Block until a message to printed has been received over 
           debugMessageQ */
        rtosApiResult = xQueueReceive(debugMessageQ, &dataToPrint,
                                      portMAX_DELAY);
        
        /* Message has been received from debugMessageQ */
        if(rtosApiResult == pdTRUE)
        {
            /* If the error code is not 0, print message string along with the
               error code (as 32-bit hexadecimal data) */
            if(dataToPrint.errorCode != 0u)
            {
                DebugPrintf("%s %"PRIX32" \r\n", dataToPrint.stringPointer,
                            dataToPrint.errorCode);
            }
            /* Otherwise, print the message string only */
            else
            {
                DebugPrintf("%s \r\n", dataToPrint.stringPointer);
            }
        }    
    }
}

/*******************************************************************************
* Function Name: void Task_Touch(void *pvParameters)   
********************************************************************************
* Summary:
*  Task that reads touch data from CapSense button and slider widgets   
*
* Parameters:
*  void *pvParameters : Task parameter defined during task creation (unused)                            
*
* Return:
*  void
*
*******************************************************************************/
void Task_Touch(void *pvParameters)    
{  
    Task_DebugPrintf(" 1. Task-Task_Touch : Scanning Capsense Widgets Periodically\r\n",0u);
    
    /* Variable that stores CapSense API results */
    cy_status capSenseApiResult;    
    /* Variable used to store the return values of RTOS APIs */
    BaseType_t rtosApiResult;       
    /* Variables that store the previous and the current touch data */   
    touch_data_t touchData =
    {   
        .dataButton0 = false,
        .dataButton1 = false,
        .dataSlider  = (uint8_t) CapSense_SLIDER_NO_TOUCH
    };
    bool    button0Data          = false;
    bool    button1Data          = false;
    uint8_t sliderData           = (uint8_t) CapSense_SLIDER_NO_TOUCH;
    /* Remove warning for unused parameter */
    (void)pvParameters ;

    /* Start the CapSense component and initialize the baselines */
    capSenseApiResult = CapSense_Start();
    
    /* Check if the operation has been successful */
    if (capSenseApiResult== CY_RET_SUCCESS)
    {
      //DEBUG_PRINTF("\r\nSuccess  : Touch - CapSense initialization\r\n");
    Capsense_initialized=true;   
    
    }
    else
    {
        Task_DebugPrintf("Failure! : Touch - CapSense initialization\r\n",capSenseApiResult);
    }
    
    /* Repeatedly running part of the task */
    for(;;)
    {
             /* Process data from the widgets */
    	CapSense_ProcessAllWidgets();                      
        /* Read touch data from the widgets */                                         
        touchData.dataButton0 
            = (CapSense_IsWidgetActive
               (CapSense_BUTTON0_WDGT_ID))? true : false;                          
        touchData.dataButton1
            = (CapSense_IsWidgetActive
               (CapSense_BUTTON1_WDGT_ID))? true : false;                                 
        touchData.dataSlider  
            = (uint8_t)CapSense_GetCentroidPos
                       (CapSense_LINEARSLIDER0_WDGT_ID);                      
           /* Start the next CapSense scan */
            CapSense_ScanAllWidgets(); 
            /* Check if button data needs to be sent */
      if ((touchData.dataButton0 != button0Data)||(touchData.dataButton1 != button1Data))
         {
            /* Pack the button data and send to the queue */
            button0Data =touchData.dataButton0;
            button1Data =touchData.dataButton1;               
            rtosApiResult=xQueueSend(rgbLedDataQ, &touchData,0u); 
              if(rtosApiResult != pdTRUE)
                {
                   Task_DebugPrintf("Failure! : Sending Button data to RGB task\r\n",rtosApiResult);
                }
               
            }           
            /* Check if slider data needs to be sent and the touch  
               position on the slider has changed */
      if ((touchData.dataSlider != sliderData))
         {
            /* Pack the slider data and send to the queue */
            sliderData = touchData.dataSlider;             
            rtosApiResult=xQueueSend(rgbLedDataQ, &touchData,0u);
              if(rtosApiResult != pdTRUE)
                {
                    Task_DebugPrintf("Failure! : Sending Slider data to RGB task\r\n",rtosApiResult);   
                }
         
        }          
        /* Wait for the completion of CapSense Widgets Scan. */        
      if(ulTaskNotifyTake(pdTRUE,portMAX_DELAY)==pdTRUE)
            {
             /* The transmission ended as expected. */
            }
         else
            {
            /* The call to ulTaskNotifyTake() timed out. */
            }
    }
}

/*******************************************************************************
* Function Name: void Task_RGB (void *pvParameters)
********************************************************************************
* Summary:
*  Task that controls the color and intensity of the RGB LED   
*
* Parameters:
*  void *pvParameters : Task parameter defined during task creation (unused)                            
*
* Return:
*  void
*
*******************************************************************************/
void Task_RGB (void *pvParameters)
{ 
    /* Variable used to store the return values of RTOS APIs */
    BaseType_t rtosApiResult;
    /* Variable used to store the received values from CapSense widgets */
    touch_data_t RGBtouchData;
    /* Remove warning for unused parameter */
    (void)pvParameters ;
    Task_DebugPrintf(" 2. Task-Task_RGB   : Waiting for the New Scanned Data from CapSense Widgets\r\n",0u);
      
    /* Repeatedly running part of the task */
    for(;;)
    {
        /* Block until RGB data has been received over rgbLedDataQ */
        rtosApiResult = xQueueReceive(rgbLedDataQ, &RGBtouchData,
                      portMAX_DELAY);
       // DEBUG_PRINTF("\r\n Button0=%d,Button1=%d,SliderData=%d\r\n ",RGBtouchData.dataButton0,RGBtouchData.dataButton1,RGBtouchData.dataSlider);   
        /* RGB data has been received from rgbLedDataQ */
        if(rtosApiResult == pdTRUE)
        {
            if(RGBtouchData.dataButton0)
            {if(counter<5) Cy_GPIO_Write(LED_G_0_PORT,LED_G_0_NUM,!RGBtouchData.dataButton0);
             else Cy_GPIO_Write(LED_B_0_PORT,LED_B_0_NUM,!RGBtouchData.dataButton0);
            counter++;
            }
            else{Cy_GPIO_Write(LED_G_0_PORT,LED_G_0_NUM,!RGBtouchData.dataButton0);
            Cy_GPIO_Write(LED_B_0_PORT,LED_B_0_NUM,!RGBtouchData.dataButton0);}
               
        }
        /* Task has timed out and received no RGB data during an interval of 
           portMAXDELAY ticks */
        else
        {
            Task_DebugPrintf("Warning! : RGB - Task Timed out ",rtosApiResult);      
        }
    }  
}   

/*****************************************************************************\
 * Function:    genericEventHandler
 * Input:       Cy_BLE Event Handler event and eventParameter
 * Returns:     void
 * Description: 
 *   This funtion is the BLE Event Handler function.  It is called by the BLE
 *   stack when an event occurs 
\*****************************************************************************/
void genericEventHandler(uint32_t event, void *eventParameter)
{
    (void)eventParameter; // not used
    switch (event)
    {
        case CY_BLE_EVT_STACK_ON:
            printf("Stack Started\r\n");
            Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST, CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
        break;

        case CY_BLE_EVT_GATT_CONNECT_IND:
            printf("Connected\r\n");
            //Cy_GPIO_Write(led9_PORT,led9_NUM,LED_ON); // Turn the LED9 On             
        break;

        case CY_BLE_EVT_GAP_DEVICE_DISCONNECTED:
            printf("Disconnected\r\n");
            //Cy_GPIO_Write(led9_PORT,led9_NUM,LED_OFF); // Turn the LED9 Off
            Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST, CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
        break;

                
        default:
        break;
    }
}

/*****************************************************************************\
 * Function:    iasEventHandler
 * Input:       BLE IAS Service Handler Function: 
 *      - eventCode (which only can be CY_BLE_EVT_IASS_WRITE_CHAR_CMD
 *      - eventParam which is a pointer to  (and unused)
 * Returns:     void
 * Description: 
 *   This is called back by the BLE stack when there is a write to the IAS
 *   service.  This only occurs when the GATT Client Writes a new value
 *   for the alert.  The function figures out the state of the alert then
 *   sends a message to the alertTask usign the EventGroup alterState
\*****************************************************************************/
void iasEventHandler(uint32_t eventCode, void *eventParam)
{
    (void)eventParam;
    uint8_t alertLevel;
    
    if(eventCode == CY_BLE_EVT_IASS_WRITE_CHAR_CMD)
    {
        /* Read the updated Alert Level value from the GATT database */
        Cy_BLE_IASS_GetCharacteristicValue(CY_BLE_IAS_ALERT_LEVEL, 
            sizeof(alertLevel), &alertLevel);
        
        switch(alertLevel)
        {
            case CY_BLE_NO_ALERT:
                Task_DebugPrintf(" BLE TASK   : Counter --\r\n",0u);
                counter --;
                vTaskDelay(1);
                alertLevel=100;
                //counter --;
                /*Cy_GPIO_Write(red_PORT,red_NUM,LED_OFF);
                Cy_GPIO_Write(green_PORT,green_NUM,LED_ON);
                Cy_GPIO_Write(blue_PORT,blue_NUM,LED_OFF);*/
            break;
            case CY_BLE_MILD_ALERT:
                printf("Medium alert\r\n");
               /* Cy_GPIO_Write(red_PORT,red_NUM,LED_OFF);
                Cy_GPIO_Write(green_PORT,green_NUM,LED_OFF);
                Cy_GPIO_Write(blue_PORT,blue_NUM,LED_ON);*/
                
            break;
            case CY_BLE_HIGH_ALERT:        
                printf("High alert\r\n");
                /*Cy_GPIO_Write(red_PORT,red_NUM,LED_ON);
                Cy_GPIO_Write(green_PORT,green_NUM,LED_OFF);
                Cy_GPIO_Write(blue_PORT,blue_NUM,LED_OFF);   */            
            break;
        }   
    }   
}

/*****************************************************************************\
 * Function:    bleInterruptNotify
 * Input:       void (it is called inside of the ISR)
 * Returns:     void
 * Description: 
 *   This is called back in the BLE ISR when an event has occured and needs to
 *   be processed.  It will then set/give the sempahore to tell the BLE task to
 *   process events.
\*****************************************************************************/
void bleInterruptNotify()
{
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(bleSemaphore, &xHigherPriorityTaskWoken); 
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

/*****************************************************************************\
 * Function:    bleTask
 * Input:       A FreeRTOS Task - void * that is unused
 * Returns:     void
 * Description: 
 *  This task starts the BLE stack... and processes events when the bleSempahore
 *  is set by the ISR.
\*****************************************************************************/

void bleTask(void *arg)
{
    (void)arg;
    
    printf("BLE Task Started\r\n");

    bleSemaphore = xSemaphoreCreateCounting(UINT_MAX,0);
    
    Cy_BLE_Start(genericEventHandler);
    
    
    while(Cy_BLE_GetState() != CY_BLE_STATE_ON) // Get the stack going
    {
        Cy_BLE_ProcessEvents();
    }
    
    Cy_BLE_RegisterAppHostCallback(bleInterruptNotify);
    Cy_BLE_IAS_RegisterAttrCallback (iasEventHandler);
    for(;;)
    {
        xSemaphoreTake(bleSemaphore,portMAX_DELAY);
        Cy_BLE_ProcessEvents();
    }
}
/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  System entrance point. This function enables the Cortex-M4
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
    __enable_irq(); /* Enable global interrupts. */

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
   

    /* Start the PWM hardware block */
    PWM_Red_Start();
    
    /* Create the queues. See the respective data-types for details of queue
       contents */
    rgbLedDataQ = xQueueCreate(RGB_LED_QUEUE_LEN,
                                        sizeof(uint32_t));
    /* Create the user Tasks. See the respective Task definition for more
       details of these tasks */       
    xTaskCreate(Task_RGB, "RGB Task", TASK_RGB_STACK_SIZE,
                NULL, TASK_RGB_PRIORITY, NULL);
    xTaskCreate(Task_Touch, "Touch Task", TASK_TOUCH_STACK_SIZE,
                NULL, TASK_TOUCH_PRIORITY, &touchTaskHandle);
    xTaskCreate(bleTask,"bleTask",8*1024,0,2,0);
    /* Initialize thread-safe debug message printing. See uart_debug.h header 
       file to enable / disable this feature */
    Task_DebugInit();   
    DebugPrintf("\r\n\nPSoC 6 MCU FreeRTOS Based CapSense Design\r\n");
    DebugPrintf("\r\n\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\r\n");
    DebugPrintf("\r\n\n");
    
    /* Start the RTOS scheduler. This function should never return */
    vTaskStartScheduler();
     
    /* Should never get here! */ 
    DebugPrintf("Error!   : RTOS - scheduler crashed \r\n");
    
    /* Halt the CPU if scheduler exits */
    CY_ASSERT(0);
    for(;;)
    {
        /* Place your application code here. */
    }
}
/*******************************************************************************
* Function Name: void vApplicationIdleHook(void)
********************************************************************************
*
* Summary:
*  This function is called when the RTOS in idle mode
*    
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void vApplicationIdleHook(void)
{
    /* Enter sleep-mode */
    Cy_SysPm_Sleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
}

/*******************************************************************************
* Function Name: void vApplicationStackOverflowHook(TaskHandle_t *pxTask, 
                                                    signed char *pcTaskName)
********************************************************************************
*
* Summary:
*  This function is called when a stack overflow has been detected by the RTOS
*    
* Parameters:
*  TaskHandle_t  : Handle to the task
*  signed char   : Name of the task
*
* Return:
*  None
*
*******************************************************************************/
void vApplicationStackOverflowHook(TaskHandle_t *pxTask, 
                                   signed char *pcTaskName)
{
    /* Remove warning for unused parameters */
    (void)pxTask;
    (void)pcTaskName;
    
    /* Print the error message with task name if debug is enabled in 
       uart_debug.h file */
    DebugPrintf("Error!   : RTOS - stack overflow in %s \r\n", pcTaskName);
    
    /* Halt the CPU */
    CY_ASSERT(0);
}

/*******************************************************************************
* Function Name: void vApplicationMallocFailedHook(void)
********************************************************************************
*
* Summary:
*  This function is called when a memory allocation operation by the RTOS
*  has failed
*    
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void vApplicationMallocFailedHook(void)
{
    /* Print the error message if debug is enabled in uart_debug.h file */
    DebugPrintf("Error!   : RTOS - Memory allocation failed \r\n");
    
    /* Halt the CPU */
    CY_ASSERT(0);
}
/* [] END OF FILE */
