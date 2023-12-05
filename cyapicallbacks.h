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
#ifndef CYAPICALLBACKS_H
#define CYAPICALLBACKS_H
#include "syslib/cy_syslib.h"

#if CY_CPU_CORTEX_M0P
    /*Define your Cortex-M0P macro callbacks here */
    
#endif

#if CY_CPU_CORTEX_M4
    
/*Define your Cortex-M4 macro callbacks here */
 /* CapSense_ENTRY_CALLBACK */
#define CapSense_ENTRY_CALLBACK  
    void CapSense_EntryCallback();
    
 /* CapSense_EXIT_CALLBACK */
#define CapSense_EXIT_CALLBACK   
    void CapSense_ExitCallback();
    
#endif

    /*For more information, refer to the Writing Code topic in the PSoC Creator Help.*/

    
#endif /* CYAPICALLBACKS_H */   
/* [] */
