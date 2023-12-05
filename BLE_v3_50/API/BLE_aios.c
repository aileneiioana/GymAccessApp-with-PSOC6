/***************************************************************************//**
* \file CYBLE_aios.c
* \version `$CY_MAJOR_VERSION`.`$CY_MINOR_VERSION`
* 
* \brief
*  Contains the source code for Automation Input Output Service.
* 
********************************************************************************
* \copyright
* Copyright 2016-2018, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "`$INSTANCE_NAME`_eventHandler.h"
#include "`$INSTANCE_NAME`_aios.h"
 

#ifdef CYBLE_AIOS_SERVER
/* Generated code */
`$AiosServer`
/* End of generated code */
    
static CYBLE_GATT_DB_ATTR_HANDLE_T cyBle_aiossReqHandle;

#endif /* CYBLE_AIOS_SERVER */

#ifdef CYBLE_AIOS_CLIENT
/* Generated code */
`$AiosClient`
/* End of generated code */

static CYBLE_GATT_DB_ATTR_HANDLE_T cyBle_aioscReqHandle;

uint8 cyBle_aioscActiveCharIndex = 0u;
uint8 cyBle_aioscActiveCharInstance = 0u;
uint8 cyBle_aioscPrevCharInstIndex;

#endif /* (CYBLE_AIOS_CLIENT) */

static CYBLE_CALLBACK_T CyBle_AiosApplCallback = NULL;

/******************************************************************************
* Function Name: CyBle_AiosInit
***************************************************************************//**
* 
*  This function initializes the Automation IO Service.
* 
******************************************************************************/
void CyBle_AiosInit(void)
{
#ifdef CYBLE_AIOS_CLIENT
   
    uint8 i;
    uint8 j;

    cyBle_aioscActiveCharIndex = 0u;
    cyBle_aioscActiveCharInstance = 0u;
    cyBle_aioscPrevCharInstIndex = 0u;

    if(cyBle_serverInfo[CYBLE_SRVI_AIOS].range.startHandle == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
        for(i = (uint8) CYBLE_AIOS_DIGITAL; (i < ((uint8) CYBLE_AIOS_CHAR_COUNT)); i++)
        {
            for(j = 0u; j < cyBle_aioscCharInstances[i]; j++)
            {
                (void) memset(&cyBle_aiosc.charInfoAddr[i].charInfoPtr[j], 0, sizeof(CYBLE_AIOSC_CHAR_T));
            }
        }
    }
    cyBle_aioscReqHandle = CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE;
    
#endif /* CYBLE_AIOS_CLIENT */

#ifdef CYBLE_AIOS_SERVER
    (void)CyBle_AiosSetAggregateChar();
#endif /* CYBLE_AIOS_SERVER */

}

/******************************************************************************
* Function Name: CyBle_AiosRegisterAttrCallback
***************************************************************************//**
* 
*  Registers a callback function for service specific attribute operations.
*  Service specific write requests from peer device will not be handled with
*  unregistered callback function.
* 
*  \param callbackFunc:  An application layer event callback function to receive
*                 events from the BLE Component. The definition of
*                 CYBLE_CALLBACK_T for AIOS Service is:\n
*                 typedef void (* CYBLE_CALLBACK_T) (uint32 eventCode,
*                                                    void *eventParam)
*                 * eventCode:  Indicates the event that triggered this callback
*                   (e.g. CYBLE_EVT_AIOS_NOTIFICATION_ENABLED).
*                 * eventParam: Contains the parameters corresponding to the
*                   current event. (e.g. Pointer to CYBLE_AIOS_CHAR_VALUE_T
*                   structure that contains details of the characteristic
*                   for which the notification enabled event was triggered).
* 
******************************************************************************/
void CyBle_AiosRegisterAttrCallback(CYBLE_CALLBACK_T callbackFunc)
{
    CyBle_AiosApplCallback = callbackFunc;
    
}

#ifdef CYBLE_AIOS_SERVER

/******************************************************************************
* Function Name: CyBle_AiossWriteEventHandler
***************************************************************************//**
* 
*  Handles the Write Request Event.
* 
*  \param eventParam: The pointer to the data structure specified by the event.
* 
* \return
*  CYBLE_GATT_ERR_CODE_T - A function return state if it succeeded
*  (CYBLE_GATT_ERR_NONE) or failed with error codes:
*   * CYBLE_GATTS_ERR_PROCEDURE_ALREADY_IN_PROGRESS - profile or service
*                           request cannot be serviced because an operation t
*                           hat has been previously triggered is still in
*                           progress.
*   * CYBLE_GATTS_ERR_CCCD_IMPROPERLY_CONFIGURED - Client Characteristic 
                            Configuration descriptor is not configured according
*                           to the requirements of the profile or service.       
* 
******************************************************************************/
CYBLE_GATT_ERR_CODE_T CyBle_AiossWriteEventHandler(CYBLE_GATTS_WRITE_REQ_PARAM_T *eventParam)
{
    uint8 i;
    uint8 j;
    CYBLE_GATT_ERR_CODE_T gattErr = CYBLE_GATT_ERR_NONE;
    CYBLE_GATT_DB_ATTR_HANDLE_T tmpHandle;
    uint8 foundItem = CYBLE_AIOS_UNDEFINED_ITEM;
    CYBLE_AIOSS_CHAR_T *aiosCharInfoPtr;
    CYBLE_AIOS_DESCR_VALUE_T wrDescrReqParam;
    uint32 event = 0u;
    CYBLE_AIOS_CHAR_VALUE_T wrCharReqParam;
    uint8 descInd;
    
    tmpHandle = eventParam->handleValPair.attrHandle;
    
    if(CyBle_AiosApplCallback != NULL)
    {
        /* Go through all possible Service characteristics.
        * Exit the handler in following conditions:
        * 1) No more characteristic left;
        * 2) Characteristic or descriptor was successfully written;
        * 3) Error occurred while writing characteristic or descriptor.
        */
        for(i = 0u; ((i < ((uint8) CYBLE_AIOS_CHAR_COUNT)) && (foundItem == 0u) && (gattErr == CYBLE_GATT_ERR_NONE));
            i++)
        {
            /* Check if characteristic is enabled. If the pointer to the characteristic
            * is not "NULL", there is at least one instance of the characteristic in
            * the AIO Service.
            */
            
            if(cyBle_aioss.charInfoAddr[i].charInfoPtr != NULL)
            {   
                for(j = 0u; j < cyBle_aiossCharInstances[i]; j++)
                {
                    aiosCharInfoPtr = &cyBle_aioss.charInfoAddr[i].charInfoPtr[j];
                    
                    /* Client Characteristic Write Request handling */
                    if(tmpHandle == aiosCharInfoPtr->charHandle)
                    {
                        wrDescrReqParam.descrIndex = CYBLE_AIOS_DESCR_COUNT;
                        foundItem = CYBLE_AIOS_CHARACTERISTIC_ITEM;
                    }                
                    
                    /* Client Characteristic Descriptor Write Request handling */
                    else
                    {
                        /* No handle match was found */
                        wrDescrReqParam.descrIndex = CYBLE_AIOS_DESCR_COUNT;
                        
                        for(descInd = 0u; (descInd < (uint8)CYBLE_AIOS_DESCR_COUNT) && (foundItem != CYBLE_AIOS_DESCRIPTOR_ITEM); descInd++)
                        {
	                        if(aiosCharInfoPtr->descrHandle[descInd] == tmpHandle)
	                        {
		                        wrDescrReqParam.descrIndex = (CYBLE_AIOS_DESCR_INDEX_T)descInd;
		                        foundItem = CYBLE_AIOS_DESCRIPTOR_ITEM;
	                        }
                        }
                    }
                       
                    /* Verify if requested handle was found and successfully handled */
                    if((gattErr == CYBLE_GATT_ERR_NONE) && (foundItem == CYBLE_AIOS_DESCRIPTOR_ITEM))
                    {
                        switch(wrDescrReqParam.descrIndex)
                        {
                        case CYBLE_AIOS_CCCD :
                             /* Verify that optional notification property is enabled for characteristic */
                            if(CYBLE_IS_NOTIFICATION_SUPPORTED(aiosCharInfoPtr->charHandle))
                            {
                                if(CYBLE_IS_NOTIFICATION_ENABLED_IN_PTR(eventParam->handleValPair.value.val))
                                {
                                    event = (uint32)CYBLE_EVT_AIOSS_NOTIFICATION_ENABLED;
                                }
                                else
                                {
                                    event = (uint32)CYBLE_EVT_AIOSS_NOTIFICATION_DISABLED;
                                }

                            #if((CYBLE_GAP_ROLE_PERIPHERAL || CYBLE_GAP_ROLE_CENTRAL) && \
                                (CYBLE_BONDING_REQUIREMENT == CYBLE_BONDING_YES))
                                /* Set flag to store bonding data to flash */
                                if(cyBle_peerBonding == CYBLE_GAP_BONDING)
                                {
                                    cyBle_pendingFlashWrite |= CYBLE_PENDING_CCCD_FLASH_WRITE_BIT;
                                }
                            #endif /* (CYBLE_BONDING_REQUIREMENT == CYBLE_BONDING_YES) */
                            }
                            else if(CYBLE_IS_INDICATION_SUPPORTED(aiosCharInfoPtr->charHandle))
                            {
                                if(CYBLE_IS_INDICATION_ENABLED_IN_PTR(eventParam->handleValPair.value.val))
                                {
                                    event = (uint32)CYBLE_EVT_AIOSS_INDICATION_ENABLED;
                                }
                                else
                                {
                                    event = (uint32)CYBLE_EVT_AIOSS_INDICATION_DISABLED;
                                }

                            #if((CYBLE_GAP_ROLE_PERIPHERAL || CYBLE_GAP_ROLE_CENTRAL) && \
                                (CYBLE_BONDING_REQUIREMENT == CYBLE_BONDING_YES))
                                /* Set flag to store bonding data to flash */
                                if(cyBle_peerBonding == CYBLE_GAP_BONDING)
                                {
                                    cyBle_pendingFlashWrite |= CYBLE_PENDING_CCCD_FLASH_WRITE_BIT;
                                }
                            #endif /* (CYBLE_BONDING_REQUIREMENT == CYBLE_BONDING_YES) */
                            }
                            else
                            {
                                gattErr = CYBLE_GATT_ERR_REQUEST_NOT_SUPPORTED;
                            }
                            break;
                            
                        case CYBLE_AIOS_CHAR_USER_DESCRIPTION_DESCR:
                            if(CYBLE_GATT_DB_ATTR_GET_ATTR_GEN_MAX_LEN(
                                aiosCharInfoPtr->descrHandle[CYBLE_AIOS_CHAR_USER_DESCRIPTION_DESCR]) >=
                                    eventParam->handleValPair.value.len)
                            {
                                /* Set new length of the descriptor value */
                                CYBLE_GATT_DB_ATTR_SET_ATTR_GEN_LEN(
                                    aiosCharInfoPtr->descrHandle[CYBLE_AIOS_CHAR_USER_DESCRIPTION_DESCR],
                                        eventParam->handleValPair.value.len);
                            }
                            else
                            {
                                gattErr = CYBLE_GATT_ERR_INVALID_ATTRIBUTE_LEN;
                            }
                            break;
                        
                        case CYBLE_AIOS_VALUE_TRIGGER_SETTINGS:  
                            /* Trigger condition value not supported.*/
                            if( eventParam->handleValPair.value.val[0u] >= CYBLE_AIOS_VALUE_TRIGGER_DESCR_MAX_VALUE) 
                            {
                                gattErr = CYBLE_GATT_ERR_TRIGGER_CODITION_VALUE_NOT_SUPPORTED;
                            }
                            break;

                        case CYBLE_AIOS_TIME_TRIGGER_SETTINGS:     
                            /* Trigger condition value not supported.*/
                            if( eventParam->handleValPair.value.val[0u] >= CYBLE_AIOS_TIME_TRIGGER_DESCR_MAX_VALUE) 
                            {
                                gattErr = CYBLE_GATT_ERR_TRIGGER_CODITION_VALUE_NOT_SUPPORTED;
                            }
                            break;
                            
                        default:
                            break;
                        }    
                    }                    
                    
                    /* Check is there no error conditions happen yet */
                    if((gattErr == CYBLE_GATT_ERR_NONE)
                    && ( (foundItem == CYBLE_AIOS_CHARACTERISTIC_ITEM) || (foundItem == CYBLE_AIOS_DESCRIPTOR_ITEM) ) )
                    {
                        gattErr = CyBle_GattDbCheckPermission(eventParam->handleValPair.attrHandle, 
                                                              &eventParam->connHandle,
                                                              CYBLE_GATT_DB_WRITE | CYBLE_GATT_DB_PEER_INITIATED);
                        
                        if(gattErr == CYBLE_GATT_ERR_NONE)
                        {
                            /* Fill data and pass it to user */
                                                          
                            if( foundItem == CYBLE_AIOS_CHARACTERISTIC_ITEM)
                            {
                                wrCharReqParam.connHandle = eventParam->connHandle;
                                wrCharReqParam.charIndex = (CYBLE_AIOS_CHAR_INDEX_T) i;
                                wrCharReqParam.charInstance = j;
                                wrCharReqParam.gattErrorCode = CYBLE_GATT_ERR_NONE;
                                wrCharReqParam.value = &eventParam->handleValPair.value;
                            
                                CyBle_AiosApplCallback(CYBLE_EVT_AIOSS_CHAR_WRITE, &wrCharReqParam);
                            }
                            else if (foundItem == CYBLE_AIOS_DESCRIPTOR_ITEM)
                            {
                                wrDescrReqParam.connHandle = eventParam->connHandle;
                                wrDescrReqParam.charIndex = (CYBLE_AIOS_CHAR_INDEX_T) i;
                                wrDescrReqParam.charInstance = j;
                                wrDescrReqParam.gattErrorCode = CYBLE_GATT_ERR_NONE;
                                wrDescrReqParam.value = &eventParam->handleValPair.value;
                            
                                CyBle_AiosApplCallback(CYBLE_EVT_AIOSS_DESCR_WRITE, &wrDescrReqParam);
                            }
                            else 
                            {
                                /* No handle match was found */
                            }

                            /* Write value to GATT database */
                            gattErr = CyBle_GattsWriteAttributeValue(&eventParam->handleValPair, 0u,
                                &eventParam->connHandle, CYBLE_GATT_DB_PEER_INITIATED);
                            
                            if( event != 0u )
                            {   
                                CyBle_AiosApplCallback(event, &wrDescrReqParam);
                            }
                            else 
                            {
                                /* No event was found */
                            }
                        }
                        cyBle_eventHandlerFlag &= (uint8)~CYBLE_CALLBACK;
                        break;
                    }
                }
            }
        }
    }

    if(gattErr != CYBLE_GATT_ERR_NONE )
    {
        /* Indicate that request was handled */
        cyBle_eventHandlerFlag &= (uint8)~CYBLE_CALLBACK;
    }
    else
    {   
        if( foundItem == CYBLE_AIOS_CHARACTERISTIC_ITEM )
        {
            gattErr = CyBle_AiosSetAggregateChar();
        }
    }
    
    return (gattErr);
}

/******************************************************************************
* Function Name: CyBle_AiossSetCharacteristicValue
***************************************************************************//**
* 
*  Sets the characteristic value of the service in the local database.
* 
*  \param charIndex:    The index of the service characteristic. Starts with 
*                       zero.
*  \param charInstance: The instance number of the characteristic specified by
*                       "charIndex".
*  \param attrSize:     The size (in bytes) of the characteristic value 
*                       attribute.
*  \param attrValue:    The pointer to the characteristic value data that 
*                       should be stored in the GATT database.
* 
* \return
*  A return value is of type CYBLE_API_RESULT_T.
*  * CYBLE_ERROR_OK - The request handled successfully.
*  * CYBLE_ERROR_INVALID_PARAMETER - Validation of the input parameter failed.
*  * CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE - An optional characteristic is
*                                              absent.
******************************************************************************/
CYBLE_API_RESULT_T CyBle_AiossSetCharacteristicValue(CYBLE_AIOS_CHAR_INDEX_T charIndex,
    uint8 charInstance, uint8 attrSize, uint8 *attrValue)
{
    CYBLE_API_RESULT_T apiResult = CYBLE_ERROR_INVALID_PARAMETER;

    /* Validate range of AIOS characteristics */
    if((charIndex < CYBLE_AIOS_CHAR_COUNT) && (charInstance < cyBle_aiossCharInstances[charIndex]))
    {
        /* Check if requested characteristic is present in Service.
        * There are three conditions which should be checked: 1) The pointer to
        * "CYBLE_AIOS_CHAR_T" is not NULL, 2) The handle of the characteristic is
        * a valid handle, 3) The requested instance is a valid characteristic
        * instance.
        */
        if((cyBle_aioss.charInfoAddr[charIndex].charInfoPtr != NULL) &&
            (CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE !=
                cyBle_aioss.charInfoAddr[charIndex].charInfoPtr[charInstance].charHandle))
        {
            CYBLE_GATT_HANDLE_VALUE_PAIR_T locHandleValuePair;

            locHandleValuePair.attrHandle =
                cyBle_aioss.charInfoAddr[charIndex].charInfoPtr[charInstance].charHandle;
            locHandleValuePair.value.len = attrSize;
            locHandleValuePair.value.val = attrValue;

            /* Store data in database */
            if(CyBle_GattsWriteAttributeValue(&locHandleValuePair, 0u, NULL, CYBLE_GATT_DB_LOCALLY_INITIATED) ==
                CYBLE_GATT_ERR_NONE)
            {
                apiResult = CYBLE_ERROR_OK;
            }
        }
        else
        {
            apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
        }
    }
    
    if ( apiResult == CYBLE_ERROR_OK )
    {   
        if(CyBle_AiosSetAggregateChar() != CYBLE_GATT_ERR_NONE)
        {   
            apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
        }
    }
    
    return (apiResult);
}

/******************************************************************************
* Function Name: CyBle_AiossGetCharacteristicValue
***************************************************************************//**
* 
*  Gets the characteristic value of the service, which is a value identified by
*  charIndex.
* 
*  \param charIndex:    The index of the service characteristic. Starts with 
*                       zero.
*  \param charInstance: The instance number of the characteristic specified by
*                       "charIndex".
*  \param attrSize:     The size of the characteristic value attribute.
*  \param attrValue:    The pointer to the location where characteristic value
*                       data should be stored.
* 
* \return
*  A return value is of type CYBLE_API_RESULT_T.
*  * CYBLE_ERROR_OK - The request handled successfully
*  * CYBLE_ERROR_INVALID_PARAMETER - Validation of the input parameter failed
*  * CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE - An optional characteristic is
*                                              absent.
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_AiossGetCharacteristicValue(CYBLE_AIOS_CHAR_INDEX_T charIndex, 
    uint8 charInstance, uint8 attrSize, uint8 *attrValue)
{
     CYBLE_API_RESULT_T apiResult = CYBLE_ERROR_INVALID_PARAMETER;

    if((charIndex < CYBLE_AIOS_CHAR_COUNT) && (charInstance < cyBle_aiossCharInstances[charIndex]))
    {
        /* Check if requested characteristic is present in Service. There are three
        * conditions to be checked: 1) The pointer to "CYBLE_AIOSS_CHAR_T" record in
        * the cyBle_aioss struct is not NULL, 2) The handle of the characteristic is a 
        * valid handle, 3) The requested instance is a valid characteristic instance.
        */
        if((cyBle_aioss.charInfoAddr[charIndex].charInfoPtr != NULL) &&
            (cyBle_aioss.charInfoAddr[charIndex].charInfoPtr[charInstance].charHandle != CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE))
        {
            CYBLE_GATT_HANDLE_VALUE_PAIR_T locHandleValuePair;

            locHandleValuePair.attrHandle = cyBle_aioss.charInfoAddr[charIndex].charInfoPtr[charInstance].charHandle;
            locHandleValuePair.value.len = attrSize;
            locHandleValuePair.value.val = attrValue;

            /* Read characteristic value from database */
            if(CyBle_GattsReadAttributeValue(&locHandleValuePair, NULL, CYBLE_GATT_DB_LOCALLY_INITIATED) ==
                CYBLE_GATT_ERR_NONE)
            {
                apiResult = CYBLE_ERROR_OK;
                
            /* Set new length of the descriptor value */
            CYBLE_GATT_DB_ATTR_SET_ATTR_GEN_LEN(cyBle_aioss.charInfoAddr[charIndex].charInfoPtr[charInstance].charHandle,
                                                attrSize);
            }
        }
        else
        {
            apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
        }
    }

    return (apiResult);
    
}

/******************************************************************************
* Function Name: CyBle_AiossGetCharacteristicDescriptor
***************************************************************************//**
* 
*  Gets a characteristic descriptor of a specified characteristic of the
*  Automation Input Outtput Service from the local GATT database.
* 
*  \param charIndex:    The index of the characteristic.
*  \param charInstance: The instance number of the characteristic specified by
*                       "charIndex".
*  \param descrIndex:   The index of the characteristic descriptor.
*  \param attrSize:     The size of the characteristic descriptor attribute.
*  \param attrValue:    The pointer to the location where characteristic  
*                       descriptor value data should be stored.
* 
* \return
*  Return value is of type CYBLE_API_RESULT_T.
*  * CYBLE_ERROR_OK - The request handled successfully.
*  * CYBLE_ERROR_INVALID_PARAMETER - Validation of the input parameter failed.
*  * CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE - Optional descriptor is absent.
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_AiossGetCharacteristicDescriptor(CYBLE_AIOS_CHAR_INDEX_T charIndex, 
     uint8 charInstance, CYBLE_AIOS_DESCR_INDEX_T descrIndex, uint8 attrSize, uint8 *attrValue)
{
    
     CYBLE_API_RESULT_T apiResult = CYBLE_ERROR_INVALID_PARAMETER;

    if((charIndex < CYBLE_AIOS_CHAR_COUNT) && (charInstance < cyBle_aiossCharInstances[charIndex]) &&
        (descrIndex < CYBLE_AIOS_DESCR_COUNT ))
    {
        /* Check if requested descriptor is present in Service.
        * There are three conditions to be checked: 1) The pointer to
        * "CYBLE_AiosS_CHAR_T" is not NULL, 2) The handle of the characteristic is
        * a valid handle, 3) The handle of the descriptor is a valid handle.
        */
        if((cyBle_aioss.charInfoAddr[charIndex].charInfoPtr != NULL) &&
            (cyBle_aioss.charInfoAddr[charIndex].charInfoPtr[charInstance].charHandle != CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE) &&
                    (cyBle_aioss.charInfoAddr[charIndex].charInfoPtr[charInstance].descrHandle[descrIndex] != CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE))
        {
            CYBLE_GATT_HANDLE_VALUE_PAIR_T locHandleValuePair;

            locHandleValuePair.attrHandle =
                cyBle_aioss.charInfoAddr[charIndex].charInfoPtr[charInstance].descrHandle[descrIndex];
            locHandleValuePair.value.len = attrSize;
            locHandleValuePair.value.val = attrValue;
                        
            /* Read characteristic value from database */
            if(CyBle_GattsReadAttributeValue(&locHandleValuePair, NULL, CYBLE_GATT_DB_LOCALLY_INITIATED) ==
                CYBLE_GATT_ERR_NONE)
            {
                apiResult = CYBLE_ERROR_OK;
                
                /* Set new length of the descriptor value */
                CYBLE_GATT_DB_ATTR_SET_ATTR_GEN_LEN(cyBle_aioss.charInfoAddr[charIndex].charInfoPtr[charInstance].
                                                    descrHandle[descrIndex],
                                                    attrSize);
            }
        }
        else
        {
            apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
        }
    }
    return (apiResult);
}

/******************************************************************************
* Function Name: CyBle_AiossSetCharacteristicDescriptor
***************************************************************************//**
* 
*  Set a characteristic descriptor of a specified characteristic of the Indoor 
*  Positioning Service from the local GATT database.
* 
*  \param charIndex:    The index of the characteristic.
*  \param charInstance: The instance number of the characteristic specified by
*                       "charIndex".
*  \param descrIndex:   The index of the characteristic descriptor.
*  \param attrSize:     The size of the characteristic descriptor attribute.
*  \param attrValue:    The pointer to the descriptor value data to 
*                       be stored in the GATT database.
* 
* \return
*  Return value is of type CYBLE_API_RESULT_T.
*  * CYBLE_ERROR_OK - The request handled successfully.
*  * CYBLE_ERROR_INVALID_PARAMETER - Validation of the input parameter failed.
*  * CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE - Optional descriptor is absent.
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_AiossSetCharacteristicDescriptor(CYBLE_AIOS_CHAR_INDEX_T charIndex, 
     uint8 charInstance, CYBLE_AIOS_DESCR_INDEX_T descrIndex, uint8 attrSize, uint8 *attrValue)
{
      CYBLE_API_RESULT_T apiResult = CYBLE_ERROR_INVALID_PARAMETER;

    if((charIndex < CYBLE_AIOS_CHAR_COUNT) && (charInstance < cyBle_aiossCharInstances[charIndex]) &&
        (descrIndex < CYBLE_AIOS_DESCR_COUNT))
    {
        /* Check if requested descriptor is present in Service.
        * There are three conditions to be checked: 1) The pointer to
        * "CYBLE_AiosS_CHAR_T" is not NULL, 2) The handle of the characteristic is
        * a valid handle, 3) The handle of the descriptor is a valid handle.
        */
        if((cyBle_aioss.charInfoAddr[charIndex].charInfoPtr != NULL) &&
            (cyBle_aioss.charInfoAddr[charIndex].charInfoPtr[charInstance].charHandle != CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE) &&
                        (cyBle_aioss.charInfoAddr[charIndex].charInfoPtr[charInstance].descrHandle[descrIndex] != CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE))
        {
            CYBLE_GATT_HANDLE_VALUE_PAIR_T locHandleValuePair;

            locHandleValuePair.attrHandle =
                cyBle_aioss.charInfoAddr[charIndex].charInfoPtr[charInstance].descrHandle[descrIndex];
            locHandleValuePair.value.len = attrSize;
            locHandleValuePair.value.val = attrValue;

            if(CyBle_GattsWriteAttributeValue(&locHandleValuePair, 0u, NULL, CYBLE_GATT_DB_LOCALLY_INITIATED) ==
                CYBLE_GATT_ERR_NONE)
            {
                apiResult = CYBLE_ERROR_OK;
            }
        }
        else
        {
            apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
        }
    }
    return (apiResult);
}

/******************************************************************************
* Function Name: CyBle_AiossSendNotification
***************************************************************************//**
* 
*  Sends a notification with a characteristic value of the Automation Input Output
*  Service, which is a value specified by charIndex, to the client's device.
*
*  On enabling notification successfully for a service characteristic it sends 
*  out a 'Handle Value Notification' which results in 
*  CYBLE_EVT_AIOSC_NOTIFICATION event at the GATT Client's end.
* 
*  \param connHandle:   The connection handle.
*  \param charIndex:    The index of the service characteristic. Starts with 
*                       zero.
*  \param charInstance: The instance number of the characteristic specified by 
*                       "charIndex".
*  \param attrSize:     The size of the characteristic value attribute.
*  \param attrValue:    The pointer to the characteristic value data that 
*                       should be sent to the client's device.
* 
* \return
*  A return value is of type CYBLE_API_RESULT_T.
*   * CYBLE_ERROR_OK - The request handled successfully.
*   * CYBLE_ERROR_INVALID_PARAMETER - Validation of the input parameter failed.
*   * CYBLE_ERROR_INVALID_OPERATION - This operation is not permitted.
*   * CYBLE_ERROR_INVALID_STATE - Connection with the client is not established.
*   * CYBLE_ERROR_MEMORY_ALLOCATION_FAILED - Memory allocation failed.
*   * CYBLE_ERROR_NTF_DISABLED - A notification is not enabled by the client.
*   * CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE - An optional characteristic is
*                                               absent.
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_AiossSendNotification(CYBLE_CONN_HANDLE_T connHandle,
    CYBLE_AIOS_CHAR_INDEX_T charIndex, uint8 charInstance, uint8 attrSize, uint8 *attrValue)
{
    CYBLE_API_RESULT_T apiResult = CYBLE_ERROR_INVALID_PARAMETER;

    if((charIndex < CYBLE_AIOS_CHAR_COUNT) && (charInstance < cyBle_aiossCharInstances[charIndex]))
    {
        if((cyBle_aioss.charInfoAddr[charIndex].charInfoPtr != NULL) && 
            (cyBle_aioss.charInfoAddr[charIndex].charInfoPtr[charInstance].descrHandle[CYBLE_AIOS_CCCD] != CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE))
        {
            /* Send notification if it is enabled and connected */
            if(CyBle_GetState() != CYBLE_STATE_CONNECTED)
            {
                apiResult = CYBLE_ERROR_INVALID_STATE;
            }
            
            else if(!CYBLE_IS_NOTIFICATION_ENABLED(
                cyBle_aioss.charInfoAddr[charIndex].charInfoPtr[charInstance].descrHandle[CYBLE_AIOS_CCCD]))
            {
                apiResult = CYBLE_ERROR_NTF_DISABLED;
            }
            else
            {
                CYBLE_GATTS_HANDLE_VALUE_NTF_T ntfReqParam;

                /* Fill all fields of write request structure ... */
                ntfReqParam.attrHandle = cyBle_aioss.charInfoAddr[charIndex].charInfoPtr[charInstance].charHandle;
                ntfReqParam.value.val = attrValue;
                ntfReqParam.value.len = attrSize;

                /* Send notification to client using previously filled structure */
                apiResult = CyBle_GattsNotification(connHandle, &ntfReqParam);
            }
        }
        else
        {
            apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
        }
    }

    return (apiResult);
}

/******************************************************************************
* Function Name: CyBle_AiossSendIndication
***************************************************************************//**
* 
*  Sends an indication with a characteristic value of the Automation Input Output
*  Service, which is a value specified by charIndex, to the client's device.
*  
*  On enabling indication successfully it sends out a 'Handle Value Indication' 
*  which results in CYBLE_EVT_AIOSC_INDICATION or 
*  CYBLE_EVT_GATTC_HANDLE_VALUE_IND (if service specific callback function is
*  not registered) event at the GATT Client's end.
*
*  \param connHandle:   The connection handle.
*  \param charIndex:    The index of the service characteristic.
*  \param charInstance: The instance number of the characteristic specified by 
*                       "charIndex".
*  \param attrSize:     The size of the characteristic value attribute.
*  \param attrValue:    The pointer to the characteristic value data that 
*                       should be sent to the client's device.
* 
* \return
*  A return value is of type CYBLE_API_RESULT_T.
*   * CYBLE_ERROR_OK - The request handled successfully.
*   * CYBLE_ERROR_INVALID_PARAMETER - Validation of the input parameter failed.
*   * CYBLE_ERROR_INVALID_OPERATION - This operation is not permitted.
*   * CYBLE_ERROR_INVALID_STATE - Connection with the client is not established.
*   * CYBLE_ERROR_MEMORY_ALLOCATION_FAILED - Memory allocation failed.
*   * CYBLE_ERROR_IND_DISABLED - Indication is not enabled by the client.
*   * CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE - An optional characteristic is
*                                               absent.
*
* \events
*  In case of successful execution (return value = CYBLE_ERROR_OK)
*  the next events can appear: \n
*   If the Aios service-specific callback is registered 
*      (with CyBle_AiosRegisterAttrCallback):
*  * CYBLE_EVT_AiosS_INDICATION_CONFIRMED -In case if the indication is
*                                successfully delivered to the peer device.
*  .
*   Otherwise (if the Aios service-specific callback is not registered):
*  * CYBLE_EVT_GATTS_HANDLE_VALUE_CNF - In case if the indication is
*                                successfully delivered to the peer device.
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_AiossSendIndication(CYBLE_CONN_HANDLE_T connHandle,
    CYBLE_AIOS_CHAR_INDEX_T charIndex, uint8 charInstance, uint8 attrSize, uint8 *attrValue)
{
    /* Store new data in database */
    CYBLE_API_RESULT_T apiResult = CYBLE_ERROR_OK;
    
    /* Send indication if it is enabled and connected */
    if(charIndex >= CYBLE_AIOS_CHAR_COUNT)
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
    else if(charInstance >= cyBle_aiossCharInstances[charIndex])
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
    else if(CYBLE_STATE_CONNECTED != CyBle_GetState())
    {
        apiResult = CYBLE_ERROR_INVALID_STATE;
    }
    else if(!CYBLE_IS_INDICATION_ENABLED(
        cyBle_aioss.charInfoAddr[charIndex].charInfoPtr[charInstance].descrHandle[CYBLE_AIOS_CCCD]))
    {
        apiResult = CYBLE_ERROR_IND_DISABLED;
    }
    else
    {
        if(cyBle_aioss.charInfoAddr[charIndex].charInfoPtr[charInstance].charHandle != CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
        {
            CYBLE_GATTS_HANDLE_VALUE_IND_T indReqParam;

            /* Fill all fields of write request structure ... */
            indReqParam.attrHandle = cyBle_aioss.charInfoAddr[charIndex].charInfoPtr[charInstance].charHandle;
            indReqParam.value.val = attrValue;
            indReqParam.value.len = attrSize;

            /* Send indication to client using previously filled structure */
            apiResult = CyBle_GattsIndication(connHandle, &indReqParam);
            /* Save handle to support service specific value confirmation response from client */
            if(apiResult == CYBLE_ERROR_OK)
            {
                cyBle_aiossReqHandle = indReqParam.attrHandle;
            }
        }
        else
        {
            apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
        }
    }
    return (apiResult);
}

/******************************************************************************
* Function Name: CyBle_AiossConfirmationEventHandler
***************************************************************************//**
* 
*  Handles a Value Confirmation request event from the BLE stack.
* 
*  eventParam - The pointer to a structure of type CYBLE_CONN_HANDLE_T.
*  
******************************************************************************/
void CyBle_AiossConfirmationEventHandler(const CYBLE_CONN_HANDLE_T *eventParam)
{
    CYBLE_AIOS_CHAR_INDEX_T locCharIndex;
    CYBLE_AIOS_CHAR_VALUE_T locCharValue;
    uint8 locCharInstance;
    uint8 locReqHandle = 0u;

    if((CyBle_AiosApplCallback != NULL) && (cyBle_aiossReqHandle != CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE))
    {
        for(locCharIndex = CYBLE_AIOS_DIGITAL; (locCharIndex < CYBLE_AIOS_CHAR_COUNT) && (locReqHandle == 0u); 
            locCharIndex++)
        {
            for(locCharInstance = 0u; (locCharInstance < cyBle_aiossCharInstances[locCharIndex]) && (locReqHandle == 0u); 
                locCharInstance++)
            {
                if(cyBle_aiossReqHandle == cyBle_aioss.charInfoAddr[locCharIndex].charInfoPtr[locCharInstance].charHandle)
                {
                    locCharValue.connHandle = *eventParam;
                    locCharValue.charIndex = locCharIndex;
                    locCharValue.charInstance = locCharInstance;
                    locCharValue.value = NULL;
                    cyBle_eventHandlerFlag &= (uint8)~CYBLE_CALLBACK;
                    cyBle_aiossReqHandle = CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE;
                    CyBle_AiosApplCallback((uint32)CYBLE_EVT_AIOSS_INDICATION_CONFIRMED, &locCharValue);
                    locReqHandle = 1u;
                }
            }
        }
    }
}

/******************************************************************************
* Function Name: CyBle_AiosSetAggregateChar
***************************************************************************//**
*
*  Sets the value for Aggregate characteristic. 
*  All of the defined Digital characteristics with the Read property set be
*  part of Aggregate (if available). The length of each of the Digital characteristics 
*  that is part of the Aggregate is determined from the number of signals defined in the 
*  Number of Digitals descriptor of the Digital characteristic. The Digital part byte 
*  aligned and trailing bits (if any) set to 0. The order of the Digital characteristics 
*  is determined by the value in the Description field of the Characteristic Presentation 
*  Format descriptor with the lowest value first. 
*  All of the defined Analog characteristics with Read property set part of the Aggregate
*  (if available). The order of the Analog characteristics is determined by the value in 
*  the Description field of the Characteristic Presentation Format descriptor with the 
*  lowest value first. 
*  The first analog input follows directly after the last Digital characteristic.
* 
* \return
*  A return value is of type CYBLE_GATT_ERR_CODE_T.
*  * CYBLE_GATT_ERR_NONE - The advertisement packet collected successfully.
*  * CYBLE_GATT_ERR_INVALID_HANDLE - 'handleValuePair.attrHandle' is not valid.
*
******************************************************************************/
CYBLE_GATT_ERR_CODE_T CyBle_AiosSetAggregateChar(void)
{
    CYBLE_GATT_ERR_CODE_T gattErr = CYBLE_GATT_ERR_NONE;
    CYBLE_GATT_HANDLE_VALUE_PAIR_T locHandleValuePair;
    CYBLE_AIOS_CHAR_INDEX_T locCharIndex;
    uint8 locCharInstance;
    uint8 locAttrValue[CYBLE_AIOS_CHAR_PRESENTATION_FORMAT_LEN];
    uint8 aggregateChar[CYBLE_GATT_MTU-3];
    uint16 currentDescriptionValue;
    uint16 minDescriptionValue;
    uint16 minDescriptionValueTmp = 0xFFFFu;
    uint16 maxDescriptionValue;
    uint16 minDescriptionNumber;
    uint8 locCharInstanceNumber;
    uint8 aggregateCharIndex = 0u;
    uint8 aggregateDigitalBits = 0u;
    uint8 numOfDigital;
    uint8 i;
    
    for(i = 0; i < (CYBLE_GATT_MTU-3u); i++)
    {   
        aggregateChar[i] = 0u;
    }
    
    for(i = 0; i < CYBLE_AIOS_CHAR_PRESENTATION_FORMAT_LEN; i++)
    {   
        locAttrValue[i] = 0u;
    }
   
    if((cyBle_aioss.charInfoAddr[CYBLE_AIOS_AGGREGATE].charInfoPtr != NULL) &&
        (CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE != cyBle_aioss.charInfoAddr[CYBLE_AIOS_AGGREGATE].charInfoPtr[0].charHandle))
    {
        for(locCharIndex = CYBLE_AIOS_DIGITAL; (locCharIndex <= CYBLE_AIOS_ANALOG) && (gattErr == CYBLE_GATT_ERR_NONE); 
                        locCharIndex++)
        {
            minDescriptionValue = 0u;
            minDescriptionNumber = 0u;
            
            for(locCharInstanceNumber = 0u; 
                    (locCharInstanceNumber < cyBle_aiossCharInstances[locCharIndex]) && (gattErr == CYBLE_GATT_ERR_NONE);
                            locCharInstanceNumber++)
            {   
                maxDescriptionValue = CYBLE_AIOS_MAX_CHAR_DESCRIPTION_VALUE;
                
                for(locCharInstance = 0u; 
                        (locCharInstance < cyBle_aiossCharInstances[locCharIndex]) && (gattErr == CYBLE_GATT_ERR_NONE);
                                locCharInstance++)
                {
                    if((cyBle_aioss.charInfoAddr[locCharIndex].charInfoPtr != NULL) &&
                        (cyBle_aioss.charInfoAddr[locCharIndex].charInfoPtr[locCharInstance].charHandle
                                != CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE) &&
                        (cyBle_aioss.charInfoAddr[locCharIndex].charInfoPtr[locCharInstance].descrHandle[CYBLE_AIOS_CHAR_PRESENTATION_FORMAT]
                                != CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE))
                    {
                        locHandleValuePair.attrHandle =
                            cyBle_aioss.charInfoAddr[locCharIndex].charInfoPtr[locCharInstance].descrHandle[CYBLE_AIOS_CHAR_PRESENTATION_FORMAT];
                        locHandleValuePair.value.len = CYBLE_AIOS_CHAR_PRESENTATION_FORMAT_LEN;
                        locHandleValuePair.value.val = locAttrValue;
                           
                        gattErr = CyBle_GattsReadAttributeValue(&locHandleValuePair, NULL, CYBLE_GATT_DB_LOCALLY_INITIATED);
                    }
                    else
                    {
                        gattErr = CYBLE_GATT_ERR_INVALID_HANDLE;
                    }
                   
                    if (gattErr == CYBLE_GATT_ERR_NONE)
                    {   
                        currentDescriptionValue = ((uint16)(((uint16)locAttrValue[CYBLE_AIOS_CHAR_PRESENTATION_FORMAT_LEN-1u]) << 8u)) | ((uint16)locAttrValue[CYBLE_AIOS_CHAR_PRESENTATION_FORMAT_LEN-2u]);
                        
                        if ((currentDescriptionValue < maxDescriptionValue) &&
                            (currentDescriptionValue > minDescriptionValue) )
                        {
                            maxDescriptionValue = currentDescriptionValue;
                            minDescriptionNumber = locCharInstance;
                            minDescriptionValueTmp = currentDescriptionValue;
                        }
                    }
                }
            
                minDescriptionValue = minDescriptionValueTmp;
                
                if (locCharIndex == CYBLE_AIOS_DIGITAL)
                {   
                    /* Read descriptor CYBLE_AIOS_NUM_OF_DIGITAL_DESCR value from database */
                    locHandleValuePair.attrHandle =
                        cyBle_aioss.charInfoAddr[CYBLE_AIOS_DIGITAL].charInfoPtr[minDescriptionNumber].descrHandle[CYBLE_AIOS_NUM_OF_DIGITAL_DESCR];
                    locHandleValuePair.value.len = CYBLE_AIOS_NUM_OF_DIGITAL_DESCR_LEN;
                    locHandleValuePair.value.val = &numOfDigital;
                    
                    gattErr = CyBle_GattsReadAttributeValue(&locHandleValuePair, NULL, CYBLE_GATT_DB_LOCALLY_INITIATED);
                        
                    if (gattErr == CYBLE_GATT_ERR_NONE)    
                    {
                        locHandleValuePair.attrHandle =
                            cyBle_aioss.charInfoAddr[CYBLE_AIOS_DIGITAL].charInfoPtr[minDescriptionNumber].charHandle;
                        locHandleValuePair.value.len = (numOfDigital / 4) + 1;
                        locHandleValuePair.value.val = &aggregateChar[aggregateCharIndex + 1];
                                            
                        gattErr = CyBle_GattsReadAttributeValue(&locHandleValuePair, NULL, CYBLE_GATT_DB_LOCALLY_INITIATED);
                        
                        if (gattErr == CYBLE_GATT_ERR_NONE)    
                        {    
                            for (i = 0; i < ((numOfDigital / 4u) + 1u); i++)
                            {
                                aggregateChar[aggregateCharIndex+i] |= (uint8)(aggregateChar[aggregateCharIndex + i + 1] << aggregateDigitalBits);
                                aggregateChar[aggregateCharIndex + i + 1] >>= (8 - aggregateDigitalBits);
                            }

                            aggregateCharIndex += (((aggregateDigitalBits / 2u) + numOfDigital) / 4u ); 
                            aggregateDigitalBits = (((aggregateDigitalBits / 2u) + numOfDigital) % 4u ) * 2u;
                        }
                    }
                }
                else if(locCharIndex == CYBLE_AIOS_ANALOG)
                {
                    /* Read characteristic CYBLE_AIOS_ANALOG value from database */ 
                    locHandleValuePair.attrHandle =
                        cyBle_aioss.charInfoAddr[locCharIndex].charInfoPtr[minDescriptionNumber].charHandle;
                    locHandleValuePair.value.len = CYBLE_AIOS_ANALOG_LEN;
                    locHandleValuePair.value.val = &aggregateChar[aggregateCharIndex];
                                        
                    gattErr = CyBle_GattsReadAttributeValue(&locHandleValuePair, NULL, CYBLE_GATT_DB_LOCALLY_INITIATED);
                    if (gattErr == CYBLE_GATT_ERR_NONE)    
                    {
                        aggregateCharIndex += 2u;
                    }
                }
                else
                {
                   /* Do nothing */
                }
            }  
            aggregateCharIndex++;
        }
        
        if (gattErr == CYBLE_GATT_ERR_NONE)
        {
            locHandleValuePair.attrHandle = cyBle_aioss.charInfoAddr[CYBLE_AIOS_AGGREGATE].charInfoPtr[0].charHandle;
            locHandleValuePair.value.len = aggregateCharIndex-1u;
            locHandleValuePair.value.val = aggregateChar;

            /* Store data in database */
            gattErr = CyBle_GattsWriteAttributeValue(&locHandleValuePair, 0u, NULL, CYBLE_GATT_DB_LOCALLY_INITIATED);
        }
    }    
    return gattErr;
}

#endif /* CYBLE_AIOS_SERVER */

#ifdef CYBLE_AIOS_CLIENT

/******************************************************************************
* Function Name: CyBle_AioscDiscoverCharacteristicsEventHandler
***************************************************************************//**
* 
*  This function is called on receiving a CYBLE_EVT_GATTC_READ_BY_TYPE_RSP
*  event. Based on the service UUID, an appropriate data structure is populated
*  using the data received as part of the callback.
* 
*  \param discCharInfo: The pointer to a characteristic information structure.
* 
******************************************************************************/
void CyBle_AioscDiscoverCharacteristicsEventHandler(const CYBLE_DISC_CHAR_INFO_T *discCharInfo)
{
    uint8 i;
    uint8 j;
    uint8 exitLoop = 0u;

    /* AIOS characteristics UUIDs */
    static const CYBLE_UUID16 cyBle_AioscCharUuid[CYBLE_AIOS_CHAR_COUNT] =
    {
        CYBLE_UUID_CHAR_DIGITAL,
        CYBLE_UUID_CHAR_ANALOG,
        CYBLE_UUID_CHAR_AGGREGATE
    };

    static CYBLE_GATT_DB_ATTR_HANDLE_T *aiosLastEndHandle = NULL;

    /* Update last characteristic endHandle to declaration handle of this characteristic */
    if(aiosLastEndHandle != NULL)
    {
        *aiosLastEndHandle = discCharInfo->charDeclHandle - 1u;
        aiosLastEndHandle = NULL;
    }

    /* Search through all available characteristics */
    for(i = (uint8) CYBLE_AIOS_DIGITAL; ((i < (uint8) CYBLE_AIOS_CHAR_COUNT) && (exitLoop == 0u)); i++)
    {
        for(j = 0u; j < cyBle_aioscCharInstances[i]; j++)
        {
            if(cyBle_AioscCharUuid[i] == discCharInfo->uuid.uuid16)
            {
                if(cyBle_aiosc.charInfoAddr[i].charInfoPtr != NULL)
                {
                    if(cyBle_aiosc.charInfoAddr[i].charInfoPtr[j].valueHandle == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
                    {
                        cyBle_aiosc.charInfoAddr[i].charInfoPtr[j].valueHandle = discCharInfo->valueHandle;
                        cyBle_aiosc.charInfoAddr[i].charInfoPtr[j].properties = discCharInfo->properties;
                        aiosLastEndHandle = &cyBle_aiosc.charInfoAddr[i].charInfoPtr[j].endHandle;
                        exitLoop = 1u;
                        break;
                    }
                }
            }
        }
    }

    /* Initialize characteristic endHandle to Service endHandle.
       Characteristic endHandle will be updated to the declaration
       handler of the following characteristic,
       in the following characteristic discovery procedure. */
    if(aiosLastEndHandle != NULL)
    {
        *aiosLastEndHandle = cyBle_serverInfo[cyBle_disCount].range.endHandle;
    }
}

/******************************************************************************
* Function Name: CyBle_AisscDiscoverCharDescriptorsEventHandler
***************************************************************************//**
* 
*  This function is called on receiving a CYBLE_EVT_GATTC_FIND_INFO_RSP event.
*  Based on the descriptor UUID, an appropriate data structure is populated using
*  the data received as part of the callback.
* 
*  \param discDescrInfo: The pointer to a descriptor information structure.
*  
******************************************************************************/
void CyBle_AioscDiscoverCharDescriptorsEventHandler(CYBLE_DISC_DESCR_INFO_T *discDescrInfo)
{
    CYBLE_AIOS_DESCR_INDEX_T locDescIndex;

    if(cyBle_aiosc.charInfoAddr[cyBle_aioscActiveCharIndex].charInfoPtr != NULL)
    {
        switch(discDescrInfo->uuid.uuid16)
        {
        case CYBLE_UUID_CHAR_CLIENT_CONFIG:
            locDescIndex = CYBLE_AIOS_CCCD;
            break;
        case CYBLE_UUID_CHAR_FORMAT:
            locDescIndex = CYBLE_AIOS_CHAR_PRESENTATION_FORMAT;
            break;
        case CYBLE_UUID_CHAR_USER_DESCRIPTION:
            locDescIndex = CYBLE_AIOS_CHAR_USER_DESCRIPTION_DESCR;
            break;
        case CYBLE_UUID_CHAR_EXTENDED_PROPERTIES:
            locDescIndex = CYBLE_AIOS_CHAR_EXTENDED_PROPERTIES;
            break;
        case CYBLE_UUID_CHAR_VALUE_TRIGGER_SETTING:
            locDescIndex = CYBLE_AIOS_VALUE_TRIGGER_SETTINGS;
            break;
        case CYBLE_UUID_CHAR_TIME_TRIGGER_SETTING:
            locDescIndex = CYBLE_AIOS_TIME_TRIGGER_SETTINGS;
            break;
        case CYBLE_UUID_CHAR_VALID_RANGE:
            locDescIndex = CYBLE_AIOS_VRD;
            break;
        case CYBLE_UUID_CHAR_NUMBER_OF_DIGITALS:
            locDescIndex = CYBLE_AIOS_NUM_OF_DIGITAL_DESCR;
            break;
        default:
            /* Not supported descriptor */
            locDescIndex = CYBLE_AIOS_DESCR_COUNT;
            break;
        }

        if(locDescIndex < CYBLE_AIOS_DESCR_COUNT)
        {
            if(cyBle_aiosc.charInfoAddr[cyBle_aioscActiveCharIndex].charInfoPtr[cyBle_aioscActiveCharInstance].descrHandle[locDescIndex] ==
                CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
            {
                cyBle_aiosc.charInfoAddr[cyBle_aioscActiveCharIndex].charInfoPtr[cyBle_aioscActiveCharInstance].descrHandle[locDescIndex] =
                    discDescrInfo->descrHandle;

            }
            else    /* Duplication of descriptor */
            {
                CyBle_ApplCallback(CYBLE_EVT_GATTC_DESCR_DUPLICATION, &discDescrInfo->uuid);
            }
        }
    }
}

/******************************************************************************
* Function Name: CyBle_AioscNotificationEventHandler
***************************************************************************//**
* 
*  Handles a notification event for the Automation IO Service.
* 
*  \param eventParam: The pointer to the data structure specified by an event.
*  
******************************************************************************/
void CyBle_AioscNotificationEventHandler(CYBLE_GATTC_HANDLE_VALUE_NTF_PARAM_T *eventParam)
{
    uint8 j;
    CYBLE_AIOS_CHAR_INDEX_T locCharIndex;
    CYBLE_AIOS_CHAR_VALUE_T notifValue;
    uint8 exitLoop = 0u;
    
    if(CyBle_AiosApplCallback != NULL)
    {
        for(locCharIndex = CYBLE_AIOS_DIGITAL; (locCharIndex < CYBLE_AIOS_CHAR_COUNT) && (exitLoop == 0u); locCharIndex++)
        {
            if(cyBle_aiosc.charInfoAddr[locCharIndex].charInfoPtr != NULL)
            {
                for(j = 0u; j < cyBle_aioscCharInstances[locCharIndex]; j++)
                {
                    if(cyBle_aiosc.charInfoAddr[locCharIndex].charInfoPtr[j].valueHandle ==
                        eventParam->handleValPair.attrHandle)
                    {
                        notifValue.connHandle = eventParam->connHandle;
                        notifValue.charIndex = locCharIndex;
                        notifValue.value = &eventParam->handleValPair.value;
                        CyBle_AiosApplCallback((uint32) CYBLE_EVT_AIOSC_NOTIFICATION, &notifValue);
                        cyBle_eventHandlerFlag &= (uint8)~CYBLE_CALLBACK;
                        exitLoop = 1u;
                        break;
                    }
                }
            }
        }
    }
}

/******************************************************************************
* Function Name: CyBle_AioscIndicationEventHandler
***************************************************************************//**
* 
*  Handles an indication event for Automation IO Service.
* 
*  \param eventParam: The pointer to the data structure specified by an event.
*  
******************************************************************************/
void CyBle_AioscIndicationEventHandler(CYBLE_GATTC_HANDLE_VALUE_IND_PARAM_T *eventParam)
{
    uint8 j;
    CYBLE_AIOS_CHAR_INDEX_T locCharIndex;
    CYBLE_AIOS_CHAR_VALUE_T indicationValue;

    if(CyBle_AiosApplCallback != NULL)
    {
        for(locCharIndex = CYBLE_AIOS_DIGITAL; locCharIndex < CYBLE_AIOS_CHAR_COUNT; locCharIndex++)
        {
            if(cyBle_aiosc.charInfoAddr[locCharIndex].charInfoPtr != NULL)
            {
                for(j = 0u; j < cyBle_aioscCharInstances[locCharIndex]; j++)
                {
                    if(cyBle_aiosc.charInfoAddr[locCharIndex].charInfoPtr[j].valueHandle ==
                        eventParam->handleValPair.attrHandle)
                    {
                        indicationValue.connHandle = eventParam->connHandle;
                        indicationValue.charIndex = locCharIndex;
                        indicationValue.value = &eventParam->handleValPair.value;
                        CyBle_AiosApplCallback((uint32) CYBLE_EVT_AIOSC_INDICATION, &indicationValue);
                        cyBle_eventHandlerFlag &= (uint8)~CYBLE_CALLBACK;
                        break;
                    }
                }
            }
        }
    }
}

/******************************************************************************
* Function Name: CyBle_AioscReadResponseEventHandler
***************************************************************************//**
* 
*  Handles a Read Response event for the Automation IO Service.
* 
*  \param eventParam: The pointer to the data structure specified by an event.
*  
******************************************************************************/
void CyBle_AioscReadResponseEventHandler(CYBLE_GATTC_READ_RSP_PARAM_T *eventParam)
{
    uint8 j;
    uint8 locReqHandle = 0u;
    CYBLE_AIOS_CHAR_INDEX_T locCharIndex;

    if((CyBle_AiosApplCallback != NULL) && (cyBle_aioscReqHandle != CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE ))
    {
        for(locCharIndex = CYBLE_AIOS_DIGITAL; (locCharIndex < CYBLE_AIOS_CHAR_COUNT) && 
            (locReqHandle == 0u); locCharIndex++)
        {
            if(cyBle_aiosc.charInfoAddr[locCharIndex].charInfoPtr != NULL)
            {
                for(j = 0u; ((j < cyBle_aioscCharInstances[locCharIndex]) && (locReqHandle == 0u)); j++)
                {
                    if(cyBle_aioscReqHandle == cyBle_aiosc.charInfoAddr[locCharIndex].charInfoPtr[j].valueHandle)
                    {
                        CYBLE_AIOS_CHAR_VALUE_T locCharValue;

                        locCharValue.connHandle = eventParam->connHandle;
                        locCharValue.charIndex = locCharIndex;
                        locCharValue.charInstance = j;
                        locCharValue.value = &eventParam->value;
                        cyBle_aioscReqHandle = CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE;
                        CyBle_AiosApplCallback((uint32) CYBLE_EVT_AIOSC_READ_CHAR_RESPONSE, &locCharValue);
                        locReqHandle = 1u;
                    }
                    else
                    {
                        CYBLE_AIOS_DESCR_INDEX_T locDescIndex;

                        for(locDescIndex = CYBLE_AIOS_CCCD; (locDescIndex < CYBLE_AIOS_DESCR_COUNT) &&
                           (locReqHandle == 0u); locDescIndex++)
                        {
                            if(cyBle_aioscReqHandle ==
                                cyBle_aiosc.charInfoAddr[locCharIndex].charInfoPtr[j].descrHandle[locDescIndex])
                            {
                                CYBLE_AIOS_DESCR_VALUE_T locDescrValue;

                                locDescrValue.connHandle = eventParam->connHandle;
                                locDescrValue.charIndex = locCharIndex;
                                locDescrValue.descrIndex = locDescIndex;
                                locDescrValue.charInstance = j;
                                locDescrValue.value = &eventParam->value;
                                cyBle_aioscReqHandle = CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE;
                                CyBle_AiosApplCallback((uint32) CYBLE_EVT_AIOSC_READ_DESCR_RESPONSE, &locDescrValue);
                                locReqHandle = 1u;
                            }
                        }
                    }
                }
            }
        }
        if(locReqHandle != 0u)
        {
            cyBle_eventHandlerFlag &= (uint8)~CYBLE_CALLBACK;
        }
    }
}

/******************************************************************************
* Function Name: CyBle_AioscWriteResponseEventHandler
***************************************************************************//**
* 
*  Handles a Write Response event for the Automation IO Service.
* 
*  \param eventParam: The pointer to a data structure specified by an event.
*  
******************************************************************************/
void CyBle_AioscWriteResponseEventHandler(const CYBLE_CONN_HANDLE_T *eventParam)
{
    uint8 j;
    uint8 locReqHandle = 0u;
    CYBLE_AIOS_CHAR_INDEX_T locCharIndex;
    CYBLE_AIOS_DESCR_VALUE_T locDescrValue;

    /* Check if service handler was registered and request handle is
    * valid.
    */
    if((CyBle_AiosApplCallback != NULL) && (cyBle_aioscReqHandle != CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE ))
    {
        for(locCharIndex = CYBLE_AIOS_DIGITAL; (locCharIndex < CYBLE_AIOS_CHAR_COUNT) &&
            (locReqHandle == 0u); locCharIndex++)
        {
            /* If this condition is false, this means that the current characteristic
            * is not included in the GUI, in other words support for the characteristic is
            * not enabled.
            */
            if(cyBle_aiosc.charInfoAddr[locCharIndex].charInfoPtr != NULL)
            {
                for(j = 0u; ((j < cyBle_aioscCharInstances[locCharIndex]) && (locReqHandle == 0u)); j++)
                {
                    /* Check if requested descriptor handle is in
                    * characteristic range.
                    */

                    if((cyBle_aiosc.charInfoAddr[locCharIndex].charInfoPtr[j].valueHandle < cyBle_aioscReqHandle) &&
                        (cyBle_aiosc.charInfoAddr[locCharIndex].charInfoPtr[j].endHandle >= cyBle_aioscReqHandle))
                    {
                        if(cyBle_aioscReqHandle ==
                            cyBle_aiosc.charInfoAddr[locCharIndex].charInfoPtr[j].descrHandle[CYBLE_AIOS_CCCD])
                        {
                            locDescrValue.descrIndex = CYBLE_AIOS_CCCD;
                        }
                        else if(cyBle_aioscReqHandle ==
                cyBle_aiosc.charInfoAddr[locCharIndex].charInfoPtr[j].descrHandle[CYBLE_AIOS_CHAR_PRESENTATION_FORMAT])
                        {
                            locDescrValue.descrIndex = CYBLE_AIOS_CHAR_PRESENTATION_FORMAT;
                        }
                        else if(cyBle_aioscReqHandle ==
                cyBle_aiosc.charInfoAddr[locCharIndex].charInfoPtr[j].descrHandle[CYBLE_AIOS_CHAR_USER_DESCRIPTION_DESCR])
                        {
                            locDescrValue.descrIndex = CYBLE_AIOS_CHAR_USER_DESCRIPTION_DESCR;
                        }
                        else if(cyBle_aioscReqHandle ==
                cyBle_aiosc.charInfoAddr[locCharIndex].charInfoPtr[j].descrHandle[CYBLE_AIOS_CHAR_EXTENDED_PROPERTIES])
                        {
                            locDescrValue.descrIndex = CYBLE_AIOS_CHAR_EXTENDED_PROPERTIES;
                        }
                        else if(cyBle_aioscReqHandle ==
                cyBle_aiosc.charInfoAddr[locCharIndex].charInfoPtr[j].descrHandle[CYBLE_AIOS_VALUE_TRIGGER_SETTINGS])
                        {
                            locDescrValue.descrIndex = CYBLE_AIOS_VALUE_TRIGGER_SETTINGS;
                        }
                        else if(cyBle_aioscReqHandle ==
                cyBle_aiosc.charInfoAddr[locCharIndex].charInfoPtr[j].descrHandle[CYBLE_AIOS_TIME_TRIGGER_SETTINGS])
                        {
                            locDescrValue.descrIndex = CYBLE_AIOS_TIME_TRIGGER_SETTINGS;
                        }
                        else if(cyBle_aioscReqHandle ==
                cyBle_aiosc.charInfoAddr[locCharIndex].charInfoPtr[j].descrHandle[CYBLE_AIOS_VRD])
                        {
                            locDescrValue.descrIndex = CYBLE_AIOS_VRD;
                        }
                        else if(cyBle_aioscReqHandle ==
                cyBle_aiosc.charInfoAddr[locCharIndex].charInfoPtr[j].descrHandle[CYBLE_AIOS_NUM_OF_DIGITAL_DESCR])
                        {
                            locDescrValue.descrIndex = CYBLE_AIOS_NUM_OF_DIGITAL_DESCR;
                        }
                        else
                        {
                            /* Should never enter here */
                        }

                        locDescrValue.connHandle = *eventParam;
                        locDescrValue.charIndex = locCharIndex;
                        locDescrValue.charInstance = j;
                        locDescrValue.value = NULL;
                        cyBle_aioscReqHandle = CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE;
                        CyBle_AiosApplCallback((uint32) CYBLE_EVT_AIOSC_WRITE_DESCR_RESPONSE, &locDescrValue);
                        locReqHandle = 1u;
                    }
                }
            }
        }

        if(locReqHandle != 0u)
        {
            cyBle_eventHandlerFlag &= (uint8)~CYBLE_CALLBACK;
        }
    }
}

/******************************************************************************
* Function Name: CyBle_AioscErrorResponseEventHandler
***************************************************************************//**
* 
*  Handles an Error Response event for the Automation IO Service.
* 
*  \param eventParam: The pointer to a data structure specified by an event.
* 
******************************************************************************/
void CyBle_AioscErrorResponseEventHandler(const CYBLE_GATTC_ERR_RSP_PARAM_T *eventParam)
{
    if((eventParam != NULL) && (eventParam->attrHandle == cyBle_aioscReqHandle))
    {
        cyBle_aioscReqHandle = CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE;
    }
}

/******************************************************************************
* Function Name: CyBle_AioscSetCharacteristicValue
***************************************************************************//**
* 
*  This function is used to write the characteristic (which is identified by
*  charIndex) value attribute in the server. As a result a Write Request is 
*  sent to the GATT Server and on successful execution of the request on the 
*  Server side the CYBLE_EVT_AIOSS_CHAR_WRITE events is generated.
*  On successful request execution on the Server side the Write Response is 
*  sent to the Client.
* 
*  \param connHandle:   The connection handle.
*  \param charIndex:    The index of the service characteristic. Starts with 
*                       zero.
*  \param charInstance: The instance number of the characteristic specified by
*                       "charIndex".
*  \param attrSize:     The size of the characteristic value attribute.
*  \param attrValue:    The pointer to the characteristic value data that 
*                       should be sent to the server device.
* 
* \return
*  A return value is of type CYBLE_API_RESULT_T.
*  * CYBLE_ERROR_OK - The request was sent successfully.
*  * CYBLE_ERROR_INVALID_PARAMETER - Validation of the input parameters failed.
*  * CYBLE_ERROR_MEMORY_ALLOCATION_FAILED - Memory allocation failed.
*  * CYBLE_ERROR_INVALID_STATE - Connection with the server is not established.
*  * CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE - The peer device doesn't have
*                                               the particular characteristic.
*  * CYBLE_ERROR_INVALID_OPERATION - Operation is invalid for this
*                                    characteristic.
*  * CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE - An optional characteristic is absent.
*
* \events
*  In case of successful execution (return value = CYBLE_ERROR_OK)
*  the next events can appear: \n
*   If the AIOS service-specific callback is registered 
*      (with CyBle_AiosRegisterAttrCallback):
*  * CYBLE_EVT_AIOSC_WRITE_CHAR_RESPONSE - In case if the requested attribute is
*                                successfully wrote on the peer device,
*                                the details (char index, etc.) are 
*                                provided with event parameter structure
*                                of type CYBLE_AIOS_CHAR_VALUE_T.
*  .
*   Otherwise (if the AIOS service-specific callback is not registered):
*  * CYBLE_EVT_GATTC_WRITE_RSP - In case if the requested attribute is 
*                                successfully wrote on the peer device.
*  * CYBLE_EVT_GATTC_ERROR_RSP - In case if there some trouble with the 
*                                requested attribute on the peer device,
*                                the details are provided with event parameters 
*                                structure (CYBLE_GATTC_ERR_RSP_PARAM_T).
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_AioscSetCharacteristicValue(CYBLE_CONN_HANDLE_T connHandle, CYBLE_AIOS_CHAR_INDEX_T charIndex,
    uint8 charInstance, uint8 attrSize, uint8 * attrValue)
{
    CYBLE_API_RESULT_T apiResult;
    CYBLE_GATT_HANDLE_VALUE_PAIR_T writeReqParam;

    if(CyBle_GetClientState() != CYBLE_CLIENT_STATE_DISCOVERED)
    {
        apiResult = CYBLE_ERROR_INVALID_STATE;
    }
    else if(charIndex >= CYBLE_AIOS_CHAR_COUNT)
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
    else if(charInstance >= cyBle_aioscCharInstances[charIndex])
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
    else if(cyBle_aiosc.charInfoAddr[charIndex].charInfoPtr[charInstance].valueHandle != CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
        writeReqParam.attrHandle = cyBle_aiosc.charInfoAddr[charIndex].charInfoPtr[charInstance].valueHandle;
        writeReqParam.value.val = attrValue;
        writeReqParam.value.len = attrSize;

        apiResult = CyBle_GattcWriteCharacteristicValue(connHandle, &writeReqParam);
        
        /* Save handle to support service specific write response from device */
        if(apiResult == CYBLE_ERROR_OK)
        {
            cyBle_aioscReqHandle = writeReqParam.attrHandle;
        }
    }
    else
    {
        apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
    }

    return (apiResult);
}


/******************************************************************************
* Function Name: CyBle_AioscGetCharacteristicValue
***************************************************************************//**
* 
*  This function is used to read a characteristic value, which is a value
*  identified by charIndex, from the server.
* 
*  \param connHandle:   The connection handle.
*  \param charIndex:    The index of the service characteristic. Starts with 
*                       zero.
*  \param charInstance: The instance number of the characteristic specified by
*                       "charIndex".
* 
* \return
*  Return value is of type CYBLE_API_RESULT_T.
*  * CYBLE_ERROR_OK - The read request was sent successfully.
*  * CYBLE_ERROR_INVALID_PARAMETER - Validation of the input parameters failed.
*  * CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE - The peer device doesn't have
*                                              the particular characteristic.
*  * CYBLE_ERROR_MEMORY_ALLOCATION_FAILED - Memory allocation failed.
*  * CYBLE_ERROR_INVALID_STATE - Connection with the server is not established.
*  * CYBLE_ERROR_INVALID_OPERATION - Operation is invalid for this
*                                    characteristic.
*  * CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE - An optional characteristic is 
*                                              absent.
*
* \events
*  In case of successful execution (return value = CYBLE_ERROR_OK)
*  the next events can appear: \n
*   If the AIOS service-specific callback is registered 
*      (with CyBle_AiosRegisterAttrCallback):
*  * CYBLE_EVT_AIOSC_READ_CHAR_RESPONSE - In case if the requested attribute is
*                                successfully wrote on the peer device,
*                                the details (char index , value, etc.) are 
*                                provided with event parameter structure
*                                of type CYBLE_AIOS_CHAR_VALUE_T.
*  .
*   Otherwise (if the AIOS service-specific callback is not registered):
*  * CYBLE_EVT_GATTC_READ_RSP - In case if the requested attribute is 
*                               successfully read on the peer device,
*                               the details (handle, value, etc.) are 
*                               provided with event parameters 
*                               structure (CYBLE_GATTC_READ_RSP_PARAM_T).
*  * CYBLE_EVT_GATTC_ERROR_RSP - In case if there some trouble with the 
*                                requested attribute on the peer device,
*                                the details are provided with event parameters 
*                                structure (CYBLE_GATTC_ERR_RSP_PARAM_T).
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_AioscGetCharacteristicValue(CYBLE_CONN_HANDLE_T connHandle, CYBLE_AIOS_CHAR_INDEX_T charIndex,
    uint8 charInstance)
{
    CYBLE_API_RESULT_T apiResult;

    if(CyBle_GetClientState() != CYBLE_CLIENT_STATE_DISCOVERED)
    {
        apiResult = CYBLE_ERROR_INVALID_STATE;
    }
    else if(charIndex >= CYBLE_AIOS_CHAR_COUNT)
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
    else if(charInstance >= cyBle_aioscCharInstances[charIndex])
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
    else if(cyBle_aiosc.charInfoAddr[charIndex].charInfoPtr[charInstance].valueHandle != CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
        apiResult = CyBle_GattcReadCharacteristicValue(connHandle,
                                            cyBle_aiosc.charInfoAddr[charIndex].charInfoPtr[charInstance].valueHandle);

        /* Save handle to support service specific read response from device */
        if(apiResult == CYBLE_ERROR_OK)
        {
            cyBle_aioscReqHandle = cyBle_aiosc.charInfoAddr[charIndex].charInfoPtr[charInstance].valueHandle;
        }
    }
    else
    {
        apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
    }

    return (apiResult);
}


/******************************************************************************
* Function Name: CyBle_AioscSetCharacteristicDescriptor
***************************************************************************//**
* 
*  This function is used to write the characteristic (which is identified by
*  charIndex) value attribute in the server. As a result a Write Request is 
*  sent to the GATT Server and on successful execution of the request on the 
*  Server side the CYBLE_EVT_AIOSS_DESCR_WRITE events is generated.
*  On successful request execution on the Server side the Write Response is 
*  sent to the Client.
* 
*  Internally, Write Request is sent to the GATT Server and on successful 
*  execution of the request on the Server side the following events can be 
*  generated: 
*  * CYBLE_EVT_AIOSS_NOTIFICATION_ENABLED ;
*  * CYBLE_EVT_AIOSS_NOTIFICATION_DISABLED;
*  * CYBLE_EVT_AIOSS_INDICATION_ENABLED;
*  * CYBLE_EVT_AIOSS_INDICATION_DISABLED.
*
*  \param connHandle:   The connection handle.
*  \param charIndex:    The index of the service characteristic. Starts with 
*                       zero.
*  \param descrIndex:   The index of the service characteristic descriptor.
*  \param charInstance: The instance number of the characteristic specified by
*                       "charIndex".
*  \param attrSize:     The size of the characteristic value attribute.
*  \param attrValue:    The pointer to the characteristic descriptor value data
*                       that should be sent to the server device.
* 
* \return
*  A return value is of type CYBLE_API_RESULT_T.
*  * CYBLE_ERROR_OK - The request was sent successfully.
*  * CYBLE_ERROR_INVALID_PARAMETER - Validation of the input parameters failed.
*  * CYBLE_ERROR_INVALID_STATE - The state is not valid.
*  * CYBLE_ERROR_MEMORY_ALLOCATION_FAILED - Memory allocation failed.
*  * CYBLE_ERROR_INVALID_OPERATION - This operation is not permitted on
*                                    the specified attribute.
*  * CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE - An optional Characteristic Descriptor
*                                              is absent.
*
* \events
*  In case of successful execution (return value = CYBLE_ERROR_OK)
*  the next events can appear: \n
*   If the AIOS service-specific callback is registered 
*      (with CyBle_AiosRegisterAttrCallback):
*  * CYBLE_EVT_AIOSC_WRITE_DESCR_RESPONSE - In case if the requested attribute 
*                                is successfully wrote on the peer device,
*                                the details (char index, descr index etc.) are
*                                provided with event parameter structure
*                                of type CYBLE_AIOS_DESCR_VALUE_T.
*  .
*   Otherwise (if the AIOS service-specific callback is not registered):
*  * CYBLE_EVT_GATTC_WRITE_RSP - In case if the requested attribute is 
*                                successfully wrote on the peer device.
*  * CYBLE_EVT_GATTC_ERROR_RSP - In case if there some trouble with the 
*                                requested attribute on the peer device,
*                                the details are provided with event parameters
*                                structure (CYBLE_GATTC_ERR_RSP_PARAM_T).
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_AioscSetCharacteristicDescriptor(CYBLE_CONN_HANDLE_T connHandle,
    CYBLE_AIOS_CHAR_INDEX_T charIndex, uint8 charInstance, CYBLE_AIOS_DESCR_INDEX_T descrIndex, uint8 attrSize,
        uint8 *attrValue)
{
    CYBLE_API_RESULT_T apiResult;
    CYBLE_GATTC_WRITE_REQ_T writeReqParam;
    
    if(cyBle_aiosc.charInfoAddr[charIndex].charInfoPtr[charInstance].descrHandle[descrIndex] != CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
        if(CyBle_GetClientState() != CYBLE_CLIENT_STATE_DISCOVERED)
        {
            apiResult = CYBLE_ERROR_INVALID_STATE;
        }
        else if((charIndex >= CYBLE_AIOS_CHAR_COUNT) || 
                (charInstance >= cyBle_aioscCharInstances[charIndex]) ||
                (descrIndex >= CYBLE_AIOS_DESCR_COUNT))
        {
            apiResult = CYBLE_ERROR_INVALID_PARAMETER;
        }
        else if((descrIndex == CYBLE_AIOS_CHAR_PRESENTATION_FORMAT) || 
                (descrIndex == CYBLE_AIOS_CHAR_EXTENDED_PROPERTIES) ||
                (descrIndex == CYBLE_AIOS_VRD) ||
                (descrIndex == CYBLE_AIOS_NUM_OF_DIGITAL_DESCR))
        {
            apiResult = CYBLE_ERROR_INVALID_OPERATION;
        }
        else
        {
         /* Fill all fields of write request structure ... */
            writeReqParam.attrHandle = cyBle_aiosc.charInfoAddr[charIndex].charInfoPtr[charInstance].descrHandle[descrIndex];
            writeReqParam.value.val = attrValue;
            writeReqParam.value.len = attrSize;

            /* ... and send request to server's device. */
            apiResult = CyBle_GattcWriteCharacteristicDescriptors(connHandle, &writeReqParam);

            /* Save handle to support service specific read response from device */
            if(apiResult == CYBLE_ERROR_OK)
            {
                cyBle_aioscReqHandle = writeReqParam.attrHandle;
            }
        }
    }
    else
    {
        apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
    }

    return (apiResult);
}

/******************************************************************************
* Function Name: CyBle_AioscGetCharacteristicDescriptor
***************************************************************************//**
* 
*  Sends a request to get the characteristic descriptor of the specified 
*  characteristic of the service.
* 
*  \param connHandle:   The connection handle.
*  \param charIndex:    The index of the service characteristic. Starts with 
*                       zero.
*  \param charInstance: The instance number of the characteristic specified by
*                       "charIndex".
*  \param descrIndex:   The index of the service characteristic descriptor.
* 
*  \return
*  * CYBLE_ERROR_OK - The request was sent successfully.
*  * CYBLE_ERROR_INVALID_PARAMETER - Validation of the input parameters failed.
*  * CYBLE_ERROR_INVALID_STATE - The state is not valid.
*  * CYBLE_ERROR_MEMORY_ALLOCATION_FAILED - Memory allocation failed.
*  * CYBLE_ERROR_INVALID_OPERATION - This operation is not permitted on
*                                    the specified attribute.
*  * CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE - An optional Characteristic Descriptor
*                                              is absent.
*  \events
*  In case of successful execution (return value = CYBLE_ERROR_OK)
*  the next events can appear: \n
*  If the AIOS service-specific callback is registered 
*      (with CyBle_AiosRegisterAttrCallback):
*  * CYBLE_EVT_AIOSC_READ_DESCR_RESPONSE - In case if the requested attribute is
*                                successfully wrote on the peer device,
*                                the details (char index, descr index, value, etc.) 
*                                are provided with event parameter structure
*                                of type CYBLE_AIOS_DESCR_VALUE_T. 
*  .
*  Otherwise (if the AIOS service-specific callback is not registered):
*  * CYBLE_EVT_GATTC_READ_RSP - In case if the requested attribute is 
*                               successfully read on the peer device,
*                               the details (handle, value, etc.) are 
*                               provided with event parameters 
*                               structure (CYBLE_GATTC_READ_RSP_PARAM_T).
*  * CYBLE_EVT_GATTC_ERROR_RSP - In case if there some trouble with the 
*                                requested attribute on the peer device,
*                                the details are provided with event parameters 
*                                structure (CYBLE_GATTC_ERR_RSP_PARAM_T).
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_AioscGetCharacteristicDescriptor(CYBLE_CONN_HANDLE_T connHandle,
    CYBLE_AIOS_CHAR_INDEX_T charIndex, uint8 charInstance, CYBLE_AIOS_DESCR_INDEX_T descrIndex)
{
    CYBLE_API_RESULT_T apiResult;

    if(cyBle_aiosc.charInfoAddr[charIndex].charInfoPtr[charInstance].descrHandle[descrIndex] != CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {

        if(CyBle_GetClientState() != CYBLE_CLIENT_STATE_DISCOVERED)
        {
            apiResult = CYBLE_ERROR_INVALID_STATE;
        }
        else if((charIndex >= CYBLE_AIOS_CHAR_COUNT) || 
                (charInstance >= cyBle_aioscCharInstances[charIndex]) ||
                (descrIndex >= CYBLE_AIOS_DESCR_COUNT))
        {
            apiResult = CYBLE_ERROR_INVALID_PARAMETER;
        }
        else
        {
            apiResult = CyBle_GattcReadCharacteristicDescriptors(connHandle,
                                    cyBle_aiosc.charInfoAddr[charIndex].charInfoPtr[charInstance].descrHandle[descrIndex]);

            /* Save handle to support service specific read response from device */
            if(apiResult == CYBLE_ERROR_OK)
            {
                cyBle_aioscReqHandle = cyBle_aiosc.charInfoAddr[charIndex].charInfoPtr[charInstance].descrHandle[descrIndex];
            }
        }
    }
    else
    {
        apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
    }

    return (apiResult);
}

/******************************************************************************
* Function Name: CyBle_AioscSetCharacteristicValueWithoutResponse
***************************************************************************//**
* 
*  This function is used to write the characteristic (which is identified by
*  charIndex) value attribute in the server without response.
* 
* 
*  \param connHandle:   The connection handle.
*  \param charIndex:    The index of the service characteristic.
*  \param charInstance: The instance number of the characteristic specified by
*                       "charIndex".
*  \param attrSize:     The size of the characteristic value attribute.
*  \param attrValue:    The pointer to the characteristic value data that 
*                       should be sent to the server device.
* 
* \return
*  A return value is of type CYBLE_API_RESULT_T.
*  * CYBLE_ERROR_OK - The request was sent successfully.
*  * CYBLE_ERROR_INVALID_PARAMETER - Validation of the input parameters failed.
*  * CYBLE_ERROR_MEMORY_ALLOCATION_FAILED - Memory allocation failed.
*  * CYBLE_ERROR_INVALID_STATE - Connection with the server is not established.
*  * CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE - The peer device doesn't have
*                                              the particular characteristic.
*  * CYBLE_ERROR_INVALID_OPERATION - Operation is invalid for this
*                                    characteristic.
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_AioscSetCharacteristicValueWithoutResponse( CYBLE_CONN_HANDLE_T connHandle,
            CYBLE_AIOS_CHAR_INDEX_T charIndex, uint8 charInstance, uint8 attrSize, uint8 * attrValue)
{
    CYBLE_API_RESULT_T apiResult;
    uint16 locMtu = CYBLE_GATT_MTU;
    
    (void) CyBle_GattGetMtuSize(&locMtu);
    
    /* Check parameters */
    if(CyBle_GetClientState() != CYBLE_CLIENT_STATE_DISCOVERED)
    {
        apiResult = CYBLE_ERROR_INVALID_STATE;
    }
    else if(charIndex >= CYBLE_AIOS_CHAR_COUNT)
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
    else if(charInstance >= cyBle_aioscCharInstances[charIndex])
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
	else if(cyBle_aiosc.charInfoAddr[charIndex].charInfoPtr[charInstance].valueHandle == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
		apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
	}
    else if((CYBLE_CHAR_PROP_WRITE & cyBle_aiosc.charInfoAddr[charIndex].charInfoPtr[charInstance].properties) == 0u)
    {
        apiResult = CYBLE_ERROR_INVALID_OPERATION;
    }
    else if((locMtu - 3u) < attrSize)
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
    else 
    {   
        CYBLE_GATTC_WRITE_CMD_REQ_T writeCmdReqParam;
        
        writeCmdReqParam.attrHandle = cyBle_aiosc.charInfoAddr[charIndex].charInfoPtr[charInstance].valueHandle;
        writeCmdReqParam.value.val = attrValue;
        writeCmdReqParam.value.len = attrSize;

        apiResult =  CyBle_GattcWriteWithoutResponse(connHandle, &writeCmdReqParam);
        
        if(apiResult == CYBLE_ERROR_OK)
        {
            cyBle_aioscReqHandle = cyBle_aiosc.charInfoAddr[charIndex].charInfoPtr[charInstance].valueHandle;
        }
    }

    return (apiResult);
}

#endif /* CYBLE_AIOS_CLIENT */


/* [] END OF FILE */
