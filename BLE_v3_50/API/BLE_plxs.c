/***************************************************************************//**
* \file cyble_plxs.c
* \version `$CY_MAJOR_VERSION`.`$CY_MINOR_VERSION`
* 
* \brief
*  This file contains the source code for the Pulse Oximeter Service of the BLE Component.
* 
********************************************************************************
* \copyright
* Copyright 2014-2018, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "`$INSTANCE_NAME`_eventHandler.h"
#include "`$INSTANCE_NAME`_plxs.h"

#if(CYBLE_MODE_PROFILE && defined(CYBLE_PLXS))
    
static CYBLE_CALLBACK_T CyBle_PlxsApplCallback;

#ifdef CYBLE_PLXS_SERVER
    
/* Generated code */    
`$PlxsServer`
/* End of generated code */
    
/* PLXS Server flags */
uint32 cyBle_plxssFlag;
static CYBLE_GATT_DB_ATTR_HANDLE_T cyble_plxssReqHandle;
    
#endif /* CYBLE_PLXS_SERVER */

#ifdef CYBLE_PLXS_CLIENT
    
/* Generated code */
`$PlxsClient`
/* End of generated code */

/* Server's PLX Service characteristics GATT DB handles structure */
CYBLE_PLXSC_T cyBle_plxsc;

/* Internal storage for last request handle to check the response */
static CYBLE_GATT_DB_ATTR_HANDLE_T cyBle_plxscReqHandle;

static uint16 cyBle_plxscRacpTimeout;

uint8 cyBle_plxscActiveCharIndex = 0u;

/* PLXS Client flags */
uint32 cyBle_plxscFlag;
#endif /* CYBLE_PLXS_CLIENT */

/****************************************************************************** 
* Function Name: CyBle_PlxsInit
***************************************************************************//**
* 
*  This function initializes the Pulse Oximeter Service.
*
******************************************************************************/
void CyBle_PlxsInit(void)
{
#ifdef CYBLE_PLXS_SERVER
    cyBle_plxssFlag = 0u;
    cyble_plxssReqHandle = CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE;
#endif /* CYBLE_PLXS_SERVER */

#ifdef CYBLE_PLXS_CLIENT
    
    if(cyBle_serverInfo[CYBLE_SRVI_PLXS].range.startHandle == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
        (void)memset(&cyBle_plxsc, 0, sizeof(CYBLE_PLXSC_T));

        /* initialize uuid  */
        cyBle_serverInfo[CYBLE_SRVI_PLXS].uuid = CYBLE_UUID_PLX_SERVICE;
        
        cyBle_plxscReqHandle = CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE;
        cyBle_plxscRacpTimeout = CYBLE_PLXS_RACP_PROCEDURE_TIMEOUT;
        cyBle_plxscFlag = 0u;
    }
          
#endif /* CYBLE_PLXS_CLIENT */
}

/******************************************************************************
* Function Name: CyBle_PlxsRegisterAttrCallback
***************************************************************************//**
* 
*  Registers a callback function for service-specific attribute operations.
*  Service specific write requests from peer device will not be handled with
*  unregistered callback function.
* 
*  \param callbackFunc:  An application layer event callback function to receive 
*                    events from the BLE Component. The definition of 
*                    cyble_callback_t for PLX Service is:\n
*                    typedef void (* cyble_callback_t) (uint32 eventCode, 
*                                                       void *eventParam)
*                    * eventCode indicates the event that triggered this 
*                      callback.
*                    * eventParam contains the parameters corresponding to the 
*                      current event.
* 
* \sideeffect    The *eventParams in the callback function should not be used 
*                by the application once the callback function execution is 
*                finished. Otherwise this data may become corrupted.
* 
******************************************************************************/
void CyBle_PlxsRegisterAttrCallback(CYBLE_CALLBACK_T callbackFunc)
{
    CyBle_PlxsApplCallback = callbackFunc;
}

#ifdef CYBLE_PLXS_SERVER

/******************************************************************************
* Function Name: CyBle_PlxssWriteEventHandler
***************************************************************************//**
* 
*  Handles the Write Request Event.
* 
*  \param eventParam: The pointer to the data structure specified by the event.
* 
* \return
*  CYBLE_GATT_ERR_CODE_T - An API result state if the API succeeded 
*  (CYBLE_GATT_ERR_NONE) or failed with error codes:
*   * CYBLE_GATTS_ERR_PROCEDURE_ALREADY_IN_PROGRESS
*   * CYBLE_GATTS_ERR_CCCD_IMPROPERLY_CONFIGURED
* 
******************************************************************************/
CYBLE_GATT_ERR_CODE_T CyBle_PlxssWriteEventHandler(CYBLE_GATTS_WRITE_REQ_PARAM_T *eventParam)
{
    CYBLE_GATT_ERR_CODE_T gattErr = CYBLE_GATT_ERR_NONE;
    CYBLE_PLXS_CHAR_VALUE_T locCharValue = { .connHandle = eventParam->connHandle };
        
    if(CyBle_PlxsApplCallback != NULL)
    {
	    if(cyBle_plxss.charInfo[CYBLE_PLXS_RACP].charHandle == eventParam->handleValPair.attrHandle)
	    {    
            if(CYBLE_PLXSS_IS_PROCEDURE_IN_PROGRESS()
                && (eventParam->handleValPair.value.val[0u] != (uint8)CYBLE_PLXS_RACP_OPC_ABORT_OPN))
            {
                gattErr = CYBLE_GATT_ERR_PROCEDURE_ALREADY_IN_PROGRESS;
            }
            else if(!CYBLE_IS_INDICATION_ENABLED(cyBle_plxss.charInfo[CYBLE_PLXS_RACP].descrHandle[CYBLE_PLXS_CCCD]))
            {
                gattErr = CYBLE_GATT_ERR_CCCD_IMPROPERLY_CONFIGURED;
            }
            else
            {   
                locCharValue.charIndex = CYBLE_PLXS_RACP;
                locCharValue.value = &eventParam->handleValPair.value;
                gattErr = CyBle_GattsWriteAttributeValue(&eventParam->handleValPair, 
    	                      0u, &eventParam->connHandle, CYBLE_GATT_DB_PEER_INITIATED);
    	        if(gattErr == CYBLE_GATT_ERR_NONE)
    	        {
    	            CyBle_PlxsApplCallback((uint32)CYBLE_EVT_PLXSS_WRITE_CHAR, &locCharValue);
                    cyBle_plxssFlag |= CYBLE_PLXS_FLAG_PROCESS;
    	        }
            }
			
			cyBle_eventHandlerFlag &= (uint8)~CYBLE_CALLBACK;
	    }
	    else
	    {    
	        for(locCharValue.charIndex = CYBLE_PLXS_SCMT;
                    locCharValue.charIndex < CYBLE_PLXS_CHAR_COUNT; locCharValue.charIndex++)
	        {
	            if(eventParam->handleValPair.attrHandle == cyBle_plxss.charInfo[locCharValue.charIndex].descrHandle[CYBLE_PLXS_CCCD])
	            {
	                locCharValue.value = NULL;
                    gattErr = CyBle_GattsWriteAttributeValue(&eventParam->handleValPair,
	                             0u, &eventParam->connHandle, CYBLE_GATT_DB_PEER_INITIATED);
                    
                    if(gattErr == CYBLE_GATT_ERR_NONE)
	                {
                        switch(locCharValue.charIndex)
                        {
                            /* INDICATION */
                            case CYBLE_PLXS_SCMT: 
                            case CYBLE_PLXS_RACP:
                                if(CYBLE_IS_INDICATION_ENABLED_IN_PTR(eventParam->handleValPair.value.val))
                                {
                                    CyBle_PlxsApplCallback((uint32)CYBLE_EVT_PLXSS_INDICATION_ENABLED, &locCharValue);
                                }
                                else
                                {
                                    CyBle_PlxsApplCallback((uint32)CYBLE_EVT_PLXSS_INDICATION_DISABLED, &locCharValue);
                                }
                                break;
                                
                            /* NOTIFICATION */
                            case CYBLE_PLXS_CTMT:  
                                if(CYBLE_IS_NOTIFICATION_ENABLED_IN_PTR(eventParam->handleValPair.value.val))
                                {
                                    CyBle_PlxsApplCallback((uint32)CYBLE_EVT_PLXSS_NOTIFICATION_ENABLED, &locCharValue);
                                }
                                else
                                {
                                    CyBle_PlxsApplCallback((uint32)CYBLE_EVT_PLXSS_NOTIFICATION_DISABLED, &locCharValue);
                                }
                                break;
                                
                            default:
                                break;
                        } 
                    }
                    cyBle_eventHandlerFlag &= (uint8)~CYBLE_CALLBACK;
                    break;
				}
            }
        }
    }
    
    if(gattErr)
    {
        /* If we have gattErr, so release flag CYBLE_PLXS_FLAG_PROCESS; */
        cyBle_plxssFlag &= (uint8_t) ~CYBLE_PLXS_FLAG_PROCESS;
    }

    return (gattErr);
}

/******************************************************************************
* Function Name: CyBle_PlxssSetCharacteristicValue
***************************************************************************//**
* 
*  Sets a characteristic value of the service, which is identified by charIndex.
* 
*  \param charIndex: The index of a service characteristic.
*  \param attrSize: The size of the characteristic value attribute.
*  \param attrValue: The pointer to the characteristic value data that should be
*               stored in the GATT database.
* 
* \return
*  Return value is of type CYBLE_API_RESULT_T.
*  * CYBLE_ERROR_OK - The request handled successfully.
*  * CYBLE_ERROR_INVALID_PARAMETER - Validation of the input parameter failed.
*  * CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE - Optional characteristic is absent
*
* \events
*  None
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_PlxssSetCharacteristicValue(CYBLE_PLXS_CHAR_INDEX_T charIndex, uint8 attrSize, uint8 *attrValue)
{
    CYBLE_API_RESULT_T apiResult;
    
    /* Check the parameters */
    if(charIndex >= CYBLE_PLXS_CHAR_COUNT) 
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
    else if(cyBle_plxss.charInfo[charIndex].charHandle == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
        apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
    }
    else
    {   CYBLE_GATT_HANDLE_VALUE_PAIR_T locHandleValuePair;
        
        /* Store characteristic value into GATT database */
        locHandleValuePair.attrHandle = cyBle_plxss.charInfo[charIndex].charHandle;
        locHandleValuePair.value.len = attrSize;
        locHandleValuePair.value.val = attrValue;
        
        if(CYBLE_GATT_ERR_NONE != CyBle_GattsWriteAttributeValue(&locHandleValuePair,
                                    0u, NULL, CYBLE_GATT_DB_LOCALLY_INITIATED))
        {
            apiResult = CYBLE_ERROR_INVALID_PARAMETER;
        }
        else
        {
            apiResult = CYBLE_ERROR_OK;
        }   
    }
    return (apiResult);
}

/******************************************************************************
* Function Name: CyBle_PlxssGetCharacteristicValue
***************************************************************************//**
* 
*  Gets a characteristic value of the service, which is identified by charIndex.
* 
*  \param charIndex: The index of a service characteristic.
*  \param attrSize: The size of the characteristic value attribute.
*  \param attrValue: Pointer to the location where Characteristic value data should 
*               be stored.
* 
* \return
*  Return value is of type CYBLE_API_RESULT_T.
*  * CYBLE_ERROR_OK - The request handled successfully
*  * CYBLE_ERROR_INVALID_PARAMETER - Validation of the input parameter failed
*  * CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE - Optional characteristic is absent
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_PlxssGetCharacteristicValue(CYBLE_PLXS_CHAR_INDEX_T charIndex, uint8 attrSize, uint8 *attrValue)
{
    CYBLE_API_RESULT_T apiResult;
    
    /* Check the parameters */
    if(charIndex >= CYBLE_PLXS_CHAR_COUNT)
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
    else if(cyBle_plxss.charInfo[charIndex].charHandle == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
        apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
    }
    else
    {   CYBLE_GATT_HANDLE_VALUE_PAIR_T locHandleValuePair;
        
        /* Get characteristic value from GATT database */
        locHandleValuePair.attrHandle = cyBle_plxss.charInfo[charIndex].charHandle;
        locHandleValuePair.value.len = attrSize;
        locHandleValuePair.value.val = attrValue;
        
        if(CYBLE_GATT_ERR_NONE != CyBle_GattsReadAttributeValue(&locHandleValuePair,
                                    0u, CYBLE_GATT_DB_LOCALLY_INITIATED))
        {
            apiResult = CYBLE_ERROR_INVALID_PARAMETER;
        }
        else
        {
            apiResult = CYBLE_ERROR_OK;
        }
    }
    return (apiResult);
}

/****************************************************************************** 
* Function Name: CyBle_PlxssSetCharacteristicDescriptor
***************************************************************************//**
* 
*  Sets a characteristic descriptor of a specified characteristic of the service.
* 
*  \param charIndex:  The index of a service characteristic of type 
*                     CYBLE_PLXS_CHAR_INDEX_T.
*  \param descrIndex: The index of a service characteristic descriptor of type 
*                     CYBLE_PLXS_DESCR_INDEX_T.
*  \param attrSize:   The size of the characteristic descriptor attribute.
*  \param attrValue:  The pointer to the descriptor value data that should 
*                     be stored to the GATT database. 
* 
* \return
*  The return value is of type CYBLE_API_RESULT_T.
*  * CYBLE_ERROR_OK - The request is handled successfully.
*  * CYBLE_ERROR_INVALID_PARAMETER - Validation of the input parameter failed.
*  * CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE - Optional descriptor is absent
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_PlxssSetCharacteristicDescriptor(CYBLE_PLXS_CHAR_INDEX_T charIndex,
                                                            CYBLE_PLXS_DESCR_INDEX_T descrIndex,
                                                            uint8 attrSize, uint8 *attrValue)
{
    CYBLE_API_RESULT_T apiResult = CYBLE_ERROR_OK;
    
    if((charIndex >= CYBLE_PLXS_CHAR_COUNT) || (descrIndex >= CYBLE_PLXS_DESCR_COUNT))
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
    else
    {   
        CYBLE_GATT_HANDLE_VALUE_PAIR_T locHandleValuePair;
        
        /* Store characteristic value into GATT database */
        locHandleValuePair.attrHandle = cyBle_plxss.charInfo[charIndex].descrHandle[descrIndex];
        locHandleValuePair.value.len = attrSize;
        locHandleValuePair.value.val = attrValue;
        
        if(CYBLE_GATT_ERR_NONE != CyBle_GattsWriteAttributeValue(&locHandleValuePair,
                                    0u, NULL, CYBLE_GATT_DB_LOCALLY_INITIATED))
        {
            apiResult = CYBLE_ERROR_INVALID_PARAMETER;
        }
        else
        {
            apiResult = CYBLE_ERROR_OK;
        }
    }
    return (apiResult);
}

/******************************************************************************
* Function Name: CyBle_PlxssGetCharacteristicDescriptor
***************************************************************************//**
* 
*  Gets the characteristic descriptor of the specified characteristic.
*
*  \param charIndex:  The index of the characteristic.
*  \param descrIndex: The index of the descriptor.
*  \param attrSize:   The size of the descriptor value attribute.
*  \param attrValue:  Pointer to the location where the descriptor value 
*                     data should be stored.
* 
* \return
*  Return value is of type CYBLE_API_RESULT_T.
*  * CYBLE_ERROR_OK - The request handled successfully
*  * CYBLE_ERROR_INVALID_PARAMETER - Validation of the input parameter failed
*  * CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE - Optional descriptor is absent
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_PlxssGetCharacteristicDescriptor(CYBLE_PLXS_CHAR_INDEX_T charIndex, 
                                                            CYBLE_PLXS_DESCR_INDEX_T descrIndex,
                                                            uint8 attrSize, uint8 *attrValue)
{
    CYBLE_API_RESULT_T apiResult;
    
    /* Check the parameters */
    if((charIndex >= CYBLE_PLXS_CHAR_COUNT) || (descrIndex >= CYBLE_PLXS_DESCR_COUNT))
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
    else if(cyBle_plxss.charInfo[charIndex].descrHandle[CYBLE_PLXS_CCCD] == 
                CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
        apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
    }
    else
    {
       CYBLE_GATT_HANDLE_VALUE_PAIR_T locHandleValuePair;
        
        /* Get characteristic value from GATT database */
        locHandleValuePair.attrHandle = cyBle_plxss.charInfo[charIndex].descrHandle[descrIndex];
        locHandleValuePair.value.len = attrSize;
        locHandleValuePair.value.val = attrValue;
        
        if(CYBLE_GATT_ERR_NONE != CyBle_GattsReadAttributeValue(&locHandleValuePair,
                                    0u, CYBLE_GATT_DB_LOCALLY_INITIATED))
        {
            apiResult = CYBLE_ERROR_INVALID_PARAMETER;
        }
        else
        {
            apiResult = CYBLE_ERROR_OK;
        }
    }
    return (apiResult);
}

/******************************************************************************
* Function Name: CyBle_PlxssSendNotification
***************************************************************************//**
* 
*  Sends a notification of the specified characteristic to the client device, 
*  as defined by the charIndex value.
* 
*  On enabling notification successfully for a service characteristic, it sends
*  out a 'Handle Value Notification' which results in 
*  CYBLE_EVT_PLXSC_NOTIFICATION event at the GATT Client's end.
*
*  \param connHandle: The connection handle which consist of the device ID and ATT 
*               connection ID.
*  \param charIndex: The index of the service characteristic.
*  \param attrSize: The size of the characteristic value attribute.
*  \param attrValue: Pointer to the Characteristic value data that should be sent 
*               to Client device.
* 
* \return
*  Return value is of type CYBLE_API_RESULT_T.
*   * CYBLE_ERROR_OK - The request handled successfully
*   * CYBLE_ERROR_INVALID_PARAMETER - Validation of the input parameter failed
*   * CYBLE_ERROR_INVALID_OPERATION - Operation is invalid for this 
*                                      characteristic
*   * CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE - Optional characteristic is absent
*   * CYBLE_ERROR_INVALID_STATE - Connection with the client is not established
*   * CYBLE_ERROR_MEMORY_ALLOCATION_FAILED - Memory allocation failed
*   * CYBLE_ERROR_NTF_DISABLED - Notification is not enabled by the client
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_PlxssSendNotification(CYBLE_CONN_HANDLE_T connHandle,
    CYBLE_PLXS_CHAR_INDEX_T charIndex, uint8 attrSize, uint8 *attrValue)
{
    CYBLE_API_RESULT_T apiResult;
    
    /* Send Notification if it is enabled and connected */
    if(CyBle_GetState() != CYBLE_STATE_CONNECTED)
    {
        apiResult = CYBLE_ERROR_INVALID_STATE;
    }
    else if(charIndex >= CYBLE_PLXS_CHAR_COUNT)
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
    else if(cyBle_plxss.charInfo[charIndex].charHandle == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
        apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
    }
    else if(cyBle_plxss.charInfo[charIndex].descrHandle[CYBLE_PLXS_CCCD] ==
                CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
        apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
    }
    else if(!CYBLE_IS_NOTIFICATION_ENABLED(cyBle_plxss.charInfo[charIndex].descrHandle[CYBLE_PLXS_CCCD]))
    {
        apiResult = CYBLE_ERROR_NTF_DISABLED;
    }
    else
    {
        CYBLE_GATTS_HANDLE_VALUE_NTF_T ntfParam;
        
        /* Fill all fields of write request structure ... */
        ntfParam.attrHandle = cyBle_plxss.charInfo[charIndex].charHandle; 
        ntfParam.value.val = attrValue;
        ntfParam.value.len = attrSize;
        
        /* Send notification to client using previously filled structure */
        apiResult = CyBle_GattsNotification(connHandle, &ntfParam);;
    }
    return (apiResult);
}

/******************************************************************************
* Function Name: CyBle_PlxssSendIndication
***************************************************************************//**
* 
*  Sends an indication of the specified characteristic to the client device, as
*  defined by the charIndex value.
* 
*  On enabling indication successfully, if the GATT server has an updated value to be 
*  indicated to the GATT Client, it sends out a 'Handle Value Indication' which
*  results in CYBLE_EVT_PLXS_INDICATION or CYBLE_EVT_GATTC_HANDLE_VALUE_IND (if 
*  service-specific callback function is not registered) event at the GATT Client's end.
*
*  \param connHandle: The connection handle which consist of the device ID and ATT 
*               connection ID.
*  \param charIndex: The index of the service characteristic.
*  \param attrSize: The size of the characteristic value attribute.
*  \param attrValue: Pointer to the Characteristic value data that should be sent 
*               to Client device.
* 
* \return
*  Return value is of type CYBLE_API_RESULT_T.
*   * CYBLE_ERROR_OK - The request handled successfully
*   * CYBLE_ERROR_INVALID_PARAMETER - Validation of the input parameter failed
*   * CYBLE_ERROR_INVALID_OPERATION - Operation is invalid for this 
*                                      characteristic
*   * CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE - Optional characteristic is absent
*   * CYBLE_ERROR_INVALID_STATE - Connection with the client is not established
*   * CYBLE_ERROR_MEMORY_ALLOCATION_FAILED - Memory allocation failed
*   * CYBLE_ERROR_IND_DISABLED - Indication is not enabled by the client
*
* \events
*  In case of successful execution (return value = CYBLE_ERROR_OK)
*  the next events can appear: \n
*   If the PLXS service-specific callback is registered 
*      (with CyBle_PlxsRegisterAttrCallback):
*  * CYBLE_EVT_PLXSS_INDICATION_CONFIRMED - in case if the indication is
*                                successfully delivered to the peer device.
*  .
*   Otherwise (if the PLXS service-specific callback is not registered):
*  * CYBLE_EVT_GATTS_HANDLE_VALUE_CNF - in case if the indication is
*                                successfully delivered to the peer device.
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_PlxssSendIndication(CYBLE_CONN_HANDLE_T connHandle,
    CYBLE_PLXS_CHAR_INDEX_T charIndex, uint8 attrSize, uint8 *attrValue)
{
    CYBLE_API_RESULT_T apiResult;
    
    /* Send Indication if it is enabled and connected */
    if(CyBle_GetState() != CYBLE_STATE_CONNECTED)
    {
        apiResult = CYBLE_ERROR_INVALID_STATE;
    }
    else if(charIndex >= CYBLE_PLXS_CHAR_COUNT)
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
    else if((cyBle_plxss.charInfo[charIndex].charHandle == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE) ||
            (cyBle_plxss.charInfo[charIndex].descrHandle[CYBLE_PLXS_CCCD] == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE))
    {
        apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
    }
    else if(!CYBLE_IS_INDICATION_ENABLED(cyBle_plxss.charInfo[charIndex].descrHandle[CYBLE_PLXS_CCCD]))
    {
        apiResult = CYBLE_ERROR_IND_DISABLED;
    }
    else
    {
         CYBLE_GATTS_HANDLE_VALUE_IND_T indParam;
        
        /* Fill all fields of write request structure ... */
        indParam.attrHandle = cyBle_plxss.charInfo[charIndex].charHandle; 
        indParam.value.val = attrValue;
        indParam.value.len = attrSize;
        
        /* Send notification to client using previously filled structure */
        apiResult = CyBle_GattsIndication(connHandle, &indParam);
        
        if(apiResult == CYBLE_ERROR_OK)
        {
            /* Save handle to support service-specific value confirmation response from client */
            cyble_plxssReqHandle = indParam.attrHandle;
        }
    }
    return (apiResult);
}

/******************************************************************************
* Function Name: CyBle_PlxssConfirmationEventHandler
***************************************************************************//**
* 
*  Handles the Value Confirmation request event from the BLE stack.
* 
*  \param eventParam: Pointer to a structure of type CYBLE_CONN_HANDLE_T
*  
******************************************************************************/
void CyBle_PlxssConfirmationEventHandler(const CYBLE_CONN_HANDLE_T *eventParam)
{
    CYBLE_PLXS_CHAR_VALUE_T locCharIndex;

    locCharIndex.connHandle = *eventParam;
    locCharIndex.value = NULL;
    
    if(CyBle_PlxsApplCallback != NULL)
    {   
        for(locCharIndex.charIndex = CYBLE_PLXS_SCMT; locCharIndex.charIndex < CYBLE_PLXS_CHAR_COUNT; 
            locCharIndex.charIndex++)
	    {
            if(cyBle_plxss.charInfo[locCharIndex.charIndex].charHandle == cyble_plxssReqHandle)
            {
                cyble_plxssReqHandle = CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE;
                
                /* We have finish of RACP procedure, so reset flag CYBLE_PLXS_FLAG_PROCESS */
                if(locCharIndex.charIndex == CYBLE_PLXS_RACP)
                {
                    cyBle_plxssFlag &= (uint8) ~CYBLE_PLXS_FLAG_PROCESS;
                }
                cyBle_eventHandlerFlag &= (uint8)~CYBLE_CALLBACK;
                CyBle_PlxsApplCallback((uint32)CYBLE_EVT_PLXSS_INDICATION_CONFIRMED, &locCharIndex);
            }
        }
    }  
}

#endif /* CYBLE_PLXS_SERVER */

#ifdef CYBLE_PLXS_CLIENT

/******************************************************************************
* Function Name: CyBle_PlxscSetCharacteristicValue
***************************************************************************//**
* 
*  This function is used to write the characteristic (which is identified by
*  charIndex) value attribute in the server. As a result a Write Request is 
*  sent to the GATT Server and on successful execution of the request on the 
*  Server side the CYBLE_EVT_PLXSS_WRITE_CHAR events is generated.
*  On successful request execution on the Server side the Write Response is 
*  sent to the Client.
* 
*  \param connHandle: The connection handle.
*  \param charIndex: The index of a service characteristic.
*  \param attrSize: The size of the characteristic value attribute.
*  \param attrValue: The pointer to the characteristic value data that should be 
*               sent to the server device.
* 
* \return
*  Return value is of type CYBLE_API_RESULT_T.
*  * CYBLE_ERROR_OK - The request was sent successfully
*  * CYBLE_ERROR_INVALID_PARAMETER - Validation of the input parameters failed
*  * CYBLE_ERROR_MEMORY_ALLOCATION_FAILED - Memory allocation failed
*  * CYBLE_ERROR_INVALID_STATE - Connection with the server is not established
*  * CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE - The peer device doesn't have
*                                               the particular characteristic
*  * CYBLE_ERROR_INVALID_OPERATION - Operation is invalid for this
*                                     characteristic
*
* \events
*  In case of successful execution (return value = CYBLE_ERROR_OK)
*  the next events can appear: \n
*   If the PLXS service-specific callback is registered 
*      (with CyBle_PlxsRegisterAttrCallback):
*  * CYBLE_EVT_PLXSC_WRITE_CHAR_RESPONSE - in case if the requested attribute is
*                                successfully wrote on the peer device,
*                                the details (char index, etc.) are 
*                                provided with event parameter structure
*                                of type CYBLE_PLXS_CHAR_VALUE_T.
*  .
*   Otherwise (if the PLXS service-specific callback is not registered):
*  * CYBLE_EVT_GATTC_WRITE_RSP - in case if the requested attribute is 
*                                successfully wrote on the peer device.
*  * CYBLE_EVT_GATTC_ERROR_RSP - in case if there some trouble with the 
*                                requested attribute on the peer device,
*                                the details are provided with event parameters 
*                                structure (cy_stc_ble_gatt_err_param_t).
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_PlxscSetCharacteristicValue(CYBLE_CONN_HANDLE_T connHandle, CYBLE_PLXS_CHAR_INDEX_T charIndex,
                                                        uint8 attrSize, uint8 * attrValue)
{
    CYBLE_API_RESULT_T apiResult;
    
    /* Check the parameters */
    if(CyBle_GetClientState() != CYBLE_CLIENT_STATE_DISCOVERED)
    {
        apiResult = CYBLE_ERROR_INVALID_STATE;
    }
    else if(charIndex >= CYBLE_PLXS_CHAR_COUNT)
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
	else if(cyBle_plxsc.charInfo[charIndex].valueHandle == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
		apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
	}
    else if((CYBLE_CHAR_PROP_WRITE & cyBle_plxsc.charInfo[charIndex].properties) == 0u)
    {
        apiResult = CYBLE_ERROR_INVALID_OPERATION;
    }
    else
    {
        CYBLE_GATT_HANDLE_VALUE_PAIR_T writeReqParam;
        
        writeReqParam.attrHandle = cyBle_plxsc.charInfo[charIndex].valueHandle;
        writeReqParam.value.val  = attrValue;
        writeReqParam.value.len  = attrSize;
        
        apiResult = CyBle_GattcWriteCharacteristicValue(connHandle, &writeReqParam);
        if(apiResult == CYBLE_ERROR_OK)
        {
            cyBle_plxscReqHandle = cyBle_plxsc.charInfo[charIndex].valueHandle;
        }
        
        if(charIndex == CYBLE_PLXS_RACP)
        {
            /* Start RACP procedure. It will ends when the Collector sends a confirmation to acknowledge the RACP 
            indication sent by the Sensor. A procedure is considered to have timed out if a RACP indication 
            (or PLX Spot-check Measurement characteristic indication) is not received within the ATT transaction timeout,
            defined as 5 seconds. CYBLE_EVT_PLXSC_TIMEOUT event indicate about CP procedure timeout.
            */
            apiResult = CyBle_StartTimer(cyBle_plxscRacpTimeout);
            if(apiResult == CYBLE_ERROR_OK)
            {
                cyBle_plxscFlag |= CYBLE_PLXS_FLAG_PROCESS;
            }
        }
    }
    return (apiResult);
}

/******************************************************************************
* Function Name: CyBle_PlxscGetCharacteristicValue
***************************************************************************//**
* 
*  This function is used to read the characteristic Value from a server
*  which is identified by charIndex.
* 
*  \param connHandle: The connection handle.
*  \param charIndex: The index of the service characteristic.
* 
* \return
*  Return value is of type CYBLE_API_RESULT_T.
*  * CYBLE_ERROR_OK - The read request was sent successfully  
*  * CYBLE_ERROR_INVALID_PARAMETER - Validation of the input parameters failed
*  * CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE - The peer device doesn't have
*                                               the particular characteristic
*  * CYBLE_ERROR_MEMORY_ALLOCATION_FAILED - Memory allocation failed
*  * CYBLE_ERROR_INVALID_STATE - Connection with the server is not established
*  * CYBLE_ERROR_INVALID_OPERATION - Operation is invalid for this 
*                                     characteristic
*
* \events
*  In case of successful execution (return value = CYBLE_ERROR_OK)
*  the next events can appear: \n
*   If the PLXS service-specific callback is registered 
*      (with CyBle_PlxsRegisterAttrCallback):
*  * CYBLE_EVT_PLXSC_READ_CHAR_RESPONSE - in case if the requested attribute is
*                                successfully wrote on the peer device,
*                                the details (char index , value, etc.) are 
*                                provided with event parameter structure
*                                of type CYBLE_PLXS_CHAR_VALUE_T.
*  .
*   Otherwise (if the PLXS service-specific callback is not registered):
*  * CYBLE_EVT_GATTC_READ_RSP - in case if the requested attribute is 
*                                successfully read on the peer device,
*                                the details (handle, value, etc.) are 
*                                provided with event parameters 
*                                structure (CYBLE_GATTC_READ_RSP_PARAM_T).
*  * CYBLE_EVT_GATTC_ERROR_RSP - in case if there some trouble with the 
*                                requested attribute on the peer device,
*                                the details are provided with event parameters 
*                                structure (cy_stc_ble_gatt_err_param_t).
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_PlxscGetCharacteristicValue(CYBLE_CONN_HANDLE_T connHandle, CYBLE_PLXS_CHAR_INDEX_T charIndex)
{
    CYBLE_API_RESULT_T apiResult;
    
    /* Check the parameters */
    if(CyBle_GetClientState() != CYBLE_CLIENT_STATE_DISCOVERED)
    {
        apiResult = CYBLE_ERROR_INVALID_STATE;
    }
    else if(charIndex >= CYBLE_PLXS_CHAR_COUNT)
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
	else if(cyBle_plxsc.charInfo[charIndex].valueHandle == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
		apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
	}
    else if((CYBLE_CHAR_PROP_READ & cyBle_plxsc.charInfo[charIndex].properties) == 0u)
    {
        apiResult = CYBLE_ERROR_INVALID_OPERATION;
    }
    else
    {
        apiResult = CyBle_GattcReadCharacteristicValue(connHandle, cyBle_plxsc.charInfo[charIndex].valueHandle);
        if(apiResult == CYBLE_ERROR_OK)
        {
            cyBle_plxscReqHandle = cyBle_plxsc.charInfo[charIndex].valueHandle;
        }
    }
    return (apiResult);
}

/******************************************************************************
* Function Name: CyBle_PlxscSetCharacteristicDescriptor
***************************************************************************//**
* 
*  Sets the Characteristic Descriptor of the specified Characteristic.
* 
*  Internally, Write Request is sent to the GATT Server and on successful 
*  execution of the request on the Server side the following events can be 
*  generated: 
*  * CYBLE_EVT_PLXSS_INDICATION_ENABLED 
*  * CYBLE_EVT_PLXSS_INDICATION_DISABLED
*  * CYBLE_EVT_PLXSS_NOTIFICATION_ENABLED 
*  * CYBLE_EVT_PLXSS_NOTIFICATION_DISABLED
*
*  \param connHandle: The connection handle.
*  \param charIndex: The index of a service characteristic.
*  \param descrIndex: The index of a service characteristic descriptor.
*  \param attrSize: The size of the characteristic descriptor value attribute.
*  \param attrValue: Pointer to the characteristic descriptor value data that should 
*               be sent to the server device.
* 
* \return
*  Return value is of type CYBLE_API_RESULT_T.
*  * CYBLE_ERROR_OK - The request was sent successfully
*  * CYBLE_ERROR_INVALID_PARAMETER - Validation of the input parameters failed
*  * CYBLE_ERROR_INVALID_STATE - The state is not valid
*  * CYBLE_ERROR_MEMORY_ALLOCATION_FAILED - Memory allocation failed
*  * CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE - The peer device doesn't have
*                                               the particular characteristic
*  * CYBLE_ERROR_INVALID_OPERATION - This operation is not permitted on 
*                                     the specified attribute
*
* \events
*  In case of successful execution (return value = CYBLE_ERROR_OK)
*  the next events can appear: \n
*   If the PLXS service-specific callback is registered 
*      (with CyBle_PlxsRegisterAttrCallback):
*  * CYBLE_EVT_PLXSC_WRITE_DESCR_RESPONSE - in case if the requested attribute is
*                                successfully wrote on the peer device,
*                                the details (char index, descr index etc.) are 
*                                provided with event parameter structure
*                                of type CYBLE_PLXS_DESCR_VALUE_T.
*  .
*   Otherwise (if the PLXS service-specific callback is not registered):
*  * CYBLE_EVT_GATTC_WRITE_RSP - in case if the requested attribute is 
*                                successfully wrote on the peer device.
*  * CYBLE_EVT_GATTC_ERROR_RSP - in case if there some trouble with the 
*                                requested attribute on the peer device,
*                                the details are provided with event parameters 
*                                structure (cy_stc_ble_gatt_err_param_t).
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_PlxscSetCharacteristicDescriptor(CYBLE_CONN_HANDLE_T connHandle, CYBLE_PLXS_CHAR_INDEX_T charIndex,
                                                            CYBLE_PLXS_DESCR_INDEX_T descrIndex,
                                                            uint8 attrSize, uint8 * attrValue)
{
    CYBLE_API_RESULT_T apiResult;
        
    /* Check the parameters */
    if(CyBle_GetClientState() != CYBLE_CLIENT_STATE_DISCOVERED)
    {
        apiResult = CYBLE_ERROR_INVALID_STATE;
    }
    else if((charIndex >= CYBLE_PLXS_CHAR_COUNT) || (descrIndex >= CYBLE_PLXS_DESCR_COUNT) || (attrSize != CYBLE_CCCD_LEN))
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
    else if(cyBle_plxsc.charInfo[charIndex].descrHandle[CYBLE_PLXS_CCCD] == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
		apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
	}
    else
    {
        /* Fill all fields of write request structure ... */
        CYBLE_GATTC_WRITE_REQ_T writeReqParam;

        writeReqParam.attrHandle = cyBle_plxsc.charInfo[charIndex].descrHandle[CYBLE_PLXS_CCCD];
        writeReqParam.value.val  = attrValue;
        writeReqParam.value.len  = CYBLE_CCCD_LEN;

        /* ... and send request to server device. */
        apiResult = CyBle_GattcWriteCharacteristicDescriptors(connHandle, &writeReqParam);
        if(apiResult == CYBLE_ERROR_OK)
        {
            /* Save handle to support service-specific read response from device */
            cyBle_plxscReqHandle = writeReqParam.attrHandle;
        }
    }
    return (apiResult);
}

/******************************************************************************
* Function Name: CyBle_PlxscGetCharacteristicDescriptor
***************************************************************************//**
* 
*  Gets the characteristic descriptor of the specified characteristic.
* 
*  \param connHandle: The connection handle.
*  \param charIndex: The index of a service characteristic.
*  \param descrIndex: The index of the service characteristic descriptor.
* 
* \return
*  Return value is of type CYBLE_API_RESULT_T.
*  * CYBLE_ERROR_OK - The request was sent successfully
*  * CYBLE_ERROR_INVALID_PARAMETER - Validation of the input parameters failed
*  * CYBLE_ERROR_INVALID_STATE - The state is not valid
*  * CYBLE_ERROR_MEMORY_ALLOCATION_FAILED - Memory allocation failed
*  * CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE - The peer device doesn't have
*                                               the particular descriptor
*  * CYBLE_ERROR_INVALID_OPERATION - This operation is not permitted on 
*                                     the specified attribute
*
* \events
*  In case of successful execution (return value = CYBLE_ERROR_OK)
*  the next events can appear: \n
*  If the PLXS service-specific callback is registered 
*      (with CyBle_PlxsRegisterAttrCallback):
*  * CYBLE_EVT_PLXSC_READ_DESCR_RESPONSE - in case if the requested attribute is
*                                successfully wrote on the peer device,
*                                the details (char index, descr index, value, etc.) 
*                                are provided with event parameter structure
*                                of type CYBLE_PLXS_DESCR_VALUE_T. 
*  .
*  Otherwise (if the PLXS service-specific callback is not registered):
*  * CYBLE_EVT_GATTC_READ_RSP - in case if the requested attribute is 
*                                successfully read on the peer device,
*                                the details (handle, value, etc.) are 
*                                provided with event parameters 
*                                structure (CYBLE_GATTC_READ_RSP_PARAM_T).
*  * CYBLE_EVT_GATTC_ERROR_RSP - in case if there some trouble with the 
*                                requested attribute on the peer device,
*                                the details are provided with event parameters 
*                                structure (cy_stc_ble_gatt_err_param_t).
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_PlxscGetCharacteristicDescriptor(CYBLE_CONN_HANDLE_T connHandle, 
                                                            CYBLE_PLXS_CHAR_INDEX_T charIndex,
                                                            CYBLE_PLXS_DESCR_INDEX_T descrIndex)
{
    CYBLE_API_RESULT_T apiResult;
    
    /* Check the parameters */
    if(CyBle_GetClientState() != CYBLE_CLIENT_STATE_DISCOVERED)
    {
        apiResult = CYBLE_ERROR_INVALID_STATE;
    }
    else if((charIndex >= CYBLE_PLXS_CHAR_COUNT) || (descrIndex >= CYBLE_PLXS_DESCR_COUNT))
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
    else if(cyBle_plxsc.charInfo[charIndex].descrHandle[CYBLE_PLXS_CCCD] == 
                CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
		apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
	}
    else
    {
        apiResult = CyBle_GattcReadCharacteristicDescriptors(connHandle,
                                    cyBle_plxsc.charInfo[charIndex].descrHandle[descrIndex]);

        if(apiResult == CYBLE_ERROR_OK)
        {
            cyBle_plxscReqHandle = cyBle_plxsc.charInfo[charIndex].descrHandle[CYBLE_PLXS_CCCD];
        }
    }
    return (apiResult);
}

/******************************************************************************
* Function Name: CyBle_PlxscDiscoverCharacteristicsEventHandler
***************************************************************************//**
* 
*  This function is called on receiving a CYBLE_EVT_GATTC_READ_BY_TYPE_RSP
*  event. Based on the service UUID, an appropriate data structure is populated
*  using the data received as part of the callback.
* 
*  \param discCharInfo: The pointer to a characteristic information structure.
* 
******************************************************************************/
void CyBle_PlxscDiscoverCharacteristicsEventHandler(CYBLE_DISC_CHAR_INFO_T *discCharInfo)
{
    /* PLX Service characteristics UUIDs */
    static const CYBLE_UUID16 cyBle_PlxsCharUuid[CYBLE_PLXS_CHAR_COUNT] =
    {
        CYBLE_UUID_CHAR_PLX_SPOT_CHK_MSRMT,
        CYBLE_UUID_CHAR_PLX_CONTINUOUS_MSRMT,
        CYBLE_UUID_CHAR_PLX_FEATURES,
        CYBLE_UUID_CHAR_RACP
    };
    static CYBLE_GATT_DB_ATTR_HANDLE_T *lastEndHandle = {NULL};
    uint32 i;

    if(discCharInfo->uuidFormat == CYBLE_GATT_16_BIT_UUID_FORMAT)
    {
        /* Update last characteristic endHandle to declaration handle of this characteristic */
        if(lastEndHandle != NULL)
        {
            *lastEndHandle = discCharInfo->charDeclHandle - 1u;
             lastEndHandle = NULL;
        }
        
        for(i = 0u; i < (uint32)CYBLE_PLXS_CHAR_COUNT; i++)
        {
            if(cyBle_PlxsCharUuid[i] == discCharInfo->uuid.uuid16)
            {
                if(cyBle_plxsc.charInfo[i].valueHandle == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
                {
                    cyBle_plxsc.charInfo[i].valueHandle = discCharInfo->valueHandle;
                    cyBle_plxsc.charInfo[i].properties = discCharInfo->properties;
                    lastEndHandle = &cyBle_plxsc.charInfo[i].endHandle;
                }
                else
                {
                    CyBle_ApplCallback((uint32)CYBLE_EVT_GATTC_CHAR_DUPLICATION, &discCharInfo);
                }
            }
        }
        
        /* Init characteristic endHandle to Service endHandle.
           Characteristic endHandle will be updated to the declaration
           Handler of the following characteristic,
           in the following characteristic discovery procedure. */
        if(lastEndHandle != NULL)
        {
            *lastEndHandle = cyBle_serverInfo[CYBLE_SRVI_PLXS].range.endHandle;
        }
    }
}

/******************************************************************************
* Function Name: CyBle_PlxscDiscoverCharDescriptorsEventHandler
***************************************************************************//**
* 
*  This function is called on receiving a CYBLE_EVT_GATTC_READ_BY_TYPE_RSP
*  event. Based on the service UUID, an appropriate data structure is populated
*  using the data received as part of the callback.
* 
*  \param discDescrInfo: The pointer to a descriptor information structure.
*  
******************************************************************************/
void CyBle_PlxscDiscoverCharDescriptorsEventHandler( CYBLE_DISC_DESCR_INFO_T *discDescrInfo)
{ 
    CYBLE_PLXS_DESCR_INDEX_T locDescIndex;

    if(discDescrInfo->uuid.uuid16 == CYBLE_UUID_CHAR_CLIENT_CONFIG)
    {
        locDescIndex = CYBLE_PLXS_CCCD;
    }
    else    /* Not supported descriptor */
    {
        locDescIndex = CYBLE_PLXS_DESCR_COUNT;
    }

    if(locDescIndex < CYBLE_PLXS_DESCR_COUNT)
    {
        if(cyBle_plxsc.charInfo[cyBle_disCount - CYBLE_SCDI_PLXS_SPOT_CHECK_MEASUREMENT].descrHandle[locDescIndex] == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
        {
            cyBle_plxsc.charInfo[cyBle_disCount - CYBLE_SCDI_PLXS_SPOT_CHECK_MEASUREMENT].descrHandle[locDescIndex] = discDescrInfo->descrHandle;
        }
        else    /* Duplication of descriptor */
        {
            CyBle_ApplCallback(CYBLE_EVT_GATTC_DESCR_DUPLICATION, &discDescrInfo->uuid);
        }
    }
}

/******************************************************************************
* Function Name: CyBle_PlxscNotificationEventHandler
***************************************************************************//**
* 
*  Handles the Notification Event.
* 
*  \param eventParam: The pointer to a data structure specified by the event.
* 
******************************************************************************/
void CyBle_PlxscNotificationEventHandler(CYBLE_GATTC_HANDLE_VALUE_NTF_PARAM_T *eventParam)
{
    if(CyBle_PlxsApplCallback != NULL)
    {
        CYBLE_PLXS_CHAR_VALUE_T locCharValue = { .connHandle = eventParam->connHandle };
        
        if(cyBle_plxsc.charInfo[CYBLE_PLXS_CTMT].valueHandle ==
            eventParam->handleValPair.attrHandle)
        {
            locCharValue.charIndex = CYBLE_PLXS_CTMT;
            cyBle_eventHandlerFlag &= (uint8) ~CYBLE_CALLBACK;
            
            if((cyBle_eventHandlerFlag & CYBLE_CALLBACK) == 0u)
            {
                locCharValue.value = &eventParam->handleValPair.value;
                CyBle_PlxsApplCallback((uint32)CYBLE_EVT_PLXSC_NOTIFICATION, &locCharValue);
            }
        }
        else
        {
            /* PLXS Service doesn't support any other notifications */
        }
    }
}

/******************************************************************************
* Function Name: CyBle_PlxscIndicationEventHandler
***************************************************************************//**
* 
*  Handles the Indication Event.
* 
*  \param eventParam:  the pointer to a data structure specified by the event.
*  
******************************************************************************/
void CyBle_PlxscIndicationEventHandler(CYBLE_GATTC_HANDLE_VALUE_IND_PARAM_T *eventParam)
{
    CYBLE_PLXS_CHAR_VALUE_T locCharValue = { .connHandle = eventParam->connHandle };
    
    if((CyBle_PlxsApplCallback != NULL))
    {
        if(cyBle_plxsc.charInfo[CYBLE_PLXS_SCMT].valueHandle == eventParam->handleValPair.attrHandle)
        {
            locCharValue.charIndex  = CYBLE_PLXS_SCMT;
            cyBle_eventHandlerFlag &= (uint8) ~CYBLE_CALLBACK;

            if((cyBle_plxscFlag & CYBLE_PLXS_FLAG_PROCESS) != 0u)
            {
                /* Restart RACP timeout timer during RACP procedure(Report stored records)*/               
                (void)CyBle_StopTimer();
                (void)CyBle_StartTimer(cyBle_plxscRacpTimeout);
            }
        }
        else if(cyBle_plxsc.charInfo[CYBLE_PLXS_RACP].valueHandle == eventParam->handleValPair.attrHandle)
        {
            locCharValue.charIndex  = CYBLE_PLXS_RACP;
            cyBle_eventHandlerFlag &= (uint8) ~CYBLE_CALLBACK;
            
            /* End of RACP procedure, so stop RACP timeout timer */               
            (void)CyBle_StopTimer();
            cyBle_plxscFlag &= ~CYBLE_PLXS_FLAG_PROCESS;
        }
        else
        {
            /* PLXS Service doesn't support any other indications */
        }
     
        if((cyBle_eventHandlerFlag & CYBLE_CALLBACK) == 0u)
        {
            locCharValue.value = &eventParam->handleValPair.value;
            CyBle_PlxsApplCallback((uint32)CYBLE_EVT_PLXSC_INDICATION, &locCharValue);
        }
    }
}

/******************************************************************************
* Function Name: CyBle_PlxscReadResponseEventHandler
***************************************************************************//**
* 
*  Handles the Read Response Event.
* 
*  \param eventParam: The pointer to the data structure.
*  
******************************************************************************/
void CyBle_PlxscReadResponseEventHandler(CYBLE_GATTC_READ_RSP_PARAM_T *eventParam)
{
   
    if((CyBle_PlxsApplCallback != NULL) && 
       (cyBle_plxscReqHandle != CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE))
    {
        CYBLE_PLXS_CHAR_INDEX_T charIdx;
        uint32 fFlag = 0u;
        
        for(charIdx = (CYBLE_PLXS_CHAR_INDEX_T)0u; (charIdx < CYBLE_PLXS_CHAR_COUNT) && (fFlag == 0u); charIdx++)
        {
            if(cyBle_plxsc.charInfo[charIdx].valueHandle == cyBle_plxscReqHandle)
            {
                CYBLE_PLXS_CHAR_VALUE_T locCharValue =
                {
                    .connHandle = eventParam->connHandle,
                    .charIndex  = charIdx,
                    .value      = &eventParam->value
                };
                cyBle_plxscReqHandle = CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE;
                cyBle_eventHandlerFlag &= (uint8)~CYBLE_CALLBACK;
                CyBle_PlxsApplCallback((uint32)CYBLE_EVT_PLXSC_READ_CHAR_RESPONSE, &locCharValue);
                fFlag = 1u; /* instead of break */
            }
            else if(cyBle_plxsc.charInfo[charIdx].descrHandle[CYBLE_PLXS_CCCD] == cyBle_plxscReqHandle)
            {
                CYBLE_PLXS_DESCR_VALUE_T locDescrValue =
                {
                    .connHandle = eventParam->connHandle,
                    .charIndex  = charIdx,
                    .descrIndex = CYBLE_PLXS_CCCD,
                    .value      = &eventParam->value
                };
                cyBle_plxscReqHandle = CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE;
                cyBle_eventHandlerFlag &= (uint8)~CYBLE_CALLBACK;
                CyBle_PlxsApplCallback((uint32)CYBLE_EVT_PLXSC_READ_DESCR_RESPONSE, &locDescrValue);
                fFlag = 1u; /* instead of break */
            }
            else
            {
                /* nothing else */
            }
        }
    }
}

/******************************************************************************
* Function Name: CyBle_PlxscWriteResponseEventHandler
***************************************************************************//**
* 
*  Handles the Write Response Event.
* 
*  \param eventParam: The pointer to a data structure specified by the event.
*  
******************************************************************************/
void CyBle_PlxscWriteResponseEventHandler(const CYBLE_CONN_HANDLE_T *eventParam)
{
    if((CyBle_PlxsApplCallback != NULL) && 
       (cyBle_plxscReqHandle != CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE))
    {
        CYBLE_PLXS_CHAR_INDEX_T charIdx;
        uint32 fFlag = 0u;
        
        for(charIdx = (CYBLE_PLXS_CHAR_INDEX_T)0u; (charIdx < CYBLE_PLXS_CHAR_COUNT) && (fFlag == 0u); charIdx++)
        {
            if(cyBle_plxsc.charInfo[charIdx].valueHandle == cyBle_plxscReqHandle)
            {
                CYBLE_PLXS_CHAR_VALUE_T locCharValue =
                {
                   .connHandle = *eventParam,
                   .charIndex = charIdx,
                   .value = NULL
                };
                cyBle_plxscReqHandle = CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE;
                cyBle_eventHandlerFlag &= (uint8)~CYBLE_CALLBACK;
                CyBle_PlxsApplCallback((uint32)CYBLE_EVT_PLXSC_WRITE_CHAR_RESPONSE, &locCharValue);
                fFlag = 1u; /* instead of break */
            }
            else if(cyBle_plxsc.charInfo[charIdx].descrHandle[CYBLE_PLXS_CCCD] == cyBle_plxscReqHandle)
            {
                CYBLE_PLXS_DESCR_VALUE_T locDescrValue =
                {
                   .connHandle = *eventParam,
                   .charIndex  = charIdx,
                   .descrIndex = CYBLE_PLXS_CCCD,
                   .value = NULL
                };
                cyBle_plxscReqHandle = CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE;
                cyBle_eventHandlerFlag &= (uint8)~CYBLE_CALLBACK;
                CyBle_PlxsApplCallback((uint32)CYBLE_EVT_PLXSC_WRITE_DESCR_RESPONSE, &locDescrValue);
                fFlag = 1u; /* instead of break */
            }
            else
            {
                /* nothing else */
            }
        }
    }
}

/******************************************************************************
* Function Name: CyBle_PlxscErrorResponseEventHandler
***************************************************************************//**
* 
*  Handles the Error Response Event.
* 
*  \param eventParam: Pointer to the data structure specified by the event.
*  
******************************************************************************/
void CyBle_PlxscErrorResponseEventHandler(const CYBLE_GATTC_ERR_RSP_PARAM_T *eventParam)
{
    if(eventParam != NULL) 
    {
        if(eventParam->attrHandle == cyBle_plxscReqHandle)
        {
            cyBle_plxscReqHandle = CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE;
        }
    }
}

#endif /* CYBLE_PLXS_CLIENT */

#endif /* defined(CYBLE_PLXS) */

/* [] END OF FILE */
