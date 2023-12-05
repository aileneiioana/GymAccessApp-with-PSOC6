/***************************************************************************//**
* \file CYBLE_gls.c
* \version `$CY_MAJOR_VERSION`.`$CY_MINOR_VERSION`
* 
* \brief
*  This file contains the source code for the Glucose Service of the BLE Component.
* 
********************************************************************************
* \copyright
* Copyright 2014-2018, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/


#include "`$INSTANCE_NAME`_eventHandler.h"

static CYBLE_CALLBACK_T CyBle_GlsApplCallback;

#ifdef CYBLE_GLS_SERVER

`$GlsServer`
    
    uint8 cyBle_glsFlag;
    
static CYBLE_GATT_DB_ATTR_HANDLE_T cyBle_glssReqHandle;

#endif /* CYBLE_GLS_SERVER */

#ifdef CYBLE_GLS_CLIENT
    
/* Server's Glucose Service characteristics GATT DB handles structure */
CYBLE_GLSC_T cyBle_glsc;

/* Internal storage for last request handle to check the response */
static CYBLE_GATT_DB_ATTR_HANDLE_T cyBle_glscReqHandle;
#endif /* CYBLE_GLS_CLIENT */


/****************************************************************************** 
* Function Name: CyBle_GlsInit
***************************************************************************//**
* 
*  Initializes the profile internals.
*  
******************************************************************************/
void CyBle_GlsInit(void)
{
#ifdef CYBLE_GLS_SERVER
    cyBle_glsFlag = 0u;
    cyBle_glssReqHandle = CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE;
#endif /* CYBLE_GLS_SERVER */
#ifdef CYBLE_GLS_CLIENT
    if(cyBle_serverInfo[CYBLE_SRVI_GLS].range.startHandle == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
        (void)memset(&cyBle_glsc, 0u, sizeof(cyBle_glsc));
    }
    cyBle_glscReqHandle = CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE;
#endif /* CYBLE_GATT_ROLE_CLIENT */
}


/******************************************************************************
* Function Name: CyBle_GlsRegisterAttrCallback
***************************************************************************//**
* 
*  Registers a callback function for service specific attribute operations.
*  Service specific write requests from peer device will not be handled with
*  unregistered callback function.
* 
*  \param callbackFunc:  An application layer event callback function to receive 
*                    events from the BLE Component. The definition of 
*                    CYBLE_CALLBACK_T for Glucose Service is:\n
*                    typedef void (* CYBLE_CALLBACK_T) (uint32 eventCode, 
*                                                       void *eventParam)
*                    * eventCode indicates the event that triggered this 
*                      callback.
*                    * eventParam contains the parameters corresponding to the 
*                      current event.
* 
* \sideeffect The *eventParams in the callback function should not be used 
*                by the application once the callback function execution is 
*                finished. Otherwise this data may become corrupted.
* 
******************************************************************************/
void CyBle_GlsRegisterAttrCallback(CYBLE_CALLBACK_T callbackFunc)
{
    CyBle_GlsApplCallback = callbackFunc;
}


#ifdef CYBLE_GLS_SERVER


/******************************************************************************
* Function Name: CyBle_GlssSetCharacteristicValue
***************************************************************************//**
* 
*  Sets a characteristic value of the service, which is identified by charIndex.
* 
*  \param charIndex:  The index of a service characteristic.
*  \param attrSize:   The size of the characteristic value attribute.
*  \param *attrValue: The pointer to the characteristic value data that should 
*                     be stored in the GATT database.
* 
* \return
*  Return value is of type CYBLE_API_RESULT_T.
*  * CYBLE_ERROR_OK - The request handled successfully.
*  * CYBLE_ERROR_INVALID_PARAMETER - Validation of the input parameter failed.
*  * CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE - Optional characteristic is 
*                                              absent.
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_GlssSetCharacteristicValue(CYBLE_GLS_CHAR_INDEX_T charIndex, uint8 attrSize, uint8 *attrValue)
{
    CYBLE_API_RESULT_T apiResult;
    
    /* Check the parameters */
    if(charIndex >= CYBLE_GLS_CHAR_COUNT) 
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
    else if(cyBle_glss.charInfo[charIndex].charHandle == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
        apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
    }
    else
    {
        CYBLE_GATT_HANDLE_VALUE_PAIR_T locHandleValuePair;
        
        /* Store characteristic value into GATT database */
        locHandleValuePair.attrHandle = cyBle_glss.charInfo[charIndex].charHandle;
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
* Function Name: CyBle_GlssGetCharacteristicValue
***************************************************************************//**
* 
*  Gets a characteristic value of the service, which is identified by charIndex.
* 
*  \param charIndex:  The index of a service characteristic.
*  \param attrSize:   The size of the characteristic value attribute.
*  \param *attrValue: Pointer to the location where Characteristic value data  
*                     should be stored.
* 
* \return
*  Return value is of type CYBLE_API_RESULT_T.
*  * CYBLE_ERROR_OK - The request handled successfully.
*  * CYBLE_ERROR_INVALID_PARAMETER - Validation of the input parameter failed.
*  * CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE - Optional characteristic is 
*                                              absent.
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_GlssGetCharacteristicValue(CYBLE_GLS_CHAR_INDEX_T charIndex, uint8 attrSize, uint8 *attrValue)
{
    CYBLE_API_RESULT_T apiResult;
    
    /* Check the parameters */
    if(charIndex >= CYBLE_GLS_CHAR_COUNT)
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
    else if(cyBle_glss.charInfo[charIndex].charHandle == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
        apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
    }
    else
    {
        CYBLE_GATT_HANDLE_VALUE_PAIR_T locHandleValuePair;
        
        /* Get characteristic value from GATT database */
        locHandleValuePair.attrHandle = cyBle_glss.charInfo[charIndex].charHandle;
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
* Function Name: CyBle_GlssGetCharacteristicDescriptor
***************************************************************************//**
* 
*  Gets the characteristic descriptor of the specified characteristic.
* 
*  \param charIndex:  The index of the characteristic.
*  \param descrIndex: The index of the descriptor.
*  \param attrSize:   The size of the descriptor value attribute.
*  \param *attrValue: Pointer to the location where the descriptor value 
*                     data should be stored.
* 
* \return
*  Return value is of type CYBLE_API_RESULT_T.
*  * CYBLE_ERROR_OK - The request handled successfully.
*  * CYBLE_ERROR_INVALID_PARAMETER - Validation of the input parameter failed.
*  * CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE - Optional descriptor is absent.
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_GlssGetCharacteristicDescriptor(CYBLE_GLS_CHAR_INDEX_T charIndex, 
                                                         CYBLE_GLS_DESCR_INDEX_T descrIndex,
                                                         uint8 attrSize, uint8 *attrValue)
{
    CYBLE_API_RESULT_T apiResult;
    
    /* Check the parameters */
    if((charIndex >= CYBLE_GLS_CHAR_COUNT) || (descrIndex >= CYBLE_GLS_DESCR_COUNT))
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
    else if(cyBle_glss.charInfo[charIndex].cccdHandle == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
        apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
    }
    else
    {
        CYBLE_GATT_HANDLE_VALUE_PAIR_T locHandleValuePair;
        
        /* Get data from database */
        locHandleValuePair.attrHandle = cyBle_glss.charInfo[charIndex].cccdHandle;
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
* Function Name: CyBle_GlssSendNotification
***************************************************************************//**
* 
*  Sends a notification of the specified characteristic to the client device, 
*  as defined by the charIndex value.
* 
*  On enabling notification successfully for a service characteristic it sends
*  out a 'Handle Value Notification' which results in 
*  CYBLE_EVT_GLSC_NOTIFICATION event at the GATT Client's end.
*
*  \param connHandle: The connection handle which consist of the device ID and  
*                     ATT connection ID.
*  \param charIndex:  The index of the service characteristic.
*  \param attrSize:   The size of the characteristic value attribute.
*  \param *attrValue: Pointer to the Characteristic value data that should be 
*                     sent to Client device.
* 
* \return
*  Return value is of type CYBLE_API_RESULT_T.
*   * CYBLE_ERROR_OK - The request handled successfully.
*   * CYBLE_ERROR_INVALID_PARAMETER - Validation of the input parameter failed.
*   * CYBLE_ERROR_INVALID_OPERATION - Operation is invalid for this
*                                     characteristic.
*   * CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE - Optional characteristic is
*                                               absent.
*   * CYBLE_ERROR_INVALID_STATE - Connection with the client is not 
*                                 established.
*   * CYBLE_ERROR_MEMORY_ALLOCATION_FAILED - Memory allocation failed.
*   * CYBLE_ERROR_NTF_DISABLED - Notification is not enabled by the client.
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_GlssSendNotification(CYBLE_CONN_HANDLE_T connHandle,
                    CYBLE_GLS_CHAR_INDEX_T charIndex, uint8 attrSize, uint8 *attrValue)
{
    CYBLE_API_RESULT_T apiResult;
    
    /* Send Notification if it is enabled and connected */
    if(CYBLE_STATE_CONNECTED != CyBle_GetState())
    {
        apiResult = CYBLE_ERROR_INVALID_STATE;
    }
    else if(charIndex >= CYBLE_GLS_CHAR_COUNT)
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
    else if(cyBle_glss.charInfo[charIndex].charHandle == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
        apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
    }
    else if(!CYBLE_IS_NOTIFICATION_ENABLED(cyBle_glss.charInfo[charIndex].cccdHandle))
    {
        apiResult = CYBLE_ERROR_NTF_DISABLED;
    }
    else
    {
        CYBLE_GATTS_HANDLE_VALUE_NTF_T ntfParam;
        
        /* Fill all fields of write request structure ... */
        ntfParam.attrHandle = cyBle_glss.charInfo[charIndex].charHandle; 
        ntfParam.value.val = attrValue;
        ntfParam.value.len = attrSize;
        
        /* Send notification to client using previously filled structure */
        apiResult = CyBle_GattsNotification(connHandle, &ntfParam);
    }

    return (apiResult);
}


/******************************************************************************
* Function Name: CyBle_GlssSendIndication
***************************************************************************//**
* 
*  Sends a indication of the specified characteristic to the client device, as
*  defined by the charIndex value.
* 
*  On enabling indication successfully it sends out a 'Handle Value Indication'
*  which results in CYBLE_EVT_GLSC_INDICATION or  
*  CYBLE_EVT_GATTC_HANDLE_VALUE_IND (if service specific callback function is 
*  not registered) event at the GATT Client's end.
*
*  \param connHandle: The connection handle which consist of the device ID and 
*                     ATT connection ID.
*  \param charIndex:  The index of the service characteristic.
*  \param attrSize:   The size of the characteristic value attribute.
*  \param *attrValue: Pointer to the Characteristic value data that should be 
*                     sent to Client device.
* 
* \return
*  Return value is of type CYBLE_API_RESULT_T.
*   * CYBLE_ERROR_OK - The request handled successfully.
*   * CYBLE_ERROR_INVALID_PARAMETER - Validation of the input parameter failed.
*   * CYBLE_ERROR_INVALID_OPERATION - Operation is invalid for this 
*                                     characteristic.
*   * CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE - Optional characteristic is 
*                                               absent.
*   * CYBLE_ERROR_INVALID_STATE - Connection with the client is not 
*                                 established.
*   * CYBLE_ERROR_MEMORY_ALLOCATION_FAILED - Memory allocation failed.
*   * CYBLE_ERROR_IND_DISABLED - Indication is not enabled by the client.
*
* \events
*  In case of successful execution (return value = CYBLE_ERROR_OK)
*  the next events can appear: \n
*   If the GLS service-specific callback is registered 
*      (with CyBle_GlsRegisterAttrCallback):
*  * CYBLE_EVT_GLSS_INDICATION_CONFIRMED - In case if the indication is
*                                successfully delivered to the peer device.
*  .
*   Otherwise (if the GLS service-specific callback is not registered):
*  * CYBLE_EVT_GATTS_HANDLE_VALUE_CNF - In case if the indication is
*                                successfully delivered to the peer device.
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_GlssSendIndication(CYBLE_CONN_HANDLE_T connHandle,
                    CYBLE_GLS_CHAR_INDEX_T charIndex, uint8 attrSize, uint8 *attrValue)
{
    CYBLE_API_RESULT_T apiResult;
    
    /* Send Indication if it is enabled and connected */
    if(CYBLE_STATE_CONNECTED != CyBle_GetState())
    {
        apiResult = CYBLE_ERROR_INVALID_STATE;
    }
    else if(charIndex >= CYBLE_GLS_CHAR_COUNT)
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
    else if(cyBle_glss.charInfo[charIndex].cccdHandle == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
        apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
    }
    else if(!CYBLE_IS_INDICATION_ENABLED(cyBle_glss.charInfo[charIndex].cccdHandle))
    {
        apiResult = CYBLE_ERROR_IND_DISABLED;
    }
    else
    {
        CYBLE_GATTS_HANDLE_VALUE_IND_T indParam;
        
        /* Fill all fields of write request structure ... */
        indParam.attrHandle = cyBle_glss.charInfo[charIndex].charHandle; 
        indParam.value.val = attrValue;
        indParam.value.len = attrSize;
        
        /* Send notification to client using previously filled structure */
        apiResult = CyBle_GattsIndication(connHandle, &indParam);
        
        if(apiResult == CYBLE_ERROR_OK)
        {
            /* Save handle to support service specific value confirmation response from client */
            cyBle_glssReqHandle = indParam.attrHandle;
        }
    }

    return (apiResult);
}


/******************************************************************************
* Function Name: CyBle_GlssConfirmationEventHandler
***************************************************************************//**
* 
*  Handles the Value Confirmation request event from the BLE stack.
* 
*  \param *eventParam: Pointer to a structure of type CYBLE_CONN_HANDLE_T.
*  
******************************************************************************/
void CyBle_GlssConfirmationEventHandler(const CYBLE_CONN_HANDLE_T *eventParam)
{
    if((NULL != CyBle_GlsApplCallback) 
        && (cyBle_glss.charInfo[CYBLE_GLS_RACP].charHandle == cyBle_glssReqHandle))
    {   
        CYBLE_GLS_CHAR_VALUE_T locCharIndex;
        
        cyBle_glssReqHandle = CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE;
        
        cyBle_glsFlag &= (uint8) ~CYBLE_GLS_FLAG_PROCESS;
        
        locCharIndex.connHandle = *eventParam;
        locCharIndex.charIndex = CYBLE_GLS_RACP;
        locCharIndex.value = NULL;
        
        cyBle_eventHandlerFlag &= (uint8)~CYBLE_CALLBACK;
        CyBle_GlsApplCallback((uint32)CYBLE_EVT_GLSS_INDICATION_CONFIRMED, &locCharIndex);
    }
}


/******************************************************************************
* Function Name: CyBle_GlssWriteEventHandler
***************************************************************************//**
* 
*  Handles the Write Request Event.
* 
*  \param *eventParam: The pointer to the data structure specified by the event.
* 
* \return
*  CYBLE_GATT_ERR_CODE_T - A function result state if the it succeeded 
*  (CYBLE_GATT_ERR_NONE) or failed with error codes:
*   * CYBLE_GATTS_ERR_PROCEDURE_ALREADY_IN_PROGRESS.
*   * CYBLE_GATTS_ERR_CCCD_IMPROPERLY_CONFIGURED.
* 
******************************************************************************/
CYBLE_GATT_ERR_CODE_T CyBle_GlssWriteEventHandler(CYBLE_GATTS_WRITE_REQ_PARAM_T *eventParam)
{
    CYBLE_GATT_ERR_CODE_T gattErr = CYBLE_GATT_ERR_NONE;
    CYBLE_GLS_CHAR_VALUE_T locCharValue;
    
    if(NULL != CyBle_GlsApplCallback)
    {
	    locCharValue.connHandle = eventParam->connHandle;
	
	    if(eventParam->handleValPair.attrHandle == cyBle_glss.charInfo[CYBLE_GLS_RACP].charHandle)
	    {    
            if(CYBLE_GLS_IS_PROCEDURE_IN_PROGRESS
                && (CYBLE_GLS_RACP_OPCODE_ABORT != eventParam->handleValPair.value.val[0u]))
            {
                gattErr = CYBLE_GATTS_ERR_PROCEDURE_ALREADY_IN_PROGRESS;
            }
            else if(!CYBLE_IS_INDICATION_ENABLED(cyBle_glss.charInfo[CYBLE_GLS_RACP].cccdHandle))
            {
                gattErr = CYBLE_GATTS_ERR_CCCD_IMPROPERLY_CONFIGURED;
            }
            else
            {
                locCharValue.charIndex = CYBLE_GLS_RACP;
                locCharValue.value = &eventParam->handleValPair.value;
                gattErr = CyBle_GattsWriteAttributeValue(&eventParam->handleValPair, 
    	                      0u, &eventParam->connHandle, CYBLE_GATT_DB_PEER_INITIATED);
    	        if(gattErr == CYBLE_GATT_ERR_NONE)
    	        {
    	            CyBle_GlsApplCallback((uint32)CYBLE_EVT_GLSS_WRITE_CHAR, &locCharValue);
                    cyBle_glsFlag |= CYBLE_GLS_FLAG_PROCESS;
    	        }
            }
			
			cyBle_eventHandlerFlag &= (uint8)~CYBLE_CALLBACK;
	    }
	    else
	    {    
	        for(locCharValue.charIndex = CYBLE_GLS_GLMT;
                    locCharValue.charIndex < CYBLE_GLS_CHAR_COUNT; locCharValue.charIndex++)
	        {
	            if(eventParam->handleValPair.attrHandle == cyBle_glss.charInfo[locCharValue.charIndex].cccdHandle)
	            {
	                locCharValue.value = NULL;
                    gattErr = CyBle_GattsWriteAttributeValue(&eventParam->handleValPair,
	                             0u, &eventParam->connHandle, CYBLE_GATT_DB_PEER_INITIATED);
	                if(gattErr == CYBLE_GATT_ERR_NONE)
	                {
                        if(locCharValue.charIndex < CYBLE_GLS_GLFT)
                        {
                            if(CYBLE_IS_NOTIFICATION_ENABLED_IN_PTR(eventParam->handleValPair.value.val))
                            {
                                CyBle_GlsApplCallback((uint32)CYBLE_EVT_GLSS_NOTIFICATION_ENABLED, &locCharValue);
                            }
                            else
                            {
                                CyBle_GlsApplCallback((uint32)CYBLE_EVT_GLSS_NOTIFICATION_DISABLED, &locCharValue);
                            }
                        }
                        else
                        {
                            if(CYBLE_IS_INDICATION_ENABLED_IN_PTR(eventParam->handleValPair.value.val))
                            {
                                CyBle_GlsApplCallback((uint32)CYBLE_EVT_GLSS_INDICATION_ENABLED, &locCharValue);
                            }
                            else
                            {
                                CyBle_GlsApplCallback((uint32)CYBLE_EVT_GLSS_INDICATION_DISABLED, &locCharValue);
                            }
                        }
                    }
                #if((CYBLE_GAP_ROLE_PERIPHERAL || CYBLE_GAP_ROLE_CENTRAL) && \
                    (CYBLE_BONDING_REQUIREMENT == CYBLE_BONDING_YES))
                    /* Set flag to store bonding data to flash */
                    if(cyBle_peerBonding == CYBLE_GAP_BONDING)
                    {
                        cyBle_pendingFlashWrite |= CYBLE_PENDING_CCCD_FLASH_WRITE_BIT;
                    }
                #endif /* (CYBLE_BONDING_REQUIREMENT == CYBLE_BONDING_YES) */
					    
                    cyBle_eventHandlerFlag &= (uint8)~CYBLE_CALLBACK;
                    break;
				}
            }
        }
    }

    return (gattErr);
}

#endif /* CYBLE_GLS_SERVER */

#ifdef CYBLE_GLS_CLIENT


/******************************************************************************
* Function Name: CyBle_GlscSetCharacteristicValue
***************************************************************************//**
* 
*  This function is used to write the characteristic (which is identified by
*  charIndex) value attribute in the server. As a result a Write Request is 
*  sent to the GATT Server and on successful execution of the request on the 
*  Server side the CYBLE_EVT_GLSS_WRITE_CHAR events is generated.
*  On successful request execution on the Server side the Write Response is 
*  sent to the Client.
* 
*  \param connHandle: The connection handle.
*  \param charIndex:  The index of a service characteristic.
*  \param attrSize:   The size of the characteristic value attribute.
*  \param *attrValue: The pointer to the characteristic value data that should 
*                     be sent to the server device.
* 
* \return
*  Return value is of type CYBLE_API_RESULT_T.
*  * CYBLE_ERROR_OK - The request was sent successfully.
*  * CYBLE_ERROR_INVALID_PARAMETER - Validation of the input parameters failed.
*  * CYBLE_ERROR_MEMORY_ALLOCATION_FAILED - Memory allocation failed.
*  * CYBLE_ERROR_INVALID_STATE - Connection with the server is not established.
*  * CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE - The peer device doesn't have
*                                              the particular characteristic.
*  * CYBLE_ERROR_INVALID_OPERATION - Operation is invalid for this
*                                    characteristic.
*
* \events
*  In case of successful execution (return value = CYBLE_ERROR_OK)
*  the next events can appear: \n
*   If the GLS service-specific callback is registered 
*      (with CyBle_GlsRegisterAttrCallback):
*  * CYBLE_EVT_GLSC_WRITE_CHAR_RESPONSE - In case if the requested attribute is
*                                successfully wrote on the peer device,
*                                the details (char index, etc.) are 
*                                provided with event parameter structure
*                                of type CYBLE_GLS_CHAR_VALUE_T.
*  .
*   Otherwise (if the GLS service-specific callback is not registered):
*  * CYBLE_EVT_GATTC_WRITE_RSP - In case if the requested attribute is 
*                                successfully wrote on the peer device.
*  * CYBLE_EVT_GATTC_ERROR_RSP - In case if there some trouble with the 
*                                requested attribute on the peer device,
*                                the details are provided with event parameters 
*                                structure (CYBLE_GATTC_ERR_RSP_PARAM_T).
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_GlscSetCharacteristicValue(CYBLE_CONN_HANDLE_T connHandle,
                                                        CYBLE_GLS_CHAR_INDEX_T charIndex,
                                                            uint8 attrSize, uint8 * attrValue)
{
    CYBLE_API_RESULT_T apiResult;
    
    /* Check the parameters */
    if(CyBle_GetClientState() != CYBLE_CLIENT_STATE_DISCOVERED)
    {
        apiResult = CYBLE_ERROR_INVALID_STATE;
    }
    else if(charIndex >= CYBLE_GLS_CHAR_COUNT)
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
	else if(cyBle_glsc.charInfo[charIndex].valueHandle == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
		apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
	}
    else if((CYBLE_CHAR_PROP_WRITE & cyBle_glsc.charInfo[charIndex].properties) == 0u)
    {
        apiResult = CYBLE_ERROR_INVALID_OPERATION;
    }
    else
    {
        CYBLE_GATTC_WRITE_REQ_T writeReqParam;
        
        writeReqParam.attrHandle = cyBle_glsc.charInfo[charIndex].valueHandle;
        writeReqParam.value.val = attrValue;
        writeReqParam.value.len = attrSize;

        apiResult = CyBle_GattcWriteCharacteristicValue(connHandle, &writeReqParam);
        if(apiResult == CYBLE_ERROR_OK)
        {
            cyBle_glscReqHandle = cyBle_glsc.charInfo[charIndex].valueHandle;
        }
    }

    return (apiResult);
}


/******************************************************************************
* Function Name: CyBle_GlscGetCharacteristicValue
***************************************************************************//**
* 
*  This function is used to read the characteristic Value from a server
*  which is identified by charIndex.
* 
*  \param connHandle: The connection handle.
*  \param charIndex:  The index of the service characteristic.
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
*
* \events
*  In case of successful execution (return value = CYBLE_ERROR_OK)
*  the next events can appear: \n
*   If the GLS service-specific callback is registered 
*      (with CyBle_GlsRegisterAttrCallback):
*  * CYBLE_EVT_GLSC_READ_CHAR_RESPONSE - In case if the requested attribute is
*                                successfully wrote on the peer device,
*                                the details (char index , value, etc.) are 
*                                provided with event parameter structure
*                                of type CYBLE_GLS_CHAR_VALUE_T.
*  .
*   Otherwise (if the GLS service-specific callback is not registered):
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
CYBLE_API_RESULT_T CyBle_GlscGetCharacteristicValue(CYBLE_CONN_HANDLE_T connHandle, CYBLE_GLS_CHAR_INDEX_T charIndex)
{
    CYBLE_API_RESULT_T apiResult;

    /* Check the parameters */
    if(CyBle_GetClientState() != CYBLE_CLIENT_STATE_DISCOVERED)
    {
        apiResult = CYBLE_ERROR_INVALID_STATE;
    }
    else if(charIndex >= CYBLE_GLS_CHAR_COUNT)
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
	else if(cyBle_glsc.charInfo[charIndex].valueHandle == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
		apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
	}
    else if((CYBLE_CHAR_PROP_READ & cyBle_glsc.charInfo[charIndex].properties) == 0u)
    {
        apiResult = CYBLE_ERROR_INVALID_OPERATION;
    }
    else
    {
        apiResult = CyBle_GattcReadCharacteristicValue(connHandle, cyBle_glsc.charInfo[charIndex].valueHandle);
        if(apiResult == CYBLE_ERROR_OK)
        {
            cyBle_glscReqHandle = cyBle_glsc.charInfo[charIndex].valueHandle;
        }
    }

    return (apiResult);
}


/******************************************************************************
* Function Name: CyBle_GlscSetCharacteristicDescriptor
***************************************************************************//**
* 
*  Sets the Characteristic Descriptor of the specified Characteristic.
* 
*  Internally, Write Request is sent to the GATT Server and on successful 
*  execution of the request on the Server side the following events can be 
*  generated: 
*  * CYBLE_EVT_GLSS_INDICATION_ENABLED.
*  * CYBLE_EVT_GLSS_INDICATION_DISABLED.
*  * CYBLE_EVT_GLSS_NOTIFICATION_ENABLED. 
*  * CYBLE_EVT_GLSS_NOTIFICATION_DISABLED.
*
*  \param connHandle: The connection handle.
*  \param charIndex:  The index of a service characteristic.
*  \param descrIndex: The index of a service characteristic descriptor.
*  \param attrSize:   The size of the characteristic descriptor value 
*                     attribute.
*  \param *attrValue: Pointer to the characteristic descriptor value data that
*                     should be sent to the server device.
* 
* \return
*  Return value is of type CYBLE_API_RESULT_T.
*  * CYBLE_ERROR_OK - The request was sent successfully.
*  * CYBLE_ERROR_INVALID_PARAMETER - Validation of the input parameters failed.
*  * CYBLE_ERROR_INVALID_STATE - The state is not valid.
*  * CYBLE_ERROR_MEMORY_ALLOCATION_FAILED - Memory allocation failed.
*  * CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE - The peer device doesn't have
*                                              the particular characteristic.
*  * CYBLE_ERROR_INVALID_OPERATION - This operation is not permitted on 
*                                    the specified attribute.
*
* \events
*  In case of successful execution (return value = CYBLE_ERROR_OK)
*  the next events can appear: \n
*   If the GLS service-specific callback is registered 
*      (with CyBle_GlsRegisterAttrCallback):
*  * CYBLE_EVT_GLSC_WRITE_DESCR_RESPONSE - In case if the requested attribute is
*                                successfully wrote on the peer device,
*                                the details (char index, descr index etc.) are 
*                                provided with event parameter structure
*                                of type CYBLE_GLS_DESCR_VALUE_T.
*  .
*   Otherwise (if the GLS service-specific callback is not registered):
*  * CYBLE_EVT_GATTC_WRITE_RSP - In case if the requested attribute is 
*                                successfully wrote on the peer device.
*  * CYBLE_EVT_GATTC_ERROR_RSP - In case if there some trouble with the 
*                                requested attribute on the peer device,
*                                the details are provided with event parameters 
*                                structure (CYBLE_GATTC_ERR_RSP_PARAM_T).
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_GlscSetCharacteristicDescriptor(CYBLE_CONN_HANDLE_T connHandle, 
    CYBLE_GLS_CHAR_INDEX_T charIndex, CYBLE_GLS_DESCR_INDEX_T descrIndex, uint8 attrSize, uint8 * attrValue)
{
    CYBLE_API_RESULT_T apiResult;
    
    /* Check the parameters */
    if(CyBle_GetClientState() != CYBLE_CLIENT_STATE_DISCOVERED)
    {
        apiResult = CYBLE_ERROR_INVALID_STATE;
    }
    else if((charIndex >= CYBLE_GLS_CHAR_COUNT)
         || (descrIndex >= CYBLE_GLS_DESCR_COUNT)
         || (attrSize != CYBLE_CCCD_LEN))
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
    else if(cyBle_glsc.charInfo[charIndex].cccdHandle == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
		apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
	}
    else
    {
        CYBLE_GATTC_WRITE_REQ_T writeReqParam;
        
        /* Fill all fields of write request structure ... */
        writeReqParam.attrHandle = cyBle_glsc.charInfo[charIndex].cccdHandle;
        writeReqParam.value.val = attrValue;
        writeReqParam.value.len = CYBLE_CCCD_LEN;

        /* ... and send request to server device. */
        apiResult = CyBle_GattcWriteCharacteristicDescriptors(connHandle, &writeReqParam);
        if(apiResult == CYBLE_ERROR_OK)
        {
            /* Save handle to support service specific read response from device */
            cyBle_glscReqHandle = writeReqParam.attrHandle;
        }
    }

    return (apiResult);
}


/******************************************************************************
* Function Name: CyBle_GlscGetCharacteristicDescriptor
***************************************************************************//**
* 
*  Gets the characteristic descriptor of the specified characteristic.
* 
*  \param connHandle: The connection handle.
*  \param charIndex:  The index of a service characteristic.
*  \param descrIndex: The index of the service characteristic descriptor.
* 
* \return
*  Return value is of type CYBLE_API_RESULT_T.
*  * CYBLE_ERROR_OK - The request was sent successfully.
*  * CYBLE_ERROR_INVALID_PARAMETER - Validation of the input parameters failed.
*  * CYBLE_ERROR_INVALID_STATE - The state is not valid.
*  * CYBLE_ERROR_MEMORY_ALLOCATION_FAILED - Memory allocation failed.
*  * CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE - The peer device doesn't have
*                                              the particular descriptor.
*  * CYBLE_ERROR_INVALID_OPERATION - This operation is not permitted on 
*                                    the specified attribute.
*
* \events
*  In case of successful execution (return value = CYBLE_ERROR_OK)
*  the next events can appear: \n
*  If the GLS service-specific callback is registered 
*      (with CyBle_GlsRegisterAttrCallback):
*  * CYBLE_EVT_GLSC_READ_DESCR_RESPONSE - In case if the requested attribute is
*                                successfully wrote on the peer device,
*                                the details (char index, descr index, value, etc.) 
*                                are provided with event parameter structure
*                                of type CYBLE_GLS_DESCR_VALUE_T. 
*  .
*  Otherwise (if the GLS service-specific callback is not registered):
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
CYBLE_API_RESULT_T CyBle_GlscGetCharacteristicDescriptor(CYBLE_CONN_HANDLE_T connHandle,
                       CYBLE_GLS_CHAR_INDEX_T charIndex, CYBLE_GLS_DESCR_INDEX_T descrIndex)
{
    CYBLE_API_RESULT_T apiResult;
    
    /* Check the parameters */
    if(CyBle_GetClientState() != CYBLE_CLIENT_STATE_DISCOVERED)
    {
        apiResult = CYBLE_ERROR_INVALID_STATE;
    }
    else if((charIndex >= CYBLE_GLS_CHAR_COUNT) || (descrIndex >= CYBLE_GLS_DESCR_COUNT))
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
    else if(cyBle_glsc.charInfo[charIndex].cccdHandle == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
		apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
	}
    else
    {
        apiResult = CyBle_GattcReadCharacteristicDescriptors(connHandle, cyBle_glsc.charInfo[charIndex].cccdHandle);
        if(apiResult == CYBLE_ERROR_OK)
        {
            cyBle_glscReqHandle = cyBle_glsc.charInfo[charIndex].cccdHandle;
        }
    }
    
    return (apiResult);
}


/******************************************************************************
* Function Name: CyBle_GlscDiscoverCharacteristicsEventHandler
***************************************************************************//**
* 
*  This function is called on receiving a CYBLE_EVT_GATTC_READ_BY_TYPE_RSP
*  event. Based on the service UUID, an appropriate data structure is populated
*  using the data received as part of the callback.
* 
*  \param *discCharInfo: The pointer to a characteristic information structure.
* 
******************************************************************************/
void CyBle_GlscDiscoverCharacteristicsEventHandler(CYBLE_DISC_CHAR_INFO_T *discCharInfo)
{
    /* Glucose Service characteristics UUIDs */
    static const CYBLE_UUID16 cyBle_glscCharUuid[CYBLE_GLS_CHAR_COUNT] =
    {
        CYBLE_UUID_CHAR_GLUCOSE_MSRMNT,
        CYBLE_UUID_CHAR_GL_MSRMT_CTX,
        CYBLE_UUID_CHAR_GL_FEATURE,
        CYBLE_UUID_CHAR_RACP
    };
    static CYBLE_GATT_DB_ATTR_HANDLE_T *glsLastEndHandle = NULL;
    uint8 i;
    
    /* Update last characteristic endHandle to declaration handle of this characteristic */
    if(glsLastEndHandle != NULL)
    {
        *glsLastEndHandle = discCharInfo->charDeclHandle - 1u;
        glsLastEndHandle = NULL;
    }
    
    for(i = 0u; i < (uint8) CYBLE_GLS_CHAR_COUNT; i++)
    {
        if(cyBle_glscCharUuid[i] == discCharInfo->uuid.uuid16)
        {
            if(cyBle_glsc.charInfo[i].valueHandle == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
            {
                cyBle_glsc.charInfo[i].valueHandle = discCharInfo->valueHandle;
                cyBle_glsc.charInfo[i].properties = discCharInfo->properties;
                glsLastEndHandle = &cyBle_glsc.charInfo[i].endHandle;
            }
            else
            {
                CyBle_ApplCallback(CYBLE_EVT_GATTC_CHAR_DUPLICATION, &discCharInfo->uuid);
            }
        }
    }
    
    /* Init characteristic endHandle to Service endHandle.
       Characteristic endHandle will be updated to the declaration
       Handler of the following characteristic,
       in the following characteristic discovery procedure. */
    if(glsLastEndHandle != NULL)
    {
        *glsLastEndHandle = cyBle_serverInfo[CYBLE_SRVI_GLS].range.endHandle;
    }
}


/******************************************************************************
* Function Name: CyBle_GlscDiscoverCharDescriptorsEventHandler
***************************************************************************//**
* 
*  This function is called on receiving a CYBLE_EVT_GATTC_READ_BY_TYPE_RSP
*  event. Based on the service UUID, an appropriate data structure is populated
*  using the data received as part of the callback.
* 
*  \param *discCharInfo: The pointer to a characteristic information structure.
*  
******************************************************************************/
void CyBle_GlscDiscoverCharDescriptorsEventHandler(CYBLE_DISC_DESCR_INFO_T * discDescrInfo)
{
    if(discDescrInfo->uuid.uuid16 == CYBLE_UUID_CHAR_CLIENT_CONFIG)
    {
        uint8 descrIdx = cyBle_disCount - CYBLE_SCDI_GLS_GLMT;
        
        if(descrIdx == (CYBLE_SCDI_GLS_RACP - CYBLE_SCDI_GLS_GLMT))
        {
            descrIdx = CYBLE_GLS_RACP;
        }
        
        if(cyBle_glsc.charInfo[descrIdx].cccdHandle == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
        {
            cyBle_glsc.charInfo[descrIdx].cccdHandle = discDescrInfo->descrHandle;
        }
        else    /* Duplication of descriptor */
        {
            CyBle_ApplCallback(CYBLE_EVT_GATTC_DESCR_DUPLICATION, &discDescrInfo->uuid);
        }
    }
}


/******************************************************************************
* Function Name: CyBle_GlscNotificationEventHandler
***************************************************************************//**
* 
*  Handles the Notification Event.
* 
*  \param *eventParam: The pointer to a data structure specified by the event.
* 
******************************************************************************/
void CyBle_GlscNotificationEventHandler(CYBLE_GATTC_HANDLE_VALUE_NTF_PARAM_T *eventParam)
{
    if(NULL != CyBle_GlsApplCallback)
    {
        CYBLE_GLS_CHAR_VALUE_T locCharValue;
        
        if(cyBle_glsc.charInfo[CYBLE_GLS_GLMT].valueHandle == eventParam->handleValPair.attrHandle)
        {
            locCharValue.charIndex = CYBLE_GLS_GLMT;
            cyBle_eventHandlerFlag &= (uint8) ~CYBLE_CALLBACK;
        }
        else if(cyBle_glsc.charInfo[CYBLE_GLS_GLMC].valueHandle == eventParam->handleValPair.attrHandle)
        {
            locCharValue.charIndex = CYBLE_GLS_GLMC;
            cyBle_eventHandlerFlag &= (uint8) ~CYBLE_CALLBACK;
        }
        else
        {
            /* Glucose Service doesn't support any other notifications */
        }
        
        if((cyBle_eventHandlerFlag & CYBLE_CALLBACK) == 0u)
        {
            locCharValue.connHandle = eventParam->connHandle;
            
            locCharValue.value = &eventParam->handleValPair.value;

            CyBle_GlsApplCallback(CYBLE_EVT_GLSC_NOTIFICATION, &locCharValue);
        }
    }
}


/******************************************************************************
* Function Name: CyBle_GlscIndicationEventHandler
***************************************************************************//**
* 
*  Handles the Indication Event.
* 
*  \param *eventParam: The pointer to a data structure specified by the event.
*  
******************************************************************************/
void CyBle_GlscIndicationEventHandler(CYBLE_GATTC_HANDLE_VALUE_IND_PARAM_T *eventParam)
{
    if(NULL != CyBle_GlsApplCallback)
    {
        if(cyBle_glsc.charInfo[CYBLE_GLS_RACP].valueHandle == eventParam->handleValPair.attrHandle)
        {
            CYBLE_GLS_CHAR_VALUE_T locCharValue;
            locCharValue.charIndex = CYBLE_GLS_RACP;
            cyBle_eventHandlerFlag &= (uint8) ~CYBLE_CALLBACK;
            locCharValue.connHandle = eventParam->connHandle;       
            locCharValue.value = &eventParam->handleValPair.value;

            CyBle_GlsApplCallback(CYBLE_EVT_GLSC_INDICATION, &locCharValue);
        }
    }
}


/******************************************************************************
* Function Name: CyBle_GlscReadResponseEventHandler
***************************************************************************//**
* 
*  Handles the Read Response Event.
* 
*  \param *eventParam: The pointer to the data structure.
*  
******************************************************************************/
void CyBle_GlscReadResponseEventHandler(CYBLE_GATTC_READ_RSP_PARAM_T *eventParam)
{
    if((NULL != CyBle_GlsApplCallback) && (CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE != cyBle_glscReqHandle))
    {
        if(cyBle_glsc.charInfo[CYBLE_GLS_GLFT].valueHandle == cyBle_glscReqHandle)
        {
            CYBLE_GLS_CHAR_VALUE_T locCharValue;
            
            locCharValue.connHandle = eventParam->connHandle;
            locCharValue.charIndex = CYBLE_GLS_GLFT;
            locCharValue.value = &eventParam->value;
            
            cyBle_glscReqHandle = CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE;
            cyBle_eventHandlerFlag &= (uint8)~CYBLE_CALLBACK;
            CyBle_GlsApplCallback((uint32)CYBLE_EVT_GLSC_READ_CHAR_RESPONSE, &locCharValue);
        }
        else
        {
            CYBLE_GLS_CHAR_INDEX_T i;
            
            for(i = CYBLE_GLS_GLMT; i < CYBLE_GLS_CHAR_COUNT; i++)
            {
                if(cyBle_glsc.charInfo[i].cccdHandle == cyBle_glscReqHandle)
                {
                    CYBLE_GLS_DESCR_VALUE_T locDescrValue;
                    
                    locDescrValue.connHandle = eventParam->connHandle;
                    locDescrValue.charIndex = i;
                    locDescrValue.descrIndex = CYBLE_GLS_CCCD;
                    locDescrValue.value = &eventParam->value;

                    cyBle_glscReqHandle = CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE;
                    cyBle_eventHandlerFlag &= (uint8)~CYBLE_CALLBACK;
                    CyBle_GlsApplCallback((uint32)CYBLE_EVT_GLSC_READ_DESCR_RESPONSE, &locDescrValue);
                    break;
                }
            }
        }
    }
}


/******************************************************************************
* Function Name: CyBle_GlscWriteResponseEventHandler
***************************************************************************//**
* 
*  Handles the Write Response Event.
* 
*  \param *eventParam: The pointer to a data structure specified by the event.
*  
******************************************************************************/
void CyBle_GlscWriteResponseEventHandler(const CYBLE_CONN_HANDLE_T *eventParam)
{
    if((NULL != CyBle_GlsApplCallback) && (CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE != cyBle_glscReqHandle))
    {
        if(cyBle_glsc.charInfo[CYBLE_GLS_RACP].valueHandle == cyBle_glscReqHandle)
        {
            CYBLE_GLS_CHAR_VALUE_T locCharIndex;
            
            locCharIndex.connHandle = *eventParam;
            locCharIndex.charIndex = CYBLE_GLS_RACP;
            locCharIndex.value = NULL;
            
            cyBle_glscReqHandle = CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE;
            cyBle_eventHandlerFlag &= (uint8)~CYBLE_CALLBACK;
            CyBle_GlsApplCallback((uint32)CYBLE_EVT_GLSC_WRITE_CHAR_RESPONSE, &locCharIndex);
        }
        else
        {
            CYBLE_GLS_CHAR_INDEX_T i;
            
            for(i = CYBLE_GLS_GLMT; i < CYBLE_GLS_CHAR_COUNT; i++)
            {
                if(cyBle_glsc.charInfo[i].cccdHandle == cyBle_glscReqHandle)
                {
                    CYBLE_GLS_DESCR_VALUE_T locDescIndex;
                    
                    locDescIndex.connHandle = *eventParam;
                    locDescIndex.charIndex = i;
                    locDescIndex.descrIndex = CYBLE_GLS_CCCD;
                    locDescIndex.value = NULL;

                    cyBle_glscReqHandle = CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE;
                    cyBle_eventHandlerFlag &= (uint8)~CYBLE_CALLBACK;
                    CyBle_GlsApplCallback((uint32)CYBLE_EVT_GLSC_WRITE_DESCR_RESPONSE, &locDescIndex);
                    break;
                }
            }
        }
    }
}


/******************************************************************************
* Function Name: CyBle_GlscErrorResponseEventHandler
***************************************************************************//**
* 
*  Handles the Error Response Event.
* 
*  \param *eventParam: Pointer to the data structure specified by the event.
*  
******************************************************************************/
void CyBle_GlscErrorResponseEventHandler(const CYBLE_GATTC_ERR_RSP_PARAM_T *eventParam)
{
    if((eventParam != NULL) && (eventParam->attrHandle == cyBle_glscReqHandle))
    {
        cyBle_glscReqHandle = CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE;
    }
}


#endif /* CYBLE_GLS_CLIENT */


/* [] END OF FILE */
