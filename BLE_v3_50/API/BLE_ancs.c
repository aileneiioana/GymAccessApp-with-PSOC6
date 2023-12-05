/***************************************************************************//**
* \file CYBLE_ancs.c
* \version `$CY_MAJOR_VERSION`.`$CY_MINOR_VERSION`
* 
* \brief
*  This file contains the source code for the Apple Notification Center
*  Service of the BLE Component.
* 
********************************************************************************
* \copyright
* Copyright 2014-2018, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/


#include "`$INSTANCE_NAME`_eventHandler.h"

static CYBLE_CALLBACK_T CyBle_AncsApplCallback;

#if(CYBLE_GAP_ROLE_CENTRAL || CYBLE_GATT_ROLE_CLIENT)
/* Apple Notification Center Service UUID */
    const CYBLE_UUID128_T cyBle_ancscServUuid =
    {
        {0xD0u, 0x00u, 0x2Du, 0x12u, 0x1Eu, 0x4Bu, 0x0Fu, 0xA4u, 0x99u, 0x4Eu, 0xCEu, 0xB5u, 0x31u, 0xF4u, 0x05u, 0x79u}
    };    

    /* Apple Notification Center Service characteristics UUIDs */
    const CYBLE_UUID128_T cyBle_ancscCharUuid[CYBLE_ANCS_CHAR_COUNT] =
    {
        /* Notification Source characteristic UUID */
        {{0xBDu, 0x1Du, 0xA2u, 0x99u, 0xE6u, 0x25u, 0x58u, 0x8Cu, 0xD9u, 0x42u, 0x01u, 0x63u, 0x0Du, 0x12u, 0xBFu, 0x9Fu}},
        /* Control Point characteristic UUID */
        {{0xD9u, 0xD9u, 0xAAu, 0xFDu, 0xBDu, 0x9Bu, 0x21u, 0x98u, 0xA8u, 0x49u, 0xE1u, 0x45u, 0xF3u, 0xD8u, 0xD1u, 0x69u}},
        /* Data Source characteristic UUID */
        {{0xFBu, 0x7Bu, 0x7Cu, 0xCEu, 0x6Au, 0xB3u, 0x44u, 0xBEu, 0xB5u, 0x4Bu, 0xD6u, 0x24u, 0xE9u, 0xC6u, 0xEAu, 0x22u}}
    };
#endif /* CYBLE_GAP_ROLE_CENTRAL || CYBLE_GATT_ROLE_CLIENT */

#ifdef CYBLE_ANCS_SERVER

`$AncsServer`

#endif /* CYBLE_ANCS_SERVER */

#ifdef CYBLE_ANCS_CLIENT
    
/* Server's Apple Notification Center Service characteristics GATT DB handles structure */
CYBLE_ANCSC_T cyBle_ancsc;

/* Internal storage for last request handle to check response */
static CYBLE_GATT_DB_ATTR_HANDLE_T cyBle_ancscReqHandle;
#endif /* CYBLE_ANCS_CLIENT */


/******************************************************************************
* Function Name: CyBle_AncsInit
***************************************************************************//**
* 
*  Initializes the profile internals.
* 
******************************************************************************/
void CyBle_AncsInit(void)
{
#ifdef CYBLE_ANCS_CLIENT
    if(cyBle_serverInfo[CYBLE_SRVI_ANCS].range.startHandle == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
        (void)memset(&cyBle_ancsc, 0, sizeof(cyBle_ancsc));
    }
    cyBle_ancscReqHandle = CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE;
#endif /* CYBLE_ANCS_CLIENT */
}


/******************************************************************************
* Function Name: CyBle_AncsRegisterAttrCallback
***************************************************************************//**
* 
*  Registers a callback function for service-specific attribute operations.
*  Service-specific write requests from a peer device will not be handled with
*  an unregistered callback function.
* 
*  \param callbackFunc: An application layer event callback function to receive 
*                       events from the BLE Component. The definition of 
*                       CYBLE_CALLBACK_T for ANCS is:
*    typedef void (* CYBLE_CALLBACK_T) (uint32 eventCode, void *eventParam),
*    where:
*      * eventCode indicates The event that triggered this callback.
*      * eventParam contains The parameters corresponding to the current event.
*
*
* \sideeffect
*  The *eventParams in the callback function should not be used by the 
*  application once the callback function execution is finished. Otherwise this
*  data may become corrupted.
*  
******************************************************************************/
void CyBle_AncsRegisterAttrCallback(CYBLE_CALLBACK_T callbackFunc)
{
    CyBle_AncsApplCallback = callbackFunc;
}


#ifdef CYBLE_ANCS_SERVER


/******************************************************************************
* Function Name: CyBle_AncssSetCharacteristicValue
***************************************************************************//**
* 
*  Sets the value of the characteristic, as identified by charIndex.
* 
*  \param charIndex: The index of the service characteristic.
*  \param attrSize:  The size of the characteristic value attribute.
*  \param attrValue: The pointer to the characteristic value data that should 
*                    be stored to the GATT database.
* 
* \return
*  Return value is of type CYBLE_API_RESULT_T.
*  * CYBLE_ERROR_OK - The request handled successfully.
*  * CYBLE_ERROR_INVALID_PARAMETER - Validation of the input parameter failed.
*  * CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE - An optional characteristic is 
*                                              absent.
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_AncssSetCharacteristicValue(CYBLE_ANCS_CHAR_INDEX_T charIndex, uint8 attrSize, uint8 *attrValue)
{
    CYBLE_API_RESULT_T apiResult;
    
    /* Check parameters */
    if(charIndex >= CYBLE_ANCS_CHAR_COUNT)
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
	else if(cyBle_ancss.charInfo[charIndex].charHandle == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
        apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
    } 
    else
    {
        CYBLE_GATT_HANDLE_VALUE_PAIR_T locHandleValuePair;
        
        /* Store characteristic value into GATT database */
        locHandleValuePair.attrHandle = cyBle_ancss.charInfo[charIndex].charHandle;
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
* Function Name: CyBle_AncssGetCharacteristicValue
***************************************************************************//**
* 
*  Gets the value of the characteristic, as identified by charIndex.
* 
*  \param charIndex: The index of the service characteristic.
*  \param attrSize:  The size of the characteristic value attribute.
*  \param attrValue: The pointer to the location where characteristic value 
*                    data should be stored.
* 
* \return
*  A return value is of type CYBLE_API_RESULT_T.
*   * CYBLE_ERROR_OK - The characteristic value was read successfully
*   * CYBLE_ERROR_INVALID_PARAMETER - Validation of the input parameter failed.
*   * CYBLE_ERROR_INVALID_OPERATION - Operation is invalid for this characteristic.
*   * CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE - A characteristic is absent.
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_AncssGetCharacteristicValue(CYBLE_ANCS_CHAR_INDEX_T charIndex, uint8 attrSize, uint8 *attrValue)
{
    CYBLE_API_RESULT_T apiResult;
    
    /* Check parameters */
    if(charIndex >= CYBLE_ANCS_CHAR_COUNT)
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
	else if(cyBle_ancss.charInfo[charIndex].charHandle == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
        apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
    }
    else
    {
        CYBLE_GATT_HANDLE_VALUE_PAIR_T locHandleValuePair;
        
        /* Get characteristic value from GATT database */
        locHandleValuePair.attrHandle = cyBle_ancss.charInfo[charIndex].charHandle;
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
* Function Name: CyBle_AncssGetCharacteristicDescriptor
***************************************************************************//**
* 
*  Gets a characteristic descriptor of the specified characteristic.
* 
*  \param charIndex:  The index of the characteristic.
*  \param descrIndex: The index of the descriptor.
*  \param attrSize:   The size of the descriptor value attribute.
*  \param attrValue: The pointer to the location where characteristic
*                      descriptor value data should be stored. 
* 
* \return
*  A return value is of type CYBLE_API_RESULT_T.
*   * CYBLE_ERROR_OK - The Characteristic Descriptor value was read successfully
*   * CYBLE_ERROR_INVALID_PARAMETER - Validation of the input parameter failed.
*   * CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE - A characteristic is absent.
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_AncssGetCharacteristicDescriptor(CYBLE_ANCS_CHAR_INDEX_T charIndex, 
                                                         CYBLE_ANCS_DESCR_INDEX_T descrIndex,
                                                         uint8 attrSize, uint8 *attrValue)
{
    CYBLE_API_RESULT_T apiResult;
    
    /* Check parameters */
	if((charIndex >= CYBLE_ANCS_CHAR_COUNT) || (descrIndex >= CYBLE_ANCS_DESCR_COUNT))
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
    else if(cyBle_ancss.charInfo[charIndex].descrHandle[CYBLE_ANCS_CCCD] == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
        apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
    }
    else
    {
        CYBLE_GATT_HANDLE_VALUE_PAIR_T locHandleValuePair;
        
        /* Get data from database */
        locHandleValuePair.attrHandle = cyBle_ancss.charInfo[charIndex].descrHandle[descrIndex];
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
* Function Name: CyBle_AncssSendNotification
***************************************************************************//**
* 
*  Sends a notification of the specified characteristic value, as identified by
*  the charIndex. 
*  On enabling notification successfully for a service characteristic it sends 
*  out a 'Handle Value Notification' which results in 
*  CYBLE_EVT_ANCSC_NOTIFICATION event at the GATT Client's end.
* 
*  \param connHandle: The connection handle that consists of the device ID and ATT 
*                     connection ID.
*  \param charIndex:  The index of the service characteristic.
*  \param attrSize:   The size of the characteristic value attribute.
*  \param attrValue:  The pointer to the characteristic value data that should
*                     be sent to the client device.
* 
* \return
*  A return value is of type CYBLE_API_RESULT_T.
*   * CYBLE_ERROR_OK - The request handled successfully
*   * CYBLE_ERROR_INVALID_PARAMETER - Validation of the input parameter failed.
*   * CYBLE_ERROR_INVALID_OPERATION - Operation is invalid for this 
*                                     characteristic.
*   * CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE - An optional characteristic is
*                                               absent.
*   * CYBLE_ERROR_INVALID_STATE - Connection with the client is not 
*                                 established.
*   * CYBLE_ERROR_MEMORY_ALLOCATION_FAILED - Memory allocation failed.
*   * CYBLE_ERROR_NTF_DISABLED - Notification is not enabled by the client.
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_AncssSendNotification(CYBLE_CONN_HANDLE_T connHandle,
                    CYBLE_ANCS_CHAR_INDEX_T charIndex, uint8 attrSize, uint8 *attrValue)
{
    CYBLE_API_RESULT_T apiResult;
    
    /* Send Notification if it is enabled and connected */
    if(CYBLE_STATE_CONNECTED != CyBle_GetState())
    {
        apiResult = CYBLE_ERROR_INVALID_STATE;
    }
    else if(charIndex >= CYBLE_ANCS_CHAR_COUNT)
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
    else if(cyBle_ancss.charInfo[charIndex].descrHandle[CYBLE_ANCS_CCCD] == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
		apiResult = CYBLE_ERROR_INVALID_OPERATION;
	}
    else if(!CYBLE_IS_NOTIFICATION_ENABLED(cyBle_ancss.charInfo[charIndex].descrHandle[CYBLE_ANCS_CCCD]))
    {
        apiResult = CYBLE_ERROR_NTF_DISABLED;
    }
    else
    {
        CYBLE_GATTS_HANDLE_VALUE_NTF_T ntfParam;
        
        /* Fill all fields of write request structure ... */
        ntfParam.attrHandle = cyBle_ancss.charInfo[charIndex].charHandle; 
        ntfParam.value.val = attrValue;
        ntfParam.value.len = attrSize;
        
        /* Send notification to client using previously filled structure */
        apiResult = CyBle_GattsNotification(connHandle, &ntfParam);
    }

    return (apiResult);
}


/******************************************************************************
* Function Name: CyBle_AncssWriteEventHandler
***************************************************************************//**
* 
*  Handles the Write Request Event.
* 
*  \param eventParam: The pointer to the data structure specified by the event.
* 
* \return
*  CYBLE_GATT_ERR_CODE_T - A function return state if it succeeded.
*  (CYBLE_GATT_ERR_NONE) or failed with error codes:
*   * CYBLE_GATTS_ERR_PROCEDURE_ALREADY_IN_PROGRESS.
*   * CYBLE_GATTS_ERR_CCCD_IMPROPERLY_CONFIGURED.
* 
******************************************************************************/
CYBLE_GATT_ERR_CODE_T CyBle_AncssWriteEventHandler(CYBLE_GATTS_WRITE_REQ_PARAM_T *eventParam)
{
    CYBLE_GATT_ERR_CODE_T gattErr = CYBLE_GATT_ERR_NONE;
    
    if(NULL != CyBle_AncsApplCallback)
    {
	    CYBLE_ANCS_CHAR_VALUE_T locCharValue;
        locCharValue.charIndex = CYBLE_ANCS_CHAR_COUNT;
	
	    if(eventParam->handleValPair.attrHandle == cyBle_ancss.charInfo[CYBLE_ANCS_NS].descrHandle[CYBLE_ANCS_CCCD])
        {
            locCharValue.charIndex = CYBLE_ANCS_NS;
        }
        else if(eventParam->handleValPair.attrHandle == cyBle_ancss.charInfo[CYBLE_ANCS_DS].descrHandle[CYBLE_ANCS_CCCD])
        {
            locCharValue.charIndex = CYBLE_ANCS_DS;
        }
        else
        {
            /* Leave locCharValue.charIndex = CYBLE_ANCS_CHAR_COUNT */
        }
	    
        if(locCharValue.charIndex != CYBLE_ANCS_CHAR_COUNT)
        {
            locCharValue.connHandle = eventParam->connHandle;
            locCharValue.value = NULL;
            gattErr = CyBle_GattsWriteAttributeValue(&eventParam->handleValPair,
                         0u, &eventParam->connHandle, CYBLE_GATT_DB_PEER_INITIATED);
            
            if(gattErr == CYBLE_GATT_ERR_NONE)
            {
                uint32 eventCode;
   
                if(CYBLE_IS_NOTIFICATION_ENABLED_IN_PTR(eventParam->handleValPair.value.val))
                {
                    eventCode = (uint32) CYBLE_EVT_ANCSS_NOTIFICATION_ENABLED;
                }
                else
                {
                    eventCode = (uint32) CYBLE_EVT_ANCSS_NOTIFICATION_DISABLED;
                }

                CyBle_AncsApplCallback(eventCode, &locCharValue);
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
        }
	    else
        {
            if(eventParam->handleValPair.attrHandle == cyBle_ancss.charInfo[CYBLE_ANCS_CP].charHandle)
            {
                locCharValue.charIndex = CYBLE_ANCS_CP;
                locCharValue.value = &eventParam->handleValPair.value;
                gattErr = CyBle_GattsWriteAttributeValue(&eventParam->handleValPair, 
    	                      0u, &eventParam->connHandle, CYBLE_GATT_DB_PEER_INITIATED);
            
                if(gattErr == CYBLE_GATT_ERR_NONE)
    	        {
    	            CYBLE_GATT_DB_ATTR_SET_ATTR_GEN_LEN
                        (cyBle_ancss.charInfo[locCharValue.charIndex].charHandle,
                            locCharValue.value->len);
                    
                    CyBle_AncsApplCallback((uint32)CYBLE_EVT_ANCSS_WRITE_CHAR, &locCharValue);
                    
                    if((locCharValue.gattErrorCode == CYBLE_GATT_ERR_ANCS_UNKNOWN_COMMAND) ||
                       (locCharValue.gattErrorCode == CYBLE_GATT_ERR_ANCS_INVALID_COMMAND) ||
                       (locCharValue.gattErrorCode == CYBLE_GATT_ERR_ANCS_INVALID_PARAMETER) ||
                       (locCharValue.gattErrorCode == CYBLE_GATT_ERR_ANCS_ACTION_FAILED))
                    {
                        gattErr = locCharValue.gattErrorCode;
                    }
    	        }
                
                cyBle_eventHandlerFlag &= (uint8)~CYBLE_CALLBACK;
			}
        }
    }

    return (gattErr);
}

#endif /* CYBLE_ANCS_SERVER */

#ifdef CYBLE_ANCS_CLIENT


/******************************************************************************
* Function Name: CyBle_AncscSetCharacteristicValue
***************************************************************************//**
* 
*  This function is used to write the characteristic (which is identified by
*  charIndex) value attribute in the server. As a result a Write Request is 
*  sent to the GATT Server and on successful execution of the request on the 
*  Server side the CYBLE_EVT_ANCSS_WRITE_CHAR events is generated.
*  On successful request execution on the Server side the Write Response is 
*  sent to the Client.
* 
*  \param connHandle: The connection handle.
*  \param charIndex:  The index of the service characteristic.
*  \param attrSize:   The size of the characteristic value attribute.
*  \param attrValue:  The pointer to the characteristic value data that should 
*                     be sent to the server device.
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
* \events
*  In the case of successful execution (return value = CYBLE_ERROR_OK)
*  the next events can appear: \n
*   If the ANCS service-specific callback is registered 
*      (with CyBle_AncsRegisterAttrCallback):
*  * CYBLE_EVT_ANCSC_WRITE_CHAR_RESPONSE - If the requested attribute is
*                                successfully written on the peer device,
*                                the details (char index, etc.) are 
*                                provided with an event parameter structure
*                                of type CYBLE_ANCS_CHAR_VALUE_T.
*  .
*   Otherwise (if the ANCS service-specific callback is not registered):
*  * CYBLE_EVT_GATTC_WRITE_RSP - If the requested attribute is 
*                                successfully written on the peer device.
*  * CYBLE_EVT_GATTC_ERROR_RSP - If there some trouble with the 
*                                requested attribute on the peer device,
*                                the details are provided with an event parameter
*                                structure (CYBLE_GATTC_ERR_RSP_PARAM_T).
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_AncscSetCharacteristicValue(CYBLE_CONN_HANDLE_T connHandle,
                                                        CYBLE_ANCS_CHAR_INDEX_T charIndex,
                                                            uint8 attrSize, uint8 * attrValue)
{
    CYBLE_API_RESULT_T apiResult;
    
    /* Check parameters */
    if(CyBle_GetClientState() != CYBLE_CLIENT_STATE_DISCOVERED)
    {
        apiResult = CYBLE_ERROR_INVALID_STATE;
    }
    else if(charIndex >= CYBLE_ANCS_CHAR_COUNT)
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
	else if(cyBle_ancsc.charInfo[charIndex].valueHandle == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
		apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
	}
    else if((CYBLE_CHAR_PROP_WRITE & cyBle_ancsc.charInfo[charIndex].properties) == 0u)
    {
        apiResult = CYBLE_ERROR_INVALID_OPERATION;
    }
    else
    {
        CYBLE_GATTC_WRITE_REQ_T writeReqParam;
        
        writeReqParam.attrHandle = cyBle_ancsc.charInfo[charIndex].valueHandle;
        writeReqParam.value.val = attrValue;
        writeReqParam.value.len = attrSize;

        apiResult = CyBle_GattcWriteCharacteristicValue(connHandle, &writeReqParam);
        if(apiResult == CYBLE_ERROR_OK)
        {
            cyBle_ancscReqHandle = cyBle_ancsc.charInfo[charIndex].valueHandle;
        }
    }

    return (apiResult);
}


/******************************************************************************
* Function Name: CyBle_AncscSetCharacteristicDescriptor
***************************************************************************//**
* 
*  This function is used to write the characteristic Value to the server,
*  as identified by its charIndex.
*   
*  Internally, Write Request is sent to the GATT Server and on successful 
*  execution of the request on the Server side the following events can be 
*  generated: 
*  * CYBLE_EVT_ANCSS_NOTIFICATION_ENABLED.
*  * CYBLE_EVT_ANCSS_NOTIFICATION_DISABLED.
* 
*  \param connHandle: The connection handle.
*  \param charIndex:  The index of the service characteristic.
*  \param descrIndex: The index of the service characteristic descriptor.
*  \param attrSize:   The size of the characteristic descriptor value attribute.
*  \param attrValue:  The pointer to the characteristic descriptor value data that
*                     should be sent to the server device.
* 
* \return
*  A return value is of type CYBLE_API_RESULT_T.
*  * CYBLE_ERROR_OK - The request was sent successfully.
*  * CYBLE_ERROR_INVALID_PARAMETER - Validation of the input parameters failed.
*  * CYBLE_ERROR_INVALID_STATE - The state is not valid.
*  * CYBLE_ERROR_MEMORY_ALLOCATION_FAILED - Memory allocation failed.
*  * CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE - The peer device doesn't have
*                                              the particular characteristic.
*  * CYBLE_ERROR_INVALID_OPERATION - This operation is not permitted on 
*                                    the specified attribute
*
* \events
*  In case of successfull execution (return value = CYBLE_ERROR_OK)
*  the next events can appear: \n
*   If the ANCS service-specific callback is registered 
*      (with CyBle_AncsRegisterAttrCallback):
*  * CYBLE_EVT_ANCSC_WRITE_DESCR_RESPONSE - If the requested attribute is
*                                successfully written on the peer device,
*                                the details (char index, descr index etc.) are
*                                provided with an event parameter structure
*                                of type CYBLE_ANCS_DESCR_VALUE_T.
*  .
*   Otherwise (if the ANCS service-specific callback is not registered):
*  * CYBLE_EVT_GATTC_WRITE_RSP - If the requested attribute is 
*                                successfully written on the peer device.
*  * CYBLE_EVT_GATTC_ERROR_RSP - If there is some trouble with the 
*                                requested attribute on the peer device,
*                                the details are provided with an event parameter
*                                structure (CYBLE_GATTC_ERR_RSP_PARAM_T.
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_AncscSetCharacteristicDescriptor(CYBLE_CONN_HANDLE_T connHandle,
    CYBLE_ANCS_CHAR_INDEX_T charIndex, CYBLE_ANCS_DESCR_INDEX_T descrIndex, uint8 attrSize, uint8 * attrValue)
{
    CYBLE_API_RESULT_T apiResult;
    
    /* Check parameters */
    if(CyBle_GetClientState() != CYBLE_CLIENT_STATE_DISCOVERED)
    {
        apiResult = CYBLE_ERROR_INVALID_STATE;
    }
    else if((charIndex >= CYBLE_ANCS_CHAR_COUNT)
         || (descrIndex >= CYBLE_ANCS_DESCR_COUNT)
         || (attrSize != CYBLE_CCCD_LEN))
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
    else if(cyBle_ancsc.charInfo[charIndex].descrHandle[descrIndex] == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
		apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
	}
    else
    {
        CYBLE_GATTC_WRITE_REQ_T writeReqParam;
        
        /* Fill all fields of write request structure ... */
        writeReqParam.attrHandle = cyBle_ancsc.charInfo[charIndex].descrHandle[descrIndex];
        writeReqParam.value.val = attrValue;
        writeReqParam.value.len = CYBLE_CCCD_LEN;

        /* ... and send request to server device. */
        apiResult = CyBle_GattcWriteCharacteristicDescriptors(connHandle, &writeReqParam);
        if(apiResult == CYBLE_ERROR_OK)
        {
            /* Save handle to support service specific read response from device */
            cyBle_ancscReqHandle = writeReqParam.attrHandle;
        }
    }

    return (apiResult);
}


/******************************************************************************
* Function Name: CyBle_AncscGetCharacteristicDescriptor
***************************************************************************//**
* 
*  Gets the characteristic descriptor of the specified characteristic.
* 
*  \param connHandle: The connection handle.
*  \param charIndex:  The index of the service characteristic.
*  \param descrIndex: The index of the service characteristic descriptor.
* 
* \return
*  A return value is of type CYBLE_API_RESULT_T.
*  * CYBLE_ERROR_OK - The request was sent successfully.
*  * CYBLE_ERROR_INVALID_PARAMETER - Validation of the input parameters failed.
*  * CYBLE_ERROR_INVALID_STATE - The state is not valid.
*  * CYBLE_ERROR_MEMORY_ALLOCATION_FAILED - Memory allocation failed.
*  * CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE - The peer device doesn't have
*                                              the particular descriptor.
*  * CYBLE_ERROR_INVALID_OPERATION - This operation is not permitted on 
*                                    the specified attribute
*
* \events
*  In case of successful execution (return value = CYBLE_ERROR_OK)
*  the next events can appear: \n
*  If the ANCS service-specific callback is registered 
*      (with CyBle_AncsRegisterAttrCallback):
*  * CYBLE_EVT_ANCSC_READ_DESCR_RESPONSE - If the requested attribute is
*                                successfully written on the peer device,
*                                the details (char index, descr index, value, etc.) 
*                                are provided with an event parameter structure
*                                of type CYBLE_ANCS_DESCR_VALUE_T. 
*  .
*  Otherwise (if the ANCS service-specific callback is not registered):
*  * CYBLE_EVT_GATTC_READ_RSP - If the requested attribute is 
*                                successfully read on the peer device,
*                                the details (handle, value, etc.) are 
*                                provided with an event parameter
*                                structure (CYBLE_GATTC_READ_RSP_PARAM_T).
*  * CYBLE_EVT_GATTC_ERROR_RSP - If there is some trouble with the 
*                                requested attribute on the peer device,
*                                the details are provided with an event parameter
*                                structure (CYBLE_GATTC_ERR_RSP_PARAM_T).
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_AncscGetCharacteristicDescriptor(CYBLE_CONN_HANDLE_T connHandle,
                       CYBLE_ANCS_CHAR_INDEX_T charIndex, CYBLE_ANCS_DESCR_INDEX_T descrIndex)
{
    CYBLE_API_RESULT_T apiResult;
    
    /* Check parameters */
    if(CyBle_GetClientState() != CYBLE_CLIENT_STATE_DISCOVERED)
    {
        apiResult = CYBLE_ERROR_INVALID_STATE;
    }
    else if((charIndex >= CYBLE_ANCS_CHAR_COUNT) || (descrIndex >= CYBLE_ANCS_DESCR_COUNT))
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
    else if(cyBle_ancsc.charInfo[charIndex].descrHandle[descrIndex] == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
		apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
	}
    else
    {
        apiResult = CyBle_GattcReadCharacteristicDescriptors(connHandle, cyBle_ancsc.charInfo[charIndex].descrHandle[descrIndex]);
        if(apiResult == CYBLE_ERROR_OK)
        {
            cyBle_ancscReqHandle = cyBle_ancsc.charInfo[charIndex].descrHandle[descrIndex];
        }
    }
    
    return (apiResult);
}

/******************************************************************************
* Function Name: CyBle_AncscDiscoverServiceEventHandler
***************************************************************************//**
* 
*  This function is called on receiving a Read-By-Group Response event or 
*  Read response with 128-bit service uuid. 
* 
*  \param discServInfo: The pointer to a service information structure.
* 
******************************************************************************/
void CyBle_AncscDiscoverServiceEventHandler(const CYBLE_DISC_SRVC128_INFO_T *discServInfo)
{
    if(memcmp(&cyBle_ancscServUuid, &discServInfo->uuid.uuid128, CYBLE_GATT_128_BIT_UUID_SIZE) == 0u)
    {
        if(cyBle_serverInfo[CYBLE_SRVI_ANCS].range.startHandle == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
        {
            cyBle_serverInfo[CYBLE_SRVI_ANCS].range = discServInfo->range;
            cyBle_disCount++;
        }
    }
}

/******************************************************************************
* Function Name: CyBle_AncscDiscoverCharacteristicsEventHandler
***************************************************************************//**
* 
*  This function is called on receiving a CYBLE_EVT_GATTC_READ_BY_TYPE_RSP
*  event. Based on the service UUID, an appropriate data structure is populated
*  using the data received as part of the callback.
* 
*  \param discCharInfo: The pointer to a characteristic information structure.
* 
******************************************************************************/
void CyBle_AncscDiscoverCharacteristicsEventHandler(CYBLE_DISC_CHAR_INFO_T *discCharInfo)
{
    static CYBLE_GATT_DB_ATTR_HANDLE_T *lnsLastEndHandle = NULL;
    CYBLE_ANCS_CHAR_INDEX_T locCharIndex;
    
    /* Update last characteristic endHandle to declaration handle of this characteristic */
    if(lnsLastEndHandle != NULL)
    {
        *lnsLastEndHandle = discCharInfo->charDeclHandle - 1u;
        lnsLastEndHandle = NULL;
    }
    
    for(locCharIndex = CYBLE_ANCS_NS; locCharIndex < CYBLE_ANCS_CHAR_COUNT; locCharIndex++)
    {
        if(memcmp(&cyBle_ancscCharUuid[locCharIndex], &discCharInfo->uuid.uuid128, CYBLE_GATT_128_BIT_UUID_SIZE) == 0u)
        {
            if(cyBle_ancsc.charInfo[locCharIndex].valueHandle == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
            {
                cyBle_ancsc.charInfo[locCharIndex].valueHandle = discCharInfo->valueHandle;
                cyBle_ancsc.charInfo[locCharIndex].properties = discCharInfo->properties;
                lnsLastEndHandle = &cyBle_ancsc.charInfo[locCharIndex].endHandle;
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
    if(lnsLastEndHandle != NULL)
    {
        *lnsLastEndHandle = cyBle_serverInfo[CYBLE_SRVI_ANCS].range.endHandle;
    }
}


/******************************************************************************
* Function Name: CyBle_AncscDiscoverCharDescriptorsEventHandler
***************************************************************************//**
* 
*  This function is called on receiving a CYBLE_EVT_GATTC_READ_BY_TYPE_RSP
*  event. Based on the service UUID, an appropriate data structure is populated
*  using the data received as part of the callback.
* 
*  \param discCharInfo: The pointer to the characteristic information structure.
* 
******************************************************************************/
void CyBle_AncscDiscoverCharDescriptorsEventHandler(CYBLE_DISC_DESCR_INFO_T * discDescrInfo)
{
    if(discDescrInfo->uuid.uuid16 == CYBLE_UUID_CHAR_CLIENT_CONFIG)
    {
        uint8 descrIdx = (cyBle_disCount - CYBLE_SCDI_ANCS_NS);
        
        if(cyBle_disCount > (uint8) CYBLE_SCDI_ANCS_NS)
        {
            descrIdx++;
        }
        
        if(cyBle_ancsc.charInfo[descrIdx].descrHandle[CYBLE_ANCS_CCCD] == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
        {
            cyBle_ancsc.charInfo[descrIdx].descrHandle[CYBLE_ANCS_CCCD] = discDescrInfo->descrHandle;
        }
        else    /* Duplication of descriptor */
        {
            CyBle_ApplCallback(CYBLE_EVT_GATTC_DESCR_DUPLICATION, &discDescrInfo->uuid);
        }
    }
}


/******************************************************************************
* Function Name: CyBle_AncscNotificationEventHandler
***************************************************************************//**
* 
*  Handles the Notification Event.
* 
*  \param eventParam: The pointer to the data structure specified by the event.
* 
******************************************************************************/
void CyBle_AncscNotificationEventHandler(CYBLE_GATTC_HANDLE_VALUE_NTF_PARAM_T *eventParam)
{
    if(NULL != CyBle_AncsApplCallback)
    {
        CYBLE_ANCS_CHAR_VALUE_T locCharValue;
        
        if(cyBle_ancsc.charInfo[CYBLE_ANCS_NS].valueHandle == eventParam->handleValPair.attrHandle)
        {
            locCharValue.charIndex = CYBLE_ANCS_NS;
            cyBle_eventHandlerFlag &= (uint8) ~CYBLE_CALLBACK;
        }
        else if(cyBle_ancsc.charInfo[CYBLE_ANCS_DS].valueHandle == eventParam->handleValPair.attrHandle)
        {
            locCharValue.charIndex = CYBLE_ANCS_DS;
            cyBle_eventHandlerFlag &= (uint8) ~CYBLE_CALLBACK;
        }
        else
        {
            /* Apple Notification Center Service doesn't support any other notifications */
        }
        
        if((cyBle_eventHandlerFlag & CYBLE_CALLBACK) == 0u)
        {
            locCharValue.connHandle = eventParam->connHandle;
            
            locCharValue.value = &eventParam->handleValPair.value;

            CyBle_AncsApplCallback(CYBLE_EVT_ANCSC_NOTIFICATION, &locCharValue);
        }
    }
}


/******************************************************************************
* Function Name: CyBle_AncscReadResponseEventHandler
***************************************************************************//**
* 
*  Handles the Read Response Event.
* 
*  \param eventParam: The pointer to the data structure.
* 
******************************************************************************/
void CyBle_AncscReadResponseEventHandler(CYBLE_GATTC_READ_RSP_PARAM_T *eventParam)
{
    if(cyBle_ancscReqHandle != CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
        if(CyBle_AncsApplCallback != NULL)
        {
            CYBLE_ANCS_DESCR_VALUE_T locDescrValue;
        
            if(cyBle_ancsc.charInfo[CYBLE_ANCS_NS].descrHandle[CYBLE_ANCS_CCCD] == cyBle_ancscReqHandle)
            {
                locDescrValue.charIndex = CYBLE_ANCS_NS;
                cyBle_eventHandlerFlag &= (uint8)~CYBLE_CALLBACK;
            }
            else if(cyBle_ancsc.charInfo[CYBLE_ANCS_DS].descrHandle[CYBLE_ANCS_CCCD] == cyBle_ancscReqHandle)
            {
                locDescrValue.charIndex = CYBLE_ANCS_DS;
                cyBle_eventHandlerFlag &= (uint8)~CYBLE_CALLBACK;
            }
            else
            {   
            }

            if((cyBle_eventHandlerFlag & CYBLE_CALLBACK) == 0u)
            {   
                locDescrValue.connHandle = eventParam->connHandle;
                locDescrValue.value = &eventParam->value;
                
                CyBle_AncsApplCallback((uint32)CYBLE_EVT_ANCSC_READ_DESCR_RESPONSE, &locDescrValue);
            }
        }
        
        cyBle_ancscReqHandle = CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE;
    }
}


/******************************************************************************
* Function Name: CyBle_AncscWriteResponseEventHandler
***************************************************************************//**
* 
*  Handles the Write Response Event.
* 
*  \param eventParam: The pointer to a data structure specified by the event.
* 
******************************************************************************/
void CyBle_AncscWriteResponseEventHandler(const CYBLE_CONN_HANDLE_T *eventParam)
{
    if((NULL != CyBle_AncsApplCallback) && (CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE != cyBle_ancscReqHandle))
    {
        if(cyBle_ancsc.charInfo[CYBLE_ANCS_CP].valueHandle == cyBle_ancscReqHandle)
        {
            CYBLE_ANCS_CHAR_VALUE_T locCharIndex;
            
            locCharIndex.connHandle = *eventParam;
            locCharIndex.charIndex = CYBLE_ANCS_CP;
            locCharIndex.value = NULL;
            
            cyBle_ancscReqHandle = CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE;
            cyBle_eventHandlerFlag &= (uint8)~CYBLE_CALLBACK;
            CyBle_AncsApplCallback((uint32)CYBLE_EVT_ANCSC_WRITE_CHAR_RESPONSE, &locCharIndex);
        }
        else
        {
            CYBLE_ANCS_DESCR_VALUE_T locDescIndex;
            
            if(cyBle_ancsc.charInfo[CYBLE_ANCS_NS].descrHandle[CYBLE_ANCS_CCCD] == cyBle_ancscReqHandle)
            {
                locDescIndex.charIndex = CYBLE_ANCS_NS;
                cyBle_eventHandlerFlag &= (uint8) ~CYBLE_CALLBACK;
            }
            else if(cyBle_ancsc.charInfo[CYBLE_ANCS_DS].descrHandle[CYBLE_ANCS_CCCD] == cyBle_ancscReqHandle)
            {
                locDescIndex.charIndex = CYBLE_ANCS_DS;
                cyBle_eventHandlerFlag &= (uint8) ~CYBLE_CALLBACK;
            }
            else
            {
                /* Apple Notification Center Service doesn't support any other notifications */
            }
            
            if((cyBle_eventHandlerFlag & CYBLE_CALLBACK) == 0u)
            {
                locDescIndex.connHandle = *eventParam;
                locDescIndex.descrIndex = CYBLE_ANCS_CCCD;
                locDescIndex.value = NULL;

                cyBle_ancscReqHandle = CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE;
                cyBle_eventHandlerFlag &= (uint8)~CYBLE_CALLBACK;
                CyBle_AncsApplCallback((uint32)CYBLE_EVT_ANCSC_WRITE_DESCR_RESPONSE, &locDescIndex);
            }
        }
    }
}


/******************************************************************************
* Function Name: CyBle_AncscErrorResponseEventHandler
***************************************************************************//**
* 
*  Handles the Error Response Event.
* 
*  \param eventParam: Pointer to the data structure specified by the event.
* 
******************************************************************************/
void CyBle_AncscErrorResponseEventHandler(const CYBLE_GATTC_ERR_RSP_PARAM_T *eventParam)
{
    if((eventParam != NULL) && (eventParam->attrHandle == cyBle_ancscReqHandle))
    {
        if((CyBle_AncsApplCallback != NULL) &&
           (cyBle_ancscReqHandle == cyBle_ancsc.charInfo[CYBLE_ANCS_CP].valueHandle))
        {
            CYBLE_ANCS_CHAR_VALUE_T locGattError;
                
            locGattError.connHandle = eventParam->connHandle;
            locGattError.charIndex = CYBLE_ANCS_CP;
            locGattError.value = NULL;
            locGattError.gattErrorCode = eventParam->errorCode;
            
            CyBle_AncsApplCallback((uint32)CYBLE_EVT_ANCSC_ERROR_RESPONSE, &locGattError);
            cyBle_eventHandlerFlag &= (uint8)~CYBLE_CALLBACK;
        }
        
        cyBle_ancscReqHandle = CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE;
    }
}


#endif /* CYBLE_ANCS_CLIENT */


/* [] END OF FILE */
