/***************************************************************************//**
* \file CYBLE_hps.c
* \version `$CY_MAJOR_VERSION`.`$CY_MINOR_VERSION`
* 
* \brief
*  Contains the source code for HTTP Proxy Service.
* 
********************************************************************************
* \copyright
* Copyright 2016-2018, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "`$INSTANCE_NAME`_eventHandler.h"

#ifdef CYBLE_HPS_SERVER
/* Generated code */
`$HpsServer`
/* End of generated code */

static CYBLE_GATT_DB_ATTR_HANDLE_T cyBle_hpssReqHandle;

#endif /* CYBLE_HPS_SERVER */

#ifdef CYBLE_HPS_CLIENT

CYBLE_HPSC_T cyBle_hpsc;

static CYBLE_GATT_DB_ATTR_HANDLE_T cyBle_hpscReqHandle;

/* Variables for Read Long Characteristic Values handling */
static uint8 * cyBle_hpscRdLongBuffPtr;
static uint16 cyBle_hpscRdLongBuffLen;
static uint16 cyBle_hpscCurrLen;

#endif /* CYBLE_HPS_CLIENT */

static CYBLE_CALLBACK_T CyBle_HpsApplCallback = NULL;


/******************************************************************************
* Function Name: CyBle_HpsInit
***************************************************************************//**
* 
*  This function initializes HTTP Proxy Service.
* 
******************************************************************************/
void CyBle_HpsInit(void)
{
#ifdef CYBLE_HPS_SERVER
    cyBle_hpssReqHandle = CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE;
#endif /* CYBLE_HPS_SERVER */

#ifdef CYBLE_HPS_CLIENT
    
    cyBle_hpscRdLongBuffPtr = NULL;
    cyBle_hpscRdLongBuffLen = 0u;
    cyBle_hpscCurrLen = 0u;
    
    if(cyBle_serverInfo[CYBLE_SRVI_HPS].range.startHandle == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
        (void)memset(&cyBle_hpsc, 0, sizeof(cyBle_hpsc));
    }
    cyBle_hpscReqHandle = CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE;

#endif /* CYBLE_HPS_CLIENT */

}


/******************************************************************************
* Function Name: CyBle_HpsRegisterAttrCallback
***************************************************************************//**
* 
*  Registers a callback function for service specific attribute operations.
*  Service specific write requests from peer device will not be handled with
*  unregistered callback function.
* 
*  \param callbackFunc:  An application layer event callback function to receive
*                 events from the BLE Component. The definition of
*                 CYBLE_CALLBACK_T is: \n
*                 typedef void (* CYBLE_CALLBACK_T) (uint32 eventCode,
*                                                    void *eventParam) \n
*                 * eventCode - Indicates the event that triggered this
*                   callback (e.g. CYBLE_EVT_HPSS_NOTIFICATION_ENABLED).
*                 * eventParam - Contains the parameters corresponding to the
*                   current event. (e.g. pointer to CYBLE_HPS_CHAR_VALUE_T
*                   structure that contains details of the characteristic
*                   for which an indication enabled event was triggered).
*
******************************************************************************/
void CyBle_HpsRegisterAttrCallback(CYBLE_CALLBACK_T callbackFunc)
{
    CyBle_HpsApplCallback = callbackFunc;
}


#ifdef CYBLE_HPS_SERVER

/******************************************************************************
* Function Name: CyBle_HpssWriteEventHandler
***************************************************************************//**
* 
*  Handles Write Request Event for HTTP Proxy Service.
* 
*  \param eventParam: The pointer to the data structure specified by the event.
* 
* \return
*  Return value is of type CYBLE_GATT_ERR_CODE_T.
*   * CYBLE_GATT_ERR_NONE - Write is successful.
*   * CYBLE_GATT_ERR_REQUEST_NOT_SUPPORTED - The request is not supported.
*   * CYBLE_GATT_ERR_INVALID_HANDLE - 'handleValuePair.attrHandle' is not valid.
*   * CYBLE_GATT_ERR_WRITE_NOT_PERMITTED - The write operation is not permitted on
*                                          this attribute.
*   * CYBLE_GATT_ERR_INVALID_OFFSET - The offset value is invalid.
*   * CYBLE_GATT_ERR_UNLIKELY_ERROR - Some other error occurred.
* 
******************************************************************************/
CYBLE_GATT_ERR_CODE_T CyBle_HpssWriteEventHandler(CYBLE_GATTS_WRITE_REQ_PARAM_T *eventParam)
{
    CYBLE_GATT_ERR_CODE_T gattErr = CYBLE_GATT_ERR_NONE;
    uint32 event = (uint32) CYBLE_EVT_HPSS_NOTIFICATION_DISABLED;
    CYBLE_HPS_CHAR_VALUE_T wrCharReqParam;

    if(CyBle_HpsApplCallback != NULL)
    {
        wrCharReqParam.connHandle = eventParam->connHandle;

        /* Client Characteristic Configuration descriptor write request */
        if(eventParam->handleValPair.attrHandle ==
            cyBle_hpss.charInfo[CYBLE_HPS_HTTP_STATUS_CODE].descrHandle[CYBLE_HPS_CCCD])
        {
            wrCharReqParam.charIndex = CYBLE_HPS_HTTP_STATUS_CODE;
            wrCharReqParam.value = NULL;

            if(CYBLE_IS_NOTIFICATION_ENABLED_IN_PTR(eventParam->handleValPair.value.val))
            {
                event = (uint32) CYBLE_EVT_HPSS_NOTIFICATION_ENABLED;
            }
        }
        else 
        {
            event = CYBLE_EVT_HPSS_CHAR_WRITE;
            wrCharReqParam.value = &eventParam->handleValPair.value;

            if (cyBle_hpss.charInfo[CYBLE_HPS_URI].charHandle == eventParam->handleValPair.attrHandle)
            {
                wrCharReqParam.charIndex = CYBLE_HPS_URI;
            }
            else if (cyBle_hpss.charInfo[CYBLE_HPS_HTTP_HEADERS].charHandle == eventParam->handleValPair.attrHandle)
            {
                wrCharReqParam.charIndex = CYBLE_HPS_HTTP_HEADERS;
            }
            else if (cyBle_hpss.charInfo[CYBLE_HPS_HTTP_ENTITY_BODY].charHandle == eventParam->handleValPair.attrHandle)
            {
                wrCharReqParam.charIndex = CYBLE_HPS_HTTP_ENTITY_BODY;
            }
            else if (cyBle_hpss.charInfo[CYBLE_HPS_HTTP_CP].charHandle == eventParam->handleValPair.attrHandle)
            {
                if(CYBLE_IS_NOTIFICATION_ENABLED(
                    cyBle_hpss.charInfo[CYBLE_HPS_HTTP_STATUS_CODE].descrHandle[CYBLE_HPS_CCCD]))
                {
                    wrCharReqParam.charIndex = CYBLE_HPS_HTTP_CP;
                }
                else
                {
                    gattErr = CYBLE_GATT_ERR_CCCD_IMPROPERLY_CONFIGURED;
                    /* Clear callback flag indicating that request was handled */
                    cyBle_eventHandlerFlag &= (uint8)~CYBLE_CALLBACK;
                }
            }
            else
            {
                /* Set charIndex  to CYBLE_HPS_CHAR_COUNT as the requested handle doesn't 
                * match to any handles of HPS characteristics.
                */
                wrCharReqParam.charIndex = CYBLE_HPS_CHAR_COUNT;
            }
        }
        
        if(gattErr == CYBLE_GATT_ERR_NONE)
        {
            if(wrCharReqParam.charIndex < CYBLE_HPS_CHAR_COUNT)            
            {
                gattErr = CyBle_GattsWriteAttributeValue(&eventParam->handleValPair, 0u, 
                    &eventParam->connHandle, CYBLE_GATT_DB_PEER_INITIATED);

                if(gattErr == CYBLE_GATT_ERR_NONE)
                {
                    if((event == (uint32) CYBLE_EVT_HPSS_CHAR_WRITE) &&
                        (wrCharReqParam.value->len <=
                            (CYBLE_GATT_DB_ATTR_GET_ATTR_GEN_MAX_LEN(eventParam->handleValPair.attrHandle))))
                    {
                        CYBLE_GATT_DB_ATTR_SET_ATTR_GEN_LEN(eventParam->handleValPair.attrHandle,
                                                            wrCharReqParam.value->len);
                    }
                    
                    /* Make sure that GATT error is set to "No error" */
                    wrCharReqParam.gattErrorCode = CYBLE_GATT_ERR_NONE;

                    CyBle_HpsApplCallback(event, &wrCharReqParam);

                    gattErr = wrCharReqParam.gattErrorCode;
                }

                if((event == (uint32) CYBLE_EVT_HPSS_NOTIFICATION_ENABLED) ||
                    (event == (uint32) CYBLE_EVT_HPSS_NOTIFICATION_DISABLED))
                {
                #if((CYBLE_GAP_ROLE_PERIPHERAL || CYBLE_GAP_ROLE_CENTRAL) && \
                    (CYBLE_BONDING_REQUIREMENT == CYBLE_BONDING_YES))
                    /* Set flag to store bonding data to flash */
                    if(cyBle_peerBonding == CYBLE_GAP_BONDING)
                    {
                        cyBle_pendingFlashWrite |= CYBLE_PENDING_CCCD_FLASH_WRITE_BIT;
                    }
                #endif /* (CYBLE_BONDING_REQUIREMENT == CYBLE_BONDING_YES) */
                }
                
                /* Clear callback flag indicating that request was handled */
                cyBle_eventHandlerFlag &= (uint8)~CYBLE_CALLBACK;
            }
        }
    }

    return(gattErr);
}


/****************************************************************************** 
* Function Name: CyBle_HpssPrepareWriteRequestEventHandler
***************************************************************************//**
* 
*  Handles the Write Request HTTP Proxy Service.
* 
*  \param eventParam: The pointer to the data that received with a prepare 
*                     write request event for the HTTP Proxy Service.
* 
******************************************************************************/
void CyBle_HpssPrepareWriteRequestEventHandler(const CYBLE_GATTS_PREP_WRITE_REQ_PARAM_T *eventParam)
{

    if(CyBle_HpsApplCallback != NULL)
    {
        if((eventParam->baseAddr[eventParam->currentPrepWriteReqCount - 1u].handleValuePair.attrHandle ==
            cyBle_hpss.charInfo[CYBLE_HPS_URI].charHandle) || 
            (eventParam->baseAddr[eventParam->currentPrepWriteReqCount - 1u].handleValuePair.attrHandle ==
            cyBle_hpss.charInfo[CYBLE_HPS_HTTP_HEADERS].charHandle) ||
            (eventParam->baseAddr[eventParam->currentPrepWriteReqCount - 1u].handleValuePair.attrHandle ==
            cyBle_hpss.charInfo[CYBLE_HPS_HTTP_ENTITY_BODY].charHandle))
        {
            if(cyBle_hpssReqHandle == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
            {
                /* Send Prepare Write Response which identifies acknowledgment for
                * long characteristic value write.
                */
                CyBle_GattsPrepWriteReqSupport(CYBLE_GATTS_PREP_WRITE_SUPPORT);

                cyBle_hpssReqHandle =
                    eventParam->baseAddr[eventParam->currentPrepWriteReqCount - 1u].handleValuePair.attrHandle;
            }
            /* Indicate that request was handled */
            cyBle_eventHandlerFlag &= (uint8)~CYBLE_CALLBACK;
        }
    }
}


/****************************************************************************** 
* Function Name: CyBle_HpssExecuteWriteRequestEventHandler
***************************************************************************//**
* 
*  Handles the Execute Write Request Event for the HTTP Proxy Service.
* 
*  \param eventParam: The pointer to the data that came with a write request
*                     for the HTTP Proxy Service.
*
******************************************************************************/
void CyBle_HpssExecuteWriteRequestEventHandler(CYBLE_GATTS_EXEC_WRITE_REQ_T *eventParam)
{
    uint8 locCount;
    uint16 locLength = 0u;
    CYBLE_GATT_VALUE_T locCharValue;
    CYBLE_HPS_CHAR_VALUE_T wrCharReqParam;

    if(CyBle_HpsApplCallback != NULL)
    {
        if((eventParam->baseAddr[0u].handleValuePair.attrHandle == cyBle_hpss.charInfo[CYBLE_HPS_URI].charHandle) ||
            (eventParam->baseAddr[0u].handleValuePair.attrHandle ==
                cyBle_hpss.charInfo[CYBLE_HPS_HTTP_HEADERS].charHandle) ||
            (eventParam->baseAddr[0u].handleValuePair.attrHandle ==
                cyBle_hpss.charInfo[CYBLE_HPS_HTTP_ENTITY_BODY].charHandle))
        {
            /* Check the execWriteFlag before execute or cancel write long operation */
            if(eventParam->execWriteFlag == CYBLE_GATT_EXECUTE_WRITE_EXEC_FLAG)
            {
                /* Calculate total length */
                for(locCount = 0u; locCount < eventParam->prepWriteReqCount; locCount++)
                {
                    locLength += eventParam->baseAddr[locCount].handleValuePair.value.len;
                }

                if(CYBLE_GATT_DB_ATTR_GET_ATTR_GEN_MAX_LEN(cyBle_hpssReqHandle) >= locLength)
                {
                    /* Fill data and pass it to user */
                    if(eventParam->baseAddr[0u].handleValuePair.attrHandle == cyBle_hpss.charInfo[CYBLE_HPS_URI].charHandle)
                    {
                        wrCharReqParam.charIndex = CYBLE_HPS_URI;
                    }
                    else if(eventParam->baseAddr[0u].handleValuePair.attrHandle ==
                            cyBle_hpss.charInfo[CYBLE_HPS_HTTP_HEADERS].charHandle)
                    {
                        wrCharReqParam.charIndex = CYBLE_HPS_HTTP_HEADERS;
                    }
                    else
                    {
                        wrCharReqParam.charIndex = CYBLE_HPS_HTTP_ENTITY_BODY;
                    }

                    wrCharReqParam.connHandle = eventParam->connHandle;
                    wrCharReqParam.gattErrorCode = CYBLE_GATT_ERR_NONE;
                    locCharValue = eventParam->baseAddr[0u].handleValuePair.value;
                    wrCharReqParam.value = &locCharValue;
                    wrCharReqParam.value->len = locLength;

                    CyBle_HpsApplCallback(CYBLE_EVT_HPSS_CHAR_WRITE, &wrCharReqParam);

                    if(wrCharReqParam.gattErrorCode == CYBLE_GATT_ERR_NONE)
                    {
                        CYBLE_GATT_DB_ATTR_SET_ATTR_GEN_LEN(cyBle_hpssReqHandle, locLength);
                    }
                }
                else
                {
                    wrCharReqParam.gattErrorCode = CYBLE_GATT_ERR_INVALID_ATTRIBUTE_LEN;
                }

                /* Pass user error code to Stack */
                eventParam->gattErrorCode = wrCharReqParam.gattErrorCode;
            }

            /* Indicate that request was handled */
            cyBle_eventHandlerFlag &= (uint8)~CYBLE_CALLBACK;

            /* Clear requested handle */
            cyBle_hpssReqHandle = CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE;
        }
    }
}


/****************************************************************************** 
* Function Name: CyBle_HpssSetCharacteristicValue
***************************************************************************//**
* 
*  Sets a value for one of characteristic values of the HTTP Proxy 
*  Service. The characteristic is identified by charIndex.
* 
*  \param charIndex: The index of a HTTP Proxy Service characteristic.
*  \param attrSize:  The size of the characteristic value attribute.
*  \param attrValue: The pointer to the characteristic value data that should be 
*                    stored to the GATT database.
* 
* \return
*  A return value is of type CYBLE_API_RESULT_T.
*   * CYBLE_ERROR_OK - The characteristic value was written successfully.
*   * CYBLE_ERROR_INVALID_PARAMETER - Validation of the input parameters failed.
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_HpssSetCharacteristicValue(CYBLE_HPS_CHAR_INDEX_T charIndex, uint16 attrSize, uint8 *attrValue)
{
    CYBLE_API_RESULT_T apiResult = CYBLE_ERROR_INVALID_PARAMETER;

    if((attrValue != NULL) && (charIndex < CYBLE_HPS_CHAR_COUNT))
    {
        CYBLE_GATT_HANDLE_VALUE_PAIR_T hpsHandleValuePair;

        /* Fill structure */
        hpsHandleValuePair.attrHandle = cyBle_hpss.charInfo[charIndex].charHandle;
        hpsHandleValuePair.value.len = attrSize;
        hpsHandleValuePair.value.val = attrValue;

        if(CyBle_GattsWriteAttributeValue(&hpsHandleValuePair, 0u, NULL, CYBLE_GATT_DB_LOCALLY_INITIATED) ==
                CYBLE_GATT_ERR_NONE)
        {
            if(attrSize <= CYBLE_GATT_DB_ATTR_GET_ATTR_GEN_MAX_LEN(hpsHandleValuePair.attrHandle))
            {
                CYBLE_GATT_DB_ATTR_SET_ATTR_GEN_LEN(hpsHandleValuePair.attrHandle, attrSize);
            }

            /* Indicate success */
            apiResult = CYBLE_ERROR_OK;
        }
    }

    /* Return status */
    return(apiResult);
}


/****************************************************************************** 
* Function Name: CyBle_HpssGetCharacteristicValue
***************************************************************************//**
* 
*  Reads a characteristic value of the HTTP Proxy Service, which is identified 
*  by charIndex from the GATT database.
* 
*  \param charIndex: The index of the HTTP Proxy Service characteristic.
*  \param attrSize:  The size of the HTTP Proxy Service characteristic value 
*                    attribute.
*  \param attrValue: The pointer to the location where characteristic value 
*                    data should be stored.
*
* \return
*  A return value is of type CYBLE_API_RESULT_T.
*   * CYBLE_ERROR_OK - The characteristic value was read successfully.
*   * CYBLE_ERROR_INVALID_PARAMETER - Validation of the input parameters failed.
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_HpssGetCharacteristicValue(CYBLE_HPS_CHAR_INDEX_T charIndex, uint16 attrSize, uint8 *attrValue)
{
    
    CYBLE_API_RESULT_T apiResult = CYBLE_ERROR_INVALID_PARAMETER;

    if((attrValue != NULL) && (charIndex < CYBLE_HPS_CHAR_COUNT))
    {
        CYBLE_GATT_HANDLE_VALUE_PAIR_T hpsHandleValuePair;

        hpsHandleValuePair.attrHandle = cyBle_hpss.charInfo[charIndex].charHandle;
        hpsHandleValuePair.value.len = attrSize;
        hpsHandleValuePair.value.val = attrValue;

        /* Get characteristic value from GATT database */
        if(CyBle_GattsReadAttributeValue(&hpsHandleValuePair, NULL, CYBLE_GATT_DB_LOCALLY_INITIATED) ==
            CYBLE_GATT_ERR_NONE)
        {
            /* Indicate success */
            apiResult = CYBLE_ERROR_OK;
        }
    }

    /* Return status */
    return(apiResult);
}


/******************************************************************************
* Function Name: CyBle_HpssSetCharacteristicDescriptor
***************************************************************************//**
* 
*  Sets the characteristic descriptor value of the specified characteristic.
* 
*  \param charIndex:  The index of the service characteristic.
*  \param descrIndex: The index of the descriptor.
*  \param attrSize:   The size of the characteristic descriptor attribute.
*  \param attrValue:  The pointer to the descriptor value data to be stored in 
*                     the GATT database.
* 
* \return
*  A return value is of type CYBLE_API_RESULT_T.
*  * CYBLE_ERROR_OK - The request handled successfully.
*  * CYBLE_ERROR_INVALID_PARAMETER - Validation of the input parameter failed.
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_HpssSetCharacteristicDescriptor(CYBLE_HPS_CHAR_INDEX_T charIndex,
    CYBLE_HPS_DESCR_INDEX_T descrIndex, uint8 attrSize, uint8 *attrValue)
{
    CYBLE_GATT_HANDLE_VALUE_PAIR_T locHandleValuePair;
    CYBLE_API_RESULT_T apiResult = CYBLE_ERROR_INVALID_PARAMETER;

    if((charIndex == CYBLE_HPS_HTTP_STATUS_CODE) && (descrIndex == CYBLE_HPS_CCCD) && (attrValue != NULL))
    {
        locHandleValuePair.attrHandle = cyBle_hpss.charInfo[CYBLE_HPS_HTTP_STATUS_CODE].descrHandle[CYBLE_HPS_CCCD];
        locHandleValuePair.value.len = attrSize;
        locHandleValuePair.value.val = attrValue;

        /* Sets characteristic value to database */
        if(CyBle_GattsWriteAttributeValue(&locHandleValuePair, 0u, NULL, CYBLE_GATT_DB_LOCALLY_INITIATED) == 
            CYBLE_GATT_ERR_NONE)
        {
            apiResult = CYBLE_ERROR_OK;
        }
    }
    
    return (apiResult);
}


/****************************************************************************** 
* Function Name: CyBle_HpssGetCharacteristicDescriptor
***************************************************************************//**
* 
*  Reads a a characteristic descriptor of a specified characteristic of the
*  HTTP Proxy Service from the GATT database.
* 
*  \param charIndex:  The index of the characteristic.
*  \param descrIndex: The index of the descriptor.
*  \param attrSize:   The size of the descriptor value.
*  \param attrValue:  The pointer to the location where characteristic 
*                     descriptor value data should be stored.
* 
* \return
*  A return value is of type CYBLE_API_RESULT_T.
*   * CYBLE_ERROR_OK - The request handled successfully.
*   * CYBLE_ERROR_INVALID_PARAMETER - Validation of the input parameter failed.
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_HpssGetCharacteristicDescriptor(CYBLE_HPS_CHAR_INDEX_T charIndex,
    CYBLE_HPS_DESCR_INDEX_T descrIndex, uint8 attrSize, uint8 *attrValue)
{
    CYBLE_API_RESULT_T apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    
    if((charIndex == CYBLE_HPS_HTTP_STATUS_CODE) && (descrIndex == CYBLE_HPS_CCCD) && (attrValue != NULL))
    {
        if(cyBle_hpss.charInfo[CYBLE_HPS_HTTP_STATUS_CODE].descrHandle[CYBLE_HPS_CCCD] !=
            CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
        {
            CYBLE_GATT_HANDLE_VALUE_PAIR_T locHandleValuePair;

            locHandleValuePair.attrHandle = 
                cyBle_hpss.charInfo[CYBLE_HPS_HTTP_STATUS_CODE].descrHandle[CYBLE_HPS_CCCD];
            locHandleValuePair.value.len = attrSize;
            locHandleValuePair.value.val = attrValue;
        
            /* Get characteristic value from GATT database */
            if(CyBle_GattsReadAttributeValue(&locHandleValuePair, NULL, CYBLE_GATT_DB_LOCALLY_INITIATED) ==
                CYBLE_GATT_ERR_NONE)
            {
                /* Indicate success */
                apiResult = CYBLE_ERROR_OK;
            }
        }
    }
    
    return (apiResult);
}


/******************************************************************************
* Function Name: CyBle_HpssSendNotification
***************************************************************************//**
* 
*  Sends a notification with a characteristic value of the HTTP Proxy
*  Service, which is a value specified by charIndex, to the client's device.
* 
*  On enabling notification successfully for a service characteristic it sends 
*  out a 'Handle Value Notification' which results in 
*  CYBLE_EVT_HPSC_NOTIFICATION event at the GATT Client's end.
*
*  \param connHandle: The connection handle.
*  \param charIndex:  The index of the service characteristic. Starts with zero.
*  \param attrSize:   The size of the characteristic value attribute.
*  \param attrValue:  The pointer to the characteristic value data that should 
*                     be sent to the client's device.
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
CYBLE_API_RESULT_T CyBle_HpssSendNotification(CYBLE_CONN_HANDLE_T connHandle,
    CYBLE_HPS_CHAR_INDEX_T charIndex, uint8 attrSize, uint8 *attrValue)
{
    CYBLE_API_RESULT_T apiResult = CYBLE_ERROR_INVALID_PARAMETER;

    if(charIndex < CYBLE_HPS_CHAR_COUNT)
    {
        if(cyBle_hpss.charInfo[CYBLE_HPS_HTTP_STATUS_CODE].descrHandle[CYBLE_HPS_CCCD] !=
            CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
        {
            /* Send notification if it is enabled and peer device is connected */
            if(CyBle_GetState() != CYBLE_STATE_CONNECTED)
            {
                apiResult = CYBLE_ERROR_INVALID_STATE;
            }
            
            else if(!CYBLE_IS_NOTIFICATION_ENABLED(
                cyBle_hpss.charInfo[CYBLE_HPS_HTTP_STATUS_CODE].descrHandle[CYBLE_HPS_CCCD]))
            {
                apiResult = CYBLE_ERROR_NTF_DISABLED;
            }
            else
            {
                CYBLE_GATTS_HANDLE_VALUE_NTF_T ntfReqParam;

                /* Fill all fields of write request structure ... */
                ntfReqParam.attrHandle = cyBle_hpss.charInfo[CYBLE_HPS_HTTP_STATUS_CODE].charHandle;
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

#endif /* CYBLE_HPS_SERVER */


#ifdef CYBLE_HPS_CLIENT

/****************************************************************************** 
* Function Name: CyBle_HpscDiscoverCharacteristicsEventHandler
***************************************************************************//**
* 
*  This function is called on receiving a CYBLE_EVT_GATTC_READ_BY_TYPE_RSP
*  event. Based on the service UUID, an appropriate data structure is populated
*  using the data received as part of the callback.
* 
*  \param discCharInfo: The pointer to a characteristic information structure.
*
******************************************************************************/
void CyBle_HpscDiscoverCharacteristicsEventHandler(const CYBLE_DISC_CHAR_INFO_T *discCharInfo)
{
    uint8 i;
    static CYBLE_GATT_DB_ATTR_HANDLE_T *hpsLastEndHandle = NULL;

    /* HPS characteristics UUIDs */
    static const CYBLE_UUID16 cyBle_hpscCharUuid[CYBLE_HPS_CHAR_COUNT] =
    {
        CYBLE_UUID_CHAR_URI,
        CYBLE_UUID_CHAR_HTTP_HEADERS,
        CYBLE_UUID_CHAR_HTTP_ENTITY_BODY,
        CYBLE_UUID_CHAR_HTTP_CP,
        CYBLE_UUID_CHAR_HTTP_STATUS_CODE,
        CYBLE_UUID_CHAR_HTTPS_SECURITY
    };

    /* Update last characteristic endHandle to declaration handle of this characteristic */
    if(hpsLastEndHandle != NULL)
    {
        *hpsLastEndHandle = discCharInfo->charDeclHandle - 1u;
        hpsLastEndHandle = NULL;
    }

    /* Search through all available characteristics */
    for(i = (uint8) CYBLE_HPS_URI; (i < (uint8) CYBLE_HPS_CHAR_COUNT); i++)
    {
        if(cyBle_hpscCharUuid[i] == discCharInfo->uuid.uuid16)
        {
            if(cyBle_hpsc.charInfo[i].valueHandle == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
            {
                cyBle_hpsc.charInfo[i].valueHandle = discCharInfo->valueHandle;
                cyBle_hpsc.charInfo[i].properties = discCharInfo->properties;
                hpsLastEndHandle = &cyBle_hpsc.charInfo[i].endHandle;
                break;
            }
        }
    }

    /* Init characteristic endHandle to Service endHandle. Characteristic endHandle
    * will be updated to the declaration handler of the following characteristic,
    * in the following characteristic discovery procedure.
    */
    if(hpsLastEndHandle != NULL)
    {
        *hpsLastEndHandle = cyBle_serverInfo[cyBle_disCount].range.endHandle;
    }
}


/******************************************************************************
* Function Name: CyBle_HpscDiscoverCharDescriptorsEventHandler
***************************************************************************//**
* 
*  This function is called on receiving a CYBLE_EVT_GATTC_FIND_INFO_RSP event.
*  Based on the descriptor UUID, an appropriate data structure is populated 
*  using the data received as part of the callback.
* 
*  \param discDescrInfo: The pointer to a descriptor information structure.
*
******************************************************************************/
void CyBle_HpscDiscoverCharDescriptorsEventHandler(CYBLE_DISC_DESCR_INFO_T *discDescrInfo)
{
    if(discDescrInfo->uuid.uuid16 == CYBLE_UUID_CHAR_CLIENT_CONFIG)
    {
        if(cyBle_hpsc.charInfo[CYBLE_HPS_HTTP_STATUS_CODE].descrHandle[CYBLE_HPS_CCCD] ==
            CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
        {
            cyBle_hpsc.charInfo[CYBLE_HPS_HTTP_STATUS_CODE].descrHandle[CYBLE_HPS_CCCD] = discDescrInfo->descrHandle;

        }
        else    /* Duplication of descriptor */
        {
            CyBle_ApplCallback(CYBLE_EVT_GATTC_DESCR_DUPLICATION, &discDescrInfo->uuid);
        }
    }
}


/******************************************************************************
* Function Name: CyBle_HpscNotificationEventHandler
***************************************************************************//**
* 
*  Handles a notification event for the HTTP Service.
* 
*  \param eventParam: The pointer to the data structure specified by an event.
*
******************************************************************************/
void CyBle_HpscNotificationEventHandler(CYBLE_GATTC_HANDLE_VALUE_NTF_PARAM_T *eventParam)
{
    CYBLE_HPS_CHAR_VALUE_T notifValue;

    if(CyBle_HpsApplCallback != NULL)
    {
        if(cyBle_hpsc.charInfo[CYBLE_HPS_HTTP_STATUS_CODE].valueHandle == eventParam->handleValPair.attrHandle)
        {
            notifValue.connHandle = eventParam->connHandle;
            notifValue.charIndex = CYBLE_HPS_HTTP_STATUS_CODE;
            notifValue.value = &eventParam->handleValPair.value;
            CyBle_HpsApplCallback((uint32) CYBLE_EVT_HPSC_NOTIFICATION, &notifValue);
            cyBle_eventHandlerFlag &= (uint8)~CYBLE_CALLBACK;
        }
    }
}


/******************************************************************************
* Function Name: CyBle_HpscReadResponseEventHandler
***************************************************************************//**
* 
*  Handles a Read Response event for the HTTP Proxy Service.
* 
*  \param eventParam: The pointer to the data structure specified by an event.
*
******************************************************************************/
void CyBle_HpscReadResponseEventHandler(CYBLE_GATTC_READ_RSP_PARAM_T *eventParam)
{
    CYBLE_HPS_CHAR_INDEX_T locCharIndex;

    if((CyBle_HpsApplCallback != NULL) && (cyBle_hpscReqHandle != CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE))
    {
        for(locCharIndex = CYBLE_HPS_URI; (locCharIndex < CYBLE_HPS_CHAR_COUNT); locCharIndex++)
        {
            if(cyBle_hpscReqHandle == cyBle_hpsc.charInfo[locCharIndex].valueHandle)
            {
                break;
            }
        }

        if(locCharIndex < CYBLE_HPS_CHAR_COUNT)
        {
            CYBLE_HPS_CHAR_VALUE_T locCharValue;

            locCharValue.connHandle = eventParam->connHandle;
            locCharValue.charIndex = locCharIndex;
            locCharValue.value = &eventParam->value;
            cyBle_hpscReqHandle = CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE;
            CyBle_HpsApplCallback((uint32) CYBLE_EVT_HPSC_READ_CHAR_RESPONSE, &locCharValue);
            cyBle_eventHandlerFlag &= (uint8)~CYBLE_CALLBACK;
        } 
        else if(cyBle_hpscReqHandle == cyBle_hpsc.charInfo[CYBLE_HPS_HTTP_STATUS_CODE].descrHandle[CYBLE_HPS_CCCD])
        {
            CYBLE_HPS_DESCR_VALUE_T locDescrValue;

            locDescrValue.connHandle = eventParam->connHandle;
            locDescrValue.charIndex = CYBLE_HPS_HTTP_STATUS_CODE;
            locDescrValue.descrIndex = CYBLE_HPS_CCCD;
            locDescrValue.value = &eventParam->value;
            cyBle_hpscReqHandle = CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE;
            CyBle_HpsApplCallback((uint32) CYBLE_EVT_HPSC_READ_DESCR_RESPONSE, &locDescrValue);
            cyBle_eventHandlerFlag &= (uint8)~CYBLE_CALLBACK;
        }
        else
        {
            /* Do nothing */
        }
    }
}


/******************************************************************************
* Function Name: CyBle_HpscReadLongRespEventHandler
***************************************************************************//**
* 
*  Handles a Read Long Response event for the HTTP Proxy Service.
* 
*  \param eventParam: The pointer to the data structure specified by an event.
*
******************************************************************************/
void CyBle_HpscReadLongRespEventHandler(const CYBLE_GATTC_READ_RSP_PARAM_T *eventParam)
{
    uint16 mtuSize;
    uint8 isReqEnded = 1u;
    CYBLE_GATT_VALUE_T rdValue;
    CYBLE_HPS_CHAR_INDEX_T locCharIndex;
    CYBLE_HPS_CHAR_VALUE_T rdCharValue;

    if(CyBle_HpsApplCallback != NULL)
    {
        /* Check if requested handle equals to any of HPS characteristic handles */
        if(cyBle_hpscReqHandle == cyBle_hpsc.charInfo[CYBLE_HPS_URI].valueHandle)
        {
            locCharIndex = CYBLE_HPS_URI;
        }
        else if(cyBle_hpscReqHandle == cyBle_hpsc.charInfo[CYBLE_HPS_HTTP_HEADERS].valueHandle)
        {
            locCharIndex = CYBLE_HPS_HTTP_HEADERS;
        }
        else if(cyBle_hpscReqHandle == cyBle_hpsc.charInfo[CYBLE_HPS_HTTP_ENTITY_BODY].valueHandle)
        {
            locCharIndex = CYBLE_HPS_HTTP_ENTITY_BODY;
        }
        else
        {
            locCharIndex = CYBLE_HPS_CHAR_COUNT;
        }

        /* If match was found then proceed with the request handling */
        if((locCharIndex < CYBLE_HPS_CHAR_COUNT) && (cyBle_hpscReqHandle != CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE))
        {   
            /* Update user buffer with received data */
            if(cyBle_hpscCurrLen < cyBle_hpscRdLongBuffLen)
            {
                (void) memcpy((void *) &cyBle_hpscRdLongBuffPtr[cyBle_hpscCurrLen], 
                            (void *) &eventParam->value.val[0],
                            eventParam->value.len);

                cyBle_hpscCurrLen += eventParam->value.len;
            }

            (void) CyBle_GattGetMtuSize(&mtuSize);

            /* If the received data length is less than the MTU size, the Read Long
            * request is completed or the provided user's buffer is full.
            */
            if(((mtuSize - 1u) > eventParam->value.len))
            {
                cyBle_hpscReqHandle = CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE;
            }
            else if(cyBle_hpscCurrLen == cyBle_hpscRdLongBuffLen)
            {
                CyBle_GattcStopCmd();
            }
            else
            {
                isReqEnded = 0u;
            }

            /* If the buffer is full, then stop processing any remaining read long
			* requests.
			*/
            if(isReqEnded == 1u)
            {
                rdValue.val = cyBle_hpscRdLongBuffPtr;
                rdValue.len = cyBle_hpscCurrLen;

                rdCharValue.connHandle = eventParam->connHandle;
                rdCharValue.charIndex = locCharIndex;
                rdCharValue.value = &rdValue;
                CyBle_HpsApplCallback((uint32) CYBLE_EVT_HPSC_READ_CHAR_RESPONSE, &rdCharValue);
                cyBle_hpscCurrLen = 0u;
            }

            cyBle_eventHandlerFlag &= (uint8)~CYBLE_CALLBACK;
        }
    }
}


/******************************************************************************
* Function Name: CyBle_HpscWriteResponseEventHandler
***************************************************************************//**
* 
*  Handles a Write Response event for the HTTP Proxy Service.
* 
*  \param eventParam: The pointer to a data structure specified by an event.
*
******************************************************************************/
void CyBle_HpscWriteResponseEventHandler(const CYBLE_CONN_HANDLE_T *eventParam)
{
    CYBLE_HPS_CHAR_INDEX_T locCharIndex;

    if((CyBle_HpsApplCallback != NULL) && (cyBle_hpscReqHandle != CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE))
    {
        
        for(locCharIndex = CYBLE_HPS_URI; (locCharIndex < CYBLE_HPS_CHAR_COUNT); locCharIndex++)
        {
            if(cyBle_hpscReqHandle == cyBle_hpsc.charInfo[locCharIndex].valueHandle)
            {
                break;
            }
        }

        if(locCharIndex < CYBLE_HPS_CHAR_COUNT)
        {
            CYBLE_HPS_CHAR_VALUE_T locCharValue;

            locCharValue.connHandle = *eventParam;
            locCharValue.charIndex = locCharIndex;
            locCharValue.value = NULL;
            cyBle_hpscReqHandle = CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE;
            CyBle_HpsApplCallback((uint32) CYBLE_EVT_HPSC_WRITE_CHAR_RESPONSE, &locCharValue);
            cyBle_eventHandlerFlag &= (uint8)~CYBLE_CALLBACK;
        }
        else if(cyBle_hpscReqHandle == cyBle_hpsc.charInfo[CYBLE_HPS_HTTP_STATUS_CODE].descrHandle[CYBLE_HPS_CCCD])
        {
            CYBLE_HPS_DESCR_VALUE_T locDescrValue;
            
            locDescrValue.connHandle = *eventParam;
            locDescrValue.charIndex = CYBLE_HPS_HTTP_STATUS_CODE;
            locDescrValue.descrIndex = CYBLE_HPS_CCCD;
            locDescrValue.value = NULL;
            cyBle_hpscReqHandle = CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE;
            CyBle_HpsApplCallback((uint32) CYBLE_EVT_HPSC_WRITE_DESCR_RESPONSE, &locDescrValue);
            cyBle_eventHandlerFlag &= (uint8)~CYBLE_CALLBACK;
        }
        else
        {
            /* The requested handle doesn't belong to HPS */
        }
    }
}


/******************************************************************************
* Function Name: CyBle_HpscExecuteWriteResponseEventHandler
***************************************************************************//**
* 
*  Handles a Execute Write Response event for the HTTP Proxy Service.
* 
*  \param eventParam: The pointer to a data structure specified by an event.
*
******************************************************************************/
void CyBle_HpscExecuteWriteResponseEventHandler(const CYBLE_GATTC_EXEC_WRITE_RSP_T *eventParam)
{
    CYBLE_HPS_CHAR_INDEX_T locCharIndex;

    /* Check if service handler was registered and request handle is valid. */
    if(CyBle_HpsApplCallback != NULL)
    {
        /* Check if requested handle equals to any of HPS characteristic handles */
        if(cyBle_hpscReqHandle == cyBle_hpsc.charInfo[CYBLE_HPS_URI].valueHandle)
        {
            locCharIndex = CYBLE_HPS_URI;
        }
        else if(cyBle_hpscReqHandle == cyBle_hpsc.charInfo[CYBLE_HPS_HTTP_HEADERS].valueHandle)
        {
            locCharIndex = CYBLE_HPS_HTTP_HEADERS;
        }
        else if(cyBle_hpscReqHandle == cyBle_hpsc.charInfo[CYBLE_HPS_HTTP_ENTITY_BODY].valueHandle)
        {
            locCharIndex = CYBLE_HPS_HTTP_ENTITY_BODY;
        }
        else
        {
            locCharIndex = CYBLE_HPS_CHAR_COUNT;
        }

        if(locCharIndex < CYBLE_HPS_CHAR_COUNT)
        {
            CYBLE_HPS_CHAR_VALUE_T locCharValue;

            locCharValue.connHandle = eventParam->connHandle;
            locCharValue.charIndex = locCharIndex;
            locCharValue.value = NULL;
            cyBle_hpscReqHandle = CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE;
            CyBle_HpsApplCallback((uint32) CYBLE_EVT_HPSC_WRITE_CHAR_RESPONSE, &locCharValue);
            cyBle_eventHandlerFlag &= (uint8)~CYBLE_CALLBACK;
        }
    }
}


/******************************************************************************
* Function Name: CyBle_HpscErrorResponseEventHandler
***************************************************************************//**
* 
*  Handles an Error Response event for the HTTP Proxy Service.
* 
*  \param eventParam - The pointer to a data structure specified by an event.
*
******************************************************************************/
void CyBle_HpscErrorResponseEventHandler(const CYBLE_GATTC_ERR_RSP_PARAM_T *eventParam)
{
    if((eventParam != NULL) && (eventParam->attrHandle == cyBle_hpscReqHandle))
    {
        cyBle_hpscReqHandle = CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE;
    }
}


/******************************************************************************
* Function Name: CyBle_HpscSetCharacteristicValue
***************************************************************************//**
* 
*  This function is used to write the characteristic (which is identified by
*  charIndex) value attribute in the server. As a result a Write Request is 
*  sent to the GATT Server and on successful execution of the request on the 
*  Server side the CYBLE_EVT_HPSS_CHAR_WRITE events is generated.
*  On successful request execution on the Server side the Write Response is 
*  sent to the Client.
* 
*  \param connHandle: The connection handle.
*  \param charIndex:  The index of the service characteristic. Starts with zero.
*  \param attrSize:   The size of the characteristic value attribute.
*  \param attrValue:  The pointer to the characteristic value data that should 
*                     be sent to the server device.
* 
* \return
*  A return value is of type CYBLE_API_RESULT_T.
*
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
*   If the HPS service-specific callback is registered 
*      (with CyBle_HpsRegisterAttrCallback):
*  * CYBLE_EVT_HPSC_WRITE_CHAR_RESPONSE - In case if the requested attribute is
*                                successfully wrote on the peer device,
*                                the details (char index, etc.) are 
*                                provided with event parameter structure
*                                of type CYBLE_HPS_CHAR_VALUE_T.
*
*   Otherwise (if the HPS service-specific callback is not registered):
*  * CYBLE_EVT_GATTC_WRITE_RSP - In case if the requested attribute is 
*                                successfully written on the peer device.
*  * CYBLE_EVT_GATTC_ERROR_RSP - In case if there were some trouble with the 
*                                requested attribute on the peer device,
*                                the details are provided with event parameters 
*                                structure (CYBLE_GATTC_ERR_RSP_PARAM_T).
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_HpscSetCharacteristicValue(CYBLE_CONN_HANDLE_T connHandle, CYBLE_HPS_CHAR_INDEX_T charIndex,
    uint16 attrSize, uint8 *attrValue)
{
    CYBLE_API_RESULT_T apiResult;

    if(CyBle_GetClientState() != CYBLE_CLIENT_STATE_DISCOVERED)
    {
        apiResult = CYBLE_ERROR_INVALID_STATE;
    }
    else if(charIndex >= CYBLE_HPS_CHAR_COUNT)
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
    else if(cyBle_hpsc.charInfo[charIndex].valueHandle != CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
        CYBLE_GATT_HANDLE_VALUE_PAIR_T writeReqParam;

		writeReqParam.attrHandle = cyBle_hpsc.charInfo[charIndex].valueHandle;
        writeReqParam.value.val = attrValue;
        writeReqParam.value.len = attrSize;

        apiResult = CyBle_GattcWriteCharacteristicValue(connHandle, &writeReqParam);
        /* Save handle to support service-specific write response from device */
        if(apiResult == CYBLE_ERROR_OK)
        {
            cyBle_hpscReqHandle = writeReqParam.attrHandle;
        }
    }
    else
    {
        apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
    }

    return (apiResult);
}


/******************************************************************************
* Function Name: CyBle_HpscGetCharacteristicValue
***************************************************************************//**
* 
*  This function is used to read a characteristic value, which is a value
*  identified by charIndex, from the server.
* 
*  \param connHandle: The connection handle.
*  \param charIndex:  The index of the service characteristic. Starts with zero.
* 
* \return
*  A return value is of type CYBLE_API_RESULT_T.
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
*   If the HPS service-specific callback is registered 
*      (with CyBle_HpsRegisterAttrCallback): \n
*  * CYBLE_EVT_HPSC_READ_CHAR_RESPONSE - If the requested attribute is
*                                successfully written on the peer device,
*                                the details (char index , value, etc.) are 
*                                provided with an event parameter structure
*                                of type CYBLE_HPS_CHAR_VALUE_T.
*  .
*   Otherwise (if the HPS service-specific callback is not registered):
*  * CYBLE_EVT_GATTC_READ_RSP - If the requested attribute is 
*                                successfully read on the peer device,
*                                the details (handle, value, etc.) are 
*                                provided with an event parameters 
*                                structure (CYBLE_GATTC_READ_RSP_PARAM_T).
*  * CYBLE_EVT_GATTC_ERROR_RSP - If there is trouble with the 
*                                requested attribute on the peer device,
*                                the details are provided with event parameters 
*                                structure (CYBLE_GATTC_ERR_RSP_PARAM_T).
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_HpscGetCharacteristicValue(CYBLE_CONN_HANDLE_T connHandle, CYBLE_HPS_CHAR_INDEX_T charIndex)
{
    CYBLE_API_RESULT_T apiResult;

    if(CyBle_GetClientState() != CYBLE_CLIENT_STATE_DISCOVERED)
    {
        apiResult = CYBLE_ERROR_INVALID_STATE;
    }
    else if(charIndex >= CYBLE_HPS_CHAR_COUNT)
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
    else if(cyBle_hpsc.charInfo[charIndex].valueHandle != CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
        apiResult = CyBle_GattcReadCharacteristicValue(connHandle, cyBle_hpsc.charInfo[charIndex].valueHandle);

        /* Save handle to support service-specific read response from device */
        if(apiResult == CYBLE_ERROR_OK)
        {
            cyBle_hpscReqHandle = cyBle_hpsc.charInfo[charIndex].valueHandle;
        }
    }
    else
    {
        apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
    }

    return (apiResult);
}


/******************************************************************************
* Function Name: CyBle_HpscSetLongCharacteristicValue
***************************************************************************//**
* 
*  Sends a request to set a long characteristic value of the service, which is
*  a value identified by charIndex, to the server's device.
* 
*  \param connHandle: The connection handle.
*  \param charIndex:  The index of the service characteristic. Starts with zero.
*  \param attrSize:   The size of the characteristic value attribute.
*  \param attrValue:  The pointer to the characteristic value data that should be
*                     sent to the server device.
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
*  In case of successful execution (return value = CYBLE_ERROR_OK)
*  the next events can appear: \n
*   If the HPS service-specific callback is registered 
*      (with CyBle_HpsRegisterAttrCallback):
*  * CYBLE_EVT_HPSC_WRITE_CHAR_RESPONSE - In case if the requested attribute is
*                                successfully wrote on the peer device,
*                                the details (char index, etc.) are 
*                                provided with event parameter structure
*                                of type CYBLE_HPS_CHAR_VALUE_T.
*
*   Otherwise (if the HPS service-specific callback is not registered):
*  * CYBLE_EVT_GATTC_EXEC_WRITE_RSP - In case if the requested attribute is 
*                                successfully wrote on the peer device.
*  * CYBLE_EVT_GATTC_ERROR_RSP - In case if there some trouble with the 
*                                requested attribute on the peer device,
*                                the details are provided with event parameters 
*                                structure (CYBLE_GATTC_ERR_RSP_PARAM_T).
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_HpscSetLongCharacteristicValue(CYBLE_CONN_HANDLE_T connHandle, CYBLE_HPS_CHAR_INDEX_T charIndex,
    uint16 attrSize, uint8 *attrValue)
{
    CYBLE_API_RESULT_T apiResult;
    CYBLE_GATTC_PREP_WRITE_REQ_T writeReqParam;

    if(CyBle_GetClientState() != CYBLE_CLIENT_STATE_DISCOVERED)
    {
        apiResult = CYBLE_ERROR_INVALID_STATE;
    }
    else if(charIndex >= CYBLE_HPS_CHAR_COUNT)
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
    else if(cyBle_hpsc.charInfo[charIndex].valueHandle == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
        apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
    }
    else
    {
        writeReqParam.handleValuePair.attrHandle = cyBle_hpsc.charInfo[charIndex].valueHandle;
        writeReqParam.handleValuePair.value.val = attrValue;
        writeReqParam.handleValuePair.value.len = attrSize;
        writeReqParam.offset = 0u;

        /* ... and send request to server's device. */
        apiResult = CyBle_GattcWriteLongCharacteristicValues(connHandle, &writeReqParam);

        /* Save handle to support service specific write response from device */
        if(apiResult == CYBLE_ERROR_OK)
        {
            cyBle_hpscReqHandle = writeReqParam.handleValuePair.attrHandle;
        }
    }

    return (apiResult);
}


/******************************************************************************
* Function Name: CyBle_HpscGetLongCharacteristicValue
***************************************************************************//**
*
*  This function is used to read a long characteristic value, which is a value
*  identified by charIndex, from the server.
* 
*  \param connHandle: The connection handle.
*  \param charIndex:  The index of the service characteristic. Starts with zero.
*  \param attrSize:   The size of the buffer to store long characteristic value.
*  \param attrValue:  The pointer to the buffer where the read long characteristic 
*                     value should be stored.
* 
* \return
*  A return value is of type CYBLE_API_RESULT_T.
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
*   If the HPS service-specific callback is registered 
*      (with CyBle_HpsRegisterAttrCallback):
*  * CYBLE_EVT_HPSC_READ_CHAR_RESPONSE - If the requested attribute is
*                                successfully written on the peer device,
*                                the details (char index , value, etc.) are 
*                                provided with an event parameter structure
*                                of type CYBLE_HPS_CHAR_VALUE_T.
*
*   Otherwise (if the HPS service-specific callback is not registered):
*  * CYBLE_EVT_GATTC_READ_BLOB_RSP - If the requested attribute is 
*                                successfully read on the peer device,
*                                the details (handle, value, etc.) are 
*                                provided with an event parameters 
*                                structure (CYBLE_GATTC_READ_RSP_PARAM_T).
*  * CYBLE_EVT_GATTC_ERROR_RSP - If there is trouble with the 
*                                requested attribute on the peer device,
*                                the details are provided with event parameters 
*                                structure (CYBLE_GATTC_ERR_RSP_PARAM_T).
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_HpscGetLongCharacteristicValue(CYBLE_CONN_HANDLE_T connHandle, CYBLE_HPS_CHAR_INDEX_T charIndex,
    uint16 attrSize, uint8 *attrValue)
{
    CYBLE_API_RESULT_T apiResult;

    if(CyBle_GetClientState() != CYBLE_CLIENT_STATE_DISCOVERED)
    {
        apiResult = CYBLE_ERROR_INVALID_STATE;
    }
    else if(charIndex >= CYBLE_HPS_CHAR_COUNT)
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
    else if(cyBle_hpsc.charInfo[charIndex].valueHandle == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
        apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
    }
    else
    {
        CYBLE_GATTC_READ_BLOB_REQ_T readLongData;

        readLongData.offset = 0u;
        readLongData.attrHandle = cyBle_hpsc.charInfo[charIndex].valueHandle;

        cyBle_hpscRdLongBuffLen = attrSize;
        cyBle_hpscRdLongBuffPtr = attrValue;

        apiResult = CyBle_GattcReadLongCharacteristicValues(connHandle, &readLongData);

        /* Save handle to support service specific read response from device */
        if(apiResult == CYBLE_ERROR_OK)
        {
            cyBle_hpscReqHandle = cyBle_hpsc.charInfo[charIndex].valueHandle;
        }
    }

    return (apiResult);
}


/******************************************************************************
* Function Name: CyBle_HpscSetCharacteristicDescriptor
***************************************************************************//**
* 
*  This function is used to write the characteristic descriptor to the server,
*  which is identified by charIndex and descrIndex.
*
*  Internally, Write Request is sent to the GATT Server and on successful 
*  execution of the request on the Server side the following events can be 
*  generated: 
*  * CYBLE_EVT_HPSS_NOTIFICATION_ENABLED
*  * CYBLE_EVT_HPSS_NOTIFICATION_DISABLED
* 
*  \param connHandle: The connection handle.
*  \param charIndex:  The index of the service characteristic. Starts with zero.
*  \param descrIndex: The index of the service characteristic descriptor.
*  \param attrSize:   The size of the characteristic value attribute.
*  \param attrValue:  The pointer to the characteristic descriptor value data that
*                     should be sent to the server device.
*
* \return
*  A return value is of type CYBLE_API_RESULT_T.
*
*  * CYBLE_ERROR_OK - The request was sent successfully.
*  * CYBLE_ERROR_INVALID_PARAMETER - Validation of the input parameters failed.
*  * CYBLE_ERROR_INVALID_STATE - The state is not valid.
*  * CYBLE_ERROR_MEMORY_ALLOCATION_FAILED - Memory allocation failed.
*  * CYBLE_ERROR_INVALID_OPERATION - This operation is not permitted on
*                                    the specified attribute.
*  * CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE - The peer device doesn't have
*                                              the particular characteristic
*
* \events
*  In case of successful execution (return value = CYBLE_ERROR_OK)
*  the next events can appear: \n
*   If the HPS service-specific callback is registered 
*      (with CyBle_HpsRegisterAttrCallback):
*  * CYBLE_EVT_HPSC_WRITE_CHAR_RESPONSE - in case if the requested attribute is
*                                successfully wrote on the peer device,
*                                the details (char index, etc.) are 
*                                provided with event parameter structure
*                                of type CYBLE_HPS_CHAR_VALUE_T.
*   Otherwise (if the HPS service-specific callback is not registered):
*  * CYBLE_EVT_GATTC_WRITE_RSP - If the requested attribute is 
*                                successfully written on the peer device.
*  * CYBLE_EVT_GATTC_ERROR_RSP - If there is trouble with the 
*                                requested attribute on the peer device,
*                                the details are provided with event parameters 
*                                structure (CYBLE_GATTC_ERR_RSP_PARAM_T).
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_HpscSetCharacteristicDescriptor(CYBLE_CONN_HANDLE_T connHandle,
    CYBLE_HPS_CHAR_INDEX_T charIndex, CYBLE_HPS_DESCR_INDEX_T descrIndex, uint8 attrSize, uint8 *attrValue)
{
    CYBLE_API_RESULT_T apiResult;

    if(CyBle_GetClientState() != CYBLE_CLIENT_STATE_DISCOVERED)
    {
        apiResult = CYBLE_ERROR_INVALID_STATE;
    }
    else if((charIndex >= CYBLE_HPS_CHAR_COUNT) || (descrIndex >= CYBLE_HPS_DESCR_COUNT))
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
    else if(cyBle_hpsc.charInfo[charIndex].descrHandle[descrIndex] == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
        apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
    }
    else
    {
        CYBLE_GATTC_WRITE_REQ_T writeReqParam;

		/* Fill all fields of write request structure ... */
        writeReqParam.attrHandle = cyBle_hpsc.charInfo[charIndex].descrHandle[descrIndex];
        writeReqParam.value.val = attrValue;
        writeReqParam.value.len = attrSize;

        /* ... and send request to server's device. */
        apiResult = CyBle_GattcWriteCharacteristicDescriptors(connHandle, &writeReqParam);

        /* Save handle to support service specific write response from device */
        if(apiResult == CYBLE_ERROR_OK)
        {
            cyBle_hpscReqHandle = writeReqParam.attrHandle;
        }
    }

    return (apiResult);
}


/******************************************************************************
* Function Name: CyBle_HpscGetCharacteristicDescriptor
***************************************************************************//**
* 
*  Sends a request to get the characteristic descriptor of the specified 
*  characteristic of the service.
* 
*  \param connHandle: The connection handle.
*  \param charIndex:  The index of the service characteristic. Starts with zero.
*  \param descrIndex: The index of the service characteristic descriptor.
* 
* \return
*  * CYBLE_ERROR_OK - The request was sent successfully.
*  * CYBLE_ERROR_INVALID_PARAMETER - Validation of the input parameters failed.
*  * CYBLE_ERROR_INVALID_STATE - The state is not valid.
*  * CYBLE_ERROR_MEMORY_ALLOCATION_FAILED - Memory allocation failed.
*  * CYBLE_ERROR_INVALID_OPERATION - This operation is not permitted on
*                                    the specified attribute.
*  * CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE - The peer device doesn't have
*                                               the particular descriptor
*
* \events
*  In case of successful execution (return value = CYBLE_ERROR_OK)
*  the next events can appear: \n
*  If the HPS service-specific callback is registered 
*      (with CyBle_HpsRegisterAttrCallback):
*  * CYBLE_EVT_HPSC_READ_DESCR_RESPONSE - In case if the requested attribute is
*                                successfully read on the peer device,
*                                the details (char index, descr index, value, etc.) 
*                                are provided with event parameter structure
*                                of type CYBLE_HPS_DESCR_VALUE_T. 
*  .
*  Otherwise (if the HPS service-specific callback is not registered):
*  * CYBLE_EVT_GATTC_READ_RSP -  If the requested attribute is 
*                                successfully read on the peer device,
*                                the details (handle, value, etc.) are 
*                                provided with an event parameters 
*                                structure (CYBLE_GATTC_READ_RSP_PARAM_T).
*  * CYBLE_EVT_GATTC_ERROR_RSP - If there is trouble with the 
*                                requested attribute on the peer device,
*                                the details are provided with event parameters 
*                                structure (CYBLE_GATTC_ERR_RSP_PARAM_T).
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_HpscGetCharacteristicDescriptor(CYBLE_CONN_HANDLE_T connHandle,
    CYBLE_HPS_CHAR_INDEX_T charIndex, CYBLE_HPS_DESCR_INDEX_T descrIndex)
{
    CYBLE_API_RESULT_T apiResult;

    if(CyBle_GetClientState() != CYBLE_CLIENT_STATE_DISCOVERED)
    {
        apiResult = CYBLE_ERROR_INVALID_STATE;
    }
    else if((charIndex >= CYBLE_HPS_CHAR_COUNT) || (descrIndex >= CYBLE_HPS_DESCR_COUNT))
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
    else if(cyBle_hpsc.charInfo[charIndex].descrHandle[descrIndex] == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
        apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
    }
    else
    {
        apiResult = CyBle_GattcReadCharacteristicDescriptors(connHandle,
                                cyBle_hpsc.charInfo[charIndex].descrHandle[descrIndex]);

        /* Save handle to support service specific read response from device */
        if(apiResult == CYBLE_ERROR_OK)
        {
            cyBle_hpscReqHandle = cyBle_hpsc.charInfo[charIndex].descrHandle[descrIndex];
        }
    }

    return (apiResult);
}

#endif /* CYBLE_HPS_CLIENT */


/* [] END OF FILE */
