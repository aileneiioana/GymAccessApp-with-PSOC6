/***************************************************************************//**
* \file CYBLE_rtus.c
* \version `$CY_MAJOR_VERSION`.`$CY_MINOR_VERSION`
* 
* \brief
*  Contains the source code for Reference Time Update Service.
* 
********************************************************************************
* 
* \copyright
* Copyright 2014-2018, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "`$INSTANCE_NAME`_eventHandler.h"

static CYBLE_CALLBACK_T CyBle_RtusApplCallback = NULL;

#if defined(CYBLE_RTUS_SERVER)

`$RtusServer`
    
#endif /* CYBLE_RTUS_SERVER */

#if defined(CYBLE_RTUS_CLIENT)

CYBLE_RTUSC_T cyBle_rtusc;
    
static CYBLE_GATT_DB_ATTR_HANDLE_T cyBle_rtuscReqHandle;

#endif /* (CYBLE_RTUS_CLIENT) */


/** \cond IGNORE */
/*******************************************************************************
* Function Name: CyBle_RtusInit
****************************************************************************//**
* 
*  This function initializes the Reference Time Update Service.
* 
*******************************************************************************/
void CyBle_RtusInit(void)
{
#ifdef CYBLE_RTUS_CLIENT

    if(cyBle_serverInfo[CYBLE_SRVI_RTUS].range.startHandle == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
        (void)memset(&cyBle_rtusc, 0, sizeof(cyBle_rtusc));
    }
    
    cyBle_rtuscReqHandle = CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE;

#endif /* CYBLE_RTUS_CLIENT */
/** \endcond */
}


/*******************************************************************************
* Function Name: CyBle_RtusRegisterAttrCallback
****************************************************************************//**
* 
*  Registers a callback function for Reference Time Update Service specific
*  attribute operations.
*  Service specific write requests from peer device will not be handled with
*  unregistered callback function.
* 
*  \param callbackFunc:  An application layer event callback function to receive 
*                     events from the BLE Component. The definition of 
*                     CYBLE_CALLBACK_T for RTUS is: \n
*                     typedef void (* CYBLE_CALLBACK_T) (uint32 eventCode, 
*                                                       void *eventParam)
*                     * eventCode indicates the event that triggered this 
*                       callback.
*                     * eventParam contains the parameters corresponding to the 
*                       current event.
* 
*
*******************************************************************************/
void CyBle_RtusRegisterAttrCallback(CYBLE_CALLBACK_T callbackFunc)
{
    CyBle_RtusApplCallback = callbackFunc;
}


#if defined(CYBLE_RTUS_SERVER)

/*******************************************************************************
* Function Name: CyBle_RtussWriteEventHandler
****************************************************************************//**
* 
*  Handles the Write Without Response Request Event for the Reference Time Update
*  Service.
* 
*  \param CYBLE_GATTS_WRITE_CMD_REQ_PARAM_T * eventParam: The pointer to a data structure
*                                                  specified by the event.
* 
* 
*******************************************************************************/
void CyBle_RtussWriteCmdEventHandler(CYBLE_GATTS_WRITE_CMD_REQ_PARAM_T *eventParam)
{
    CYBLE_RTUS_CHAR_VALUE_T wrCmdParam;
    
    if(NULL != CyBle_RtusApplCallback)
    {
        /* Check if attribute handle is handle of Time Update Control Point
           Characteristic of Reference Time Update Service. */
        if((cyBle_rtuss.timeUpdateCpHandle == eventParam->handleValPair.attrHandle) &&
            (CYBLE_RTUS_TIME_UPDATE_CP_SIZE == eventParam->handleValPair.value.len) &&
                (CYBLE_RTUS_CANCEL_REF_UPDATE >= eventParam->handleValPair.value.val[0u]))
        {   
            /* Input parameters validation passed, so save Time Update Control Point value */
            if(CyBle_GattsWriteAttributeValue(&eventParam->handleValPair,
                                               0u,
                                               &eventParam->connHandle,
                                               CYBLE_GATT_DB_PEER_INITIATED) == CYBLE_GATT_ERR_NONE)
            {
                wrCmdParam.connHandle = eventParam->connHandle;
                wrCmdParam.charIndex = CYBLE_RTUS_TIME_UPDATE_CONTROL_POINT;
                wrCmdParam.value = &eventParam->handleValPair.value;
                
                /* Send callback to user if no error occurred while writing Time Update Control
                * Point.
                */
                CyBle_RtusApplCallback((uint32) CYBLE_EVT_RTUSS_WRITE_CHAR_CMD, (void *) &wrCmdParam);
                cyBle_eventHandlerFlag &= (uint8) (~CYBLE_CALLBACK);
            }
        }
        /* As this handler handles Write Without Response request the error response
          can't be sent for the Client. The erroneous value will be with
          CYBLE_EVT_GATTS_WRITE_CMD_REQ event. User will decide how to handle it. */
    }
}


/*******************************************************************************
* Function Name: CyBle_RtussSetCharacteristicValue
****************************************************************************//**
* 
*  Sets characteristic value of the Reference Time Update Service, which is
*  identified by charIndex in the local database.
* 
*  \param charIndex:  The index of a service characteristic of
*                     type CYBLE_RTUS_CHAR_INDEX_T.
*  \param attrSize:   The size of the characteristic value attribute.
*  \param attrValue: The pointer to the characteristic value
*                     data that should be stored to the GATT database.
* 
* \return
*  Return value is of type CYBLE_API_RESULT_T.
*   * CYBLE_ERROR_OK - the request is handled successfully
*   * CYBLE_ERROR_INVALID_PARAMETER - validation of the input parameters failed
*
*******************************************************************************/
CYBLE_API_RESULT_T CyBle_RtussSetCharacteristicValue(CYBLE_RTUS_CHAR_INDEX_T charIndex,
    uint8 attrSize, uint8 *attrValue)
{
    CYBLE_RTUS_TIME_UPDATE_STATE_T tUState;
    CYBLE_API_RESULT_T apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    CYBLE_GATT_HANDLE_VALUE_PAIR_T locHandleValuePair;

    if((CYBLE_RTUS_CHAR_COUNT > charIndex) )
    {
        if(charIndex == CYBLE_RTUS_TIME_UPDATE_CONTROL_POINT)
        {
            locHandleValuePair.attrHandle = cyBle_rtuss.timeUpdateCpHandle;

            if(*attrValue <= CYBLE_RTUS_CANCEL_REF_UPDATE)
            {
                apiResult = CYBLE_ERROR_OK;
            }
        }
        else
        {
            locHandleValuePair.attrHandle = cyBle_rtuss.timeUpdateStateHandle;

            tUState = *(CYBLE_RTUS_TIME_UPDATE_STATE_T *) attrValue;

            if((tUState.result <= CYBLE_RTUS_RESULT_NO_UPDATE_ATTEMPT) && 
                (tUState.currentState <= CYBLE_RTUS_CURR_STATE_UPDATE_PENDING))
            {
                apiResult = CYBLE_ERROR_OK;
            }
        }
        locHandleValuePair.value.len = attrSize;
        locHandleValuePair.value.val = attrValue;

        /* Store data in database */
        if(apiResult == CYBLE_ERROR_OK)
        {
            if(CYBLE_GATT_ERR_NONE !=
                CyBle_GattsWriteAttributeValue(&locHandleValuePair, 0u, NULL, CYBLE_GATT_DB_LOCALLY_INITIATED))
            {
                apiResult = CYBLE_ERROR_INVALID_PARAMETER;
            }
        }
    }

    return (apiResult);
}


/*******************************************************************************
* Function Name: CyBle_RtussGetCharacteristicValue
****************************************************************************//**
* 
*  Gets a characteristic value of the Reference Time Update Service, which
*  is identified by charIndex.
* 
*  \param charIndex:  The index of a service characteristic of
*                     type CYBLE_RTUS_CHAR_INDEX_T.
* 
*  \param attrSize:   The size of the characteristic value attribute.
* 
*  \param attrValue: The pointer to the location where 
*                     characteristic value data should be stored.
* 
* \return
*  Return value is of type CYBLE_API_RESULT_T.
*   * CYBLE_ERROR_OK - the request is handled successfully;
*   * CYBLE_ERROR_INVALID_PARAMETER - validation of the input parameter failed.
*
*******************************************************************************/
CYBLE_API_RESULT_T CyBle_RtussGetCharacteristicValue(CYBLE_RTUS_CHAR_INDEX_T charIndex,
    uint8 attrSize, uint8 *attrValue)
{
    CYBLE_API_RESULT_T apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    CYBLE_GATT_HANDLE_VALUE_PAIR_T locHandleValuePair;

    if(CYBLE_RTUS_CHAR_COUNT > charIndex)
    {
        /* Read characteristic value from database */
        if(charIndex == CYBLE_RTUS_TIME_UPDATE_CONTROL_POINT)
        {
            locHandleValuePair.attrHandle = cyBle_rtuss.timeUpdateCpHandle;
        }
        else
        {
            locHandleValuePair.attrHandle = cyBle_rtuss.timeUpdateStateHandle;
        }
        locHandleValuePair.value.len = attrSize;
        locHandleValuePair.value.val = attrValue;

        if(CyBle_GattsReadAttributeValue(&locHandleValuePair, NULL, CYBLE_GATT_DB_LOCALLY_INITIATED)
            == CYBLE_GATT_ERR_NONE)
        {
            apiResult = CYBLE_ERROR_OK;
        }
    }
    
    return (apiResult);
}

#endif /* (CYBLE_RTUS_SERVER) */


#if defined(CYBLE_RTUS_CLIENT)

/*******************************************************************************
* Function Name: CyBle_RtuscDiscoverCharacteristicsEventHandler
****************************************************************************//**
* 
*  This function is called on receiving a "CYBLE_EVT_GATTC_READ_BY_TYPE_RSP"
*  event. Based on the service UUID, an appropriate data structure is populated
*  using the data received as part of the callback.
* 
*  \param discCharInfo: The pointer to a characteristic information structure.
*  
* 
*******************************************************************************/
void CyBle_RtuscDiscoverCharacteristicsEventHandler(CYBLE_DISC_CHAR_INFO_T *discCharInfo)
{
    uint8 locCharIndex = (uint8) CYBLE_RTUS_INVALID_CHAR_INDEX;

    /* Using characteristic UUID store handle of requested characteristic */
    switch(discCharInfo->uuid.uuid16)
    {
    case CYBLE_UUID_CHAR_TIME_UPDATE_CONTROL_POINT:
        locCharIndex = (uint8) CYBLE_RTUS_TIME_UPDATE_CONTROL_POINT;
        break;

    case CYBLE_UUID_CHAR_TIME_UPDATE_STATE:
        locCharIndex = (uint8) CYBLE_RTUS_TIME_UPDATE_STATE;
        break;

    default:
        break;
    }

    if(((uint8) CYBLE_RTUS_CHAR_COUNT) > locCharIndex)
    {
        if(cyBle_rtusc.charInfo[locCharIndex].valueHandle == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
        {
            cyBle_rtusc.charInfo[locCharIndex].valueHandle = discCharInfo->valueHandle;
            cyBle_rtusc.charInfo[locCharIndex].properties  = discCharInfo->properties;
        }
        else
        {
            CyBle_ApplCallback(CYBLE_EVT_GATTC_CHAR_DUPLICATION, &discCharInfo->uuid);
        }
    }
}


/*******************************************************************************
* Function Name: CyBle_RtuscReadResponseEventHandler
****************************************************************************//**
* 
*  Handles Read Response Event for Reference Time Update Service.
* 
*  \param eventParam: The pointer to the data that came with a read response for RTUS.
* 
* 
******************************************************************************/
void CyBle_RtuscReadResponseEventHandler(CYBLE_GATTC_READ_RSP_PARAM_T *eventParam)
{
    CYBLE_RTUS_CHAR_VALUE_T rdRspParam;
    
    if((NULL != CyBle_RtusApplCallback) && (CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE != cyBle_rtuscReqHandle))
    {
        if(cyBle_rtusc.charInfo[CYBLE_RTUS_TIME_UPDATE_STATE].valueHandle == cyBle_rtuscReqHandle)
        {
            /* Fill Reference Time Update Service read response parameter structure with
            * characteristic info.
            */
            rdRspParam.connHandle = eventParam->connHandle;
            rdRspParam.charIndex = CYBLE_RTUS_TIME_UPDATE_STATE;
            rdRspParam.value = &eventParam->value;

            CyBle_RtusApplCallback((uint32)CYBLE_EVT_RTUSC_READ_CHAR_RESPONSE, (void *)&rdRspParam);
                
            cyBle_eventHandlerFlag &= (uint8) ~CYBLE_CALLBACK;
            cyBle_rtuscReqHandle = CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE;
        }
    }
}


/*******************************************************************************
* Function Name: CyBle_RtuscSetCharacteristicValue
****************************************************************************//**
* 
*  This function is used to write the characteristic (which is identified by
*  charIndex) value attribute in the server. As a result a Write Request is 
*  sent to the GATT Server and on successful execution of the request on the 
*  Server side the CYBLE_EVT_RTUSS_WRITE_CHAR_CMD event is generated.
* 
*  \param connHandle:    The connection handle.
*  \param charIndex:     The index of a service characteristic.
*  \param attrSize:      Size of the characteristic value attribute.
*  \param attrValue:    Pointer to the characteristic value data 
*                        that should be sent to the server device.
* 
* \return
*  Return value is of type CYBLE_API_RESULT_T.
*   * CYBLE_ERROR_OK - The request was sent successfully.
*   * CYBLE_ERROR_INVALID_STATE - Connection with the Client is not established.
*   * CYBLE_ERROR_INVALID_PARAMETER - Validation of the input parameters failed.
*   * CYBLE_ERROR_MEMORY_ALLOCATION_FAILED - Memory allocation failed.
*   * CYBLE_ERROR_INVALID_OPERATION - Operation is invalid for this 
*                                      characteristic.
*
*******************************************************************************/
CYBLE_API_RESULT_T CyBle_RtuscSetCharacteristicValue(CYBLE_CONN_HANDLE_T connHandle, CYBLE_RTUS_CHAR_INDEX_T charIndex,
    uint8 attrSize, uint8 *attrValue)
{
    CYBLE_GATTC_WRITE_CMD_REQ_T wrReqParam;
    CYBLE_API_RESULT_T apiResult = CYBLE_ERROR_INVALID_PARAMETER;

    if(CyBle_GetClientState() != CYBLE_CLIENT_STATE_DISCOVERED)
    {
        apiResult = CYBLE_ERROR_INVALID_STATE;
    }
    else if((attrValue != NULL) && (charIndex == CYBLE_RTUS_TIME_UPDATE_CONTROL_POINT) &&
        (cyBle_rtusc.charInfo[CYBLE_RTUS_TIME_UPDATE_CONTROL_POINT].valueHandle != CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE))
    {
        /* Fill all fields of write command request structure ... */
        wrReqParam.value.val = attrValue;
        wrReqParam.value.len = attrSize;
        wrReqParam.attrHandle = cyBle_rtusc.charInfo[CYBLE_RTUS_TIME_UPDATE_CONTROL_POINT].valueHandle;

        /* Send request to write characteristic value */
        apiResult = CyBle_GattcWriteWithoutResponse(connHandle, &wrReqParam);
    }
    else
    {
        /* apiResult equals to CYBLE_ERROR_INVALID_PARAMETER */
    }

    return(apiResult);
}


/*******************************************************************************
* Function Name: CyBle_RtuscGetCharacteristicValue
****************************************************************************//**
* 
*  Sends a request to a peer device to set characteristic value of the Reference 
*  Time Update Service, which is identified by charIndex.
* 
*  \param connHandle:    The connection handle.
*  \param charIndex:     The index of a service characteristic.
* 
* \return
*  Return value is of type CYBLE_API_RESULT_T.
*   * CYBLE_ERROR_OK - the request was sent successfully;
*   * CYBLE_ERROR_INVALID_STATE - connection with the Client is not established.
*   * CYBLE_ERROR_INVALID_PARAMETER - validation of the input parameters failed.
*   * CYBLE_ERROR_INVALID_OPERATION - This operation is not permitted
*   * CYBLE_ERROR_MEMORY_ALLOCATION_FAILED - Memory allocation failed
*
* \events
*  In case of successful execution (return value = CYBLE_ERROR_OK)
*  the next events can appear: \n
*   If the RTUS service-specific callback is registered 
*      (with CyBle_RtusRegisterAttrCallback):
*  * CYBLE_EVT_RTUSC_READ_CHAR_RESPONSE - in case if the requested attribute is
*                                successfully wrote on the peer device,
*                                the details (char index , value, etc.) are 
*                                provided with event parameter structure
*                                of type CYBLE_RTUS_CHAR_VALUE_T.
*  .
*   Otherwise (if the RTUS service-specific callback is not registered):
*  * CYBLE_EVT_GATTC_READ_RSP - in case if the requested attribute is 
*                                successfully read on the peer device,
*                                the details (handle, value, etc.) are 
*                                provided with event parameters 
*                                structure (CYBLE_GATTC_READ_RSP_PARAM_T).
*  * CYBLE_EVT_GATTC_ERROR_RSP - in case if there some trouble with the 
*                                requested attribute on the peer device,
*                                the details are provided with event parameters 
*                                structure (CYBLE_GATTC_ERR_RSP_PARAM_T).
*
*******************************************************************************/
CYBLE_API_RESULT_T CyBle_RtuscGetCharacteristicValue(CYBLE_CONN_HANDLE_T connHandle, CYBLE_RTUS_CHAR_INDEX_T charIndex)
{
    CYBLE_API_RESULT_T apiResult = CYBLE_ERROR_OK;

    if(CyBle_GetClientState() != CYBLE_CLIENT_STATE_DISCOVERED)
    {
        apiResult = CYBLE_ERROR_INVALID_STATE;
    }
    else
    {
        if(charIndex == CYBLE_RTUS_TIME_UPDATE_STATE)
        {
            /* Send request to read characteristic value */
            apiResult = CyBle_GattcReadCharacteristicValue(connHandle,
                cyBle_rtusc.charInfo[CYBLE_RTUS_TIME_UPDATE_STATE].valueHandle);
        
            if(apiResult == CYBLE_ERROR_OK)
            {
                cyBle_rtuscReqHandle = cyBle_rtusc.charInfo[CYBLE_RTUS_TIME_UPDATE_STATE].valueHandle;
            }
        }
    }

    return(apiResult);
}


#endif /* CYBLE_RTUS_CLIENT */


/* [] END OF FILE */
