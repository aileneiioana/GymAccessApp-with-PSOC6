/***************************************************************************//**
* \file CYBLE_ndcs.c
* \version `$CY_MAJOR_VERSION`.`$CY_MINOR_VERSION`
* 
* \brief
*  Contains the source code for Next DST Change Service.
* 
********************************************************************************
* \copyright
* Copyright 2014-2018, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "`$INSTANCE_NAME`_eventHandler.h"

#if defined(CYBLE_NDCS_SERVER)

`$NdcsServer`
    
#endif /* CYBLE_NDCS_SERVER */

#if defined(CYBLE_NDCS_CLIENT)

CYBLE_NDCSC_T cyBle_ndcsc;
    
static CYBLE_GATT_DB_ATTR_HANDLE_T cyBle_ndcscReqHandle;
static CYBLE_CALLBACK_T CyBle_NdcsApplCallback = NULL;

#endif /* (CYBLE_NDCS_CLIENT) */


/** \cond IGNORE */
/*******************************************************************************
* Function Name: CyBle_NdcsInit
****************************************************************************//**
* 
*  This function initializes the Next DST Change Service.
* 
*******************************************************************************/
void CyBle_NdcsInit(void)
{
#ifdef CYBLE_NDCS_CLIENT

    if(cyBle_serverInfo[CYBLE_SRVI_NDCS].range.startHandle == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
        (void)memset(&cyBle_ndcsc, 0, sizeof(cyBle_ndcsc));
    }
    cyBle_ndcscReqHandle = CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE;

#endif /* CYBLE_NDCS_CLIENT */
/** \endcond */
}


/*******************************************************************************
* Function Name: CyBle_NdcsRegisterAttrCallback
****************************************************************************//**
* 
*  Registers a callback function for Next DST Change Service specific attribute
*  operations.
*  Service specific write requests from peer device will not be handled with
*  unregistered callback function.
* 
*  \param callbackFunc:  An application layer event callback function to receive 
*                 events from the BLE Component. The definition of 
*                 CYBLE_CALLBACK_T for NDCS is: \n
*                 typedef void (* CYBLE_CALLBACK_T) (uint32 eventCode, 
*                                                   void *eventParam)
*                 * eventCode indicates the event that triggered this 
*                   callback.
*                 * eventParam contains the parameters corresponding to the 
*                   current event.
* 
*******************************************************************************/
void CyBle_NdcsRegisterAttrCallback(CYBLE_CALLBACK_T callbackFunc)
{
#if defined(CYBLE_NDCS_CLIENT)
    CyBle_NdcsApplCallback = callbackFunc;
#else
    if(callbackFunc != NULL) /* Callback doesn't have events in server role */
    {
    }
#endif /* (CYBLE_NDCS_CLIENT) */
}


#if defined(CYBLE_NDCS_SERVER)

/*******************************************************************************
* Function Name: CyBle_NdcssSetCharacteristicValue
****************************************************************************//**
* 
*  Sets characteristic value of the Next DST Change Service, which is identified
*  by charIndex in the local database.
* 
*  \param charIndex: The index of a service characteristic of type CYBLE_NDCS_CHAR_INDEX_T.
*  \param attrSize:  The size of the characteristic value attribute.
*  \param attrValue: The pointer to the characteristic value data that should be stored to the GATT database.
* 
* \return
*  Return value is of type CYBLE_API_RESULT_T.
*   * CYBLE_ERROR_OK - the request is handled successfully;
*   * CYBLE_ERROR_INVALID_PARAMETER - validation of the input parameters failed.
*
*/
CYBLE_API_RESULT_T CyBle_NdcssSetCharacteristicValue(CYBLE_NDCS_CHAR_INDEX_T charIndex,
    uint8 attrSize, uint8 *attrValue)
{
    CYBLE_API_RESULT_T apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    CYBLE_GATT_HANDLE_VALUE_PAIR_T locHandleValuePair;

    if(charIndex == CYBLE_NDCS_TIME_WITH_DST)
    {
        /* Store data in database */
        locHandleValuePair.attrHandle = cyBle_ndcss.timeWithDst;
        locHandleValuePair.value.len = attrSize;
        locHandleValuePair.value.val = attrValue;
        
        if(CyBle_GattsWriteAttributeValue(&locHandleValuePair, 0u, NULL, CYBLE_GATT_DB_LOCALLY_INITIATED)
           == CYBLE_GATT_ERR_NONE)
        {
            apiResult = CYBLE_ERROR_OK;
        }
    }

    return (apiResult);
}


/*******************************************************************************
* Function Name: CyBle_NdcssGetCharacteristicValue
****************************************************************************//**
* 
*  Gets a characteristic value of the Next DST Change Service, which is
*  identified by charIndex.
* 
*  \param charIndex:  The index of a service characteristic of
*                     type CYBLE_NDCS_CHAR_INDEX_T.
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
CYBLE_API_RESULT_T CyBle_NdcssGetCharacteristicValue(CYBLE_NDCS_CHAR_INDEX_T charIndex,
    uint8 attrSize, uint8 *attrValue)
{
    CYBLE_API_RESULT_T apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    CYBLE_GATT_HANDLE_VALUE_PAIR_T locHandleValuePair;

    if(charIndex == CYBLE_NDCS_TIME_WITH_DST)
    {
        /* Read characteristic value from database */
        locHandleValuePair.attrHandle = cyBle_ndcss.timeWithDst;
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

#endif /* CYBLE_NDCS_SERVER */


#if defined(CYBLE_NDCS_CLIENT)

/*******************************************************************************
* Function Name: CyBle_NdcscDiscoverCharacteristicsEventHandler
****************************************************************************//**
* 
*  This function is called on receiving the CYBLE_EVT_GATTC_READ_BY_TYPE_RSP
*  event. Based on the service UUID, an appropriate data structure is populated
*  using the data received as part of the callback.
* 
*  \param discCharInfo: The pointer to a characteristic information structure.
*  
* 
*******************************************************************************/
void CyBle_NdcscDiscoverCharacteristicsEventHandler(CYBLE_DISC_CHAR_INFO_T *discCharInfo)
{
    if(discCharInfo->uuid.uuid16 == CYBLE_UUID_CHAR_TIME_WITH_DST)
    {
        /* Using characteristic UUID store handle of requested characteristic */
        if(cyBle_ndcsc.charInfo[CYBLE_NDCS_TIME_WITH_DST].valueHandle == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
        {
            cyBle_ndcsc.charInfo[CYBLE_NDCS_TIME_WITH_DST].valueHandle = discCharInfo->valueHandle;
            cyBle_ndcsc.charInfo[CYBLE_NDCS_TIME_WITH_DST].properties  = discCharInfo->properties;
        }
        else
        {
            CyBle_ApplCallback(CYBLE_EVT_GATTC_CHAR_DUPLICATION, &discCharInfo->uuid);
        }
    }
}


/*******************************************************************************
* Function Name: CyBle_NdcscReadResponseEventHandler
****************************************************************************//**
* 
*  Handles Read Response Event for Next DST Change Service.
* 
*  \param eventParam: The pointer to the data that came with a read response for NDCS.
* 
*******************************************************************************/
void CyBle_NdcscReadResponseEventHandler(CYBLE_GATTC_READ_RSP_PARAM_T *eventParam)
{
    CYBLE_NDCS_CHAR_VALUE_T rdRspParam;
    
    if((NULL != CyBle_NdcsApplCallback) && (CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE != cyBle_ndcscReqHandle))
    {
        if(cyBle_ndcsc.charInfo[CYBLE_NDCS_TIME_WITH_DST].valueHandle == cyBle_ndcscReqHandle)
        {
            /* Fill Reference Time Update Service read response parameter structure with
            * characteristic info.
            */
            rdRspParam.connHandle = eventParam->connHandle;
            rdRspParam.charIndex = CYBLE_NDCS_TIME_WITH_DST;
            rdRspParam.value = &eventParam->value;

            CyBle_NdcsApplCallback((uint32)CYBLE_EVT_NDCSC_READ_CHAR_RESPONSE, (void *)&rdRspParam);
                
            cyBle_eventHandlerFlag &= (uint8) ~CYBLE_CALLBACK;
            cyBle_ndcscReqHandle = CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE;
        }
    }
}


/*******************************************************************************
* Function Name: CyBle_NdcscGetCharacteristicValue
****************************************************************************//**
* 
*  Sends a request to peer device to set characteristic value of the Next 
*  DST Change Service, which is identified by charIndex.
* 
*  \param connHandle:    The connection handle.
*  \param charIndex:     The index of a service characteristic.
* 
* \return
*  Return value is of type CYBLE_API_RESULT_T.
*   * CYBLE_ERROR_OK - the request was sent successfully.
*   * CYBLE_ERROR_INVALID_STATE - connection with the client is not established.
*   * CYBLE_ERROR_INVALID_PARAMETER - validation of the input parameters failed.
*   * CYBLE_ERROR_MEMORY_ALLOCATION_FAILED - Memory allocation failed.
*   * CYBLE_ERROR_INVALID_OPERATION - Operation is invalid for this 
*                                      characteristic.
*
* \events
*  In case of successful execution (return value = CYBLE_ERROR_OK)
*  the next events can appear: \n
*   If the NDCS service-specific callback is registered 
*      (with CyBle_NdcsRegisterAttrCallback):
*  * CYBLE_EVT_NDCSC_READ_CHAR_RESPONSE - in case if the requested attribute is
*                                successfully wrote on the peer device,
*                                the details (char index , value, etc.) are 
*                                provided with event parameter structure
*                                of type CYBLE_NDCS_CHAR_VALUE_T.
*  .
*   Otherwise (if the NDCS service-specific callback is not registered):
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
CYBLE_API_RESULT_T CyBle_NdcscGetCharacteristicValue(CYBLE_CONN_HANDLE_T connHandle, CYBLE_NDCS_CHAR_INDEX_T charIndex)
{
    CYBLE_API_RESULT_T apiResult = CYBLE_ERROR_INVALID_PARAMETER;

    if(CyBle_GetClientState() != CYBLE_CLIENT_STATE_DISCOVERED)
    {
        apiResult = CYBLE_ERROR_INVALID_STATE;
    }
    else if(charIndex == CYBLE_NDCS_TIME_WITH_DST)
    {
        /* Send request to read characteristic value */
        apiResult = CyBle_GattcReadCharacteristicValue(connHandle,
            cyBle_ndcsc.charInfo[CYBLE_NDCS_TIME_WITH_DST].valueHandle);

        if(apiResult == CYBLE_ERROR_OK)
        {
            cyBle_ndcscReqHandle = cyBle_ndcsc.charInfo[CYBLE_NDCS_TIME_WITH_DST].valueHandle;
        }
    }
    else
    {
        /* apiResult equals to CYBLE_ERROR_INVALID_PARAMETER */
    }

    return(apiResult);
}

#endif /* CYBLE_NDCS_CLIENT */
    

/* [] END OF FILE */
