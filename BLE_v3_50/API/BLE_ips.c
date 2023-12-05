/***************************************************************************//**
* \file CYBLE_ips.c
* \version `$CY_MAJOR_VERSION`.`$CY_MINOR_VERSION`
* 
* \brief
*  Contains the source code for Indoor Positioning Service.
* 
********************************************************************************
* \copyright
* Copyright 2016-2018, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "`$INSTANCE_NAME`_eventHandler.h"
#include "`$INSTANCE_NAME`_ips.h"
 

#ifdef CYBLE_IPS_SERVER
/* Generated code */
`$IpsServer`
/* End of generated code */

#endif /* CYBLE_IPS_SERVER */

#ifdef CYBLE_IPS_CLIENT
    
/* Server's Indoor Positioning Service characteristics GATT DB handles structure */
CYBLE_IPSC_T cyBle_ipsc;


static CYBLE_GATT_DB_ATTR_HANDLE_T cyBle_ipscReqHandle;

/* Read Long Descriptors variables */
static uint8 * cyBle_ipscRdLongBuffPtr = NULL;
static uint8 cyBle_ipscRdLongBuffLen = 0u;
static uint8 cyBle_ipscCurrLen = 0u;

#endif /* (CYBLE_IPS_CLIENT) */

static CYBLE_CALLBACK_T CyBle_IpsApplCallback = NULL;


/******************************************************************************
* Function Name: CyBle_IpsInit
***************************************************************************//**
* 
*  This function initializes the Indoor Positioning Service.
* 
******************************************************************************/
void CyBle_IpsInit(void)
{

#ifdef CYBLE_IPS_CLIENT

    cyBle_ipscRdLongBuffPtr = NULL;
    cyBle_ipscRdLongBuffLen = 0u;
    cyBle_ipscCurrLen = 0u;

    
    if(cyBle_serverInfo[CYBLE_SRVI_IPS].range.startHandle == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
        (void)memset(&cyBle_ipsc, 0, sizeof(cyBle_ipsc));
    }
    cyBle_ipscReqHandle = CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE;    
    

#endif /* CYBLE_IPS_CLIENT */

}

/******************************************************************************
* Function Name: CyBle_IpsRegisterAttrCallback
***************************************************************************//**
* 
*  Registers a callback function for service specific attribute operations.
*  Service specific write requests from peer device will not be handled with
*  unregistered callback function.
* 
*  \param callbackFunc:  An application layer event callback function to receive
*                 events from the BLE Component. The definition of
*                 CYBLE_CALLBACK_T for IPS Service is:\n
*                 typedef void (* CYBLE_CALLBACK_T) (uint32 eventCode,
*                                                    void *eventParam)
*                 * eventCode:  Indicates the event that triggered this callback
*                   (e.g. CYBLE_EVT_IPS_NOTIFICATION_ENABLED).
*                 * eventParam: Contains the parameters corresponding to the
*                   current event. (e.g. Pointer to CYBLE_IPS_CHAR_VALUE_T
*                   structure that contains details of the characteristic
*                   for which the notification enabled event was triggered).
* 
******************************************************************************/
void CyBle_IpsRegisterAttrCallback(CYBLE_CALLBACK_T callbackFunc)
{
    CyBle_IpsApplCallback = callbackFunc;
}


#ifdef CYBLE_IPS_SERVER

/******************************************************************************
* Function Name: CyBle_IpssWriteEventHandler
***************************************************************************//**
* 
*  Handles the Write Request Event.
* 
*  \param eventParam: The pointer to the data structure specified by the event.
* 
* \return
*  CYBLE_GATT_ERR_CODE_T - A function return state if it succeeded
*  (CYBLE_GATT_ERR_NONE) or failed with error codes:
*   * CYBLE_GATTS_ERR_PROCEDURE_ALREADY_IN_PROGRESS.
*   * CYBLE_GATTS_ERR_CCCD_IMPROPERLY_CONFIGURED.
* 
******************************************************************************/
CYBLE_GATT_ERR_CODE_T CyBle_IpssWriteEventHandler(CYBLE_GATTS_WRITE_REQ_PARAM_T *eventParam)
{
    CYBLE_GATT_ERR_CODE_T gattErr = CYBLE_GATT_ERR_NONE;
    
    if(NULL != CyBle_IpsApplCallback)
    {
	    CYBLE_IPS_CHAR_VALUE_T locCharValue;
        locCharValue.connHandle = eventParam->connHandle;
        locCharValue.gattErrorCode = CYBLE_GATT_ERR_NONE;
	
	    
        for(locCharValue.charIndex = CYBLE_IPS_INDOOR_POSITINING_CONFIG; 
            locCharValue.charIndex < CYBLE_IPS_CHAR_COUNT; 
            locCharValue.charIndex++)
        {
            if((eventParam->handleValPair.attrHandle == cyBle_ipss.charInfo[locCharValue.charIndex].charHandle)
            || (eventParam->handleValPair.attrHandle == cyBle_ipss.charInfo[locCharValue.charIndex].descrHandle[CYBLE_IPS_SCCD])
            || (eventParam->handleValPair.attrHandle == cyBle_ipss.charInfo[locCharValue.charIndex].descrHandle[CYBLE_IPS_CEPD]) )
            {
                gattErr = CyBle_GattDbCheckPermission(eventParam->handleValPair.attrHandle, 
                                                     &eventParam->connHandle,
                                                     CYBLE_GATT_DB_WRITE | CYBLE_GATT_DB_PEER_INITIATED);
                
                if(gattErr == CYBLE_GATT_ERR_NONE)
                {
                    locCharValue.value = &eventParam->handleValPair.value;
                    
                    if(!CYBLE_GATT_DB_ATTR_CHECK_PRPTY(eventParam->handleValPair.attrHandle, CYBLE_GATT_DB_CH_PROP_WRITE))
                    {
                        gattErr = CYBLE_GATT_ERR_WRITE_NOT_PERMITTED;
                    }
                    else if(eventParam->handleValPair.value.len > CYBLE_GATT_DB_ATTR_GET_ATTR_GEN_MAX_LEN(eventParam->handleValPair.attrHandle))
                    {
                        gattErr = CYBLE_GATT_ERR_INVALID_ATTRIBUTE_LEN;   
                    }
                    else
                    {
                        CyBle_IpsApplCallback((uint32)CYBLE_EVT_IPSS_WRITE_CHAR, &locCharValue);
                        
                        if(locCharValue.gattErrorCode == CYBLE_GATT_ERR_USER_DATA_ACCESS_NOT_PERMITTED)
        	            {
                            gattErr = CYBLE_GATT_ERR_USER_DATA_ACCESS_NOT_PERMITTED;
                        }
                        else
                        {
                            gattErr = CyBle_GattsWriteAttributeValue(&eventParam->handleValPair, 
        	                      0u, &eventParam->connHandle, CYBLE_GATT_DB_PEER_INITIATED);
                
                            if( (gattErr == CYBLE_GATT_ERR_NONE)
                            && (eventParam->handleValPair.attrHandle == cyBle_ipss.charInfo[locCharValue.charIndex].charHandle) )
                	        {
                                CYBLE_GATT_DB_ATTR_SET_ATTR_GEN_LEN
                                    (cyBle_ipss.charInfo[locCharValue.charIndex].charHandle, locCharValue.value->len);
                                
                                (void)CyBle_IpssSetAdvertisementData();
                	        }
                        }
                    }
                }
                cyBle_eventHandlerFlag &= (uint8)~CYBLE_CALLBACK;
                break;
			}
        }
    }
    
    
    return (gattErr);
}

/******************************************************************************
* Function Name: CyBle_IpssWriteCmdEventHandler
***************************************************************************//**
* 
*  Handles the Write Without Response Request Event.
* 
*  \param eventParam: The pointer to the data structure specified by the event.
* 
******************************************************************************/
void CyBle_IpssWriteCmdEventHandler(CYBLE_GATTS_WRITE_REQ_PARAM_T *eventParam) 
{
    CYBLE_IPS_CHAR_VALUE_T locCharValue;
    CYBLE_GATT_ERR_CODE_T gattErr = CYBLE_GATT_ERR_NONE;
    
    for(locCharValue.charIndex = CYBLE_IPS_INDOOR_POSITINING_CONFIG; 
            locCharValue.charIndex < CYBLE_IPS_CHAR_COUNT; 
            locCharValue.charIndex++)
    {
        if(eventParam->handleValPair.attrHandle == cyBle_ipss.charInfo[locCharValue.charIndex].charHandle)
        {
            if(NULL != CyBle_IpsApplCallback)
            {
                locCharValue.connHandle = eventParam->connHandle;
                locCharValue.value = &eventParam->handleValPair.value;
                gattErr = CyBle_GattsWriteAttributeValue(&eventParam->handleValPair, 
                              0u, &eventParam->connHandle, CYBLE_GATT_DB_PEER_INITIATED);
                if(CYBLE_GATT_ERR_NONE == gattErr)
                {
                    CyBle_IpsApplCallback((uint32)CYBLE_EVT_IPSS_WRITE_CHAR, &locCharValue);
                    
                    (void)CyBle_IpssSetAdvertisementData();
                }
                
            	cyBle_eventHandlerFlag &= (uint8)~CYBLE_CALLBACK;
            }
        }
    }
}

/****************************************************************************** 
* Function Name: CyBle_IpssPrepareWriteRequestEventHandler
***************************************************************************//**
* 
*  Handles the Prepare Write Request Event.
* 
*  \param eventParam: The pointer to the data that comes with a prepare 
*                     write request.
*                                                  
******************************************************************************/
void CyBle_IpssPrepareWriteRequestEventHandler(CYBLE_GATTS_PREP_WRITE_REQ_PARAM_T *eventParam)
{
    CYBLE_IPS_CHAR_INDEX_T locCharIndex;
      
    for(locCharIndex = CYBLE_IPS_INDOOR_POSITINING_CONFIG; locCharIndex < CYBLE_IPS_CHAR_COUNT; locCharIndex++)
    {
        if(eventParam->baseAddr[eventParam->currentPrepWriteReqCount - 1u].handleValuePair.attrHandle == 
                cyBle_ipss.charInfo[locCharIndex].charHandle)
        {
            if((cyBle_ipss.charInfo[locCharIndex].descrHandle[CYBLE_IPS_CEPD] != CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
            && CYBLE_IS_RELIABLE_WRITE_ENABLED(cyBle_ipss.charInfo[locCharIndex].descrHandle[CYBLE_IPS_CEPD]))
            {   
                #ifdef ATT_QUEUED_WRITE_SUPPORT
                    if(eventParam->currentPrepWriteReqCount == CYBLE_GATTS_FIRST_PREP_WRITE_REQ)
                    {
                        CyBle_GattsPrepWriteReqSupport(CYBLE_GATTS_PREP_WRITE_SUPPORT);
                    }
                #endif /* ATT_QUEUED_WRITE_SUPPORT */
            } 
            else
            {
                eventParam->gattErrorCode = CYBLE_GATT_ERR_REQUEST_NOT_SUPPORTED;
            }
            
            /* Indicate that request was handled */
            cyBle_eventHandlerFlag &= (uint8)~CYBLE_CALLBACK;
            break;
        }
    }
}

/****************************************************************************** 
* Function Name: CyBle_IpssExecuteWriteRequestEventHandler
***************************************************************************//**
* 
*  Handles the Execute Write Request Event.
* 
*  \param eventParam: The pointer to the data that came with a write request.
* 
******************************************************************************/
void CyBle_IpssExecuteWriteRequestEventHandler(CYBLE_GATTS_EXEC_WRITE_REQ_T *eventParam)
{
    uint8 locCount;
    CYBLE_IPS_CHAR_VALUE_T locCharValue[CYBLE_IPS_CHAR_COUNT];
    CYBLE_GATT_VALUE_T locGattValue[CYBLE_IPS_CHAR_COUNT];
    CYBLE_IPS_CHAR_INDEX_T locCharIndex;
    
    for(locCharIndex = CYBLE_IPS_INDOOR_POSITINING_CONFIG; locCharIndex < CYBLE_IPS_CHAR_COUNT; locCharIndex++)
    {
        locGattValue[locCharIndex].len = 0u;
        locGattValue[locCharIndex].val = NULL;
    }
    
    for(locCount = 0u; locCount < eventParam->prepWriteReqCount; locCount++)
    {
        for(locCharIndex = CYBLE_IPS_INDOOR_POSITINING_CONFIG; locCharIndex < CYBLE_IPS_CHAR_COUNT; locCharIndex++)
        {
            if(eventParam->baseAddr[locCount].handleValuePair.attrHandle ==
                cyBle_ipss.charInfo[locCharIndex].charHandle)
            {
                locGattValue[locCharIndex].len = eventParam->baseAddr[locCount].offset + 
                                   eventParam->baseAddr[locCount].handleValuePair.value.len;

                if(locGattValue[locCharIndex].val == NULL)
                {
                    locGattValue[locCharIndex].val = eventParam->baseAddr[locCount].handleValuePair.value.val;
                }
                else if(eventParam->baseAddr[locCount].offset == 0u)
                {
                    /* Case when client wants to rewrite value from beginning */
                    locGattValue[locCharIndex].val = eventParam->baseAddr[locCount].handleValuePair.value.val;
                }
                else
                {
                    /* Do nothing */
                }
            }
        }
    }
    
    for(locCharIndex = CYBLE_IPS_INDOOR_POSITINING_CONFIG; locCharIndex < CYBLE_IPS_CHAR_COUNT; locCharIndex++)
    {
        if((locGattValue[locCharIndex].len != 0u) &&
           (locGattValue[locCharIndex].len <= 
            CYBLE_GATT_DB_ATTR_GET_ATTR_GEN_MAX_LEN(cyBle_ipss.charInfo[locCharIndex].charHandle)))
        {
            /* Check the execWriteFlag before execute or cancel write long operation */
            if(eventParam->execWriteFlag == CYBLE_GATT_EXECUTE_WRITE_EXEC_FLAG)
            {
                locCharValue[locCharIndex].gattErrorCode = CYBLE_GATT_ERR_NONE;
                locCharValue[locCharIndex].connHandle = eventParam->connHandle;
                locCharValue[locCharIndex].charIndex = locCharIndex;
                locCharValue[locCharIndex].value = &locGattValue[locCharIndex];
            
                CyBle_IpsApplCallback((uint32)CYBLE_EVT_IPSS_WRITE_CHAR, &locCharValue[locCharIndex]);

                CYBLE_GATT_DB_ATTR_SET_ATTR_GEN_LEN(cyBle_ipss.charInfo[locCharIndex].charHandle,
                                                            locGattValue[locCharIndex].len);    
            }
            
            /* Indicate that request was handled */
            cyBle_eventHandlerFlag &= (uint8)~CYBLE_CALLBACK;
        }
    }
}

/******************************************************************************
* Function Name: CyBle_IpssSetCharacteristicValue
***************************************************************************//**
* 
*  Sets the characteristic value of the service in the local database.
* 
*  \param charIndex: The index of the service characteristic. Starts with zero.
*  \param attrSize:  The size (in bytes) of the characteristic value attribute.
*  \param attrValue: The pointer to the characteristic value data that should 
*                    be stored in the GATT database.
* 
* \return
*  A return value is of type CYBLE_API_RESULT_T.
*  * CYBLE_ERROR_OK - The request handled successfully.
*  * CYBLE_ERROR_INVALID_PARAMETER - Validation of the input parameter failed.
*  * CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE - An optional characteristic is
*                                              absent.
******************************************************************************/
CYBLE_API_RESULT_T CyBle_IpssSetCharacteristicValue(CYBLE_IPS_CHAR_INDEX_T charIndex,
    uint8 attrSize, uint8 *attrValue)
{
    CYBLE_API_RESULT_T apiResult = CYBLE_ERROR_OK;

    if(charIndex >= CYBLE_IPS_CHAR_COUNT)
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
    else if(CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE == cyBle_ipss.charInfo[charIndex].charHandle)
    {
        apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
    }
    else
    {
        CYBLE_GATT_HANDLE_VALUE_PAIR_T locHandleValuePair;
        
        /* Store characteristic value into GATT database */
        locHandleValuePair.attrHandle = cyBle_ipss.charInfo[charIndex].charHandle;
        locHandleValuePair.value.len = attrSize;
        locHandleValuePair.value.val = attrValue;
        
        if(CYBLE_GATT_ERR_NONE != CyBle_GattsWriteAttributeValue(&locHandleValuePair,
                                    0u, NULL, CYBLE_GATT_DB_LOCALLY_INITIATED))
        {
            apiResult = CYBLE_ERROR_INVALID_PARAMETER;
        }
        else
        {
            apiResult = CyBle_IpssSetAdvertisementData();    
        }
    }

    return (apiResult);
}

/******************************************************************************
* Function Name: CyBle_IpssGetCharacteristicValue
***************************************************************************//**
* 
*  Gets the characteristic value of the service, which is a value identified by
*  charIndex.
* 
*  \param charIndex: The index of the service characteristic. Starts with zero.
*  \param attrSize:  The size of the characteristic value attribute.
*  \param attrValue: The pointer to the location where characteristic value 
*                    data should be stored.
* 
* \return
*  A return value is of type CYBLE_API_RESULT_T.
*  * CYBLE_ERROR_OK - The request handled successfully
*  * CYBLE_ERROR_INVALID_PARAMETER - Validation of the input parameter failed.
*  * CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE - An optional characteristic is
*                                              absent.
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_IpssGetCharacteristicValue(CYBLE_IPS_CHAR_INDEX_T charIndex, uint8 attrSize, uint8 *attrValue)
{
    CYBLE_API_RESULT_T apiResult;
    
    /* Check parameters */
    if(charIndex >= CYBLE_IPS_CHAR_COUNT)
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
	else if(CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE == cyBle_ipss.charInfo[charIndex].charHandle)
    {
        apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
    }
    else
    {
        CYBLE_GATT_HANDLE_VALUE_PAIR_T locHandleValuePair;
        
        /* Get characteristic value from GATT database */
        locHandleValuePair.attrHandle = cyBle_ipss.charInfo[charIndex].charHandle;
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
* Function Name: CyBle_IpssGetCharacteristicDescriptor
***************************************************************************//**
* 
*  Gets a characteristic descriptor of a specified characteristic of the Indoor 
*  Positioning Service from the local GATT database.
* 
*  \param charIndex:  The index of the characteristic.
*  \param descrIndex: The index of the characteristic descriptor.
*  \param attrSize:   The size of the characteristic descriptor attribute.
*  \param attrValue:  The pointer to the location where characteristic 
*                     descriptor value data should be stored.
* 
* \return
*  Return value is of type CYBLE_API_RESULT_T.
*  * CYBLE_ERROR_OK - The request handled successfully
*  * CYBLE_ERROR_INVALID_PARAMETER - Validation of the input parameter failed.
*  * CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE - Optional descriptor is absent.
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_IpssGetCharacteristicDescriptor(CYBLE_IPS_CHAR_INDEX_T charIndex, 
                     CYBLE_IPS_DESCR_INDEX_T descrIndex, uint8 attrSize, uint8 *attrValue)
{
    CYBLE_API_RESULT_T apiResult = CYBLE_ERROR_OK;
    
    /* Check parameters */
    if((charIndex >= CYBLE_IPS_CHAR_COUNT) || (descrIndex >= CYBLE_IPS_DESCR_COUNT))
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
	else if(CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE == cyBle_ipss.charInfo[charIndex].descrHandle[descrIndex])
    {
        apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
    }
    else
    {
        CYBLE_GATT_HANDLE_VALUE_PAIR_T locHandleValuePair;
        
        /* Get data from database */
        locHandleValuePair.attrHandle = cyBle_ipss.charInfo[charIndex].descrHandle[descrIndex];
        locHandleValuePair.value.len = attrSize;
        locHandleValuePair.value.val = attrValue;

        if(CYBLE_GATT_ERR_NONE != CyBle_GattsReadAttributeValue(&locHandleValuePair,
                                    0u, CYBLE_GATT_DB_LOCALLY_INITIATED))
        {
            apiResult = CYBLE_ERROR_INVALID_PARAMETER;
        }
    }

    return (apiResult);
}

/******************************************************************************
* Function Name: CyBle_IpssSetCharacteristicDescriptor
***************************************************************************//**
* 
*  Set a characteristic descriptor of a specified characteristic of the Indoor 
*  Positioning Service from the local GATT database.
* 
*  \param charIndex:  The index of the characteristic.
*  \param descrIndex: The index of the characteristic descriptor.
*  \param attrSize:   The size of the characteristic descriptor attribute.
*  \param attrValue:  The pointer to the descriptor value data to 
*                     be stored in the GATT database.
* 
* \return
*  Return value is of type CYBLE_API_RESULT_T.
*  * CYBLE_ERROR_OK - The request handled successfully.
*  * CYBLE_ERROR_INVALID_PARAMETER - Validation of the input parameter failed.
*  * CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE - Optional descriptor is absent.
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_IpssSetCharacteristicDescriptor(CYBLE_IPS_CHAR_INDEX_T charIndex, 
                  CYBLE_IPS_DESCR_INDEX_T descrIndex, uint8 attrSize, uint8 *attrValue)
{
    CYBLE_API_RESULT_T apiResult = CYBLE_ERROR_OK;
    
    /* Check parameters */
    if((charIndex >= CYBLE_IPS_CHAR_COUNT) || (descrIndex >= CYBLE_IPS_DESCR_COUNT))
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
	else if(CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE == cyBle_ipss.charInfo[charIndex].descrHandle[descrIndex])
    {
        apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
    }
    else
    {
        CYBLE_GATT_HANDLE_VALUE_PAIR_T locHandleValuePair;
        
        /* Get data from database */
        locHandleValuePair.attrHandle = cyBle_ipss.charInfo[charIndex].descrHandle[descrIndex];
        locHandleValuePair.value.len = attrSize;
        locHandleValuePair.value.val = attrValue;

        if(CYBLE_GATT_ERR_NONE != CyBle_GattsWriteAttributeValue(&locHandleValuePair, 
                                            0u, NULL, CYBLE_GATT_DB_LOCALLY_INITIATED))
        {
            apiResult = CYBLE_ERROR_INVALID_PARAMETER;
        }
        else
        {
            apiResult = CyBle_IpssSetAdvertisementData();    
        }
    }

    return (apiResult);
}

#if(CYBLE_GAP_ROLE_PERIPHERAL)

/******************************************************************************
* Function Name: CyBle_IpssSetAdvertisementData
***************************************************************************//**
*
*  Sets the characteristics values in advertisement packet depending on a value 
*  of Indoor Positioning Configuration characteristic.
*
*  This Function must be called when CyBle_GetBleSsState() returns
*  CYBLE_BLESS_STATE_EVENT_CLOSE state.
* 
* \return
*  A return value is of type CYBLE_API_RESULT_T.
*  * CYBLE_ERROR_OK - The advertisement packet collected successfully.
*  * CYBLE_ERROR_INVALID_PARAMETER - Read operation is not permitted on this 
*                attribute or on NULL pointer, Data length in input 
*                parameter exceeds 31 bytes. 
*  * CYBLE_ERROR_INVALID_OPERATION - The advertisement packet doesn't contain
*                the User List or it is to small or ADV Event is not closed, 
*                BLESS is active or ADV is not enabled.
*
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_IpssSetAdvertisementData(void)
{
    uint8 flag = 0u;
    uint8 adLength = 0u;
    uint8 dataLengthAddress = 0u;
    uint8 byteCounter = 0u;
    CYBLE_API_RESULT_T apiResult = CYBLE_ERROR_OK;
    CYBLE_GAPP_DISC_DATA_T ipssAdvertisementData;
    uint8 ipssConfigFlags;
    CYBLE_GATT_HANDLE_VALUE_PAIR_T locHandleValuePair;
    
    while((byteCounter < cyBle_discoveryData.advDataLen) && (flag == 0u))
    {
        adLength = cyBle_discoveryData.advData[byteCounter];
        
        if(0u != adLength)
        {
            /* Increment byte counter so that it points to AD type field */
            byteCounter++;
            
            if(cyBle_discoveryData.advData[byteCounter] == CYBLE_IPS_AD_TYPE)
            {
                /* Start if a "Indoor Positioning" AD type was found. Set flag and exit the loop. */
                flag = 1u;
                dataLengthAddress = byteCounter - 1u;
            }
            else
            {
                byteCounter += adLength;
            }
        }
        else
        {
            /* End of advertisement data structure was encountered so exit loop. */
            break;
        }
    }

    if(0u != flag)
    {
        /* Increment byte counter to point to Indoor Positioning data */
        byteCounter++;
        
        /* Get Configuration value from GATT database */
        locHandleValuePair.attrHandle = cyBle_ipss.charInfo[CYBLE_IPS_INDOOR_POSITINING_CONFIG].charHandle;
        locHandleValuePair.value.len = 1u;
        locHandleValuePair.value.val = &ipssConfigFlags;
        
        if(CYBLE_GATT_ERR_NONE != CyBle_GattsReadAttributeValue(&locHandleValuePair,
                                    0u, CYBLE_GATT_DB_LOCALLY_INITIATED))
        {
            apiResult = CYBLE_ERROR_INVALID_PARAMETER;
        }
        else if ((ipssConfigFlags & CYBLE_IPS_CHARACTERISTICS_IN_ADVERTISING) != 0u)
        {
            cyBle_discoveryData.advData[byteCounter] = ipssConfigFlags;
            byteCounter++;
            
            if ((ipssConfigFlags & CYBLE_IPS_COORDINATES_IN_ADVERTISING) != 0u)
            {
                if ((ipssConfigFlags & CYBLE_IPS_TYPE_OF_COORDINATE_IN_ADVERTISING) == CYBLE_IPS_WGS84_COORDINATE)
                {  
                    /* CYBLE_IPS_WGS84_COORDINATE */
                    /* Latitude */
                    locHandleValuePair.attrHandle = cyBle_ipss.charInfo[CYBLE_IPS_LATITUDE].charHandle;
                    locHandleValuePair.value.len = 4u;
                    locHandleValuePair.value.val = &cyBle_discoveryData.advData[byteCounter];
                    
                    if(CYBLE_GATT_ERR_NONE != CyBle_GattsReadAttributeValue(&locHandleValuePair,
                                    0u, CYBLE_GATT_DB_LOCALLY_INITIATED))
                    {
                        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
                    }
                    byteCounter += locHandleValuePair.value.len;
                    
                    /* Longitude */
                    locHandleValuePair.attrHandle = cyBle_ipss.charInfo[CYBLE_IPS_LONGITUDE].charHandle;
                    locHandleValuePair.value.len = 4u;
                    locHandleValuePair.value.val = &cyBle_discoveryData.advData[byteCounter];
                   
                    if(CYBLE_GATT_ERR_NONE != CyBle_GattsReadAttributeValue(&locHandleValuePair,
                                    0u, CYBLE_GATT_DB_LOCALLY_INITIATED))
                    {
                        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
                    }
                    byteCounter += locHandleValuePair.value.len;
                    
                }
                else
                {   /*  CYBLE_IPS_LOCAL_COORDINATE  */
                    
                    /* North coordinate */
                    locHandleValuePair.attrHandle = cyBle_ipss.charInfo[CYBLE_IPS_LOCAL_NORTH_COORDINATE].charHandle;
                    locHandleValuePair.value.len = 2u;
                    locHandleValuePair.value.val = &cyBle_discoveryData.advData[byteCounter];

                    if(CYBLE_GATT_ERR_NONE != CyBle_GattsReadAttributeValue(&locHandleValuePair,
                                    0u, CYBLE_GATT_DB_LOCALLY_INITIATED))
                    {
                        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
                    }
                    byteCounter += locHandleValuePair.value.len;
                    
                    /* East coordinate */
                    locHandleValuePair.attrHandle = cyBle_ipss.charInfo[CYBLE_IPS_LOCAL_EAST_COORDINATE].charHandle;
                    locHandleValuePair.value.len = 2u;
                    locHandleValuePair.value.val = &cyBle_discoveryData.advData[byteCounter];

                    if(CYBLE_GATT_ERR_NONE != CyBle_GattsReadAttributeValue(&locHandleValuePair,
                                    0u, CYBLE_GATT_DB_LOCALLY_INITIATED))
                    {
                        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
                    }
                    byteCounter += locHandleValuePair.value.len;
                }
            }
            
            if ((ipssConfigFlags & CYBLE_IPS_TX_POWER_IN_ADVERTISING) != 0u)
            {
                cyBle_discoveryData.advData[byteCounter] = 0xAA;
                byteCounter ++;
            }

            if ((ipssConfigFlags & CYBLE_IPS_FLOOR_NUMBER_IN_ADVERTISING) != 0u)
            {
                locHandleValuePair.attrHandle = cyBle_ipss.charInfo[CYBLE_IPS_FLOOR_NUMBER].charHandle;
                locHandleValuePair.value.len = 1u;
                locHandleValuePair.value.val = &cyBle_discoveryData.advData[byteCounter];

                if(CYBLE_GATT_ERR_NONE != CyBle_GattsReadAttributeValue(&locHandleValuePair,
                                0u, CYBLE_GATT_DB_LOCALLY_INITIATED))
                {
                    apiResult = CYBLE_ERROR_INVALID_PARAMETER;
                }
                byteCounter += locHandleValuePair.value.len;
            }
            
            if ((ipssConfigFlags & CYBLE_IPS_ALTITUDE_IN_ADVERTISING) != 0u)
            {
                locHandleValuePair.attrHandle = cyBle_ipss.charInfo[CYBLE_IPS_ALTITUDE].charHandle;
                locHandleValuePair.value.len = 2u;
                locHandleValuePair.value.val = &cyBle_discoveryData.advData[byteCounter];

                if(CYBLE_GATT_ERR_NONE != CyBle_GattsReadAttributeValue(&locHandleValuePair,
                                0u, CYBLE_GATT_DB_LOCALLY_INITIATED))
                {
                    apiResult = CYBLE_ERROR_INVALID_PARAMETER;
                }
                byteCounter += locHandleValuePair.value.len;
            }
            
            if ((ipssConfigFlags & CYBLE_IPS_UNCERTAINTY_NUMBER_IN_ADVERTISING) != 0u)
            {
                locHandleValuePair.attrHandle = cyBle_ipss.charInfo[CYBLE_IPS_UNCERTAINTY].charHandle;
                locHandleValuePair.value.len = 2u;
                locHandleValuePair.value.val = &cyBle_discoveryData.advData[byteCounter];

                if(CYBLE_GATT_ERR_NONE != CyBle_GattsReadAttributeValue(&locHandleValuePair,
                                0u, CYBLE_GATT_DB_LOCALLY_INITIATED))
                {
                    apiResult = CYBLE_ERROR_INVALID_PARAMETER;
                }
                byteCounter += locHandleValuePair.value.len;
            }
        }    
        else
        {
                    /* Do nothing */
        }
        
        cyBle_discoveryData.advData[dataLengthAddress] = byteCounter - dataLengthAddress - 1u;
        cyBle_discoveryData.advDataLen = byteCounter;

        if(CyBle_GetState() == CYBLE_STATE_ADVERTISING)
        {
            /* The Stack stores the pointer to the advertisement data so
            * need to have a different location with the advertisement data when
            * device in the advertisement mode.
            */
            (void) memcpy((void *) &ipssAdvertisementData, (void *) &cyBle_discoveryData,
                sizeof(CYBLE_GAPP_DISC_DATA_T));
            
            /*  Waiting closing ADV Event
            *   CYBLE_ERROR_INVALID_OPERATION - ADV Event is not closed, BLESS is active or ADV is not enabled.
            */
            do 
            {   
                apiResult = CyBle_GapUpdateAdvData(&ipssAdvertisementData, NULL);
                
            } while (apiResult == CYBLE_ERROR_INVALID_OPERATION);
        }    
    }
    return (apiResult);
}

#endif /* (CYBLE_GAP_ROLE_PERIPHERAL) */

#endif /* CYBLE_IPS_SERVER */

#ifdef CYBLE_IPS_CLIENT
         
/******************************************************************************
* Function Name: CyBle_IpscSetCharacteristicValueWithoutResponse
***************************************************************************//**
* 
*  This function is used to write the characteristic (which is identified by
*  charIndex) value attribute in the server without response.
* 
* 
*  \param connHandle: The connection handle.
*  \param charIndex:  The index of the service characteristic.
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
******************************************************************************/
CYBLE_API_RESULT_T CyBle_IpscSetCharacteristicValueWithoutResponse( CYBLE_CONN_HANDLE_T connHandle,
                                                                    CYBLE_IPS_CHAR_INDEX_T charIndex,
                                                                    uint8 attrSize, uint8 * attrValue)
{
    CYBLE_API_RESULT_T apiResult;
    uint16 locMtu = CYBLE_GATT_MTU;
    (void) CyBle_GattGetMtuSize(&locMtu);
    
    /* Check parameters */
    if(CyBle_GetClientState() != CYBLE_CLIENT_STATE_DISCOVERED)
    {
        apiResult = CYBLE_ERROR_INVALID_STATE;
    }
    else if(charIndex >= CYBLE_IPS_CHAR_COUNT)
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
	else if(cyBle_ipsc.charInfo[charIndex].valueHandle == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
		apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
	}
    else if((CYBLE_CHAR_PROP_WRITE & cyBle_ipsc.charInfo[charIndex].properties) == 0u)
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
        
        writeCmdReqParam.attrHandle = cyBle_ipsc.charInfo[charIndex].valueHandle;
        writeCmdReqParam.value.val = attrValue;
        writeCmdReqParam.value.len = attrSize;

        apiResult =  CyBle_GattcWriteWithoutResponse(connHandle, &writeCmdReqParam);
        
        if(apiResult == CYBLE_ERROR_OK)
        {
            cyBle_ipscReqHandle = cyBle_ipsc.charInfo[charIndex].valueHandle;
        }
    }

    return (apiResult);
}
    
/******************************************************************************
* Function Name: CyBle_IpscSetCharacteristicValue
***************************************************************************//**
* 
*  This function is used to write the characteristic (which is identified by
*  charIndex) value attribute in the server. As a result a Write Request is 
*  sent to the GATT Server and on successful execution of the request on the 
*  Server side the CYBLE_EVT_IPSS_WRITE_CHAR events is generated.
*  On successful request execution on the Server side the Write Response is 
*  sent to the Client.
* 
*  The Write Response just confirms the operation success.
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
*   If the IPS service-specific callback is registered 
*      (with CyBle_IpsRegisterAttrCallback):
*  * CYBLE_EVT_IPSC_WRITE_CHAR_RESPONSE - If the requested attribute is
*                                successfully written on the peer device,
*                                the details (char index, etc.) are 
*                                provided with an event parameter structure
*                                of type CYBLE_IPS_CHAR_VALUE_T.
*  .
*  Otherwise (if the IPS service-specific callback is not registered):
*  * CYBLE_EVT_GATTC_WRITE_RSP - If the requested attribute is 
*                                successfully written on the peer device.
*  * CYBLE_EVT_GATTC_EXEC_WRITE_RSP - If the requested attribute is 
*                                successfully written on the peer device.
*  * CYBLE_EVT_GATTC_ERROR_RSP - If there is some trouble with the 
*                                requested attribute on the peer device,
*                                the details are provided with an event parameter
*                                structure (CYBLE_GATTC_ERR_RSP_PARAM_T).
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_IpscSetCharacteristicValue(CYBLE_CONN_HANDLE_T connHandle,
                                                        CYBLE_IPS_CHAR_INDEX_T charIndex,
                                                            uint8 attrSize, uint8 * attrValue)
{
    CYBLE_API_RESULT_T apiResult;
    uint16 locMtu = CYBLE_GATT_MTU;
    (void) CyBle_GattGetMtuSize(&locMtu);
    
    /* Check parameters */
    if(CyBle_GetClientState() != CYBLE_CLIENT_STATE_DISCOVERED)
    {
        apiResult = CYBLE_ERROR_INVALID_STATE;
    }
    else if(charIndex >= CYBLE_IPS_CHAR_COUNT)
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
	else if(cyBle_ipsc.charInfo[charIndex].valueHandle == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
		apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
	}
    else if((CYBLE_CHAR_PROP_WRITE & cyBle_ipsc.charInfo[charIndex].properties) == 0u)
    {
        apiResult = CYBLE_ERROR_INVALID_OPERATION;
    }
    else if((locMtu - 3u) < attrSize)
    {
        CYBLE_GATTC_PREP_WRITE_REQ_T prepWriteReqParam;
        
        prepWriteReqParam.handleValuePair.attrHandle = cyBle_ipsc.charInfo[charIndex].valueHandle;
        prepWriteReqParam.offset = 0u;
        prepWriteReqParam.handleValuePair.value.val = attrValue;
        prepWriteReqParam.handleValuePair.value.len = attrSize;   
        
        apiResult = CyBle_GattcWriteLongCharacteristicValues(connHandle, &prepWriteReqParam);
        
        if(apiResult == CYBLE_ERROR_OK)
        {
            cyBle_ipscReqHandle = cyBle_ipsc.charInfo[charIndex].valueHandle;
        }
    }
    else 
    {
        CYBLE_GATTC_WRITE_REQ_T writeReqParam;
        
        writeReqParam.attrHandle = cyBle_ipsc.charInfo[charIndex].valueHandle;
        writeReqParam.value.val = attrValue;
        writeReqParam.value.len = attrSize;

        apiResult = CyBle_GattcWriteCharacteristicValue(connHandle, &writeReqParam);
        if(apiResult == CYBLE_ERROR_OK)
        {
            cyBle_ipscReqHandle = cyBle_ipsc.charInfo[charIndex].valueHandle;
        }
    }

    return (apiResult);
}


/******************************************************************************
* Function Name: CyBle_IpscReliableWriteCharacteristicValue
***************************************************************************//**
* 
*  This function is used to perform a reliable write command for the 
*  Indoor Positioning Service (identified by charIndex) value attribute to the server.
* 
*  The Write response just confirms the operation success.
* 
*  \param connHandle: The connection handle.
*  \param charIndex:  The index of a service characteristic.
*  \param attrSize:   The size of the characteristic value attribute.
*  \param attrValue:  The pointer to the characteristic value data that should 
*                     be sent to the server device.
* 
* \return
*  The return value is of type CYBLE_API_RESULT_T.
*  * CYBLE_ERROR_OK - The request was sent successfully.
*  * CYBLE_ERROR_INVALID_PARAMETER - Validation of the input parameters failed.
*  * CYBLE_ERROR_MEMORY_ALLOCATION_FAILED - Memory allocation failed.
*  * CYBLE_ERROR_INVALID_STATE - Connection with the server is not established.
*  * CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE - The peer device doesn't have
*                                               the particular characteristic.
*  * CYBLE_ERROR_INVALID_OPERATION - Operation is invalid for this
*                                     characteristic.
* \events
*  In case of successful execution (return value = CYBLE_ERROR_OK)
*  the next events can appear: \n
*   If the IPS service-specific callback is registered 
*      (with CyBle_IpsRegisterAttrCallback):
*  * CYBLE_EVT_IPSC_WRITE_CHAR_RESPONSE - in case if the requested attribute is
*                                successfully wrote on the peer device,
*                                the details (char index, etc.) are 
*                                provided with event parameter structure
*                                of type CYBLE_IPS_CHAR_VALUE_T.
*  .
*   Otherwise (if the IPS service-specific callback is not registered):
*  * CYBLE_EVT_GATTC_EXEC_WRITE_RSP - in case if the requested attribute is 
*                                successfully wrote on the peer device.
*  * CYBLE_EVT_GATTC_ERROR_RSP - in case if there some trouble with the 
*                                requested attribute on the peer device,
*                                the details are provided with event parameters 
*                                structure (CYBLE_GATTC_ERR_RSP_PARAM_T).
* 
******************************************************************************/
CYBLE_API_RESULT_T CyBle_IpscReliableWriteCharacteristicValue(CYBLE_CONN_HANDLE_T connHandle,
                                                        CYBLE_IPS_CHAR_INDEX_T charIndex,
                                                            uint8 attrSize, uint8 * attrValue)
{
    CYBLE_API_RESULT_T apiResult;
    
    /* Check parameters */
    if(CyBle_GetClientState() != CYBLE_CLIENT_STATE_DISCOVERED)
    {
        apiResult = CYBLE_ERROR_INVALID_STATE;
    }
    else if(charIndex >= CYBLE_IPS_CHAR_COUNT)
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
	else if(cyBle_ipsc.charInfo[charIndex].valueHandle == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
		apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
	}
    else if((CYBLE_CHAR_PROP_EXTENDED_PROPERTIES & cyBle_ipsc.charInfo[charIndex].properties) == 0u)
    {
        apiResult = CYBLE_ERROR_INVALID_OPERATION;
    }
    else
    {
        CYBLE_GATTC_PREP_WRITE_REQ_T prepWriteReqParam;
        
        prepWriteReqParam.handleValuePair.attrHandle = cyBle_ipsc.charInfo[charIndex].valueHandle;
        prepWriteReqParam.handleValuePair.value.val = attrValue;
        prepWriteReqParam.handleValuePair.value.len = attrSize;
        prepWriteReqParam.offset = 0u;

        apiResult = CyBle_GattcReliableWrites(connHandle, &prepWriteReqParam, 1u);
        if(apiResult == CYBLE_ERROR_OK)
        {
            cyBle_ipscReqHandle = cyBle_ipsc.charInfo[charIndex].valueHandle;
        }
    }

    return (apiResult);
}

/******************************************************************************
* Function Name: CyBle_IpscGetCharacteristicValue
***************************************************************************//**
* 
*  This function is used to read the characteristic Value from a server,
*  as identified by its charIndex
* 
*  The Read Response returns the characteristic Value in the Attribute Value
*  parameter.
* 
*  \param connHandle: The connection handle.
*  \param charIndex:  The index of the service characteristic.
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
*   If the IPS service-specific callback is registered 
*      (with CyBle_IpsRegisterAttrCallback):
*  * CYBLE_EVT_IPSC_READ_CHAR_RESPONSE - If the requested attribute is
*                                successfully written on the peer device,
*                                the details (char index , value, etc.) are 
*                                provided with an event parameter structure
*                                of type CYBLE_IPS_CHAR_VALUE_T.
*  .
*   Otherwise (if the IPS service-specific callback is not registered):
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
CYBLE_API_RESULT_T CyBle_IpscGetCharacteristicValue(CYBLE_CONN_HANDLE_T connHandle, CYBLE_IPS_CHAR_INDEX_T charIndex)
{
    CYBLE_API_RESULT_T apiResult;

    /* Check parameters */
    if(CyBle_GetClientState() != CYBLE_CLIENT_STATE_DISCOVERED)
    {
        apiResult = CYBLE_ERROR_INVALID_STATE;
    }
    else if(charIndex >= CYBLE_IPS_CHAR_COUNT)
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
	else if(cyBle_ipsc.charInfo[charIndex].valueHandle == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
		apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
	}
    else if((CYBLE_CHAR_PROP_READ & cyBle_ipsc.charInfo[charIndex].properties) == 0u)
    {
        apiResult = CYBLE_ERROR_INVALID_OPERATION;
    }
    else
    {
        apiResult = CyBle_GattcReadCharacteristicValue(connHandle, cyBle_ipsc.charInfo[charIndex].valueHandle);
    }
    
    if(apiResult == CYBLE_ERROR_OK)
    {
        cyBle_ipscReqHandle = cyBle_ipsc.charInfo[charIndex].valueHandle;
    }

    return (apiResult);
}


/******************************************************************************
* Function Name: CyBle_IpscGetLongCharacteristicValue
***************************************************************************//**
* 
*  Sends a request to read a long characteristic.
* 
*  \param connHandle: The connection handle.
*  \param charIndex:  The index of the service characteristic.
*  \param attrSize:   The size of the characteristic value attribute.
*  \param attrValue:  The pointer to the buffer where the read long characteristic 
*                     descriptor value should be stored.
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
*                                    characteristic
*
* \events
*  In the case of successful execution (return value = CYBLE_ERROR_OK)
*  the next events can appear: \n
*   If the IPS service-specific callback is registered 
*      (with CyBle_IpsRegisterAttrCallback):
*  * CYBLE_EVT_IPSC_READ_CHAR_RESPONSE - If the requested attribute is
*                                successfully written on the peer device,
*                                the details (char index , value, etc.) are 
*                                provided with an event parameter structure
*                                of type CYBLE_IPS_CHAR_VALUE_T.
*  .
*   Otherwise (if the IPS service-specific callback is not registered):
*  * CYBLE_EVT_GATTC_READ_BLOB_RSP - If the requested attribute is 
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
CYBLE_API_RESULT_T CyBle_IpscGetLongCharacteristicValue(CYBLE_CONN_HANDLE_T connHandle, CYBLE_IPS_CHAR_INDEX_T charIndex, 
                                                        uint16 attrSize,
                                                        uint8 *attrValue)
{
    CYBLE_API_RESULT_T apiResult;

    /* Check parameters */
    if(CyBle_GetClientState() != CYBLE_CLIENT_STATE_DISCOVERED)
    {
        apiResult = CYBLE_ERROR_INVALID_STATE;
    }
    else if((attrSize == 0u) || (attrValue == NULL))
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
	else if(cyBle_ipsc.charInfo[charIndex].valueHandle == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
		apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
	}
    else if((CYBLE_CHAR_PROP_READ & cyBle_ipsc.charInfo[charIndex].properties) == 0u)
    {
        apiResult = CYBLE_ERROR_INVALID_OPERATION;
    }
    else
    {
        CYBLE_GATTC_READ_BLOB_REQ_T	readBlobReqParam;
        readBlobReqParam.attrHandle = cyBle_ipsc.charInfo[charIndex].valueHandle;
        readBlobReqParam.offset = 0u;
        
        apiResult = CyBle_GattcReadLongCharacteristicValues(connHandle, &readBlobReqParam);
    }
    
    if(apiResult == CYBLE_ERROR_OK)
    {
        cyBle_ipscReqHandle = cyBle_ipsc.charInfo[charIndex].valueHandle;
        cyBle_ipscRdLongBuffLen = attrSize;
        cyBle_ipscRdLongBuffPtr = attrValue;
        cyBle_ipscCurrLen = 0u;
    }

    return (apiResult);
}


/******************************************************************************
* Function Name: CyBle_IpscGetMultipleCharacteristicValues
***************************************************************************//**
* 
*  This function reads multiple Characteristic Values from a GATT Server when
*  the GATT Client knows the Characteristic value handles. This is a 
*  non-blocking function.
* 
*  Internally, Read Multiple Request is sent to the peer device in response to
*  which Read Multiple Response is received. This results in 
*  CYBLE_EVT_GATTC_READ_MULTI_RSP event, which is propagated to the application
*  layer.
* 
*  An Error Response event is sent by the server (CYBLE_EVT_GATTC_ERROR_RSP) in
*  response to the Read Multiple Request if insufficient authentication, 
*  insufficient authorization, insufficient encryption key size is used by the
*  client, or if a read operation is not permitted on any of the Characteristic
*  values. The Error Code parameter is set as specified in the Attribute
*  Protocol.
* 
*  Refer to Bluetooth 4.1 core specification, Volume 3, Part G, section 4.8.4
*  for more details on the sequence of operations.
* 
*  \param connHandle: Connection handle to identify the peer GATT entity, of 
*                     type CYBLE_CONN_HANDLE_T.
*  \param charIndexesList: Pointer to a list of Characteristic value handles
*  \param numberOfCharIndexes: Number of requested Characteristic handles
* 
* \return
*  CYBLE_API_RESULT_T : Return value indicates if the function succeeded or
*  failed. Following are the possible error codes.
*
*  Errors codes                             | Description
*  ------------                             | -----------
*   CYBLE_ERROR_OK                          | On successful operation
*   CYBLE_ERROR_INVALID_PARAMETER           | 'connHandle' value does not represent any existing entry in the Stack
*   CYBLE_ERROR_INVALID_OPERATION           | This operation is not permitted
*   CYBLE_ERROR_MEMORY_ALLOCATION_FAILED    | Memory allocation failed
*   CYBLE_ERROR_INVALID_STATE               | Connection with the Client is not established.
*   CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE | The peer device doesn't have the particular characteristic.
*
* \events
*  In case of successful execution (return value = CYBLE_ERROR_OK)
*  the next events can appear: \n
*   If the IPS service-specific callback is registered 
*      (with CyBle_IpsRegisterAttrCallback):
*  * CYBLE_EVT_IPSC_READ_MULTIPLE_CHAR_RESPONSE - If the requested attribute is
*                                successfully written on the peer device,
*                                the details (char index , value, etc.) are 
*                                provided with an event parameter structure
*                                of type CYBLE_IPS_CHAR_VALUE_T.
*  .
*   Otherwise (if the IPS service-specific callback is not registered):
*  * CYBLE_EVT_GATTC_READ_MULTI_RSP - If the requested attribute is 
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
CYBLE_API_RESULT_T CyBle_IpscGetMultipleCharacteristicValues(CYBLE_CONN_HANDLE_T connHandle,
                                                const CYBLE_IPS_CHAR_INDEX_T  *charIndexesList,
                                                uint8 numberOfCharIndexes)

{
    CYBLE_API_RESULT_T apiResult = CYBLE_ERROR_OK;
    CYBLE_GATTC_READ_MULT_REQ_T	readMultiReqParam;
    uint16 valueHandles[CYBLE_IPS_CHAR_COUNT];
    uint8 indexInList;
    
    
    /* Check parameters */
    if(CyBle_GetClientState() != CYBLE_CLIENT_STATE_DISCOVERED)
    {
        apiResult = CYBLE_ERROR_INVALID_STATE;
    }
    else 
    {        
        for (indexInList = 0u; (indexInList < numberOfCharIndexes) && (apiResult == CYBLE_ERROR_OK); indexInList++)
        {   
            if(cyBle_ipsc.charInfo[charIndexesList[indexInList]].valueHandle == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
		    {
                apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
	        }
            else if((CYBLE_CHAR_PROP_READ & cyBle_ipsc.charInfo[charIndexesList[indexInList]].properties) == 0u)
            {
                apiResult = CYBLE_ERROR_INVALID_OPERATION;
            }
            else
            {   
                valueHandles[indexInList] = cyBle_ipsc.charInfo[charIndexesList[indexInList]].valueHandle;
            }
        }
        
        if(apiResult == CYBLE_ERROR_OK)
        {
            readMultiReqParam.listCount = numberOfCharIndexes;
            readMultiReqParam.handleList = valueHandles;
            
            apiResult = CyBle_GattcReadMultipleCharacteristicValues(connHandle, &readMultiReqParam);
        }
    }
    
    return (apiResult);
}


/******************************************************************************
* Function Name: CyBle_IpscSetCharacteristicDescriptor
***************************************************************************//**
* 
*  This function is used to write the characteristic Value to the server,
*  as identified by its charIndex.
* 
*  \param connHandle: The connection handle.
*  \param charIndex:  The index of the service characteristic.
*  \param descrIndex: The index of the service characteristic descriptor.
*  \param attrSize:   The size of the characteristic descriptor value 
*                     attribute.
*  \param attrValue:  The pointer to the characteristic descriptor value data
*                     type should be sent to the server device.
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
*  In the case of successful execution (return value = CYBLE_ERROR_OK)
*  the next events can appear: \n
*   If the IPS service-specific callback is registered 
*      (with CyBle_IpsRegisterAttrCallback):
*  * CYBLE_EVT_IPSC_WRITE_DESCR_RESPONSE - If the requested attribute is
*                                successfully written on the peer device,
*                                the details (char index, descr index etc.) are
*                                provided with an event parameter structure
*                                of type CYBLE_IPS_DESCR_VALUE_T.
*  .
*   Otherwise (if the IPS service-specific callback is not registered):
*  * CYBLE_EVT_GATTC_WRITE_RSP - If the requested attribute is 
*                                successfully written on the peer device.
*  * CYBLE_EVT_GATTC_ERROR_RSP - If there is some trouble with the 
*                                requested attribute on the peer device,
*                                the details are provided with an event parameter
*                                structure (CYBLE_GATTC_ERR_RSP_PARAM_T).
*
******************************************************************************/
CYBLE_API_RESULT_T CyBle_IpscSetCharacteristicDescriptor(CYBLE_CONN_HANDLE_T connHandle,
    CYBLE_IPS_CHAR_INDEX_T charIndex, CYBLE_IPS_DESCR_INDEX_T descrIndex, uint8 attrSize, uint8 * attrValue)
{
    CYBLE_API_RESULT_T apiResult;
    
    /* Check parameters */
    if(CyBle_GetClientState() != CYBLE_CLIENT_STATE_DISCOVERED)
    {
        apiResult = CYBLE_ERROR_INVALID_STATE;
    }
    else if((charIndex >= CYBLE_IPS_CHAR_COUNT)
         || (descrIndex >= CYBLE_IPS_DESCR_COUNT)
         || (attrSize != CYBLE_CCCD_LEN))
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
    else if(cyBle_ipsc.charInfo[charIndex].descrHandle[descrIndex] == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
		apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
	}
    else
    {
        CYBLE_GATTC_WRITE_REQ_T writeReqParam;
        
        /* Fill all fields of write request structure ... */
        writeReqParam.attrHandle = cyBle_ipsc.charInfo[charIndex].descrHandle[descrIndex];
        writeReqParam.value.val = attrValue;
        writeReqParam.value.len = CYBLE_CCCD_LEN;

        /* ... and send request to server device. */
        apiResult = CyBle_GattcWriteCharacteristicDescriptors(connHandle, &writeReqParam);
        if(apiResult == CYBLE_ERROR_OK)
        {
            /* Save handle to support service specific read response from device */
            cyBle_ipscReqHandle = writeReqParam.attrHandle;
        }
    }

    return (apiResult);
}


/******************************************************************************
* Function Name: CyBle_IpscGetCharacteristicDescriptor
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
*                                    the specified attribute.
*
* \events
*  In the case of successful execution (return value = CYBLE_ERROR_OK)
*  the next events can appear: \n
*  If the IPS service-specific callback is registered 
*      (with CyBle_IpsRegisterAttrCallback):
*  * CYBLE_EVT_IPSC_READ_DESCR_RESPONSE - If the requested attribute is
*                                successfully written on the peer device,
*                                the details (char index, descr index, value, etc.) 
*                                are provided with an event parameter structure
*                                of type CYBLE_IPS_DESCR_VALUE_T. 
*  .
*  Otherwise (if the IPS service-specific callback is not registered):
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
CYBLE_API_RESULT_T CyBle_IpscGetCharacteristicDescriptor(CYBLE_CONN_HANDLE_T connHandle,
                       CYBLE_IPS_CHAR_INDEX_T charIndex, CYBLE_IPS_DESCR_INDEX_T descrIndex)
{
    CYBLE_API_RESULT_T apiResult;
    
    /* Check parameters */
    if(CyBle_GetClientState() != CYBLE_CLIENT_STATE_DISCOVERED)
    {
        apiResult = CYBLE_ERROR_INVALID_STATE;
    }
    else if((charIndex >= CYBLE_IPS_CHAR_COUNT) || (descrIndex >= CYBLE_IPS_DESCR_COUNT))
    {
        apiResult = CYBLE_ERROR_INVALID_PARAMETER;
    }
    else if(cyBle_ipsc.charInfo[charIndex].descrHandle[descrIndex] == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
    {
		apiResult = CYBLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE;
	}
    else
    {
        apiResult = CyBle_GattcReadCharacteristicDescriptors(connHandle, cyBle_ipsc.charInfo[charIndex].descrHandle[descrIndex]);
        if(apiResult == CYBLE_ERROR_OK)
        {
            cyBle_ipscReqHandle = cyBle_ipsc.charInfo[charIndex].descrHandle[descrIndex];
        }
    }
    
    return (apiResult);
}


/******************************************************************************
* Function Name: CyBle_IpscDiscoverCharacteristicsEventHandler
***************************************************************************//**
* 
*  This function is called on receiving a CYBLE_EVT_GATTC_READ_BY_TYPE_RSP
*  event. Based on the service UUID, an appropriate data structure is populated
*  using the data received as part of the callback.
* 
*  \param discCharInfo: The pointer to a characteristic information structure.
* 
******************************************************************************/
void CyBle_IpscDiscoverCharacteristicsEventHandler(CYBLE_DISC_CHAR_INFO_T *discCharInfo)
{
    /* User Data Service characteristics UUIDs */
    static const CYBLE_UUID16 cyBle_ipscCharUuid[CYBLE_IPS_CHAR_COUNT] =
    {
        CYBLE_UUID_CHAR_INDOOR_POSITINING_CONFIG,         /**< Set of characteristic values included in the Indoor Positioning Service AD type */
        CYBLE_UUID_CHAR_LATITUDE,                         /**< WGS84 North coordinate of the device.*/
        CYBLE_UUID_CHAR_LONGITUDE,                        /**< WGS84 East coordinate of the device.*/
        CYBLE_UUID_CHAR_LOCAL_NORTH_COORDINATE,           /**< North coordinate of the device using local coordinate system. */
        CYBLE_UUID_CHAR_LOCAL_EAST_COORDINATE,            /**< East coordinate of the device using local coordinate system. */
        CYBLE_UUID_CHAR_FLOOR_NUMBER,                     /**< Describes in which floor the device is installed in. */
        CYBLE_UUID_CHAR_ALTITUDE,                         /**< Altitude of the device. */
        CYBLE_UUID_CHAR_UNCERTAINTY,                      /**< Uncertainty of the location information the device exposes. */
        CYBLE_UUID_CHAR_LOCATION_NAME                     /**< Name of the location the device is installed in. */
        
    };
    static CYBLE_GATT_DB_ATTR_HANDLE_T *ipsLastEndHandle = NULL;
    uint8 i;
    
    /* Update last characteristic endHandle to declaration handle of this characteristic */
    if(ipsLastEndHandle != NULL)
    {
        *ipsLastEndHandle = discCharInfo->charDeclHandle - 1u;
        ipsLastEndHandle = NULL;
    }
    
    for(i = 0u; i < (uint8) CYBLE_IPS_CHAR_COUNT; i++)
    {
        if(cyBle_ipscCharUuid[i] == discCharInfo->uuid.uuid16)
        {
            if(cyBle_ipsc.charInfo[i].valueHandle == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
            {
                cyBle_ipsc.charInfo[i].valueHandle = discCharInfo->valueHandle;
                cyBle_ipsc.charInfo[i].properties = discCharInfo->properties;
                ipsLastEndHandle = &cyBle_ipsc.charInfo[i].endHandle;
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
    if(ipsLastEndHandle != NULL)
    {
        *ipsLastEndHandle = cyBle_serverInfo[CYBLE_SRVI_IPS].range.endHandle;
    }
}


/******************************************************************************
* Function Name: CyBle_IpscDiscoverCharDescriptorsEventHandler
***************************************************************************//**
* 
*  This function is called on receiving a CYBLE_EVT_GATTC_READ_BY_TYPE_RSP
*  event. Based on the service UUID, an appropriate data structure is populated
*  using the data received as part of the callback.
* 
*  \param discCharInfo: The pointer to the characteristic information structure.
* 
******************************************************************************/
void CyBle_IpscDiscoverCharDescriptorsEventHandler(CYBLE_IPS_CHAR_INDEX_T discoveryCharIndex, 
                                                   CYBLE_DISC_DESCR_INFO_T *discDescrInfo)
{
    CYBLE_IPS_DESCR_INDEX_T locDescIndex;

    if(discDescrInfo->uuid.uuid16 == CYBLE_UUID_CHAR_SERVER_CONFIG)
    {
        locDescIndex = CYBLE_IPS_SCCD;
    }
    else if(discDescrInfo->uuid.uuid16 == CYBLE_UUID_CHAR_EXTENDED_PROPERTIES)
    {
        locDescIndex = CYBLE_IPS_CEPD;
    }
    else    /* Not supported descriptor */
    {
        locDescIndex = CYBLE_IPS_DESCR_COUNT;
    }

    if(locDescIndex < CYBLE_IPS_DESCR_COUNT)
    {
        if(cyBle_ipsc.charInfo[discoveryCharIndex].descrHandle[locDescIndex] == CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE)
        {
            cyBle_ipsc.charInfo[discoveryCharIndex].descrHandle[locDescIndex] = discDescrInfo->descrHandle;
        }
        else    /* Duplication of descriptor */
        {
            CyBle_ApplCallback(CYBLE_EVT_GATTC_DESCR_DUPLICATION, &discDescrInfo->uuid);
        }
    }
}


/******************************************************************************
* Function Name: CyBle_IpscReadResponseEventHandler
***************************************************************************//**
* 
*  Handles the Read Response Event.
* 
*  \param eventParam - The pointer to the data structure.
* 
******************************************************************************/
void CyBle_IpscReadResponseEventHandler(CYBLE_GATTC_READ_RSP_PARAM_T *eventParam)
{
    uint8 locReqHandle = 0u;
    CYBLE_IPS_CHAR_INDEX_T locCharIndex;

    if((NULL != CyBle_IpsApplCallback) && (CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE != cyBle_ipscReqHandle))
    {
        for(locCharIndex = CYBLE_IPS_INDOOR_POSITINING_CONFIG; (locCharIndex < CYBLE_IPS_CHAR_COUNT) && (locReqHandle == 0u); 
            locCharIndex++)
        {
            if(cyBle_ipscReqHandle == cyBle_ipsc.charInfo[locCharIndex].valueHandle)
            {
                CYBLE_IPS_CHAR_VALUE_T locCharValue;
                
                locCharValue.connHandle = eventParam->connHandle;
                locCharValue.charIndex = locCharIndex;
                locCharValue.value = &eventParam->value;
                cyBle_ipscReqHandle = CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE;
                CyBle_IpsApplCallback((uint32)CYBLE_EVT_IPSC_READ_CHAR_RESPONSE, &locCharValue);
                locReqHandle = 1u;
            }
            else
            {
                CYBLE_IPS_DESCR_INDEX_T locDescIndex;

                for(locDescIndex = CYBLE_IPS_SCCD; (locDescIndex < CYBLE_IPS_DESCR_COUNT) &&
                   (locReqHandle == 0u); locDescIndex++)
                {
                    if(cyBle_ipscReqHandle == cyBle_ipsc.charInfo[locCharIndex].descrHandle[locDescIndex])
                    {
                        CYBLE_IPS_DESCR_VALUE_T locDescrValue;
                        
                        locDescrValue.connHandle = eventParam->connHandle;
                        locDescrValue.charIndex = locCharIndex;
                        locDescrValue.descrIndex = locDescIndex;
                        locDescrValue.value = &eventParam->value;
                        cyBle_ipscReqHandle = CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE;
                        CyBle_IpsApplCallback((uint32)CYBLE_EVT_IPSC_READ_DESCR_RESPONSE, &locDescrValue);
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
* Function Name: CyBle_IpscReadResponseEventHandler
***************************************************************************//**
* 
*  Handles the Read Response Event.
* 
*  \param eventParam - The pointer to the data structure.
* 
******************************************************************************/
void CyBle_IpscReadMultipleResponseEventHandler(CYBLE_GATTC_READ_RSP_PARAM_T *eventParam)
{
    if(NULL != CyBle_IpsApplCallback)
    {
        CyBle_IpsApplCallback((uint32)CYBLE_EVT_IPSC_READ_MULTIPLE_CHAR_RESPONSE, eventParam);
        cyBle_eventHandlerFlag &= (uint8)~CYBLE_CALLBACK;
    }
}

/******************************************************************************
* Function Name: CyBle_IpscReadLongRespEventHandler
***************************************************************************//**
* 
*  Handles a Read Long Response event.
* 
*  \param eventParam: The pointer to the data structure specified by an event.
* 
******************************************************************************/
void CyBle_IpscReadLongRespEventHandler(const CYBLE_GATTC_READ_RSP_PARAM_T *eventParam)
{
    if((NULL != CyBle_IpsApplCallback) && (cyBle_ipscReqHandle != CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE))
    {
        CYBLE_IPS_CHAR_VALUE_T locCharValue;
        CYBLE_GATT_VALUE_T locGattValue;
        /* Go through all long characteristics */
        for(locCharValue.charIndex = CYBLE_IPS_INDOOR_POSITINING_CONFIG;
            locCharValue.charIndex < CYBLE_IPS_CHAR_COUNT;
            locCharValue.charIndex++)
        {
            if(cyBle_ipsc.charInfo[locCharValue.charIndex].valueHandle == cyBle_ipscReqHandle)
            {
                uint8 i;
                
                /* Update user buffer with received data */
                for(i = 0u; i < eventParam->value.len; i++)
                {
                    if(cyBle_ipscCurrLen < cyBle_ipscRdLongBuffLen)
                    {
                        cyBle_ipscRdLongBuffPtr[cyBle_ipscCurrLen] = eventParam->value.val[i];
                        cyBle_ipscCurrLen++;
                    }
                    else
                    {
                        CyBle_GattcStopCmd();
                        cyBle_eventHandlerFlag &= (uint8)~CYBLE_CALLBACK;
                        break;
                    }
                }
                
                if((cyBle_eventHandlerFlag & CYBLE_CALLBACK) != 0u)
                {
                    uint16 mtuSize;
                    (void) CyBle_GattGetMtuSize(&mtuSize);
                    
                    /* If received data length is less than MTU size, Read Long
                    * request is completed or provided user's buffer is full.
                    */
                    locGattValue.val = cyBle_ipscRdLongBuffPtr;
                    locGattValue.len = cyBle_ipscCurrLen;
                    locCharValue.value = &locGattValue;
                    locCharValue.connHandle = eventParam->connHandle;
                    
                    if(((mtuSize - 1u) > eventParam->value.len))
                    {
                        CyBle_IpsApplCallback((uint32) CYBLE_EVT_IPSC_READ_CHAR_RESPONSE, &locCharValue);
                        cyBle_ipscReqHandle = CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE;
                    }
                    else
                    {  
                        CyBle_IpsApplCallback((uint32) CYBLE_EVT_IPSC_READ_BLOB_RSP, &locCharValue);
                    }
                    
                    cyBle_eventHandlerFlag &= (uint8)~CYBLE_CALLBACK;
                }
                
                break;
            }
        }
    }
}


/******************************************************************************
* Function Name: CyBle_IpscWriteResponseEventHandler
***************************************************************************//**
* 
*  Handles the Write Response Event.
* 
*  \param eventParam: The pointer to a data structure specified by the event.
* 
******************************************************************************/
void CyBle_IpscWriteResponseEventHandler(const CYBLE_CONN_HANDLE_T *eventParam)
{
    uint8 locReqHandle = 0u;
    CYBLE_IPS_CHAR_INDEX_T locCharIndex;
    
    if((NULL != CyBle_IpsApplCallback) && (CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE != cyBle_ipscReqHandle))
    {
        for(locCharIndex = CYBLE_IPS_INDOOR_POSITINING_CONFIG; (locCharIndex < CYBLE_IPS_CHAR_COUNT) && (locReqHandle == 0u); 
            locCharIndex++)
        {
            if(cyBle_ipscReqHandle == cyBle_ipsc.charInfo[locCharIndex].valueHandle)
            {
                CYBLE_IPS_CHAR_VALUE_T locCharValue;
                
                locCharValue.connHandle = *eventParam;
                locCharValue.charIndex = locCharIndex;
                locCharValue.value = NULL;
                cyBle_ipscReqHandle = CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE;
                CyBle_IpsApplCallback((uint32)CYBLE_EVT_IPSC_WRITE_CHAR_RESPONSE, &locCharValue);
                locReqHandle = 1u;
            }
            else 
            {
            
                CYBLE_IPS_DESCR_INDEX_T locDescIndex;

                for(locDescIndex = CYBLE_IPS_SCCD; (locDescIndex < CYBLE_IPS_DESCR_COUNT) &&
                   (locReqHandle == 0u); locDescIndex++)
                {
                    if(cyBle_ipscReqHandle == cyBle_ipsc.charInfo[locCharIndex].descrHandle[locDescIndex])
                    {
                        CYBLE_IPS_DESCR_VALUE_T locDescrValue;
                        
                        locDescrValue.connHandle = *eventParam;
                        locDescrValue.charIndex = locCharIndex;
                        locDescrValue.descrIndex = locDescIndex;
                        locDescrValue.value = NULL;
                        cyBle_ipscReqHandle = CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE;
                        CyBle_IpsApplCallback((uint32)CYBLE_EVT_IPSC_WRITE_DESCR_RESPONSE, &locDescrValue);
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
* Function Name: CyBle_IpscExecuteWriteResponseEventHandler
***************************************************************************//**
* 
*  Handles a Execute Write Response event for the User Data Service.
* 
*  \param eventParam: The pointer to a data structure specified by an event.
* 
******************************************************************************/
void CyBle_IpscExecuteWriteResponseEventHandler(const CYBLE_GATTC_EXEC_WRITE_RSP_T *eventParam)
{
    uint8 i;
    CYBLE_IPS_CHAR_VALUE_T locCharVal;

    if((NULL != CyBle_IpsApplCallback) && (CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE != cyBle_ipscReqHandle))
    {
        for(i = 0u; i < ((uint8) CYBLE_IPS_CHAR_COUNT); i++)
        {
            if(cyBle_ipsc.charInfo[i].valueHandle == cyBle_ipscReqHandle)
            {
                locCharVal.connHandle = eventParam->connHandle;
                locCharVal.charIndex = (CYBLE_IPS_CHAR_INDEX_T) i;
                locCharVal.value = NULL;
                cyBle_ipscReqHandle = CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE;
                CyBle_IpsApplCallback((uint32) CYBLE_EVT_IPSC_WRITE_CHAR_RESPONSE, &locCharVal);
                cyBle_eventHandlerFlag &= (uint8)~CYBLE_CALLBACK;
            }
        }
    }
}

/******************************************************************************
* Function Name: CyBle_IpscErrorResponseEventHandler
***************************************************************************//**
* 
*  Handles the Error Response Event.
* 
*  \param eventParam: The pointer to the data structure specified by the event.
* 
******************************************************************************/
void CyBle_IpscErrorResponseEventHandler(const CYBLE_GATTC_ERR_RSP_PARAM_T *eventParam)
{
    if((eventParam != NULL) && (eventParam->attrHandle == cyBle_ipscReqHandle))
    {
        if((eventParam->opCode == CYBLE_GATT_READ_BLOB_REQ) &&
           (eventParam->errorCode == CYBLE_GATT_ERR_ATTRIBUTE_NOT_LONG))
        {
            (void)CyBle_GattcReadCharacteristicValue(eventParam->connHandle, eventParam->attrHandle);
            cyBle_eventHandlerFlag &= (uint8)~CYBLE_CALLBACK;
        }
        else
        {   
            if(CyBle_IpsApplCallback != NULL)
            {
                CYBLE_IPS_CHAR_VALUE_T locGattError;
                locGattError.connHandle = eventParam->connHandle;
                locGattError.value = NULL;
                locGattError.gattErrorCode = eventParam->errorCode;
            
                for(locGattError.charIndex = CYBLE_IPS_INDOOR_POSITINING_CONFIG; 
                    locGattError.charIndex < CYBLE_IPS_CHAR_COUNT; 
                    locGattError.charIndex++)
                {
                    if(cyBle_ipsc.charInfo[locGattError.charIndex].valueHandle == eventParam->attrHandle)
                    {
                        CyBle_IpsApplCallback((uint32)CYBLE_EVT_IPSC_ERROR_RESPONSE, &locGattError);
                        cyBle_eventHandlerFlag &= (uint8)~CYBLE_CALLBACK;
                        break;
                    }
                }
            }
            
            cyBle_ipscReqHandle = CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE;
        }
    }
}

#endif /* CYBLE_IPS_CLIENT */


/* [] END OF FILE */
