/***************************************************************************//**
* \file CYBLE_aios.h
* \version `$CY_MAJOR_VERSION`.`$CY_MINOR_VERSION`
* 
* \brief
*  Contains the function prototypes and constants for the Automation Input
*  Output Service.
* 
********************************************************************************
* \copyright
* Copyright 2016-2018, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/


#if !defined(CY_BLE_CYBLE_AIOS_H)
#define CY_BLE_CYBLE_AIOS_H

#include "`$INSTANCE_NAME`_gatt.h"

/***************************************
* Conditional Compilation Parameters
***************************************/
#ifdef CYBLE_AIOS_CLIENT

#define CYBLE_AIO_TOTAL_CHAR_COUNT            (`$AioscCharCount`)

#endif /* CYBLE_AISS_CLIENT */

/**
 \addtogroup group_service_api_AIOS_definitions
 @{
*/

/***************************************
* Data Struct Definition
***************************************/

/** AIOS Characteristic indexes */
typedef enum
{
    CYBLE_AIOS_DIGITAL,                          /**< AIOS Digital characteristic */
    CYBLE_AIOS_ANALOG,                           /**< AIOS Analog characteristic */
    CYBLE_AIOS_AGGREGATE,                        /**< AIOS Aggregate characteristic */
    CYBLE_AIOS_CHAR_COUNT                        /**< Total count of AIOS characteristics */
}CYBLE_AIOS_CHAR_INDEX_T;

/** AIOS Characteristic Descriptors indexes */
typedef enum
{
    CYBLE_AIOS_CCCD,                             /**< Client Characteristic Configuration Descriptor index */
    CYBLE_AIOS_CHAR_PRESENTATION_FORMAT,         /**< Characteristic Presentation Format Descriptor index */
    CYBLE_AIOS_CHAR_USER_DESCRIPTION_DESCR,      /**< Characteristic User Description Descriptor index */
    CYBLE_AIOS_CHAR_EXTENDED_PROPERTIES,         /**< Characteristic Extended Properties Descriptor index */
    CYBLE_AIOS_VALUE_TRIGGER_SETTINGS,           /**< AIO Value Trigger Settings Descriptor index */
    CYBLE_AIOS_TIME_TRIGGER_SETTINGS,            /**< AIO Time Trigger Settings Descriptor index */
    CYBLE_AIOS_VRD,                              /**< Valid Range Descriptor index */
    CYBLE_AIOS_NUM_OF_DIGITAL_DESCR = 0x06,      /**< Number of Digitals Descriptor index */
    CYBLE_AIOS_DESCR_COUNT                       /**< Total count of descriptors */
}CYBLE_AIOS_DESCR_INDEX_T;

#ifdef CYBLE_AIOS_SERVER

/** AIOS Characteristic with descriptors */
typedef struct
{
    /** Handles of Characteristic value */
    CYBLE_GATT_DB_ATTR_HANDLE_T charHandle;

    /** Array of Descriptor handles */
    CYBLE_GATT_DB_ATTR_HANDLE_T descrHandle[CYBLE_AIOS_DESCR_COUNT];
} CYBLE_AIOSS_CHAR_T;

/** Structure to hold pointer to CYBLE_AIOSS_CHAR_T */
typedef struct
{
    /** Pointer to CYBLE_AIOSS_CHAR_T which holds information about specific
    * AIO Characteristic
    */
    CYBLE_AIOSS_CHAR_T *charInfoPtr;
} CYBLE_AIOSS_CHAR_INFO_PTR_T;

/** Structure with Automation Input Output Service attribute handles */
typedef struct
{
    /** Automation Input Output Service handle */
    CYBLE_GATT_DB_ATTR_HANDLE_T serviceHandle;

    /** Automation Input Output Service Array with pointers to
    * Characteristic handles.
    */
    CYBLE_AIOSS_CHAR_INFO_PTR_T charInfoAddr[CYBLE_AIOS_CHAR_COUNT];
    
} CYBLE_AIOSS_T;

#endif /* CYBLE_AIOS_SERVER */

#ifdef CYBLE_AIOS_CLIENT
    
/** AIOS Characteristic with descriptors */
typedef struct
{
    CYBLE_GATT_DB_ATTR_HANDLE_T valueHandle;                         /**< Handle of characteristic value */
    CYBLE_GATT_DB_ATTR_HANDLE_T endHandle;                           /**< End handle of characteristic */
    CYBLE_GATT_DB_ATTR_HANDLE_T descrHandle[CYBLE_AIOS_DESCR_COUNT]; /**< Array of Descriptor handles */
    uint8  properties;                                               /**< Properties for value field */
} CYBLE_AIOSC_CHAR_T;

/** Structure to hold pointer to CYBLE_AIOSC_CHAR_T */
typedef struct
{
    /** Pointer to CYBLE_AIOSC_CHAR_T which holds information about specific AIO Characteristic. */
    CYBLE_AIOSC_CHAR_T *charInfoPtr;
} CYBLE_AIOSC_CHAR_INFO_PTR_T;

/** Structure with discovered attributes information of Automation Input Output Service. */
typedef struct
{
    /** Automation Input Output Service handle */
    CYBLE_GATT_DB_ATTR_HANDLE_T serviceHandle;

    /** Automation Input Output Service Array with pointers to characteristic information. */
    CYBLE_AIOSC_CHAR_INFO_PTR_T charInfoAddr[CYBLE_AIOS_CHAR_COUNT];
        
} CYBLE_AIOSC_T;

#endif /* CYBLE_AIOS_CLIENT */

/** AIOS Characteristic value parameter structure */
typedef struct
{
    CYBLE_CONN_HANDLE_T connHandle;                         /**< Peer device handle */
    CYBLE_AIOS_CHAR_INDEX_T charIndex;                      /**< Index of service characteristic */
    uint8 charInstance;                                     /**< Instance of specific service characteristic */
    CYBLE_GATT_VALUE_T *value;                              /**< Characteristic value */
    CYBLE_GATT_ERR_CODE_T gattErrorCode;                    /**< GATT error code for access control */
} CYBLE_AIOS_CHAR_VALUE_T;

/** AIOS Characteristic descriptor value parameter structure */
typedef struct
{
    CYBLE_CONN_HANDLE_T connHandle;                         /**< Peer device handle */
    CYBLE_AIOS_CHAR_INDEX_T charIndex;                      /**< Index of service characteristic */
    uint8 charInstance;                                     /**< Instance of specific service characteristic */
    CYBLE_AIOS_DESCR_INDEX_T descrIndex;                    /**< Index of descriptor */
    CYBLE_GATT_ERR_CODE_T gattErrorCode;                    /**< Error code received from application (optional) */
    CYBLE_GATT_VALUE_T *value;                              /**< Characteristic value */
} CYBLE_AIOS_DESCR_VALUE_T;

/** @} */

/***************************************
* API Constants
***************************************/
#ifdef CYBLE_AIOS_SERVER
    
#define CYBLE_AIOS_UNDEFINED_ITEM                            (0x00u)
#define CYBLE_AIOS_CHARACTERISTIC_ITEM                       (0x01u)
#define CYBLE_AIOS_DESCRIPTOR_ITEM                           (0x02u)

#define CYBLE_AIOS_CHAR_PRESENTATION_FORMAT_LEN              (0x07u)
#define CYBLE_AIOS_NUM_OF_DIGITAL_DESCR_LEN                  (0x01u)
#define CYBLE_AIOS_ANALOG_LEN                                (0x02u)

#define CYBLE_AIOS_VALUE_TRIGGER_DESCR_MAX_VALUE             (0x07u)
#define CYBLE_AIOS_TIME_TRIGGER_DESCR_MAX_VALUE              (0x03u)
    
#define CYBLE_AIOS_MAX_CHAR_DESCRIPTION_VALUE                (0xFFFFu)

#endif /* CYBLE_AIOS_SERVER */

/***************************************
* Function Prototypes
***************************************/

/** \addtogroup group_service_api_AIOS_server_client 
@{ 
*/
void CyBle_AiosRegisterAttrCallback(CYBLE_CALLBACK_T callbackFunc);
/** @} */

#ifdef CYBLE_AIOS_SERVER
/**
 \addtogroup group_service_api_AIOS_server
 @{
*/

CYBLE_API_RESULT_T CyBle_AiossSetCharacteristicValue(CYBLE_AIOS_CHAR_INDEX_T charIndex, 
    uint8 charInstance, uint8 attrSize, uint8 *attrValue);
CYBLE_API_RESULT_T CyBle_AiossGetCharacteristicValue(CYBLE_AIOS_CHAR_INDEX_T charIndex, 
    uint8 charInstance, uint8 attrSize, uint8 *attrValue);
CYBLE_API_RESULT_T CyBle_AiossSetCharacteristicDescriptor(CYBLE_AIOS_CHAR_INDEX_T charIndex, 
    uint8 charInstance, CYBLE_AIOS_DESCR_INDEX_T descrIndex, uint8 attrSize, uint8 *attrValue);
CYBLE_API_RESULT_T CyBle_AiossGetCharacteristicDescriptor(CYBLE_AIOS_CHAR_INDEX_T charIndex, 
    uint8 charInstance, CYBLE_AIOS_DESCR_INDEX_T descrIndex, uint8 attrSize, uint8 *attrValue);
CYBLE_API_RESULT_T CyBle_AiossSendNotification(CYBLE_CONN_HANDLE_T connHandle,
    CYBLE_AIOS_CHAR_INDEX_T charIndex, uint8 charInstance, uint8 attrSize, uint8 *attrValue);
CYBLE_API_RESULT_T CyBle_AiossSendIndication(CYBLE_CONN_HANDLE_T connHandle,
    CYBLE_AIOS_CHAR_INDEX_T charIndex, uint8 charInstance, uint8 attrSize, uint8 *attrValue);

#endif /* CYBLE_AIOS_SERVER */

/** @} */

#ifdef CYBLE_AIOS_CLIENT
/**
 \addtogroup group_service_api_AIOS_client
 @{
*/
CYBLE_API_RESULT_T CyBle_AioscSetCharacteristicValueWithoutResponse( CYBLE_CONN_HANDLE_T connHandle,
    CYBLE_AIOS_CHAR_INDEX_T charIndex, uint8 charInstance, uint8 attrSize, uint8 * attrValue);

CYBLE_API_RESULT_T CyBle_AioscSetCharacteristicValue(CYBLE_CONN_HANDLE_T connHandle,
    CYBLE_AIOS_CHAR_INDEX_T charIndex, uint8 charInstance, uint8 attrSize, uint8 *attrValue);

CYBLE_API_RESULT_T CyBle_AioscGetCharacteristicValue(CYBLE_CONN_HANDLE_T connHandle, 
    CYBLE_AIOS_CHAR_INDEX_T charIndex, uint8 charInstance);

CYBLE_API_RESULT_T CyBle_AioscSetCharacteristicDescriptor(CYBLE_CONN_HANDLE_T connHandle,
    CYBLE_AIOS_CHAR_INDEX_T charIndex,uint8 charInstance, CYBLE_AIOS_DESCR_INDEX_T descrIndex, uint8 attrSize, uint8 *attrValue);

CYBLE_API_RESULT_T CyBle_AioscGetCharacteristicDescriptor(CYBLE_CONN_HANDLE_T connHandle,
    CYBLE_AIOS_CHAR_INDEX_T charIndex, uint8 charInstance, CYBLE_AIOS_DESCR_INDEX_T descrIndex);

/** @} */
#endif /* CYBLE_AIOS_CLIENT */


/***************************************
* Private Function Prototypes
***************************************/

/** \cond IGNORE */
void CyBle_AiosInit(void);

#ifdef CYBLE_AIOS_SERVER

CYBLE_GATT_ERR_CODE_T CyBle_AiossWriteEventHandler(CYBLE_GATTS_WRITE_REQ_PARAM_T *eventParam);
void CyBle_AiossConfirmationEventHandler(const CYBLE_CONN_HANDLE_T *eventParam);
CYBLE_GATT_ERR_CODE_T CyBle_AiosSetAggregateChar(void);

#endif /* CYBLE_AIOS_SERVER */

#ifdef CYBLE_AIOS_CLIENT
    
void CyBle_AioscDiscoverCharacteristicsEventHandler(const CYBLE_DISC_CHAR_INFO_T * discCharInfo);
void CyBle_AioscDiscoverCharDescriptorsEventHandler(CYBLE_DISC_DESCR_INFO_T * discDescrInfo);
void CyBle_AioscReadResponseEventHandler(CYBLE_GATTC_READ_RSP_PARAM_T *eventParam);
void CyBle_AioscWriteResponseEventHandler(const CYBLE_CONN_HANDLE_T *eventParam);
void CyBle_AioscErrorResponseEventHandler(const CYBLE_GATTC_ERR_RSP_PARAM_T *eventParam);
void CyBle_AioscNotificationEventHandler(CYBLE_GATTC_HANDLE_VALUE_NTF_PARAM_T *eventParam);
void CyBle_AioscIndicationEventHandler(CYBLE_GATTC_HANDLE_VALUE_IND_PARAM_T *eventParam);

#endif /* CYBLE_AIOS_CLIENT */
/** \endcond */



/***************************************
* External data references
***************************************/

#ifdef CYBLE_AIOS_SERVER

extern const CYBLE_AIOSS_T cyBle_aioss;
extern uint8 cyBle_aiossCharInstances[CYBLE_AIOS_CHAR_COUNT];
    
#endif /* CYBLE_AIOS_SERVER */

#ifdef CYBLE_AIOS_CLIENT
    
extern CYBLE_AIOSC_T cyBle_aiosc;
extern uint8 cyBle_aioscActiveCharIndex;
extern uint8 cyBle_aioscActiveCharInstance;
extern uint8 cyBle_aioscPrevCharInstIndex;
extern uint8 cyBle_aioscCharInstances[CYBLE_AIOS_CHAR_COUNT];

#endif /* (CYBLE_AIOS_CLIENT) */

/***************************************
* Macro Functions
***************************************/

#define CyBle_AiosGet16ByPtr    CyBle_Get16ByPtr

#ifdef CYBLE_AIOS_CLIENT

#define CyBle_AioscGetCharacteristicValueHandle(charIndex, charInstance)    \
    ((((charIndex) >= CYBLE_AIOS_CHAR_COUNT) ||((charInstance) >= cyBle_aiossCharInstances[charIndex])) ? \
            CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE : \
            cyBle_aiosc.charInfoAddr[charIndex].charInfoPtr[charInstance].valueHandle)

#define CyBle_AioscGetCharacteristicDescriptorHandle(charIndex, charInstance, descrIndex)    \
    ((((charIndex) >= CYBLE_AIOS_CHAR_COUNT) ||((charInstance) >= cyBle_aiossCharInstances[charIndex]) || ((descrIndex) >= CYBLE_AIOS_DESCR_COUNT)) ? \
            CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE : \
             cyBle_aiosc.charInfoAddr[charIndex].charInfoPtr[charInstance].descrHandle[descrIndex])

#endif /* (CYBLE_AIOS_CLIENT) */

#endif /* CY_BLE_CYBLE_AIOS_H */


/* [] END OF FILE */
