/***************************************************************************//**
* \file CYBLE_ancs.h
* \version `$CY_MAJOR_VERSION`.`$CY_MINOR_VERSION`
* 
* \brief
*  This file contains the function prototypes and constants used in
*  the Apple Notification Center (ANC) Service of the BLE Component.
* 
********************************************************************************
* \copyright
* Copyright 2014-2018, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/


#if !defined(CY_BLE_CYBLE_ANCS_H)
#define CY_BLE_CYBLE_ANCS_H

#include "`$INSTANCE_NAME`_gatt.h"
    
#define CYBLE_ANCS_FLAG_PROCESS      (0x01u)


/***************************************
* Data Types
***************************************/

/**
 \addtogroup group_service_api_ANCS_definitions
 @{
*/

/** ANC Service Characteristics indexes */
typedef enum
{
    CYBLE_ANCS_NS,          /**< Notification Source characteristic index */
	CYBLE_ANCS_CP,          /**< Control Point characteristic index */
	CYBLE_ANCS_DS,          /**< Data Source characteristic index */
    CYBLE_ANCS_CHAR_COUNT   /**< Total count of ANCS characteristics */
}CYBLE_ANCS_CHAR_INDEX_T;

/** ANC Service Characteristic Descriptors indexes */
typedef enum
{
    CYBLE_ANCS_CCCD,        /**< Client Characteristic Configuration descriptor index */
    CYBLE_ANCS_DESCR_COUNT  /**< Total count of ANCS descriptors */
}CYBLE_ANCS_DESCR_INDEX_T;


#ifdef CYBLE_ANCS_SERVER
/** ANC Service Characteristic structure type */
typedef struct
{
    CYBLE_GATT_DB_ATTR_HANDLE_T charHandle;                          /**< Handle of characteristic value */
    CYBLE_GATT_DB_ATTR_HANDLE_T descrHandle[CYBLE_ANCS_DESCR_COUNT]; /**< Handle of descriptor */
}CYBLE_ANCSS_CHAR_T;
    
/** Structure with ANC Service attribute handles */
typedef struct
{
    /** ANC Service handle */
    CYBLE_GATT_DB_ATTR_HANDLE_T serviceHandle;
    
    /** ANC Service characteristics info array */
    CYBLE_ANCSS_CHAR_T charInfo[CYBLE_ANCS_CHAR_COUNT];
} CYBLE_ANCSS_T;
#endif /* CYBLE_ANCS_SERVER */

#ifdef CYBLE_ANCS_CLIENT
/** ANCS client characteristic structure type */
typedef struct
{
    /** Properties for value field */
    uint8  properties;

    /** Handle of server database attribute value entry */
    CYBLE_GATT_DB_ATTR_HANDLE_T valueHandle;
    
    /** ANCS client char. descriptor handle */
    CYBLE_GATT_DB_ATTR_HANDLE_T descrHandle[CYBLE_ANCS_DESCR_COUNT];
    
    /** Characteristic End Handle */
    CYBLE_GATT_DB_ATTR_HANDLE_T endHandle;
}CYBLE_ANCSC_CHAR_T;

/** Structure with discovered attributes information of ANC Service */
typedef struct
{
    CYBLE_ANCSC_CHAR_T charInfo[CYBLE_ANCS_CHAR_COUNT];   /**< Characteristics handle + properties array */
}CYBLE_ANCSC_T;

#endif /* CYBLE_ANCS_CLIENT */

/** ANCS Characteristic Value parameter structure */
typedef struct
{
    CYBLE_CONN_HANDLE_T connHandle;                         /**< Peer device handle */
    CYBLE_ANCS_CHAR_INDEX_T charIndex;                      /**< Index of service characteristic */
    CYBLE_GATT_VALUE_T *value;                              /**< Characteristic value */
    CYBLE_GATT_ERR_CODE_T gattErrorCode;                    /**< GATT error code for access control */
} CYBLE_ANCS_CHAR_VALUE_T;

/** ANCS Characteristic Descriptor Value parameter structure */
typedef struct
{
    CYBLE_CONN_HANDLE_T connHandle;                         /**< Peer device handle */
    CYBLE_ANCS_CHAR_INDEX_T charIndex;                      /**< Index of service characteristic */
    CYBLE_ANCS_DESCR_INDEX_T descrIndex;                    /**< Index of service characteristic descriptor */
    CYBLE_GATT_VALUE_T *value;                              /**< Descriptor value */
} CYBLE_ANCS_DESCR_VALUE_T;

/** @} */

/***************************************
* Function Prototypes
***************************************/

/** \addtogroup group_service_api_ANCS_server_client 
@{ 
*/
void CyBle_AncsRegisterAttrCallback(CYBLE_CALLBACK_T callbackFunc);
/** @} */

#ifdef CYBLE_ANCS_SERVER
/**
 \addtogroup group_service_api_ANCS_server
 @{
*/

CYBLE_API_RESULT_T CyBle_AncssSetCharacteristicValue(CYBLE_ANCS_CHAR_INDEX_T charIndex,
                                                        uint8 attrSize, uint8 *attrValue);
CYBLE_API_RESULT_T CyBle_AncssGetCharacteristicValue(CYBLE_ANCS_CHAR_INDEX_T charIndex, 
                                                        uint8 attrSize, uint8 *attrValue);
CYBLE_API_RESULT_T CyBle_AncssGetCharacteristicDescriptor(CYBLE_ANCS_CHAR_INDEX_T charIndex, 
                                                        CYBLE_ANCS_DESCR_INDEX_T descrIndex,
                                                        uint8 attrSize, uint8 *attrValue);
CYBLE_API_RESULT_T CyBle_AncssSendNotification(CYBLE_CONN_HANDLE_T connHandle, 
                                                        CYBLE_ANCS_CHAR_INDEX_T charIndex,
                                                        uint8 attrSize, uint8 *attrValue);
/** @} */
#endif /* CYBLE_ANCS_SERVER */

#ifdef CYBLE_ANCS_CLIENT
/**
 \addtogroup group_service_api_ANCS_client
 @{
*/

CYBLE_API_RESULT_T CyBle_AncscSetCharacteristicValue(CYBLE_CONN_HANDLE_T connHandle,
                                                        CYBLE_ANCS_CHAR_INDEX_T charIndex,
                                                        uint8 attrSize, uint8 *attrValue);
CYBLE_API_RESULT_T CyBle_AncscSetCharacteristicDescriptor(CYBLE_CONN_HANDLE_T connHandle,
                                                        CYBLE_ANCS_CHAR_INDEX_T charIndex,
                                                        CYBLE_ANCS_DESCR_INDEX_T descrIndex,
                                                        uint8 attrSize, uint8 *attrValue);
CYBLE_API_RESULT_T CyBle_AncscGetCharacteristicDescriptor(CYBLE_CONN_HANDLE_T connHandle,
                                                        CYBLE_ANCS_CHAR_INDEX_T charIndex,
                                                        CYBLE_ANCS_DESCR_INDEX_T descrIndex);
/** @} */
#endif /* CYBLE_ANCS_CLIENT */


/***************************************
* Private Function Prototypes
***************************************/

/** \cond IGNORE */
void CyBle_AncsInit(void);

#ifdef CYBLE_ANCS_SERVER

void CyBle_AncssConfirmationEventHandler(const CYBLE_CONN_HANDLE_T *eventParam);
CYBLE_GATT_ERR_CODE_T CyBle_AncssWriteEventHandler(CYBLE_GATTS_WRITE_REQ_PARAM_T *eventParam);

#endif /* CYBLE_ANCS_SERVER */

#ifdef CYBLE_ANCS_CLIENT

void CyBle_AncscDiscoverServiceEventHandler(const CYBLE_DISC_SRVC128_INFO_T *discServInfo);
void CyBle_AncscDiscoverCharacteristicsEventHandler(CYBLE_DISC_CHAR_INFO_T *discCharInfo);
void CyBle_AncscDiscoverCharDescriptorsEventHandler(CYBLE_DISC_DESCR_INFO_T *discDescrInfo);
void CyBle_AncscNotificationEventHandler(CYBLE_GATTC_HANDLE_VALUE_NTF_PARAM_T *eventParam);
void CyBle_AncscReadResponseEventHandler(CYBLE_GATTC_READ_RSP_PARAM_T *eventParam);
void CyBle_AncscWriteResponseEventHandler(const CYBLE_CONN_HANDLE_T *eventParam);
void CyBle_AncscErrorResponseEventHandler(const CYBLE_GATTC_ERR_RSP_PARAM_T *eventParam);

#endif /* CYBLE_ANCS_CLIENT */
/** \endcond */


/***************************************
*  External data references
***************************************/

#if(CYBLE_GAP_ROLE_CENTRAL || CYBLE_GATT_ROLE_CLIENT)
    extern const CYBLE_UUID128_T cyBle_ancscServUuid;
    extern const CYBLE_UUID128_T cyBle_ancscCharUuid[CYBLE_ANCS_CHAR_COUNT];
#endif /* CYBLE_GAP_ROLE_CENTRAL || CYBLE_GATT_ROLE_CLIENT */

#ifdef CYBLE_ANCS_SERVER
    /* ANC Service GATT DB handles structure */
    extern const CYBLE_ANCSS_T   cyBle_ancss;
#endif /* CYBLE_ANCSC_SERVER */

#ifdef CYBLE_ANCS_CLIENT
    /* ANC Service server's GATT DB handles structure */
    extern CYBLE_ANCSC_T         cyBle_ancsc;
#endif /* CYBLE_ANCS_CLIENT */

#endif /* CY_BLE_CYBLE_ANCS_H */

/* [] END OF FILE */
