/***************************************************************************//**
* \file CYBLE_ips.h
* \version `$CY_MAJOR_VERSION`.`$CY_MINOR_VERSION`
* 
* \brief
*  Contains the function prototypes and constants for the Indoor Positioning
*  Service.
* 
********************************************************************************
* \copyright
* Copyright 2016-2018, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

/**
 \addtogroup group_service_api_IPS_definitions
 @{
*/


#if !defined(CY_BLE_CYBLE_IPS_H)
#define CY_BLE_CYBLE_IPS_H

#include "`$INSTANCE_NAME`_gatt.h"

/***************************************
* Data Struct Definition
***************************************/

/** IPS Characteristic indexes */
typedef enum
{
    CYBLE_IPS_INDOOR_POSITINING_CONFIG,         /**< Set of characteristic values included in the Indoor Positioning Service AD type. */
    CYBLE_IPS_LATITUDE,                         /**< WGS84 North coordinate of the device. */
    CYBLE_IPS_LONGITUDE,                        /**< WGS84 East coordinate of the device. */
    CYBLE_IPS_LOCAL_NORTH_COORDINATE,           /**< North coordinate of the device using local coordinate system. */
    CYBLE_IPS_LOCAL_EAST_COORDINATE,            /**< East coordinate of the device using local coordinate system. */
    CYBLE_IPS_FLOOR_NUMBER,                     /**< Describes in which floor the device is installed in. */
    CYBLE_IPS_ALTITUDE,                         /**< Altitude of the device. */
    CYBLE_IPS_UNCERTAINTY,                      /**< Uncertainty of the location information the device exposes. */
    CYBLE_IPS_LOCATION_NAME,                    /**< Name of the location the device is installed in. */
    CYBLE_IPS_CHAR_COUNT                        /**< Total count of IPS characteristics */
}CYBLE_IPS_CHAR_INDEX_T;

/** IPS Characteristic Descriptors indexes */
typedef enum
{   
    CYBLE_IPS_CEPD,                              /**< Characteristic Extended Properties descriptor index */
    CYBLE_IPS_SCCD,                              /**< Server Characteristic Configuration Descriptor index */
    CYBLE_IPS_DESCR_COUNT                        /**< Total count of descriptors */
}CYBLE_IPS_DESCR_INDEX_T;

#ifdef CYBLE_IPS_SERVER

/** IPS Characteristic with descriptors */
typedef struct
{
    /** Handles of Characteristic value */
    CYBLE_GATT_DB_ATTR_HANDLE_T charHandle;

    /** Array of Descriptor handles */
    CYBLE_GATT_DB_ATTR_HANDLE_T descrHandle[CYBLE_IPS_DESCR_COUNT];
} CYBLE_IPSS_CHAR_T;

/** Structure to hold pointer to CYBLE_IPSS_CHAR_T */
typedef struct
{
    /** Pointer to CYBLE_IPSS_CHAR_T which holds information about specific
    * IP Characteristic
    */
    CYBLE_IPSS_CHAR_T *charInfoPtr;
} CYBLE_IPSS_CHAR_INFO_PTR_T;

/** Structure with Indoor Positioning Service attribute handles */
typedef struct
{
    /** Indoor Positioning Service handle */
    CYBLE_GATT_DB_ATTR_HANDLE_T serviceHandle;

    /** Indoor Positioning Service Array with pointers to
    * Characteristic handles.
    */
    CYBLE_IPSS_CHAR_T charInfo[CYBLE_IPS_CHAR_COUNT];
} CYBLE_IPSS_T;

#endif /* CYBLE_IPS_SERVER */

#ifdef CYBLE_IPS_CLIENT

/** IPS Characteristic with descriptors */
typedef struct
{
    CYBLE_GATT_DB_ATTR_HANDLE_T valueHandle;                        /**< Handle of characteristic value */
    CYBLE_GATT_DB_ATTR_HANDLE_T endHandle;                          /**< End handle of characteristic */
    CYBLE_GATT_DB_ATTR_HANDLE_T descrHandle[CYBLE_IPS_DESCR_COUNT]; /**< Array of Descriptor handles */
    uint8  properties;                                              /**< Properties for value field */
} CYBLE_IPSC_CHAR_T;

/** Structure to hold pointer to CYBLE_IPSC_CHAR_T */
typedef struct
{
    /** Pointer to CYBLE_IPSC_CHAR_T which holds information about specific IP Characteristic. */
    CYBLE_IPSC_CHAR_T *charInfoPtr;
} CYBLE_IPSC_CHAR_INFO_PTR_T;

/** Structure with discovered attributes information of Indoor Positioning Service. */
typedef struct
{
    /** Indoor Positioning Service handle */
    CYBLE_GATT_DB_ATTR_HANDLE_T serviceHandle;
       
    /** Indoor Positioning Service characteristics info array */
    CYBLE_IPSC_CHAR_T charInfo[CYBLE_IPS_CHAR_COUNT];
} CYBLE_IPSC_T;

#endif /* CYBLE_IPS_CLIENT */

/** IPS Characteristic value parameter structure */
typedef struct
{
    CYBLE_CONN_HANDLE_T connHandle;                         /**< Peer device handle */
    CYBLE_IPS_CHAR_INDEX_T charIndex;                       /**< Index of service characteristic */
    CYBLE_GATT_VALUE_T *value;                              /**< Characteristic value */
    CYBLE_GATT_ERR_CODE_T gattErrorCode;                    /**< GATT error code for access control */

} CYBLE_IPS_CHAR_VALUE_T;

/** IPS Characteristic descriptor value parameter structure */
typedef struct
{
    CYBLE_CONN_HANDLE_T connHandle;                         /**< Peer device handle */
    CYBLE_IPS_CHAR_INDEX_T charIndex;                       /**< Index of service characteristic */
    CYBLE_IPS_DESCR_INDEX_T descrIndex;                     /**< Index of descriptor */
    CYBLE_GATT_ERR_CODE_T gattErrorCode;                    /**< Error code received from application (optional) */
    CYBLE_GATT_VALUE_T *value;                              /**< Characteristic value */
} CYBLE_IPS_DESCR_VALUE_T;

/** @} */

/***************************************
* API Constants
***************************************/
/* IPS specific GATT errors  */
#define CYBLE_GATT_ERR_WRITE_REQ_REJECTED                   (0x80u)


#if(CYBLE_GAP_ROLE_PERIPHERAL)
    
#define CYBLE_IPS_AD_TYPE                                   (0x25u)
#define CYBLE_IPS_SERVICE_DATA_LENGTH                       (0x05u)
  
#define CYBLE_IPS_CHARACTERISTICS_IN_ADVERTISING            (0x7Du)
#define CYBLE_IPS_COORDINATES_IN_ADVERTISING                (0x01u)
#define CYBLE_IPS_COORDINATES_NOT_PRESENT                   (0u)
#define CYBLE_IPS_COORDINATES_PRESENT                       (1u)
#define CYBLE_IPS_TYPE_OF_COORDINATE_IN_ADVERTISING         (0x02u)
#define CYBLE_IPS_WGS84_COORDINATE                          (0u)
#define CYBLE_IPS_LOCAL_COORDINATE                          (1u)
#define CYBLE_IPS_TX_POWER_IN_ADVERTISING                   (0x04u)
#define CYBLE_IPS_TX_POWER_NOT_PRESENT                      (0u)
#define CYBLE_IPS_TX_POWER_PRESENT                          (1u)
#define CYBLE_IPS_ALTITUDE_IN_ADVERTISING                   (0x08u)
#define CYBLE_IPS_ALTITUDE_NOT_PRESENT                      (0u)
#define CYBLE_IPS_ALTITUDE_PRESENT                          (1u)
#define CYBLE_IPS_FLOOR_NUMBER_IN_ADVERTISING               (0x10u)
#define CYBLE_IPS_FLOOR_NUMBER_NOT_PRESENT                  (0u)
#define CYBLE_IPS_FLOOR_NUMBER_PRESENT                      (1u)
#define CYBLE_IPS_UNCERTAINTY_NUMBER_IN_ADVERTISING         (0x20u)
#define CYBLE_IPS_UNCERTAINTY_NUMBER_NOT_PRESENT            (0u)
#define CYBLE_IPS_UNCERTAINTY_NUMBER_PRESENT                (1u)
#define CYBLE_IPS_LOCATION_NAME_IN_GATT_DATABASE            (0x40u)
#define CYBLE_IPS_LOCATION_NAME_NOT_PRESENT                 (0u)
#define CYBLE_IPS_LOCATION_NAME_PRESENT                     (1u)
    
    
#endif /* (CYBLE_GAP_ROLE_PERIPHERAL) */

/***************************************
* Function Prototypes
***************************************/

/** \addtogroup group_service_api_IPS_server_client 
@{ 
*/
void CyBle_IpsRegisterAttrCallback(CYBLE_CALLBACK_T callbackFunc);
/** @} */

#ifdef CYBLE_IPS_SERVER
/**
 \addtogroup group_service_api_IPS_server
 @{
*/

CYBLE_API_RESULT_T CyBle_IpssSetCharacteristicValue(CYBLE_IPS_CHAR_INDEX_T charIndex, uint8 attrSize, uint8 *attrValue);
CYBLE_API_RESULT_T CyBle_IpssGetCharacteristicValue(CYBLE_IPS_CHAR_INDEX_T charIndex, uint8 attrSize, uint8 *attrValue);
CYBLE_API_RESULT_T CyBle_IpssSetCharacteristicDescriptor(CYBLE_IPS_CHAR_INDEX_T charIndex, CYBLE_IPS_DESCR_INDEX_T descrIndex, uint8 attrSize, uint8 *attrValue);
CYBLE_API_RESULT_T CyBle_IpssGetCharacteristicDescriptor(CYBLE_IPS_CHAR_INDEX_T charIndex, CYBLE_IPS_DESCR_INDEX_T descrIndex, uint8 attrSize, uint8 *attrValue);

#endif /* CYBLE_IPS_SERVER */

/** @} */

#ifdef CYBLE_IPS_CLIENT
/**
 \addtogroup group_service_api_IPS_client
 @{
*/
CYBLE_API_RESULT_T CyBle_IpscSetCharacteristicValueWithoutResponse( CYBLE_CONN_HANDLE_T connHandle,
                                                        CYBLE_IPS_CHAR_INDEX_T charIndex,
                                                        uint8 attrSize, uint8 * attrValue);

CYBLE_API_RESULT_T CyBle_IpscSetCharacteristicValue(CYBLE_CONN_HANDLE_T connHandle,
                                                        CYBLE_IPS_CHAR_INDEX_T charIndex,
                                                        uint8 attrSize, uint8 *attrValue);

CYBLE_API_RESULT_T CyBle_IpscReliableWriteCharacteristicValue(CYBLE_CONN_HANDLE_T connHandle,
                                                        CYBLE_IPS_CHAR_INDEX_T charIndex,
                                                        uint8 attrSize, uint8 * attrValue);

CYBLE_API_RESULT_T CyBle_IpscGetCharacteristicValue(CYBLE_CONN_HANDLE_T connHandle, 
                                                        CYBLE_IPS_CHAR_INDEX_T charIndex);

CYBLE_API_RESULT_T CyBle_IpscGetMultipleCharacteristicValues(CYBLE_CONN_HANDLE_T connHandle,
                                                        const CYBLE_IPS_CHAR_INDEX_T  *charIndexesList,
                                                        uint8 numberOfCharIndexes);

CYBLE_API_RESULT_T CyBle_IpscGetLongCharacteristicValue(CYBLE_CONN_HANDLE_T connHandle, 
                                                        CYBLE_IPS_CHAR_INDEX_T charIndex, 
                                                        uint16 attrSize,
                                                        uint8 *attrValue);

CYBLE_API_RESULT_T CyBle_IpscSetCharacteristicDescriptor(CYBLE_CONN_HANDLE_T connHandle,
                                                        CYBLE_IPS_CHAR_INDEX_T charIndex,
                                                        CYBLE_IPS_DESCR_INDEX_T descrIndex,
                                                        uint8 attrSize, uint8 *attrValue);

CYBLE_API_RESULT_T CyBle_IpscGetCharacteristicDescriptor(CYBLE_CONN_HANDLE_T connHandle,
                                                        CYBLE_IPS_CHAR_INDEX_T charIndex,
                                                        CYBLE_IPS_DESCR_INDEX_T descrIndex);

/** @} */
#endif /* CYBLE_IPS_CLIENT */


/***************************************
* Private Function Prototypes
***************************************/

/** \cond IGNORE */
void CyBle_IpsInit(void);

#ifdef CYBLE_IPS_SERVER

CYBLE_GATT_ERR_CODE_T CyBle_IpssWriteEventHandler(CYBLE_GATTS_WRITE_REQ_PARAM_T *eventParam);
void CyBle_IpssPrepareWriteRequestEventHandler(CYBLE_GATTS_PREP_WRITE_REQ_PARAM_T *eventParam);
void CyBle_IpssExecuteWriteRequestEventHandler(CYBLE_GATTS_EXEC_WRITE_REQ_T *eventParam);
void CyBle_IpssConfirmationEventHandler(const CYBLE_CONN_HANDLE_T *eventParam);
void CyBle_IpssWriteCmdEventHandler(CYBLE_GATTS_WRITE_REQ_PARAM_T *eventParam);
CYBLE_API_RESULT_T CyBle_IpssSetAdvertisementData(void);

#endif /* CYBLE_IPS_SERVER */

#ifdef CYBLE_IPS_CLIENT
    
void CyBle_IpscDiscoverCharacteristicsEventHandler(CYBLE_DISC_CHAR_INFO_T * discCharInfo);
void CyBle_IpscDiscoverCharDescriptorsEventHandler(CYBLE_IPS_CHAR_INDEX_T discoveryCharIndex, CYBLE_DISC_DESCR_INFO_T * discDescrInfo);
void CyBle_IpscReadResponseEventHandler(CYBLE_GATTC_READ_RSP_PARAM_T *eventParam);
void CyBle_IpscReadMultipleResponseEventHandler(CYBLE_GATTC_READ_RSP_PARAM_T *eventParam);
void CyBle_IpscReadLongRespEventHandler(const CYBLE_GATTC_READ_RSP_PARAM_T *eventParam);
void CyBle_IpscWriteResponseEventHandler(const CYBLE_CONN_HANDLE_T *eventParam);
void CyBle_IpscErrorResponseEventHandler(const CYBLE_GATTC_ERR_RSP_PARAM_T *eventParam);
void CyBle_IpscExecuteWriteResponseEventHandler(const CYBLE_GATTC_EXEC_WRITE_RSP_T *eventParam);

#endif /* CYBLE_IPS_CLIENT */
/** \endcond */


/***************************************
* Macro Functions
***************************************/

#define CyBle_IpsGet16ByPtr    CyBle_Get16ByPtr

#ifdef CYBLE_IPS_CLIENT

#define CyBle_IpscGetCharacteristicValueHandle(charIndex)    \
    (((charIndex) >= CYBLE_IPS_CHAR_COUNT) ? \
            CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE : \
            cyBle_ipsc.charInfo[charIndex].valueHandle)

#define CyBle_IpscGetCharacteristicDescriptorHandle(charIndex, descrIndex)    \
    ((((charIndex) >= CYBLE_IPS_CHAR_COUNT) || ((descrIndex) >= CYBLE_IPS_DESCR_COUNT)) ? \
            CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE : \
            cyBle_ipsc.charInfo[charIndex].descrHandle[descrIndex])

#endif /* (CYBLE_IPS_CLIENT) */


/***************************************
* External data references
***************************************/

#ifdef CYBLE_IPS_SERVER

extern const CYBLE_IPSS_T cyBle_ipss;
extern uint8 cyBle_ipsFlag;

#endif /* CYBLE_IPS_SERVER */

#ifdef CYBLE_IPS_CLIENT

extern CYBLE_IPSC_T cyBle_ipsc;

#endif /* (CYBLE_IPS_CLIENT) */

#endif /* CY_BLE_CYBLE_IPS_H */


/* [] END OF FILE */
