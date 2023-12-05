/***************************************************************************//**
* \file CYBLE_hps.h
* \version `$CY_MAJOR_VERSION`.`$CY_MINOR_VERSION`
* 
* \brief
*  Contains the function prototypes and constants for HTTP Proxy Service.
*  
* 
********************************************************************************
* \copyright
* Copyright 2016-2018, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/


#if !defined(CY_BLE_CYBLE_HPS_H)
#define CY_BLE_CYBLE_HPS_H

#include "`$INSTANCE_NAME`_gatt.h"


/***************************************
* Conditional Compilation Parameters
***************************************/

/**
 \addtogroup group_service_api_HPS_definitions
 @{
*/

/***************************************
* Data Struct Definition
***************************************/

/** HPS Characteristic indexes */
typedef enum
{
    CYBLE_HPS_URI,                              /**< Universal Resource Identifier Characteristics index */
    CYBLE_HPS_HTTP_HEADERS,                     /**< HTTP Headers Characteristics index */
    CYBLE_HPS_HTTP_ENTITY_BODY,                 /**< HTTP Entity Body Characteristics index */
    CYBLE_HPS_HTTP_CP,                          /**< HTTP Control Point Characteristics index */
    CYBLE_HPS_HTTP_STATUS_CODE,                 /**< HTTP Status Code Characteristics index */
    CYBLE_HPS_HTTPS_SECURITY,                   /**< HTTPS Security Characteristics index */
    CYBLE_HPS_CHAR_COUNT                        /**< Total count of HPS Characteristics */
}CYBLE_HPS_CHAR_INDEX_T;

/** HPS Characteristic Descriptors indexes */
typedef enum
{
    CYBLE_HPS_CCCD,                              /**< Client Characteristic Configuration Descriptor index */
    CYBLE_HPS_DESCR_COUNT                        /**< Total count of Descriptors */
} CYBLE_HPS_DESCR_INDEX_T;

/** HPS Characteristic value parameter structure */
typedef struct
{
    CYBLE_CONN_HANDLE_T connHandle;                         /**< Peer device handle */
    CYBLE_HPS_CHAR_INDEX_T charIndex;                       /**< Index of service characteristic */
    CYBLE_GATT_ERR_CODE_T gattErrorCode;                    /**< Error code received from application (optional) */
    CYBLE_GATT_VALUE_T *value;                              /**< Characteristic value */
} CYBLE_HPS_CHAR_VALUE_T;

/** HPS Characteristic descriptor value parameter structure */
typedef struct
{
    CYBLE_CONN_HANDLE_T connHandle;                         /**< Peer device handle */
    CYBLE_HPS_CHAR_INDEX_T charIndex;                       /**< Index of service characteristic */
    CYBLE_HPS_DESCR_INDEX_T descrIndex;                     /**< Index of descriptor */
    CYBLE_GATT_ERR_CODE_T gattErrorCode;                    /**< Error code received from application (optional) */
    CYBLE_GATT_VALUE_T *value;                              /**< Characteristic value */
} CYBLE_HPS_DESCR_VALUE_T;

#ifdef CYBLE_HPS_SERVER

/** Structure with HTTP Proxy Service attribute handles */
typedef struct
{
    /** Handle of characteristic value */
    CYBLE_GATT_DB_ATTR_HANDLE_T charHandle;
    /** Array of descriptor handles */
    CYBLE_GATT_DB_ATTR_HANDLE_T descrHandle[CYBLE_HPS_DESCR_COUNT];
} CYBLE_HPSS_CHAR_T;

/** HPS Characteristic with descriptors handles */
typedef struct
{
    CYBLE_GATT_DB_ATTR_HANDLE_T serviceHandle;              /**< HTTP Proxy Service handle */
    CYBLE_HPSS_CHAR_T charInfo[CYBLE_HPS_CHAR_COUNT];       /**< Array of characteristics and descriptors handles */
} CYBLE_HPSS_T;

#endif /* CYBLE_HPS_SERVER */

#ifdef CYBLE_HPS_CLIENT

/** HPS Service full characteristic information structure */
typedef struct
{
    CYBLE_GATT_DB_ATTR_HANDLE_T valueHandle;                        /**< Handle of characteristic value */
    uint8  properties;                                              /**< Properties for value field */
    CYBLE_GATT_DB_ATTR_HANDLE_T endHandle;                          /**< End handle of characteristic */
    CYBLE_GATT_DB_ATTR_HANDLE_T descrHandle[CYBLE_HPS_DESCR_COUNT]; /**< Array of descriptor handles */
} CYBLE_HPSC_CHAR_T;

/** Structure with discovered attributes information of HTTP Proxy Service */
typedef struct
{
    /** HTTP Proxy Service handle */
    CYBLE_GATT_DB_ATTR_HANDLE_T serviceHandle;
    /** HTTP Proxy Service characteristics info structure */
    CYBLE_HPSC_CHAR_T charInfo[CYBLE_HPS_CHAR_COUNT];
} CYBLE_HPSC_T;

#endif /* CYBLE_HPS_CLIENT */

/** HTTP Requests */
typedef enum
{
    CYBLE_HPS_HTTP_GET = 0x01u,                   /**< HTTP GET Request */
    CYBLE_HPS_HTTP_HEAD,                          /**< HTTP HEAD Request */
    CYBLE_HPS_HTTP_POST,                          /**< HTTP POST Request */
    CYBLE_HPS_HTTP_PUT,                           /**< HTTP PUT Request */
    CYBLE_HPS_HTTP_DELETE,                        /**< HTTP DELETE Request */
    CYBLE_HPS_HTTPS_GET,                          /**< HTTS GET Request */
    CYBLE_HPS_HTTPS_HEAD,                         /**< HTTPS HEAD Request */
    CYBLE_HPS_HTTPS_POST,                         /**< HTTPS POST Request */
    CYBLE_HPS_HTTPS_PUT,                          /**< HTTPS PUT Request */
    CYBLE_HPS_HTTPS_DELETE,                       /**< HTTPS DELETE Request */
    CYBLE_HPS_HTTP_REQ_CANCEL                     /**< HTTP CANCEL Request */
}CYBLE_HPS_HTTP_REQUEST_T;

/** @} */

/***************************************
* API Constants
***************************************/
/* HTTP Status Code Data Status bits  */
#define CYBLE_HPS_HTTP_HEADERS_RECEIVED         (0x01u)
#define CYBLE_HPS_HTTP_HEADERS_TRUNCATED        (0x02u)
#define CYBLE_HPS_HTTP_BODY_RECEIVED            (0x04u)
#define CYBLE_HPS_HTTP_BODY_TRUNCATED           (0x08u)


/***************************************
* Function Prototypes
***************************************/

/** \addtogroup group_service_api_HPS_server_client
 @{ 
 */
void CyBle_HpsRegisterAttrCallback(CYBLE_CALLBACK_T callbackFunc);
/** @} */

/**
 \addtogroup group_service_api_HPS_server
 @{
*/

#ifdef CYBLE_HPS_SERVER

CYBLE_API_RESULT_T CyBle_HpssSetCharacteristicValue(CYBLE_HPS_CHAR_INDEX_T charIndex, uint16 attrSize, uint8 *attrValue);
CYBLE_API_RESULT_T CyBle_HpssGetCharacteristicValue(CYBLE_HPS_CHAR_INDEX_T charIndex, uint16 attrSize, uint8 *attrValue);
CYBLE_API_RESULT_T CyBle_HpssSetCharacteristicDescriptor(CYBLE_HPS_CHAR_INDEX_T charIndex,
    CYBLE_HPS_DESCR_INDEX_T descrIndex, uint8 attrSize, uint8 *attrValue);
CYBLE_API_RESULT_T CyBle_HpssGetCharacteristicDescriptor(CYBLE_HPS_CHAR_INDEX_T charIndex,
    CYBLE_HPS_DESCR_INDEX_T descrIndex, uint8 attrSize, uint8 *attrValue);
CYBLE_API_RESULT_T CyBle_HpssSendNotification(CYBLE_CONN_HANDLE_T connHandle, CYBLE_HPS_CHAR_INDEX_T charIndex,
    uint8 attrSize, uint8 *attrValue);

#endif /* CYBLE_HPS_SERVER */

/** @} */

/**
 \addtogroup group_service_api_HPS_client
 @{
*/

#ifdef CYBLE_HPS_CLIENT

CYBLE_API_RESULT_T CyBle_HpscSetCharacteristicValue(CYBLE_CONN_HANDLE_T connHandle, CYBLE_HPS_CHAR_INDEX_T charIndex,
    uint16 attrSize, uint8 *attrValue);
CYBLE_API_RESULT_T CyBle_HpscGetCharacteristicValue(CYBLE_CONN_HANDLE_T connHandle, CYBLE_HPS_CHAR_INDEX_T charIndex);
CYBLE_API_RESULT_T CyBle_HpscSetLongCharacteristicValue(CYBLE_CONN_HANDLE_T connHandle, CYBLE_HPS_CHAR_INDEX_T charIndex,
    uint16 attrSize, uint8 *attrValue);
CYBLE_API_RESULT_T CyBle_HpscGetLongCharacteristicValue(CYBLE_CONN_HANDLE_T connHandle, CYBLE_HPS_CHAR_INDEX_T charIndex,
    uint16 attrSize, uint8 *attrValue);
CYBLE_API_RESULT_T CyBle_HpscSetCharacteristicDescriptor(CYBLE_CONN_HANDLE_T connHandle,
    CYBLE_HPS_CHAR_INDEX_T charIndex, CYBLE_HPS_DESCR_INDEX_T descrIndex, uint8 attrSize, uint8 *attrValue);
CYBLE_API_RESULT_T CyBle_HpscGetCharacteristicDescriptor(CYBLE_CONN_HANDLE_T connHandle,
    CYBLE_HPS_CHAR_INDEX_T charIndex, CYBLE_HPS_DESCR_INDEX_T descrIndex);

#endif /* CYBLE_HPS_CLIENT */

/** @} */

/***************************************
* Private Function Prototypes
***************************************/

/** \cond IGNORE */
void CyBle_HpsInit(void);

#ifdef CYBLE_HPS_SERVER

CYBLE_GATT_ERR_CODE_T CyBle_HpssWriteEventHandler(CYBLE_GATTS_WRITE_REQ_PARAM_T *eventParam);
void CyBle_HpssPrepareWriteRequestEventHandler(const CYBLE_GATTS_PREP_WRITE_REQ_PARAM_T *eventParam);
void CyBle_HpssExecuteWriteRequestEventHandler(CYBLE_GATTS_EXEC_WRITE_REQ_T *eventParam);

#endif /* CYBLE_HPS_SERVER */

#ifdef CYBLE_HPS_CLIENT

void CyBle_HpscDiscoverCharacteristicsEventHandler(const CYBLE_DISC_CHAR_INFO_T *discCharInfo);
void CyBle_HpscDiscoverCharDescriptorsEventHandler(CYBLE_DISC_DESCR_INFO_T *discDescrInfo);
void CyBle_HpscNotificationEventHandler(CYBLE_GATTC_HANDLE_VALUE_NTF_PARAM_T *eventParam);
void CyBle_HpscReadResponseEventHandler(CYBLE_GATTC_READ_RSP_PARAM_T *eventParam);
void CyBle_HpscReadLongRespEventHandler(const CYBLE_GATTC_READ_RSP_PARAM_T *eventParam);
void CyBle_HpscWriteResponseEventHandler(const CYBLE_CONN_HANDLE_T *eventParam);
void CyBle_HpscExecuteWriteResponseEventHandler(const CYBLE_GATTC_EXEC_WRITE_RSP_T *eventParam);
void CyBle_HpscErrorResponseEventHandler(const CYBLE_GATTC_ERR_RSP_PARAM_T *eventParam);

#endif /* CYBLE_HPS_CLIENT */

/** \endcond */


/***************************************
* External data references
***************************************/

#ifdef CYBLE_HPS_SERVER
    extern const CYBLE_HPSS_T cyBle_hpss;
#endif /* CYBLE_HPS_SERVER */

#ifdef CYBLE_HPS_CLIENT
    extern CYBLE_HPSC_T cyBle_hpsc;
#endif /* CYBLE_HPS_CLIENT */


#endif /* CY_BLE_CYBLE_HPS_H */

/* [] END OF FILE */
