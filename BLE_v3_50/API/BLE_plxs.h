/***************************************************************************//**
* \file cy_ble_plxs.h
* \version `$CY_MAJOR_VERSION`.`$CY_MINOR_VERSION`
* 
* \brief
*  Contains the function prototypes and constants for the Pulse Oximeter
*  Service.
* 
********************************************************************************
* \copyright
* Copyright 2017-2018, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#ifndef CYBLE_PLXS_H
#define CYBLE_PLXS_H
    
#include "`$INSTANCE_NAME`_eventHandler.h"
#include "`$INSTANCE_NAME`_gatt.h"
#include "`$INSTANCE_NAME`_plxs.h"
    
/**
 \addtogroup group_service_api_PLXS_definitions
 @{
*/


#if(CYBLE_MODE_PROFILE && defined(CYBLE_PLXS))


/***************************************
* Data Struct Definition
***************************************/

/** PLXS Characteristic indexes */
typedef enum
{   
    CYBLE_PLXS_SCMT,                        /**< The PLX Spot-check Measurement characteristic, if supported, 
                                                is used to send Spot-check measurements of SpO2 (Percent oxygen 
                                                saturation of hemoglobin) and PR (pulse rate). This 
                                                characteristic is a variable length structure containing the 
                                                Flags field, the SpO2PR-Spot-Check field, and depending on the 
                                                contents of the Flags field, the Timestamp field, the Measurement
                                                Status field, the Device and Sensor Status field, and/or the 
                                                Pulse Amplitude Index field. */
    
    CYBLE_PLXS_CTMT,                        /**< The PLX Continuous Measurement characteristic, if supported, 
                                                is used to send periodic pulse oximetry measurements. This 
                                                characteristic is a variable length structure containing the 
                                                Flags field (to indicate presence of optional fields), the 
                                                SpO2PR-Normal field, and depending on the contents of the Flags
                                                field, the SpO2PR-Fast field, the SpO2PR-Slow field, the 
                                                Measurement Status field, the Device and Sensor Status field, 
                                                and/or the Pulse Amplitude Index field.*/
    
    CYBLE_PLXS_FEAT,                        /**< The PLX Features characteristic is used to describe the 
                                                supported features of the Server. Included in the characteristic 
                                                is a PLX Features field, and, depending on the contents of the 
                                                PLX Features field, the Measurement Status Support field, and 
                                                the Device and Sensor Status Support field. */
		
    CYBLE_PLXS_RACP,                        /**< This control point is used with a service to provide basic 
                                                management functionality for the PLX Sensor patient record 
                                                database. This enables functions including counting records, 
                                                transmitting records and clearing records based on filter 
                                                criterion. The filter criterion in the Operand field is defined 
                                                by the service that references this characteristic as is the 
                                                format of a record (that may be comprised of one or more 
                                                characteristics) and the sequence of transferred records.*/
    
    CYBLE_PLXS_CHAR_COUNT                   /**< Total count of PLXS characteristics */
    
}CYBLE_PLXS_CHAR_INDEX_T;


/** PLXS Characteristic Descriptors indexes */
typedef enum
{   
    CYBLE_PLXS_CCCD,                              /**< Client Characteristic Configuration Descriptor index */
    CYBLE_PLXS_DESCR_COUNT                        /**< Total count of descriptors */
}CYBLE_PLXS_DESCR_INDEX_T;

/** PLXS Characteristic with descriptors */
typedef struct
{
    CYBLE_GATT_DB_ATTR_HANDLE_T charHandle;                           /**< Handle of characteristic value */
    CYBLE_GATT_DB_ATTR_HANDLE_T descrHandle[CYBLE_PLXS_DESCR_COUNT];  /**< Handle of descriptor */
} CYBLE_PLXSS_CHAR_T;

/** Structure with PLXS attribute handles */
typedef struct
{
    /** PLXS handle */
    CYBLE_GATT_DB_ATTR_HANDLE_T serviceHandle;
    
    /** PLXS Characteristic handles */
    CYBLE_PLXSS_CHAR_T charInfo[CYBLE_PLXS_CHAR_COUNT];
} CYBLE_PLXSS_T;

/** PLXS Characteristic with descriptors */
typedef struct
{
    CYBLE_GATT_DB_ATTR_HANDLE_T valueHandle;                         /**< Handle of characteristic value */
    CYBLE_GATT_DB_ATTR_HANDLE_T endHandle;                           /**< End handle of characteristic */
    CYBLE_GATT_DB_ATTR_HANDLE_T descrHandle[CYBLE_PLXS_DESCR_COUNT]; /**< Array of Descriptor handles */
    uint8_t  properties;                                               /**< Properties for value field */
} CYBLE_PLXSC_CHAR_T;

/** Structure with discovered attributes information of Pulse Oximeter Service. */
typedef struct
{
    /** Pulse Oximeter Service handle */
    CYBLE_GATT_DB_ATTR_HANDLE_T serviceHandle;
        
    /** PLXS characteristics info array */
    CYBLE_PLXSC_CHAR_T charInfo[CYBLE_PLXS_CHAR_COUNT];
} CYBLE_PLXSC_T;


/** PLXS Characteristic value parameter structure */
typedef struct
{
    CYBLE_CONN_HANDLE_T connHandle;                         /**< Peer device handle */
    CYBLE_PLXS_CHAR_INDEX_T charIndex;                      /**< Index of service characteristic */
    CYBLE_GATT_VALUE_T *value;                              /**< Characteristic value */
    CYBLE_GATT_ERR_CODE_T gattErrorCode;                    /**< GATT error code for access control */

} CYBLE_PLXS_CHAR_VALUE_T;

/** PLXS Characteristic descriptor value parameter structure */
typedef struct
{
    CYBLE_CONN_HANDLE_T connHandle;                         /**< Peer device handle */
    CYBLE_PLXS_CHAR_INDEX_T charIndex;                      /**< Index of service characteristic */
    CYBLE_PLXS_DESCR_INDEX_T descrIndex;                    /**< Index of descriptor */
    CYBLE_GATT_ERR_CODE_T gattErrorCode;                    /**< Error code received from application (optional) */
    CYBLE_GATT_VALUE_T *value;                              /**< Characteristic value */
} CYBLE_PLXS_DESCR_VALUE_T;

/** Record Access Control Point characteristic fields defines */
/** Opcode of the Record Access Control Point characteristic value type */
typedef enum
{
    CYBLE_PLXS_RACP_OPC_RESERVED = 0u,          /**< Reserved for future use (Operator:N/A) */
    CYBLE_PLXS_RACP_OPC_REPORT_REC,             /**< Report stored records (Operator: Value from Operator Table) */
    CYBLE_PLXS_RACP_OPC_DELETE_REC,             /**< Delete stored records (Operator: Value from Operator Table) */
    CYBLE_PLXS_RACP_OPC_ABORT_OPN,              /**< Abort operation (Operator: Null 'value of 0x00 from Operator Table') */
    CYBLE_PLXS_RACP_OPC_REPORT_NUM_REC,         /**< Report number of stored records (Operator: Value from Operator Table) */
    CYBLE_PLXS_RACP_OPC_NUM_REC_RSP,            /**< Number of stored records response (Operator: Null 'value of 0x00 from Operator Table') */
    CYBLE_PLXS_RACP_OPC_RSP_CODE,               /**< Response Code (Operator: Null 'value of 0x00 from Operator Table') */
    
} CYBLE_PLXS_RACP_OPC_T;

/** Operator of the Record Access Control Point characteristic value type */
typedef enum
{
    CYBLE_PLXS_RACP_OPR_NULL = 0u,              /**< Null */
    CYBLE_PLXS_RACP_OPR_ALL,                    /**< All records */
    CYBLE_PLXS_RACP_OPR_LESS,                   /**< Less than or equal to */
    CYBLE_PLXS_RACP_OPR_GREAT,                  /**< Greater than or equal to */
    CYBLE_PLXS_RACP_OPR_WITHIN,                 /**< Within range of (inclusive) */
    CYBLE_PLXS_RACP_OPR_FIRST,                  /**< First record(i.e. oldest record) */
    CYBLE_PLXS_RACP_OPR_LAST                    /**< Last record (i.e. most recent record) */
} CYBLE_PLXS_RACP_OPR_T;

/** Operand of the Record Access Control Point characteristic value type */
typedef enum
{
    CYBLE_PLXS_RACP_OPD_NA = 0u,                /**< N/A */
    CYBLE_PLXS_RACP_OPD_1,                      /**< Filter parameters (as appropriate to Operator and Service) */
    CYBLE_PLXS_RACP_OPD_2,                      /**< Filter parameters (as appropriate to Operator and Service) */
    CYBLE_PLXS_RACP_OPD_NO_INCL,                /**< Not included */
    CYBLE_PLXS_RACP_OPD_4,                      /**< Filter parameters (as appropriate to Operator and Service) */
    CYBLE_PLXS_RACP_OPD_NUM_REC,                /**< Number of Records (Field size defined per service) */
    CYBLE_PLXS_RACP_OPD_RSP,                    /**< Request Op Code, Response Code Value */
    
} CYBLE_PLXS_RACP_OPD_T;

/** Operand Response Code Values of the Record Access Control Point characteristic value type */
typedef enum
{
    CYBLE_PLXS_RACP_RSP_NA = 0u,                /**< N/A */
    CYBLE_PLXS_RACP_RSP_SUCCESS,                /**< Normal response for successful operation */
    CYBLE_PLXS_RACP_RSP_UNSPRT_OPC,             /**< Normal response if unsupported Op Code is received */
    CYBLE_PLXS_RACP_RSP_INV_OPR,                /**< Normal response if Operator received does not meet the
                                                     requirements of the service (e.g. Null was expected) */
    CYBLE_PLXS_RACP_RSP_UNSPRT_OPR,             /**< Normal response if unsupported Operator is received */
    CYBLE_PLXS_RACP_RSP_INV_OPD,                /**< Normal response if Operand received does not meet the 
                                                     requirements of the service */
    CYBLE_PLXS_RACP_RSP_NO_REC,                 /**< Normal response if request to report stored records
                                                     or request to delete stored records resulted in no
                                                     records meeting criteria. */
    CYBLE_PLXS_RACP_RSP_UNSUCCESS,              /**< Normal response if request for Abort cannot be completed */
    CYBLE_PLXS_RACP_RSP_NO_COMPL,               /**< Normal response if request for Abort cannot be completed */
    CYBLE_PLXS_RACP_RSP_UNSPRT_OPD              /**< Normal response if unsupported Operand is received */
    
} CYBLE_PLXS_RACP_RSP_T;

/***************************************
* API Constants
***************************************/

/** \cond IGNORE */
/* RACP Procedure Timeout */
#define CYBLE_PLXS_RACP_PROCEDURE_TIMEOUT               (5u)          /* Seconds */

/* PLX internal flag */
#define CYBLE_PLXS_FLAG_PROCESS                         (1u)          /* Flag indicates that RACP is running */  
/** \endcond */

/** "Device and Sensor Status" field bits */
#define CYBLE_PLXS_DSS_EDU_BIT              (0x01u << 0u)     /**< Extended Display Update Ongoing bit */
#define CYBLE_PLXS_DSS_EMD_BIT              (0x01u << 1u)     /**< Equipment Malfunction Detected bit */
#define CYBLE_PLXS_DSS_SPID_BIT             (0x01u << 2u)     /**< Signal Processing Irregularity Detected bit */
#define CYBLE_PLXS_DSS_ISD_BIT              (0x01u << 3u)     /**< Inadequite Signal Detected bit */
#define CYBLE_PLXS_DSS_PSD_BIT              (0x01u << 4u)     /**< Poor Signal Detected bit */
#define CYBLE_PLXS_DSS_LPD_BIT              (0x01u << 5u)     /**< Low Perfusion Detected bit */
#define CYBLE_PLXS_DSS_ESD_BIT              (0x01u << 6u)     /**< Erratic Signal Detected bit */
#define CYBLE_PLXS_DSS_NSD_BIT              (0x01u << 7u)     /**< Nonpulsatile Signal Detected bit */
#define CYBLE_PLXS_DSS_QPD_BIT              (0x01u << 8u)     /**< Questionable Pulse Detected bit */
#define CYBLE_PLXS_DSS_SA_BIT               (0x01u << 9u)     /**< Signal Analysis Ongoing bit */
#define CYBLE_PLXS_DSS_SID_BIT              (0x01u << 10u)    /**< Sensor Interface Detected bit */
#define CYBLE_PLXS_DSS_SUTU_BIT             (0x01u << 11u)    /**< Sensor Unconnected to User bit */
#define CYBLE_PLXS_DSS_USC_BIT              (0x01u << 12u)    /**< Unknown Sensor Connected bit */
#define CYBLE_PLXS_DSS_SD_BIT               (0x01u << 13u)    /**< Sensor Displaced bit */        
#define CYBLE_PLXS_DSS_SM_BIT               (0x01u << 14u)    /**< Sensor Malfunctioning bit */        
#define CYBLE_PLXS_DSS_SDISC_BIT            (0x01u << 15u)    /**< Sensor Disconnected bit */         

/** "Measurement Status" field bits */
#define CYBLE_PLXS_MS_MEAS_BIT              (0x01u << 5u)    /**< Measurement Ongoing bit */
#define CYBLE_PLXS_MS_EED_BIT               (0x01u << 6u)    /**< Early Estimated Data bit */
#define CYBLE_PLXS_MS_VDATA_BIT             (0x01u << 7u)    /**< Validated Data bit */
#define CYBLE_PLXS_MS_FQDATA_BIT            (0x01u << 8u)    /**< Fully Qualified Data bit */
#define CYBLE_PLXS_MS_DFMS_BIT              (0x01u << 9u)    /**< Data from Measurement Storage bit */
#define CYBLE_PLXS_MS_DFDEMO_BIT            (0x01u << 10u)   /**< Data for Demonstration bit */
#define CYBLE_PLXS_MS_DFTEST_BIT            (0x01u << 11u)   /**< Data for Testing bit */
#define CYBLE_PLXS_MS_CALIB_BIT             (0x01u << 12u)   /**< Calibration Ongoing bit */
#define CYBLE_PLXS_MS_MUN_BIT               (0x01u << 13u)   /**< Measurement Unavailable bit */
#define CYBLE_PLXS_MS_QMD_BIT               (0x01u << 14u)   /**< Questionable Measurement Detected bit */
#define CYBLE_PLXS_MS_IMD_BIT               (0x01u << 15u)   /**< Invalid Measurement Detected bit */          
         
/** "Flag" field bits of PLX Spot-Check Measurement characteristic */
#define CYBLE_PLXS_SCMT_FLAG_TMSF_BIT       (0x01u << 0u)    /**< Timestamp field bit */
#define CYBLE_PLXS_SCMT_FLAG_MSF_BIT        (0x01u << 1u)    /**< Measurement Status Field bit */
#define CYBLE_PLXS_SCMT_FLAG_DSSF_BIT       (0x01u << 2u)    /**< Device and Sensor Status Field bit */
#define CYBLE_PLXS_SCMT_FLAG_PAIF_BIT       (0x01u << 3u)    /**< Pulse Amplitude Index field bit */
#define CYBLE_PLXS_SCMT_FLAG_DEVCLK_BIT     (0x01u << 4u)    /**< Device Clock is Not Set bit */
            
/** "Flag" field bits of PLX Continuous Measurement characteristic */
#define CYBLE_PLXS_CTMT_FLAG_FAST_BIT       (0x01u << 0u)    /**< SpO2PR-Fast field bit */
#define CYBLE_PLXS_CTMT_FLAG_SLOW_BIT       (0x01u << 1u)    /**< SpO2PR-Slow field bit */
#define CYBLE_PLXS_CTMT_FLAG_MSF_BIT        (0x01u << 2u)    /**< Measurement Status field bit */
#define CYBLE_PLXS_CTMT_FLAG_DSSF_BIT       (0x01u << 3u)    /**< Device and Sensor Status field bit */
#define CYBLE_PLXS_CTMT_FLAG_PAIF_BIT       (0x01u << 4u)    /**< Pulse Amplitude Index field bit */

/** "Supported Features"  bits of PLX Features characteristic */
#define CYBLE_PLXS_FEAT_SUPPORT_MEAS_BIT    (0x01u << 0u)    /**< Measurement Status support bit */
#define CYBLE_PLXS_FEAT_SUPPORT_DSS_BIT     (0x01u << 1u)    /**< Device and Sensor Status support bit */
#define CYBLE_PLXS_FEAT_SUPPORT_MSSC_BIT    (0x01u << 2u)    /**< Measurement Storage for Spot-check measurements bit */
#define CYBLE_PLXS_FEAT_SUPPORT_TMSF_BIT    (0x01u << 3u)    /**< Timestamp for Spot-check measurements bit */
#define CYBLE_PLXS_FEAT_SUPPORT_FAST_BIT    (0x01u << 4u)    /**< SpO2PR-Fast metric bit */
#define CYBLE_PLXS_FEAT_SUPPORT_SLOW_BIT    (0x01u << 5u)    /**< SpO2PR-Slow metric bit */
#define CYBLE_PLXS_FEAT_SUPPORT_PAI_BIT     (0x01u << 6u)    /**< Pulse Amplitude Index field bit */
#define CYBLE_PLXS_FEAT_SUPPORT_MBS_BIT     (0x01u << 7u)    /**< Multiple Bonds Supported bit */   

/** @} */

/***************************************
* Function Prototypes
***************************************/

/** \addtogroup group_service_api_PLXS_server_client 
@{ 
*/
void CyBle_PlxsInit(void);
void CyBle_PlxsRegisterAttrCallback(CYBLE_CALLBACK_T callbackFunc);

/** @} */

#ifdef CYBLE_PLXS_SERVER
/**
 \addtogroup group_service_api_PLXS_server
 @{
*/

CYBLE_API_RESULT_T CyBle_PlxssSetCharacteristicValue(CYBLE_PLXS_CHAR_INDEX_T charIndex, 
                                                        uint8 attrSize, uint8 *attrValue);

CYBLE_API_RESULT_T CyBle_PlxssGetCharacteristicValue(CYBLE_PLXS_CHAR_INDEX_T charIndex, 
                                                        uint8 attrSize, uint8 *attrValue);

CYBLE_API_RESULT_T CyBle_PlxssSetCharacteristicDescriptor(CYBLE_PLXS_CHAR_INDEX_T charIndex, 
                                                            CYBLE_PLXS_DESCR_INDEX_T descrIndex, 
                                                            uint8 attrSize, uint8 *attrValue);

CYBLE_API_RESULT_T CyBle_PlxssGetCharacteristicDescriptor(CYBLE_PLXS_CHAR_INDEX_T charIndex, 
                                                            CYBLE_PLXS_DESCR_INDEX_T descrIndex, 
                                                            uint8 attrSize, uint8 *attrValue);

CYBLE_API_RESULT_T CyBle_PlxssSendNotification( CYBLE_CONN_HANDLE_T connHandle,
                                                CYBLE_PLXS_CHAR_INDEX_T charIndex, 
                                                uint8 attrSize, uint8 *attrValue);

CYBLE_API_RESULT_T CyBle_PlxssSendIndication(   CYBLE_CONN_HANDLE_T connHandle,
                                                CYBLE_PLXS_CHAR_INDEX_T charIndex, 
                                                uint8 attrSize, uint8 *attrValue);

/** @} */
#endif /* CYBLE_PLXS_SERVER */



#ifdef CYBLE_PLXS_CLIENT
/**
 \addtogroup group_service_api_PLXS_client
 @{
*/
CYBLE_API_RESULT_T CyBle_PlxscSetCharacteristicValue(CYBLE_CONN_HANDLE_T connHandle,
    CYBLE_PLXS_CHAR_INDEX_T charIndex, uint8 attrSize, uint8 *attrValue);

CYBLE_API_RESULT_T CyBle_PlxscGetCharacteristicValue(CYBLE_CONN_HANDLE_T connHandle, 
    CYBLE_PLXS_CHAR_INDEX_T charIndex);

CYBLE_API_RESULT_T CyBle_PlxscSetCharacteristicDescriptor(CYBLE_CONN_HANDLE_T connHandle,
    CYBLE_PLXS_CHAR_INDEX_T charIndex, CYBLE_PLXS_DESCR_INDEX_T descrIndex, uint8 attrSize, uint8 *attrValue);

CYBLE_API_RESULT_T CyBle_PlxscGetCharacteristicDescriptor(CYBLE_CONN_HANDLE_T connHandle,
    CYBLE_PLXS_CHAR_INDEX_T charIndex, CYBLE_PLXS_DESCR_INDEX_T descrIndex);

/** @} */
#endif /* CYBLE_PLXS_CLIENT */

/***************************************
* Macro Functions
***************************************/

#define CyBle_PlxpGet16ByPtr    CyBle_Get16ByPtr

#ifdef CYBLE_PLXS_CLIENT
#define cyBle_PlxscGetCharacteristicValueHandle(charIndex)                                       \
    (((charIndex) >= CYBLE_PLXS_CHAR_COUNT) ? CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE :             \
                                              cyBle_plxsc.charInfo[charIndex].valueHandle)

#define cyBle_PlxscGetCharacteristicDescriptorHandle(charIndex, descrIndex)                      \
    ((((charIndex) >= CYBLE_PLXS_CHAR_COUNT) || ((descrIndex) >= CYBLE_PLXS_DESCR_COUNT)) ?      \
                                        CYBLE_GATT_INVALID_ATTR_HANDLE_VALUE :                   \
                                        cyBle_plxsc.charInfo[charIndex].descrHandle[descrIndex])
#endif /* (CYBLE_PLXS_CLIENT) */

#ifdef CYBLE_PLXS_SERVER
#define CYBLE_PLXSS_IS_PROCEDURE_IN_PROGRESS()     ((cyBle_plxssFlag & CYBLE_PLXS_FLAG_PROCESS) != 0u)
#endif /* CYBLE_PLXS_SERVER */

/***************************************
*  External data references
***************************************/

#ifdef CYBLE_PLXS_SERVER
    extern const CYBLE_PLXSS_T cyBle_plxss;
    extern uint32 cyBle_plxssFlag;
#endif /* CYBLE_PLXS_SERVER */

#ifdef CYBLE_PLXS_CLIENT
    extern uint32 cyBle_plxscFlag;
    extern CYBLE_PLXSC_T cyBle_plxsc;
#endif /* CYBLE_PLXS_CLIENT */

#endif /* CYBLE_MODE_PROFILE && defined(CYBLE_PLXS) */
#endif /* CYBLE_PLXS_H */

/***************************************
* Private Function Prototypes
***************************************/
#ifdef CYBLE_PLXS_SERVER
    void CyBle_PlxssConfirmationEventHandler(const CYBLE_CONN_HANDLE_T *eventParam);
    CYBLE_GATT_ERR_CODE_T CyBle_PlxssWriteEventHandler(CYBLE_GATTS_WRITE_REQ_PARAM_T *eventParam);
#endif /* CYBLE_PLXS_SERVER */

#ifdef CYBLE_PLXS_CLIENT
    void CyBle_PlxscDiscoverCharacteristicsEventHandler(CYBLE_DISC_CHAR_INFO_T *discCharInfo);
    void CyBle_PlxscDiscoverCharDescriptorsEventHandler(CYBLE_DISC_DESCR_INFO_T *discDescrInfo);
    void CyBle_PlxscNotificationEventHandler(CYBLE_GATTC_HANDLE_VALUE_NTF_PARAM_T *eventParam);
    void CyBle_PlxscIndicationEventHandler(CYBLE_GATTC_HANDLE_VALUE_IND_PARAM_T *eventParam);
    void CyBle_PlxscReadResponseEventHandler(CYBLE_GATTC_READ_RSP_PARAM_T *eventParam);
    void CyBle_PlxscWriteResponseEventHandler(const CYBLE_CONN_HANDLE_T *eventParam);
    void CyBle_PlxscErrorResponseEventHandler(const CYBLE_GATTC_ERR_RSP_PARAM_T *eventParam);
#endif /* CYBLE_PLXS_CLIENT */

/* [] END OF FILE */
