/**
 *   @file  mmw_messages.h
 *
 *   @brief
 *      This is the main header file for the Millimeter Wave Demo
 *
 *  Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/ 
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef MMW_MESSAGES_H
#define MMW_MESSAGES_H

#include "mmw_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief
 *  Message types used in Millimeter Wave Demo for Mailbox communication 
 * between MSS and DSS.
 *
 * @details
 *  The enum is used to hold all the messages types used for Mailbox communication
 * between MSS and DSS in mmw Demo.
 */
typedef enum MmwDemo_message_type_e 
{
    /*! @brief   message types for MSS to DSS communication */
    MMWDEMO_MSS2DSS_GUIMON_CFG = 0xFEED0001,
    MMWDEMO_MSS2DSS_CFAR_CFG,
    MMWDEMO_MSS2DSS_DOA_CFG,
    MMWDEMO_MSS2DSS_DBSCAN_CFG,
    MMWDEMO_MSS2DSS_TRACKING_CFG,
    MMWDEMO_MSS2DSS_DETOBJ_SHIPPED,
    MMWDEMO_MSS2DSS_SET_DATALOGGER,
    MMWDEMO_MSS2DSS_ADCBUFCFG,

    /*! @brief   message types for DSS to MSS communication */
    MMWDEMO_DSS2MSS_CONFIGDONE = 0xFEED0100,
    MMWDEMO_DSS2MSS_DETOBJ_READY

}MmwDemo_message_type;

/**
 * @brief
 *  Message for reporting detected objects from data path.
 *
 * @details
 *  The structure defines the message body for detected objects from from data path. 
 */
typedef struct MmwDemo_detObjMsg_t
{
    /*! @brief Address of the detected objects matix */
    uint32_t   detObjOutAddress;

    /*! @brief size of the detected objects matix in bytes */
    uint32_t   detObjOutsize;
}MmwDemo_detObjMsg;

/**
 * @brief
 *  Message for reporting static info from DSS to MSS.
 *
 * @details
 *  The structure defines the message body for heatmap size and address, and other static info after DSS configured, etc. 
 */
typedef struct MmwDemo_dssStaticInfo_t
{
    /*! @brief Address of the heatmap, stored in memory row by row*/
    uint32_t   heatmapAddress;

    /*! @brief row length of the heatmap */
    uint32_t   heatmapRowLen;

    /*! @brief number of rows of the heatmap */
    uint32_t   heatmapNumRows;
	
}MmwDemo_dssStaticInfo;

/**
 * @brief
 *  Message body used in Millimeter Wave Demo for passing configuration from MSS
 * to DSS.
 *
 * @details
 *  The union defines the message body for various configuration messages. 
 */
typedef union MmwDemo_message_body_u 
{
    /*! @brief   Detected Objects message */
    MmwDemo_detObjMsg     detObj;

    /*! @brief   Gui Monitor Selection */
    MmwDemo_GuiMonSel     guiMonSel;

    /*! @brief   CFAR configuraiton */
    mmwDemoCfarConfig       cfar;

    /*! @brief   DOA configuration */
    mmwDemoDoaConfig       doa;

    /*! @brief   tracking configuration */
    GTRACK_moduleConfig       tracking;

    /*! @brief   ADC Buffer configuration */
    MmwDemo_ADCBufCfg           adcBufCfg;

   /*! @brief   DSS static info message */
    MmwDemo_dssStaticInfo     dssStaticInfo;
	
    /*! @brief   Datapath output logger setting */
    uint8_t               dataLogger;
}MmwDemo_message_body;

/**
 * @brief
 *  DSS/MSS communication messages
 *
 * @details
 *  The structure defines the message structure used to commuincate between MSS
 * and DSS.
 */
typedef struct MmwDemo_message_t
{
    /*! @brief   message type */
    MmwDemo_message_type      type;

    /*! @brief   message length : PROC_TBA does body need to be pointer and not a union structure?*/
    //uint32_t                  len;

    /*! @brief  message body */
    MmwDemo_message_body      body;

} MmwDemo_message;

#ifdef __cplusplus
}
#endif

#endif /* MMW_MESSAGES_H */
