/**
 *   @file  mmw_output.h
 *
 *   @brief
 *      This is the interface/message header file for the Millimeter Wave Demo
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2016 Texas Instruments, Inc.
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
#ifndef MMW_OUTPUT_H
#define MMW_OUTPUT_H

#ifdef __cplusplus
extern "C" {
#endif
#ifdef SUBSYS_MSS
#include <gtrack.h>
#endif

#define ShortTarget
/** @brief Output packet length is a multiple of this value, must be power of 2*/
#define MMWDEMO_OUTPUT_MSG_SEGMENT_LEN 32

/*!
 * @brief
 *  Message types used in Millimeter Wave Demo for the communication between
 *  target and host, and also for Mailbox communication
 *  between MSS and DSS on the XWR16xx platform. Message types are used to indicate
 *  different type detection information sent out from the target.
 *
 */
typedef enum MmwDemo_output_message_type_e
{
    /*! @brief   List of detected points */
    MMWDEMO_OUTPUT_MSG_DETECTED_POINTS = 1,

    /*! @brief   Range profile */
    MMWDEMO_OUTPUT_MSG_RANGE_PROFILE,

    /*! @brief   Noise floor profile */
    MMWDEMO_OUTPUT_MSG_NOISE_PROFILE,

    /*! @brief   Samples to calculate static azimuth  heatmap */
    MMWDEMO_OUTPUT_MSG_AZIMUT_STATIC_HEAT_MAP,

    /*! @brief   Range/Doppler detection matrix */
    MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP,

    /*! @brief   Point Cloud - Array of detected points (range/angle/doppler) */
    MMWDEMO_OUTPUT_MSG_POINT_CLOUD,

    /*! @brief   Target List - Array of detected targets (position, velocity, error covariance) */
    MMWDEMO_OUTPUT_MSG_TARGET_LIST,

    /*! @brief   Target List - Array of target indices */
    MMWDEMO_OUTPUT_MSG_TARGET_INDEX,

    /*! @brief   Stats information */
    MMWDEMO_OUTPUT_MSG_STATS,

    MMWDEMO_OUTPUT_MSG_MAX
} MmwDemo_output_message_type;

/*!
 * @brief
 *  Message header for reporting detection information from data path.
 *
 * @details
 *  The structure defines the message header.
 */
typedef struct MmwDemo_output_message_header_t
{
    /*! @brief   Output buffer magic word (sync word). It is initialized to  {0x0102,0x0304,0x0506,0x0708} */
    uint16_t    magicWord[4];

    /*! @brief SW Version: : MajorNum * 2^24 + MinorNum * 2^16 + BugfixNum * 2^8 + BuildNum   */
    uint32_t    version;

    /*! @brief HW platform type */
    uint32_t    platform;

    /*! @brief Time in CPU cycles when the message was created, R4F CPU cycles */
    uint32_t    timeStamp;

    /*! @brief   Total packet length including header in Bytes */
    uint32_t    totalPacketLen;
    
    /*! @brief   Frame number */
    uint32_t    frameNumber;

    /*! @brief   For Advanced Frame config, this is the sub-frame number in the range
     * 0 to (number of subframes - 1). For frame config (not advanced), this is always
     * set to 0. */
    uint32_t    subFrameNumber;

    /*! @brief Detection Layer Margins */
    uint32_t    chirpProcessingMargin;
    uint32_t    frameProcessingMargin;
    
    /*! @brief Localization Layer Timing */
    uint32_t    trackingProcessingTime;
    uint32_t    uartSendingTime;

    /*! @brief Number of TLVs in this message*/
    uint16_t    numTLVs;
    /*! @brief Header checksum */
    uint16_t    checksum;

} MmwDemo_output_message_header;

/*!
 * @brief
 * Structure holds message stats information from data path.
 *
 * @details
 *  The structure holds stats information. This is a payload of the TLV message item
 *  that holds stats information.
 */
typedef struct MmwDemo_output_message_stats_t
{
    /*! @brief   Interframe processing time in usec */
    uint32_t     interFrameProcessingTime;

    /*! @brief   Transmission time of output detection informaion in usec */
    uint32_t     transmitOutputTime;

    /*! @brief   Interframe processing margin in usec */
    uint32_t     interFrameProcessingMargin;

    /*! @brief   Interchirp processing margin in usec */
    uint32_t     interChirpProcessingMargin;
    
    /*! @brief   CPU Load (%) during active frame duration */
    uint32_t     activeFrameCPULoad;
    
    /*! @brief   CPU Load (%) during inter frame duration */
    uint32_t     interFrameCPULoad;
    
} MmwDemo_output_message_stats;

/*!
 * @brief
 * Structure holds information about detected objects.
 *
 * @details
 * This information is sent in front of the array of detected objects
 */
typedef struct MmwDemo_output_message_dataObjDescr_t
{
    /*! @brief   Number of detected objects */
    uint16_t     numDetetedObj;

    /*! @brief   Q format of detected objects x/y/z coordinates */
    uint16_t     xyzQFormat;

} MmwDemo_output_message_dataObjDescr;


/**
 * @brief
 *  Message for reporting detected objects from data path.
 *
 * @details
 *  The structure defines the message body for detected objects from from data path. 
 */
typedef struct MmwDemo_output_message_tl_t
{
    /*! @brief   TLV type */
    uint32_t    type;
    
    /*! @brief   Length in bytes */
    uint32_t    length;

} MmwDemo_output_message_tl;

/*!
 * @brief
 * Structure holds the message body for the  Point Cloud
 *
 * @details
 * For each detected point, we report range, azimuth, and doppler 
 */
typedef struct MmwDemo_output_message_point_t
{
    /*! @brief Detected point range, in m */
    float		range;
    /*! @brief Detected point azimuth, in rad */
    float		azimuth;
    /*! @brief Detected point elev, in rad */
    float        elev;
    /*! @brief Detected point doppler, in m/s */
    float		doppler;
    /*! @brief Range detection SNR, linear */
    float       snr;

} MmwDemo_output_message_point;

typedef struct MmwDemo_output_message_pointCloud_t
{
    MmwDemo_output_message_tl       header;
    //MmwDemo_output_message_point    point[1];
    GTRACK_measurementPoint         point[1];
} MmwDemo_output_message_pointCloud;


/*!
 * @brief
 * Structure holds the target features
 *
 * @details
 * For each detected target, we report position, velocity, and measurement error covariance
 */
typedef struct MmwDemo_output_message_target_t
{
    /*! @brief   tracking ID */
    uint32_t    tid;
    /*! @brief   Detected target X coordinate, in m */
    float		posX;
    /*! @brief   Detected target Y coordinate, in m */
    float		posY;
    float       posZ;
    /*! @brief   Detected target X velocity, in m/s */
    float		velX;
    /*! @brief   Detected target Y velocity, in m/s */
    float		velY;
    float       velZ;
    /*! @brief   Detected target X acceleration, in m/s2 */
    float       accX;
    /*! @brief   Detected target Y acceleration, in m/s2 */
    float       accY;
    float       accZ;
#ifndef ShortTarget
    /*! @brief   Target Error covarience matrix, [3x3 float], in row major order, range, azimuth, doppler */
    float		ec[16];
    float       g;
#endif
	
} MmwDemo_output_message_target;

//typedef struct MmwDemo_output_message_short_target_t
//{
//    uint32_t tid;
//    /*! @brief   Detected target X coordinate, in m */
//    float       posX;
//    /*! @brief   Detected target Y coordinate, in m */
//    float       posY;
//    float       posZ;
//    /*! @brief   Detected target X velocity, in m/s */
//    float       velX;
//    /*! @brief   Detected target Y velocity, in m/s */
//    float       velY;
//    float       velZ;
//    /*! @brief   Detected target X acceleration, in m/s2 */
//    float       accX;
//    /*! @brief   Detected target Y acceleration, in m/s2 */
//    float       accY;
//    float       accZ;
//} MmwDemo_output_message_short_target;

/*!
 * @brief
 * Structure holds the message body for the Target List
 *
 * @details
 * For each detected target, we report position, velocity, and measurement error covariance
 */
typedef struct MmwDemo_output_message_targetList_t
{
    MmwDemo_output_message_tl       header;
    MmwDemo_output_message_target   target[1];
} MmwDemo_output_message_targetList;


////target list that only has S_hat info for each track, shortens UART send time
//typedef struct MmwDemo_output_message_short_targetList_t
//{
//    MmwDemo_output_message_tl       header;
//    MmwDemo_output_message_short_target   target[1];
//} MmwDemo_output_message_short_targetList;

/*!
 * @brief
 * Structure holds the message body for the Target Index.
 * This TLV is used to correlate each point in the point cloud with target
 *
 * @details
 * For each detected point, we report target index
 */
typedef struct MmwDemo_output_message_targetIndex_t
{
    MmwDemo_output_message_tl       header;
    uint8_t                         index[1];
} MmwDemo_output_message_targetIndex;

typedef struct MmwDemo_targetDescrHandle_t
{
    bool    currentDescr;
    MmwDemo_output_message_targetList *tList[2];
    MmwDemo_output_message_targetIndex *tIndex[2];
    /*! @brief   UART processing time in usec */
    uint32_t     uartProcessingTime;
    /*! @brief   track processing time in usec */
    uint32_t     trackProcessingTime;
} MmwDemo_targetDescrHandle;



#ifdef __cplusplus
}
#endif

#endif /* MMW_OUTPUT_H */
