/*!
 *  \file   RADARDEMO_clusterTracker.h
 *
 *  \brief   Header file for RADARDEMO_clusterTracker module
 *
 * Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
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
 *
*/
//! \brief include file <swpform.h>
//!

#include <swpform.h>

//! \brief include file <radarOsal_malloc.h>
//!

#include <modules/utilities/radarOsal_malloc.h>
//#include <modules/tracking/common/api/RADARDEMO_tracking_commonDef.h>


#ifndef RADARDEMO_CLUSTERTRACKER_H
#define RADARDEMO_CLUSTERTRACKER_H


#define RADARDEMO_CT_MAX_NUM_TRACKER (64) // be multiple of 8 
#define RADARDEMO_CT_MAX_NUM_CLUSTER (24) // be multiple of 8

typedef struct _RADARDEMO_trackerInput_dataType_
{
    uint16_t    numPoints;             /**< number of points in this cluster */
    int16_t     xCenter;               /**< the clustering center on x direction */
    int16_t     yCenter;               /**< the clustering center on y direction */
    int16_t     xSize;                 /**< the clustering size on x direction */
    int16_t     ySize;                 /**< the clustering size on y direction */
    int16_t     avgVel;                /**< the min velocity within this cluster */
	float       centerRangeVar;        /**< variance of the range estimation */
	float       centerAngleVar;        /**< variance of the angle estimation */
	float       centerDopplerVar;      /**< variance of the doppler estimation */
}RADARDEMO_trackerInput_dataType;


typedef struct _RADARDEMO_trackerOutput_dataType_
{
    uint16_t      trackerID;                    /**< tracker ID.*/
    uint16_t      state;                        /**< tracker state.*/
    float         S_hat[4];                     /**< state vector is [x, y, vx, vy].*/
	float         xSize;                        /**< size in x direction */
	float         ySize;                        /**< size in y direction */
} RADARDEMO_trackerOutput_dataType;


typedef struct _RADARDEMO_tracker_input_
{
	uint16_t      totalNumInput;
	RADARDEMO_trackerInput_dataType *inputInfo;
} RADARDEMO_tracker_input;


typedef struct _RADARDEMO_tracker_output_
{
	uint16_t       totalNumOfOutput;
	RADARDEMO_trackerOutput_dataType *outputInfo;
} RADARDEMO_tracker_output;


//!  \brief   Error code for tracking module.
//!
typedef enum
{
    RADARDEMO_CLUSTERTRACKER_NO_ERROR = 0,                   /**< no error */
    RADARDEMO_CLUSTERTRACKER_FAIL_ALLOCATE_HANDLE,           /**< RADARDEMO_clusterTracker_create failed to allocate handle */
    RADARDEMO_CLUSTERTRACKER_FAIL_ALLOCATE_LOCALINSTMEM,     /**< RADARDEMO_clusterTracker_create failed to allocate memory for buffers in local instance */ 
    RADARDEMO_CLUSTERTRACKER_FAIL_ALLOCATE_TRACKER,          /**< RADARDEMO_clusterTracker_run failed to allocate new trackers  */
    RADARDEMO_CLUSTERTRACKER_INOUTPTR_NOTCORRECT,            /**< input and/or output buffer for RADARDEMO_clusterTracker_run are either NULL, or not aligned properly  */
    RADARDEMO_CLUSTERTRACKER_NUM_TRACKER_EXCEED_MAX,         /**< the number of tracker exceed the max value */
    RADARDEMO_CLUSTERTRACKER_NUM_ASSOC_EXCEED_MAX,           /**< the number of measure association exceed the max value */
    RADARDEMO_CLUSTERTRACKER_INPUT_EXCEED_MAX                /**< the number of input exceed the max value */
} RADARDEMO_clusterTracker_errorCode;


//!  \brief   Structure element of the list of descriptors for RADARDEMO_aoAEstBF configuration.
//!
typedef struct _RADARDEMO_clusterTracker_config_
{
    float        trackerAssociationThreshold;     /**< tracker association threshold.*/
	float        measurementNoiseVariance;        /**< measuremnet noise Variance */
	float        timePerFrame;                    /**< time peroid per frame */
    float        iirForgetFactor;                 /**< IIR filter forgetting factor */
	uint16_t     trackerActiveThreshold;          /**< counter threshold for tracker to turn from detect to active */
	uint16_t     trackerForgetThreshold;          /**< counter threshold for tracker to turn from active to expire */
	uint16_t     fxInputScalar;                   /**< fixed point input scaler */
} RADARDEMO_clusterTracker_config;



/*!
 *   \fn     RADARDEMO_clusterTracker_create
 *
 *   \brief   Create and initialize RADARDEMO_clusterTracker module.
 *
 *   \param[in]    moduleConfig
 *
 *   \param[out]    errorCode
 *
 *   \ret     void pointer to the module handle
 *
 *   \pre       none
 *
 *   \post      none
 *
 *
 */
extern void * RADARDEMO_clusterTracker_create(
                            IN  RADARDEMO_clusterTracker_config * moduleConfig,
                            OUT RADARDEMO_clusterTracker_errorCode * errorCode);

/*!
 *   \fn     RADARDEMO_clusterTracker_delete
 *
 *   \brief   Delete RADARDEMO_clusterTracker module.
 *
 *   \param[in]    handle
 *               Module handle.
 *
 *   \pre       none
 *
 *   \post      none
 *
 *
 */
extern void RADARDEMO_clusterTracker_delete(
                            IN  void * handle);


/*!
 *   \fn     RADARDEMO_clusterTracker_run
 *
 *   \brief   associate the input points/cluster with tracker and allocate new tracker / merge trackers/delete trackers if neccessary.
 *
 *   \param[in]    handle
 *               Module handle.
 *
 *   \param[in]    input
 *               Pointer to the points/cluster input info.
 *
 *   \param[out]    output
 *               Pointer to the tracker output info.
 *   \ret  error code
 *
 *   \pre       none
 *
 *   \post      none
 *
 *
 */
extern RADARDEMO_clusterTracker_errorCode RADARDEMO_clusterTracker_run(
                            IN  void * handle,
                            IN  RADARDEMO_tracker_input * input,
							IN  float dt,
                            OUT RADARDEMO_tracker_output * output);

#endif //RADARDEMO_CLUSTERTRACKER_H

