/*!
 *  \file   RADARDEMO_clusterTracker_priv.h
 *
 *  \brief   Header file for RADARDEMO_clusterTracker_priv.c
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
#ifndef RADARDEMO_CLUSTERTRACKER_PRIV_H
#define RADARDEMO_CLUSTERTRACKER_PRIV_H

#include <swpform.h>

#define RADARDEMO_CT_MAX_NUM_ASSOC (6)
#define RADARMEDO_CT_MAX_NUM_EXPIRE (16)
#define RADARDEMO_CT_MAX_DIST (10000000000.f)

#define RADARDEMO_CLUSTERTRACKER_PIOVER180 (3.141592653589793/180.0)      //!< define the pi/180
#define RADARDEMO_CLUSTERTRACKER_PI        (3.141592653589793f)           //!< define pi

typedef enum
{
	RADARDEMO_TRACKER_STATE_EXPIRE,
    RADARDEMO_TRACKER_STATE_DETECTION,
    RADARDEMO_TRACKER_STATE_ACTIVE
} RADARDEMO_clusterTracker_state;

typedef struct _RADARDEMO_trackerInputInternal_dataType_
{
	uint16_t     numPoints;                     /**< Number of samples in this cluster */
    float        range;                         /**< cluster center location in range direction.*/
	float        azimuth;                       /**< cluster center location in angle .*/
	float        doppler;                       /**< averge doppler of the cluster.*/
	float        xSize;                         /**< max distance of point from cluster center in x direction.*/
	float        ySize;                         /**< max distance of point from cluster center in y direction.*/
	float        rangeVar;                      /**< variance of the range estimation */
	float        angleVar;                      /**< variance of the angle estimation */
	float        dopplerVar;                    /**< variance of the doppler estimation */
}RADARDEMO_trackerInputInternal_dataType;

//!   \brief  Internal structure for each tracker
typedef struct _RADARDEMO_trackerInternal_dataType_
{ 
    RADARDEMO_clusterTracker_state      state;                        /**< tracker state.*/
    uint16_t      detect2activeCount;
    uint16_t      detect2freeCount;
    uint16_t      active2freeCount;
    uint16_t      activeThreshold;
    uint16_t      forgetThreshold;
    float         S_hat[4];                     /**< state vector is [x, y, vx, vy].*/
	float         P[16];
    float         P_apriori[16];
    float         S_apriori_hat[4];
    float         H_s_apriori_hat[3];
	float         speed2;                       /**< speed in amplitude square: vx^2 + vy^2  */
	float         doppler;                      /**< average doppler for all the associated points/clusters */
	float         xSize;                        /**< xSize after IIR filtering */
	float         ySize;                        /**< ySize after IIR filtering */
	float         diagonal2;                    /**< diagnonal square for this tracker */
} RADARDEMO_trackerInternal_dataType;


typedef struct _RADARDEMO_trackerListElement_
{
	int16_t  tid;
	//RADARDEMO_trackerListElement *next;
	void *next;
} RADARDEMO_trackerListElement;


typedef struct _RADARDEMO_trackerList_
{
	int16_t  size;
	RADARDEMO_trackerListElement *first;
	RADARDEMO_trackerListElement *last;
} RADARDEMO_trackerList;


//!  \brief   Structure element of the list of descriptors for UL allocations.
//!
typedef struct _RADARDEMO_clusterTracker_handle
{    
    float trackerAssociationThreshold;
    uint16_t trackerActiveThreshold;
	uint16_t trackerForgetThreshold;
	uint16_t fxInputScalar;
	float measurementNoiseVariance;
	float iirForgetFactor;
	float Q[16]; //4x4
	float F[16]; //4x4
    RADARDEMO_trackerListElement *trackerElementArray; //[RADARDEMO_CT_MAX_NUM_TRACKER];
	RADARDEMO_trackerList idleTrackerList;
	RADARDEMO_trackerList activeTrackerList;
    int16_t *associatedList; //[RADARDEMO_CT_MAX_NUM_TRACKER * RADARDEMO_CT_MAX_NUM_ASSOC];
	uint16_t *pendingIndication; //[RADARDEMO_CT_MAX_NUM_TRACKER];
    RADARDEMO_trackerInternal_dataType *Tracker; //[RADARDEMO_CT_MAX_NUM_TRACKER];
	char *scratchPad;
	uint16_t numOfInputMeasure;
	RADARDEMO_trackerInputInternal_dataType *inputInfo; 
	uint16_t *numAssoc;  
	float *dist; //pointer to the distance matrix
} RADARDEMO_clusterTracker_handle;

void RADARDEMO_clusterTracker_inputDataTransfer(uint16_t numCluster, uint16_t scale, RADARDEMO_trackerInput_dataType *inputInfo, RADARDEMO_trackerInputInternal_dataType *internalInputInfo);
void RADARDEMO_clusterTracker_updateFQ(RADARDEMO_clusterTracker_handle *handle, float dt);
void RADARDEMO_clusterTracker_timeUpdateTrackers(RADARDEMO_clusterTracker_handle *handle);
RADARDEMO_clusterTracker_errorCode RADARDEMO_clusterTracker_associateTrackers(RADARDEMO_clusterTracker_handle *handle);
void RADARDEMO_clusterTracker_mergeTrackers(RADARDEMO_clusterTracker_handle *handle);
RADARDEMO_clusterTracker_errorCode RADARDEMO_clusterTracker_allocateNewTrackers(RADARDEMO_clusterTracker_handle *handle);
void RADARDEMO_clusterTracker_updateTrackers(RADARDEMO_clusterTracker_handle *handle, float *scratchPad);
void RADARDEMO_clusterTracker_reportTrackers(RADARDEMO_clusterTracker_handle *handle, RADARDEMO_tracker_output *output);

void RADARDEMO_clusterTrackerList_init(RADARDEMO_clusterTracker_handle *handle); 
uint16_t RADARDEMO_clusterTrackerList_addToList(RADARDEMO_clusterTracker_handle *handle); 
void RADARDEMO_clusterTrackerList_removeFromList(RADARDEMO_clusterTracker_handle *handle, uint16_t listId); 

void RADARDEMO_clusterTracker_updateTrackerStateMachine(RADARDEMO_trackerInternal_dataType *tracker,  bool hitFlag);

void RADARDEMO_clusterTracker_kalmanUpdate(RADARDEMO_trackerInternal_dataType *tracker, 
	                                       RADARDEMO_trackerInputInternal_dataType  *input,
										   float *F,
										   float *Q,
										   float *R,
										   float *scratchPad);
void RADARDEMO_clusterTracker_kalmanUpdateWithNoMeasure(RADARDEMO_trackerInternal_dataType *tracker, float *F, float *Q);

void RADARDEMO_tracker_matrixMultiply(uint16_t m1, uint16_t m2, uint16_t m3, float *A, float *B, float *C);
void RADARDEMO_tracker_matrixConjugateMultiply(uint16_t m1, uint16_t m2, uint16_t m3, float *A, float *B, float *C);
void RADARDEMO_tracker_matrixAdd(uint16_t dim, float *A, float *B, float *C);
void RADARDEMO_tracker_matrixSub(uint16_t dim, float *A, float *B, float *C);
void RADARDEMO_cluster_matInv3(float *Ac, float *Ainv);
void RADARDEMO_tracker_cholesky3(float *A, float *G);
void RADARDEMO_tracker_computeH(float *S, float *H);
void RADARDEMO_clusterTracker_computeCartesian(float *H, float *S);
void RADARDEMO_clusterTracker_computeJacobian(float *S, float *J);
void RADARDEMO_clusterTracker_distCalc(RADARDEMO_trackerInputInternal_dataType *measureInfo, RADARDEMO_trackerInternal_dataType *tracker, float *dist);
float RADARDEMO_clusterTracker_associateThresholdCalc(float presetTH, RADARDEMO_trackerInternal_dataType *tracker);
void RADARDEMO_clusterTracker_combineMeasure(RADARDEMO_trackerInputInternal_dataType *inputInfo, int16_t *associatedList, uint16_t numAssoc, RADARDEMO_trackerInputInternal_dataType *combinedInput);
float RADARDEMO_clusterTracker_IIRFilter(float x, float y, float forgetFactor);


#endif //RADARDEMO_CLUSTERTRACKER_PRIV_H

