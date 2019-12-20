/*!
 *  \file   RADARDEMO_clusterTracker.c
 *  \brief  Cluster tracking functions.
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

//#include <modules/tracking/common/api/RADARDEMO_tracking_commonDef.h>
#include <modules/tracking/clusterTracker/api/RADARDEMO_clusterTracker.h>
#include <modules/tracking/clusterTracker/src/RADARDEMO_clusterTracker_priv.h>
#include <math.h>
#include <stdio.h>
#include <string.h>


#define DEBUG(_x) //_x

#ifdef _TMS320C6X
#include "c6x.h"
#endif

//! \copydoc RADARDEMO_clusterTracker_create
void    * RADARDEMO_clusterTracker_create(
                            IN  RADARDEMO_clusterTracker_config * moduleConfig,
                            OUT RADARDEMO_clusterTracker_errorCode * errorCode)

{
    RADARDEMO_clusterTracker_handle * handle;
	uint16_t index;

    *errorCode  =   RADARDEMO_CLUSTERTRACKER_NO_ERROR;

    handle              =   (RADARDEMO_clusterTracker_handle *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, sizeof(RADARDEMO_clusterTracker_handle), 1);
    if (handle == NULL)
    {
        *errorCode = RADARDEMO_CLUSTERTRACKER_FAIL_ALLOCATE_HANDLE;
        return (handle);
    }
	// initialized with configuration structure
	handle->trackerAssociationThreshold = moduleConfig->trackerAssociationThreshold;
	handle->trackerActiveThreshold = moduleConfig->trackerActiveThreshold;
	handle->trackerForgetThreshold = moduleConfig->trackerForgetThreshold;
	handle->measurementNoiseVariance = moduleConfig->measurementNoiseVariance;
	handle->iirForgetFactor = moduleConfig->iirForgetFactor;
	handle->fxInputScalar = moduleConfig->fxInputScalar; 

	// memory allocation
    handle->trackerElementArray =   (RADARDEMO_trackerListElement *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, RADARDEMO_CT_MAX_NUM_TRACKER * sizeof(RADARDEMO_trackerListElement), 8);
    if (handle->trackerElementArray == NULL)
    {
        *errorCode  =   RADARDEMO_CLUSTERTRACKER_FAIL_ALLOCATE_LOCALINSTMEM;
        return (handle);
    }

    handle->Tracker =   (RADARDEMO_trackerInternal_dataType *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, RADARDEMO_CT_MAX_NUM_TRACKER * sizeof(RADARDEMO_trackerInternal_dataType), 8);
    if (handle->Tracker == NULL)
    {
        *errorCode  =   RADARDEMO_CLUSTERTRACKER_FAIL_ALLOCATE_LOCALINSTMEM;
        return (handle);
    }

	//scratchPad allocation for memory reuse cross different modules
	index = RADARDEMO_CT_MAX_NUM_TRACKER * sizeof(uint16_t); //PENDING indication
	index += RADARDEMO_CT_MAX_NUM_TRACKER * RADARDEMO_CT_MAX_NUM_ASSOC * sizeof(int16_t); // associateList
	index += RADARDEMO_CT_MAX_NUM_TRACKER * sizeof(uint16_t); //numAssoc
	index += RADARDEMO_CT_MAX_NUM_TRACKER * RADARDEMO_CT_MAX_NUM_CLUSTER * sizeof(float); // distance
    index += RADARDEMO_CT_MAX_NUM_CLUSTER * sizeof(RADARDEMO_trackerInputInternal_dataType); // inputInfo
	index += 96 * sizeof(float); // for 6 internal temp matrix operation
    handle->scratchPad  =   (char *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL1, 1, index, 8);
    if (handle->scratchPad == NULL)
    {
        *errorCode  =   RADARDEMO_CLUSTERTRACKER_FAIL_ALLOCATE_LOCALINSTMEM;
        return (handle);
    }

	// initialized the trackerList
	RADARDEMO_clusterTrackerList_init(handle);

    return((void *)handle);

}

//! \copydoc RADARDEMO_clusterTracker_delete
void    RADARDEMO_clusterTracker_delete(
                            IN  void * handle)
{
    RADARDEMO_clusterTracker_handle *cluterTrackerInst;

    cluterTrackerInst    =   (RADARDEMO_clusterTracker_handle *) handle;
    radarOsal_memFree(cluterTrackerInst->trackerElementArray,  RADARDEMO_CT_MAX_NUM_TRACKER * sizeof(RADARDEMO_trackerListElement));
    radarOsal_memFree(cluterTrackerInst->Tracker,  RADARDEMO_CT_MAX_NUM_TRACKER * sizeof(RADARDEMO_trackerInternal_dataType));
    radarOsal_memFree(cluterTrackerInst->scratchPad, 2 * RADARDEMO_CT_MAX_NUM_ASSOC * RADARDEMO_CT_MAX_NUM_TRACKER * sizeof(uint16_t));

    radarOsal_memFree(handle, sizeof(RADARDEMO_clusterTracker_handle));
}


//! \copydoc RADARDEMO_clusterTracker_run
RADARDEMO_clusterTracker_errorCode    RADARDEMO_clusterTracker_run(
                            IN  void * handle,
                            IN  RADARDEMO_tracker_input * input,
							IN  float dt,
                            OUT RADARDEMO_tracker_output * output)

{
	uint32_t index;
	uint16_t nTrack, i;
    RADARDEMO_clusterTracker_handle *clusterTrackerInst;
    RADARDEMO_clusterTracker_errorCode errorCode = RADARDEMO_CLUSTERTRACKER_NO_ERROR;

    clusterTrackerInst    =   (RADARDEMO_clusterTracker_handle *) handle;
	clusterTrackerInst->numOfInputMeasure = input->totalNumInput;

	if (input->totalNumInput > RADARDEMO_CT_MAX_NUM_CLUSTER)
		return (RADARDEMO_CLUSTERTRACKER_INPUT_EXCEED_MAX); 

    if ( (input->inputInfo == NULL) && (input->totalNumInput > 0))
        errorCode   =   RADARDEMO_CLUSTERTRACKER_INOUTPTR_NOTCORRECT;
    if ( output == NULL)
        errorCode   =   RADARDEMO_CLUSTERTRACKER_INOUTPTR_NOTCORRECT;
    if (  clusterTrackerInst->scratchPad == NULL)
        errorCode   =   RADARDEMO_CLUSTERTRACKER_INOUTPTR_NOTCORRECT;
    if (errorCode > RADARDEMO_CLUSTERTRACKER_NO_ERROR)
        return (errorCode);

    // Create measurements initialize pendingIndication, associationList, numAssoc and distance matrix
	index = 0; 
    clusterTrackerInst->pendingIndication = (uint16_t *) &clusterTrackerInst->scratchPad[index];
	for (i = 0; i < RADARDEMO_CT_MAX_NUM_TRACKER; i++)
		clusterTrackerInst->pendingIndication[i] = 1;
	index += RADARDEMO_CT_MAX_NUM_TRACKER * sizeof(uint16_t);
	
	clusterTrackerInst->associatedList = (int16_t *) &clusterTrackerInst->scratchPad[index];
	for (i = 0; i < RADARDEMO_CT_MAX_NUM_TRACKER * RADARDEMO_CT_MAX_NUM_ASSOC; i++)
		clusterTrackerInst->associatedList[i] = -1;	
	index += RADARDEMO_CT_MAX_NUM_TRACKER * RADARDEMO_CT_MAX_NUM_ASSOC * sizeof(int16_t);
	
	clusterTrackerInst->numAssoc = (uint16_t *) &clusterTrackerInst->scratchPad[index];
	for (i = 0; i < RADARDEMO_CT_MAX_NUM_TRACKER; i++)
		clusterTrackerInst->numAssoc[i] = 0;	
	index += RADARDEMO_CT_MAX_NUM_TRACKER * sizeof(uint16_t);
	
	clusterTrackerInst->dist = (float *) &clusterTrackerInst->scratchPad[index];
	
	nTrack = clusterTrackerInst->activeTrackerList.size;
	for (i = 0; i < input->totalNumInput * nTrack; i++)
		clusterTrackerInst->dist[i] = RADARDEMO_CT_MAX_DIST;
	index +=  RADARDEMO_CT_MAX_NUM_TRACKER * RADARDEMO_CT_MAX_NUM_CLUSTER * sizeof(float);

   	clusterTrackerInst->inputInfo = (RADARDEMO_trackerInputInternal_dataType *) &clusterTrackerInst->scratchPad[index];
	index += RADARDEMO_CT_MAX_NUM_CLUSTER * sizeof(RADARDEMO_trackerInputInternal_dataType); 

	RADARDEMO_clusterTracker_inputDataTransfer(input->totalNumInput, clusterTrackerInst->fxInputScalar, input->inputInfo, clusterTrackerInst->inputInfo);
	RADARDEMO_clusterTracker_updateFQ(clusterTrackerInst, dt);
    RADARDEMO_clusterTracker_timeUpdateTrackers(clusterTrackerInst);
	if (input->totalNumInput > 0)
	{
        errorCode = RADARDEMO_clusterTracker_associateTrackers(clusterTrackerInst);
        if (errorCode > RADARDEMO_CLUSTERTRACKER_NO_ERROR)
            return (errorCode);
        
		errorCode = RADARDEMO_clusterTracker_allocateNewTrackers(clusterTrackerInst);
        if (errorCode > RADARDEMO_CLUSTERTRACKER_NO_ERROR)
            return (errorCode);    
	}

    RADARDEMO_clusterTracker_updateTrackers(clusterTrackerInst, (float *) &clusterTrackerInst->scratchPad[index]);        
	RADARDEMO_clusterTracker_reportTrackers(clusterTrackerInst, output);
    return (errorCode);
}

