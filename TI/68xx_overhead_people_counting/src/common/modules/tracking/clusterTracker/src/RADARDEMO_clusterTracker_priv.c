/*!
 *   \file   RADARDEMO_clusterTracker_priv.c
 *
 *   \brief   Private functions for RADARDEMO_clusterTracker.
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
#include "modules/tracking/clusterTracker/api/RADARDEMO_clusterTracker.h"
//#include "modules/tracking/common/api/RADARDEMO_tracking_commonDef.h"
#include "RADARDEMO_clusterTracker_priv.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

#define DEBUG(_x) //_x

#ifdef _TMS320C6X
#include "c6x.h"
#endif


void RADARDEMO_clusterTracker_updateFQ(RADARDEMO_clusterTracker_handle *handle, float dt)
{
	uint16_t i;
    // initialize F and Q
    //float F[16] = {
	//	1, 0, dt, 0,
	//	0, 1, 0, dt,
	//	0, 0, 1, 0 ,
	//	0, 0, 0, 1};
    
	//float d = 1.0;
	float c = (float)(dt*dt*4.0); // (dt*2)^2 *d
	float b = (float)(c*dt*2);    // (dt*2)^3 *d
	float a = (float)(c*c);       // (dt*2)^4 *d

    //float Q[16] = {
	//	a, 0, b, 0,
	//	0, a, 0, b,
	//	b, 0, c, 0,
	//	0, b, 0, c};
	for (i = 0; i < 16; i++)
	{
		handle->F[i] = 0;
		handle->Q[i] = 0;
	}
	handle->F[0] = 1;
	handle->F[2] = dt;
	handle->F[5] = 1;
	handle->F[7] = dt;
	handle->F[10] = 1;
	handle->F[15] = 1;

	handle->Q[0] = a;
	handle->Q[2] = b;
	handle->Q[5] = a;
	handle->Q[7] = b;
	handle->Q[8] = b;
	handle->Q[10] = c;
	handle->Q[13] = b;
	handle->Q[15] = c;
}


void RADARDEMO_clusterTrackerList_init(RADARDEMO_clusterTracker_handle *handle) 
{
	uint16_t i;

	// initialize the idle and active tracker list
	handle->activeTrackerList.size = 0;
	handle->activeTrackerList.first = NULL;
	handle->activeTrackerList.last = NULL;
	handle->idleTrackerList.size = RADARDEMO_CT_MAX_NUM_TRACKER;
	handle->idleTrackerList.first = &handle->trackerElementArray[0];
	handle->idleTrackerList.last = &handle->trackerElementArray[RADARDEMO_CT_MAX_NUM_TRACKER-1];
	for (i = 1; i< RADARDEMO_CT_MAX_NUM_TRACKER; i++)
		handle->trackerElementArray[i-1].next =  &handle->trackerElementArray[i];
	for (i = 0; i< RADARDEMO_CT_MAX_NUM_TRACKER; i++)
		handle->trackerElementArray[i].tid =  i;
}

uint16_t RADARDEMO_clusterTrackerList_addToList(RADARDEMO_clusterTracker_handle *handle) 
{
	uint16_t tid;
	RADARDEMO_trackerListElement *firstPointer, *nextPointer;
	handle->activeTrackerList.size ++;
	firstPointer = handle->idleTrackerList.first;
	nextPointer = (RADARDEMO_trackerListElement *)(firstPointer->next); 
	tid = firstPointer->tid;
	firstPointer->next = handle->activeTrackerList.first;
	handle->activeTrackerList.first = firstPointer;
	//handle->activeTrackerList.last->next = firstPointer;
	//handle->activeTrackerList.last = firstPointer;
	//handle->activeTrackerList.last->next = NULL;
	handle->idleTrackerList.size --;
	handle->idleTrackerList.first = nextPointer;
	//debug zg:
	//printf("Tracker %d allocated", tid);
	return (tid);
}

void RADARDEMO_clusterTrackerList_removeFromList(RADARDEMO_clusterTracker_handle *handle, uint16_t listId) 
{
	uint16_t i;
	RADARDEMO_trackerListElement *cand, *oldLast, *newLast;
	if (listId == 0)
	{
		newLast = handle->activeTrackerList.first;
		handle->activeTrackerList.first = (RADARDEMO_trackerListElement *)(newLast->next);
	}
	else
	{
		cand =  handle->activeTrackerList.first;
		for (i = 0; i<(listId-1); i++)
			cand = (RADARDEMO_trackerListElement *)(cand->next);
		newLast = (RADARDEMO_trackerListElement *)(cand->next);
		cand->next = newLast->next;
	}
	handle->activeTrackerList.size --;
	
	handle->idleTrackerList.size ++;
	oldLast = handle->idleTrackerList.last;
	oldLast->next = newLast;
	newLast->next = NULL;
	handle->idleTrackerList.last = newLast;
	//debug zg:
	//printf("Tracker %d released", newLast->tid);

}

void RADARDEMO_clusterTracker_timeUpdateTrackers(RADARDEMO_clusterTracker_handle *handle)
{
     uint16_t i, tid;
	 int16_t numTracker = handle->activeTrackerList.size;
	 RADARDEMO_trackerListElement *tElem;
	 RADARDEMO_trackerInternal_dataType *tracker;
	 tElem = handle->activeTrackerList.first;
	 for (i = 0; i < numTracker; i++)
	 {
		 tid = tElem->tid; 
		 tracker = &handle->Tracker[tid];
		 // Prediction based on state equation
		 RADARDEMO_tracker_matrixMultiply(4, 4, 1, handle->F, tracker->S_hat, tracker->S_apriori_hat);
		 // convert from spherical to Cartesian coordinates
		 RADARDEMO_tracker_computeH(tracker->S_apriori_hat, tracker->H_s_apriori_hat);
		 tElem = (RADARDEMO_trackerListElement *)(tElem->next);
	 }
}

RADARDEMO_clusterTracker_errorCode RADARDEMO_clusterTracker_associateTrackers(RADARDEMO_clusterTracker_handle *handle)
{
	uint16_t nTrack = handle->activeTrackerList.size;
	uint16_t nMeas = handle->numOfInputMeasure;
	uint16_t mid, n, j, tid, index, minTid, len, assocIndex;
	float minDist, distTH, dist;
	float *distPtr;
    RADARDEMO_trackerListElement *tElem;
	RADARDEMO_trackerInternal_dataType *tracker;
	RADARDEMO_clusterTracker_errorCode errorCode;
	bool hitFlag;

	errorCode = RADARDEMO_CLUSTERTRACKER_NO_ERROR;
    // Apply association only if both trackerList and number of measurement are nonZero
	if ((nMeas > 0) && (nTrack > 0)) 
	{
		index = 0;
		for (mid = 0; mid < nMeas; mid++)
		{
			tElem = handle->activeTrackerList.first;
			// calculate the distance from one measure to all the trackers
			// record the min dist, and associate index
			minDist = RADARDEMO_CT_MAX_DIST;
			for (j = 0; j < nTrack; j ++)
			{
			    tid = tElem->tid;
		        tracker = &handle->Tracker[tid];
				distPtr = &handle->dist[index + j];
				RADARDEMO_clusterTracker_distCalc(&handle->inputInfo[mid], tracker, distPtr);
				if ((*distPtr) < minDist)
				{
					minDist = (*distPtr);
					//minTrackListIndex = j;
					minTid = tid;
				}
				tElem = (RADARDEMO_trackerListElement *)(tElem->next);
			}
    		// compare minDist with threshold
			distTH = RADARDEMO_clusterTracker_associateThresholdCalc(handle->trackerAssociationThreshold, &handle->Tracker[minTid]);
			if (minDist < distTH)
			{
				len = handle->numAssoc[minTid]; 
				assocIndex = minTid * RADARDEMO_CT_MAX_NUM_ASSOC + len;
				// add to the association list
				handle->associatedList[assocIndex] = mid; 
				handle->numAssoc[minTid] ++;
				// remove from pending Indication list
				handle->pendingIndication[mid] = 0;
				// error protection
				if (handle->numAssoc[minTid] >= RADARDEMO_CT_MAX_NUM_ASSOC)
				{
					errorCode = RADARDEMO_CLUSTERTRACKER_NUM_ASSOC_EXCEED_MAX;
					return (errorCode);
				}
			}
			index = index + nTrack;
		} // for nMeas

    	tElem = handle->activeTrackerList.first;
		for (n = 0; n < nTrack; n++)
		{
		    tid = tElem->tid;
			if (handle->numAssoc[tid] == 0)
			{
				minDist = RADARDEMO_CT_MAX_DIST; 
				for (mid = 0; mid < nMeas; mid++)
				{
					index = mid * nTrack + n; 
					dist = handle->dist[index]; 
					if ( dist < minDist)
						minDist = dist;
				}
    		    // compare minDist with threshold
			    distTH = RADARDEMO_clusterTracker_associateThresholdCalc(handle->trackerAssociationThreshold, &handle->Tracker[tid]);
				if (minDist < distTH)
                {
					hitFlag = 1;
 		            RADARDEMO_clusterTracker_updateTrackerStateMachine(&handle->Tracker[tid], hitFlag);
				}
			}
			tElem = (RADARDEMO_trackerListElement *)(tElem->next);
		}



	} // if 
	return (errorCode);
}

RADARDEMO_clusterTracker_errorCode RADARDEMO_clusterTracker_allocateNewTrackers(RADARDEMO_clusterTracker_handle *handle)
{
	//uint16_t nTrack = handle->activeTrackerList.size;
	uint16_t nMeas = handle->numOfInputMeasure;
	uint16_t i, j, mid, tid;
	RADARDEMO_clusterTracker_errorCode errorCode; 
	RADARDEMO_trackerInternal_dataType *tracker;
	RADARDEMO_trackerInputInternal_dataType *inputPoint;
	float H[3];

	errorCode = RADARDEMO_CLUSTERTRACKER_NO_ERROR;
	for (mid = 0; mid < nMeas; mid++)
	{
		if (handle->pendingIndication[mid] == 1)
		{
			if (handle->idleTrackerList.size > 0)
			{
				tid = RADARDEMO_clusterTrackerList_addToList(handle); 
				tracker = &handle->Tracker[tid];
				inputPoint = &handle->inputInfo[mid];
				tracker->state = RADARDEMO_TRACKER_STATE_DETECTION;
				tracker->detect2activeCount = 0;
				tracker->detect2freeCount = 0;
				tracker->active2freeCount = 0;
				tracker->activeThreshold = handle->trackerActiveThreshold;
				tracker->forgetThreshold = handle->trackerForgetThreshold;
				tracker->xSize = inputPoint->xSize;
				tracker->ySize = inputPoint->ySize;
				tracker->diagonal2 = inputPoint->xSize * inputPoint->xSize + 
					inputPoint->ySize * inputPoint->ySize;
				tracker->speed2 = inputPoint->doppler * inputPoint->doppler;
				tracker->doppler = inputPoint->doppler;

				// may add the debug print out for new tracker allocation
				handle->pendingIndication[mid] = 0;
				//handle->associatedList[tid * RADARDEMO_CT_MAX_NUM_ASSOC] = mid;
				//handle->numAssoc[tid] = 1;

				// Initialization S_hat, P
				H[0] = inputPoint->range;
				H[1] = inputPoint->azimuth;
				H[2] = inputPoint->doppler;
				RADARDEMO_clusterTracker_computeCartesian(H, tracker->S_hat); 
				//obj.P(:,:,id) = diag(ones(1,obj.pStateVectorLength));
				for (i = 0; i < 4; i++)
				{
					for (j = 0; j < 4; j++)
						tracker->P[i*4 + j] = 0;
					tracker->P[i*4 + i] = 1;
				}
				RADARDEMO_tracker_matrixMultiply(4, 4, 1, handle->F, tracker->S_hat, tracker->S_apriori_hat);
				RADARDEMO_tracker_computeH(tracker->S_apriori_hat, tracker->H_s_apriori_hat);  
			}
			else
				errorCode = RADARDEMO_CLUSTERTRACKER_FAIL_ALLOCATE_TRACKER;
		}
	}

	return(errorCode);
}

void RADARDEMO_clusterTracker_updateTrackers(RADARDEMO_clusterTracker_handle *handle, float *scratchPad)
{
	float R[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
	uint16_t nTrack = handle->activeTrackerList.size;
	uint16_t numExpireTracker = 0;
	uint16_t expireList[RADARMEDO_CT_MAX_NUM_EXPIRE];
    RADARDEMO_trackerListElement *tElem;
	RADARDEMO_trackerInternal_dataType *tracker;
	RADARDEMO_trackerInputInternal_dataType  combinedInput;
	RADARDEMO_trackerInputInternal_dataType  *combinedInputPtr;
	uint16_t i, tid, mid, index;
	int16_t j;
	float diag2;
    bool hitFlag;
    
	tElem = handle->activeTrackerList.first;
	//debug zg:
	//printf("\n");
	for (i = 0; i < nTrack; i++)
	{
		tid = tElem->tid;
		index = tid * RADARDEMO_CT_MAX_NUM_ASSOC;
		tracker = &handle->Tracker[tid];
		//debug zg:
		//printf("associatedList[%d] = %d   ", index, handle->associatedList[index] + 1);

		if (handle->associatedList[index] > -1)
		{
			//there is some measurement associate with this tracker
			hitFlag = 1;
			RADARDEMO_clusterTracker_updateTrackerStateMachine(tracker, hitFlag);

			//calculate the new measure based on all the assocated measures, xSize and ySize will be set to the maximum xSize and ySize
			mid = handle->associatedList[index];
			combinedInputPtr = &handle->inputInfo[mid];
			if (handle->numAssoc[tid] > 1)
			{
				RADARDEMO_clusterTracker_combineMeasure(handle->inputInfo, &handle->associatedList[index], handle->numAssoc[tid], &combinedInput); 
				combinedInputPtr = &combinedInput;
			}
				
			// construct R matrix
			R[0] = combinedInputPtr->rangeVar * handle->measurementNoiseVariance;
			R[4] = combinedInputPtr->angleVar * handle->measurementNoiseVariance;
			R[8] = combinedInputPtr->dopplerVar * handle->measurementNoiseVariance;
			
			RADARDEMO_clusterTracker_kalmanUpdate(tracker, combinedInputPtr, handle->F, handle->Q, R, scratchPad);

			// update speed2 and doppler
			tracker->speed2 = tracker->S_hat[2] * tracker->S_hat[2] +  tracker->S_hat[3] * tracker->S_hat[3];
            tracker->doppler = combinedInputPtr->doppler;

			// update xSize and ySize
			tracker->xSize = RADARDEMO_clusterTracker_IIRFilter(tracker->xSize, combinedInputPtr->xSize, handle->iirForgetFactor);
			tracker->ySize = RADARDEMO_clusterTracker_IIRFilter(tracker->ySize, combinedInputPtr->ySize, handle->iirForgetFactor);
            diag2 = tracker->xSize * tracker->xSize + tracker->ySize * tracker->ySize;
			if (diag2 > tracker->diagonal2)
			    tracker->diagonal2 = diag2;

		}
		else
		{
			hitFlag = 0;
			RADARDEMO_clusterTracker_updateTrackerStateMachine(tracker, hitFlag);

			// consider free the tracker or do time update, if too much tracker needs to be released then wait for next time.
            if ((tracker->state == RADARDEMO_TRACKER_STATE_EXPIRE) && (numExpireTracker < (RADARMEDO_CT_MAX_NUM_EXPIRE -1) ))
			{
				//collect the free list and free at the end of the function;
				expireList[numExpireTracker] = i;
                numExpireTracker ++;
			}
            else 
                RADARDEMO_clusterTracker_kalmanUpdateWithNoMeasure(tracker, handle->F, handle->Q);

		}
		tElem = (RADARDEMO_trackerListElement *)(tElem->next);
	}
	// free the expired tracker
	for (j = numExpireTracker - 1; j >= 0; j--)
	    RADARDEMO_clusterTrackerList_removeFromList(handle, expireList[j]);

}

void RADARDEMO_clusterTracker_updateTrackerStateMachine(RADARDEMO_trackerInternal_dataType *tracker,  bool hitFlag)
{

    switch (tracker->state)
	{
	    case RADARDEMO_TRACKER_STATE_DETECTION:
			if(hitFlag)
			{
				tracker->detect2freeCount = 0;
				if(tracker->detect2activeCount > tracker->activeThreshold)
					tracker->state = RADARDEMO_TRACKER_STATE_ACTIVE;
				else
					tracker->detect2activeCount ++;
			}
			else
			{
				if(tracker->detect2freeCount > tracker->forgetThreshold)
					tracker->state = RADARDEMO_TRACKER_STATE_EXPIRE;
				else
					tracker->detect2freeCount ++;
				if(tracker->detect2activeCount > 0)
					tracker->detect2activeCount --;
			}
			break;

	    case RADARDEMO_TRACKER_STATE_ACTIVE:
			if(hitFlag)
			{
				if(tracker->active2freeCount > 0)
					tracker->active2freeCount --;                                  
			}
			else
			{
				if(tracker->active2freeCount > tracker->forgetThreshold)
					tracker->state = RADARDEMO_TRACKER_STATE_EXPIRE;
				else
					tracker->active2freeCount ++;
			}
			break;
	}
}


void RADARDEMO_clusterTracker_reportTrackers(RADARDEMO_clusterTracker_handle *handle, RADARDEMO_tracker_output *output)
{
	uint16_t n, nTracker, tid;
    RADARDEMO_trackerListElement *tElem;
	RADARDEMO_trackerInternal_dataType *tracker;
	nTracker = handle->activeTrackerList.size;

	if (nTracker > RADARDEMO_CT_MAX_NUM_TRACKER)
		nTracker = RADARDEMO_CT_MAX_NUM_TRACKER;
	output->totalNumOfOutput = nTracker;
	
	tElem = handle->activeTrackerList.first;
	for (n = 0; n < nTracker; n++)
	{
		tid = tElem->tid;
		tracker = &handle->Tracker[tid];
		output->outputInfo[n].trackerID = tid;
		output->outputInfo[n].S_hat[0] = tracker->S_hat[0];
		output->outputInfo[n].S_hat[1] = tracker->S_hat[1];
		output->outputInfo[n].S_hat[2] = tracker->S_hat[2];
		output->outputInfo[n].S_hat[3] = tracker->S_hat[3];
		output->outputInfo[n].xSize = tracker->xSize;
		output->outputInfo[n].ySize = tracker->ySize;
		tElem = (RADARDEMO_trackerListElement *)(tElem->next);

	}

}

void RADARDEMO_clusterTracker_inputDataTransfer(uint16_t numCluster, uint16_t scale, RADARDEMO_trackerInput_dataType *inputInfo, RADARDEMO_trackerInputInternal_dataType *internalInputInfo)
{

	uint16_t i;
	float posx, posy;
	float scaleInv = 1.0 / (float)(scale); 
	for (i = 0; i < numCluster; i++)
	{
		internalInputInfo[i].xSize = (float)(inputInfo[i].xSize) *scaleInv;
		internalInputInfo[i].ySize = (float)(inputInfo[i].ySize) *scaleInv;
		internalInputInfo[i].doppler = (float)(inputInfo[i].avgVel) *scaleInv;
		internalInputInfo[i].numPoints = inputInfo[i].numPoints;
		internalInputInfo[i].rangeVar = inputInfo[i].centerRangeVar *scaleInv *scaleInv;
		internalInputInfo[i].angleVar = inputInfo[i].centerAngleVar;
		internalInputInfo[i].dopplerVar = inputInfo[i].centerDopplerVar * scaleInv *scaleInv;
		posx = (float)(inputInfo[i].xCenter) *scaleInv;
		posy = (float)(inputInfo[i].yCenter) *scaleInv;
		internalInputInfo[i].range = (float)(sqrt(posx*posx + posy*posy)); 

		//calc azimuth
		if (posx == 0)
			internalInputInfo[i].azimuth = RADARDEMO_CLUSTERTRACKER_PI/2;
		else if (posx > 0)
			internalInputInfo[i].azimuth = (float)(atan(posy/posx));
		else
			internalInputInfo[i].azimuth = (float)(atan(posy/posx)) + RADARDEMO_CLUSTERTRACKER_PI;
	}

}
