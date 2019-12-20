/*!
 *  \file   RADARDEMO_clusteringDBscan.c
 *
 *  \brief   DBscan clustering module.
 *
 *  Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/ 
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

#include <math.h>
#include <string.h>
#include <modules/clustering/dbscan/api/RADARDEMO_clusteringDBscan.h>
#include "RADARDEMO_clusteringDBscan_priv.h"



//! \copydoc RADARDEMO_clusteringDBscanCreate
void * RADARDEMO_clusteringDBscanCreate( RADARDEMO_clusteringDBscanConfig *param)
{

    RADARDEMO_clusteringDBscanInstance *inst;
    unsigned int memoryUsed = 0;

    memoryUsed          +=  sizeof(RADARDEMO_clusteringDBscanInstance);
    inst                =   (RADARDEMO_clusteringDBscanInstance *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, sizeof(RADARDEMO_clusteringDBscanInstance), 8);
    inst->epsilon       =   param->epsilon;
    inst->vFactor       =   param->vFactor;
    inst->weight        =   param->weight * param->weight * param->epsilon; //inst->weight * inst->weight * epsilon;
    inst->maxClusters   =   param->maxClusters;
    inst->minPointsInCluster    =   param->minPointsInCluster;
    inst->maxPoints     =   param->maxPoints;
    inst->inputPntPrecision     =   param->inputPntPrecision;
    inst->fixedPointScale      =   param->fixedPointScale;

    memoryUsed          +=  inst->maxPoints *(sizeof(char) *2 + sizeof(uint16_t));

    inst->scratchPad    =   (char *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL1, 1, inst->maxPoints *(sizeof(char) *2 + sizeof(uint16_t)), 8); /* 1 is the flag for scratch*/
    inst->visited       =   (char *)& inst->scratchPad[0];
    inst->scope         =   (char *)& inst->scratchPad[inst->maxPoints];
    inst->neighbors     =   (uint16_t *)& inst->scratchPad[2 * inst->maxPoints];

#ifdef DBSCAN_DEBUG
	memoryUsed          +=  sizeof(RADARDEMO_clusteringDBscanDebugInfo);
    inst->debugInfo     =   (RADARDEMO_clusteringDBscanDebugInfo *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0 , sizeof(RADARDEMO_clusteringDBscanDebugInfo), 8);

    memoryUsed          +=  sizeof(RADARDEMO_clusteringDBscanClusterInfo);
    inst->debugInfo->clusterInfo=   (RADARDEMO_clusteringDBscanClusterInfo *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, sizeof(RADARDEMO_clusteringDBscanClusterInfo), 8);
    inst->debugInfo->memoryUsed =   memoryUsed;
#endif

    return (void *)inst;
}

//! \copydoc RADARDEMO_clusteringDBscanRun
int32_t RADARDEMO_clusteringDBscanRun( void *handle,
                                       RADARDEMO_clusteringDBscanInput *input,
                                       RADARDEMO_clusteringDBscanOutput *output){

    RADARDEMO_clusteringDBscanInstance *inst = (RADARDEMO_clusteringDBscanInstance *)handle;

    uint16_t *neighLast;
    uint16_t *neighCurrent;
    uint16_t neighCount;
    uint16_t newCount;
    uint16_t point, member;
    uint16_t numPoints;
    uint16_t clusterId;
    uint16_t ind;

    float epsilon,epsilon2, weight;
    int32_t epsilon2fixed;

    numPoints	=	input->numPoints;
    clusterId	=	0;

    epsilon		=	inst->epsilon;
    epsilon2	=	epsilon*epsilon;
    weight		=	inst->weight;
#ifdef __TI_ARM_V7R4__
    epsilon2fixed   =   (int32_t) (epsilon2 * (float)(inst->fixedPointScale * inst->fixedPointScale));
#else
    epsilon2fixed   =   _spint(epsilon2 * (float)(inst->fixedPointScale * inst->fixedPointScale));
#endif
    memset(inst->visited, POINT_UNKNOWN, numPoints*sizeof(char));
#ifdef DBSCAN_DEBUG
	inst->debugInfo->clusterInfo->numPoints[0] = 0;
#endif

    // scan through all the points to find its neighbors
    for(point = 0; point < numPoints; point++)
    {
        if(inst->visited[point] != POINT_VISITED)
        {
            inst->visited[point] = POINT_VISITED;

            neighCurrent = neighLast = inst->neighbors;
            // scope is the local copy of visit
            memcpy(inst->scope, inst->visited, numPoints*sizeof(char));

            if (input->inputPntPrecision == DBSCAN_INPUT_FLOATINGPT)
                neighCount = RADARDEMO_clusteringDBscan_findNeighbors2(
					(RADARDEMO_clusteringDBscanPoint2d *)input->pointArray.pointArrayFloat, 
					input->speed.speedFloat, 
					point, 
					neighLast, 
					numPoints, 
					epsilon2, 
					weight, 
					inst->vFactor,
					inst->scope, 
					&newCount);
                // ZY: optimized function RADARDEMO_clusteringDBscan_findNeighbors3 need to rework
                //neighCount = RADARDEMO_clusteringDBscan_findNeighbors3((RADARDEMO_clusteringDBscanPoint2d *)input->pointArray, point, neighLast, numPoints, epsilon2, inst->scope, &newCount);
            else
                neighCount = RADARDEMO_clusteringDBscan_findNeighbors2Fixed(
					(RADARDEMO_clusteringDBscanPoint2dfxdp *)input->pointArray.pointArrayInt16, 
					input->speed.speedInt16, 
					point, 
					neighLast, 
					numPoints, 
					epsilon2fixed, 
					weight, 
					(int32_t)(inst->vFactor * inst->fixedPointScale),
					inst->scope, 
					&newCount);
                // ZY: optimized function RADARDEMO_clusteringDBscan_findNeighbors3Fixed need to rework
                //neighCount = RADARDEMO_clusteringDBscan_findNeighbors3Fixed((RADARDEMO_clusteringDBscanPoint2dfxdp *)input->pointArray, point, neighLast, numPoints, epsilon2fixed, inst->scope, &newCount);

            if(neighCount < inst->minPointsInCluster)
            {
                // This point is Noise
                output->IndexArray[point] = 0;
#ifdef DBSCAN_DEBUG
				inst->debugInfo->clusterInfo->numPoints[0]++;
#endif
            }
            else
            {
                // This point belongs to a New Cluster
                clusterId++;                                // New cluster ID
                output->IndexArray[point] = clusterId;      // This point belong to this cluster
#ifdef DBSCAN_DEBUG
                inst->debugInfo->clusterInfo->numPoints[clusterId] = 1;
#endif

                // tag all the neighbors as visited in scope so that it will not be found again when searching neighbor's neighbor.
                for (ind = 0; ind < newCount; ind ++) {
                    member = neighLast[ind];
                    inst->scope[member] = POINT_VISITED;
                }
                neighLast += newCount;

                while (neighCurrent != neighLast)               // neigh shall be at least minPoints in front of neighborhood pointer
                {
                    // Explore the neighborhood
                    member = *neighCurrent++;               // Take point from the neighborhood
                    output->IndexArray[member] = clusterId; // All points from the neighborhood also belong to this cluster
                    inst->visited[member] = POINT_VISITED;

#ifdef DBSCAN_DEBUG
                    inst->debugInfo->clusterInfo->numPoints[clusterId]++;
#endif
                    if (input->inputPntPrecision == DBSCAN_INPUT_FLOATINGPT)
                        neighCount = RADARDEMO_clusteringDBscan_findNeighbors2(
							(RADARDEMO_clusteringDBscanPoint2d *)input->pointArray.pointArrayFloat , 
							input->speed.speedFloat, 
							member, 
							neighLast, 
							numPoints, 
							epsilon2, 
							weight, 
							inst->vFactor,
							inst->scope, 
							&newCount);
                        // ZY: optimized function RADARDEMO_clusteringDBscan_findNeighbors3 need to rework
                        //neighCount = RADARDEMO_clusteringDBscan_findNeighbors3((RADARDEMO_clusteringDBscanPoint2d *)input->pointArray, member, neighLast, numPoints, epsilon2, inst->scope, &newCount);
                    else
                        neighCount = RADARDEMO_clusteringDBscan_findNeighbors2Fixed(
							(RADARDEMO_clusteringDBscanPoint2dfxdp *)input->pointArray.pointArrayInt16, 
							input->speed.speedInt16, 
							member, 
							neighLast, 
							numPoints, 
							epsilon2fixed, 
							weight, 
							(int32_t)(inst->vFactor * inst->fixedPointScale),
							inst->scope, 
							&newCount);
                        // ZY: optimized function RADARDEMO_clusteringDBscan_findNeighbors3Fixed need to rework
                        //neighCount = RADARDEMO_clusteringDBscan_findNeighbors3Fixed((RADARDEMO_clusteringDBscanPoint2dfxdp *)input->pointArray, member, neighLast, numPoints, epsilon2fixed, inst->scope, &newCount);

                    if(neighCount >= inst->minPointsInCluster)
                    {
                        for (ind = 0; ind < newCount; ind ++) {
                            member = neighLast[ind];
                            inst->scope[member] = POINT_VISITED;
                        }
                        neighLast += newCount;              // Member is a core point, and its neighborhood is added to the cluster
                    }
                }
            	if (clusterId >= inst->maxClusters)
            	   return DBSCAN_ERROR_CLUSTER_LIMIT_REACHED;

                // calculate the clustering center and edge information
                if (input->inputPntPrecision == DBSCAN_INPUT_FLOATINGPT)
                    RADARDEMO_clusteringDBscan_calcInfo(
					    inst->fixedPointScale,
						(RADARDEMO_clusteringDBscanPoint2d *)input->pointArray.pointArrayFloat , 
						input->speed.speedFloat, 
						input->SNRArray,
						input->aoaVar,
						inst->neighbors, 
						neighLast, 
						(RADARDEMO_clusteringDBscanReport *)&output->report[clusterId - 1]);
                else
                    RADARDEMO_clusteringDBscan_calcInfoFixed(
						(RADARDEMO_clusteringDBscanPoint2dfxdp *)input->pointArray.pointArrayInt16, 
						input->speed.speedInt16, 
						input->SNRArray,
						input->aoaVar,
						inst->neighbors, 
						neighLast, 
						(RADARDEMO_clusteringDBscanReport *)&output->report[clusterId - 1]);
            }
        }
    } // for
    output->numCluster = clusterId;

    return DBSCAN_OK;
}

//! \copydoc RADARDEMO_clusteringDBscanDelete
int RADARDEMO_clusteringDBscanDelete(void *handle) {
    RADARDEMO_clusteringDBscanInstance *inst = (RADARDEMO_clusteringDBscanInstance *)handle;

	radarOsal_memFree(inst->scratchPad, inst->maxPoints *(sizeof(char) *2 + sizeof(uint16_t)));
#ifdef DBSCAN_DEBUG
    radarOsal_memFree(inst->debugInfo->clusterInfo, sizeof(RADARDEMO_clusteringDBscanClusterInfo));
    radarOsal_memFree(inst->debugInfo, sizeof(RADARDEMO_clusteringDBscanDebugInfo));
#endif
    radarOsal_memFree(handle, sizeof(RADARDEMO_clusteringDBscanInstance));
    return DBSCAN_OK;
}

