/*!
 *   \file   RADARDEMO_clusteringDBscan_priv.h
 *
 *   \brief   Private functions for DBscan clustering module.
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

#ifndef _RADARDEMO_DBSCAN_PRIV_H_
#define _RADARDEMO_DBSCAN_PRIV_H_

#include <math.h>
#include <swpform.h>
#include <modules/clustering/dbscan/api/RADARDEMO_clusteringDBscan.h>

#ifndef _TMS320C6600
#include <modules/utilities/radar_c674x.h>
#endif

#define DBSCAN_FIXEDWEIGHTSHIFT (3)  //ASSUMPTION: weight is smaller than 8!!!!
#define DBSCAN_PIOVER180 (3.141592653589793/180.0)      //!< define the pi/180

typedef struct
{
    float x;
    float y;
} RADARDEMO_clusteringDBscanPoint2d;

typedef struct
{
    int16_t x;
    int16_t y;
} RADARDEMO_clusteringDBscanPoint2dfxdp;

typedef struct
{
    float x;
    float y;
    float z;
    float spped;
} RADARDEMO_clusteringDBscanPoint3d;

typedef struct
{
    uint16_t numPoints[10];
} RADARDEMO_clusteringDBscanClusterInfo;

typedef struct
{
    RADARDEMO_clusteringDBscanClusterInfo *clusterInfo;
    unsigned int memoryUsed;
    uint64_t benchPoint[10];
} RADARDEMO_clusteringDBscanDebugInfo;


typedef struct
{
    float       epsilon;
    float       weight;
	float       vFactor;
    uint16_t    minPointsInCluster;
    uint16_t    maxClusters;
    uint16_t    pointIndex;

    uint16_t    maxPoints;  // Maximum number of points that can be services per dbscanRun call

	char        *scratchPad;
    char        *visited;
    char        *scope;
    uint16_t    *neighbors;
    float       *distances;
    RADARDEMO_clusteringDBscanDebugInfo *debugInfo;

    uint8_t     inputPntPrecision;    /**< Precision of the input point: 0 -- single-preision floating point; 1 -- 16-bit fixed-point */
    uint16_t     fixedPointScale;     /**< Block scale value to convert x-y from floating-point to fixed-point. 
										  Should be the same the one used in converting r-theta to x-y, not used for floating-point input */

} RADARDEMO_clusteringDBscanInstance;

typedef enum
{
    POINT_UNKNOWN = 0,
    POINT_VISITED
} RADARDEMO_clusteringDBscanPointState;

//To add comments!!!
extern uint16_t RADARDEMO_clusteringDBscan_findNeighbors2(
                            IN RADARDEMO_clusteringDBscanPoint2d *pointArray,
							IN float *speedArray,
                            IN uint16_t point,
                            IN uint16_t *neigh,
                            IN uint16_t numPoints,
                            IN float epsilon2,
                            IN float weight,
                            IN float vFactor,
                            IN char *visited,
                            OUT uint16_t *newCount);

extern uint16_t RADARDEMO_clusteringDBscan_findNeighbors2Fixed(
                            IN RADARDEMO_clusteringDBscanPoint2dfxdp *RESTRICT pointArray,
							IN int16_t *RESTRICT speedArray,
                            IN uint16_t point,
                            IN uint16_t *RESTRICT neigh,
                            IN uint16_t numPoints,
                            IN int32_t epsilon2,
                            IN float weight,
                            IN int32_t vFactor,
                            IN char *RESTRICT visited,
                            OUT uint16_t *newCount);

extern void RADARDEMO_clusteringDBscan_calcInfo(
							IN uint16_t scale, 
                            IN RADARDEMO_clusteringDBscanPoint2d *pointArray,
							IN float *speedArray,
							IN float  *SNRArray,
							IN float  *aoaVar,
                            IN uint16_t *neighStart,
                            IN uint16_t *neighLast,
                            OUT RADARDEMO_clusteringDBscanReport *report);

extern void RADARDEMO_clusteringDBscan_calcInfoFixed(
                            IN RADARDEMO_clusteringDBscanPoint2dfxdp *pointArray,
							IN int16_t *speedArray,
							IN float  *SNRArray,
							IN float  *aoaVar,
                            IN uint16_t *neighStart,
                            IN uint16_t *neighLast,
                            OUT RADARDEMO_clusteringDBscanReport *report);


#endif //_RADARDEMO_DBSCAN_PRIV_H_

