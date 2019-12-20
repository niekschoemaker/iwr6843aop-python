/*!
 *  \file   RADARDEMO_clusteringDBscan.h
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

#include <swpform.h>
#include <modules/utilities/radarOsal_malloc.h>

#ifndef _RADARDEMO_DBSCAN_H_
#define _RADARDEMO_DBSCAN_H_

typedef void* dbscanHandle;

//#define DBSCAN_DEBUG


#define DBSCAN_ERROR_CODE_OFFSET 100

//!  \brief   error code for RADARDEMO_clusteringDBscan.
//!
typedef enum
{
    DBSCAN_INPUT_FLOATINGPT = 0,			/**< Input format SP floating-point */
    DBSCAN_INPUT_16BITFIXEDPT,					/**< Input format 16-bit fixed-point */
    DBSCAN_INPUTFORMAT_NOT_SUPPORTED                                     /**< To be added */
} RADARDEMO_clusteringDBscanInputFormat;



//!  \brief   Structure element of the list of descriptors for RADARDEMO_clusteringDBscan configuration.
//!
typedef struct
{
    float       epsilon;              /**< distance threhold for cluster check*/
    float       weight;               /**< the weight between the distance and speed */
    float       vFactor;               /**< additional velocity factor for speed delta  */
    uint16_t    minPointsInCluster;   /**< minimum number of points in a cluster*/
    uint16_t    maxClusters;          /**< maximum number of clusters*/

    uint16_t    maxPoints;            /**< Maximum number of points that can be services per dbscanRun call */
    RADARDEMO_clusteringDBscanInputFormat     inputPntPrecision;    /**< Precision of the input point: 0 -- single-preision floating point; 1 -- 16-bit fixed-point */
    uint16_t     fixedPointScale;     /**< Block scale value to convert x-y from floating-point to fixed-point. 
										  Should be the same the one used in converting r-theta to x-y, not used for floating-point input */
} RADARDEMO_clusteringDBscanConfig;

//!  \brief   error code for RADARDEMO_clusteringDBscan.
//!
typedef enum
{
    DBSCAN_OK = 0,                                                  /**< To be added */
    DBSCAN_ERROR_MEMORY_ALLOC_FAILED = DBSCAN_ERROR_CODE_OFFSET,    /**< To be added */
    DBSCAN_ERROR_NOT_SUPPORTED,                                     /**< To be added */
    DBSCAN_ERROR_CLUSTER_LIMIT_REACHED                              /**< To be added */
} RADARDEMO_clusteringDBscanErrorCodes;


//!  \brief   Structure element of the list of descriptors for RADARDEMO_clusteringDBscan input.
//!
typedef struct
{
    uint16_t numPoints;                      /**< Number of point for clustering */
    RADARDEMO_clusteringDBscanInputFormat  inputPntPrecision;            /**< Flag to indicate input point array and speed array in 16-bit fixed-point format */
	union {
	  int16_t * pointArrayInt16;
	  float * pointArrayFloat;
    } pointArray;                      /**< gr location info for each detected point, points are store in x0, y0, x1, y1 fashion */
    union {
	  int16_t * speedInt16;
	  float * speedFloat;
    } speed;							/**< gr speed info for each detected point */
    float *SNRArray;
    float *aoaVar;
} RADARDEMO_clusteringDBscanInput;

//!  \brief   Structure for each cluster information report .
//!
typedef struct
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
} RADARDEMO_clusteringDBscanReport;

//!  \brief   Structure of clustering output.
//!
typedef struct
{
    uint16_t *IndexArray;                       /**< Clustering result index array */
    //uint64_t *benchMarkArray;                   /**< benchMark information */
    uint16_t numCluster;                        /**< number of cluster detected */
    RADARDEMO_clusteringDBscanReport *report;   /**< information report for each cluster*/
} RADARDEMO_clusteringDBscanOutput;

/*!
   \fn     RADARDEMO_clusteringDBscanCreate

   \brief   Create and initialize RADARDEMO_clusteringDBscan module.

   \param[in]    param
               Pointer to input configurations structure for RADARDEMO_clusteringDBscan module.

   \ret     void pointer to the module handle. Return value of NULL indicates failed module creation.

   \pre       none

   \post      none


 */
extern void * RADARDEMO_clusteringDBscanCreate( RADARDEMO_clusteringDBscanConfig *param);


/*!
   \fn     RADARDEMO_clusteringDBscanRun

   \brief   Run RADARDEMO_clusteringDBscan module.

   \param[in]    handle
               Pointer to input handle for RADARDEMO_clusteringDBscan module.

   \param[in]    input
               Pointer to input data to RADARDEMO_clusteringDBscan module.

   \param[out]    output
               Pointer to output data from RADARDEMO_clusteringDBscan module.

   \ret     Error code.

   \pre       none

   \post      none


 */
 extern int32_t RADARDEMO_clusteringDBscanRun( void *handle,  RADARDEMO_clusteringDBscanInput *input, RADARDEMO_clusteringDBscanOutput *output);

/*!
   \fn     RADARDEMO_clusteringDBscanDelete

   \brief   Delete fucntion for RADARDEMO_clusteringDBscan module.

   \param[in]    handle
               Pointer to input handle for RADARDEMO_clusteringDBscan module.

   \ret     Error code.

   \pre       none

   \post      none


 */
extern int32_t RADARDEMO_clusteringDBscanDelete( void *handle);

#endif //_RADARDEMO_DBSCAN_H_
