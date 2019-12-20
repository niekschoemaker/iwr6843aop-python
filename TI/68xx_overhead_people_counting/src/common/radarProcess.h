/*!
 *  \file   radarProcess.h
 *
 *  \brief   Header file for radar signal processing chain.
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
#ifndef _RADARPROCESS_H
#define _RADARPROCESS_H


#include <swpform.h>
#include <modules/clustering/dbscan/api/RADARDEMO_clusteringDBscan.h>
#include <modules/tracking/clusterTracker/api/RADARDEMO_clusterTracker.h>

#if defined(_WIN32) || defined(CCS)
#include <stdio.h>
#endif


/**
 *  \def  M_PI
 *
 *  \brief   constant value used for pi.
 *
 *  \sa
 */
#define  M_PI 3.14159265358979323846f

#define  NUM_RADAR_TXANT        2    // 2 transmitting antennas
#define  NUM_RADAR_RXANT        4    // 4 receiving antennas

#define  DOA_OUTPUT_MAXPOINTS   250
#define  MAX_RESOLVED_OBJECTS_PER_FRAME   DOA_OUTPUT_MAXPOINTS  //zz: src is defined in odmInterface.h
#define  MAX_NUM_RANGE_BINS (1024)

#define RADAR_DSP_BUFF_CONT_MAX_NUM_BUFF  2U

// Output data to Gtracker
typedef struct {
    int32_t object_count;                         //number of objects (points)

    float          range[DOA_OUTPUT_MAXPOINTS];
    float          angle[DOA_OUTPUT_MAXPOINTS];
    float          elev[DOA_OUTPUT_MAXPOINTS];
    float          doppler[DOA_OUTPUT_MAXPOINTS];
    float          snr[DOA_OUTPUT_MAXPOINTS];
//    float          rangeVar[DOA_OUTPUT_MAXPOINTS];
//    float          dopplerVar[DOA_OUTPUT_MAXPOINTS];
} radarProcessOutputToTracker;

typedef struct _radarProcessBenchmarkElem_
{
	uint32_t numDetPnts;
	uint32_t cfarDetectionCycles;
	uint32_t dopplerEstCycles;
} radarProcessBenchmarkElem;

typedef struct _radarProcessBenchmarkObj_
{
	uint32_t bufferLen;
	uint32_t bufferIdx;
	radarProcessBenchmarkElem *buffer;
} radarProcessBenchmarkObj;

typedef struct _radarModuleCfarConfig_
{
	uint16_t	refWinSize[2]; /**< reference window size in each side in two directions for clutter variance estimation. */
	uint16_t	guardWinSize[2]; /**< guard window size in each side in two directions. */
	uint16_t	cfarDiscardAngleLeft;  /**< Number of left cells discarded.*/
	uint16_t	cfarDiscardAngleRight;  /**< Number of left cells discarded.*/
	uint16_t	cfarDiscardRangeLeft;  /**< Number of left cells discarded.*/
	uint16_t	cfarDiscardRangeRight;  /**< Number of left cells discarded.*/
	uint16_t	cfarMethod; /**< CFAR_CA (2-pass) = 1; CFAR_OS = 2 (2-pass); CFAR_CASO = 3 (1-pass); CFAR_CASO = 4 (2-pass); */
	uint32_t    log2MagFlag; /**<use log2(mag) as input*/
	float		rangeThre; /**< threshold used for compare. */
	float       azimuthThre;  	        /**< Doppler search relative threshold.*/
	float		rangeRes; /**< range resolution in meters. */
	float		velocityRes; /**< velocity resolution in meters/sec. */
} radarModuleCfarConfig;

typedef struct _radarModuleDoaConfig_
{
    float        estAngleResolution;             /**< Estimation resolution in degree.*/
    float        estAngleRange;                  /**< Range of the estimation, from -estRange to +estRange degrees*/
    float         gamma;						 /**< Diagnol loading scaling factor*/
	uint8_t      clutterRemovalFlag;			 /**< flag to indicate clutter removal needed. */
} radarModuleDoaConfig;

//user input configuration parameters
typedef struct _radarModuleConfig_
{
	//rangeFFT/Doppler parameters
	uint16_t	framePeriod; /**< Frame period in msec. */
	uint16_t	numAdcSamplePerChirp; /**< number of adc samples per chirp. */
	uint16_t	numAdcBitsPerSample; /**< number of adc bits per sample. */
	uint16_t	numChirpPerFrame; /**< number of chirps per frame. */
	uint16_t	numTxAntenna; /**< number of antennas. */
	uint16_t	numAntenna; /**< number of virtual antennas. */
	uint16_t	numPhyRxAntenna; /**< number of physical RX antennas. */
	uint16_t	rangeWinSize; /**< range window size. */
	uint16_t    mimoModeFlag;    /**<Flag for MIMO mode: 0 -- SIMO, 1 -- TDM MIMO, 2 -- FDM or BF*/
	uint32_t    numTotalChirpProfile;    /**<number of chirp profiles*/
	uint32_t    numUniqueChirpProfile;    /**<number of unique chirp profiles*/
	float		rangeWindow[16]; /**< pointer to pre-calcuted window coeff for range FFT. */
	float       chirpInterval;

	//detection parameters
	radarModuleCfarConfig cfarConfig;

	radarModuleDoaConfig doaConfig;

	uint16_t numAzimuthBin;
	uint16_t	fftSize1D;
	uint16_t	fftSize2D;
	cplx16_t    * pFFT1DBuffer;
	uint8_t		maxNumDetObj;
	uint32_t    heatMapMemSize;
	float       *heatMapMem;
	radarProcessBenchmarkObj * benchmarkPtr;
}radarProcessConfig_t;

typedef enum
{
	PROCESS_OK = 0,
	PROCESS_ERROR_INIT_MEMALLOC_FAILED,
	PROCESS_ERROR_RANGEPROC_INIT_FAILED,
	PROCESS_ERROR_RANGEPROC_INOUTALLOC_FAILED,
	PROCESS_ERROR_CFARPROC_NONSUPPORTEDMETHOD,
	PROCESS_ERROR_CFARPROC_INIT_FAILED,
	PROCESS_ERROR_CFARPROC_INOUTALLOC_FAILED,
	PROCESS_ERROR_CFARPROC_NDET_EXCEEDLIMIT,
	PROCESS_ERROR_DOAPROC_NONSUPPORTEDMETHOD,
	PROCESS_ERROR_DOAPROC_INIT_FAILED,
	PROCESS_ERROR_DOAPROC_INOUTALLOC_FAILED,
	PROCESS_ERROR_NOT_SUPPORTED
} ProcessErrorCodes;


//! \brief      initializes data processing task
//!
void *radarProcessCreate(radarProcessConfig_t  * config, ProcessErrorCodes * procErrorCode);

/**
 *  @brief      Perform range processing. Processing is done per antenna per chirp.
 *
 *  @param[in]  handle               Pointer to instance handler
 *  @param[in]  pDataIn              Pointer to the input Data
 *  @param[out] pDataOut             Pointer to the output Data
 *
 *  @remarks
 */

extern uint8_t radarRangeProcessRun(void *handle, cplx16_t * pDataIn, cplx16_t * pDataOut);

/**
 *  @brief      Perform Doppler processing. Processing is done per all antennas per range bin.
 *
 *  @param[in]  handle               Pointer to instance handler
 *  @param[in]  frameNum             Frame number
 *  @param[in]  rangeIdx             Input range index.
 *  @param[in]  pDataIn              Pointer to the input data (1D FFT output) for all antennas.
 *  @param[out] pDataOut             Pointer to the output integrated signal.
 *
 *  @remarks
 */

extern uint8_t radarRangeAzimuthProcessRun(void *handle, uint32_t frameNum, int32_t rangeIdx, cplx16_t * pDataIn, float *pDataOut);

/**
 *  @brief      Perform frame processing.
 *
 *  @param[in]  handle               Pointer to instance handler
 *  @param[out] outBuffer             Pointer to the output X-Y Data
 *
 *  @remarks
 */

extern uint8_t radarFrameProcessRun(void *handle, void * outBuffer);

//! \brief      release process instance
//!
int32_t processDelete(void *handle);

#endif  // _PROCESS_H

/* Nothing past this point */
