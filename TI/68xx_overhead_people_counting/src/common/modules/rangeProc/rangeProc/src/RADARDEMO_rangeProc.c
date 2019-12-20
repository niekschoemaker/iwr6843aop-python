/*! 
 *  \file   RADARDEMO_rangeProc.c
 *
 *  \brief   Range processing. 
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

#include <modules/rangeProc/rangeProc/api/RADARDEMO_rangeProc.h>
#include "RADARDEMO_rangeProc_priv.h"
#include <math.h>
#include <stdio.h>

#define DEBUG(_x) //_x

#ifdef _TMS320C6X
#include "c6x.h"
#endif


static unsigned char brev[64] = {
    0x0, 0x20, 0x10, 0x30, 0x8, 0x28, 0x18, 0x38,
    0x4, 0x24, 0x14, 0x34, 0xc, 0x2c, 0x1c, 0x3c,
    0x2, 0x22, 0x12, 0x32, 0xa, 0x2a, 0x1a, 0x3a,
    0x6, 0x26, 0x16, 0x36, 0xe, 0x2e, 0x1e, 0x3e,
    0x1, 0x21, 0x11, 0x31, 0x9, 0x29, 0x19, 0x39,
    0x5, 0x25, 0x15, 0x35, 0xd, 0x2d, 0x1d, 0x3d,
    0x3, 0x23, 0x13, 0x33, 0xb, 0x2b, 0x1b, 0x3b,
    0x7, 0x27, 0x17, 0x37, 0xf, 0x2f, 0x1f, 0x3f
};

/*! 
   \fn     RADARDEMO_rangeProc_create
 
   \brief   Create and initialize RADARDEMO_rangeProc module. 
  
   \param[in]    moduleConfig
               Pointer to input configurations structure for RADARDEMO_rangeProc module.
			   
   \param[in]    errorCode
               Pointer to error code.
			   
   \ret     void pointer to the module handle. Return value of NULL indicates failed module creation.
			   
   \pre       none
 
   \post      none
  
 
 */

void	* RADARDEMO_rangeProc_create(
                            IN  RADARDEMO_rangeProc_config * moduleConfig, 
							OUT RADARDEMO_rangeProc_errorCode * errorCode)
							
{
	int32_t     i;
	RADARDEMO_rangeProc_handle * handle;
	
	*errorCode		=	RADARDEMO_RANGEPROC_NO_ERROR;

	/* Check error configurations */
	/* unsupported FFT type */
	if (moduleConfig->fft1DType >= RADARDEMO_RANGEPROC_1DFFT_NOT_SUPPORTED)
		*errorCode	=	RADARDEMO_RANGEPROC_1DFFTTYPE_NOTSUPPORTED;
		
	/* wrong FFT size */
	if (moduleConfig->fft1DSize < moduleConfig->nSamplesPerChirp)
		*errorCode	=	RADARDEMO_RANGEPROC_1DFFTSIZE_NOTVALID;
	i = 30 - _norm(moduleConfig->fft1DSize);
	if ( (1 << i) != moduleConfig->fft1DSize)
		*errorCode	=	RADARDEMO_RANGEPROC_1DFFTSIZE_NOTVALID;

	/* wrong number of ADC bits */
	if (moduleConfig->adcNumOutputBits > RADARDEMO_RANGEPROC_MAXADCNBITS)
		*errorCode	=	RADARDEMO_RANGEPROC_ADCNBITS_OUTOFRANGE;
		
	/* wrong number of ADC bits */
	if (moduleConfig->adjustDCforInput >= RADARDEMO_RANGEPROC_ADCADJUSTTYPE_NOT_SUPPORTED)
		*errorCode	=	RADARDEMO_RANGEPROC_UNSUPPORTEDADCREFORMAT;
		
	/* output type does not match */
	if (moduleConfig->fft1DType == RADARDEMO_RANGEPROC_1DFFT_16x16)
	{
		if ((moduleConfig->include2DwinFlag == 0) && (moduleConfig->outputFixedpFlag == 0))
			*errorCode	=	RADARDEMO_RANGEPROC_OUTPUTANDFFTTYPE_MISMATCH;
	}
	if (moduleConfig->fft1DType == RADARDEMO_RANGEPROC_1DFFT_SPxSP)
	{
		if (moduleConfig->outputFixedpFlag == 1)
			*errorCode	=	RADARDEMO_RANGEPROC_OUTPUTANDFFTTYPE_MISMATCH;
	}

	handle						=	(RADARDEMO_rangeProc_handle *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, sizeof(RADARDEMO_rangeProc_handle), 1);
	if (handle == NULL)
	{
		*errorCode	=	RADARDEMO_RANGEPROC_FAIL_ALLOCATE_HANDLE;
		return (handle);
	}
	
	handle->nRxAnt				=	moduleConfig->nRxAnt;
	handle->nSamplesPerChirp	=	moduleConfig->nSamplesPerChirp;
	handle->fft1DSize			=	moduleConfig->fft1DSize;
	handle->numChirpsPerFrame	=	moduleConfig->numChirpsPerFrame;
	handle->adcNumOutputBits	=	moduleConfig->adcNumOutputBits;
	handle->fft1DType			=	(uint8_t) moduleConfig->fft1DType;
	handle->include2DwinFlag	=	moduleConfig->include2DwinFlag;
	handle->outputFixedpFlag	=	moduleConfig->outputFixedpFlag;
	handle->win1DLength			=	moduleConfig->win1DLength;
	handle->adjustDCforInput	=	(uint8_t) moduleConfig->adjustDCforInput;

	handle->win1D				=	(int16_t *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, handle->win1DLength * sizeof(int16_t), 1);
	handle->win2D				=	NULL;
	if (moduleConfig->include2DwinFlag)
		handle->win2D				=	(int16_t *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, (moduleConfig->numChirpsPerFrame >> 1) * sizeof(int16_t), 1);

	if (handle->win1D == NULL)
	{
		*errorCode	=	RADARDEMO_RANGEPROC_FAIL_ALLOCATE_LOCALINSTMEM;
		return (handle);
	}
	if ((handle->win2D == NULL) && (moduleConfig->include2DwinFlag))
	{
		*errorCode	=	RADARDEMO_RANGEPROC_FAIL_ALLOCATE_LOCALINSTMEM;
		return (handle);
	}

	for (i = 0; i < (int32_t) handle->win1DLength; i++ )
	{
		handle->win1D[i]		=	moduleConfig->win1D[i];
	}
	if (moduleConfig->include2DwinFlag)
	{
		for (i = 0; i < (int32_t) (moduleConfig->numChirpsPerFrame >> 1); i++ )
		{
			handle->win2D[i]		=	moduleConfig->win2D[i];
		}
	}
	if (moduleConfig->fft1DType == RADARDEMO_RANGEPROC_1DFFT_16x16)
	{
		int16_t *tempTPtr;
		tempTPtr				=	(int16_t *)radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, 2 * (handle->fft1DSize + DSPLIBFFT16X16MEMPAD)* sizeof(int16_t), 8);
		handle->twiddle			=	(void *)&tempTPtr[2 * DSPLIBFFT16X16MEMPAD];
		tempTPtr				=	(int16_t *) handle->twiddle;
		if (handle->twiddle == NULL)
		{
			*errorCode	=	RADARDEMO_RANGEPROC_FAIL_ALLOCATE_LOCALINSTMEM;
			return (handle);
		}
		gen_twiddle_fft16x16_imre(tempTPtr, handle->fft1DSize);
		if(handle->adjustDCforInput != RADARDEMO_RANGEPROC_ADC_NO_ADJUST)
		{
			tempTPtr				=	(int16_t *)radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 1, 2 * (handle->fft1DSize + DSPLIBFFT16X16MEMPAD)* sizeof(int16_t), 8);
			if (tempTPtr == NULL)
			{
				handle->scratchPad		=	NULL;
				*errorCode	=	RADARDEMO_RANGEPROC_FAIL_ALLOCATE_LOCALINSTMEM;
				return (handle);
			}
			handle->scratchPad		=	(void *)&tempTPtr[2 * DSPLIBFFT16X16MEMPAD];
		}
		else
		{
			handle->scratchPad		=	NULL;
		}
	}
	else if (moduleConfig->fft1DType == RADARDEMO_RANGEPROC_1DFFT_SPxSP)
	{
		float * tempTPtr;
		handle->twiddle			=	(void *)radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, 2 * handle->fft1DSize * sizeof(float), 8);
		if (handle->twiddle == NULL)
		{
			*errorCode	=	RADARDEMO_RANGEPROC_FAIL_ALLOCATE_LOCALINSTMEM;
			return (handle);
		}
		tempTPtr				=	(float *) handle->twiddle;
		tw_gen_float(tempTPtr, handle->fft1DSize);
		handle->scratchPad		=	(void *)radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 1, 2 * handle->fft1DSize * sizeof(float), 8);
		if (handle->scratchPad == NULL)
		{
			*errorCode	=	RADARDEMO_RANGEPROC_FAIL_ALLOCATE_LOCALINSTMEM;
			return (handle);
		}
	}
	
	return((void *)handle);
}

/*! 
   \fn     RADARDEMO_rangeProc_delete
 
   \brief   Delete RADARDEMO_rangeProc module. 
  
   \param[in]    handle
               Module handle.
			   
   \pre       none
 
   \post      none
  
 
 */

void	RADARDEMO_rangeProc_delete(
                            IN  void * handle)
{
	RADARDEMO_rangeProc_handle *rangeProcInst;
	
	rangeProcInst	=	(RADARDEMO_rangeProc_handle *) handle;
	
	if ((RADARDEMO_rangeProc_1DFFTType) rangeProcInst->fft1DType == RADARDEMO_RANGEPROC_1DFFT_16x16)
	{
		int16_t *tempTPtr;

		tempTPtr	=	(int16_t *)rangeProcInst->twiddle;
		radarOsal_memFree(&tempTPtr[-2 * DSPLIBFFT16X16MEMPAD], rangeProcInst->fft1DSize * 2 * sizeof(int16_t));
		if(rangeProcInst->adjustDCforInput != RADARDEMO_RANGEPROC_ADC_NO_ADJUST)
		{
			tempTPtr	=	(int16_t *)rangeProcInst->scratchPad;
			radarOsal_memFree(&tempTPtr[-2 * DSPLIBFFT16X16MEMPAD], rangeProcInst->fft1DSize * 2 * sizeof(int16_t));
		}
	}	
	else if ((RADARDEMO_rangeProc_1DFFTType)rangeProcInst->fft1DType == RADARDEMO_RANGEPROC_1DFFT_16x16)
	{
		radarOsal_memFree(rangeProcInst->twiddle, rangeProcInst->fft1DSize * 2 * sizeof(float));
		radarOsal_memFree(rangeProcInst->scratchPad, rangeProcInst->fft1DSize * 2 * sizeof(float));
	}	
	radarOsal_memFree(rangeProcInst->win1D, rangeProcInst->win1DLength * sizeof(int16_t));
	if (rangeProcInst->win2D != NULL )
		radarOsal_memFree(rangeProcInst->win2D, ((rangeProcInst->numChirpsPerFrame >> 1) + 1) * sizeof(int16_t));

	radarOsal_memFree(handle, sizeof(RADARDEMO_rangeProc_handle));
}



/*! 
   \fn     RADARDEMO_rangeProc_run
 
   \brief   Range processing, always called per chirp per antenna.
  
   \param[in]    handle
               Module handle.
 
   \param[in]    rangeProcInput
               Input signal from ADC and corresponding chirp number.
 
   \param[out]    outputSignal
               Output signal from range processing. if handle->outputFixedpFlag = 1, output is cplx16_t type with size handle->fft1DSize.
			   if handle->outputFixedpFlag = 0, output is float type with size 2 * handle->fft1DSize and stored in real imag order in memory.
 
   \return    errorCode
               Error code.
			   
	\pre       none
 
   \post      none
  
 
 */

RADARDEMO_rangeProc_errorCode	RADARDEMO_rangeProc_run(
                            IN  void * handle,
							IN  RADARDEMO_rangeProc_input * rangeProcInput,
							OUT void   * outputSignal)

{
	int32_t		j, chirpNum;
	RADARDEMO_rangeProc_handle *rangeProcInst;
	cplx16_t    * inputSignal;
	RADARDEMO_rangeProc_errorCode errorCode = RADARDEMO_RANGEPROC_NO_ERROR;

	inputSignal		=	rangeProcInput->inputSignal;
	rangeProcInst	=	(RADARDEMO_rangeProc_handle *) handle;

#ifndef _WIN32
	if (((uint32_t) outputSignal & 0x7) != 0)
		errorCode	=	RADARDEMO_RANGEPROC_INOUTPTR_NOTCORRECT;
	if (((uint32_t) inputSignal & 0x7) != 0)
		errorCode	=	RADARDEMO_RANGEPROC_INOUTPTR_NOTCORRECT;
#endif

	if (outputSignal == NULL)
		errorCode	=	RADARDEMO_RANGEPROC_INOUTPTR_NOTCORRECT;
	if (inputSignal == NULL)
		errorCode	=	RADARDEMO_RANGEPROC_INOUTPTR_NOTCORRECT;

	if(((RADARDEMO_rangeProc_1DFFTType)rangeProcInst->fft1DType == RADARDEMO_RANGEPROC_1DFFT_16x16) && (rangeProcInst->adjustDCforInput != RADARDEMO_RANGEPROC_ADC_NO_ADJUST))
	{
		if (rangeProcInst->scratchPad == NULL)
			errorCode	=	RADARDEMO_RANGEPROC_INOUTPTR_NOTCORRECT;
	}
	if (rangeProcInst->win1D == NULL)
		errorCode	=	RADARDEMO_RANGEPROC_INOUTPTR_NOTCORRECT;

	if (errorCode > RADARDEMO_RANGEPROC_NO_ERROR)
		return(errorCode);
	
	chirpNum		=	(int32_t) rangeProcInput->chirpNumber;
	if (chirpNum >= (int32_t)rangeProcInst->numChirpsPerFrame >> 1)
		chirpNum	=	(int32_t)rangeProcInst->numChirpsPerFrame - chirpNum - 1;


	if ((RADARDEMO_rangeProc_1DFFTType) rangeProcInst->fft1DType == RADARDEMO_RANGEPROC_1DFFT_16x16)
	{
		if (rangeProcInst->adjustDCforInput != RADARDEMO_RANGEPROC_ADC_NO_ADJUST)
		{
			cplx16_t * win1DOutputPtr;
			cplx16_t * fft1DOutputPtr;

			win1DOutputPtr		=	(cplx16_t *) rangeProcInst->scratchPad;
			RADARDEMO_rangeProcWin1D1AntFxdp(
								rangeProcInst->nSamplesPerChirp,
								rangeProcInst->fft1DSize,
								rangeProcInst->win1D,
								rangeProcInst->adcNumOutputBits,
								inputSignal,
								rangeProcInst->adjustDCforInput,
								win1DOutputPtr);

			fft1DOutputPtr 	= 	(cplx16_t *)outputSignal;
			DSP_fft16x16_imre(
						(short *) rangeProcInst->twiddle,
						rangeProcInst->fft1DSize,
						(short *) win1DOutputPtr,
						(short *) fft1DOutputPtr);

			if (rangeProcInst->include2DwinFlag == 1)
			{
				if (rangeProcInst->outputFixedpFlag == 1)
				{
					RADARDEMO_rangeProcWin2D1AntFxdp(
								rangeProcInst->fft1DSize,
								rangeProcInst->win2D[chirpNum],
								fft1DOutputPtr);
				}
				else
				{
					RADARDEMO_rangeProcWin2D1AntFxdpinFltOut(
								rangeProcInst->fft1DSize,
								rangeProcInst->win2D[chirpNum],
								(void *)fft1DOutputPtr);
				}
			}
		}
		else
		{
			RADARDEMO_rangeProcWin1D1AntFxdpPartial(
								rangeProcInst->nSamplesPerChirp,
								rangeProcInst->win1D,
								rangeProcInst->win1DLength,
								inputSignal);
			for (j = (int32_t)rangeProcInst->nSamplesPerChirp; j < (int32_t)rangeProcInst->fft1DSize; j++)
			{
				_amem4(&inputSignal[j])		=	0;
			}
			DSP_fft16x16_imre(
						(short *) rangeProcInst->twiddle,
						rangeProcInst->fft1DSize,
						(short *) inputSignal,
						(short *) outputSignal);

		}
	}	
	else if ((RADARDEMO_rangeProc_1DFFTType)rangeProcInst->fft1DType == RADARDEMO_RANGEPROC_1DFFT_SPxSP)
	{
		float * win1DOutputPtr;
		float * fft1DOutputPtr;
		int32_t rad1D;
		//unsigned char *brev; /* dummy pointer, not used inside DSPF_sp_fftSPxSP */

		win1DOutputPtr		=	(float *) rangeProcInst->scratchPad;
		
		RADARDEMO_rangeProcWin1D1AntFltp(
                            rangeProcInst->nSamplesPerChirp,
                            rangeProcInst->fft1DSize,
							rangeProcInst->adcNumOutputBits,
                            rangeProcInst->win1D,
							inputSignal,
							rangeProcInst->adjustDCforInput,
							win1DOutputPtr);		
							
		fft1DOutputPtr 	= 	(float *)outputSignal;
		
		j  = 30 - _norm(rangeProcInst->fft1DSize);
		if ((j & 1) == 0)
			rad1D = 4;
		else
			rad1D = 2;


		DSPF_sp_fftSPxSP (
				rangeProcInst->fft1DSize,
				win1DOutputPtr,
				(float *)rangeProcInst->twiddle, 
				fft1DOutputPtr,
				brev,
				rad1D,
				0,
				rangeProcInst->fft1DSize);
				
		if (rangeProcInst->include2DwinFlag == 1)
		{
			float ftemp;
			ftemp	=	(float) rangeProcInst->win2D[chirpNum];
			ftemp	=	ftemp/32768.f;
			RADARDEMO_rangeProcWin2D1Antfltp(
                rangeProcInst->fft1DSize,
				ftemp,
				fft1DOutputPtr);
		}				
	}	
	return(errorCode);
}

