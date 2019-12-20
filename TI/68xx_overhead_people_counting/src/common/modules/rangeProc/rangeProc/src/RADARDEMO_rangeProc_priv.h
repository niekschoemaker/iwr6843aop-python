/*! 
 *  \file   RADARDEMO_rangeProc_priv.h
 *
 *  \brief   Header file for RADARDEMO_rangeProc module's internal functions
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

#ifndef RADARDEMO_RANGEPROC_RPIV_H
#define RADARDEMO_RANGEPROC_RPIV_H

#include <swpform.h>
#ifndef _WIN32
#ifndef _TMS320C6600

#ifdef CCS
//#include "dsplib674x.h"
//#include "dsplib.h"
#include <ti/dsplib/src/DSP_fft16x16_imre/c64P/DSP_fft16x16_imre.h>
#include <src/DSPF_sp_fftSPxSP/DSPF_sp_fftSPxSP.h>
#else
//#include <ti/dsplib/src/DSP_fft16x16_imre/c64P/DSP_fft16x16_imre.h>
//#include <ti/dsplib/src/DSPF_sp_fftSPxSP/DSPF_sp_fftSPxSP.h>
#include <DSP_fft16x16_imre.h>
#include <DSPF_sp_fftSPxSP.h>
#endif
#else

#include <ti/dsplib/src/DSP_fft16x16_imre/c66/DSP_fft16x16_imre.h>
#include <ti/dsplib/src/DSPF_sp_fftSPxSP/c66/DSPF_sp_fftSPxSP.h>
#endif
#endif

#ifndef _TMS320C6600
#include <modules/utilities/radar_c674x.h>
#endif
#define WIN1DLEFTSHFTGAP (1) 

/** 
 *  \struct   _RADARDEMO_rangeProc_handle_
 *   {
 *	void *        twiddle;          
 *	void *        scratchPad;       
 *	uint32_t     fft1DSize;  		
 *	uint32_t     nSamplesPerChirp;	
 *  uint32_t     numChirpsPerFrame;
 *	uint8_t      nRxAnt;  	       	
 *	uint8_t      adcNumOutputBits;  
 *   uint8_t 	 fft1DType;  		
 *   uint8_t      include2DwinFlag;  
 *   uint8_t      outputFixedpFlag;  
 *   int16_t      * win1D;  	   
 *   int16_t      * win2D;  	 
 *   int16_t      win1DLength;
 *   uint8_t      adjustDCforInput;
 *   }   _RADARDEMO_rangeProc_handle_;
 *
 *  \brief   Structure element of the list of descriptors for UL allocations.
 *
 *
 */

typedef struct _RADARDEMO_rangeProc_handle_
{
	void *        twiddle;                  	/**< Twiddle factors for FFT.*/
	void *        scratchPad;                  	/**< Scratch memory to hold intermediate results, of size fft1DSize * 2 * sizeof (fft input type, float or int16)*/
	uint32_t     fft1DSize;  					/**< 1D FFT size*/
	uint32_t     nSamplesPerChirp;				/**< number of samples per chirp*/
	uint32_t     numChirpsPerFrame;				/**< number of chirp per frame*/
	uint8_t      nRxAnt;  	       				/**< number of receive antennas.*/
	uint8_t      adcNumOutputBits;  	        /**< number of ADC output bits.*/
	uint8_t 	 fft1DType;  					/**< Type of 1D FFT.*/
	uint8_t      include2DwinFlag;  	    	/**< Flag indicating 2D windowing is done with range processing when set to 1. Otherwise, 2D windowing is done in Doppler processing*/
	uint8_t      outputFixedpFlag;  	    	/**< Flag indicating range processing output is 16x16 fixed point when set to 1. Otherwise, output single precision floating point.*/
	int16_t      * win1D;  	    	            /**< pointer to 1D windowing function.*/
	int16_t      win1DLength;  	    	        /**< half length of the 1D windowing function.*/
	int16_t      * win2D;  	    	            /**< pointer to 2D windowing function.*/
	uint8_t      adjustDCforInput;  	    /**< Flag for ADC sample adjustment, see definition of RADARDEMO_rangeProc_ADCAdjustType*/
} RADARDEMO_rangeProc_handle;

/*!
   \fn     RADARDEMO_rangeProcWin1D1AntFxdp

   \brief   Windowing function for 16x16 1D FFT and prepare for the input.

   \param[in]    nSamplesValidPerChirp
               Number of valid samples per chirp.

   \param[in]    fftSize1D
               1D FFT size.

   \param[in]    fftWin1D
               Input pointer to 1D window function.
			   
   \param[in]    ADCnob
               Number of ADC bits.

   \param[in]    inputPtr
               Pointer to input signal.

   \param[in]    adjustDCforInput
               Flag to enable DC adjustment for ADC samples when set to 1. Otherwise, no preprocessing for ADC samples.

   \param[out]    outputPtr
               Pointer to the output signal.

   \pre       none

   \post      none


 */

extern void	RADARDEMO_rangeProcWin1D1AntFxdp(
                            IN  uint32_t nSamplesValidPerChirp,
                            IN  uint32_t fftSize1D,
                            IN  int16_t  * fftWin1D,
							IN 	uint8_t  ADCnob,
							IN  cplx16_t * inputPtr,
							IN  uint8_t  adjustDCforInput,
							OUT cplx16_t * outputPtr);
							

/*!
   \fn     RADARDEMO_windowing2DPartial1Antfltp

   \brief   Windowing function for 2D FFT and prepare for the input.

   \param[in]    fftSize1D
               1D FFT size.

   \param[in]    inWin
               Input 2D windowing coefficient for this chirp.

   \param[out]    outputPtr
               Pointer to the output signal.

   \pre       none

   \post      none


 */

extern void	RADARDEMO_windowing2DPartial1Antfltp(
                            IN  uint32_t fftSize1D,
							IN  float inWin,
							OUT float * outputPtr);
							
/*!
   \fn     RADARDEMO_rangeProcWin2D1AntFxdp

   \brief   Windowing function for 2D FFT and prepare for the input.

   \param[in]    fftSize1D
               1D FFT size.

   \param[in]    inWin
               Input 2D windowing coefficient for the whole chirp.

   \param[out]    outputPtr
               Pointer to the 16-bit I/Q samples after windowing.

   \pre       none

   \post      none


 */

extern void	RADARDEMO_rangeProcWin2D1AntFxdp(
                            IN  uint32_t fftSize1D,
							IN  int16_t inWin,
							OUT cplx16_t * outputPtr);	

/*!
   \fn     RADARDEMO_rangeProcWin1D1AntFxdpPartial

   \brief   Fixed point windowing function for 1D FFT per antenna per chirp and prepare for the input to 16x16 fixed point 1D FFT, 
            windowing only apply to the window length. This is for XWR16 only.

   \param[in]    nSamplesValidPerChirp
               Number of valid samples per chirp.

   \param[in]    fftSize1D
               1D FFT size.

   \param[in]    fftWin1D
               Input pointer to the rising half of the 1D window function excluding the midpoint, assuming windowing function is symmetric.
			   
   \param[in]    winLength
               Length of fftWin1D, which is (total length of windowing function - 1)/2.

   \param[in, out]    inputPtr
               Pointer to input/output signal.

   \pre       winLength must be smaller than nSamplesValidPerChirp/2.
			  1D windowing function must be symmetric. 
			  winLength must be multiple of 2.

   \post      none


 */

extern void	RADARDEMO_rangeProcWin1D1AntFxdpPartial(
                            IN  uint32_t nSamplesValidPerChirp,
                            IN  int16_t  * fftWin1D,
							IN 	int16_t  winLength,
							INOUT  cplx16_t * inputPtr);

/*!
   \fn     RADARDEMO_rangeProcWin2D1AntFxdpinFltOut

   \brief   Windowing function for 2D FFT per antenna per chirp and prepare for the input to floating point 2D FFT. The input to this windowing function is from 16x16 fixed point 1D FFT.

   \param[in]    fftSize1D
               Input 1D FFT size.

   \param[in]    inWin
               Input 2D windowing coefficients for the current chirp.

   \param[out]    outputPtr
               Pointer to floating-point output buffer after 2D windowing, in the order of real[0] imag[0] real[1] imag[1]... in memory.
               Must be aligned to 8-byte boundary.

   \pre       LITTLE ENDIAN ONLY

   \post      Output is stored in memory in the order of real imag real imag...


 */

extern void	RADARDEMO_rangeProcWin2D1AntFxdpinFltOut(
							IN  uint32_t 		fftSize1D,
							IN  int16_t 	inWin,
							INOUT void 		* outputPtr);
							
/*!
   \fn     RADARDEMO_rangeProcWin1D1AntFltp

   \brief   Windowing function for 1D FFT per antenna per chirp and prepare for the input to floating point 1D FFT.

   \param[in]    nSamplesValidPerChirp
               Number of valid samples per chirp.

   \param[in]    fftSize1D
               1D FFT size.

   \param[in]    ADCnob
               Number of ADC bits.

   \param[in]    fftWin1D
               Input pointer to 1D window function.
			   
   \param[in]    inputPtr
               Pointer to input signal.

   \param[in]    adjustDCforInput
               Flag to enable DC adjustment for ADC samples when set to 1. Otherwise, no preprocessing for ADC samples.

   \param[out]    outputPtr
               Pointer to the output signal.

   \pre       none

   \post      none


 */
extern void	RADARDEMO_rangeProcWin1D1AntFltp(
                            IN  uint32_t nSamplesValidPerChirp,
                            IN  uint32_t fftSize1D,
							IN 	uint8_t  ADCnob,
                            IN  int16_t  * fftWin1D,
							IN  cplx16_t * inputPtr,
							IN  uint8_t  adjustDCforInput,
							OUT float * outputPtr);		

/*!
   \fn     RADARDEMO_rangeProcWin2D1Antfltp

   \brief   Windowing function for 2D FFT per antenna per per chirp and prepare for the input to floating point 2D FFT.

   \param[in]    fftSize1D
               1D FFT size.

   \param[in]    inWin
               Input 2D windowing coefficient for this chirp.

   \param[in, out]    outputPtr
               Pointer to the output signal.

   \pre       windowing function is done in place of the input memory.

   \post      none


 */

extern void	RADARDEMO_rangeProcWin2D1Antfltp(
                            IN  uint32_t fftSize1D,
							IN  float inWin,
							OUT float * outputPtr);							
							
extern int gen_twiddle_fft16x16_imre (
    short *w,
    int n
);

extern void tw_gen_float (float *w, int n);

#ifdef _WIN32
extern void DSP_fft16x16_imre (
    const short * RESTRICT ptr_w,
    int npoints,
    short * RESTRICT ptr_x,
    short * RESTRICT ptr_y
);

extern void DSPF_sp_fftSPxSP (int N, float *ptr_x, float *ptr_w, float *ptr_y,
    unsigned char *brev, int n_min, int offset, int n_max);
#endif
#endif //RADARDEMO_RANGEPROC_RPIV_H

