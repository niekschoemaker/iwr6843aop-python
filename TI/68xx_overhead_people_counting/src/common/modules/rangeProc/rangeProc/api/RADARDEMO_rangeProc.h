/*! 
 *  \file   RADARDEMO_rangeProc.h
 *
 *  \brief   Header file for RADARDEMO_rangeProc module
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

#ifndef RADARDEMO_RANGEPROC_H
#define RADARDEMO_RANGEPROC_H

#include <swpform.h>
#include <modules/utilities/radarOsal_malloc.h>

#define RADARDEMO_RANGEPROC_MAXADCNBITS (16)   /**< Maximum number of ADC output bits*/
//#ifndef _TMS320C6600 //C674x
#define DSPLIBFFT16X16MEMPAD (0)
//#else
//#define DSPLIBFFT16X16MEMPAD (0)
//#endif


/**
 *  \enum   
 *   {
 *	RADARDEMO_RANGEPROC_NO_ERROR = 0,				
 *	RADARDEMO_RANGEPROC_1DFFTTYPE_NOTSUPPORTED,		
 *	RADARDEMO_RANGEPROC_1DFFTSIZE_NOTVALID,        
 *  RADARDEMO_RANGEPROC_ADCNBITS_OUTOFRANGE,       
 *  RADARDEMO_RANGEPROC_OUTPUTANDFFTTYPE_MISMATCH, 
 *  RADARDEMO_RANGEPROC_FAIL_ALLOCATE_HANDLE,		
 *  RADARDEMO_RANGEPROC_FAIL_ALLOCATE_LOCALINSTMEM,
 *  RADARDEMO_RANGEPROC_INOUTPTR_NOTCORRECT,	   
 *   }   RADARDEMO_rangeProc_errorCode;
 *
 *  \brief   enum for range processing error code.
 *
 *
 */

typedef enum
{
	RADARDEMO_RANGEPROC_NO_ERROR = 0,				/**< no error */
	RADARDEMO_RANGEPROC_1DFFTTYPE_NOTSUPPORTED,		/**< 1D FFT non supported type */
	RADARDEMO_RANGEPROC_1DFFTSIZE_NOTVALID,         /**< 1D FFT non valid size */ 
	RADARDEMO_RANGEPROC_ADCNBITS_OUTOFRANGE,        /**< number of ADC bits out of range (> 16 bits) */ 
	RADARDEMO_RANGEPROC_UNSUPPORTEDADCREFORMAT, /**< Unsupported ADC reformatting request*/ 
	RADARDEMO_RANGEPROC_OUTPUTANDFFTTYPE_MISMATCH,  /**< range processing output type and FFT type mismatch */ 
	RADARDEMO_RANGEPROC_FAIL_ALLOCATE_HANDLE,		/**< RADARDEMO_rangeProc_create failed to allocate handle */ 
	RADARDEMO_RANGEPROC_FAIL_ALLOCATE_LOCALINSTMEM,	/**< RADARDEMO_rangeProc_create failed to allocate memory for buffers in local instance  */ 
	RADARDEMO_RANGEPROC_INOUTPTR_NOTCORRECT	    /**< input and/or output buffer for RADARDEMO_rangeProc_run are either NULL, or not aligned properly  */
} RADARDEMO_rangeProc_errorCode;


/**
 *  \enum   
 *   {
 *	RADARDEMO_RANGEPROC_1DFFT_16x16 = 0,
 *	RADARDEMO_RANGEPROC_1DFFT_SPxSP,
 *	RADARDEMO_RANGEPROC_1DFFT_NOT_SUPPORTED
 *   }   RADARDEMO_rangeProc_1DFFTType;
 *
 *  \brief   enum for 1D FFT types.
 *
 *
 */

typedef enum
{
	RADARDEMO_RANGEPROC_1DFFT_16x16 = 0,			/**< 1D FFT type: 16x16 fixed point*/
	RADARDEMO_RANGEPROC_1DFFT_SPxSP,				/**< 1D FFT type: spxsp single precision floating point*/
	RADARDEMO_RANGEPROC_1DFFT_NOT_SUPPORTED
} RADARDEMO_rangeProc_1DFFTType;

/**
 *  \enum   
 *   {
 *	RADARDEMO_RANGEPROC_ADC_NO_ADJUST = 0,	
 *	RADARDEMO_RANGEPROC_ADC_DCREMOVAL_ONLY,	
 *	RADARDEMO_RANGEPROC_ADC_DCREM_SIGNEXT,	
 *   RADARDEMO_RANGEPROC_ADCADJUSTTYPE_NOT_SUPPORTED
 *   }   RADARDEMO_rangeProc_ADCAdjustType;
 *
 *  \brief   enum for 1D FFT types.
 *
 *
 */

typedef enum
{
	RADARDEMO_RANGEPROC_ADC_NO_ADJUST = 0,			/**< No adjustment for input ADC samples*/
	RADARDEMO_RANGEPROC_ADC_DCREMOVAL_ONLY,		/**< Only remove DC components*/
	RADARDEMO_RANGEPROC_ADC_DCREM_SIGNEXT,		/**< Sign-extend for input ADC samples, and remove DC components*/
	RADARDEMO_RANGEPROC_ADCADJUSTTYPE_NOT_SUPPORTED
} RADARDEMO_rangeProc_ADCAdjustType;


/**
 *  \struct   _RADARDEMO_rangeProc_input_
 *   {
 *   	cplx16_t     *inputSignal;  		
 *		uint32_t     chirpNumber;	
 *   }   RADARDEMO_rangeProc_input;
 *
 *  \brief   Structure for input to RADARDEMO_rangeProc module.
 *
 *
 */

typedef struct _RADARDEMO_rangeProc_input_
{
	cplx16_t     *inputSignal;  				/**< Input signal from ADC*/
	uint32_t     chirpNumber;					/**< chirp number: 0 to number of chirps per frame*/
} RADARDEMO_rangeProc_input;

/**
 *  \struct   _RADARDEMO_rangeProc_config_
 *   {
 *   	uint32_t     fft1DSize;  					
 *   	uint32_t     nSamplesPerChirp;				
 *      uint32_t     numChirpsPerFrame;
 *		uint8_t      nRxAnt;  	    
 *      uint8_t      adcNumOutputBits; 
 *		RADARDEMO_rangeProc_1DFFTType fft1DType;  	   
 *		uint8_t      include2DwinFlag;  	    	
 *  	uint8_t      outputFixedpFlag;  	  
 *      int16_t      * win1D;  	   
 *      int16_t      * win2D;
 *      int16_t      win1DLength;
 *      RADARDEMO_rangeProc_ADCAdjustType      adjustDCforInput;
 *   }   RADARDEMO_rangeProc_config;
 *
 *  \brief   Structure element of the list of descriptors for RADARDEMO_rangeProc configuration.
 *
 *
 */

typedef struct _RADARDEMO_rangeProc_config_
{
	uint32_t     fft1DSize;  					/**< 1D FFT size*/
	uint32_t     nSamplesPerChirp;				/**< number of samples per chirp*/
	uint32_t     numChirpsPerFrame;				/**< number of chirp per frame*/
	uint8_t      nRxAnt;  	       				/**< number of receive antennas.*/
	uint8_t      adcNumOutputBits;  	        /**< number of ADC output bits.*/
	RADARDEMO_rangeProc_1DFFTType fft1DType;  	/**< Type of 1D FFT.*/
	uint8_t      include2DwinFlag;  	    	/**< Flag indicating 2D windowing is done with range processing when set to 1. Otherwise, 2D windowing is done in Doppler processing*/
	uint8_t      outputFixedpFlag;  	    	/**< Flag indicating range processing output is 16x16 fixed point when set to 1. Otherwise, output single precision floating point.*/
	int16_t      * win1D;  	    	            /**< pointer to 1D windowing function.*/
	int16_t      * win2D;  	    	            /**< pointer to 2D windowing function.*/
	int16_t      win1DLength;  	    	        /**< half length of the 1D windowing function.*/
	RADARDEMO_rangeProc_ADCAdjustType      adjustDCforInput;  	   /**< Flag for ADC sample adjustment, see definition of RADARDEMO_rangeProc_ADCAdjustType*/
} RADARDEMO_rangeProc_config;


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

extern void	* RADARDEMO_rangeProc_create(
                            IN  RADARDEMO_rangeProc_config * moduleConfig, 
							OUT RADARDEMO_rangeProc_errorCode * errorCode);

/*! 
   \fn     RADARDEMO_rangeProc_delete
 
   \brief   Delete RADARDEMO_rangeProc module. 
  
   \param[in]    handle
               Module handle.
			   
   \pre       none
 
   \post      none
  
 
 */

extern void	RADARDEMO_rangeProc_delete(
                            IN  void * handle);


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

extern RADARDEMO_rangeProc_errorCode	RADARDEMO_rangeProc_run(
                            IN  void * handle,
							IN  RADARDEMO_rangeProc_input * rangeProcInput,
							OUT void   * outputSignal);

#endif //RADARDEMO_RANGEPROC_H

