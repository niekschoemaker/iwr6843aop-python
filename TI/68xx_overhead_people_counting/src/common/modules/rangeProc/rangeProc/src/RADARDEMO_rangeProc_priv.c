/*! 
 *  \file   RADARDEMO_rangeProc_priv.c
 *
 *  \brief   Windowing functions for range processing.
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

#include "RADARDEMO_rangeProc_priv.h"

#ifdef _TMS320C6X
#include "c6x.h"
#endif

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
void	RADARDEMO_rangeProcWin1D1AntFltp(
                            IN  uint32_t nSamplesValidPerChirp,
                            IN  uint32_t fftSize1D,
							IN 	uint8_t  ADCnob,
                            IN  int16_t  * fftWin1D,
							IN  cplx16_t * inputPtr,
							IN  uint8_t  adjustDCforInput,
							OUT float * outputPtr)

{

	int32_t		j, leftover;
	int32_t		winCoef0, itemp0;
	int64_t    inputMean2;
	int64_t 	lltemp1, lltemp3;
	int32_t 	* RESTRICT winFunc, inWin, * RESTRICT tempInput1Ant;
	int64_t		* RESTRICT input1Ant1, * RESTRICT input2Ant1;
	__float2_t  * RESTRICT output1Ant1, * RESTRICT output2Ant1, f2temp1;
	int16_t     * RESTRICT tempWin;
	int32_t     inputMean;


	if (adjustDCforInput == 1)
	{
		float        fscale;
		__float2_t   inputTotal1, f2scale, inputTotal2, f2mean;

		input1Ant1	=	(int64_t  *) &inputPtr[0];
		inputTotal1 =	_ftof2(0.f, 0.f);
		inputTotal2 =	_ftof2(0.f, 0.f);

		for (j = 0; j < (int32_t) nSamplesValidPerChirp/8; j++)
		{
			lltemp1		=	_amem8(input1Ant1++);
			lltemp3		=	_amem8(input1Ant1++);
			lltemp1		=	_davg2(lltemp1, lltemp3);
			itemp0		=	_avg2(_hill(lltemp1), _loll(lltemp1));
			inputTotal1	=	_daddsp(inputTotal1, _dinthsp(itemp0));
			lltemp1		=	_amem8(input1Ant1++);
			lltemp3		=	_amem8(input1Ant1++);
			lltemp1		=	_davg2(lltemp1, lltemp3);
			itemp0		=	_avg2(_hill(lltemp1), _loll(lltemp1));
			inputTotal2	=	_daddsp(inputTotal2, _dinthsp(itemp0));
		}
		inputTotal1	=	_daddsp(inputTotal1, inputTotal2);

		tempInput1Ant	=	(int32_t *) input1Ant1;
		for (j = 0; j < (int32_t) (nSamplesValidPerChirp & 0x7); j++)
		{
			itemp0			=	_amem4(&tempInput1Ant[j]);
			inputTotal1		=	_daddsp(inputTotal1, _dinthsp(_shr2(itemp0, 2)));
		}
		fscale			=	_rcpsp((float)(nSamplesValidPerChirp));
		fscale			=   fscale * (2.f - fscale * (float)nSamplesValidPerChirp);
		fscale			=   fscale * (2.f - fscale * (float)nSamplesValidPerChirp);
		fscale			=	fscale * 4.f;
		f2scale			=	_ftof2(fscale, fscale);
		f2mean			=	_dmpysp(inputTotal1, f2scale);
		inputMean		=	_dspinth(f2mean);
	}
	else if (adjustDCforInput == 2)
	{
		float        fscale, ftemp2;
		__float2_t   inputTotal1, f2scale, inputTotal2, f2mean;

		input1Ant1	=	(int64_t  *) &inputPtr[0];
		inputTotal1 =	_ftof2(0.f, 0.f);
		inputTotal2 =	_ftof2(0.f, 0.f);
		if (ADCnob == 16)
		{
			for (j = 0; j < (int32_t) nSamplesValidPerChirp/8; j++)
			{
				lltemp1		=	_amem8(input1Ant1++);
				lltemp3		=	_amem8(input1Ant1++);
				lltemp1		=	_davg2(lltemp1, lltemp3);
				itemp0		=	_avg2(_hill(lltemp1), _loll(lltemp1));
				inputTotal1	=	_daddsp(inputTotal1, _dinthsp(itemp0));
				lltemp1		=	_amem8(input1Ant1++);
				lltemp3		=	_amem8(input1Ant1++);
				lltemp1		=	_davg2(lltemp1, lltemp3);
				itemp0		=	_avg2(_hill(lltemp1), _loll(lltemp1));
				inputTotal2	=	_daddsp(inputTotal2, _dinthsp(itemp0));
			}
			inputTotal1	=	_daddsp(inputTotal1, inputTotal2);

			tempInput1Ant	=	(int32_t *) input1Ant1;
			for (j = 0; j < (int32_t) (nSamplesValidPerChirp & 0x7); j++)
			{
				itemp0			=	_amem4(&tempInput1Ant[j]);
				inputTotal1		=	_daddsp(inputTotal1, _dinthsp(_shr2(itemp0, 2)));
			}
		}
		else
		{
			uint64_t 	mask;
			int64_t     signCheck, signAdjust;
			int32_t		itemp1;
			
			itemp0		=	(1 << (ADCnob-1)) - 1;
			itemp0		=	_pack2(itemp0, itemp0);
			signCheck	=	_itoll(itemp0, itemp0);
			itemp0		=	1 << ADCnob;
			itemp0		=	_pack2(itemp0, itemp0);
			signAdjust	=	_itoll(itemp0, itemp0);
			
			for (j = 0; j < (int32_t) nSamplesValidPerChirp/8; j++)
			{
				lltemp1		=	_amem8(input1Ant1);
				itemp1		=	_dcmpgt2(lltemp1, signCheck);
				mask		=	_dxpnd2(itemp1);
				lltemp1		=	_dssub2(lltemp1,  mask & signAdjust);
				_amem8(input1Ant1++) = lltemp1;
				lltemp3		=	_amem8(input1Ant1);
				itemp1		=	_dcmpgt2(lltemp3, signCheck);
				mask		=	_dxpnd2(itemp1);
				lltemp3		=	_dssub2(lltemp3,  mask & signAdjust);
				_amem8(input1Ant1++) = lltemp3;
				lltemp1		=	_davg2(lltemp1, lltemp3);
				itemp0		=	_avg2(_hill(lltemp1), _loll(lltemp1));
				inputTotal1	=	_daddsp(inputTotal1, _dinthsp(itemp0));
				//inputTotal1	=	_daddsp(inputTotal1, _dinthsp(_hill(lltemp1)));
				//inputTotal1	=	_daddsp(inputTotal1, _dinthsp(_loll(lltemp1)));
				
				lltemp1		=	_amem8(input1Ant1);
				itemp1		=	_dcmpgt2(lltemp1, signCheck);
				mask		=	_dxpnd2(itemp1);
				lltemp1		=	_dssub2(lltemp1,  mask & signAdjust);
				_amem8(input1Ant1++) = lltemp1;
				lltemp3		=	_amem8(input1Ant1);
				itemp1		=	_dcmpgt2(lltemp3, signCheck);
				mask		=	_dxpnd2(itemp1);
				lltemp3		=	_dssub2(lltemp3,  mask & signAdjust);
				_amem8(input1Ant1++) = lltemp3;
				lltemp1		=	_davg2(lltemp1, lltemp3);
				itemp0		=	_avg2(_hill(lltemp1), _loll(lltemp1));
				inputTotal2	=	_daddsp(inputTotal2, _dinthsp(itemp0));
				//inputTotal2	=	_daddsp(inputTotal2, _dinthsp(_hill(lltemp1)));
				//inputTotal2	=	_daddsp(inputTotal2, _dinthsp(_loll(lltemp1)));
			}
			inputTotal1	=	_daddsp(inputTotal1, inputTotal2);

			tempInput1Ant	=	(int32_t *) input1Ant1;
			for (j = 0; j < (int32_t) (nSamplesValidPerChirp & 0x7); j++)
			{
				itemp0			=	_amem4(&tempInput1Ant[j]);
				itemp1			=	_cmpgt2(itemp0, _hill(signCheck));
				itemp1			=	_xpnd2(itemp1);
				itemp0			=	_ssub2(itemp0,  itemp1 & _hill(signAdjust));
				_amem4(&tempInput1Ant[j]) = itemp0;
				inputTotal1		=	_daddsp(inputTotal1, _dinthsp(_shr2(itemp0, 2)));
			}
		}
		ftemp2			=	(float)(nSamplesValidPerChirp) * 0.25f;
		fscale			=	_rcpsp(ftemp2);
		fscale			=   fscale * (2.f - fscale * ftemp2);
		fscale			=   fscale * (2.f - fscale * ftemp2);
		f2scale			=	_ftof2(fscale, fscale);
		f2mean			=	_dmpysp(inputTotal1, f2scale);
		inputMean		=	_dspinth(f2mean);
	}
	else
	{
		inputMean		=	0;
	}
	inputMean2			=	_itoll(inputMean, inputMean);

	winFunc = (int32_t *)fftWin1D;
	input1Ant1 = (int64_t  *) &inputPtr[0];
	input2Ant1 = (int64_t  *) &inputPtr[nSamplesValidPerChirp - 2];
	output1Ant1 = (__float2_t  *) outputPtr;
	output2Ant1 = (__float2_t  *) &outputPtr[2 * nSamplesValidPerChirp - 2];

	/* windowing - assuming symmetric*/
	if( (nSamplesValidPerChirp & 0x3) == 0)
	{
		for (j = 0; j < (int32_t) nSamplesValidPerChirp/4; j++)
		{
			inWin       =   _amem4(&winFunc[j]);
			lltemp1		=	_itoll(_packh2(inWin, 0), _pack2(inWin, 0));
			lltemp3		=	_dcmpyr1(lltemp1, _dssub2(_amem8(input1Ant1++), inputMean2));
			itemp0		=	_loll(lltemp3);
			_amem8_f2(output1Ant1++) =	_dinthsp(_packlh2(itemp0, itemp0));
			itemp0		=	_hill(lltemp3);
			_amem8_f2(output1Ant1++) =	_dinthsp(_packlh2(itemp0, itemp0));

			lltemp1		=	_itoll(_pack2(inWin, 0), _packh2(inWin, 0));
			lltemp3		=	_dcmpyr1(lltemp1, _dssub2(_amem8(input2Ant1--), inputMean2));
			itemp0		=	_hill(lltemp3);
			_amem8_f2(output2Ant1--)		=	_dinthsp(_packlh2(itemp0, itemp0));
			itemp0		=	_loll(lltemp3);
			_amem8_f2(output2Ant1--)		=	_dinthsp(_packlh2(itemp0, itemp0));
		}
	}
	else if ( (nSamplesValidPerChirp & 0x3) == 2)
	{
		for (j = 0; j < (int32_t) nSamplesValidPerChirp/4; j++)
		{
			inWin       =   _amem4(&winFunc[j]);
			lltemp1		=	_itoll(_packh2(inWin, 0), _pack2(inWin, 0));
			lltemp3		=	_dcmpyr1(lltemp1, _dssub2(_amem8(input1Ant1++), inputMean2));
			itemp0		=	_loll(lltemp3);
			_amem8_f2(output1Ant1++) =	_dinthsp(_packlh2(itemp0, itemp0));
			itemp0		=	_hill(lltemp3);
			_amem8_f2(output1Ant1++) =	_dinthsp(_packlh2(itemp0, itemp0));

			lltemp1		=	_itoll(_pack2(inWin, 0), _packh2(inWin, 0));
			lltemp3		=	_dcmpyr1(lltemp1, _dssub2(_amem8(input2Ant1--), inputMean2));
			itemp0		=	_hill(lltemp3);
			_amem8_f2(output2Ant1--)		=	_dinthsp(_packlh2(itemp0, itemp0));
			itemp0		=	_loll(lltemp3);
			_amem8_f2(output2Ant1--)		=	_dinthsp(_packlh2(itemp0, itemp0));
		}
		//last pair
		inWin       =   _amem4(&winFunc[j]);
		lltemp1		=	_itoll(_packh2(inWin, 0), _pack2(inWin, 0));
		lltemp3		=	_dcmpyr1(lltemp1, _dssub2(_amem8(input1Ant1++), inputMean2));
		itemp0		=	_loll(lltemp3);
		_amem8_f2(output1Ant1++) =	_dinthsp(_packlh2(itemp0, itemp0));
		itemp0		=	_hill(lltemp3);
		_amem8_f2(output1Ant1++) =	_dinthsp(_packlh2(itemp0, itemp0));
	}
	else
	{
		for (j = 0; j < (int32_t) nSamplesValidPerChirp/4; j++)
		{
			inWin       =   _amem4(&winFunc[j]);
			lltemp1		=	_itoll(_packh2(inWin, 0), _pack2(inWin, 0));
			lltemp3		=	_dcmpyr1(lltemp1, _dssub2(_amem8(input1Ant1++), inputMean2));
			itemp0		=	_loll(lltemp3);
			_amem8_f2(output1Ant1++) =	_dinthsp(_packlh2(itemp0, itemp0));
			itemp0		=	_hill(lltemp3);
			_amem8_f2(output1Ant1++) =	_dinthsp(_packlh2(itemp0, itemp0));

			lltemp1		=	_itoll(_pack2(inWin, 0), _packh2(inWin, 0));
			lltemp3		=	_dcmpyr1(lltemp1, _dssub2(_mem8(input2Ant1--), inputMean2));
			itemp0		=	_hill(lltemp3);
			_amem8_f2(output2Ant1--)		=	_dinthsp(_packlh2(itemp0, itemp0));
			itemp0		=	_loll(lltemp3);
			_amem8_f2(output2Ant1--)		=	_dinthsp(_packlh2(itemp0, itemp0));
		}
		//last 1 or 3
		tempInput1Ant	=	(int32_t *) input1Ant1;
		tempWin			=	(int16_t *) &winFunc[j];
		for (j = 0; j < ((int32_t) (nSamplesValidPerChirp & 0x3) - 1); j++)
		{
			inWin		=	tempWin[j];
			winCoef0	=	_pack2(inWin, 0);
			itemp0		=	_cmpyr1(_ssub2(_amem4(&tempInput1Ant[j]), inputMean), winCoef0); 
			_amem8_f2(output1Ant1++)=	_ftof2((float)(_ext(itemp0, 16, 16)), (float)(_ext(itemp0, 0, 16)));
		}
		if((nSamplesValidPerChirp & 0x3) > 2)
			inWin		=	tempWin[j-2];
		else
			inWin		=	tempWin[j];
		winCoef0	=	_pack2(inWin, 0);
		itemp0		=	_cmpyr1(_ssub2(_amem4(&tempInput1Ant[j]), inputMean), winCoef0); 
		_amem8_f2(output1Ant1++)=	_ftof2((float)(_ext(itemp0, 16, 16)), (float)(_ext(itemp0, 0, 16)));
	}
	leftover 	=	(4 - (nSamplesValidPerChirp & 0x3)) & 0x3;
	for (j = 0; j < leftover * 2; j++)
	{
		outputPtr[2 * nSamplesValidPerChirp + j] = 0.f;
	}

	/*zero padding to FFT1DSize*/
	output1Ant1 = (__float2_t  *) &outputPtr[2 * nSamplesValidPerChirp + 2 * leftover];
	f2temp1 = _ftof2(0.f, 0.f);
	for (j = 0; j < ((int32_t) fftSize1D - (int32_t) nSamplesValidPerChirp)/2; j++)
	{
		_amem8_f2(output1Ant1++) = f2temp1;
		_amem8_f2(output1Ant1++) = f2temp1;
	}
}


/*!
   \fn     RADARDEMO_rangeProcWin1D1AntFxdp

   \brief   Fixed point windowing function for 1D FFT per antenna per chirp and prepare for the input to 16x16 fixed point 1D FFT.

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

void	RADARDEMO_rangeProcWin1D1AntFxdp(
                            IN  uint32_t nSamplesValidPerChirp,
                            IN  uint32_t fftSize1D,
                            IN  int16_t  * fftWin1D,
							IN 	uint8_t  ADCnob,
							IN  cplx16_t * inputPtr,
							IN  uint8_t  adjustDCforInput,
							OUT cplx16_t * outputPtr)

{

	int32_t		j, leftover, scale;
	int32_t		winCoef0, itemp0;
	int64_t 	inputMean2;
	int64_t 	lltemp1, lltemp3;
	int32_t 	* RESTRICT winFunc, inWin, * RESTRICT tempInput1Ant, * RESTRICT tempOutput1Ant;
	int64_t		* RESTRICT input1Ant1, * RESTRICT input2Ant1, * RESTRICT output1Ant1, * RESTRICT output2Ant1;
	int16_t     * RESTRICT tempWin;
	int32_t     inputMean;

	if (adjustDCforInput == 1)
	{
		float        fscale;
		__float2_t   inputTotal1, f2scale, inputTotal2, f2mean;

		input1Ant1	=	(int64_t  *) &inputPtr[0];
		inputTotal1 =	_ftof2(0.f, 0.f);
		inputTotal2 =	_ftof2(0.f, 0.f);

		for (j = 0; j < (int32_t) nSamplesValidPerChirp/8; j++)
		{
			lltemp1		=	_amem8(input1Ant1++);
			lltemp3		=	_amem8(input1Ant1++);
			lltemp1		=	_davg2(lltemp1, lltemp3);
			itemp0		=	_avg2(_hill(lltemp1), _loll(lltemp1));
			inputTotal1	=	_daddsp(inputTotal1, _dinthsp(itemp0));
			lltemp1		=	_amem8(input1Ant1++);
			lltemp3		=	_amem8(input1Ant1++);
			lltemp1		=	_davg2(lltemp1, lltemp3);
			itemp0		=	_avg2(_hill(lltemp1), _loll(lltemp1));
			inputTotal2	=	_daddsp(inputTotal2, _dinthsp(itemp0));
		}
		inputTotal1	=	_daddsp(inputTotal1, inputTotal2);

		tempInput1Ant	=	(int32_t *) input1Ant1;
		for (j = 0; j < (int32_t) (nSamplesValidPerChirp & 0x7); j++)
		{
			itemp0			=	_amem4(&tempInput1Ant[j]);
			inputTotal1		=	_daddsp(inputTotal1, _dinthsp(_shr2(itemp0, 2)));
		}
		fscale			=	_rcpsp((float)(nSamplesValidPerChirp));
		fscale			=   fscale * (2.f - fscale * (float)nSamplesValidPerChirp);
		fscale			=   fscale * (2.f - fscale * (float)nSamplesValidPerChirp);
		fscale			=	fscale * 4.f;
		f2scale			=	_ftof2(fscale, fscale);
		f2mean			=	_dmpysp(inputTotal1, f2scale);
		inputMean		=	_dspinth(f2mean);
	}
	else if (adjustDCforInput == 2)
	{
		float        fscale, ftemp2;
		__float2_t   inputTotal1, f2scale, inputTotal2, f2mean;

		input1Ant1	=	(int64_t  *) &inputPtr[0];
		inputTotal1 =	_ftof2(0.f, 0.f);
		inputTotal2 =	_ftof2(0.f, 0.f);
		if (ADCnob == 16)
		{
			for (j = 0; j < (int32_t) nSamplesValidPerChirp/8; j++)
			{
				lltemp1		=	_amem8(input1Ant1++);
				lltemp3		=	_amem8(input1Ant1++);
				lltemp1		=	_davg2(lltemp1, lltemp3);
				itemp0		=	_avg2(_hill(lltemp1), _loll(lltemp1));
				inputTotal1	=	_daddsp(inputTotal1, _dinthsp(itemp0));
				lltemp1		=	_amem8(input1Ant1++);
				lltemp3		=	_amem8(input1Ant1++);
				lltemp1		=	_davg2(lltemp1, lltemp3);
				itemp0		=	_avg2(_hill(lltemp1), _loll(lltemp1));
				inputTotal2	=	_daddsp(inputTotal2, _dinthsp(itemp0));
			}
			inputTotal1	=	_daddsp(inputTotal1, inputTotal2);

			tempInput1Ant	=	(int32_t *) input1Ant1;
			for (j = 0; j < (int32_t) (nSamplesValidPerChirp & 0x7); j++)
			{
				itemp0			=	_amem4(&tempInput1Ant[j]);
				inputTotal1		=	_daddsp(inputTotal1, _dinthsp(_shr2(itemp0, 2)));
			}
		}
		else
		{
			uint64_t 	mask;
			int64_t     signCheck, signAdjust;
			int32_t		itemp1;
			
			itemp0		=	(1 << (ADCnob-1)) - 1;
			itemp0		=	_pack2(itemp0, itemp0);
			signCheck	=	_itoll(itemp0, itemp0);
			itemp0		=	1 << ADCnob;
			itemp0		=	_pack2(itemp0, itemp0);
			signAdjust	=	_itoll(itemp0, itemp0);
			
			for (j = 0; j < (int32_t) nSamplesValidPerChirp/8; j++)
			{
				lltemp1		=	_amem8(input1Ant1);
				itemp1		=	_dcmpgt2(lltemp1, signCheck);
				mask		=	_dxpnd2(itemp1);
				lltemp1		=	_dssub2(lltemp1,  mask & signAdjust);
				_amem8(input1Ant1++) = lltemp1;
				lltemp3		=	_amem8(input1Ant1);
				itemp1		=	_dcmpgt2(lltemp3, signCheck);
				mask		=	_dxpnd2(itemp1);
				lltemp3		=	_dssub2(lltemp3,  mask & signAdjust);
				_amem8(input1Ant1++) = lltemp3;
				lltemp1		=	_davg2(lltemp1, lltemp3);
				itemp0		=	_avg2(_hill(lltemp1), _loll(lltemp1));
				inputTotal1	=	_daddsp(inputTotal1, _dinthsp(itemp0));
				//inputTotal1	=	_daddsp(inputTotal1, _dinthsp(_hill(lltemp1)));
				//inputTotal1	=	_daddsp(inputTotal1, _dinthsp(_loll(lltemp1)));
				
				lltemp1		=	_amem8(input1Ant1);
				itemp1		=	_dcmpgt2(lltemp1, signCheck);
				mask		=	_dxpnd2(itemp1);
				lltemp1		=	_dssub2(lltemp1,  mask & signAdjust);
				_amem8(input1Ant1++) = lltemp1;
				lltemp3		=	_amem8(input1Ant1);
				itemp1		=	_dcmpgt2(lltemp3, signCheck);
				mask		=	_dxpnd2(itemp1);
				lltemp3		=	_dssub2(lltemp3,  mask & signAdjust);
				_amem8(input1Ant1++) = lltemp3;
				lltemp1		=	_davg2(lltemp1, lltemp3);
				itemp0		=	_avg2(_hill(lltemp1), _loll(lltemp1));
				inputTotal2	=	_daddsp(inputTotal2, _dinthsp(itemp0));
				//inputTotal2	=	_daddsp(inputTotal2, _dinthsp(_hill(lltemp1)));
				//inputTotal2	=	_daddsp(inputTotal2, _dinthsp(_loll(lltemp1)));
			}
			inputTotal1	=	_daddsp(inputTotal1, inputTotal2);

			tempInput1Ant	=	(int32_t *) input1Ant1;
			for (j = 0; j < (int32_t) (nSamplesValidPerChirp & 0x7); j++)
			{
				itemp0			=	_amem4(&tempInput1Ant[j]);
				itemp1			=	_cmpgt2(itemp0, _hill(signCheck));
				itemp1			=	_xpnd2(itemp1);
				itemp0			=	_ssub2(itemp0,  itemp1 & _hill(signAdjust));
				_amem4(&tempInput1Ant[j]) = itemp0;
				inputTotal1		=	_daddsp(inputTotal1, _dinthsp(_shr2(itemp0, 2)));
			}
		}
		ftemp2			=	(float)(nSamplesValidPerChirp) * 0.25f;
		fscale			=	_rcpsp(ftemp2);
		fscale			=   fscale * (2.f - fscale * ftemp2);
		fscale			=   fscale * (2.f - fscale * ftemp2);
		f2scale			=	_ftof2(fscale, fscale);
		f2mean			=	_dmpysp(inputTotal1, f2scale);
		inputMean		=	_dspinth(f2mean);
	}
	else
	{
		inputMean		=	0;
	}
#ifdef _TMS320C6600
	inputMean2			=	_itoll(inputMean, inputMean);
#endif


	winFunc = (int32_t *)fftWin1D;
	input1Ant1 = (int64_t  *) &inputPtr[0];
	input2Ant1 = (int64_t  *) &inputPtr[nSamplesValidPerChirp - 2];
	output1Ant1 = (int64_t  *) outputPtr;
	output2Ant1 = (int64_t  *) &outputPtr[nSamplesValidPerChirp - 2];
	scale		=	15 - ADCnob - WIN1DLEFTSHFTGAP;

	/* windowing - assuming symmetric*/
	if( (nSamplesValidPerChirp & 0x3) == 0)
	{
		for (j = 0; j < (int32_t) nSamplesValidPerChirp/4; j++)
		{
			inWin       =   _amem4(&winFunc[j]);
			lltemp1		=	_itoll(_packh2(inWin, 0), _pack2(inWin, 0));
			_amem8(output1Ant1++)		=	_dcmpyr1(lltemp1, _dshl2(_dssub2(_amem8(input1Ant1++), inputMean2), scale));
			lltemp1		=	_itoll(_pack2(inWin, 0), _packh2(inWin, 0));
			_amem8(output2Ant1--)		=	_dcmpyr1(lltemp1, _dshl2(_dssub2(_amem8(input2Ant1--), inputMean2), scale));
		}
	}
	else if ( (nSamplesValidPerChirp & 0x3) == 2)
	{
		for (j = 0; j < (int32_t) nSamplesValidPerChirp/4; j++)
		{
			inWin       =   _amem4(&winFunc[j]);
			lltemp1		=	_itoll(_packh2(inWin, 0), _pack2(inWin, 0));
			_amem8(output1Ant1++)		=	_dcmpyr1(lltemp1, _dshl2(_dssub2(_amem8(input1Ant1++), inputMean2), scale));
			lltemp1		=	_itoll(_pack2(inWin, 0), _packh2(inWin, 0));
			_amem8(output2Ant1--)		=	_dcmpyr1(lltemp1, _dshl2(_dssub2(_amem8(input2Ant1--), inputMean2), scale));
		}
		//last pair
		inWin       =   _amem4(&winFunc[j]);
		lltemp1		=	_itoll(_packh2(inWin, 0), _pack2(inWin, 0));
		_amem8(output1Ant1++)		=	_dcmpyr1(lltemp1, _dshl2(_dssub2(_amem8(input1Ant1++), inputMean2), scale));
	}
	else
	{
		for (j = 0; j < (int32_t) nSamplesValidPerChirp/4; j++)
		{
			inWin       =   _amem4(&winFunc[j]);
			lltemp1		=	_itoll(_packh2(inWin, 0), _pack2(inWin, 0));
			_amem8(output1Ant1++)		=	_dcmpyr1(lltemp1, _dshl2(_dssub2(_amem8(input1Ant1++), inputMean2), scale));
			lltemp1		=	_itoll(_pack2(inWin, 0), _packh2(inWin, 0));
			_mem8(output2Ant1--)		=	_dcmpyr1(lltemp1, _dshl2(_dssub2(_mem8(input2Ant1--), inputMean2), scale));
		}
		//last 1 or 3
		tempInput1Ant	=	(int32_t *) input1Ant1;
		tempWin			=	(int16_t *) &winFunc[j];
		tempOutput1Ant	=	(int32_t *) output1Ant1;
		for (j = 0; j < ((int32_t) (nSamplesValidPerChirp & 0x3) - 1); j++)
		{
			inWin		=	tempWin[j];
			winCoef0	=	_pack2(inWin, 0);
			itemp0		=	_cmpyr1(_sshl(_ssub2(_amem4(&tempInput1Ant[j]), inputMean), scale), winCoef0);  //_sshl is ok because we know for sure input is ADCnob bit
			_amem4(&tempOutput1Ant[j]) = itemp0;
		}
		if ((nSamplesValidPerChirp & 0x3) > 2)
		{
			inWin		=	tempWin[j-2];
		}
		else
		{
			inWin		=	tempWin[j];
		}
		winCoef0	=	_pack2(inWin, 0);
		itemp0		=	_cmpyr1(_sshl(_ssub2(_amem4(&tempInput1Ant[j]), inputMean), scale), winCoef0);  //_sshl is ok because we know for sure input is ADCnob bit
		_amem4(&tempOutput1Ant[j]) = itemp0;
	}
	leftover 	=	(4 - (nSamplesValidPerChirp & 0x3)) & 0x3;
	for (j = 0; j < leftover; j++)
	{
		_amem4(&outputPtr[nSamplesValidPerChirp + j]) = 0;
	}

	/*zero padding to FFT1DSize*/
	output1Ant1 = (int64_t  *) &outputPtr[nSamplesValidPerChirp + leftover];
	lltemp3 = _itoll(0, 0);
	for (j = 0; j < ((int32_t) fftSize1D - (int32_t) nSamplesValidPerChirp)/4; j++)
	{
		_amem8(output1Ant1++) = lltemp3;
		_amem8(output1Ant1++) = lltemp3;
	}
}


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

void	RADARDEMO_rangeProcWin2D1Antfltp(
                            IN  uint32_t fftSize1D,
							IN  float inWin,
							OUT float * outputPtr)

{

	int32_t		j;
	__float2_t	lltemp3;
	__float2_t  * RESTRICT input1Ant1, * RESTRICT output1Ant1;

	input1Ant1 = (__float2_t  *) outputPtr;
	output1Ant1 = (__float2_t  *) outputPtr;

	/* windowing - assuming symmetric*/
	lltemp3     =   _ftof2(inWin,inWin);
	for (j = 0; j < (int32_t) fftSize1D/2; j++)
	{
		_amem8_f2(output1Ant1++) =	_dmpysp(_amem8_f2(input1Ant1++), lltemp3);
		_amem8_f2(output1Ant1++) =	_dmpysp(_amem8_f2(input1Ant1++), lltemp3);
	}
}


/*!
   \fn     RADARDEMO_rangeProcWin2D1AntFxdp

   \brief   Windowing function for 2D FFT per antenna per chirp and prepare for the input to 16x16 fixed point 2D FFT.

   \param[in]    fftSize1D
               1D FFT size.

   \param[in]    inWin
               Input 2D windowing coefficient for the whole chirp.

   \param[in, out]    outputPtr
               Pointer to the 16-bit I/Q input and output.

   \pre       windowing function is done in place of the input memory.

   \post      none


 */

void	RADARDEMO_rangeProcWin2D1AntFxdp(
                            IN  uint32_t fftSize1D,
							IN  int16_t inWin,
							OUT cplx16_t * outputPtr)

{
	int32_t		j;
	int64_t		lltemp3;
	int64_t		* RESTRICT input1Ant1, * RESTRICT output1Ant1;

	input1Ant1	=	(int64_t  *) outputPtr;
	output1Ant1 =	(int64_t  *) outputPtr;
	lltemp3     =   _itoll(_pack2(inWin, 0), _pack2(inWin, 0));

	/* windowing - assuming symmetric*/
	for (j = 0; j < (int32_t) fftSize1D/4; j++)
	{
		_amem8(output1Ant1++) =	_dcmpyr1(_amem8(input1Ant1++), lltemp3);
		_amem8(output1Ant1++) =	_dcmpyr1(_amem8(input1Ant1++), lltemp3);
	}
}


/*!
   \fn     RADARDEMO_rangeProcWin2D1AntFxdpinFltOut

   \brief   Windowing function for 2D FFT per antenna per chirp and prepare for the input to floating point 2D FFT. The input to this windowing function is from 16x16 fixed point 1D FFT.

   \param[in]    fftSize1D
               Input 1D FFT size.

   \param[in]    inWin
               Input 2D windowing coefficients for the current chirp.

   \param[in, out]    inoutPtr
               Pointer to floating-point output buffer after 2D windowing, in the order of real[0] imag[0] real[1] imag[1]... in memory.
               Must be aligned to 8-byte boundary.

   \pre       windowing function is done in place of the input memory. Input is at the first half of the buffer pointed by inoutPtr, in cplx16_t format

   \post      Output is floating point, stored in memory in the order of real imag real imag...


 */

void	RADARDEMO_rangeProcWin2D1AntFxdpinFltOut(
							IN  uint32_t 	fftSize1D,
							IN  int16_t 	inWin,
							INOUT void 		* inoutPtr)

{

	int32_t		j;
	int64_t		lltemp1;
	int64_t		lltemp3;
	int64_t		* RESTRICT input1Ant1;
	__float2_t   * RESTRICT output1Ant1;
	int32_t  	win2, * fxpTempPtr;
	int32_t		itemp;
	float       * fltpTempPtr;

	fxpTempPtr  =	(int32_t  *) inoutPtr;
	input1Ant1	=	(int64_t  *) &fxpTempPtr[fftSize1D - 2];
	fltpTempPtr =	(float  *) inoutPtr;
	output1Ant1 =	(__float2_t  *) &fltpTempPtr[2 * fftSize1D - 2];
	win2		=	_pack2(inWin, 0);

	/* windowing - assuming symmetric*/
	for (j = 0; j < (int32_t) fftSize1D/2; j++)
	{
		lltemp1					=	_amem8(input1Ant1--);

		lltemp3					=	_dcmpyr1(lltemp1, _itoll(win2, win2));
		itemp					=	_hill(lltemp3);
		_amem8_f2(output1Ant1--) =	_dinthsp(_packlh2(itemp, itemp));
		itemp					=	_loll(lltemp3);
		_amem8_f2(output1Ant1--) =	_dinthsp(_packlh2(itemp, itemp));
	}
}


/*!
   \fn     RADARDEMO_rangeProcWin1D1AntFxdpPartial

   \brief   Fixed point windowing function for 1D FFT per antenna per chirp and prepare for the input to 16x16 fixed point 1D FFT, 
            windowing only apply to the window length. This is for XWR16 only.

   \param[in]    nSamplesValidPerChirp
               Number of valid samples per chirp.

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

void	RADARDEMO_rangeProcWin1D1AntFxdpPartial(
                            IN  uint32_t nSamplesValidPerChirp,
                            IN  int16_t  * fftWin1D,
							IN 	int16_t  winLength,
							INOUT  cplx16_t * inputPtr)

{

	int32_t		j;
	int64_t 	lltemp1;
	int32_t 	* RESTRICT winFunc, inWin;
	int64_t		* RESTRICT input1Ant1, * RESTRICT input2Ant1, * RESTRICT output1Ant1, * RESTRICT output2Ant1;

	winFunc = (int32_t *)fftWin1D;
	input1Ant1 = (int64_t  *) &inputPtr[0];
	input2Ant1 = (int64_t  *) &inputPtr[nSamplesValidPerChirp - 2];
	output1Ant1 = (int64_t  *) &inputPtr[0];
	output2Ant1 = (int64_t  *) &inputPtr[nSamplesValidPerChirp - 2];

	/* windowing - assuming symmetric*/
	for (j = 0; j < (int32_t) winLength/2; j++)
	{
		inWin       =   _amem4(&winFunc[j]);
		lltemp1		=	_itoll(_packh2(inWin, 0), _pack2(inWin, 0));
		_amem8(output1Ant1++)		=	_dcmpyr1(lltemp1, _amem8(input1Ant1++));
		lltemp1		=	_itoll(_pack2(inWin, 0), _packh2(inWin, 0));
		_mem8(output2Ant1--)		=	_dcmpyr1(lltemp1, _mem8(input2Ant1--));
	}
}


