/*! 
 *  \file   RADARDEMO_rangeProc_utils.c
 *
 *  \brief   Utility functions for Range processing. 
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
#ifdef _TMS320C6X
#include "c6x.h"
#endif
#include "math.h"

#ifndef _GENTWIDDLE_FFT16x16_IMRE
#define _GENTWIDDLE_FFT16x16_IMRE

/* ======================================================================== */
/*  D2S -- Truncate a 'double' to a 'short', with clamping.                 */
/* ======================================================================== */

static short d2s(double d)
{
    d = floor(0.5 + d);  // Explicit rounding to integer //
    if (d >=  32767.0) return  32767;
    if (d <= -32768.0) return -32768;
    return (short)d;
}

/* ======================================================================== */
/*  GEN_TWIDDLE -- Generate twiddle factors for TI's custom FFTs.           */
/*                                                                          */
/*  USAGE                                                                   */
/*      This routine is called as follows:                                  */
/*                                                                          */
/*          int gen_twiddle_fft16x16_imre(short *w, int n)                  */
/*                                                                          */
/*          short *w      Pointer to twiddle-factor array                   */
/*          int   n       Size of FFT                                       */
/*                                                                          */
/*      The routine will generate the twiddle-factors directly into the     */
/*      array you specify.  The array needs to be approximately 2*N         */
/*      elements long.  (The actual size, which is slightly smaller, is     */
/*      returned by the function.)                                          */
/* ======================================================================== */
#ifdef _TMS320C6600 //C66x
//#if 0
int gen_twiddle_fft16x16_imre(short *w, int n)
{
    int i, j, k;
    double M = 32767.5;
    const double PI = 3.14159265358979323846;

    for (j = 1, k = 0; j < n >> 2; j = j << 2) {
        for (i = 0; i < n >> 2; i += j << 1) {

            w[k +  3] =  d2s(M * cos(2.0 * PI * (i + j) / n));
            w[k +  2] = -d2s(M * sin(2.0 * PI * (i + j) / n));
            w[k +  1] =  d2s(M * cos(2.0 * PI * (i    ) / n));
            w[k +  0] = -d2s(M * sin(2.0 * PI * (i    ) / n));

            k += 4;
        }
    }

    w[k + 3] =  w[k - 1];
    w[k + 2] =  w[k - 2];
    w[k + 1] =  w[k - 3];
    w[k + 0] =  w[k - 4];
    k += 4;

    return k;
}
#else
int gen_twiddle_fft16x16_imre(short *w, int n)
{
    int i, j, k;
    double M = 32767.5;
    const double PI = 3.14159265358979323846;

    for (j = 1, k = 0; j < n >> 2; j = j << 2) {
        for (i = 0; i < n >> 2; i += j << 1) {
            w[k + 11] =  d2s(M * cos(6.0 * PI * (i + j) / n));
            w[k + 10] = -d2s(M * sin(6.0 * PI * (i + j) / n));
            w[k +  9] =  d2s(M * cos(6.0 * PI * (i    ) / n));
            w[k +  8] = -d2s(M * sin(6.0 * PI * (i    ) / n));

            w[k +  7] = -d2s(M * cos(4.0 * PI * (i + j) / n));
            w[k +  6] =  d2s(M * sin(4.0 * PI * (i + j) / n));
            w[k +  5] = -d2s(M * cos(4.0 * PI * (i    ) / n));
            w[k +  4] =  d2s(M * sin(4.0 * PI * (i    ) / n));

            w[k +  3] =  d2s(M * cos(2.0 * PI * (i + j) / n));
            w[k +  2] = -d2s(M * sin(2.0 * PI * (i + j) / n));
            w[k +  1] =  d2s(M * cos(2.0 * PI * (i    ) / n));
            w[k +  0] = -d2s(M * sin(2.0 * PI * (i    ) / n));

            k += 12;
        }
    }
    return k;
}
#endif
#endif //_GENTWIDDLE_FFT16x16_IMRE

#ifndef _GENTWIDDLE_FFTFLOAT
#define _GENTWIDDLE_FFTFLOAT
#ifdef _TMS320C6600 //C66x

void tw_gen_float(float *w, int n)
{
    int i, j, k;
    const double PI = 3.141592654;

    for (j = 1, k = 0; j <= n >> 2; j = j << 2)
    {
        for (i = 0; i < n >> 2; i += j)
        {
#ifdef _LITTLE_ENDIAN
            w[k]     = (float) sin (2 * PI * i / n);
            w[k + 1] = (float) cos (2 * PI * i / n);
            w[k + 2] = (float) sin (4 * PI * i / n);
            w[k + 3] = (float) cos (4 * PI * i / n);
            w[k + 4] = (float) sin (6 * PI * i / n);
            w[k + 5] = (float) cos (6 * PI * i / n);
#else
            w[k]     = (float)  cos (2 * PI * i / n);
            w[k + 1] = (float) -sin (2 * PI * i / n);
            w[k + 2] = (float)  cos (4 * PI * i / n);
            w[k + 3] = (float) -sin (4 * PI * i / n);
            w[k + 4] = (float)  cos (6 * PI * i / n);
            w[k + 5] = (float) -sin (6 * PI * i / n);
#endif
            k += 6;
        }
    }
}
#else  //C674x

void tw_gen_float (float *w, int n)
{
    int i, j, k;
    double x_t, y_t, theta1, theta2, theta3;
    const double PI = 3.141592654;

    for (j = 1, k = 0; j <= n >> 2; j = j << 2)
    {
        for (i = 0; i < n >> 2; i += j)
        {
            theta1 = 2 * PI * i / n;
            x_t = cos (theta1);
            y_t = sin (theta1);
            w[k] = (float) x_t;
            w[k + 1] = (float) y_t;

            theta2 = 4 * PI * i / n;
            x_t = cos (theta2);
            y_t = sin (theta2);
            w[k + 2] = (float) x_t;
            w[k + 3] = (float) y_t;

            theta3 = 6 * PI * i / n;
            x_t = cos (theta3);
            y_t = sin (theta3);
            w[k + 4] = (float) x_t;
            w[k + 5] = (float) y_t;
            k += 6;
        }
    }
}
#endif
#endif //_GENTWIDDLE_FFTFLOAT
