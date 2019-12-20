/**
 *   @file  cycle_measure.c
 *
 *   @brief
 *      Cycle measure utility functions for DSP
 *
 * Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/ 
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
 */

#include "cycle_measure.h"
#include <time.h>

#ifdef _TMS320C6X
#define L1PConfRegAddr 0x1840020
#define L1DConfRegAddr 0x1840040
#define L2ConfRegAddr  0x1840000
#define L1DWBINVRegAddr  0x1845044

void cache_setL1PSize(int cacheConf)
{
	*((int *)L1PConfRegAddr) &= 0xFFFFFFF8;
	*((int *)L1PConfRegAddr) |= cacheConf;
}

void cache_setL1DSize(int cacheConf)
{
	*((int *)L1DConfRegAddr) &= 0xFFFFFFF8;
	*((int *)L1DConfRegAddr) |= cacheConf;
}

void cache_setL2Size(int cacheConf)
{
	*((int *)L2ConfRegAddr) &= 0xFFFFFFF8;
	*((int *)L2ConfRegAddr) |= cacheConf;
}

void     cache_wbInvAllL2Wait()
{
	*((int *)L1DWBINVRegAddr) |= 0x1;
#ifdef _TMS320C6600
	asm(" MFENCE");
	asm(" NOP 4");
	asm(" NOP 4");
	asm(" NOP 4");
	asm(" NOP 4");
#endif
}


void startClock()
{
	TSCL = 0;
}


int ranClock()
{
   return (TSCL);
}

double getCPUTime()
{
   return 0;
}

void cache_setMar(unsigned int * baseAddr, unsigned int byteSize, unsigned int value)
{
	unsigned int maxAddr;
	unsigned int firstMar, lastMar;
	unsigned int marNum;
    volatile unsigned int *marBase = (unsigned int *)MAR;

    /* caculate the maximum address */
    maxAddr = (unsigned int)baseAddr + (byteSize - 1);

    /* range of MAR's that need to be modified */
    firstMar = (unsigned int)baseAddr >> 24;
    lastMar = (unsigned int)maxAddr >> 24;

    /* write back invalidate all cached entries */
    cache_wbInvAllL2Wait();

    /* loop through the number of MAR registers affecting the address range */
    for (marNum = firstMar; marNum <= lastMar; marNum++) {
        /* set the MAR registers to the specified value */
        marBase[marNum] = value;
    }
}

#else
#ifdef _WIN32
int ranClock()
{
   int   x1 ;
   x1 = (int) __rdtsc();

   return (x1);
}


void cache_setL1PSize(int cacheConf)
{
}

void cache_setL1DSize(int cacheConf)
{
}

void cache_setL2Size(int cacheConf)
{
}

void     cache_wbInvAllL2Wait()
{

}

void startClock()
{
}

void cache_setMar(unsigned int * baseAddr, unsigned int byteSize, unsigned int value)
{
}


/**
 * Returns the amount of CPU time used by the current process, in miliseconds
 */
double getCPUTime( )
{
	FILETIME createTime;
	FILETIME exitTime;
	FILETIME kernelTime;
	FILETIME userTime;
	if ( GetProcessTimes( GetCurrentProcess( ),
		&createTime, &exitTime, &kernelTime, &userTime ) != -1 )
	{
		SYSTEMTIME userSystemTime;
		if ( FileTimeToSystemTime( &userTime, &userSystemTime ) != -1 )
			return (double)userSystemTime.wSecond * 1000.0 +
				(double)userSystemTime.wMilliseconds;
	}
	return 0;
}


void touch(
           void * m,     /* Pointer to vector  */                          
           int  num_bytes /* Length of vector in bytes      */             
    )
{
	return;
}


#endif
#endif
