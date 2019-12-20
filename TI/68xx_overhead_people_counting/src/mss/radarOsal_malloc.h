/*! 
 *  \file   radarOsal_malloc.h
 *
 *  \brief   Header file for radarOsal_malloc.c for radar demo.
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

#ifndef _RADAROSALMALLOC_H
#define _RADAROSALMALLOC_H


#include <swpform.h>
#ifdef _TMS320C6X
#include "c6x.h"
#include <stdlib.h>
#include <string.h>
#endif

#ifdef _WIN32
#include <stdlib.h>
#endif

#ifdef _TMS320C6X
#if !(defined SOC_XWR16XX) && !(defined SOC_XWR18XX)
typedef enum
{
    RADARMEMOSAL_HEAPTYPE_DDR_CACHED = 0,		/**< Heap ID of heap in cached DDR*/
	RADARMEMOSAL_HEAPTYPE_LL2,					/**< Heap ID of heap in local L2 */
    RADARMEMOSAL_HEAPTYPE_MAXNUMHEAPS
    /**< max Heap type */
} radarMemOsal_HeapType;
#define RADARMEMOSAL_HEAPTYPE_LL1 RADARMEMOSAL_HEAPTYPE_LL2

#else
typedef enum
{
    RADARMEMOSAL_HEAPTYPE_DDR_CACHED = 0,		/**< Heap ID of heap in cached DDR*/
	RADARMEMOSAL_HEAPTYPE_LL2,					/**< Heap ID of heap in local L2 */
	RADARMEMOSAL_HEAPTYPE_LL1,					/**< Heap ID of heap in local L1 */
	RADARMEMOSAL_HEAPTYPE_HSRAM,				/**< Heap ID of heap in HSRAM */
    RADARMEMOSAL_HEAPTYPE_MAXNUMHEAPS
    /**< max Heap type */
} radarMemOsal_HeapType;
#endif
#else //_WIN32
#if !(defined SOC_XWR16XX) && !(defined SOC_XWR18XX)
typedef enum
{
    RADARMEMOSAL_HEAPTYPE_DDR_CACHED = 0,		/**< Heap ID of heap in cached DDR*/
	RADARMEMOSAL_HEAPTYPE_LL2,					/**< Heap ID of heap in local L2 */
    RADARMEMOSAL_HEAPTYPE_MAXNUMHEAPS
    /**< max Heap type */
} radarMemOsal_HeapType;
#define RADARMEMOSAL_HEAPTYPE_LL1 RADARMEMOSAL_HEAPTYPE_LL2
#else
typedef enum
{
    RADARMEMOSAL_HEAPTYPE_DDR_CACHED = 0,		/**< Heap ID of heap in cached DDR*/
	RADARMEMOSAL_HEAPTYPE_LL2,					/**< Heap ID of heap in local L2 */
	RADARMEMOSAL_HEAPTYPE_LL1,					/**< Heap ID of heap in local L1 */
	RADARMEMOSAL_HEAPTYPE_HSRAM,				/**< Heap ID of heap in HSRAM */
    RADARMEMOSAL_HEAPTYPE_MAXNUMHEAPS
    /**< max Heap type */
} radarMemOsal_HeapType;
#endif
#endif

#define RADARMEMOSAL_FAIL (1)
#define RADARMEMOSAL_PASS (0)

#if !(defined SOC_XWR16XX) && !(defined SOC_XWR18XX)
typedef struct {
    char   heapName[32]; /**< Name of heap */
    int8_t  *heapAddr;     /**< Physical base address of heap */
    uint32_t heapSize;      /**< Total size of heap in bytes */
    int8_t  *scratchAddr;     /**< Physical base address of scratch from this heap */
    uint32_t scratchSize;      /**< Total size of scratch in bytes */
} radarOsal_heapConfig;
typedef struct {
    char   heapName[32]; /**< Name of heap */
	radarMemOsal_HeapType   heapType; /**< type of heap */
    int8_t  *heapAddr;     /**< Physical base address of heap */
    uint32_t heapSize;      /**< Total size of heap in bytes */
    uint32_t heapAllocOffset;    /**< Heap alloc offset, only valid for L2 heap */
    int8_t  *scratchAddr;     /**< Physical base address of scratch from this heap */
    uint32_t  maxScratchSizeUsed;     /**< maximum size of the scratch memory requested */
    uint32_t scratchSize;      /**< Total size of scratch in bytes */
} radarOsal_heapObj;
#else
typedef struct {
	radarMemOsal_HeapType   heapType; /**< type of heap */
    int8_t  *heapAddr;     /**< Physical base address of heap */
    uint32_t heapSize;      /**< Total size of heap in bytes */
    int8_t  *scratchAddr;     /**< Physical base address of scratch from this heap */
    uint32_t scratchSize;      /**< Total size of scratch in bytes */
} radarOsal_heapConfig;
typedef struct {
	radarMemOsal_HeapType   heapType; /**< type of heap */
    int8_t  *heapAddr;     /**< Physical base address of heap */
    uint32_t heapSize;      /**< Total size of heap in bytes */
    uint32_t heapAllocOffset;    /**< Heap alloc offset, only valid for L2 heap */
    int8_t  *scratchAddr;     /**< Physical base address of scratch from this heap */
    uint32_t  maxScratchSizeUsed;     /**< maximum size of the scratch memory requested */
    uint32_t scratchSize;      /**< Total size of scratch in bytes */
} radarOsal_heapObj;
#endif


/*! 
   \fn     radarOsal_memInit
   \brief   OSAL function for heap memory structure initialization.
  
   \return    RADAROSAL_FAIL if heap init failed, RADAROSAL_PASS if heap init passed. 
   \pre       none
   \post      none
 */
extern int32_t radarOsal_memInit(radarOsal_heapConfig * config, uint8_t numHeap);

/*! 
   \fn     radarOsal_memDeInit
   \brief   OSAL function for heap memory structure de-initialization.
  
   \return    RADAROSAL_FAIL if heap deinit failed, RADAROSAL_PASS if heap deinit passed. 
   \pre       none
   \post      none
 */
extern int32_t radarOsal_memDeInit(void);

/*! 
   \fn     radarOsal_memAlloc
   \brief   OSAL function for memory allocation.

   \param[in]    memoryType
               input radarMemOsal heap type. Definition of the types depends on used platform/OS.
 
   \param[in]    scratchFlag
               Input flag to indicate whether request memory is a scratch that can be shared across modules. 1 for scratch memory request, and 0 otherwise.
 
   \param[in]    size
               Request memory size in number of bytes.

   \param[in]    alignment
               Request memory alignment in number of bytes. Alignment has to be power of 2. Alignment = 1 for no alignment requirement.

   \return    NULL if malloc failed, void pointer if malloc passed. 
   \pre       none
   \post      none
 */
extern void * radarOsal_memAlloc(uint8_t memoryType, uint8_t scratchFlag, uint32_t size, uint16_t alignment);

/*! 
   \fn     radarOsal_memFree
   \brief   OSAL function for memory free.
  
   \param[in]    ptr
               input poointer to be freed.
 
   \param[in]    size
               Size of the memory to be freed, in number of bytes .

   \return    none. 
   \pre       none
   \post      none
 */
extern void radarOsal_memFree(void *ptr, uint32_t size);

#endif //_RADAROSALMALLOC_H
