/*
 *   @file  dss_main.c
 *
 *   @brief
 *      Millimeter Wave Demo running on DSS
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2016 Texas Instruments, Inc.
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

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* BIOS/XDC Include Files. */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/family/c64p/Cache.h>
#include <ti/sysbios/family/c64p/Hwi.h>
#include <ti/sysbios/family/c64p/EventCombiner.h>
#include <ti/sysbios/utils/Load.h>


/* MMWSDK Include Files. */
#include <ti/drivers/soc/soc.h>
#include <ti/common/sys_common.h>
#include <ti/common/mmwave_sdk_version.h>
#include <ti/utils/cycleprofiler/cycle_profiler.h>

/* MMWAVE Demo Include Files */
#include "dss_mmw.h"
#include "dss_data_path.h"
//#include "mmw_messages.h"
#include <mmw_messages.h>

/* C674x mathlib */
/* Suppress the mathlib.h warnings
 *  #48-D: incompatible redefinition of macro "TRUE"
 *  #48-D: incompatible redefinition of macro "FALSE" 
 */
#pragma diag_push
#pragma diag_suppress 48
#include <ti/mathlib/mathlib.h>
#pragma diag_pop

/* Related to linker copy table for copying from L3 to L1PSRAM for example */
#include <cpy_tbl.h>

/**************************************************************************
 *************************** MmwDemo External DSS Functions ******************
 **************************************************************************/


/**************************************************************************
 *************************** Global Definitions ********************************
 **************************************************************************/
/**
 * @brief
 *  DSS stores demo output and DSS to MSS ISR information (for fast exception 
 *  signalling) in HSRAM.
 */
/*!   */
typedef struct MmwDemo_HSRAM_t_ {
#define MMW_DATAPATH_DET_PAYLOAD_SIZE (SOC_HSRAM_SIZE -  sizeof(uint8_t))
    /*! @brief data path processing/detection related message payloads, these
               messages are signalled through DSS to MSS mailbox */ 
    uint8_t  dataPathDetectionPayload[MMW_DATAPATH_DET_PAYLOAD_SIZE];

    /*! @brief Information relayed through DSS triggering software interrupt to
               MSS. It stores one of the exception IDs @ref DSS_TO_MSS_EXCEPTION_IDS */
    uint8_t  dss2MssIsrInfo;
} MmwDemo_HSRAM_t;

#pragma DATA_SECTION(gHSRAM, ".demoSharedMem");
#pragma DATA_ALIGN(gHSRAM, 4);
MmwDemo_HSRAM_t gHSRAM;


/*! @brief Flag to enable/disable two peak detection in azimuth for same range and velocity */
#define MMWDEMO_AZIMUTH_TWO_PEAK_DETECTION_ENABLE 1

/*! @brief Threshold for two peak detection in azimuth for same range and velocity,
 *         if 2nd peak heigth > first peak height * this scale then declare
 *         2nd peak as detected. The peaks are in @ref MmwDemo_DSS_DataPathObj::azimuthMagSqr */
#define MMWDEMO_AZIMUTH_TWO_PEAK_THRESHOLD_SCALE  (0.5)


#define MMWDEMO_SPEED_OF_LIGHT_IN_METERS_PER_SEC (3.0e8)

//#define DBG

/**
 * @brief
 *  Global Variable for tracking information required by the mmw Demo
 */
MmwDemo_DSS_MCB    gMmwDssMCB;

volatile cycleLog_t gCycleLog;

//#pragma DATA_SECTION(DOA_2D_storage, ".angle")
cmplx32ReIm_t DOA_2D_storage[MMW_NUM_ANGLE_BINS][MMW_NUM_ANGLE_BINS];

radarProcessOutputToTracker outputFrame100;

/* copy table related */
extern far COPY_TABLE _MmwDemo_fastCode_L1PSRAM_copy_table;

/**************************************************************************
 ************************* MmwDemo Functions Prototype  **********************
 **************************************************************************/

/* Copy table related */
static void MmwDemo_edmaBlockCopy(EDMA_Handle handle, uint32_t loadAddr, 
                                  uint32_t runAddr, uint16_t size);
static void MmwDemo_copyTable(EDMA_Handle handle, COPY_TABLE *tp);

/* Internal DataPath Functions */
int32_t MmwDemo_dssDataPathInit(void);
static int32_t MmwDemo_dssDataPathConfig(void);
static int32_t MmwDemo_dssDataPathStart(bool doRFStart);
static int32_t MmwDemo_dssDataPathStop(void);
static int32_t MmwDemo_dssDataPathProcessEvents(UInt event);
static int32_t MmwDemo_dssDataPathReconfig(MmwDemo_DSS_DataPathObj *obj);
static void MmwDemo_measurementResultOutput(MmwDemo_DSS_DataPathObj *obj);

/* Internal MMWave Call back Functions */
static int32_t MmwDemo_dssMmwaveEventCallbackFxn(uint16_t msgId, uint16_t sbId, 
                                                 uint16_t sbLen, uint8_t *payload);
static void MmwDemo_dssMmwaveConfigCallbackFxn(MMWave_CtrlCfg* ptrCtrlCfg);
static void MmwDemo_dssMmwaveStartCallbackFxn(MMWave_CalibrationCfg* ptrCalibrationCfg);
static void MmwDemo_dssMmwaveStopCallbackFxn(void);
static void MmwDemo_dssMmwaveOpenCallbackFxn(MMWave_OpenCfg* ptrOpenCfg);
static void MmwDemo_dssMmwaveCloseCallbackFxn(void);

/* Internal Interrupt handler */
static void MmwDemo_dssChirpIntHandler(uintptr_t arg);
static void MmwDemo_dssFrameStartIntHandler(uintptr_t arg);

/* Internal mmwDemo Tasks running on DSS */
static void MmwDemo_dssInitTask(UArg arg0, UArg arg1);
static void MmwDemo_dssDataPathTask(UArg arg0, UArg arg1);
static void MmwDemo_dssMMWaveCtrlTask(UArg arg0, UArg arg1);

/* Internal mmwDemo function to trigger DSS to MSS ISR for urgent exception signalling */
static void MmwDemo_triggerDss2MssISR(uint8_t dss2MssIsrInfo);

/* external sleep function when in idle (used in .cfg file) */
void MmwDemo_sleep(void);

static int32_t MmwDemo_dssSendProcessOutputToMSS
(
    uint8_t           *ptrHsmBuffer,
    uint32_t           outputBufSize,
    MmwDemo_DSS_DataPathObj   *obj
);
void MmwDemo_dssDataPathOutputLogging(    MmwDemo_DSS_DataPathObj   * dataPathObj);

/**************************************************************************
 *************************** MmwDemo DSS Functions **************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      Interrupt handler callback for chirp available. It runs in the ISR context.
 *
 *  @retval
 *      Not Applicable.
 */
 #ifdef DBG
 #define NUM_CHIRP_TIME_STAMPS 128
 uint32_t gChirpTimeStamp[NUM_CHIRP_TIME_STAMPS];
 #endif

static void MmwDemo_dssChirpIntHandler(uintptr_t arg)
{
    MmwDemo_DSS_DataPathObj * dpObj = &gMmwDssMCB.dataPathObj[gMmwDssMCB.subFrameIndx];

    if((gMmwDssMCB.state == MmwDemo_DSS_STATE_STOPPED) ||
       (gMmwDssMCB.dataPathContext.interFrameProcToken <= 0))
    {
        gMmwDssMCB.stats.chirpIntSkipCounter++;
        return;
    }

#ifdef DBG
    if (dpObj->chirpCount < NUM_CHIRP_TIME_STAMPS)
    {
        gChirpTimeStamp[dpObj->chirpCount] =
            Cycleprofiler_getTimeStamp();
    }
#endif

    if (dpObj->chirpCount == 0)
    {
        MmwDemo_DSS_DataPathObj * dpObjPrev = dpObj;
        uint8_t subFrameIndxPrev;
        if (gMmwDssMCB.numSubFrames > 1)
        {
            if (gMmwDssMCB.subFrameIndx == 0)
            {
                subFrameIndxPrev = gMmwDssMCB.numSubFrames - 1;
            }
            else
            {
                subFrameIndxPrev = gMmwDssMCB.subFrameIndx - 1;
            }
            dpObjPrev = &gMmwDssMCB.dataPathObj[subFrameIndxPrev];
        }

        /* Note: this is valid after the first frame */
        dpObjPrev->timingInfo.interFrameProcessingEndMargin =
            Cycleprofiler_getTimeStamp() - dpObjPrev->timingInfo.interFrameProcessingEndTime -
            dpObjPrev->timingInfo.subFrameSwitchingCycles;
    }
    else if (dpObj->chirpCount == dpObj->numChirpsPerChirpEvent)
    {
        dpObj->timingInfo.chirpProcessingEndMarginMin =
            Cycleprofiler_getTimeStamp() - dpObj->timingInfo.chirpProcessingEndTime;
        dpObj->timingInfo.chirpProcessingEndMarginMax =
            dpObj->timingInfo.chirpProcessingEndMarginMin;
    }
    else
    {
        uint32_t margin = Cycleprofiler_getTimeStamp() - dpObj->timingInfo.chirpProcessingEndTime;
        if (margin > dpObj->timingInfo.chirpProcessingEndMarginMax)
        {
            dpObj->timingInfo.chirpProcessingEndMarginMax = margin;
        }
        if (margin < dpObj->timingInfo.chirpProcessingEndMarginMin)
        {
            dpObj->timingInfo.chirpProcessingEndMarginMin = margin;
        }
    }
    /* Increment interrupt counter for debugging purpose */
    gMmwDssMCB.stats.chirpIntCounter++;

    /* Check if previous chirp processing has completed */
    if (gMmwDssMCB.dataPathContext.chirpProcToken != 0)
    {
        MmwDemo_triggerDss2MssISR(MMWDEMO_DSS2MSS_CHIRP_PROC_DEADLINE_MISS_EXCEPTION);
        //DebugP_assert(0);
    }

    gMmwDssMCB.dataPathContext.chirpProcToken++;

    /* Post event to notify chirp available interrupt */
    Event_post(gMmwDssMCB.eventHandle, MMWDEMO_CHIRP_EVT);
}

/**
 *  @b Description
 *  @n
 *      Interrupt handler callback for frame start ISR.
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_dssFrameStartIntHandler(uintptr_t arg)
{
    if(gMmwDssMCB.state == MmwDemo_DSS_STATE_STOPPED)
    {
        gMmwDssMCB.stats.frameIntSkipCounter++;
        return;
    }

    /* Check if previous chirp processing has completed */
    if (gMmwDssMCB.dataPathContext.interFrameProcToken != 0)
    {
        MmwDemo_triggerDss2MssISR(MMWDEMO_DSS2MSS_FRAME_PROC_DEADLINE_MISS_EXCEPTION);
        //DebugP_assert(0);
    }

    gMmwDssMCB.dataPathContext.interFrameProcToken++;

    /* Increment interrupt counter for debugging purpose */    
    if (gMmwDssMCB.subFrameIndx == 0)
    {
        gMmwDssMCB.stats.frameStartIntCounter++;
    }

    /* Post event to notify frame start interrupt */
    Event_post(gMmwDssMCB.eventHandle, MMWDEMO_FRAMESTART_EVT);
}

/**
 *  @b Description
 *  @n
 *      Registered event callback function on DSS which is invoked by MMWAVE library when an event from the
 *      BSS is received.
 *
 *  @param[in]  msgId
 *      Message Identifier
 *  @param[in]  sbId
 *      Subblock identifier
 *  @param[in]  sbLen
 *      Length of the subblock
 *  @param[in]  payload
 *      Pointer to the payload buffer
 *
 *  @retval
 *      Always return 0. [Pass the event to the remote domain]
 */
static int32_t MmwDemo_dssMmwaveEventCallbackFxn(uint16_t msgId, uint16_t sbId, uint16_t sbLen, uint8_t *payload)
{
    uint16_t asyncSB = RL_GET_SBID_FROM_UNIQ_SBID(sbId);

    /* Debug Message: */
    /*System_printf ("Debug: MMWDemoDSS received BSS Event MsgId: %d [Sub Block Id: %d Sub Block Length: %d]\n",
                    msgId, sbId, sbLen); */

    /* Process the received message: */
    switch (msgId)
    {
        case RL_RF_ASYNC_EVENT_MSG:
        {
            /* Received Asychronous Message: */
            switch (asyncSB)
            {
                case RL_RF_AE_CPUFAULT_SB:
                {
                    /* BSS fault */
                    MmwDemo_dssAssert(0);
                    break;
                }
                case RL_RF_AE_ESMFAULT_SB:
                {
                    /* BSS fault */
                    MmwDemo_dssAssert(0);
                    break;
                }
                case RL_RF_AE_INITCALIBSTATUS_SB:
                {
                    /* This event should be handled by mmwave internally, ignore the event here */
                    break;
                }

                case RL_RF_AE_FRAME_TRIGGER_RDY_SB:
                {
                    /* Post event to datapath task notify BSS events */
                    Event_post(gMmwDssMCB.eventHandle, MMWDEMO_BSS_FRAME_TRIGGER_READY_EVT);

                    break;
                }
                case RL_RF_AE_MON_TIMING_FAIL_REPORT_SB:
                {
                    /* Increment the statistics to reports that the calibration failed */
                    gMmwDssMCB.stats.numFailedTimingReports++;
                    break;
                }
                case RL_RF_AE_RUN_TIME_CALIB_REPORT_SB:
                {
                    /* Increment the statistics to indicate that a calibration report was received */
                    gMmwDssMCB.stats.numCalibrationReports++;
                    break;
                }
                case RL_RF_AE_FRAME_END_SB:
                {
                    /*Received Frame Stop async event from BSS. Post event to datapath task.*/
                    Event_post(gMmwDssMCB.eventHandle, MMWDEMO_BSS_STOP_COMPLETE_EVT);
                    break;
                }
                default:
                {
                    System_printf ("Error: Asynchronous Event SB Id %d not handled\n", asyncSB);
                    break;
                }
            }
            break;
        }
        default:
        {
            System_printf ("Error: Asynchronous message %d is NOT handled\n", msgId);
            break;
        }
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Registered config callback function on DSS which is invoked by MMWAVE library when the remote side
 *  has finished configure mmWaveLink and BSS. The configuration need to be saved on DSS and used for DataPath.
 *
 *  @param[in]  ptrCtrlCfg
 *      Pointer to the control configuration
 *
 *  @retval
 *      Not applicable
 */
static void MmwDemo_dssMmwaveConfigCallbackFxn(MMWave_CtrlCfg* ptrCtrlCfg)
{
    /* Save the configuration */
    memcpy((void *)(&gMmwDssMCB.cfg.ctrlCfg), (void *)ptrCtrlCfg, sizeof(MMWave_CtrlCfg));

    gMmwDssMCB.stats.configEvt++;

    /* Post event to notify configuration is done */
    Event_post(gMmwDssMCB.eventHandle, MMWDEMO_CONFIG_EVT);

    return;
}

/**
 *  @b Description
 *  @n
 *      Registered open callback function which is invoked when the mmWave module
 *      has been opened on the MSS
 *
 *  @param[in]  ptrOpenCfg
 *      Pointer to the open configuration
 *
 *  @retval
 *      Not applicable
 */
static void MmwDemo_dssMmwaveOpenCallbackFxn(MMWave_OpenCfg* ptrOpenCfg)
{
    /* Save the configuration */
    memcpy((void *)(&gMmwDssMCB.cfg.openCfg), (void *)ptrOpenCfg, sizeof(MMWave_OpenCfg));
    gMmwDssMCB.stats.openEvt++;
    return;
}

/**
 *  @b Description
 *  @n
 *      Registered close callback function which is invoked when the mmWave module
 *      has been close on the MSS
 *
 *  @retval
 *      Not applicable
 */
static void MmwDemo_dssMmwaveCloseCallbackFxn(void)
{
    gMmwDssMCB.stats.closeEvt++;
    return;
}

/**
 *  @b Description
 *  @n
 *      Registered Start callback function on DSS which is invoked by MMWAVE library
 *    when the remote side has started mmWaveLink and BSS. This Callback function passes
 *    the event to DataPath task.
 *
 *  @param[in]  ptrCalibrationCfg
 *      Pointer to the calibration configuration
 *
 *  @retval
 *      Not applicable
 */
static void MmwDemo_dssMmwaveStartCallbackFxn(MMWave_CalibrationCfg* ptrCalibrationCfg)
{
    gMmwDssMCB.stats.startEvt++;

    /* Post event to start is done */
    Event_post(gMmwDssMCB.eventHandle, MMWDEMO_START_EVT);
}

/**
 *  @b Description
 *  @n
 *      Registered Start callback function on DSS which is invoked by MMWAVE library
 *    when the remote side has stop mmWaveLink and BSS. This Callback function passes
 *    the event to DataPath task.
 *
 *  @retval
 *      Not applicable
 */
static void MmwDemo_dssMmwaveStopCallbackFxn(void)
{
    gMmwDssMCB.stats.stopEvt++;

}

/**
 *  @b Description
 *  @n
 *      Function to send a message to peer through Mailbox virtural channel
 *
 *  @param[in]  message
 *      Pointer to the Captuere demo message.
 *
 *  @retval
 *      Success    - 0
 *      Fail       < -1
 */
static int32_t MmwDemo_mboxWrite(MmwDemo_message    *message)
{
    int32_t                  retVal = -1;

    retVal = Mailbox_write (gMmwDssMCB.peerMailbox, (uint8_t*)message, sizeof(MmwDemo_message));
    if (retVal == sizeof(MmwDemo_message))
    {
        retVal = 0;
    }
    return retVal;
}

//static int32_t MmwDemo_mboxWrite2(MmwDemo_message_2    *message)
//{
//    int32_t                  retVal = -1;
//
//    retVal = Mailbox_write (gMmwDssMCB.peerMailbox, (uint8_t*)message, sizeof(MmwDemo_message_2));
//    if (retVal == sizeof(MmwDemo_message))
//    {
//        retVal = 0;
//    }
//    return retVal;
//}

static void MmwDemo_cfgUpdate(void *srcPtr, uint32_t offset, uint32_t size, int8_t subFrameNum)
{    
    /* if subFrameNum undefined, broadcast to all sub-frames */
    if(subFrameNum == MMWDEMO_SUBFRAME_NUM_FRAME_LEVEL_CONFIG)
    {
        uint8_t  indx;
        for(indx = 0; indx < RL_MAX_SUBFRAMES; indx++)
        {
            memcpy((void *)((uint32_t) &gMmwDssMCB.cliCfg[indx] + offset), srcPtr, size);
        }
        
    }
    else
    {
        /* Apply configuration to specific subframe (or to position zero for the legacy case
           where there is no advanced frame config) */
        memcpy((void *)((uint32_t) &gMmwDssMCB.cliCfg[subFrameNum] + offset), srcPtr, size);
    }
}

/**
 *  @b Description
 *  @n
 *      Function that acts upon receiving message that BSS has stopped
 *      successfully.
 *
 *  @retval
 *      Not applicable
 */
static void MmwDemo_bssStopDone(void)
{    
    /*Change state to stop_pending*/
    gMmwDssMCB.state = MmwDemo_DSS_STATE_STOP_PENDING;    
    
    if(gMmwDssMCB.dataPathContext.interFrameProcToken == 0)
    {
        /*BSS stop message received after inter-frame processing
          is completed (including sending out UART data).
          */
         Event_post(gMmwDssMCB.eventHandle, MMWDEMO_STOP_COMPLETE_EVT);       
    }
    else
    {
        /*BSS stop message received during inter-frame processing.
          Switch to stop pending state and stop once inter frame
          processing done. Nothing to be done here.
          */
        
    }

}

/**
 *  @b Description
 *  @n
 *      The Task is used to handle  the mmw demo messages received from Mailbox virtual channel.
 *
 *  @param[in]  arg0
 *      arg0 of the Task. Not used
 *  @param[in]  arg1
 *      arg1 of the Task. Not used
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_mboxReadTask(UArg arg0, UArg arg1)
{
    MmwDemo_message      message;
    int32_t              retVal = 0;
    int8_t               subFrameNum;
    uint32_t             log2NumAvgChirpsTemp;

    /* wait for new message and process all the messsages received from the peer */
    while(1)
    {
        Semaphore_pend(gMmwDssMCB.mboxSemHandle, BIOS_WAIT_FOREVER);

        /* Read the message from the peer mailbox: We are not trying to protect the read
         * from the peer mailbox because this is only being invoked from a single thread */
        retVal = Mailbox_read(gMmwDssMCB.peerMailbox, (uint8_t*)&message, sizeof(MmwDemo_message));
        if (retVal < 0)
        {
            /* Error: Unable to read the message. Setup the error code and return values */
            System_printf ("Error: Mailbox read failed [Error code %d]\n", retVal);
        }
        else if (retVal == 0)
        {
            /* We are done: There are no messages available from the peer execution domain. */
            continue;
        }
        else
        {
            /* Flush out the contents of the mailbox to indicate that we are done with the message. This will
             * allow us to receive another message in the mailbox while we process the received message. */
            Mailbox_readFlush (gMmwDssMCB.peerMailbox);

            /* Process the received message: */
            subFrameNum = message.subFrameNum;                     

            switch (message.type)
            {
                case MMWDEMO_MSS2DSS_GUIMON_CFG:
                {
                    /* Save guimon configuration */
                    MmwDemo_cfgUpdate((void *)&message.body.guiMonSel, 
                                         offsetof(MmwDemo_CliCfg_t, guiMonSel),
                                         sizeof(MmwDemo_GuiMonSel), subFrameNum);
                    break;
                }
                case MMWDEMO_MSS2DSS_CFAR_RANGE_CFG:
                {
                    MmwDemo_cfgUpdate((void *)&message.body.cfarCfg, 
                                         offsetof(MmwDemo_CliCfg_t, cfarCfgRange),
                                         sizeof(MmwDemo_CfarCfg), subFrameNum);
                    break;
                }
                case MMWDEMO_MSS2DSS_CFAR_DOPPLER_CFG:
                {
                    /* Save cfarDoppler configuration */
                    MmwDemo_cfgUpdate((void *)&message.body.cfarCfg, 
                                         offsetof(MmwDemo_CliCfg_t, cfarCfgDoppler),
                                         sizeof(MmwDemo_CfarCfg), subFrameNum);
                    
                    break;
                }
                case MMWDEMO_MSS2DSS_PEAK_GROUPING_CFG:
                {
                    /* Save Peak grouping configuration */
                    MmwDemo_cfgUpdate((void *)&message.body.peakGroupingCfg, 
                                         offsetof(MmwDemo_CliCfg_t, peakGroupingCfg),
                                         sizeof(MmwDemo_PeakGroupingCfg), subFrameNum);       
                    break;
                }
                case MMWDEMO_MSS2DSS_MULTI_OBJ_BEAM_FORM:
                {
                    /* Save multi object beam forming configuration */
                    MmwDemo_cfgUpdate((void*)&message.body.multiObjBeamFormingCfg,
                                      offsetof(MmwDemo_CliCfg_t, multiObjBeamFormingCfg),
                                      sizeof(MmwDemo_MultiObjBeamFormingCfg),
                                      subFrameNum);
                    break;
                }
                case MMWDEMO_MSS2DSS_CALIB_DC_RANGE_SIG:
                {
                    /* Save  of DC range antenna signature configuration */
                    MmwDemo_cfgUpdate((void *)&message.body.calibDcRangeSigCfg, 
                                         offsetof(MmwDemo_CliCfg_t, calibDcRangeSigCfg),
                                         sizeof(MmwDemo_CalibDcRangeSigCfg), subFrameNum);
                                      
                    log2NumAvgChirpsTemp = MmwDemo_floorLog2(message.body.calibDcRangeSigCfg.numAvgChirps);
                    /* Save log2NumAvgChirps  */
                    if (subFrameNum == MMWDEMO_SUBFRAME_NUM_FRAME_LEVEL_CONFIG)
                    {
                        uint8_t indx;
                        for(indx = 0; indx < RL_MAX_SUBFRAMES; indx++)
                        {
                            gMmwDssMCB.dataPathObj[indx].dcRangeSigCalibCntr = 0;
                            gMmwDssMCB.dataPathObj[indx].log2NumAvgChirps = log2NumAvgChirpsTemp;
                        }
                    }
                    else
                    {
                        gMmwDssMCB.dataPathObj[subFrameNum].dcRangeSigCalibCntr = 0;
                        gMmwDssMCB.dataPathObj[subFrameNum].log2NumAvgChirps = log2NumAvgChirpsTemp;
                    }
                    break;
                }
                case MMWDEMO_MSS2DSS_EXTENDED_MAX_VELOCITY:
                {
                    /* Save  of extended velocity configuration */
                    MmwDemo_cfgUpdate((void *)&message.body.extendedMaxVelocityCfg, 
                                         offsetof(MmwDemo_CliCfg_t, extendedMaxVelocityCfg),
                                         sizeof(MmwDemo_ExtendedMaxVelocityCfg), subFrameNum);
                    break;
                }
                case MMWDEMO_MSS2DSS_CLUTTER_REMOVAL:
                {
                    /* Save  clutter removal configuration */
                    MmwDemo_cfgUpdate((void *)&message.body.clutterRemovalCfg, 
                                         offsetof(MmwDemo_CliCfg_t, clutterRemovalCfg),
                                         sizeof(MmwDemo_ClutterRemovalCfg), subFrameNum);
                    break;
                }
                case MMWDEMO_MSS2DSS_ADCBUFCFG:
                {
                    /* Save ADCBUF configuration */ 
                    MmwDemo_cfgUpdate((void *)&message.body.adcBufCfg, 
                                         offsetof(MmwDemo_CliCfg_t, adcBufCfg),
                                         sizeof(MmwDemo_ADCBufCfg), subFrameNum);
                    break;
                }
                case MMWDEMO_MSS2DSS_COMP_RANGE_BIAS_AND_RX_CHAN_PHASE:
                {
                    /* Save range bias and Rx channels phase compensation */
                    memcpy((void *) &gMmwDssMCB.cliCommonCfg.compRxChanCfg, (void *)&message.body.compRxChanCfg, sizeof(MmwDemo_compRxChannelBiasCfg_t));
                    break;
                }
                case MMWDEMO_MSS2DSS_MEASURE_RANGE_BIAS_AND_RX_CHAN_PHASE:
                {
                    /* Save range bias and Rx channels phase compensation */
                    memcpy((void *) &gMmwDssMCB.cliCommonCfg.measureRxChanCfg, (void *)&message.body.measureRxChanCfg, sizeof(MmwDemo_measureRxChannelBiasCfg_t));
                    break;
                }
                case MMWDEMO_MSS2DSS_DETOBJ_SHIPPED:
                {
                    MmwDemo_DSS_DataPathObj *dataPathCurrent, *dataPathNext;

                    dataPathCurrent = &gMmwDssMCB.dataPathObj[gMmwDssMCB.subFrameIndx];
                    dataPathCurrent->timingInfo.transmitOutputCycles =
                        Cycleprofiler_getTimeStamp() - dataPathCurrent->timingInfo.interFrameProcessingEndTime;

                    gMmwDssMCB.subFrameIndx++;
                    if (gMmwDssMCB.subFrameIndx == gMmwDssMCB.numSubFrames)
                    {
                        gMmwDssMCB.subFrameIndx = 0;
                    }

                    dataPathNext = &gMmwDssMCB.dataPathObj[gMmwDssMCB.subFrameIndx];
                    
                    /* execute subframe switching related functions */
                    if (gMmwDssMCB.numSubFrames > 1)
                    {
                        volatile uint32_t startTime;
                        startTime = Cycleprofiler_getTimeStamp();

                        MmwDemo_dssDataPathReconfig(dataPathNext);

                        dataPathCurrent->timingInfo.subFrameSwitchingCycles = Cycleprofiler_getTimeStamp() -
                                                                           startTime;
                    }
                    else
                    {
                        dataPathCurrent->timingInfo.subFrameSwitchingCycles = 0;
                    }

                    MmwDemo_checkDynamicConfigErrors(dataPathNext);

                    gMmwDssMCB.dataPathContext.interFrameProcToken--;

                    gMmwDssMCB.loggingBufferAvailable = 1;

                    /* Post event to complete stop operation, if pending */
                    if ((gMmwDssMCB.state == MmwDemo_DSS_STATE_STOP_PENDING) && (gMmwDssMCB.subFrameIndx == 0))
                    {
                        Event_post(gMmwDssMCB.eventHandle, MMWDEMO_STOP_COMPLETE_EVT);
                    }
                    break;
                }
                case MMWDEMO_MSS2DSS_SET_DATALOGGER:
                {
                    gMmwDssMCB.cfg.dataLogger = message.body.dataLogger;
                    break;
                }
                case MMWDEMO_MSS2DSS_SNR_THRESHOLD:
                {
                    gMmwDssMCB.cliCfg[0].snrThresh.range = message.body.thresh.range;
                    gMmwDssMCB.cliCfg[0].snrThresh.threshold = message.body.thresh.threshold;
                    break;
                }
                default:
                {
                    /* Message not support */
                    System_printf ("Error: unsupport Mailbox message id=%d\n", message.type);
                    MmwDemo_dssAssert(0);
                    break;
                }
            }
        }
    }
}

/**
 *  @b Description
 *  @n
 *      This function is a callback funciton that invoked when a message is received from the peer.
 *
 *  @param[in]  handle
 *      Handle to the Mailbox on which data was received
 *  @param[in]  peer
 *      Peer from which data was received

 *  @retval
 *      Not Applicable.
 */
void MmwDemo_mboxCallback
(
    Mbox_Handle  handle,
    Mailbox_Type    peer
)
{
    /* Message has been received from the peer endpoint. Wakeup the mmWave thread to process
     * the received message. */
    Semaphore_post (gMmwDssMCB.mboxSemHandle);
}

/**
 *  @b Description
 *  @n
 *      Function to send detected objects to MSS logger.
 *
 *  @param[in]  ptrHsmBuffer
 *      Pointer to the output buffer
 *  @param[in]  outputBufSize
 *      Size of the output buffer
 *  @param[in]  obj
 *      Handle to the Data Path Object
 *
 *  @retval
 *      =0    Success
 *      <0    Failed
 */

int32_t MmwDemo_dssSendProcessOutputToMSS
(
    uint8_t           *ptrHsmBuffer,
    uint32_t           outputBufSize,
    MmwDemo_DSS_DataPathObj   *obj
)
{
    int32_t             retVal = 0;
    MmwDemo_message     message;

    static uint32_t frameCount = 0;
    uint32_t infoSize;
    uint32_t totalSize;
    MmwDemo_detOutputHdr           *ptrDetOutputHdr = (MmwDemo_detOutputHdr *)&(obj->outputDataToArm.outputHeader);
    radarProcessOutputToTracker *outputObjBuff = (radarProcessOutputToTracker *)&(obj->outputDataToArm.outputToTracker);

    /* Clear message to MSS */
    /*Debug*/
//    int oi = 0;
//    if (frameCount == 100) {
//        for (oi = 0; oi < obj->numDetObj; oi++) {
//            outputFrame100.range[oi] = obj->outputDataToArm.outputToTracker.range[oi];
//            outputFrame100.angle[oi] = obj->outputDataToArm.outputToTracker.angle[oi];
//            outputFrame100.elev[oi] = obj->outputDataToArm.outputToTracker.elev[oi];
//            outputFrame100.doppler[oi] = obj->outputDataToArm.outputToTracker.doppler[oi];
//            outputFrame100.snr[oi] = obj->outputDataToArm.outputToTracker.snr[oi];
//        }
//    }

    /* Validate input params */
    if(ptrHsmBuffer == NULL)
    {
        retVal = -1;
        goto Exit;
    }
    ptrDetOutputHdr->magicWord[0] = 0x0102;
    ptrDetOutputHdr->magicWord[1] = 0x0304;
    ptrDetOutputHdr->magicWord[2] = 0x0506;
    ptrDetOutputHdr->magicWord[3] = 0x0708;

    ptrDetOutputHdr->chirpProcessingMarginInUsec = 500;
    ptrDetOutputHdr->frameProcessingMarginInUsec = 500;


    ptrDetOutputHdr->numDetectedObj = obj->numDetObj;
    outputObjBuff->object_count = obj->numDetObj;
    ptrDetOutputHdr->frameNumber = frameCount;
    frameCount += 1;
    infoSize    =   sizeof(radarProcessOutputToTracker);
    totalSize   =   sizeof(MmwDemo_detOutputHdr) + infoSize;

    memcpy(ptrHsmBuffer, ptrDetOutputHdr, totalSize);
    /* Clear message to MSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));
    message.type = MMWDEMO_DSS2MSS_DETOBJ_READY;
    message.body.detObj.detObjOutAddress = (uint32_t)ptrHsmBuffer;
    message.body.detObj.detObjOutsize = totalSize;

    if (MmwDemo_mboxWrite(&message) != 0)
    {
        retVal = -1;
    }

#ifdef OOB
    MmwDemo_GuiMonSel   *pGuiMonSel;
    uint32_t            tlvIdx = 0;
    uint8_t i = 0;
    uint8_t             *ptrCurrBuffer;
    uint32_t            totalHsmSize = 0;
    uint32_t            totalPacketLen = sizeof(MmwDemo_output_message_header);
    uint32_t            itemPayloadLen;

    /* Get Gui Monitor configuration */
   pGuiMonSel = &obj->cliCfg->guiMonSel;

    memset((void *)&message, 0, sizeof(MmwDemo_message));
    message.type = MMWDEMO_DSS2MSS_DETOBJ_READY;
    /* Header: */
    message.body.detObj.header.platform = 0xA1642;
    message.body.detObj.header.magicWord[0] = 0x0102;
    message.body.detObj.header.magicWord[1] = 0x0304;
    message.body.detObj.header.magicWord[2] = 0x0506;
    message.body.detObj.header.magicWord[3] = 0x0708;
    message.body.detObj.header.numDetectedObj = obj->numDetObj;
    message.body.detObj.header.version =    MMWAVE_SDK_VERSION_BUILD |   //DEBUG_VERSION
                                            (MMWAVE_SDK_VERSION_BUGFIX << 8) |
                                            (MMWAVE_SDK_VERSION_MINOR << 16) |
                                            (MMWAVE_SDK_VERSION_MAJOR << 24);

    /* Set pointer to HSM buffer */
    ptrCurrBuffer = ptrHsmBuffer;

    /* Put detected Objects in HSM buffer: sizeof(MmwDemo_objOut_t) * numDetObj  */
    if ((pGuiMonSel->detectedObjects == 1) && (obj->numDetObj > 0))
    {
        /* Add objects descriptor */
        MmwDemo_output_message_dataObjDescr descr;
        descr.numDetetedObj = obj->numDetObj;
        descr.xyzQFormat = obj->xyzOutputQFormat;
        itemPayloadLen = sizeof(MmwDemo_output_message_dataObjDescr);
        totalHsmSize += itemPayloadLen;
        if(totalHsmSize > outputBufSize)
        {
            retVal = -1;
            goto Exit;
        }
        memcpy(ptrCurrBuffer, (void *)&descr, itemPayloadLen);

        /* Add array of objects */
        itemPayloadLen = sizeof(MmwDemo_detectedObj) * obj->numDetObj;
        totalHsmSize += itemPayloadLen;
        if(totalHsmSize > outputBufSize)
        {
            retVal = -1;
            goto Exit;
        }
        memcpy(&ptrCurrBuffer[sizeof(MmwDemo_output_message_dataObjDescr)], (void *)obj->detObj2D, itemPayloadLen);

        message.body.detObj.tlv[tlvIdx].length = itemPayloadLen + sizeof(MmwDemo_output_message_dataObjDescr);
        message.body.detObj.tlv[tlvIdx].type = MMWDEMO_OUTPUT_MSG_DETECTED_POINTS;
        message.body.detObj.tlv[tlvIdx].address = (uint32_t) ptrCurrBuffer;
        tlvIdx++;

        /* Incrementing pointer to HSM buffer */
        ptrCurrBuffer += itemPayloadLen + sizeof(MmwDemo_output_message_dataObjDescr);
        totalPacketLen += sizeof(MmwDemo_output_message_tl) + itemPayloadLen + sizeof(MmwDemo_output_message_dataObjDescr);
    }

    /* Sending range profile:  2bytes * numRangeBins */
    if (pGuiMonSel->logMagRange == 1)
    {
        itemPayloadLen = sizeof(uint16_t) * obj->numRangeBins;
        totalHsmSize += itemPayloadLen;
        if(totalHsmSize > outputBufSize)
        {
            retVal = -1;
            goto Exit;
        }

        uint16_t *ptrMatrix = (uint16_t *)ptrCurrBuffer;
        for(i = 0; i < obj->numRangeBins; i++)
        {
            ptrMatrix[i] = obj->detMatrix[i*obj->numDopplerBins];
        }

        message.body.detObj.tlv[tlvIdx].length = itemPayloadLen;
        message.body.detObj.tlv[tlvIdx].type = MMWDEMO_OUTPUT_MSG_RANGE_PROFILE;
        message.body.detObj.tlv[tlvIdx].address = (uint32_t) ptrCurrBuffer;
        tlvIdx++;

        /* Incrementing pointer to HSM buffer */
        ptrCurrBuffer = (uint8_t *)((uint32_t)ptrHsmBuffer + totalHsmSize);
        totalPacketLen += sizeof(MmwDemo_output_message_tl) + itemPayloadLen;
   }

    /* Sending range profile:  2bytes * numRangeBins */
    if (pGuiMonSel->noiseProfile == 1)
    {
        uint32_t maxDopIdx = obj->numDopplerBins/2 -1;
        itemPayloadLen = sizeof(uint16_t) * obj->numRangeBins;
        totalHsmSize += itemPayloadLen;
        if(totalHsmSize > outputBufSize)
        {
            retVal = -1;
            goto Exit;
        }

        uint16_t *ptrMatrix = (uint16_t *)ptrCurrBuffer;
        for(i = 0; i < obj->numRangeBins; i++)
        {
            ptrMatrix[i] = obj->detMatrix[i*obj->numDopplerBins + maxDopIdx];
        }

        message.body.detObj.tlv[tlvIdx].length = itemPayloadLen;
        message.body.detObj.tlv[tlvIdx].type = MMWDEMO_OUTPUT_MSG_NOISE_PROFILE;
        message.body.detObj.tlv[tlvIdx].address = (uint32_t) ptrCurrBuffer;
        tlvIdx++;

        /* Incrementing pointer to HSM buffer */
        ptrCurrBuffer = (uint8_t *)((uint32_t)ptrHsmBuffer + totalHsmSize);
        totalPacketLen += sizeof(MmwDemo_output_message_tl) + itemPayloadLen;
   }

    /* Sending range Azimuth Heat Map */
    if (pGuiMonSel->rangeAzimuthHeatMap == 1)
    {
        itemPayloadLen = obj->numRangeBins * obj->numVirtualAntAzim * sizeof(cmplx16ImRe_t);
        message.body.detObj.tlv[tlvIdx].length = itemPayloadLen;
        message.body.detObj.tlv[tlvIdx].type = MMWDEMO_OUTPUT_MSG_AZIMUT_STATIC_HEAT_MAP;
        message.body.detObj.tlv[tlvIdx].address = (uint32_t) obj->azimuthStaticHeatMap;
        tlvIdx++;

        totalPacketLen += sizeof(MmwDemo_output_message_tl) + itemPayloadLen;
    }


    /* Sending range Doppler Heat Map  */
    if (pGuiMonSel->rangeDopplerHeatMap == 1)
    {
        itemPayloadLen = obj->numRangeBins * obj->numDopplerBins * sizeof(uint16_t);
        message.body.detObj.tlv[tlvIdx].length = itemPayloadLen;
        message.body.detObj.tlv[tlvIdx].type = MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP;
        message.body.detObj.tlv[tlvIdx].address = (uint32_t) obj->detMatrix;
        tlvIdx++;

        totalPacketLen += sizeof(MmwDemo_output_message_tl) + itemPayloadLen;
    }

    /* Sending stats information  */
    if (pGuiMonSel->statsInfo == 1)
    {
        MmwDemo_output_message_stats stats;
        itemPayloadLen = sizeof(MmwDemo_output_message_stats);
        totalHsmSize += itemPayloadLen;
        if(totalHsmSize > outputBufSize)
        {
            retVal = -1;
            goto Exit;
        }

        stats.interChirpProcessingMargin = (uint32_t) (obj->timingInfo.chirpProcessingEndMarginMin/DSP_CLOCK_MHZ);
        stats.interFrameProcessingMargin = (uint32_t) (obj->timingInfo.interFrameProcessingEndMargin/DSP_CLOCK_MHZ);
        stats.interFrameProcessingTime = (uint32_t) (obj->timingInfo.interFrameProcCycles/DSP_CLOCK_MHZ);
        stats.transmitOutputTime = (uint32_t) (obj->timingInfo.transmitOutputCycles/DSP_CLOCK_MHZ);
        stats.activeFrameCPULoad = obj->timingInfo.activeFrameCPULoad;
        stats.interFrameCPULoad = obj->timingInfo.interFrameCPULoad;
        memcpy(ptrCurrBuffer, (void *)&stats, itemPayloadLen);

        message.body.detObj.tlv[tlvIdx].length = itemPayloadLen;
        message.body.detObj.tlv[tlvIdx].type = MMWDEMO_OUTPUT_MSG_STATS;
        message.body.detObj.tlv[tlvIdx].address = (uint32_t) ptrCurrBuffer;;
        tlvIdx++;

        /* Incrementing pointer to HSM buffer */
        ptrCurrBuffer = (uint8_t *)((uint32_t)ptrHsmBuffer + totalHsmSize);
        totalPacketLen += sizeof(MmwDemo_output_message_tl) + itemPayloadLen;
    }

    if( retVal == 0)
    {
        message.body.detObj.header.numTLVs = tlvIdx;
        /* Round up packet length to multiple of MMWDEMO_OUTPUT_MSG_SEGMENT_LEN */
        message.body.detObj.header.totalPacketLen = MMWDEMO_OUTPUT_MSG_SEGMENT_LEN *
                ((totalPacketLen + (MMWDEMO_OUTPUT_MSG_SEGMENT_LEN-1))/MMWDEMO_OUTPUT_MSG_SEGMENT_LEN);
        message.body.detObj.header.timeCpuCycles =  Cycleprofiler_getTimeStamp();
        message.body.detObj.header.frameNumber = gMmwDssMCB.stats.frameStartIntCounter;
        message.body.detObj.header.subFrameNumber = gMmwDssMCB.subFrameIndx;
        if (MmwDemo_mboxWrite(&message) != 0)
        {
            retVal = -1;
        }
    }
#endif
Exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      Function to trigger DSS to MSS ISR for fast signalling
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_triggerDss2MssISR(uint8_t dss2MssIsrInfo)
{
    int32_t errCode;

    gHSRAM.dss2MssIsrInfo = dss2MssIsrInfo;
    if (SOC_triggerDSStoMSSsoftwareInterrupt(gMmwDssMCB.socHandle, 
            MMWDEMO_DSS2MSS_EXCEPTION_SIGNALLING_SW_INT_DSS, &errCode) != 0)
    {
        System_printf("Failed to trigger software interrupt %d, error code = %d\n",
            MMWDEMO_DSS2MSS_EXCEPTION_SIGNALLING_SW_INT_DSS, errCode);
    }
}

/**
 *  @b Description
 *  @n
 *      Function to send data path detection output.
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_dssDataPathOutputLogging(MmwDemo_DSS_DataPathObj   * dataPathObj)
{
    /* Sending detected objects to logging buffer and shipped out from MSS UART */
    if (gMmwDssMCB.loggingBufferAvailable == 1)
    {
        /* Set the logging buffer available flag to be 0 */
        gMmwDssMCB.loggingBufferAvailable = 0;

        /* Save output in logging buffer - HSRAM memory and a message is sent to MSS to notify
           logging buffer is ready */
        if (MmwDemo_dssSendProcessOutputToMSS((uint8_t *)&gHSRAM.dataPathDetectionPayload[0],
                                             (uint32_t)MMW_DATAPATH_DET_PAYLOAD_SIZE,
                                             dataPathObj) < 0)
        {
                /* Increment logging error */
                gMmwDssMCB.stats.detObjLoggingErr++;
        }
    }
    else
    {
        /* Logging buffer is not available, skip saving detected objects to logging buffer */
        gMmwDssMCB.stats.detObjLoggingSkip++;
    }
}

/**
 *  @b Description
 *  @n
 *      Function to do Data Path Initialization on DSS.
 *
 *  @retval
 *      Not Applicable.
 */
static int32_t MmwDemo_dataPathAdcBufInit(MmwDemo_DSS_dataPathContext_t *context)
{
    ADCBuf_Params       ADCBufparams;

    /*****************************************************************************
     * Initialize ADCBUF driver
     *****************************************************************************/
    ADCBuf_init();

    /* ADCBUF Params initialize */
    ADCBuf_Params_init(&ADCBufparams);
    ADCBufparams.chirpThresholdPing = 1;
    ADCBufparams.chirpThresholdPong = 1;
    ADCBufparams.continousMode  = 0;

    /* Open ADCBUF driver */
   context->adcbufHandle = ADCBuf_open(0, &ADCBufparams);
    if (context->adcbufHandle == NULL)
    {
        System_printf("Error: MMWDemoDSS Unable to open the ADCBUF driver\n");
        return -1;
    }
    System_printf("Debug: MMWDemoDSS ADCBUF Instance(0) %p has been opened successfully\n", 
        context->adcbufHandle);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      Performs linker generated copy table copy using EDMA. Currently this is
 *      used to page in fast code from L3 to L1PSRAM.
 *  @param[in]  handle EDMA handle
 *  @param[in]  tp Pointer to copy table
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_copyTable(EDMA_Handle handle, COPY_TABLE *tp)
{
    uint16_t i;
    COPY_RECORD crp;
    uint32_t loadAddr;
    uint32_t runAddr;

    for (i = 0; i < tp->num_recs; i++)
    {
        crp = tp->recs[i];
        loadAddr = (uint32_t)crp.load_addr;
        runAddr = (uint32_t)crp.run_addr;

        /* currently we use only one count of EDMA which is 16-bit so we cannot
           handle tables bigger than 64 KB */
        MmwDemo_dssAssert(crp.size <= 65536U);

        if (crp.size)
        {
            MmwDemo_edmaBlockCopy(handle, loadAddr, runAddr, crp.size);
        }
    }
}

/**
 *  @b Description
 *  @n
 *      Performs simple block copy using EDMA. Used for the purpose of copying
 *      linker table for L3 to L1PSRAM copy. memcpy cannot be used because there is
 *      no data bus access to L1PSRAM.
 *
 *  @param[in]  handle EDMA handle
 *  @param[in]  loadAddr load address
 *  @param[in]  runAddr run address
 *  @param[in]  size size in bytes
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_edmaBlockCopy(EDMA_Handle handle, uint32_t loadAddr, uint32_t runAddr, uint16_t size)
{
    EDMA_channelConfig_t config;
    volatile bool isTransferDone;

    config.channelId = EDMA_TPCC0_REQ_FREE_0;
    config.channelType = (uint8_t)EDMA3_CHANNEL_TYPE_DMA;
    config.paramId = (uint16_t)EDMA_TPCC0_REQ_FREE_0;
    config.eventQueueId = 0;

    config.paramSetConfig.sourceAddress = (uint32_t) SOC_translateAddress((uint32_t)loadAddr,
        SOC_TranslateAddr_Dir_TO_EDMA, NULL);
    config.paramSetConfig.destinationAddress = (uint32_t) SOC_translateAddress((uint32_t)runAddr,
        SOC_TranslateAddr_Dir_TO_EDMA, NULL);

    config.paramSetConfig.aCount = size;
    config.paramSetConfig.bCount = 1U;
    config.paramSetConfig.cCount = 1U;
    config.paramSetConfig.bCountReload = 0U;

    config.paramSetConfig.sourceBindex = 0U;
    config.paramSetConfig.destinationBindex = 0U;

    config.paramSetConfig.sourceCindex = 0U;
    config.paramSetConfig.destinationCindex = 0U;

    config.paramSetConfig.linkAddress = EDMA_NULL_LINK_ADDRESS;
    config.paramSetConfig.transferType = (uint8_t)EDMA3_SYNC_A;
    config.paramSetConfig.transferCompletionCode = (uint8_t) EDMA_TPCC0_REQ_FREE_0;
    config.paramSetConfig.sourceAddressingMode = (uint8_t) EDMA3_ADDRESSING_MODE_LINEAR;
    config.paramSetConfig.destinationAddressingMode = (uint8_t) EDMA3_ADDRESSING_MODE_LINEAR;

    /* don't care because of linear addressing modes above */
    config.paramSetConfig.fifoWidth = (uint8_t) EDMA3_FIFO_WIDTH_8BIT;

    config.paramSetConfig.isStaticSet = false;
    config.paramSetConfig.isEarlyCompletion = false;
    config.paramSetConfig.isFinalTransferInterruptEnabled = true;
    config.paramSetConfig.isIntermediateTransferInterruptEnabled = false;
    config.paramSetConfig.isFinalChainingEnabled = false;
    config.paramSetConfig.isIntermediateChainingEnabled = false;
    config.transferCompletionCallbackFxn = NULL;
    config.transferCompletionCallbackFxnArg = NULL;

    if (EDMA_configChannel(handle, &config, false) != EDMA_NO_ERROR)
    {
        MmwDemo_dssAssert(0);
    }

    if (EDMA_startDmaTransfer(handle, config.channelId) != EDMA_NO_ERROR)
    {
        MmwDemo_dssAssert(0);
    }

    /* wait until transfer done */
    do {
        if (EDMA_isTransferComplete(handle,
                config.paramSetConfig.transferCompletionCode,
                (bool *)&isTransferDone) != EDMA_NO_ERROR)
        {
            MmwDemo_dssAssert(0);
        }
    } while (isTransferDone == false);

    /* make sure to disable channel so it is usable later */
    EDMA_disableChannel(handle, config.channelId, config.channelType);
}

/**
 *  @b Description
 *  @n
 *      Function to do Data Path Initialization on DSS.
 *
 *  @retval
 *      Not Applicable.
 */
int32_t MmwDemo_dssDataPathInit(void)
{
    int32_t retVal;
    SOC_SysIntListenerCfg  socIntCfg;
    int32_t errCode;
    MmwDemo_DSS_dataPathContext_t *context;
    uint32_t subFrameIndx;

    context = &gMmwDssMCB.dataPathContext;

    for(subFrameIndx = 0; subFrameIndx < RL_MAX_SUBFRAMES; subFrameIndx++)
    {
        MmwDemo_DSS_DataPathObj *obj;

        obj = &gMmwDssMCB.dataPathObj[subFrameIndx];
        MmwDemo_dataPathObjInit(obj,
                                context,
                                &gMmwDssMCB.cliCfg[subFrameIndx],
                                &gMmwDssMCB.cliCommonCfg);
        MmwDemo_dataPathInit1Dstate(obj);
    }

    retVal = MmwDemo_dataPathInitEdma(context);
    if (retVal < 0)
    {
        return -1;
    }

    /* Copy code from L3 to L1PSRAM, this code related to data path processing */
    MmwDemo_copyTable(context->edmaHandle[0], &_MmwDemo_fastCode_L1PSRAM_copy_table);

    retVal = MmwDemo_dataPathAdcBufInit(context);
    if (retVal < 0)
    {
        return -1;
    }

    /* Register chirp interrupt listener */
    socIntCfg.systemInterrupt  = SOC_XWR68XX_DSS_INTC_EVENT_CHIRP_AVAIL;
    socIntCfg.listenerFxn      = MmwDemo_dssChirpIntHandler;
    socIntCfg.arg              = (uintptr_t)NULL;
    if (SOC_registerSysIntListener(gMmwDssMCB.socHandle, &socIntCfg, &errCode) == NULL)
    {
        System_printf("Error: Unable to register chirp interrupt listener , error = %d\n", errCode);
        return -1;
    }

    /* Register frame start interrupt listener */
    socIntCfg.systemInterrupt  = SOC_XWR68XX_DSS_INTC_EVENT_FRAME_START;
    socIntCfg.listenerFxn      = MmwDemo_dssFrameStartIntHandler;
    socIntCfg.arg              = (uintptr_t)NULL;
    if (SOC_registerSysIntListener(gMmwDssMCB.socHandle, &socIntCfg, &errCode) == NULL)
    {
        System_printf("Error: Unable to register frame start interrupt listener , error = %d\n", errCode);
        return -1;
    }

    /* Initialize detected objects logging */
    gMmwDssMCB.loggingBufferAvailable = 1;

    return 0;
}

/**
 *  @b Description
 *  @n
 *      Function to configure ADCBUF driver based on CLI inputs.
 *  @param[in] ptrDataPathObj Pointer to data path object.
 *
 *  @retval
 *      0 if no error, else error (there will be system prints for these).
 */
static int32_t MmwDemo_dssDataPathConfigAdcBuf(MmwDemo_DSS_DataPathObj *ptrDataPathObj)
{
    ADCBuf_dataFormat   dataFormat;
    ADCBuf_RxChanConf   rxChanConf;
    int32_t             retVal;
    uint8_t             channel;
    uint8_t             numBytePerSample = 0;
    MMWave_OpenCfg*     ptrOpenCfg;
    uint32_t            chirpThreshold;
    uint32_t            rxChanMask = 0xF;
    uint32_t            maxChirpThreshold;
    uint32_t            bytesPerChirp;
    MmwDemo_DSS_dataPathContext_t *context = ptrDataPathObj->context;
    MmwDemo_ADCBufCfg   *adcBufCfg = &ptrDataPathObj->cliCfg->adcBufCfg;

    /* Get data path object and control configuration */
    ptrOpenCfg = &gMmwDssMCB.cfg.openCfg;

    /*****************************************************************************
     * Data path :: ADCBUF driver Configuration
     *****************************************************************************/
    /* Validate the adcFmt */
    if(adcBufCfg->adcFmt == 1)
    {
        /* Real dataFormat has 2 bytes */
        numBytePerSample =  2;
    }
    else if(adcBufCfg->adcFmt == 0)
    {
        /* Complex dataFormat has 4 bytes */
        numBytePerSample =  4;
    }
    else
    {
        MmwDemo_dssAssert(0); /* Data format not supported */
    }

    /* On XWR16xx only channel non-interleaved mode is supported */
    if(adcBufCfg->chInterleave != 1)
    {
        MmwDemo_dssAssert(0); /* Not supported */
    }

    /* Populate data format from configuration */
    dataFormat.adcOutFormat       = adcBufCfg->adcFmt;
    dataFormat.channelInterleave  = adcBufCfg->chInterleave;
    dataFormat.sampleInterleave   = adcBufCfg->iqSwapSel;

    /* Disable all ADCBuf channels */
    if ((retVal = ADCBuf_control(context->adcbufHandle, ADCBufMMWave_CMD_CHANNEL_DISABLE, (void *)&rxChanMask)) < 0)
    {
       System_printf("Error: Disable ADCBuf channels failed with [Error=%d]\n", retVal);
       return retVal;
    }

    retVal = ADCBuf_control(context->adcbufHandle, ADCBufMMWave_CMD_CONF_DATA_FORMAT, (void *)&dataFormat);
    if (retVal < 0)
    {
        System_printf ("Error: MMWDemoDSS Unable to configure the data formats\n");
        return -1;
    }

    memset((void*)&rxChanConf, 0, sizeof(ADCBuf_RxChanConf));

    /* calculate max possible chirp threshold */        
    bytesPerChirp = ptrDataPathObj->numAdcSamples * ptrDataPathObj->numRxAntennas * numBytePerSample;
    
    /* find maximum number of full chirps that can fit in the ADCBUF memory, while
       also being able to divide numChirpsPerFrame, we do not want remainder processing */
    maxChirpThreshold = SOC_ADCBUF_SIZE / bytesPerChirp;
    if (maxChirpThreshold >= ptrDataPathObj->numChirpsPerFrame)
    {
        maxChirpThreshold = ptrDataPathObj->numChirpsPerFrame;
    }
    else
    {
        /* find largest divisor of numChirpsPerFrame no bigger than maxChirpThreshold */
        while (ptrDataPathObj->numChirpsPerFrame % maxChirpThreshold)
        {
            maxChirpThreshold--;
        }
    }

    /* ADCBuf control function requires argument alignment at 4 bytes boundary */
    chirpThreshold = adcBufCfg->chirpThreshold;
    
    /* if automatic, set to the calculated max */
    if (chirpThreshold == 0)
    {
        chirpThreshold = maxChirpThreshold;
    }
    else
    {
        if (chirpThreshold > maxChirpThreshold)
        {
            System_printf("Desired chirpThreshold %d higher than max possible of %d, setting to max\n", 
                chirpThreshold, maxChirpThreshold);
            chirpThreshold = maxChirpThreshold;
        }
        else
        {
            /* check for divisibility of the user provided threshold */
            MmwDemo_dssAssert((ptrDataPathObj->numChirpsPerFrame % chirpThreshold) == 0);
        }
    }

    ptrDataPathObj->numChirpsPerChirpEvent = chirpThreshold;

    /* Enable Rx Channels */
    for (channel = 0; channel < SYS_COMMON_NUM_RX_CHANNEL; channel++)
    {
        if(ptrOpenCfg->chCfg.rxChannelEn & (0x1U << channel))
        {
            /* Populate the receive channel configuration: */
            rxChanConf.channel = channel;
            retVal = ADCBuf_control(context->adcbufHandle, ADCBufMMWave_CMD_CHANNEL_ENABLE, (void *)&rxChanConf);
            if (retVal < 0)
            {
                System_printf("Error: MMWDemoDSS ADCBuf Control for Channel %d Failed with error[%d]\n", channel, retVal);
                return -1;
            }
            rxChanConf.offset  += ptrDataPathObj->numAdcSamples * numBytePerSample * chirpThreshold;
        }
    }

    /* Set ping/pong chirp threshold: */
    retVal = ADCBuf_control(context->adcbufHandle, ADCBufMMWave_CMD_SET_PING_CHIRP_THRESHHOLD,
                            (void *)&chirpThreshold);
    if(retVal < 0)
    {
        System_printf("Error: ADCbuf Ping Chirp Threshold Failed with Error[%d]\n", retVal);
        return -1;
    }
    retVal = ADCBuf_control(context->adcbufHandle, ADCBufMMWave_CMD_SET_PONG_CHIRP_THRESHHOLD,
                            (void *)&chirpThreshold);
    if(retVal < 0)
    {
        System_printf("Error: ADCbuf Pong Chirp Threshold Failed with Error[%d]\n", retVal);
        return -1;
    }

    return 0;
}

static uint16_t MmwDemo_getChirpStartIdx(MMWave_CtrlCfg *cfg, uint8_t subFrameIndx)
{
    if (cfg->dfeDataOutputMode == MMWave_DFEDataOutputMode_ADVANCED_FRAME)
    {
        return(cfg->u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[subFrameIndx].chirpStartIdx);
    }
    else 
    {
        return(cfg->u.frameCfg.frameCfg.chirpStartIdx);
    }
}

static uint16_t MmwDemo_getChirpEndIdx(MMWave_CtrlCfg *cfg, uint8_t subFrameIndx)
{
    if (cfg->dfeDataOutputMode == MMWave_DFEDataOutputMode_ADVANCED_FRAME)
    {
        return(cfg->u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[subFrameIndx].chirpStartIdx +
              (cfg->u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[subFrameIndx].numOfChirps - 1));
    }
    else 
    {
        return(cfg->u.frameCfg.frameCfg.chirpEndIdx);
    }
}

static uint16_t MmwDemo_getNumLoops(MMWave_CtrlCfg *cfg, uint8_t subFrameIndx)
{
    if (cfg->dfeDataOutputMode == MMWave_DFEDataOutputMode_ADVANCED_FRAME)
    {
        return(cfg->u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[subFrameIndx].numLoops);
    }
    else 
    {
        return(cfg->u.frameCfg.frameCfg.numLoops);
    }
}

static MMWave_ProfileHandle MmwDemo_getProfileHandle(MMWave_CtrlCfg *cfg, uint32_t profileLoopIdx)
{
    if (cfg->dfeDataOutputMode == MMWave_DFEDataOutputMode_ADVANCED_FRAME)
    {
        return(cfg->u.advancedFrameCfg.profileHandle[profileLoopIdx]);
    }
    else 
    {
        return(cfg->u.frameCfg.profileHandle[profileLoopIdx]);
    }
}

/**
 *  @b Description
 *  @n
 *      parses Profile, Chirp and Frame config and extracts parameters
 *      needed for processing chain configuration
 */
bool MmwDemo_parseProfileAndChirpConfig(MmwDemo_DSS_DataPathObj *dataPathObj,
    MMWave_CtrlCfg* ptrCtrlCfg, uint8_t subFrameIndx)
{
    uint16_t        frameChirpStartIdx;
    uint16_t        frameChirpEndIdx;
    uint16_t        numLoops;
    int16_t         frameTotalChirps;
    int32_t         errCode;
    uint32_t        profileLoopIdx, chirpLoopIdx;
    bool            foundValidProfile = false;
    uint16_t        channelTxEn;
    uint8_t         channel;
    uint8_t         numRxChannels = 0;
    MMWave_OpenCfg* ptrOpenCfg;
    uint8_t         rxAntOrder [SYS_COMMON_NUM_RX_CHANNEL];
    uint8_t         txAntOrder [SYS_COMMON_NUM_TX_ANTENNAS];
    int32_t         i;
    int32_t         txIdx, rxIdx;

    /* Get data path object and control configuration */
    ptrOpenCfg = &gMmwDssMCB.cfg.openCfg;

    /* Get the Transmit channel enable mask: */
    channelTxEn = ptrOpenCfg->chCfg.txChannelEn;

    /* Find total number of Rx Channels */
    for (channel = 0; channel < SYS_COMMON_NUM_RX_CHANNEL; channel++)
    {
        rxAntOrder[channel] = 0;
        if(ptrOpenCfg->chCfg.rxChannelEn & (0x1U << channel))
        {
            rxAntOrder[numRxChannels] = channel;
            /* Track the number of receive channels: */
            numRxChannels += 1;
        }
    }
    dataPathObj->numRxAntennas = numRxChannels;

    if (ptrCtrlCfg->dfeDataOutputMode == MMWave_DFEDataOutputMode_ADVANCED_FRAME)
    {
        MmwDemo_dssAssert(
            ptrCtrlCfg->u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[subFrameIndx].numOfBurst == 1);
    }

    /* read frameCfg chirp start/stop*/
    frameChirpStartIdx = MmwDemo_getChirpStartIdx(ptrCtrlCfg, subFrameIndx);
    frameChirpEndIdx   = MmwDemo_getChirpEndIdx(ptrCtrlCfg, subFrameIndx);
    numLoops = MmwDemo_getNumLoops(ptrCtrlCfg, subFrameIndx);

    frameTotalChirps   = frameChirpEndIdx - frameChirpStartIdx + 1;

    /* since validChirpTxEnBits is static array of 32 */
    MmwDemo_dssAssert(frameTotalChirps <= 32);

    /* loop for profiles and find if it has valid chirps */
    /* we support only one profile in this processing chain */
    for (profileLoopIdx = 0;
        ((profileLoopIdx < MMWAVE_MAX_PROFILE) && (foundValidProfile == false));
        profileLoopIdx++)
    {
        uint32_t    mmWaveNumChirps = 0;
        bool        validProfileHasOneTxPerChirp = false;
        uint16_t    validProfileTxEn = 0;
        uint16_t    validChirpTxEnBits[32] = {0};
        MMWave_ProfileHandle profileHandle;

        profileHandle = MmwDemo_getProfileHandle(ptrCtrlCfg, profileLoopIdx);
        if (profileHandle == NULL)
            continue; /* skip this profile */

        /* get numChirps for this profile; skip error checking */
        MMWave_getNumChirps(profileHandle, &mmWaveNumChirps, &errCode);
        /* loop for chirps and find if it has valid chirps for the frame
           looping around for all chirps in a profile, in case
           there are duplicate chirps
         */
        for (chirpLoopIdx = 1; chirpLoopIdx <= mmWaveNumChirps; chirpLoopIdx++)
        {
            MMWave_ChirpHandle chirpHandle;
            /* get handle and read ChirpCfg */
            if (MMWave_getChirpHandle(profileHandle, chirpLoopIdx, &chirpHandle, &errCode)==0)
            {
                rlChirpCfg_t chirpCfg;
                if (MMWave_getChirpCfg(chirpHandle, &chirpCfg, &errCode)==0)
                {
                    uint16_t chirpTxEn = chirpCfg.txEnable;
                    /* do chirps fall in range and has valid antenna enabled */
                    if ((chirpCfg.chirpStartIdx >= frameChirpStartIdx) &&
                        (chirpCfg.chirpEndIdx <= frameChirpEndIdx) &&
                        ((chirpTxEn & channelTxEn) > 0))
                    {
                        uint16_t idx = 0;
                        for (idx = (chirpCfg.chirpStartIdx - frameChirpStartIdx);
                             idx <= (chirpCfg.chirpEndIdx - frameChirpStartIdx); idx++)
                        {
                            validChirpTxEnBits[idx] = chirpTxEn;
                            foundValidProfile = true;
                        }

                    }
                }
            }
        }
        /* now loop through unique chirps and check if we found all of the ones
           needed for the frame and then determine the azimuth antenna
           configuration
         */
        if (foundValidProfile) {
            for (chirpLoopIdx = 0; chirpLoopIdx < frameTotalChirps; chirpLoopIdx++)
            {
                bool validChirpHasOneTxPerChirp = false;
                uint16_t chirpTxEn = validChirpTxEnBits[chirpLoopIdx];
                if (chirpTxEn == 0) {
                    /* this profile doesnt have all the needed chirps */
                    foundValidProfile = false;
                    break;
                }
                validChirpHasOneTxPerChirp = ((chirpTxEn == 0x1) || (chirpTxEn == 0x2) || (chirpTxEn == 0x4));
                /* if this is the first chirp, record the chirp's
                   MIMO config as profile's MIMO config. We dont handle intermix
                   at this point */
                if (chirpLoopIdx==0) {
                    validProfileHasOneTxPerChirp = validChirpHasOneTxPerChirp;
                }
                /* check the chirp's MIMO config against Profile's MIMO config */
                if (validChirpHasOneTxPerChirp != validProfileHasOneTxPerChirp)
                {
                    /* this profile doesnt have all chirps with same MIMO config */
                    foundValidProfile = false;
                    break;
                }
                /* save the antennas actually enabled in this profile */
                validProfileTxEn |= chirpTxEn;
            }
        }

        /* found valid chirps for the frame; mark this profile valid */
        if (foundValidProfile == true) {
            rlProfileCfg_t  profileCfg;
            uint32_t        numTxAntAzim = 0;
            uint32_t        numTxAntElev = 0;

            dataPathObj->validProfileIdx = profileLoopIdx;
            dataPathObj->numTxAntennas = 0;
            if (!validProfileHasOneTxPerChirp)
            {
                numTxAntAzim=1;
            }
            else
            {
                if (validProfileTxEn & 0x1)
                {
                    numTxAntAzim++;
                }
                if (validProfileTxEn & 0x2)
                {
                    numTxAntElev++;
                }
				if (validProfileTxEn & 0x4)
                {
                    numTxAntAzim++;
                }
            }
            //System_printf("Azimuth Tx: %d (MIMO:%d) \n",numTxAntAzim,validProfileHasMIMO);
            dataPathObj->numTxAntennas       = numTxAntAzim + numTxAntElev;
            dataPathObj->numVirtualAntAzim   = numTxAntAzim * dataPathObj->numRxAntennas;
            dataPathObj->numVirtualAntElev   = numTxAntElev * dataPathObj->numRxAntennas;
            dataPathObj->numVirtualAntennas  = dataPathObj->numVirtualAntAzim + dataPathObj->numVirtualAntElev;

            /* Copy the Rx channel compensation coefficients from common area to data path structure */
            if (validProfileHasOneTxPerChirp)
            {
                for (i = 0; i < dataPathObj->numTxAntennas; i++)
                {
                    txAntOrder[i] = MmwDemo_floorLog2(validChirpTxEnBits[i]);
                }
                for (txIdx = 0; txIdx < dataPathObj->numTxAntennas; txIdx++)
                {
                    for (rxIdx = 0; rxIdx < dataPathObj->numRxAntennas; rxIdx++)
                    {
                        dataPathObj->compRxChanCfg.rxChPhaseComp[txIdx*dataPathObj->numRxAntennas + rxIdx] =
                                dataPathObj->cliCommonCfg->compRxChanCfg.rxChPhaseComp[txAntOrder[txIdx]*SYS_COMMON_NUM_RX_CHANNEL + rxAntOrder[rxIdx]];

                    }

                }
            }
            else
            {
                cmplx16ImRe_t one;
                one.imag = 0;
                one.real = 0x7fff;
                for (txIdx = 0; txIdx < dataPathObj->numTxAntennas; txIdx++)
                {
                    for (rxIdx = 0; rxIdx < dataPathObj->numRxAntennas; rxIdx++)
                    {
                        dataPathObj->compRxChanCfg.rxChPhaseComp[txIdx*dataPathObj->numRxAntennas + rxIdx] = one;
                    }

                }
            }
            /* Get the profile configuration: */
            if (MMWave_getProfileCfg (profileHandle, &profileCfg, &errCode) < 0)
            {
                System_printf ("Error: Unable to get the profile configuration [Error code %d]\n", errCode);
                MmwDemo_dssAssert (0);
                return -1;
            }

            /* multiplicity of 4 due to windowing library function requirement */
            if ((profileCfg.numAdcSamples % 4) != 0)
            {
                System_printf("Number of ADC samples must be multiple of 4\n");
                MmwDemo_dssAssert(0);
            }
            dataPathObj->numAdcSamples       = profileCfg.numAdcSamples;
            dataPathObj->numRangeBins        = MmwDemo_pow2roundup(dataPathObj->numAdcSamples);
            dataPathObj->numChirpsPerFrame   = (frameChirpEndIdx -frameChirpStartIdx + 1) *
                                               numLoops;
            dataPathObj->numAngleBins        = MMW_NUM_ANGLE_BINS;
            dataPathObj->numDopplerBins      = dataPathObj->numChirpsPerFrame/dataPathObj->numTxAntennas;

            /* multiplicity of 4 due to windowing library function requirement */
            if ((dataPathObj->numDopplerBins % 4) != 0)
            {
                System_printf("Number of Doppler bins must be multiple of 4\n");
                MmwDemo_dssAssert(0);
            }

            if (dataPathObj->cliCfg->extendedMaxVelocityCfg.enabled  && 
                dataPathObj->cliCfg->multiObjBeamFormingCfg.enabled)
            {
                System_printf("Simultaneous multi object beam forming and extended maximum velocity is not supported.\n");
                MmwDemo_dssAssert(0);
            }

            if (dataPathObj->cliCfg->extendedMaxVelocityCfg.enabled  && numTxAntAzim == 1)
            {
                System_printf("Extended maximum velocity technique is supported only in TDM MIMO\n");
                MmwDemo_dssAssert(0);
            }

#ifndef MMW_ENABLE_NEGATIVE_FREQ_SLOPE
            /* Check frequency slope */
            if (profileCfg.freqSlopeConst < 0)
            {
                System_printf("Frequency slope must be positive\n");
                MmwDemo_dssAssert(0);
            }
#endif
            dataPathObj->rangeResolution = MMWDEMO_SPEED_OF_LIGHT_IN_METERS_PER_SEC * 
                profileCfg.digOutSampleRate * 1e3 / 
                (2 * (profileCfg.freqSlopeConst) * ((3.6*1e3*900) /
                 (1U << 26)) * 1e12 * dataPathObj->numRangeBins);
            dataPathObj->xyzOutputQFormat = (uint8_t) ceil(log10(16. /
                fabs(dataPathObj->rangeResolution))/log10(2));
        }
    }
    return foundValidProfile;
}

/**
 *  @b Description
 *  @n
 *      Function to do Data Path Re-Configuration on DSS.
 *      This is used when switching between sub-frames.
 *
 *  @retval
 *      -1 if error, 0 otherwise.
 */
static int32_t MmwDemo_dssDataPathReconfig(MmwDemo_DSS_DataPathObj *obj)
{
    int32_t retVal;

    retVal = MmwDemo_dssDataPathConfigAdcBuf(obj);
    if (retVal < 0)
    {
        return -1;
    }

    MmwDemo_dataPathConfigFFTs(obj);

    /* must be after MmwDemo_dssDataPathConfigAdcBuf above as it calculates 
       numChirpsPerChirpEvent that is used in EDMA configuration */
    MmwDemo_dataPathConfigEdma(obj);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      Function to do Data Path Configuration on DSS.
 *
 *  @retval
 *      Not Applicable.
 */
static int32_t MmwDemo_dssDataPathConfig(void)
{
    int32_t             retVal = 0;
    MMWave_CtrlCfg      *ptrCtrlCfg;
    MmwDemo_DSS_DataPathObj *dataPathObj;
    uint8_t subFrameIndx;

    /* Get data path object and control configuration */
    ptrCtrlCfg   = &gMmwDssMCB.cfg.ctrlCfg;

    if (ptrCtrlCfg->dfeDataOutputMode == MMWave_DFEDataOutputMode_ADVANCED_FRAME)
    {
        gMmwDssMCB.numSubFrames = 
            ptrCtrlCfg->u.advancedFrameCfg.frameCfg.frameSeq.numOfSubFrames;
    }
    else
    {
        gMmwDssMCB.numSubFrames = 1;
    }

    for(subFrameIndx = 0; subFrameIndx < gMmwDssMCB.numSubFrames; subFrameIndx++)
    {
        dataPathObj  = &gMmwDssMCB.dataPathObj[subFrameIndx];

        /*****************************************************************************
         * Data path :: Algorithm Configuration
         *****************************************************************************/

        /* Parse the profile and chirp configs and get the valid number of TX Antennas */
        if (MmwDemo_parseProfileAndChirpConfig(dataPathObj, ptrCtrlCfg, subFrameIndx) == true)
        {
            /* Data path configurations */
            MmwDemo_dataPathConfigBuffers(dataPathObj, SOC_XWR68XX_DSS_ADCBUF_BASE_ADDRESS);
            MmwDemo_dataPathComputeDerivedConfig(dataPathObj);

            /* Below configurations are to be reconfigured every sub-frame so do only for first one */
            if (subFrameIndx == 0)
            {
                retVal = MmwDemo_dssDataPathReconfig(dataPathObj);
                if (retVal < 0)
                {
                    return -1;
                }
            }
        }
        else
        {
            /* no valid profile found - assert! */
            MmwDemo_dssAssert(0);
        }
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      Function to start Data Path on DSS.
 *
 *  @retval
 *      Not Applicable.
 */
static int32_t MmwDemo_dssDataPathStart(bool doRFStart)
{
    int32_t    errCode;
    MMWave_CalibrationCfg   calibrationCfg;

    gMmwDssMCB.dataPathContext.chirpProcToken = 0;
    gMmwDssMCB.dataPathContext.interFrameProcToken = 0;

    if (doRFStart)
    {
        /* Initialize the calibration configuration: */
        memset ((void*)&calibrationCfg, 0, sizeof(MMWave_CalibrationCfg));

        /* Populate the calibration configuration: */
        calibrationCfg.dfeDataOutputMode                          = 
            gMmwDssMCB.cfg.ctrlCfg.dfeDataOutputMode;
        calibrationCfg.u.chirpCalibrationCfg.enableCalibration    = false;
        calibrationCfg.u.chirpCalibrationCfg.enablePeriodicity    = false;
        calibrationCfg.u.chirpCalibrationCfg.periodicTimeInFrames = 10U;

    /* Start the mmWave module: The configuration has been applied successfully. */
        if (MMWave_start (gMmwDssMCB.ctrlHandle, &calibrationCfg, &errCode) < 0)
        {
            MMWave_ErrorLevel                       errorLevel;
            int16_t                                 mmWaveError;
            int16_t                                 subSysError;
            MMWave_decodeError (errCode, &errorLevel, &mmWaveError, &subSysError);
            System_printf ("Error: MMWDemoDSS mmWave Start failed [Error: %d mmWave: %d SubSys: %d]\n",
                                errCode, mmWaveError, subSysError);
            return -1;
            /* Error: Unable to start the mmWave control */
//            System_printf ("Error: MMWDemoDSS mmWave Start failed [Error code %d]\n", errCode);
//            return -1;
        }
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      Function to process Data Path events at runtime.
 *
 *  @param[in]  event
 *      Data Path Event
 *
 *  @retval
 *      Not Applicable.
 */
static int32_t MmwDemo_dssDataPathProcessEvents(UInt event)
{
    MmwDemo_DSS_DataPathObj *dataPathObj;
    volatile uint32_t startTime;

    dataPathObj = &gMmwDssMCB.dataPathObj[gMmwDssMCB.subFrameIndx];

    /* Handle dataPath events */
    switch(event)
    {
        case MMWDEMO_CHIRP_EVT:
            /* Increment event stats */
            gMmwDssMCB.stats.chirpEvt++;
            {
                uint16_t chirpIndex;
                for (chirpIndex = 0; chirpIndex < dataPathObj->numChirpsPerChirpEvent; chirpIndex++)
                {
                    MmwDemo_processChirp(dataPathObj, (uint16_t) chirpIndex);
                }
            }
            gMmwDssMCB.dataPathContext.chirpProcToken--;
            dataPathObj->timingInfo.chirpProcessingEndTime = Cycleprofiler_getTimeStamp();

            if (dataPathObj->chirpCount == 0)
            {
                MmwDemo_waitEndOfChirps(dataPathObj);
                Load_update();
                dataPathObj->timingInfo.activeFrameCPULoad = Load_getCPULoad();

                dataPathObj->cycleLog.interChirpProcessingTime = gCycleLog.interChirpProcessingTime;
                dataPathObj->cycleLog.interChirpWaitTime = gCycleLog.interChirpWaitTime;
                gCycleLog.interChirpProcessingTime = 0;
                gCycleLog.interChirpWaitTime = 0;

                startTime = Cycleprofiler_getTimeStamp();
                MmwDemo_interFrameProcessing(dataPathObj);
                dataPathObj->timingInfo.interFrameProcCycles = (Cycleprofiler_getTimeStamp() - startTime);

                dataPathObj->cycleLog.interFrameProcessingTime = gCycleLog.interFrameProcessingTime;
                dataPathObj->cycleLog.interFrameWaitTime = gCycleLog.interFrameWaitTime;
                gCycleLog.interFrameProcessingTime = 0;
                gCycleLog.interFrameWaitTime = 0;

                /* Sending range bias and Rx channel phase offset measurements to MSS and from there to CLI */
                if(dataPathObj->cliCommonCfg->measureRxChanCfg.enabled)
                {
                    MmwDemo_measurementResultOutput (dataPathObj);
                }

                /* Sending detected objects to logging buffer */
                MmwDemo_dssDataPathOutputLogging (dataPathObj);
                dataPathObj->timingInfo.interFrameProcessingEndTime = Cycleprofiler_getTimeStamp();
            }
            break;

        case MMWDEMO_FRAMESTART_EVT:
            /* Increment event stats */
            gMmwDssMCB.stats.frameStartEvt++;
            Load_update();
            dataPathObj->timingInfo.interFrameCPULoad = Load_getCPULoad();
            MmwDemo_dssAssert(dataPathObj->chirpCount == 0);
            break;

        case MMWDEMO_BSS_FRAME_TRIGGER_READY_EVT:
            /* Increment event stats */
            gMmwDssMCB.stats.frameTrigEvt++;
            break;

        default:
            break;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Function to stop Data Path on DSS. Assume BSS has been stopped by mmWave already.
 *      This also sends the STOP done message back to MSS to signal the procssing
 *      chain has come to a stop.
 *
 *  @retval
 *      Not Applicable.
 */
static int32_t MmwDemo_dssDataPathStop(void)
{
    MmwDemo_message     message;
    uint32_t numFrameEvtsHandled = gMmwDssMCB.stats.frameStartEvt;


    /* move to stop state */
    gMmwDssMCB.state = MmwDemo_DSS_STATE_STOPPED;

    /* Update stats */
    gMmwDssMCB.stats.chirpEvt = 0;
    gMmwDssMCB.stats.frameStartEvt = 0;
    gMmwDssMCB.stats.frameTrigEvt = 0;
    gMmwDssMCB.stats.numFailedTimingReports = 0;
    gMmwDssMCB.stats.numCalibrationReports = 0;

    /* send message back to MSS */
    message.type = MMWDEMO_DSS2MSS_STOPDONE;
    MmwDemo_mboxWrite(&message);

    System_printf ("Debug: MMWDemoDSS Data Path stop succeeded stop%d,frames:%d \n",
                    gMmwDssMCB.stats.stopEvt,numFrameEvtsHandled);

    return 0;
}


/**
 *  @b Description
 *  @n
 *      The task is used to provide an execution context for the mmWave
 *      control task
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_dssMMWaveCtrlTask(UArg arg0, UArg arg1)
{
    int32_t errCode;

    while (1)
    {
        /* Execute the mmWave control module: */
        if (MMWave_execute (gMmwDssMCB.ctrlHandle, &errCode) < 0)
            System_printf ("Error: MMWDemoDSS mmWave control execution failed [Error code %d]\n", errCode);
    }
}

/**
 *  @b Description
 *  @n
 *      Data Path main task that handles events from remote and do dataPath processing.
 *  This task is created when MSS is responsible for the mmwave Link and DSS is responsible
 *  for data path processing.
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_dssDataPathTask(UArg arg0, UArg arg1)
{
    int32_t       retVal = 0;
    UInt          event;

    /************************************************************************
     * Data Path :: Config
     ************************************************************************/

    /* Waiting for Config event from Remote - MSS */
    Event_pend(gMmwDssMCB.eventHandle, MMWDEMO_CONFIG_EVT, Event_Id_NONE, BIOS_WAIT_FOREVER);
    if ((retVal = MmwDemo_dssDataPathConfig()) < 0 )
    {
        System_printf ("Debug: MMWDemoDSS Data Path config failed with Error[%d]\n",retVal);
        goto exit;
    }

    /************************************************************************
     * Data Path :: Start, mmwaveLink start will be triggered from DSS!
     ************************************************************************/
    if ((retVal = MmwDemo_dssDataPathStart(true)) < 0 )
    {
        System_printf ("Debug: MMWDemoDSS Data Path start failed with Error[%d]\n",retVal);
        goto exit;
    }
    gMmwDssMCB.state = MmwDemo_DSS_STATE_STARTED;

    /************************************************************************
     * Data Path :: Main loop
     ************************************************************************/
    while (1)
    {
        event = Event_pend(gMmwDssMCB.eventHandle,
                          Event_Id_NONE,
                          MMWDEMO_FRAMESTART_EVT | MMWDEMO_CHIRP_EVT |
                          MMWDEMO_BSS_STOP_COMPLETE_EVT | MMWDEMO_CONFIG_EVT |
                          MMWDEMO_STOP_COMPLETE_EVT | MMWDEMO_START_EVT, 
                          BIOS_WAIT_FOREVER);


        if(event & MMWDEMO_BSS_STOP_COMPLETE_EVT)
        {
            MmwDemo_bssStopDone();
        }

        /************************************************************************
         * Data Path process frame start event
         ************************************************************************/
        if(event & MMWDEMO_FRAMESTART_EVT)
        {
            if((gMmwDssMCB.state == MmwDemo_DSS_STATE_STARTED) || (gMmwDssMCB.state == MmwDemo_DSS_STATE_STOP_PENDING))
            {
                if ((retVal = MmwDemo_dssDataPathProcessEvents(MMWDEMO_FRAMESTART_EVT)) < 0 )
                {
                    System_printf ("Error: MMWDemoDSS Data Path process frame start event failed with Error[%d]\n",
                                  retVal);
                }
            }
        }

        /************************************************************************
         * Data Path process chirp event
         ************************************************************************/
        if(event & MMWDEMO_CHIRP_EVT)
        {
            if((gMmwDssMCB.state == MmwDemo_DSS_STATE_STARTED) || (gMmwDssMCB.state == MmwDemo_DSS_STATE_STOP_PENDING))
            {
                if ((retVal = MmwDemo_dssDataPathProcessEvents(MMWDEMO_CHIRP_EVT)) < 0 )
                {
                    System_printf ("Error: MMWDemoDSS Data Path process chirp event failed with Error[%d]\n",
                                  retVal);
                }
            }
        }

        /************************************************************************
         * Data Path re-config, only supported reconfiguration in stop state
         ************************************************************************/
        if(event & MMWDEMO_CONFIG_EVT)
        {
            if(gMmwDssMCB.state == MmwDemo_DSS_STATE_STOPPED)
            {
                if ((retVal = MmwDemo_dssDataPathConfig()) < 0 )
                {
                    System_printf ("Debug: MMWDemoDSS Data Path config failed with Error[%d]\n",retVal);
                    goto exit;
                }

                /************************************************************************
                 * Data Path :: Start, mmwaveLink start will be triggered from DSS!
                 ************************************************************************/
                if ((retVal = MmwDemo_dssDataPathStart(true)) < 0 )
                {
                    System_printf ("Error: MMWDemoDSS Data Path start failed with Error[%d]\n",retVal);
                    goto exit;
                }
                gMmwDssMCB.state = MmwDemo_DSS_STATE_STARTED;
            }
            else
            {
                System_printf ("Error: MMWDemoDSS Data Path config event in wrong state[%d]\n", gMmwDssMCB.state);
                goto exit;
            }
        }

        /************************************************************************
         * Quick start after stop
         ************************************************************************/
        if(event & MMWDEMO_START_EVT)
        {
            if(gMmwDssMCB.state == MmwDemo_DSS_STATE_STOPPED)
            {
                /* RF start is done by MSS in this case; so just do DSS start */
                if ((retVal = MmwDemo_dssDataPathStart(false)) < 0 )
                {
                    System_printf ("Error: MMWDemoDSS Data Path start failed with Error[%d]\n",retVal);
                    goto exit;
    }
                gMmwDssMCB.state = MmwDemo_DSS_STATE_STARTED;
            }
            else
            {
                System_printf ("Error: MMWDemoDSS Data Path config event in wrong state[%d]\n", gMmwDssMCB.state);
                goto exit;
            }
        }

        /************************************************************************
         * Data Path process frame start event
         ************************************************************************/
        if(event & MMWDEMO_STOP_COMPLETE_EVT)
        {
            if (gMmwDssMCB.state == MmwDemo_DSS_STATE_STOP_PENDING)
            {
                /************************************************************************
                 * Local Data Path Stop
                 ************************************************************************/
                if ((retVal = MmwDemo_dssDataPathStop()) < 0 )
                {
                    System_printf ("Debug: MMWDemoDSS Data Path stop failed with Error[%d]\n",retVal);
                }
            }
        }
    }
exit:
    System_printf("Debug: MMWDemoDSS Data path exit\n");

}

/*initialize the MCB with CFAR, Chirp, etc */
static void MCBInit(MmwDemo_DSS_MCB * mcb)
{
    uint8_t  indx;
    for(indx = 0; indx < RL_MAX_SUBFRAMES; indx++)
    {
        /*ADC Buffer*/
        mcb->cliCfg[0].adcBufCfg.adcFmt = 0;
        mcb->cliCfg[0].adcBufCfg.iqSwapSel = 0;
        mcb->cliCfg[0].adcBufCfg.chInterleave = 1;
        mcb->cliCfg[0].adcBufCfg.chirpThreshold = 0;

        /*Clutter Removal */
        mcb->cliCfg[0].clutterRemovalCfg.enabled = 1;

        /* Peak grouping */
        mcb->cliCfg[0].peakGroupingCfg.inDopplerDirectionEn = 0;
        mcb->cliCfg[0].peakGroupingCfg.inRangeDirectionEn = 0;
        mcb->cliCfg[0].peakGroupingCfg.maxRangeIndex = 255;
        mcb->cliCfg[0].peakGroupingCfg.minRangeIndex = 1;
        mcb->cliCfg[0].peakGroupingCfg.scheme = 1;

        /* GUI Monitor */
        mcb->cliCfg[0].guiMonSel.detectedObjects = 1;
        mcb->cliCfg[0].guiMonSel.logMagRange = 1;
        mcb->cliCfg[0].guiMonSel.noiseProfile = 1;

        /*Multi Object Beam forming */
        mcb->cliCfg[0].multiObjBeamFormingCfg.enabled = 1;
        mcb->cliCfg[0].multiObjBeamFormingCfg.multiPeakThrsScal = 0.5;

        /*Extended Max Velocity */
        mcb->cliCfg[0].extendedMaxVelocityCfg.enabled = 0;

        /*Calib DC range Sig CFG */
        mcb->cliCfg[0].calibDcRangeSigCfg.enabled = 0;
        mcb->cliCfg[0].calibDcRangeSigCfg.negativeBinIdx = -5;
        mcb->cliCfg[0].calibDcRangeSigCfg.positiveBinIdx = 8;
        mcb->cliCfg[0].calibDcRangeSigCfg.numAvgChirps = 256;

        /*RX chan and phase comp*/
        mcb->cliCommonCfg.compRxChanCfg.rangeBias = 0.0;
        int io = 0;
        cmplx16ImRe_t fillval;
        fillval.imag = 0;
        fillval.real = 1;
        for (io = 0; io < 12; io++) {
            mcb->cliCommonCfg.compRxChanCfg.rxChPhaseComp[io] = fillval;
        }

    }

    return;

}

/**
 *  @b Description
 *  @n
 *      System Initialization Task which initializes the various
 *      components in the system.
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_dssInitTask(UArg arg0, UArg arg1)
{
    int32_t             errCode;
    MMWave_InitCfg      initCfg;
    Task_Params         taskParams;
    UART_Params         uartParams;
    Semaphore_Params    semParams;
    Mailbox_Config      mboxCfg;
    Error_Block         eb;
    MmwDemo_message     message;

    /* Initialize the UART */
    UART_init();

    /* Initialize the Mailbox */
    Mailbox_init(MAILBOX_TYPE_DSS);

    /*****************************************************************************
     * Open & configure the drivers:
     *****************************************************************************/

    /* Setup the default logging UART Parameters */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.clockFrequency  = DSS_SYS_VCLK;
    uartParams.baudRate        = gMmwDssMCB.cfg.loggingBaudRate;
    uartParams.isPinMuxDone    = 1;

    /* Open the Logging UART Instance: */
    gMmwDssMCB.loggingUartHandle = UART_open(0, &uartParams);
    if (gMmwDssMCB.loggingUartHandle == NULL)
    {
        System_printf("Error: Unable to open the Logging UART Instance\n");
        return;
    }
    System_printf("Debug: Logging UART Instance %p has been opened successfully\n", gMmwDssMCB.loggingUartHandle);
    MCBInit(&gMmwDssMCB);
    /*****************************************************************************
     * Create mailbox Semaphore:
     *****************************************************************************/
    /* Create a binary semaphore which is used to handle mailbox interrupt. */
    Semaphore_Params_init(&semParams);
    semParams.mode             = Semaphore_Mode_BINARY;
    gMmwDssMCB.mboxSemHandle = Semaphore_create(0, &semParams, NULL);

    /* Setup the default mailbox configuration */
    Mailbox_Config_init(&mboxCfg);

    /* Setup the configuration: */
    mboxCfg.chType       = MAILBOX_CHTYPE_MULTI;
    mboxCfg.chId         = MAILBOX_CH_ID_0;
    mboxCfg.writeMode    = MAILBOX_MODE_BLOCKING;
    mboxCfg.readMode     = MAILBOX_MODE_CALLBACK;
    mboxCfg.readCallback = &MmwDemo_mboxCallback;

    gMmwDssMCB.peerMailbox = Mailbox_open(MAILBOX_TYPE_MSS, &mboxCfg, &errCode);
    if (gMmwDssMCB.peerMailbox == NULL)
    {
        /* Error: Unable to open the mailbox */
        System_printf("Error: Unable to open the Mailbox to the DSS [Error code %d]\n", errCode);
        return;
    }

     /* Debug Message: */
    System_printf("Debug: DSS Mailbox Handle %p\n", gMmwDssMCB.peerMailbox);

    /* Create task to handle mailbox messges */
    Task_Params_init(&taskParams);
    Task_create(MmwDemo_mboxReadTask, &taskParams, NULL);

    /*****************************************************************************
     * Create Event to handle mmwave callback and system datapath events
     *****************************************************************************/
    /* Default instance configuration params */
    Error_init(&eb);
    gMmwDssMCB.eventHandle = Event_create(NULL, &eb);
    if (gMmwDssMCB.eventHandle == NULL)
    {
        /* FATAL_TBA */
        System_printf("Error: MMWDemoDSS Unable to create an event handle\n");
        return ;
    }
    System_printf("Debug: MMWDemoDSS create event handle succeeded\n");

    /************************************************************************
     * mmwave library initialization
     ************************************************************************/

    /* Populate the init configuration for mmwave library: */
    initCfg.domain                      = MMWave_Domain_DSS;
    initCfg.socHandle                   = gMmwDssMCB.socHandle;
    initCfg.eventFxn                    = MmwDemo_dssMmwaveEventCallbackFxn;
    initCfg.linkCRCCfg.useCRCDriver     = 1U;
    initCfg.linkCRCCfg.crcChannel       = CRC_Channel_CH1;
    initCfg.cfgMode                     = MMWave_ConfigurationMode_FULL;
    initCfg.executionMode               = MMWave_ExecutionMode_COOPERATIVE;
    initCfg.cooperativeModeCfg.cfgFxn   = MmwDemo_dssMmwaveConfigCallbackFxn;
    initCfg.cooperativeModeCfg.startFxn = MmwDemo_dssMmwaveStartCallbackFxn;
    initCfg.cooperativeModeCfg.stopFxn  = MmwDemo_dssMmwaveStopCallbackFxn;
    initCfg.cooperativeModeCfg.openFxn  = MmwDemo_dssMmwaveOpenCallbackFxn;
    initCfg.cooperativeModeCfg.closeFxn = MmwDemo_dssMmwaveCloseCallbackFxn;

    /* Initialize and setup the mmWave Control module */
    gMmwDssMCB.ctrlHandle = MMWave_init (&initCfg, &errCode);
    if (gMmwDssMCB.ctrlHandle == NULL)
    {
        /* Error: Unable to initialize the mmWave control module */
        System_printf ("Error: MMWDemoDSS mmWave Control Initialization failed [Error code %d]\n", errCode);
        return;
    }
    System_printf ("Debug: MMWDemoDSS mmWave Control Initialization succeeded\n");

    /******************************************************************************
     * TEST: Synchronization
     * - The synchronization API always needs to be invoked.
     ******************************************************************************/
    while (1)
    {
        int32_t syncStatus;

        /* Get the synchronization status: */
        syncStatus = MMWave_sync (gMmwDssMCB.ctrlHandle , &errCode);
        if (syncStatus < 0)
        {
            /* Error: Unable to synchronize the mmWave control module */
            System_printf ("Error: MMWDemoDSS mmWave Control Synchronization failed [Error code %d]\n", errCode);
            return;
        }
        if (syncStatus == 1)
        {
            /* Synchronization acheived: */
            break;
        }
        /* Sleep and poll again: */
        Task_sleep(1);
    }

    /* Send the DSS to MSS signalling ISR payload address to the DSS. Note this
       should be done after both sides mailboxes have been opened, and because
       MMWave_sync above is a good one to check for synchronization, this is a good place */
    message.type = MMWDEMO_DSS2MSS_ISR_INFO_ADDRESS;
    message.body.dss2mssISRinfoAddress = (uint32_t) &gHSRAM.dss2MssIsrInfo;
    MmwDemo_mboxWrite(&message);

    /*****************************************************************************
     * Launch the mmWave control execution task
     * - This should have a higher priority than any other task which uses the
     *   mmWave control API
     *****************************************************************************/
    Task_Params_init(&taskParams);
    taskParams.priority = 6;
    taskParams.stackSize = 4*1024;
    Task_create(MmwDemo_dssMMWaveCtrlTask, &taskParams, NULL);

    /*****************************************************************************
     * Data path Startup
     *****************************************************************************/
    if ((errCode = MmwDemo_dssDataPathInit()) < 0 )
    {
        System_printf ("Error: MMWDemoDSS Data Path init failed with Error[%d]\n",errCode);
        return;
    }
    System_printf ("Debug: MMWDemoDSS Data Path init succeeded\n");
    gMmwDssMCB.state = MmwDemo_DSS_STATE_INIT;

    /* Start data path task */
    Task_Params_init(&taskParams);
    taskParams.priority = 5;
    taskParams.stackSize = 4*1024;
    Task_create(MmwDemo_dssDataPathTask, &taskParams, NULL);

    System_printf("Debug: MMWDemoDSS initTask exit\n");
    return;
}

/**
 *  @b Description
 *  @n
 *     Function to sleep the DSP using IDLE instruction. When DSP has no work left to do,
 *     the BIOS will be in Idle thread and will call this function. The DSP will
 *     wake-up on any interrupt (e.g chirp interrupt).
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_sleep(void)
{
    /* issue IDLE instruction */
    asm(" IDLE ");
}

/**
 *  @b Description
 *  @n
 *      Sends DSS assert information to MSS
 *
 *  @retval
 *      Not Applicable.
 */
void _MmwDemo_dssAssert(int32_t expression,const char *file, int32_t line)
{
    MmwDemo_message  message;
    uint32_t         nameSize;

    if (!expression) 
    {
        message.type = MMWDEMO_DSS2MSS_ASSERT_INFO;
        nameSize = strlen(file);
        if(nameSize > MMWDEMO_MAX_FILE_NAME_SIZE)
            nameSize = MMWDEMO_MAX_FILE_NAME_SIZE;
            
        memcpy((void *) &message.body.assertInfo.file[0], (void *)file, nameSize);
        message.body.assertInfo.line = (uint32_t)line;
        if (MmwDemo_mboxWrite(&message) != 0)
        {
            System_printf ("Error: Failed to send exception information to MSS.\n");
        }
        
    }    
}        

/**
 *  @b Description
 *  @n
 *      Sends DSS range bias and rx channel phase offset measurement information to MSS
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_measurementResultOutput(MmwDemo_DSS_DataPathObj *obj)
{
    MmwDemo_message  message;

    message.type = MMWDEMO_DSS2MSS_MEASUREMENT_INFO;

    memcpy((void *) &message.body.compRxChanCfg, (void *)&obj->cliCommonCfg->compRxChanCfg, sizeof(MmwDemo_compRxChannelBiasCfg_t));
    if (MmwDemo_mboxWrite(&message) != 0)
    {
        System_printf ("Error: Failed to send measurement information to MSS.\n");
    }
}

/**
 *  @b Description
 *  @n
 *      Entry point into the test code.
 *
 *  @retval
 *      Not Applicable.
 */
int main (void)
{
    Task_Params    taskParams;
    SOC_Cfg        socCfg;
    int32_t        errCode;

    /* Initialize and populate the demo MCB */
    memset ((void*)&gMmwDssMCB, 0, sizeof(MmwDemo_DSS_MCB));

    /* Initialize the SOC confiugration: */
    memset ((void *)&socCfg, 0, sizeof(SOC_Cfg));

    /* Populate the SOC configuration: */
    socCfg.clockCfg = SOC_SysClock_BYPASS_INIT;

    /* Initialize the SOC Module: This is done as soon as the application is started
     * to ensure that the MPU is correctly configured. */
    gMmwDssMCB.socHandle = SOC_init (&socCfg, &errCode);
    if (gMmwDssMCB.socHandle == NULL)
    {
        System_printf ("Error: SOC Module Initialization failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Initialize the DEMO configuration: */
    gMmwDssMCB.cfg.sysClockFrequency = DSS_SYS_VCLK;
    gMmwDssMCB.cfg.loggingBaudRate   = 921600;

    Cycleprofiler_init();

    /* Initialize the Task Parameters. */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 3*1024;
    Task_create(MmwDemo_dssInitTask, &taskParams, NULL);

    /* Start BIOS */
    BIOS_start();
    return 0;
}

