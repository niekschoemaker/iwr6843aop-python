/**
 *   @file  mss_mmw.h
 *
 *   @brief
 *      This is the main header file for the Millimeter Wave Demo
 *
 *  Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/ 
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
#ifndef MSS_MMW_DEMO_H
#define MSS_MMW_DEMO_H

#include <ti/common/mmwave_error.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/crc/crc.h>
#include <ti/drivers/uart/UART.h>
#include <ti/drivers/pinmux/pinmux.h>
#include <ti/drivers/esm/esm.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/mailbox/mailbox.h>
#include <ti/control/mmwave/mmwave.h>

#include <ti/sysbios/knl/Semaphore.h>

/* MMW Demo Include Files */
//#include <ti/demo/io_interface/mmw_config.h>
#include <mmw_output.h>
#include <mmw_config.h>
#include <radarProcess.h>
#include <gtrack.h>


#ifdef __cplusplus
extern "C" {
#endif

/*! @brief Version of the MMW Demo. */
#define MMW_VERSION        "0.0.0.2"

/*! @brief   sensor start CLI event */
#define MMWDEMO_CLI_SENSORSTART_EVT                     Event_Id_00

/*! @brief   sensor stop CLI  event */
#define MMWDEMO_CLI_SENSORSTOP_EVT                      Event_Id_01

/*! @brief   sensor frame start CLI  event */
#define MMWDEMO_CLI_FRAMESTART_EVT                      Event_Id_02

/*! @brief   BSS CPUFAULT event */
#define MMWDEMO_BSS_CPUFAULT_EVT                        Event_Id_03

/*! @brief   BSS ESMFAULT event */
#define MMWDEMO_BSS_ESMFAULT_EVT                        Event_Id_04

/* All CLI events */
#define MMWDEMO_CLI_EVENTS                              (MMWDEMO_CLI_SENSORSTART_EVT |    \
                                                         MMWDEMO_CLI_SENSORSTOP_EVT |     \
                                                         MMWDEMO_CLI_FRAMESTART_EVT)

/* All BSS faults events */
#define MMWDEMO_BSS_FAULT_EVENTS                        (MMWDEMO_BSS_CPUFAULT_EVT |     \
                                                         MMWDEMO_BSS_ESMFAULT_EVT )

extern radarOsal_heapObj gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_MAXNUMHEAPS];

/*! @brief R4F cycle profiling structure to accumulate different
    processing times getting the results from DSP and optional algorithms such as tracking, and time
    spent on sending final results to UART */
typedef struct cycleLog_t_ {
    float copyResultsTimeCurrInusec; /*!< @brief current time spent on copying results from DSP from shared memory */
    float copyResultsTimeMaxInusec; /*!< @brief maximum time spent on copying results from DSP from shared memory */
    float trackingTimeCurrInusec; /*!< @brief current time spent on tracking */
    float trackingTimeMaxInusec; /*!< @brief maximum time spent on tracking */
    float sendingToUARTTimeCurrInusec; /*!< @brief current time spent on sending final results to UART */
    float sendingToUARTTimeMaxInusec; /*!< @brief maximum time spent on sending final results to UART */
    uint32_t    benchmarks[32]; /*!< @brief locally used to cycle count tracking steps */
} cycleLog_t;


/**
 * @brief
 *  Millimeter Wave Demo statistics
 *
 * @details
 *  The structure is used to hold the statistics information for the
 *  Millimeter Wave demo
 */
typedef struct MmwDemo_MSS_STATS_t
{
    /*! @brief   CLI event for sensorStart */
    uint8_t      cliSensorStartEvt;

    /*! @brief   CLI event for sensorStop */
    uint8_t      cliSensorStopEvt;

    /*! @brief   CLI event for frameStart */
    uint8_t      cliFrameStartEvt;

    /*! @brief   Counter which tracks the number of datapath config event detected
     *           The event is triggered in mmwave config callback function */
    uint8_t      datapathConfigEvt;

    /*! @brief   Counter which tracks the number of failed calibration reports
     *           The event is triggered by an asynchronous event from the BSS */
    uint32_t     numFailedTimingReports;

    /*! @brief   Counter which tracks the number of buffer size error from BSS */
    uint32_t     numWrongBSSReportSize;

    /*! @brief   Counter which tracks the number of calibration reports received
     *           The event is triggered by an asynchronous event from the BSS */
    uint32_t     numCalibrationReports;

    /*! @brief   Counter which tracks the number of datapath start event  detected 
     *           The event is triggered in mmwave start callback function */
    uint8_t      datapathStartEvt;

    /*! @brief   Counter which tracks the number of datapath stop event detected 
     *           The event is triggered in mmwave stop callback function */
    uint8_t      datapathStopEvt;
}MmwDemo_MSS_STATS;


/**
 * @brief
 *  Millimeter Wave Demo input from DSP
 *
 * @details
 *  The structure describes the format of the detection data
 */

typedef struct inputFromDSP_t
{
    MmwDemo_detOutputHdr header;
    radarProcessOutputToTracker outputToTracker;
} inputFromDSP;

/**
 * @brief
 *  Millimeter Wave Demo Data Path Information.
 *
 * @details
 *  The structure is used to hold all the relevant information for
 *  the data path.
 */
typedef struct MmwDemo_MSS_DataPathObj_t
{
	inputFromDSP inputInfo;

	uint32_t inputInfoBuffSize;
	
	uint8_t groupTrackerEnabled;

	RADARDEMO_clusterTracker_errorCode trackingErrorCode;
	void  * trackingInstance;
	uint8_t  tracking_enabled;
	RADARDEMO_tracker_input * trackingInputData;
	RADARDEMO_tracker_output * trackingOutputData;
	float  trackingCallPeriod;

	float * heatmapAddress;
    uint32_t   heatmapRowLen;
    uint32_t   heatmapNumRows;
	
	cycleLog_t cycleLog;

}MmwDemo_MSS_DataPathObj;



/**
 * @brief
 *  Millimeter Wave Demo MCB
 *
 * @details
 *  The structure is used to hold all the relevant information for the
 *  Millimeter Wave demo
 */
typedef struct MmwDemo_MCB_t
{
    /*! @brief   Configuration which is used to execute the demo */
    MmwDemo_Cfg                 cfg;

    /*! * @brief   Handle to the SOC Module */
    SOC_Handle                  socHandle;

    /*! @brief   UART Logging Handle */
    UART_Handle                 loggingUartHandle;

    /*! @brief   UART Command Rx/Tx Handle */
    UART_Handle                 commandUartHandle;

    /*! @brief   This is the mmWave control handle which is used
     * to configure the BSS. */
    MMWave_Handle               ctrlHandle;

    /*!@brief   Handle to the peer Mailbox */
    Mbox_Handle              peerMailbox;

    /*! @brief   Semaphore handle for the mailbox communication */
    Semaphore_Handle            mboxSemHandle;


    /*! @brief   Semaphore handle for the application task */
    Semaphore_Handle                    appSemHandle;
	
    /*! @brief   Point Cloud */
    MmwDemo_output_message_pointCloud   *pointCloud;

    /*! @brief   Target Descriptors */
    MmwDemo_targetDescrHandle   *targetDescrHandle;

    /*! @brief   Tracker Handle */
    void     					        *gtrackHandle;

    /*! @brief   MSS system event handle */
    Event_Handle                eventHandle;

    /*! @brief   Handle to the SOC chirp interrupt listener Handle */
    SOC_SysIntListenerHandle    chirpIntHandle;

    /*! @brief   Handle to the SOC frame start interrupt listener Handle */
    SOC_SysIntListenerHandle    frameStartIntHandle;

    /*! @brief   Data Path object: currently only for receiving data from DSS. Potentially adding other block such as tracking here*/
    MmwDemo_MSS_DataPathObj     mssDataPathObj;

    /*! @brief   Has the mmWave module been opened? */
    bool                        isMMWaveOpen;

    /*! @brief   mmw Demo stats */
    MmwDemo_MSS_STATS           stats;

    //GTRACK_targetDesc *         trackerOut;
//
//    int32_t numPoints;
} MmwDemo_MCB;

/**************************************************************************
 *************************** Extern Definitions ***************************
 **************************************************************************/
extern int32_t MmwDemo_mssDataPathConfig(void);
extern int32_t MmwDemo_mssDataPathStart(void);
extern int32_t MmwDemo_mssDataPathStop(void);

#ifdef __cplusplus
}
#endif

#endif /* MSS_MMW_DEMO_H */

