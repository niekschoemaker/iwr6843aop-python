/**
 *   @file  mss_main.c
 *
 *   @brief
 *     MSS main implementation of the millimeter wave Demo
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
#include <ti/sysbios/family/arm/v7a/Pmu.h>
#include <ti/sysbios/family/arm/v7r/vim/Hwi.h>

/* mmWave SDK Include Files: */
#include <ti/common/sys_common.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/esm/esm.h>
#include <ti/drivers/crc/crc.h>
#include <ti/drivers/uart/UART.h>
#include <ti/drivers/gpio/gpio.h>
#include <ti/drivers/mailbox/mailbox.h>
#include <ti/control/mmwave/mmwave.h>
#include <ti/utils/cli/cli.h>
#include <ti/drivers/osal/DebugP.h>
#include <ti/drivers/osal/HwiP.h>
#include <ti/utils/cycleprofiler/cycle_profiler.h>

/* Demo Include Files */
#include "mss_mmw.h"
#include <mmw_messages.h>


/**************************************************************************
 *************************** Local Definitions ****************************
 **************************************************************************/

/**************************************************************************
 *************************** Global Definitions ***************************
 **************************************************************************/

/**
 * @brief
 *  Global Variable for tracking information required by the mmw Demo
 */
MmwDemo_MCB    gMmwMssMCB;


#define SOC_XWR16XX_MSS_MAXNUMHEAPS (RADARMEMOSAL_HEAPTYPE_MAXNUMHEAPS)
#define SOC_XWR16XX_MSS_L1_SCRATCH_SIZE           0x2100U
#define SOC_XWR16XX_MSS_L2_BUFF_SIZE              0x2000U
#define SOC_XWR16XX_MSS_L3RAM_BUFF_SIZE           0x0000U

/*! L2 RAM buffer */
#pragma DATA_SECTION(gMmwL2, ".l2data");
#pragma DATA_ALIGN(gMmwL2, 8);
uint8_t gMmwL2[SOC_XWR16XX_MSS_L2_BUFF_SIZE];

/*! L2 RAM scratch */
#pragma DATA_SECTION(gMmwL1Scratch, ".l2data");
#pragma DATA_ALIGN(gMmwL1Scratch, 8);
uint8_t gMmwL1Scratch[SOC_XWR16XX_MSS_L1_SCRATCH_SIZE];




/**************************************************************************
 *************************** Extern Definitions *******************************
 **************************************************************************/
/* CLI Init function */
extern void MmwDemo_CLIInit (void);

/**************************************************************************
 ************************* Millimeter Wave Demo Functions Prototype**************
 **************************************************************************/

/* Data path functions */
int32_t MmwDemo_mssDataPathConfig(void);
int32_t MmwDemo_mssDataPathStart(void);
int32_t MmwDemo_mssDataPathStop(void);

/* mmwave library call back fundtions */
static void MmwDemo_mssMmwaveConfigCallbackFxn(MMWave_CtrlCfg* ptrCtrlCfg);
static void MmwDemo_mssMmwaveStartCallbackFxn(MMWave_CalibrationCfg* ptrCalibrationCfg);
static void MmwDemo_mssMmwaveOpenCallbackFxn(MMWave_OpenCfg* ptrOpenCfg);
static void MmwDemo_mssMmwaveCloseCallbackFxn(void);
static void MmwDemo_mssMmwaveStopCallbackFxn(void);
static int32_t MmwDemo_mssMmwaveEventCallbackFxn(uint16_t msgId, uint16_t sbId, uint16_t sbLen, uint8_t *payload);

/* MMW demo Task */
void MmwDemo_mssInitTask(UArg arg0, UArg arg1);
void MmwDemo_mmWaveCtrlTask(UArg arg0, UArg arg1);
void MmwDemo_mssCtrlPathTask(UArg arg0, UArg arg1);

void MmwDemo_mboxReadTask(UArg arg0, UArg arg1);
void MmwDemo_appTask(UArg arg0, UArg arg1);

/**************************************************************************
 ************************* Millimeter Wave Demo Functions **********************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      Registered event function which is invoked when an event from the
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
 *      Always returns 0 [Continue passing the event to the peer domain]
 */
static int32_t MmwDemo_mssMmwaveEventCallbackFxn(uint16_t msgId, uint16_t sbId, uint16_t sbLen, uint8_t *payload)
{
    uint16_t asyncSB = RL_GET_SBID_FROM_UNIQ_SBID(sbId);

#if 0
    System_printf ("Debug: BSS Event MsgId: %d [Sub Block Id: %d Sub Block Length: %d]\n",
                    msgId, sbId, sbLen);
#endif

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
                    /* Post event to datapath task notify BSS events */
                    Event_post(gMmwMssMCB.eventHandle, MMWDEMO_BSS_CPUFAULT_EVT);
                    break;
                }
                case RL_RF_AE_ESMFAULT_SB:
                {
                    /* Post event to datapath task notify BSS events */
                    Event_post(gMmwMssMCB.eventHandle, MMWDEMO_BSS_ESMFAULT_EVT);
                    break;
                }
                case RL_RF_AE_INITCALIBSTATUS_SB:
                {
                    /* This event should be handled by mmwave internally, ignore the event here */
                    break;
                }

                case RL_RF_AE_FRAME_TRIGGER_RDY_SB:
                {
                    /* This event is not handled on MSS */
                    break;
                }
                case RL_RF_AE_MON_TIMING_FAIL_REPORT_SB:
                {
                    /* Increment the statistics for the number of failed reports */
                    gMmwMssMCB.stats.numFailedTimingReports++;

				    #if 0 /*Need to decide what (if anything) to do when this event is received
					        if something needs to be done then need to implement the function
							to handle the event below in MmwDemo_mssCtrlPathTask()*/

                    /* Post event to datapath task notify BSS events */
                    Event_post(gMmwMssMCB.eventHandle, MMWDEMO_BSS_MONITORING_REP_EVT);
					#endif
                    break;
                }
                case RL_RF_AE_RUN_TIME_CALIB_REPORT_SB:
                {
                    /* Increment the statistics for the number of received calibration reports */
                    gMmwMssMCB.stats.numCalibrationReports++;

				    #if 0 /*Need to decide what (if anything) to do when this event is received
					        if something needs to be done then need to implement the function
							to handle the event below in MmwDemo_mssCtrlPathTask()*/

                    /* Post event to datapath task notify BSS events */
                    Event_post(gMmwMssMCB.eventHandle, MMWDEMO_BSS_CALIBRATION_REP_EVT);
					#endif
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
 *      Application registered callback function which is invoked after the configuration
 *      has been used to configure the mmWave link and the BSS. This is applicable only for
 *      the XWR16xx. The BSS can be configured only by the MSS *or* DSS. The callback API is
 *      triggered on the remote execution domain (which did not configure the BSS)
 *
 *  @param[in]  ptrCtrlCfg
 *      Pointer to the control configuration
 *
 *  @retval
 *      Not applicable
 */
static void MmwDemo_mssMmwaveConfigCallbackFxn(MMWave_CtrlCfg* ptrCtrlCfg)
{
    /* For mmw Demo, mmwave_config() will always be called from MSS,
       due to the fact CLI is running on MSS, hence this callback won't be called */
    gMmwMssMCB.stats.datapathConfigEvt ++;
}

/**
 *  @b Description
 *  @n
 *      Application registered callback function which is invoked the mmWave link on BSS
 *      has been opened.
 *
 *  @param[in]  ptrOpenCfg
 *      Pointer to the open configuration
 *
 *  @retval
 *      Not applicable
 */
static void MmwDemo_mssMmwaveOpenCallbackFxn(MMWave_OpenCfg* ptrOpenCfg)
{
    return;
}

/**
 *  @b Description
 *  @n
 *      Application registered callback function which is invoked the mmWave link on BSS
 *      has been closed.
 *
 *  @retval
 *      Not applicable
 */
static void MmwDemo_mssMmwaveCloseCallbackFxn(void)
{
    return;
}

/**
 *  @b Description
 *  @n
 *      Application registered callback function which is invoked the mmWave link on BSS
 *      has been started. This is applicable only for the XWR16xx. The BSS can be configured
 *      only by the MSS *or* DSS. The callback API is triggered on the remote execution
 *      domain (which did not configure the BSS)
 *
 *  @retval
 *      Not applicable
 */
static void MmwDemo_mssMmwaveStartCallbackFxn(MMWave_CalibrationCfg* ptrCalibrationCfg)
{
    /* Post an event to main data path task.
       This function in only called when mmwave_start() is called on DSS */
    gMmwMssMCB.stats.datapathStartEvt ++;
}

/**
 *  @b Description
 *  @n
 *      Application registered callback function which is invoked the mmWave link on BSS
 *      has been stopped. This is applicable only for the XWR16xx. The BSS can be configured
 *      only by the MSS *or* DSS. The callback API is triggered on the remote execution
 *      domain (which did not configure the BSS)
 *
 *  @retval
 *      Not applicable
 */
static void MmwDemo_mssMmwaveStopCallbackFxn(void)
{
    /* Possible sceanarios:
       1. CLI sensorStop command triggers mmwave_stop() to be called from MSS
       2. In case of Error, mmwave_stop() will be triggered either from MSS or DSS
     */
    gMmwMssMCB.stats.datapathStopEvt ++;
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
    Semaphore_post (gMmwMssMCB.mboxSemHandle);
}

void MmwDemo_printHeapStats(void)
{
	Memory_Stats            startMemoryStats;
	
    HeapMem_getStats (heap0, &startMemoryStats);
    System_printf ("Debug: System Heap (TCM): Size: %d, Used = %d, Free = %d bytes\n", startMemoryStats.totalSize, startMemoryStats.totalSize - startMemoryStats.totalFreeSize, startMemoryStats.totalFreeSize);
}


/**
 *  @b Description
 *  @n
 *      Function to do Data Path Configuration on MSS. After received Configuration from
 *    CLI, this function will start the system configuration process, inclucing mmwaveLink, BSS
 *    and DSS.
 *
 *  @retval
 *      0  - Success.
 *      <0 - Failed with errors
 */
int32_t MmwDemo_mssDataPathConfig(void)
{
    int32_t  errCode;

    /* Has the mmWave module been opened? */
    if (gMmwMssMCB.isMMWaveOpen == false)
    {
        /*Get the open config from CLI*/
        CLI_getMMWaveExtensionOpenConfig (&gMmwMssMCB.cfg.openCfg);

        /* NO: Setup the calibration frequency: */
        gMmwMssMCB.cfg.openCfg.freqLimitLow  = 0U;
        gMmwMssMCB.cfg.openCfg.freqLimitHigh = 0U;

        /*No custom calibration*/
        gMmwMssMCB.cfg.openCfg.useCustomCalibration         = false;
        gMmwMssMCB.cfg.openCfg.customCalibrationEnableMask  = 0x0;

        /* Open the mmWave module: */
        if (MMWave_open (gMmwMssMCB.ctrlHandle, &gMmwMssMCB.cfg.openCfg, NULL, &errCode) < 0)
        {
            System_printf ("Error: MMWDemoMSS mmWave open configuration failed [Error code %d]\n", errCode);
            return -1;
        }
        System_printf("MMWDemoMSS open config success!\n");

        /* mmWave module has been opened. */
        gMmwMssMCB.isMMWaveOpen = true;
    }

    CLI_getMMWaveExtensionConfig (&gMmwMssMCB.cfg.ctrlCfg);
    /* Configure the mmWave module: */
    if (MMWave_config (gMmwMssMCB.ctrlHandle, &gMmwMssMCB.cfg.ctrlCfg, &errCode) < 0)
    {
        System_printf ("Error: MMWDemoMSS mmWave Configuration failed [Error code %d]\n", errCode);
        return -1;
    }
    System_printf("MMWDemoMSS config success!\n");

	memset(&(gMmwMssMCB.mssDataPathObj.cycleLog), 0, sizeof(gMmwMssMCB.mssDataPathObj.cycleLog));
	//other init for data path
	gMmwMssMCB.mssDataPathObj.groupTrackerEnabled = 1;
	
	MmwDemo_printHeapStats();

    System_printf ("Debug: MMWDemoMSS mmWave  config succeeded \n");

    return 0;
}

/**
 *  @b Description
 *  @n
 *      Function to do Data Path Start on MSS. After received SensorStart command, MSS will
 *    start all data path componets including mmwaveLink, BSS and DSS.
 *
 *  @retval
 *      0  - Success.
 *      <0 - Failed with errors
 */
int32_t MmwDemo_mssDataPathStart(void)
{
    int32_t                 errCode;
    MMWave_CalibrationCfg   calibrationCfg;

    /* Initialize the calibration configuration: */
    memset ((void*)&calibrationCfg, 0, sizeof(MMWave_CalibrationCfg));

    /* Populate the calibration configuration: */
    calibrationCfg.dfeDataOutputMode                          = MMWave_DFEDataOutputMode_FRAME;
    calibrationCfg.u.chirpCalibrationCfg.enableCalibration    = true;
    calibrationCfg.u.chirpCalibrationCfg.enablePeriodicity    = true;
    calibrationCfg.u.chirpCalibrationCfg.periodicTimeInFrames = 10U;

    /* Start the mmWave module: The configuration has been applied successfully. */
    if (MMWave_start (gMmwMssMCB.ctrlHandle, &calibrationCfg, &errCode) < 0)
    {
        /* Error: Unable to start the mmWave control */
        System_printf ("Error: MMWDemoMSS mmWave Start failed [Error code %d]\n", errCode);
        return -1;
    }

    System_printf ("Debug: MMWDemoMSS mmWave Start succeeded\n");
    return 0;
}


/**
 *  @b Description
 *  @n
 *      Function to do Data Path Start on MSS. After received SensorStart command, MSS will
 *    start all data path componets including mmwaveLink, BSS and DSS.
 *
 *  @retval
 *      0  - Success.
 *      <0 - Failed with errors
 */
int32_t MmwDemo_mssDataPathStop(void)
{
    int32_t    errCode;

    /* Start the mmWave module: The configuration has been applied successfully. */
    if (MMWave_stop (gMmwMssMCB.ctrlHandle, &errCode) < 0)
    {
        /* Error: Unable to start the mmWave control */
        System_printf ("Error: MMWDemoMSS mmWave Stop failed [Error code %d]\n", errCode);
        return -1;
    }

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
void MmwDemo_mmWaveCtrlTask(UArg arg0, UArg arg1)
{
    int32_t errCode;

    while (1)
    {
        /* Execute the mmWave control module: */
        if (MMWave_execute (gMmwMssMCB.ctrlHandle, &errCode) < 0)
            System_printf ("Error: mmWave control execution failed [Error code %d]\n", errCode);
    }
}

/**
 *  @b Description
 *  @n
 *      The task is used to process data path events
 *      control task
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_mssCtrlPathTask(UArg arg0, UArg arg1)
{
    UInt          event;

    /* Data Path management task Main loop */
    while (1)
    {
        event = Event_pend(gMmwMssMCB.eventHandle,
                          Event_Id_NONE,
                          MMWDEMO_CLI_EVENTS | MMWDEMO_BSS_FAULT_EVENTS,
                          BIOS_WAIT_FOREVER);

        /************************************************************************
         * CLI event:: SensorStart
         ************************************************************************/

        if(event & MMWDEMO_CLI_SENSORSTART_EVT)
        {
            System_printf ("Debug: MMWDemoMSS Received CLI sensorStart Event\n");

            /* Setup the data path: */
            if(MmwDemo_mssDataPathConfig () < 0)
            {
                continue;
            }
        }

        /************************************************************************
         * CLI event:: SensorStop
         ************************************************************************/
        if(event & MMWDEMO_CLI_SENSORSTOP_EVT)
        {
            if (MmwDemo_mssDataPathStop() < 0 )
            {
                continue;
            }
        }

        /************************************************************************
         * CLI event:: Framestart
         ************************************************************************/
        if(event & MMWDEMO_CLI_FRAMESTART_EVT)
        {
            if (MmwDemo_mssDataPathStart() < 0 )
            {
                continue;
            }
        }

        /************************************************************************
         * BSS event:: CPU fault
         ************************************************************************/
        if(event & MMWDEMO_BSS_CPUFAULT_EVT)
        {
            DebugP_assert(0);
            break;
        }

        /************************************************************************
         * BSS event:: ESM fault
         ************************************************************************/
        if(event & MMWDEMO_BSS_ESMFAULT_EVT)
        {
            DebugP_assert(0);
            break;
        }
    }

    System_printf("Debug: MMWDemoDSS Data path exit\n");
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
void MmwDemo_mssInitTask(UArg arg0, UArg arg1)
{
    int32_t             errCode;
    MMWave_InitCfg      initCfg;
    UART_Params         uartParams;
    Task_Params         taskParams;
    Semaphore_Params    semParams;
    Mailbox_Config      mboxCfg;

    DMA_Params          dmaParams;
    DMA_Handle          dmaHandle;

    Error_Block         eb;

    /* Debug Message: */
    System_printf("Debug: MMWDemoMSS Launched the Initialization Task\n");
    MmwDemo_printHeapStats();

    /*****************************************************************************
     * Initialize the mmWave SDK components:
     *****************************************************************************/
    /* Pinmux setting */

    /* Setup the PINMUX to bring out the UART-1 */
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINN5_PADBE, SOC_XWR68XX_PINN5_PADBE_MSS_UARTA_TX);
    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINN5_PADBE, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINN4_PADBD, SOC_XWR68XX_PINN4_PADBD_MSS_UARTA_RX);
    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINN4_PADBD, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);

    /* Setup the PINMUX to bring out the MSS UART-3 */
    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINF14_PADAJ, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINF14_PADAJ, SOC_XWR68XX_PINF14_PADAJ_MSS_UARTB_TX);

    /* Setup the PINMUX to bring out the DSS UART */
    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINP8_PADBM, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINP8_PADBM, SOC_XWR68XX_PINP8_PADBM_DSS_UART_TX);

    /* Initialize the UART */
    UART_init();

    /* Initialize the DMA */
    DMA_init ();

    /* Open the DMA Instance */
    DMA_Params_init(&dmaParams);
    dmaHandle = DMA_open(0, &dmaParams, &errCode);
    if (dmaHandle == NULL)
    {
        printf ("Error: Unable to open the DMA Instance [Error code %d]\n", errCode);
        return;
    }


    /* Initialize the Mailbox */
    Mailbox_init(MAILBOX_TYPE_MSS);

    /*****************************************************************************
     * Open & configure the drivers:
     *****************************************************************************/

    /* Setup the default UART Parameters */
    UART_Params_init(&uartParams);
    uartParams.clockFrequency  = gMmwMssMCB.cfg.sysClockFrequency;
    uartParams.baudRate        = gMmwMssMCB.cfg.commandBaudRate;
    uartParams.isPinMuxDone    = 1U;

    /* Open the UART Instance */
    gMmwMssMCB.commandUartHandle = UART_open(0, &uartParams);
    if (gMmwMssMCB.commandUartHandle == NULL)
    {
        System_printf("Error: MMWDemoMSS Unable to open the Command UART Instance\n");
        return;
    }

    /* Setup the default UART Parameters */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.clockFrequency = gMmwMssMCB.cfg.sysClockFrequency;
    uartParams.baudRate       = gMmwMssMCB.cfg.loggingBaudRate;
    uartParams.isPinMuxDone   = 1U;

    uartParams.dmaHandle      = dmaHandle;
    uartParams.txDMAChannel   = 1U;
    uartParams.rxDMAChannel   = 2U;

    /* Open the Logging UART Instance: */
    gMmwMssMCB.loggingUartHandle = UART_open(1, &uartParams);
    if (gMmwMssMCB.loggingUartHandle == NULL)
    {
        System_printf("Error: MMWDemoMSS Unable to open the Logging UART Instance\n");
        return;
    }

    /*****************************************************************************
     * Creating communication channel between MSS & DSS
     *****************************************************************************/

    /* Create a binary semaphore which is used to handle mailbox interrupt. */
    Semaphore_Params_init(&semParams);
    semParams.mode             = Semaphore_Mode_BINARY;
    gMmwMssMCB.mboxSemHandle = Semaphore_create(0, &semParams, NULL);

    /* Setup the default mailbox configuration */
    Mailbox_Config_init(&mboxCfg);

    /* Setup the configuration: */
    mboxCfg.chType       = MAILBOX_CHTYPE_MULTI;
    mboxCfg.chId         = MAILBOX_CH_ID_0;
    mboxCfg.writeMode    = MAILBOX_MODE_BLOCKING;
    mboxCfg.readMode     = MAILBOX_MODE_CALLBACK;
    mboxCfg.readCallback = &MmwDemo_mboxCallback;

    /* Initialization of Mailbox Virtual Channel  */
    gMmwMssMCB.peerMailbox = Mailbox_open(MAILBOX_TYPE_DSS, &mboxCfg, &errCode);
    if (gMmwMssMCB.peerMailbox == NULL)
    {
        /* Error: Unable to open the mailbox */
        System_printf("Error: Unable to open the Mailbox to the DSS [Error code %d]\n", errCode);
        return;
    }

    /* Create task to handle mailbox messages */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 4*1024;
    taskParams.priority = 4;
    Task_create(MmwDemo_mboxReadTask, &taskParams, NULL);

    /*****************************************************************************
     * Create Event to handle mmwave callback and system datapath events
     *****************************************************************************/
    /* Default instance configuration params */
    Error_init(&eb);
    gMmwMssMCB.eventHandle = Event_create(NULL, &eb);
    if (gMmwMssMCB.eventHandle == NULL)
    {
        DebugP_assert(0);
        return ;
    }

    /*****************************************************************************
     * mmWave: Initialization of the high level module
     *****************************************************************************/

    /* Initialize the mmWave control init configuration */
    memset ((void*)&initCfg, 0 , sizeof(MMWave_InitCfg));

    /* Populate the init configuration for mmwave library: */
    initCfg.domain                      = MMWave_Domain_MSS;
    initCfg.socHandle                   = gMmwMssMCB.socHandle;
    initCfg.eventFxn                    = MmwDemo_mssMmwaveEventCallbackFxn;
    initCfg.linkCRCCfg.useCRCDriver     = 1U;
    initCfg.linkCRCCfg.crcChannel       = CRC_Channel_CH1;
    initCfg.cfgMode                     = MMWave_ConfigurationMode_FULL;
    initCfg.executionMode               = MMWave_ExecutionMode_COOPERATIVE;
    initCfg.cooperativeModeCfg.cfgFxn   = MmwDemo_mssMmwaveConfigCallbackFxn;
    initCfg.cooperativeModeCfg.openFxn  = MmwDemo_mssMmwaveOpenCallbackFxn;
    initCfg.cooperativeModeCfg.closeFxn = MmwDemo_mssMmwaveCloseCallbackFxn;
    initCfg.cooperativeModeCfg.startFxn = MmwDemo_mssMmwaveStartCallbackFxn;
    initCfg.cooperativeModeCfg.stopFxn  = MmwDemo_mssMmwaveStopCallbackFxn;

    /* Initialize and setup the mmWave Control module */
    gMmwMssMCB.ctrlHandle = MMWave_init (&initCfg, &errCode);
    if (gMmwMssMCB.ctrlHandle == NULL)
    {
        /* Error: Unable to initialize the mmWave control module */
        System_printf("Error: MMWDemoMSS mmWave Control Initialization failed [Error code %d]\n", errCode);
        return;
    }
    System_printf("Debug: MMWDemoMSS mmWave Control Initialization was successful\n");

    /* Synchronization: This will synchronize the execution of the control module
     * between the domains. This is a prerequiste and always needs to be invoked. */
    while (1)
    {
        int32_t syncStatus;

        /* Get the synchronization status: */
        syncStatus = MMWave_sync (gMmwMssMCB.ctrlHandle , &errCode);
        if (syncStatus < 0)
        {
            /* Error: Unable to synchronize the mmWave control module */
            System_printf ("Error: MMWDemoMSS mmWave Control Synchronization failed [Error code %d]\n", errCode);
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

    /*****************************************************************************
     * Launch the mmWave control execution task
     * - This should have a higher priority than any other task which uses the
     *   mmWave control API
     *****************************************************************************/
    Task_Params_init(&taskParams);
    taskParams.priority = 6;
    taskParams.stackSize = 4*1024;
    Task_create(MmwDemo_mmWaveCtrlTask, &taskParams, NULL);

    /*****************************************************************************
     * Create a data path management task to handle data Path events
     *****************************************************************************/
    Task_Params_init(&taskParams);
    taskParams.priority = 4;
    taskParams.stackSize = 4*1024;
    Task_create(MmwDemo_mssCtrlPathTask, &taskParams, NULL);


    /*****************************************************************************
     * Create an application task to handle high layer processing
     *****************************************************************************/

	/* Create a binary semaphore for application task to pend */
    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;
    gMmwMssMCB.appSemHandle = Semaphore_create(0, &semParams, NULL);

	Task_Params_init(&taskParams);
    taskParams.priority = 2;
    taskParams.stackSize = 8*1024;
    Task_create(MmwDemo_appTask, &taskParams, NULL);

    /*****************************************************************************
     * At this point, MSS and DSS are both up and synced. Configuration is ready to be sent.
     * Start CLI to get configuration from user
     *****************************************************************************/
    MmwDemo_CLIInit();

    /*****************************************************************************
     * Benchmarking Count init
     *****************************************************************************/
    /* Configure benchmark counter */
    Pmu_configureCounter(0, 0x11, FALSE);
    Pmu_startCounter(0);

    return;
}

/**
 *  @b Description
 *  @n
 *      Entry point into the Millimeter Wave Demo
 *
 *  @retval
 *      Not Applicable.
 */
int main (void)
{
    Task_Params     taskParams;
    int32_t         errCode;
    SOC_Cfg         socCfg;

    /* Initialize the ESM: */
    ESM_init(0U); //dont clear errors as TI RTOS does it

    /* Initialize and populate the demo MCB */
    memset ((void*)&gMmwMssMCB, 0, sizeof(MmwDemo_MCB));

    /* Initialize the SOC confiugration: */
    memset ((void *)&socCfg, 0, sizeof(SOC_Cfg));

    /* Populate the SOC configuration: */
    socCfg.clockCfg = SOC_SysClock_INIT;

    /* Initialize the SOC Module: This is done as soon as the application is started
     * to ensure that the MPU is correctly configured. */
    gMmwMssMCB.socHandle   = SOC_init (&socCfg, &errCode);
    if (gMmwMssMCB.socHandle  == NULL)
    {
        System_printf ("Error: SOC Module Initialization failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Initialize the DEMO configuration: */
    gMmwMssMCB.cfg.sysClockFrequency = MSS_SYS_VCLK;
    gMmwMssMCB.cfg.loggingBaudRate   = 921600;
    gMmwMssMCB.cfg.commandBaudRate   = 115200;

    Cycleprofiler_init();

    /* Debug Message: */
    System_printf ("**********************************************\n");
    System_printf ("Debug: Launching the Millimeter Wave Demo\n");
    System_printf ("**********************************************\n");

    /* Initialize the Task Parameters. */
    Task_Params_init(&taskParams);
    taskParams.priority = 3;
    Task_create(MmwDemo_mssInitTask, &taskParams, NULL);

    /* Start BIOS */
    BIOS_start();
    return 0;
}
