/**
 *   @file  mmw_config.h
 *
 *   @brief
 *      This is the header file that describes configurations for the Millimeter
 *   Wave Demo.
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
#ifndef MMW_CONFIG_H
#define MMW_CONFIG_H

/* MMWAVE library Include Files */
#include <ti/control/mmwave/mmwave.h>
#include <gtrack.h>
#include <chains/RadarReceiverPeopleCounting/radarProcess.h>

#ifdef __cplusplus
extern "C" {
#endif

#define mmwDemoCfarConfig radarModuleCfarConfig
#define mmwDemoDoaConfig radarModuleDoaConfig

/**
 *  @b Description
 *  @n
 *      Structure stores meta information for detected objects.
 *
 *
 *  @details
 *      This structure holds the meta information for detected Objects.
 */
typedef struct MmwDemo_detOutputHdr_t
{
    /*! brief   Output buffer magic word */
    uint16_t    magicWord[4];

    /*! brief   chirp processing margin in usec */
    uint32_t    chirpProcessingMarginInUsec;

    /*! brief   frame processing margin in usec */
    uint32_t    frameProcessingMarginInUsec;

    /*! brief   Number of detected Objects */
    uint32_t     numDetectedObj;

    /*! brief   Frame number */
    uint32_t     frameNumber;
}MmwDemo_detOutputHdr;

/**
 * @brief
 *  ADCBUF configuration
 *
 * @details
 *  The structure is used to hold all the relevant configuration
 *  which is used to configure ADCBUF.
 */
typedef struct MmwDemo_ADCBufCfg_t
{
    /*! ADCBUF out format:
        0-Complex,
        1-Real */
    uint8_t     adcFmt;

    /*! ADCBUF IQ swap selection:
        0-I in LSB, Q in MSB,
        1-Q in LSB, I in MSB */
    uint8_t     iqSwapSel;

    /*! ADCBUF channel interleave configuration:
        0-interleaved(not supported on XWR16xx),
        1- non-interleaved */
    uint8_t     chInterleave;

    /**
     * @brief   Chirp Threshold configuration used for ADCBUF buffer
     */
    uint8_t     chirpThreshold;
}MmwDemo_ADCBufCfg;

/**
 * @brief
 *  Millimeter Wave Demo Gui Monitor Selection
 *
 * @details
 *  The structure contains the flags which select what information is sent to gui
 *
 */
typedef struct MmwDemo_GuiMonSel_t
{
    /*! @brief   Send list of detected objects (structure outObj) */
    uint8_t        detectedObjects;

    /*! @brief   Send log magnitude range array  */
    uint8_t        logMagRange;

    /*! @brief   Send complex range bins at zero doppler, all antenna symbols for range-azimuth heat map */
    uint8_t        rangeAzimuthHeatMap;

    /*! @brief   Send complex range bins at zero doppler, (all antenna symbols), for range-azimuth heat map */
    uint8_t        rangeDopplerHeatMap;
} MmwDemo_GuiMonSel;

/**
 * @brief
 *  Tracking configuration
 *
 * @details
 *  The structure is used to hold all the relevant configuration
 *  which is used to configure Tracking module
 */
typedef struct MmwDemo_TrackingCfg_t
{
    GTRACK_moduleConfig         config;
    GTRACK_advancedParameters   params;
}MmwDemo_TrackingCfg;

/**
 * @brief
 *  Application level configuration parameters
 *
 * @details
 *  The structure is used to hold all the relevant configuration
 *  which is used by application task
 */
typedef struct MmwDemo_ApplicationCfg_t
{
    float sensorAzimuthTilt;
    float leftWallPos;
    float rightWallPos;
}MmwDemo_ApplicationCfg;

/**
 * @brief
 *  Millimeter Wave Demo configuration
 *
 * @details
 *  The structure is used to hold all the relevant configuration
 *  which is used to execute the Millimeter Wave Demo.
 */
typedef struct MmwDemo_Cfg_t
{
    /*! @brief   CPU Clock Frequency. */
    uint32_t            sysClockFrequency;

    /*! @brief   UART Logging Baud Rate. */
    uint32_t            loggingBaudRate;

    /*! @brief   UART Command Baud Rate. */
    uint32_t            commandBaudRate;

    /*! @brief   mmWave Control Configuration. */
    MMWave_CtrlCfg      ctrlCfg;

    /*! @brief   mmWave Open Configuration. */
    MMWave_OpenCfg      openCfg;

    /*! @brief   Gui Monitor Selection */
    MmwDemo_GuiMonSel   guiMonSel;

    /*! @brief   CFAR Configuration */
    mmwDemoCfarConfig     cfarCfg;

    /*! @brief   DOA Configuration */
    mmwDemoDoaConfig     doaCfg;

    /*! @brief   ADC Buffer Configuration */
    MmwDemo_ADCBufCfg    adcBufCfg;

    /*! @brief   Tracking Configuration */
    MmwDemo_TrackingCfg		trackingCfg;

    /*! @brief   Application Configuration */
    MmwDemo_ApplicationCfg     applicationCfg;

    /*! @brief   Datapath output loggerSetting
                 0 (default): MSS UART logger
                 1: DSS UART logger
     */
    uint8_t              dataLogger;
} MmwDemo_Cfg;

#ifdef __cplusplus
}
#endif

#endif /* MMW_CONFIG_H */
