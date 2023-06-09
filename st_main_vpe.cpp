/* SigmaStar trade secret */
/* Copyright (c) [2019~2020] SigmaStar Technology.
All rights reserved.

Unless otherwise stipulated in writing, any and all information contained
herein regardless in any format shall remain the sole proprietary of
SigmaStar and be kept in strict confidence
(SigmaStar Confidential Information) by the recipient.
Any unauthorized act including without limitation unauthorized disclosure,
copying, use, reproduction, sale, distribution, modification, disassembling,
reverse engineering and compiling of the contents of SigmaStar Confidential
Information is unlawful and strictly prohibited. SigmaStar hereby reserves the
rights to any and all damages, losses, costs and expenses resulting therefrom.
*/
#include "st_vpe_stress_test.h"

#include "log.h"
#include "mi_hdmi.h"
#include <mi_vdisp.h>
#include <mi_vdisp_datatype.h>
#include <mi_scl.h>
#include <mi_scl_datatype.h>
#include <mi_disp.h>
#include <mi_disp_datatype.h>
#include "mi_sensor.h"
#include "mi_sensor_datatype.h"
#include "mi_vif.h"
#include "mi_isp.h"
#include "json.hpp"

using json = nlohmann::json;
#include "scl_module_node.h"


#define MAX_U32_VALUE (0XFFFFFFFF)


#if (defined CONFIG_SIGMASTAR_CHIP_I6E) && (CONFIG_SIGMASTAR_CHIP_I6E == 1)
        MI_U16 u16VencMaxW = 3840;
        MI_U16 u16VencMaxH = 2160;
        MI_U16 u16JpegMaxW = 3840;
        MI_U16 u16JpegMaxH = 2160;
#elif ((defined CONFIG_SIGMASTAR_CHIP_M6) && (CONFIG_SIGMASTAR_CHIP_M6 == 1))
        MI_U16 u16VencMaxW = 3840;
        MI_U16 u16VencMaxH = 2160;
        MI_U16 u16JpegMaxW = 8192;
        MI_U16 u16JpegMaxH = 4096;
#elif ((defined CONFIG_SIGMASTAR_CHIP_P3) && CONFIG_SIGMASTAR_CHIP_P3 == 1)
        MI_U16 u16VencMaxW = 1280;
        MI_U16 u16VencMaxH = 720;
        MI_U16 u16JpegMaxW = 1280;
        MI_U16 u16JpegMaxH = 720;
#elif ((defined CONFIG_SIGMASTAR_CHIP_I7) && (CONFIG_SIGMASTAR_CHIP_I7 == 1))
        MI_U16 u16VencMaxW = 8192;
        MI_U16 u16VencMaxH = 4096;
        MI_U16 u16JpegMaxW = 8192;
        MI_U16 u16JpegMaxH = 8192;
#elif ((defined CONFIG_SIGMASTAR_CHIP_M6P) && (CONFIG_SIGMASTAR_CHIP_M6P == 1))
        MI_U16 u16VencMaxW = 8192;
        MI_U16 u16VencMaxH = 4096;
        MI_U16 u16JpegMaxW = 8192;
        MI_U16 u16JpegMaxH = 8192;
#else
        MI_U16 u16VencMaxW = 2688;
        MI_U16 u16VencMaxH = 1944;
        MI_U16 u16JpegMaxW = 2688;
        MI_U16 u16JpegMaxH = 1944;
#endif

#define SUB_STREAM0                "video0"
#define SUB_STREAM1                "video1"
#define SUB_STREAM2                "video2"
#define SUB_STREAM3                "video3"


ST_VifModAttr_t   gstVifModule;
ST_IspModeAttr_t gstIspModule;
ST_SclModeAttr_t gstSclModule;
#define IQ_FILE_PATH               "/customer/imx283_api.bin"
#define IQ_FILE1_PATH               "/customer/imx335_api.bin"


static MI_U32 bIQp = 25;
static MI_U32 bPQp = 25;

static Live555RTSPServer *g_pRTSPServer = NULL;
static ST_DynamicTestInfo_t gstDynamicTest;
static MI_S32 gbPreviewByDisp = FALSE;
static MI_S32 gbMutiVpeChnNum = -1;
static ST_Sensor_Attr_t  gstSensorAttr[ST_MAX_SENSOR_NUM];
static ST_VencAttr_t gstVencattr[ST_MAX_VENC_NUM];
static ST_JpdModeAttr_t gstJpdModeAttr;
static MI_BOOL bExit = FALSE;
static ST_DispModAttr_t   gstDispModule;
static MI_U32 u32ChnNum=0;
static MI_U32 u32TimeOutSExit=0;
static MI_BOOL bUseUserSensor=FALSE;
static MI_BOOL bUseDisp=FALSE;
static MI_BOOL btest_1080=FALSE;
static MI_BOOL TEST_ISP_MULTI_DEV=FALSE;
static MI_BOOL bDetectAD=FALSE;
static MI_BOOL bDetectLog= FALSE;
static UT_CaseStatus_e UTStatus = UT_CASE_SUCCESS;
static volatile int RoomID = 0;

static pthread_mutex_t gIniMd5Mutex;
static ST_Md5Action_e gMd5Action = E_ST_MD5_ACTION_NONE;

MI_S32 ST_Sys_Bind_List(ST_Sys_BindInfo_T *pstBindInfo)
{
    ST_Output_BindParam_List_t *head = NULL;
    ST_Output_BindParam_List_t *list_member = NULL;
    MI_U32 u32DevId = pstBindInfo->stSrcChnPort.u32DevId;
    MI_U32 u32ChnId = pstBindInfo->stSrcChnPort.u32ChnId;
    MI_U32 u32PortId = pstBindInfo->stSrcChnPort.u32PortId;
    MI_S32 s32Ret = MI_SUCCESS;

    if(pstBindInfo->stSrcChnPort.eModId == E_MI_MODULE_ID_VIF)
    {
        MI_U8 u8VifGroupId =0;
        MI_U8 vifDevPerGroup = 0;

        u8VifGroupId = u32DevId/ST_MAX_VIF_DEV_PERGROUP;
        vifDevPerGroup = u32DevId%ST_MAX_VIF_DEV_PERGROUP;
        head = &gstVifModule.stVifGroupAttr[u8VifGroupId].stVifDevAttr[vifDevPerGroup].stVifOutPortAttr[u32PortId].head;
    }
    else if(pstBindInfo->stSrcChnPort.eModId == E_MI_MODULE_ID_ISP)
    {
        head = &gstIspModule.stIspDevAttr[u32DevId].stIspChnlAttr[u32ChnId].stIspOutPortAttr[u32PortId].head;
    }
    else if(pstBindInfo->stSrcChnPort.eModId == E_MI_MODULE_ID_SCL)
    {
        head = &gstSclModule.stSclDevAttr[u32DevId].stSclChnlAttr[u32ChnId].stSclOutPortAttr[u32PortId].head;
    }
    else if(pstBindInfo->stSrcChnPort.eModId == E_MI_MODULE_ID_JPD)
    {
        head = &gstJpdModeAttr.stJpdChnAttr[u32ChnId].head;
    }
    else
    {
        printf("[%s]%d module:%d not support \n", __FUNCTION__,__LINE__,pstBindInfo->stSrcChnPort.eModId);
        s32Ret = -1;
        goto EXIT;
    }

    ExecFuncResult(ST_Sys_Bind(pstBindInfo), s32Ret);

    list_member = (ST_Output_BindParam_List_t *)malloc(sizeof(ST_Output_BindParam_List_t));
    if(list_member == NULL)
    {
        printf("[%s]%d malloc failed, no memory \n", __FUNCTION__,__LINE__);
        s32Ret = -1;
        goto EXIT;
    }
    memset(list_member, 0x0, sizeof(ST_Output_BindParam_List_t));

    memcpy(&list_member->stBindInfo, pstBindInfo, sizeof(ST_Sys_BindInfo_T));

    list_add_tail(&list_member->pos, &head->pos);

EXIT:
    return s32Ret;
}

MI_S32 ST_Sys_UnBind_List(ST_Sys_BindInfo_T *pstBindInfo)
{
    MI_S32 s32Ret = MI_SUCCESS;
    MI_U32 u32DevId = pstBindInfo->stSrcChnPort.u32DevId;
    MI_U32 u32ChnId = pstBindInfo->stSrcChnPort.u32ChnId;
    MI_U32 u32PortId = pstBindInfo->stSrcChnPort.u32PortId;
    ST_Output_BindParam_List_t *head = NULL, *node = NULL, *node_safe = NULL;

    if(pstBindInfo->stSrcChnPort.eModId == E_MI_MODULE_ID_VIF)
    {
        MI_U8 u8VifGroupId =0;
        MI_U8 vifDevPerGroup = 0;

        u8VifGroupId = u32DevId/ST_MAX_VIF_DEV_PERGROUP;
        vifDevPerGroup = u32DevId%ST_MAX_VIF_DEV_PERGROUP;
        head = &gstVifModule.stVifGroupAttr[u8VifGroupId].stVifDevAttr[vifDevPerGroup].stVifOutPortAttr[u32PortId].head;
    }
    else if(pstBindInfo->stSrcChnPort.eModId == E_MI_MODULE_ID_ISP)
    {
        head = &gstIspModule.stIspDevAttr[u32DevId].stIspChnlAttr[u32ChnId].stIspOutPortAttr[u32PortId].head;
    }
    else if(pstBindInfo->stSrcChnPort.eModId == E_MI_MODULE_ID_SCL)
    {
        head = &gstSclModule.stSclDevAttr[u32DevId].stSclChnlAttr[u32ChnId].stSclOutPortAttr[u32PortId].head;
    }
    else if(pstBindInfo->stSrcChnPort.eModId == E_MI_MODULE_ID_JPD)
    {
        head = &gstJpdModeAttr.stJpdChnAttr[u32ChnId].head;
    }
    else
    {
        printf("[%s]%d module:%d not support \n", __FUNCTION__,__LINE__,pstBindInfo->stSrcChnPort.eModId);
        s32Ret = -1;
        goto EXIT;
    }

    ExecFuncResult(ST_Sys_UnBind(pstBindInfo), s32Ret);

    list_for_each_entry_safe(node, node_safe, &head->pos, pos)
    {
        s32Ret = memcmp(&(node->stBindInfo.stDstChnPort), &(pstBindInfo->stDstChnPort), sizeof(MI_SYS_ChnPort_t));
        if(s32Ret == 0)
        {
            list_del(&node->pos);
            free(node);
            node = NULL;
            break;
        }
    }
EXIT:
    return s32Ret;
}

static void Convert12BitsTo16Bits(unsigned char *pInbuf, unsigned short *pOutbuf, unsigned long *pu32InSize, unsigned long *pu32OutSize)
{
    pOutbuf[0] = (((unsigned short)pInbuf[0] << 4) & 0x0ff0) + (((unsigned short)pInbuf[1] << 12) & 0xf000);
    pOutbuf[1] = (((unsigned short)pInbuf[1]) & 0x00f0) + (((unsigned short)pInbuf[2] << 8) & 0xff00);
    pOutbuf[2] = (((unsigned short)pInbuf[3] << 4) & 0x0ff0) + (((unsigned short)pInbuf[4] << 12) & 0xf000);
    pOutbuf[3] = (((unsigned short)pInbuf[4]) & 0x00f0) + (((unsigned short)pInbuf[5] << 8) & 0xff00);
    *pu32InSize = 6;
    *pu32OutSize = 4;
}

static void Convert10BitsTo16Bits(unsigned char *pInbuf, unsigned short *pOutbuf, unsigned long *pu32InSize, unsigned long *pu32OutSize)
{
    pOutbuf[0] = (((unsigned short)pInbuf[0] << 6) & 0x3fc0) + (((unsigned short)pInbuf[1] << 14) & 0xc000);
    pOutbuf[1] = (((unsigned short)pInbuf[1] << 4) & 0x0fc0) + (((unsigned short)pInbuf[2] << 12) & 0xf000);
    pOutbuf[2] = (((unsigned short)pInbuf[2] << 2) & 0x03c0) + (((unsigned short)pInbuf[3] << 10) & 0xfc00);
    pOutbuf[3] = (((unsigned short)pInbuf[3]) & 0x00c0) + (((unsigned short)pInbuf[4] << 8) & 0xff00);
    *pu32InSize = 5;
    *pu32OutSize = 4;
}

static void Convert8BitsTo16Bits(unsigned char *pInbuf, unsigned short *pOutbuf, unsigned long *pu32InSize, unsigned long *pu32OutSize)
{
    pOutbuf[0] = (((unsigned short)pInbuf[0] << 8) & 0xff00);
    pOutbuf[1] = (((unsigned short)pInbuf[1] << 8) & 0xff00);
    pOutbuf[2] = (((unsigned short)pInbuf[2] << 8) & 0xff00);
    pOutbuf[3] = (((unsigned short)pInbuf[3] << 8) & 0xff00);
    *pu32InSize = 4;
    *pu32OutSize = 4;
}

static void ConvertOneLineTo16Bits(unsigned char *pbufin, unsigned short *pbufout, unsigned long ulinbufsize, unsigned long uloutbufsize, unsigned short usbitwidth)
{
    unsigned long u32InSize, u32OutSize;
    unsigned long InCnt = 0;
    unsigned long OutCnt = 0;

    memset((void *)pbufout, 0x0, uloutbufsize);

    u32InSize = 0;
    u32OutSize = 0;
    while ((u32InSize < ulinbufsize) && (u32OutSize < uloutbufsize))
    {
        if (usbitwidth == 12)
        {
            Convert12BitsTo16Bits(pbufin, pbufout, &InCnt, &OutCnt);
        }
        else if (usbitwidth == 10)
        {
            Convert10BitsTo16Bits(pbufin, pbufout, &InCnt, &OutCnt);
        }
        else if (usbitwidth == 8)
        {
            Convert8BitsTo16Bits(pbufin, pbufout, &InCnt, &OutCnt);
        }
        else
        {
            printf("unsupport bit width:%d\n", usbitwidth);
            return;
        }
        pbufin += InCnt;
        pbufout += OutCnt;
        u32InSize += InCnt;
        u32OutSize += OutCnt;
    }
}

static void ConvertRawImageTo16Bits(void *pBufin, void *pBufout, unsigned short u16Width, unsigned short u16Height, MI_SYS_DataPrecision_e eBitDepth, MI_BOOL bReverse)
{
    unsigned char *pbufin;
    unsigned short *pbufout;
    unsigned long buf_size_in, buf_size_out;
    unsigned long linecnt = 0;
    unsigned short bitwidth = 0;
    unsigned int read_offset = 0;
    unsigned int write_offset = 0;

    if (eBitDepth == E_MI_SYS_DATA_PRECISION_8BPP)  // 8-bits
    {
        buf_size_in = BUFFER_SIZE_IN_8BITS(u16Width);
        bitwidth = 8;
    }
    else if (eBitDepth == E_MI_SYS_DATA_PRECISION_10BPP)  // 10 bits
    {
        buf_size_in = BUFFER_SIZE_IN_10BITS(u16Width);
        bitwidth = 10;
    }
    else if (eBitDepth == E_MI_SYS_DATA_PRECISION_12BPP)  // 12-bits
    {
        buf_size_in = BUFFER_SIZE_IN_12BITS(u16Width);
        bitwidth = 12;
    }
    else
    {
        printf("Error! Don't need to convert image!!!eBitDepth = %d\n", eBitDepth);
        UTStatus = UT_CASE_FAIL;
        return;
    }

    buf_size_out = (BUFFER_SIZE_OUT_16BITS(u16Width) >> 1);  // by unsigned short
    read_offset = bReverse ? (buf_size_in * (u16Height-1)) : 0;
    write_offset = bReverse ? (buf_size_out * (u16Height-1)) : 0;

    linecnt = 0;
    pbufin = (unsigned char *)pBufin + read_offset;
    pbufout = (unsigned short *)pBufout + write_offset;
    while (linecnt < u16Height)
    {
        ConvertOneLineTo16Bits(pbufin, pbufout, buf_size_in, buf_size_out, bitwidth);
        linecnt++;
        pbufin += (bReverse ? -buf_size_in : buf_size_in);
        pbufout += (bReverse ? -buf_size_out : buf_size_out);
    }
}

void ST_GetBayerStride(MI_SYS_DataPrecision_e ePrecision, MI_U16 u16Width, MI_U32 *pu32Stride)
{
    MI_U8 denomt = 0;

    if(ePrecision == E_MI_SYS_DATA_PRECISION_8BPP)
    {
        denomt = 16;
        *pu32Stride = BAYER_LIMIT_BITS((MI_U32)(((u16Width + denomt - 1) / denomt) * 1), 12) * 16;
    }
    else if(ePrecision == E_MI_SYS_DATA_PRECISION_10BPP)
    {
        denomt = 64;
        *pu32Stride = BAYER_LIMIT_BITS((MI_U32)(((u16Width + denomt - 1) / denomt) * 5), 12) * 16;
    }
    else if(ePrecision == E_MI_SYS_DATA_PRECISION_12BPP)
    {
        denomt = 32;
        *pu32Stride = BAYER_LIMIT_BITS((MI_U32)(((u16Width + denomt - 1) / denomt) * 3), 12) * 16;
    }
    else if(ePrecision == E_MI_SYS_DATA_PRECISION_14BPP || ePrecision == E_MI_SYS_DATA_PRECISION_16BPP)
    {
        denomt = 8;
        *pu32Stride = BAYER_LIMIT_BITS((MI_U32)(((u16Width + denomt - 1) / denomt) * 1), 12) * 16;
    }
}

MI_S32 s32ST_StatisticsMD5Value(MI_U8 *pVirAddr, MI_U32 u32BuffSize, ST_MD5Info_t *pstSaveMd5Value);

MI_S32 ST_DispModuleInit(MI_DISP_DEV s32Dev)
{
    MI_S32 s32Ret = MI_SUCCESS;
#if MI_DISP_SUPPORT

    ST_DispDevAttr_t    *pstDispDevAttr = &gstDispModule.stDispDevAttr[s32Dev];
    MI_DISP_LAYER s32Layer = 0;
    MI_DISP_INPUTPORT u32Port =0;
    MI_DISP_VideoLayerAttr_t stDispLayerAttr;
    ST_DispLayerAttr_t *pstDispLayerAttr = NULL;

    //step1
    MI_DISP_PubAttr_t   stDispPubAttr;
    memset(&stDispPubAttr, 0, sizeof(MI_DISP_PubAttr_t));
    stDispPubAttr.eIntfSync = pstDispDevAttr->eIntfSync;
    stDispPubAttr.eIntfType = pstDispDevAttr->eIntfType;
    stDispPubAttr.u32BgColor = YUYV_BLACK;
    ExecFuncResult(MI_DISP_SetPubAttr(s32Dev, &stDispPubAttr), s32Ret);
    ExecFuncResult(MI_DISP_Enable(s32Dev), s32Ret);
    pstDispDevAttr->bCreate = true;

    //step2
    if(E_MI_DISP_INTF_TTL == pstDispDevAttr->eIntfType)
    {
        ExecFuncResult(MI_PANEL_Init(E_MI_PNL_INTF_TTL), s32Ret);
    }
    else if (E_MI_DISP_INTF_HDMI == pstDispDevAttr->eIntfType)
    {
        MI_HDMI_DeviceId_e eHdmi = E_MI_HDMI_ID_0;
        MI_HDMI_InitParam_t stInitParam;
        MI_HDMI_TimingType_e eHdmiTiming;
        memset(&stInitParam, 0, sizeof(MI_HDMI_InitParam_t));
        stInitParam.pCallBackArgs = NULL;
        stInitParam.pfnHdmiEventCallback = NULL;
        ExecFuncResult(MI_HDMI_Init(&stInitParam), s32Ret);
        ExecFuncResult(MI_HDMI_Open(eHdmi), s32Ret);
        eHdmiTiming = pstDispDevAttr->eHdmiTiming;

        MI_HDMI_Attr_t stAttr;
        memset(&stAttr, 0, sizeof(MI_HDMI_Attr_t));
        if (eHdmiTiming == E_MI_HDMI_TIMING_MAX)
        {
            ST_DBG("[%s][%d]unsupported hdmi timing %d,reset to 1080p60\n", __FUNCTION__, __LINE__, eHdmiTiming);
            eHdmiTiming = E_MI_HDMI_TIMING_1080_60P;
        }
        stAttr.stEnInfoFrame.bEnableAudInfoFrame = FALSE;
        stAttr.stEnInfoFrame.bEnableAviInfoFrame = FALSE;
        stAttr.stEnInfoFrame.bEnableSpdInfoFrame = FALSE;
        stAttr.stAudioAttr.bEnableAudio = TRUE;
        stAttr.stAudioAttr.bIsMultiChannel = 0;
        stAttr.stAudioAttr.eBitDepth = E_MI_HDMI_BIT_DEPTH_16;
        stAttr.stAudioAttr.eCodeType = E_MI_HDMI_ACODE_PCM;
        stAttr.stAudioAttr.eSampleRate = E_MI_HDMI_AUDIO_SAMPLERATE_48K;
        stAttr.stVideoAttr.bEnableVideo = TRUE;
        stAttr.stVideoAttr.eColorType = E_MI_HDMI_COLOR_TYPE_YCBCR444;//default color type
        stAttr.stVideoAttr.eDeepColorMode = E_MI_HDMI_DEEP_COLOR_MAX;
        stAttr.stVideoAttr.eTimingType = eHdmiTiming;
        stAttr.stVideoAttr.eOutputMode = E_MI_HDMI_OUTPUT_MODE_HDMI;
        ExecFuncResult(MI_HDMI_SetAttr(eHdmi, &stAttr), s32Ret);
        ExecFuncResult(MI_HDMI_Start(eHdmi), s32Ret);
        ST_DBG("hdmi init\n");
    }

    //step3
    for(s32Layer = 0;s32Layer < ST_MAX_DISP_LAYER_NUM; s32Layer++)
    {
        pstDispLayerAttr = &pstDispDevAttr->stDispLayerAttr[s32Layer];
        if(true == pstDispLayerAttr->bUsed && false == pstDispLayerAttr->bCreate)
        {
            MI_DISP_RotateConfig_t  stRotateConfig;
            memset(&stRotateConfig, 0, sizeof(MI_DISP_RotateConfig_t));
            memset(&stDispLayerAttr, 0, sizeof(MI_DISP_VideoLayerAttr_t));

            stDispLayerAttr.stVidLayerDispWin.u16X = pstDispLayerAttr->stDispLayerWin.u16X;
            stDispLayerAttr.stVidLayerDispWin.u16Y = pstDispLayerAttr->stDispLayerWin.u16Y;
            stDispLayerAttr.stVidLayerDispWin.u16Width = pstDispLayerAttr->stDispLayerWin.u16Width;
            stDispLayerAttr.stVidLayerDispWin.u16Height = pstDispLayerAttr->stDispLayerWin.u16Height;
            stDispLayerAttr.stVidLayerSize.u16Width = pstDispLayerAttr->u16Width;
            stDispLayerAttr.stVidLayerSize.u16Height = pstDispLayerAttr->u16Height;

            stRotateConfig.eRotateMode=pstDispLayerAttr->eRotMode;

            ExecFuncResult(MI_DISP_BindVideoLayer(s32Layer, s32Dev), s32Ret);
            ExecFuncResult(MI_DISP_SetVideoLayerAttr(s32Layer, &stDispLayerAttr), s32Ret);
            ExecFuncResult(MI_DISP_SetVideoLayerRotateMode(s32Layer, &stRotateConfig), s32Ret);
            ExecFuncResult(MI_DISP_EnableVideoLayer(s32Layer), s32Ret);
            pstDispLayerAttr->bCreate = true;
        }

        MI_U32 PortNum = (s32Layer == 0) ? ST_MAX_DISP_LAYER0_PORT_NUM : ST_MAX_DISP_LAYER1_PORT_NUM;
        MI_DISP_InputPortAttr_t stDispInputPortAttr;
        ST_DispPortAttr_t *pstDispPortAttr = NULL;
        for(u32Port = 0; u32Port < PortNum;u32Port++)
        {
            pstDispPortAttr = &pstDispLayerAttr->stDispPortAttr[u32Port];
            if(true == pstDispPortAttr->bUsed && false == pstDispPortAttr->bCreate)
            {
                //step4
                memset(&stDispInputPortAttr, 0, sizeof(MI_DISP_InputPortAttr_t));
                stDispInputPortAttr.u16SrcWidth = pstDispPortAttr->u16Width;
                stDispInputPortAttr.u16SrcHeight = pstDispPortAttr->u16Height;
                stDispInputPortAttr.stDispWin.u16X = pstDispPortAttr->stDispPortWin.u16X;
                stDispInputPortAttr.stDispWin.u16Y = pstDispPortAttr->stDispPortWin.u16Y;

                if(pstDispLayerAttr->eRotMode == E_MI_DISP_ROTATE_90
                    || pstDispLayerAttr->eRotMode == E_MI_DISP_ROTATE_270)
                {
                    stDispInputPortAttr.stDispWin.u16Width = pstDispPortAttr->stDispPortWin.u16Height;
                    stDispInputPortAttr.stDispWin.u16Height = pstDispPortAttr->stDispPortWin.u16Width;
                }
                else
                {
                    stDispInputPortAttr.stDispWin.u16Width = pstDispPortAttr->stDispPortWin.u16Width;
                    stDispInputPortAttr.stDispWin.u16Height = pstDispPortAttr->stDispPortWin.u16Height;
                }
                ExecFuncResult(MI_DISP_SetInputPortAttr(s32Layer, u32Port, &stDispInputPortAttr), s32Ret);
                ExecFuncResult(MI_DISP_EnableInputPort(s32Layer, u32Port), s32Ret);
                pstDispPortAttr->bCreate = true;

                //step5
                ST_Sys_BindInfo_T stBindInfo;
                memset(&stBindInfo, 0x0, sizeof(ST_Sys_BindInfo_T));

                stBindInfo.stSrcChnPort.eModId = pstDispPortAttr->stBindInfo.stChnPort.eModId;
                stBindInfo.stSrcChnPort.u32DevId = pstDispPortAttr->stBindInfo.stChnPort.u32DevId;
                stBindInfo.stSrcChnPort.u32ChnId = pstDispPortAttr->stBindInfo.stChnPort.u32ChnId;
                stBindInfo.stSrcChnPort.u32PortId = pstDispPortAttr->stBindInfo.stChnPort.u32PortId;

                stBindInfo.stDstChnPort.eModId = E_MI_MODULE_ID_DISP;
                stBindInfo.stDstChnPort.u32DevId = s32Dev;
                stBindInfo.stDstChnPort.u32ChnId = s32Layer;
                if(1 == s32Layer)
                {
                    stBindInfo.stDstChnPort.u32ChnId = 16;
                }
                stBindInfo.stDstChnPort.u32PortId = u32Port;
                stBindInfo.u32SrcFrmrate = pstDispPortAttr->stBindInfo.u32SrcFrmrate;
                stBindInfo.u32DstFrmrate = pstDispPortAttr->stBindInfo.u32DstFrmrate;
                stBindInfo.eBindType = pstDispPortAttr->stBindInfo.eBindType;
                ExecFuncResult(ST_Sys_Bind_List(&stBindInfo), s32Ret);
            }
        }
    }
EXIT:
#endif
    return s32Ret;
}

MI_S32 ST_DispModuleUnInit(MI_DISP_DEV s32Dev)
{
    MI_S32 s32Ret = MI_SUCCESS;
#if MI_DISP_SUPPORT

    ST_DispDevAttr_t    *pstDispDevAttr = &gstDispModule.stDispDevAttr[s32Dev];
    MI_DISP_LAYER s32Layer = 0;
    MI_DISP_INPUTPORT u32Port =0;
    ST_DispLayerAttr_t *pstDispLayerAttr = NULL;

    for(s32Layer = 0;s32Layer < ST_MAX_DISP_LAYER_NUM;s32Layer++)
    {
        pstDispLayerAttr = &pstDispDevAttr->stDispLayerAttr[s32Layer];
        if(false == pstDispLayerAttr->bUsed || false == pstDispLayerAttr->bCreate)
        {
            continue;
        }
        MI_U32 u32PortNum = (0 == s32Layer) ? ST_MAX_DISP_LAYER0_PORT_NUM : ST_MAX_DISP_LAYER1_PORT_NUM;
        ST_DispPortAttr_t *pstDispPortAttr = NULL;
        for(u32Port = 0;u32Port < u32PortNum;u32Port++)
        {
            pstDispPortAttr = &pstDispLayerAttr->stDispPortAttr[u32Port];
            if(false == pstDispPortAttr->bUsed || false == pstDispPortAttr->bCreate)
            {
                continue;
            }

            //step1
            ST_Sys_BindInfo_T stBindInfo;
            memset(&stBindInfo, 0x0, sizeof(ST_Sys_BindInfo_T));
            stBindInfo.stSrcChnPort.eModId = pstDispPortAttr->stBindInfo.stChnPort.eModId;
            stBindInfo.stSrcChnPort.u32DevId = pstDispPortAttr->stBindInfo.stChnPort.u32DevId;
            stBindInfo.stSrcChnPort.u32ChnId = pstDispPortAttr->stBindInfo.stChnPort.u32ChnId;
            stBindInfo.stSrcChnPort.u32PortId = pstDispPortAttr->stBindInfo.stChnPort.u32PortId;

            stBindInfo.stDstChnPort.eModId = E_MI_MODULE_ID_DISP;
            stBindInfo.stDstChnPort.u32DevId = s32Dev;
            stBindInfo.stDstChnPort.u32ChnId = s32Layer;
            if(1 == s32Layer)
            {
                stBindInfo.stDstChnPort.u32ChnId = 16;
            }
            stBindInfo.stDstChnPort.u32PortId = u32Port;
            stBindInfo.u32SrcFrmrate = pstDispPortAttr->stBindInfo.u32SrcFrmrate;
            stBindInfo.u32DstFrmrate = pstDispPortAttr->stBindInfo.u32DstFrmrate;
            stBindInfo.eBindType = pstDispPortAttr->stBindInfo.eBindType;
            ExecFuncResult(ST_Sys_UnBind_List(&stBindInfo), s32Ret);
            ExecFuncResult(MI_DISP_DisableInputPort(s32Layer,u32Port), s32Ret);
            pstDispPortAttr->bCreate = false;
            pstDispDevAttr->stDispLayerAttr[s32Layer].u16PortUseCount--;
        }
        ExecFuncResult(MI_DISP_DisableVideoLayer(s32Layer), s32Ret);
        ExecFuncResult(MI_DISP_UnBindVideoLayer(s32Layer, s32Dev), s32Ret);
        pstDispDevAttr->stDispLayerAttr[s32Layer].bCreate = false;
    }
    if(E_MI_DISP_INTF_TTL == pstDispDevAttr->eIntfType)
    {
        ExecFuncResult(MI_PANEL_DeInit(E_MI_PNL_INTF_TTL), s32Ret);
    }
    else if (E_MI_DISP_INTF_HDMI == pstDispDevAttr->eIntfType)
    {
        ExecFuncResult(MI_HDMI_DeInit(), s32Ret);
    }
    ExecFuncResult(MI_DISP_Disable(s32Dev), s32Ret);
    pstDispDevAttr->bCreate = false;

EXIT:
#endif
    return s32Ret;
}

MI_S32 ST_DispParserIni(dictionary *pstDict)
{
    MI_U8 i,j,k = 0;
    char s8GetIniInfo[128] = {0};
    ST_DispModAttr_t *pstDispModAttr = NULL;
    pstDispModAttr = &gstDispModule;

    for(i = 0;i < ST_MAX_DISP_DEV_NUM;i++)
    {
        if(false == pstDispModAttr->stDispDevAttr[i].bUsed)
        {
            continue;
        }

        if(E_MI_DISP_INTF_TTL == pstDispModAttr->stDispDevAttr[i].eIntfType)
        {
            pstDispModAttr->stDispDevAttr[i].eIntfSync=E_MI_DISP_OUTPUT_USER;
        }
        else
        {
            sprintf(s8GetIniInfo, ":DispDev%dIntfSync", i);
            pstDispModAttr->stDispDevAttr[i].eIntfSync=(MI_DISP_OutputTiming_e)iniparser_getint(pstDict, s8GetIniInfo, 32);
        }

        sprintf(s8GetIniInfo, ":DispDev%dHdmiTiming", i);
        pstDispModAttr->stDispDevAttr[i].eHdmiTiming = (MI_HDMI_TimingType_e)iniparser_getint(pstDict, s8GetIniInfo, 9);

        sprintf(s8GetIniInfo, ":DispDev%dWidth", i);
        pstDispModAttr->stDispDevAttr[i].u16Width= (MI_DISP_Interface_e)iniparser_getint(pstDict, s8GetIniInfo, 1024);
        sprintf(s8GetIniInfo, ":DispDev%dHeight", i);
        pstDispModAttr->stDispDevAttr[i].u16Height= (MI_DISP_Interface_e)iniparser_getint(pstDict, s8GetIniInfo, 600);
        ST_DispLayerAttr_t *pstDispLayerAttr = NULL;
        for(j = 0;j < ST_MAX_DISP_LAYER_NUM;j++)
        {
            pstDispLayerAttr = &pstDispModAttr->stDispDevAttr[i].stDispLayerAttr[j];
            if(false == pstDispLayerAttr->bUsed)
            {
                continue;
            }

            sprintf(s8GetIniInfo, ":DispLayer%dWinWidth", j);
            pstDispLayerAttr->stDispLayerWin.u16Width= iniparser_getint(pstDict, s8GetIniInfo, pstDispModAttr->stDispDevAttr[i].u16Width);
            sprintf(s8GetIniInfo, ":DispLayer%dWinHeight", j);
            pstDispLayerAttr->stDispLayerWin.u16Height= iniparser_getint(pstDict, s8GetIniInfo, pstDispModAttr->stDispDevAttr[i].u16Height);
            sprintf(s8GetIniInfo, ":DispLayer%dWinX", j);
            pstDispLayerAttr->stDispLayerWin.u16X= iniparser_getint(pstDict, s8GetIniInfo, 0);
            sprintf(s8GetIniInfo, ":DispLayer%dWinY", j);
            pstDispLayerAttr->stDispLayerWin.u16Y= iniparser_getint(pstDict, s8GetIniInfo, 0);
            sprintf(s8GetIniInfo, ":DispLayer%dWidth", j);
            pstDispLayerAttr->u16Width= iniparser_getint(pstDict, s8GetIniInfo, pstDispModAttr->stDispDevAttr[i].u16Width);
            sprintf(s8GetIniInfo, ":DispLayer%dHeight", j);
            pstDispLayerAttr->u16Height= iniparser_getint(pstDict, s8GetIniInfo, pstDispModAttr->stDispDevAttr[i].u16Height);

            sprintf(s8GetIniInfo, ":DispLayer%dRot", j);
            pstDispLayerAttr->eRotMode= (MI_DISP_RotateMode_e)iniparser_getint(pstDict, s8GetIniInfo, E_MI_DISP_ROTATE_NONE);

            MI_U32 u32PortNum = (0 == j) ? ST_MAX_DISP_LAYER0_PORT_NUM : ST_MAX_DISP_LAYER1_PORT_NUM;
            ST_DispPortAttr_t *pstDispPortAttr = NULL;
            MI_U16 u16DtConut = u32PortNum;
            MI_U16 u16DtX = 0;
            MI_U16 u16DtY = 0;
            MI_U16 u16DtLevel = 0;
            if(u16DtConut%2)
            {
                u16DtConut++;
            }
            u16DtConut = u16DtConut/2;
            u16DtX = pstDispLayerAttr->u16Width/2;
            u16DtY = pstDispLayerAttr->u16Height/u16DtConut;
            for(k = 0;k < u32PortNum;k++)
            {
                pstDispPortAttr = &pstDispLayerAttr->stDispPortAttr[k];
                if (false == pstDispPortAttr->bUsed)
                {
                    continue;
                }
                pstDispPortAttr->stDispPortWin.u16Y= u16DtLevel*u16DtY;
                if(false == k%2)
                {
                    pstDispPortAttr->stDispPortWin.u16X= 0;
                }
                else
                {
                    pstDispPortAttr->stDispPortWin.u16X= u16DtX;
                    u16DtLevel++;
                }
                sprintf(s8GetIniInfo, ":DispPort%dWidth", i);
                pstDispPortAttr->stDispPortWin.u16Width= iniparser_getint(pstDict, s8GetIniInfo, pstDispModAttr->stDispDevAttr[i].u16Width);
                sprintf(s8GetIniInfo, ":DispPort%dHeight", i);
                pstDispPortAttr->stDispPortWin.u16Height= iniparser_getint(pstDict, s8GetIniInfo, pstDispModAttr->stDispDevAttr[i].u16Height);

                pstDispPortAttr->u16Width = pstDispPortAttr->stDispPortWin.u16Width;
                pstDispPortAttr->u16Height = pstDispPortAttr->stDispPortWin.u16Height;

                pstDispPortAttr->stBindInfo.u32DstFrmrate= 30;
                pstDispPortAttr->stBindInfo.u32SrcFrmrate= 30;
                pstDispPortAttr->stBindInfo.eBindType= E_MI_SYS_BIND_TYPE_FRAME_BASE;
            }
        }
    }
    return MI_SUCCESS;
}


void ST_Flush(void)
{
    char c;

    while((c = getchar()) != '\n' && c != EOF);
}

void *ST_OpenStream(char const *szStreamName, void *arg)
{
    //ST_VencAttr_t *pstVencAttr = NULL;

#if MI_VENC_SUPPORT

    MI_U32 i = 0;
    MI_S32 s32Ret = MI_SUCCESS;

    for(i = 0; i < ST_MAX_VENC_NUM; i ++)
    {
        if(!strncmp(szStreamName, gstVencattr[i].szStreamName,
                    strlen(szStreamName)))
        {
            break;
        }
    }

    if(i >= ST_MAX_VENC_NUM)
    {
        ST_ERR("not found this stream, \"%s\"", szStreamName);
        UTStatus = UT_CASE_FAIL;
        return NULL;
    }

    ST_VencAttr_t *pstVencAttr = &gstVencattr[i];

    s32Ret = MI_VENC_RequestIdr(gstVencattr[i].DevId,pstVencAttr->vencChn, TRUE);

    ST_DBG("open stream \"%s\" success, chn:%d\n", szStreamName, pstVencAttr->vencChn);

    if(MI_SUCCESS != s32Ret)
    {
        ST_WARN("request IDR fail, error:%x\n", s32Ret);
        UTStatus = UT_CASE_FAIL;
    }
#endif
    return pstVencAttr;
}

MI_U32 u32GetCnt=0, u32ReleaseCnt=0;
int ST_VideoReadStream(void *handle, unsigned char *ucpBuf, int BufLen, struct timeval *p_Timestamp, void *arg)
{
#if MI_VENC_SUPPORT

    MI_SYS_BufInfo_t stBufInfo;
    MI_S32 s32Ret = MI_SUCCESS;
    MI_S32 len = 0;
    MI_VENC_Stream_t stStream;
    MI_VENC_Pack_t stPack;
    MI_VENC_ChnStat_t stStat;
    MI_VENC_CHN vencChn ;
    MI_VENC_DEV DevId =0;

    if(handle == NULL)
    {
        return -1;
    }

    ST_VencAttr_t *pstStreamInfo = (ST_VencAttr_t *)handle;

    vencChn = pstStreamInfo->vencChn;
    DevId = pstStreamInfo->DevId;

    //if(pstStreamInfo->bCreate == FALSE)
        //return 0;

    /*s32Ret = MI_VENC_GetChnDevid(vencChn, &u32DevId);

    if(MI_SUCCESS != s32Ret)
    {
        ST_INFO("MI_VENC_GetChnDevid %d error, %X\n", vencChn, s32Ret);
    }*/

    memset(&stBufInfo, 0x0, sizeof(MI_SYS_BufInfo_t));
    memset(&stStream, 0, sizeof(stStream));
    memset(&stPack, 0, sizeof(stPack));
    stStream.pstPack = &stPack;
    stStream.u32PackCount = 1;
    s32Ret = MI_VENC_Query(DevId, vencChn, &stStat);

    if(s32Ret != MI_SUCCESS || stStat.u32CurPacks == 0)
    {
        return 0;
    }

    s32Ret = MI_VENC_GetStream(DevId, vencChn, &stStream, 40);

    if(MI_SUCCESS == s32Ret)
    {
        u32GetCnt++;
        len = stStream.pstPack[0].u32Len;
        memcpy(ucpBuf, stStream.pstPack[0].pu8Addr, MIN(len, BufLen));

        s32Ret = MI_VENC_ReleaseStream(DevId, vencChn, &stStream);
        if(s32Ret != MI_SUCCESS)
        {
            ST_WARN("RELEASE venc buffer fail\n");
        }
        u32ReleaseCnt ++;
        return len;
    }
#endif
    return 0;
}

int ST_CloseStream(void *handle, void *arg)
{
    if(handle == NULL)
    {
        return -1;
    }

    ST_VencAttr_t *pstStreamInfo = (ST_VencAttr_t *)handle;

    ST_DBG("close \"%s\" success\n", pstStreamInfo->szStreamName);
    return 0;
}

MI_S32 ST_RtspServerStart(void)
{
    unsigned int rtspServerPortNum = RTSP_LISTEN_PORT;
    int iRet = 0;
    char *urlPrefix = NULL;
    int arraySize = ARRAY_SIZE(gstVencattr);
    ST_VencAttr_t *pstStreamAttr = gstVencattr;
    int i = 0;
    ServerMediaSession *mediaSession = NULL;
    ServerMediaSubsession *subSession = NULL;
    Live555RTSPServer *pRTSPServer = NULL;

    pRTSPServer = new Live555RTSPServer();

    if(pRTSPServer == NULL)
    {
        ST_ERR("malloc error\n");
        UTStatus = UT_CASE_FAIL;
        return -1;
    }

    iRet = pRTSPServer->SetRTSPServerPort(rtspServerPortNum);

    while(iRet < 0)
    {
        rtspServerPortNum++;

        if(rtspServerPortNum > 65535)
        {
            ST_INFO("Failed to create RTSP server: %s\n", pRTSPServer->getResultMsg());
            delete pRTSPServer;
            pRTSPServer = NULL;
            return -2;
        }

        iRet = pRTSPServer->SetRTSPServerPort(rtspServerPortNum);
    }

    urlPrefix = pRTSPServer->rtspURLPrefix();

    for(i = 0; i < arraySize; i ++)
    {
		if(pstStreamAttr[i].bEnable != TRUE)
		 {
			 //printf("VENC CHN%d have not benn open\n",i);
			 continue;
		 }


        printf("=================URL===================\n");
        printf("%s%s\n", urlPrefix, pstStreamAttr[i].szStreamName);
        printf("=================URL===================\n");

        pRTSPServer->createServerMediaSession(mediaSession,
                                              pstStreamAttr[i].szStreamName,
                                              NULL, NULL);

        if(pstStreamAttr[i].eType == E_MI_VENC_MODTYPE_H264E)
        {
            subSession = WW_H264VideoFileServerMediaSubsession::createNew(
                             *(pRTSPServer->GetUsageEnvironmentObj()),
                             pstStreamAttr[i].szStreamName,
                             ST_OpenStream,
                             ST_VideoReadStream,
                             ST_CloseStream, 30);
        }
        else if(pstStreamAttr[i].eType == E_MI_VENC_MODTYPE_H265E)
        {
            subSession = WW_H265VideoFileServerMediaSubsession::createNew(
                             *(pRTSPServer->GetUsageEnvironmentObj()),
                             pstStreamAttr[i].szStreamName,
                             ST_OpenStream,
                             ST_VideoReadStream,
                             ST_CloseStream, 30);
        }
        else if(pstStreamAttr[i].eType == E_MI_VENC_MODTYPE_JPEGE)
        {
            subSession = WW_JPEGVideoFileServerMediaSubsession::createNew(
                             *(pRTSPServer->GetUsageEnvironmentObj()),
                             pstStreamAttr[i].szStreamName,
                             ST_OpenStream,
                             ST_VideoReadStream,
                             ST_CloseStream, 30);
        }

        pRTSPServer->addSubsession(mediaSession, subSession);
        pRTSPServer->addServerMediaSession(mediaSession);
    }

    pRTSPServer->Start();

    g_pRTSPServer = pRTSPServer;

    return 0;
}

MI_S32 ST_RtspServerStop(void)
{
    if(g_pRTSPServer)
    {
        g_pRTSPServer->Join();
        delete g_pRTSPServer;
        g_pRTSPServer = NULL;
    }

    return 0;
}

MI_S32 ST_WriteOneFrame(FILE *fp, int offset, char *pDataFrame, int line_offset, int line_size, int lineNumber)
{
    int size = 0;
    int i = 0;
    char *pData = NULL;
    int yuvSize = line_size;
    MI_S32 s32Ret = -1;

    for(i = 0; i < lineNumber; i++)
    {
        pData = pDataFrame + line_offset * i;
        yuvSize = line_size;

        do
        {
            if(yuvSize < 256)
            {
                size = yuvSize;
            }
            else
            {
                size = 256;
            }

            size = fwrite(pData, 1, size, fp);

            if(size == 0)
            {
                break;
            }
            else if(size < 0)
            {
                break;
            }

            pData += size;
            yuvSize -= size;
        }
        while(yuvSize > 0);
        s32Ret = MI_SUCCESS;
    }

    return s32Ret;
}

MI_S32 ST_CheckMkdirOutFile(char *pFilePath)//file path and file name
{
    MI_S32 s32Ret = MI_SUCCESS;
    MI_U16 u16size;
    MI_U8 i;
    char FilePath[256];
    char *p =NULL;

    strcpy(FilePath,pFilePath);

    p=strrchr(FilePath, '/'); // get file path
    if(p)
    {
        *(p+1)='\0';
    }
    u16size = strlen(FilePath);
    DBG_INFO("ST_CheckMkdirOutFile FilePath: %s ,u16size = %d \n",FilePath , u16size);

    //filepath is exist
    s32Ret = access(pFilePath, 0);
    if(s32Ret == 0)
    {
        goto EXIT;
    }

    //skip the first '/', e.g:/tmp/stable_m6
    for(i=1; i<u16size; i++)
    {
        if(FilePath[i] == '/')
        {
            FilePath[i] = '\0';
            s32Ret = access(FilePath, 0);
            if(s32Ret != MI_SUCCESS)
            {
                s32Ret = mkdir(FilePath, 0777);
                if(s32Ret != MI_SUCCESS)
                {
                    DBG_ERR("FilePath %s mkdir fail\n", FilePath);
                    UTStatus = UT_CASE_FAIL;
                    goto EXIT;
                }
            }
            FilePath[i] = '/';
        }
    }
    s32Ret = access(FilePath, 0);
    if(u16size > 0 && s32Ret != 0)
    {
        s32Ret = mkdir(FilePath, 0777);
        if(s32Ret != MI_SUCCESS)
        {
            DBG_ERR("FilePath %s mkdir fail\n", FilePath);
            UTStatus = UT_CASE_FAIL;
            goto EXIT;
        }
    }

EXIT:
    return s32Ret;
}

MI_S32 ST_WriteFile(char *pFilePath, FILE **fp, MI_U8 *pData, MI_U32 u32BufSize, MI_U16 *pu16DumpNum, MI_BOOL bRecover)//pFilePath=/path/name
{
    MI_S32 s32Ret = MI_SUCCESS;

    if(*fp == NULL)
    {
        const char *mode = bRecover ? "wb":"ab";
        s32Ret = ST_CheckMkdirOutFile(pFilePath);
        if(s32Ret != MI_SUCCESS)
        {
            return -1;
        }

        *fp = fopen(pFilePath,mode);
        if(*fp == NULL)
        {
            printf("file %s open fail\n", pFilePath);
            *pu16DumpNum = 0;
            return -1;
        }
    }

    fwrite(pData, u32BufSize, 1, *fp);
    DBG_INFO("write file %s id %d done  \n", pFilePath, *pu16DumpNum);

    *pu16DumpNum -=1;

    if(*pu16DumpNum == 0)
    {
        fclose(*fp);
        *fp=NULL;
        DBG_INFO("close file %s done  \n", pFilePath);
    }

    return s32Ret;
}

MI_S32 ST_GetModuleOutputData(MI_SYS_ChnPort_t *pstChnPort, char *sFilePath, MI_S32 s32DumpBuffNum)
{
    time_t stTime = 0;
    memset(&stTime, 0, sizeof(stTime));
    MI_U32 u32DevId=pstChnPort->u32DevId;
    MI_U32 u32ChnId=pstChnPort->u32ChnId;
    MI_U32 u32PortId=pstChnPort->u32PortId;

    DBG_INFO("(%d,%d,%d,%d) filepath %s  buffernum %d \n", pstChnPort->eModId,pstChnPort->u32DevId,pstChnPort->u32ChnId,pstChnPort->u32PortId,
        sFilePath, s32DumpBuffNum);

    if(pstChnPort->eModId == E_MI_MODULE_ID_VIF)
    {
        ST_VifPortAttr_t *pstVifPortAttr = NULL;
        if(u32DevId<ST_MAX_VIF_DEV_NUM && u32ChnId<1 && u32PortId<ST_MAX_VIF_OUTPORT_NUM)
        {
            MI_U16 u16VifGroupId=u32DevId/ST_MAX_VIF_DEV_PERGROUP;
            MI_U16 u16VifDevIdInGroup=u32DevId%ST_MAX_VIF_DEV_PERGROUP;

            pstVifPortAttr = &gstVifModule.stVifGroupAttr[u16VifGroupId].stVifDevAttr[u16VifDevIdInGroup].stVifOutPortAttr[u32PortId];
            if(pstVifPortAttr->bUsed != TRUE)
            {
                DBG_ERR("port %d, not valid \n", u32PortId);
                UTStatus = UT_CASE_FAIL;
                return -1;
            }
            pthread_mutex_lock(&pstVifPortAttr->stoutFileAttr.Portmutex);
            sprintf(pstVifPortAttr->stoutFileAttr.FilePath, "%s", sFilePath);
            pstVifPortAttr->stoutFileAttr.s32DumpBuffNum = s32DumpBuffNum;
            pthread_mutex_unlock(&pstVifPortAttr->stoutFileAttr.Portmutex);
        }
        else
        {
            UTStatus = UT_CASE_FAIL;
            DBG_ERR("(%d,%d,%d,%d) param over range(%d,%d,%d) \n", pstChnPort->eModId, u32DevId, u32ChnId, u32PortId, ST_MAX_VIF_DEV_NUM, 1, ST_MAX_VIF_OUTPORT_NUM);
        }
    }
    else if(pstChnPort->eModId == E_MI_MODULE_ID_ISP)
    {
        ST_PortAttr_t *pstIspPortAttr = NULL;
        if(u32DevId<ST_MAX_ISP_DEV_NUM && u32ChnId<ST_MAX_ISP_CHN_NUM && u32PortId<ST_MAX_ISP_OUTPORT_NUM)
        {
            pstIspPortAttr = &gstIspModule.stIspDevAttr[u32DevId].stIspChnlAttr[u32ChnId].stIspOutPortAttr[u32PortId];
            if(pstIspPortAttr->bUsed != TRUE)
            {
                UTStatus = UT_CASE_FAIL;
                DBG_ERR("port %d, not valid \n", u32PortId);
                return -1;
            }
            pthread_mutex_lock(&pstIspPortAttr->stoutFileAttr.Portmutex);
            sprintf(pstIspPortAttr->stoutFileAttr.FilePath, "%s", sFilePath);
            pstIspPortAttr->stoutFileAttr.s32DumpBuffNum = s32DumpBuffNum;
            pthread_mutex_unlock(&pstIspPortAttr->stoutFileAttr.Portmutex);
        }
        else
        {
            UTStatus = UT_CASE_FAIL;
            DBG_ERR("(%d,%d,%d,%d) param over range(%d,%d,%d) \n", pstChnPort->eModId, u32DevId, u32ChnId, u32PortId, ST_MAX_ISP_DEV_NUM, ST_MAX_ISP_CHN_NUM, ST_MAX_ISP_OUTPORT_NUM);
        }
    }
    else if(pstChnPort->eModId == E_MI_MODULE_ID_SCL)
    {
        ST_PortAttr_t *pstSclPortAttr = NULL;
        if(u32DevId<ST_MAX_SCL_DEV_NUM && u32ChnId<ST_MAX_SCL_CHN_NUM && u32PortId<ST_MAX_SCL_OUTPORT_NUM)
        {
            pstSclPortAttr = &gstSclModule.stSclDevAttr[u32DevId].stSclChnlAttr[u32ChnId].stSclOutPortAttr[u32PortId];
            if(pstSclPortAttr->bUsed != TRUE)
            {
                UTStatus = UT_CASE_FAIL;
                DBG_ERR("port %d, not valid \n", u32PortId);
                return -1;
            }
            pthread_mutex_lock(&pstSclPortAttr->stoutFileAttr.Portmutex);
            sprintf(pstSclPortAttr->stoutFileAttr.FilePath, "%s", sFilePath);
            pstSclPortAttr->stoutFileAttr.s32DumpBuffNum = s32DumpBuffNum;
            pthread_mutex_unlock(&pstSclPortAttr->stoutFileAttr.Portmutex);
        }
        else
        {
            UTStatus = UT_CASE_FAIL;
            DBG_ERR("(%d,%d,%d,%d) param over range(%d,%d,%d) \n", pstChnPort->eModId, u32DevId, u32ChnId, u32PortId, ST_MAX_SCL_DEV_NUM, ST_MAX_SCL_CHN_NUM, ST_MAX_SCL_OUTPORT_NUM);
        }
    }

    return 0;
}

MI_S32 ST_GetVencOut(MI_S32 VencChn, MI_S32 s32DumpBuffNum, char *sFilePath)
{
#if MI_VENC_SUPPORT

    MI_S32 s32Ret = MI_SUCCESS;
    MI_VENC_Stream_t stStream;
    MI_VENC_Pack_t stPack;
    MI_U32 u32BypassCnt = 0;
    MI_VENC_Pack_t *pstPack = NULL;
    MI_U32  i=0;
    FILE *fp = NULL;
    char FilePath[256];
    time_t stTime = 0;
    MI_VENC_ChnStat_t stStat;
    MI_VENC_ChnAttr_t stVencAttr;
    ST_VencAttr_t  *pstVencAttr = NULL;
    MI_VENC_DEV VencDevId =0;
    MD5_CTX stMD5Ctx;
    MI_U8 au8MD5Value[16];

    memset(&stVencAttr, 0x0, sizeof(MI_VENC_ChnAttr_t));
    memset(&stStat, 0x0, sizeof(MI_VENC_ChnStat_t));
    memset(&stStream, 0, sizeof(MI_VENC_Stream_t));
    memset(&stPack, 0, sizeof(MI_VENC_Pack_t));
    stStream.pstPack = &stPack;
    stStream.u32PackCount = 1;

    if(VencChn >= ST_MAX_VENC_NUM)
    {
        printf("channel %d, not valid \n", VencChn);
        return -1;
    }

    pstVencAttr= &gstVencattr[VencChn];
    VencDevId = pstVencAttr->DevId;

    s32Ret = MI_VENC_GetChnAttr(VencDevId, VencChn, &stVencAttr);

    if(s32Ret != MI_SUCCESS)
    {
        printf("channel %d, not valid \n", VencChn);
        return -1;
    }

    s32Ret = ST_CheckMkdirOutFile(sFilePath);
    if(s32Ret != MI_SUCCESS)
    {
        return -1;
    }

    if(stVencAttr.stVeAttr.eType == E_MI_VENC_MODTYPE_JPEGE)
        sprintf(FilePath, "%s/vencchn%d_%ld.jpg", sFilePath, VencChn, time(&stTime));
    else
        sprintf(FilePath, "%s/vencchn%d_%ld.es", sFilePath, VencChn, time(&stTime));

    while(s32DumpBuffNum > 0)
    {
        s32Ret = MI_VENC_Query(VencDevId,VencChn, &stStat);
        if(s32Ret != MI_SUCCESS || stStat.u32CurPacks == 0)
        {
            usleep(THREAD_SLEEP_TIME_US);
            continue;
        }

        s32Ret = MI_VENC_GetStream(VencDevId,VencChn, &stStream, 100);
        if(MI_SUCCESS == s32Ret)
        {
            if (0 == u32BypassCnt)
            {
                MD5Init(&stMD5Ctx);
                DBG_INFO("##########Start to write file %s, id %d packcnt %d!!!#####################\n",FilePath, s32DumpBuffNum, stStream.u32PackCount);

                for(i = 0; i < stStream.u32PackCount; i ++)
                {
                    pstPack = &stStream.pstPack[i];
                    if(MI_SUCCESS != ST_WriteFile(FilePath, &fp, pstPack->pu8Addr + pstPack->u32Offset, pstPack->u32Len - pstPack->u32Offset, (MI_U16 *)&s32DumpBuffNum,FALSE))
                    {
                        DBG_ERR("venc write file err \n");
                        return -1;
                    }

                    MD5Update(&stMD5Ctx, pstPack->pu8Addr + pstPack->u32Offset, pstPack->u32Len - pstPack->u32Offset);
                }

                MD5Final(&stMD5Ctx, au8MD5Value);

                if(MI_SUCCESS != ST_MD5Action2(au8MD5Value, &pstVencAttr->stMd5Attr, pstVencAttr->stMd5Attr.eMd5Action))
                {
                    UTStatus = UT_CASE_FAIL;

                    {
                        char Md5FailFilePath[256];
                        FILE *Md5Failfp = NULL;
                        MI_U16 u16DumpBufferNum =1;

                        sprintf(Md5FailFilePath, "%s_md5fail_value%s", sFilePath, pstVencAttr->stMd5Attr.Md5ValueString);
                        if(MI_SUCCESS != ST_WriteFile(Md5FailFilePath, &Md5Failfp, pstPack->pu8Addr + pstPack->u32Offset, pstPack->u32Len - pstPack->u32Offset, &u16DumpBufferNum,TRUE))
                        {
                            DBG_ERR("venc write file err \n");
                            return -1;
                        }
                    }
                }
            }
            else
            {
                u32BypassCnt--;
                printf("Bypasss frame %d !\n", u32BypassCnt);
            }

            s32Ret = MI_VENC_ReleaseStream(VencDevId,VencChn, &stStream);
            if(MI_SUCCESS != s32Ret)
            {
                ST_DBG("MI_VENC_ReleaseStream fail, ret:0x%x\n", s32Ret);
                goto EXIT;
            }

        }

        usleep(THREAD_SLEEP_TIME_US);
    }

EXIT:
#endif
    return s32Ret;
}

int file_size(char* filename)
{

    struct stat statbuf;

    int ret;

    ret = stat(filename,&statbuf);

    if(ret != 0) return -1;

    return statbuf.st_size;
}

MI_S32 vpe_OpenSourceFile(const char *pFileName, int *pSrcFd)
{
    int src_fd = open(pFileName, O_RDONLY);
    if (src_fd < 0)
    {
        UTStatus = UT_CASE_FAIL;
        perror("open");
        return -1;
    }
    *pSrcFd = src_fd;

    return TRUE;
}
MI_S32 vpe_GetOneFrame(int srcFd, char *pData, int yuvSize)
{
    off_t current = lseek(srcFd,0L, SEEK_CUR);
    off_t end = lseek(srcFd,0L, SEEK_END);

    if((end - current) == 0 || (end - current) < yuvSize)
    {
        lseek(srcFd,0,SEEK_SET);
        current = lseek(srcFd,0,SEEK_CUR);
        //end = lseek(srcFd,0L,SEEK_END);
    }
    /*if ((end - current) < yuvSize)
    {
        return -1;
    }*/
    lseek(srcFd, current, SEEK_SET);
    if (read(srcFd, pData, yuvSize) == yuvSize)
    {
        return 1;
    }

    return 0;
}

MI_S32 vpe_GetOneFrameByStride(int srcFd, char *pData,int width, int stride, int yuvsize)
{
    off_t current = lseek(srcFd,0L, SEEK_CUR);
    off_t end = lseek(srcFd,0L, SEEK_END);
    int bufferheight = yuvsize/stride;
    int i=0;

    if((end - current) == 0 || (end - current) < yuvsize)
    {
        lseek(srcFd,0,SEEK_SET);
        current = lseek(srcFd,0,SEEK_CUR);
        //end = lseek(srcFd,0L,SEEK_END);
    }
    /*if ((end - current) < yuvSize)
    {
        return -1;
    }*/
    lseek(srcFd, current, SEEK_SET);

    printf("width %d, stride %d, buffer height %d \n", width, stride, bufferheight);
    for (i = 0; i < bufferheight; i++)
    {
        if (read(srcFd, pData+ i * stride, width) < width)
        {
            UTStatus = UT_CASE_FAIL;
            printf("linecnt %d read size err \n", i);
            return 0;
        }
    }

    if(i==bufferheight)
        return 1;

    return 0;
}

#define VESFILE_READER_BATCH (1024 * 1024)
#define JPD_MIN(a, b) ((a) < (b) ? (a) : (b))

static __attribute__((unused)) MI_BOOL find_start_code(MI_U8 *pu8Buf)
{
    if (0xFF == pu8Buf[0] && (0xD8 == pu8Buf[1]))
        return TRUE;
    else
        return FALSE;
}

static __attribute__((unused)) MI_BOOL find_end_code(MI_U8 *pu8Buf)
{
    if (0xFF == pu8Buf[0] && (0xD9 == pu8Buf[1]))
        return TRUE;
    else
        return FALSE;
}

void *ST_PutJpdInputDataThread(void *args)
{
    printf("[%s]:%d\n", __FUNCTION__,__LINE__);
#if MI_JPD_SUPPORT
    MI_S32 s32Ret = MI_SUCCESS;

    MI_U32 JpdDev=0;
    MI_JPD_CHN JpdChn;
    MI_U32 u32FrmRate   = 0;
    MI_U8 *pu8Buf       = NULL;
    MI_U32 u32FrameLen  = 0;
    //MI_U32 u32Pos       = 0;
    MI_JPD_ChnStatus_t stChnStatus;
    MI_S32 s32TimeOutMs      = 0;
    //MI_U32 u32FpBackLen      = 0; // if send stream failed, file pointer back length
    MI_U32 u32RequiredLength = 0;
    MI_JPD_StreamBuf_t stRetStreamBuf;
    ST_JpdChnAttr_t *pstJpdChnAttr = (ST_JpdChnAttr_t*)args;
    MI_U32 u32JpdDevId =0;

    JpdChn     = pstJpdChnAttr->u32ChnId;
    u32FrmRate = pstJpdChnAttr->u32FrameRate;
    pu8Buf     = (MI_U8 *)malloc(VESFILE_READER_BATCH);
    printf("[%s]:%d\n", __FUNCTION__,__LINE__);

    FILE *pFile =fopen(pstJpdChnAttr->InputFilePath, "rb");
    if(pFile == NULL)
    {
        printf("openfile %s fail \n", pstJpdChnAttr->InputFilePath);
        free(pu8Buf);
        return 0;
    }

    while(!bExit)
    {
        usleep(1000 / u32FrmRate * 1000);

        MI_BOOL bFindStartCode = FALSE;
        MI_BOOL bFindEndCode   = FALSE;
        MI_S32  s32StartPos    = 0;
        MI_S32  s32EndPos      = 0;

        if (2 != fread(pu8Buf, 1, 2, pFile))
        {
            fseek(pFile, 0, SEEK_SET);
            continue;
        }

        s32StartPos= 2;
        bFindStartCode = find_start_code(pu8Buf);

        while (!bFindStartCode)
        {
            if (feof(pFile))
            {
                fseek(pFile, 0, SEEK_SET);
                break;
            }

            pu8Buf[s32StartPos++] = fgetc(pFile);
            if (s32StartPos >= VESFILE_READER_BATCH)
            {
                UTStatus = UT_CASE_FAIL;
                DBG_ERR("dev%u chn%u s32StartPos:%d over buffer size:%d\n", JpdDev, JpdChn, s32StartPos,
                       VESFILE_READER_BATCH);
            }
            bFindStartCode = find_start_code(&pu8Buf[s32StartPos - 2]);
        }

        if (!bFindStartCode)
        {
            fseek(pFile, 0, SEEK_SET);
            continue;
        }

        s32EndPos = s32StartPos;
        s32StartPos -= 2;

        while (!bFindEndCode)
        {
            if (feof(pFile))
            {
                fseek(pFile, 0, SEEK_SET);
                break;
            }

            pu8Buf[s32EndPos++] = fgetc(pFile);
            if (s32EndPos >= VESFILE_READER_BATCH)
            {
                UTStatus = UT_CASE_FAIL;
                DBG_ERR("dev%u chn%u s32EndPos:%d over buffer size:%d\n", JpdDev, JpdChn, s32EndPos,
                       VESFILE_READER_BATCH);
            }
            bFindEndCode = find_end_code(&pu8Buf[s32EndPos - 2]);
        }

        if (!bFindEndCode)
        {
            fseek(pFile, 0, SEEK_SET);
            continue;
        }

        u32FrameLen = s32EndPos - s32StartPos;
        pu8Buf += s32StartPos;
        /*printf("u32FrameLen:%u s32StartPos:%d s32EndPos:%d\n", u32FrameLen, s32StartPos, s32EndPos);*/
        /*save_stream_data(JpdDev, JpdChn, pu8Buf, u32FrameLen);*/

        u32RequiredLength = u32FrameLen;
        memset(&stRetStreamBuf, 0x0, sizeof(MI_JPD_StreamBuf_t));
        s32TimeOutMs = 30; // timeout is 30ms
        s32Ret       = MI_JPD_GetStreamBuf(u32JpdDevId, JpdChn, u32RequiredLength, &stRetStreamBuf, s32TimeOutMs);
        if (MI_SUCCESS != s32Ret)
        {
            printf("chn%u MI_JPD_GetStreamBuf failed, s32Ret: 0x%x.\n", JpdChn, s32Ret);
            continue;
        }

        memcpy(stRetStreamBuf.pu8HeadVirtAddr, pu8Buf, JPD_MIN(stRetStreamBuf.u32HeadLength, u32RequiredLength));

        if (stRetStreamBuf.u32HeadLength + stRetStreamBuf.u32TailLength < u32RequiredLength)
        {
            // something wrong happen
            printf("MI_JPD_GetStreamBuf return wrong value: HeadLen%u TailLen%u RequiredLength%u\n",
                   stRetStreamBuf.u32HeadLength, stRetStreamBuf.u32TailLength, u32RequiredLength);
            s32Ret = MI_JPD_DropStreamBuf(u32JpdDevId,JpdChn, &stRetStreamBuf);
            if (MI_SUCCESS != s32Ret)
            {
                printf("chn%u MI_JPD_DropStreamBuf failed, s32Ret: 0x%x.\n", JpdChn, s32Ret);
                continue;
            }
        }
        else if (stRetStreamBuf.u32TailLength > 0)
        {
            memcpy(stRetStreamBuf.pu8TailVirtAddr, pu8Buf + stRetStreamBuf.u32HeadLength,
                   JPD_MIN(stRetStreamBuf.u32TailLength, u32RequiredLength - stRetStreamBuf.u32HeadLength));
        }

        stRetStreamBuf.u32ContentLength = u32RequiredLength;
        s32Ret                          = MI_JPD_PutStreamBuf(u32JpdDevId,JpdChn, &stRetStreamBuf);
        if (MI_SUCCESS != s32Ret)
        {
            printf("chn%u MI_JPD_PutStreamBuf failed, s32Ret: 0x%x.\n", JpdChn, s32Ret);
            continue;
        }
        /*printf("chn%u frame:%u PA:0x%llx VA:0x%p Length:0x%x\n", JpdChn, u32PushFrames++, stRetStreamBuf.u64HeadPhyAddr,
               stRetStreamBuf.pu8HeadVirtAddr, stRetStreamBuf.u32HeadLength);*/

        memset(&stChnStatus, 0x0, sizeof(stChnStatus));
        MI_JPD_GetChnStatus(u32JpdDevId, JpdChn, &stChnStatus);
        /*printf("Chn(%d), Frame Dec:%d\n", JpdChn, stChnStatus.u32DecodeStreamFrames);*/
    }

    fclose(pFile);
    free(pu8Buf);
#endif
    return NULL;
}

void * ST_AiIspThread(void * args)
{
    MI_S32 s32Ret = MI_SUCCESS;
    MI_SYS_BUF_HANDLE hBufHandle;
    MI_SYS_BufInfo_t stInputBufInfo;
    MI_SYS_BufInfo_t stOutputBufInfo;
    ST_AiIspThreadAttr_t *pstAiIspThreadAttr = (ST_AiIspThreadAttr_t*)args;
    //MI_U8 *pSrcData = NULL, *pDstData = NULL;
    MI_ISP_DEV devId = (MI_ISP_DEV)pstAiIspThreadAttr->u8DevId;
    MI_ISP_CHANNEL chnId = (MI_ISP_CHANNEL)pstAiIspThreadAttr->u8ChnId;

    printf("%s devId %u, chnId %u\n", __func__, devId, chnId);

    while(!pstAiIspThreadAttr->bExit)
    {
        s32Ret = MI_ISP_GetCustSegBuf(devId, chnId, &hBufHandle,
            &stInputBufInfo, &stOutputBufInfo, 0);
        if(s32Ret == MI_SUCCESS)
        {
            if(E_MI_SYS_BUFDATA_FRAME == stInputBufInfo.eBufType
               && E_MI_SYS_BUFDATA_FRAME == stOutputBufInfo.eBufType)
            {
#if MI_IPU_SUPPORT
                if(pstAiIspThreadAttr->bUseIpu)
                {
                    ST_Ipu_Process(pstAiIspThreadAttr->u32IpuChn, &stInputBufInfo, &stOutputBufInfo);
                }
                else
#endif
                {
                    memcpy(stOutputBufInfo.stFrameData.pVirAddr[0], stInputBufInfo.stFrameData.pVirAddr[0], stOutputBufInfo.stFrameData.u32BufSize);
                    memset(stOutputBufInfo.stFrameData.pVirAddr[0], 0, stOutputBufInfo.stFrameData.u32BufSize/8);
                }
                MI_ISP_PutCustSegBuf(devId, chnId, hBufHandle);
            }
            else
            {
                printf("buffer type not support now. type(%d, %d) \n", stInputBufInfo.eBufType, stOutputBufInfo.eBufType);
            }
        }
        else
        {
            usleep(10000);
        }

    }

    printf("%s devId %u, chnId %u exit\n", __func__, devId, chnId);
    pthread_exit(NULL);
    return NULL;
}

MI_U32 ST_CalculateFrameSize(MI_SYS_BufInfo_t *pstBufInfo)
{
    MI_U32 u32FrameSize = MAX_U32_VALUE;
    if(E_MI_SYS_BUFDATA_FRAME == pstBufInfo->eBufType)
    {
        if(E_MI_SYS_COMPRESS_MODE_NONE == pstBufInfo->stFrameData.eCompressMode &&
           (E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420 ==pstBufInfo->stFrameData.ePixelFormat ||
            E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420_NV21 ==pstBufInfo->stFrameData.ePixelFormat))
        {
            u32FrameSize = pstBufInfo->stFrameData.u32Stride[0] * pstBufInfo->stFrameData.u16Height * 3 /2;
            if(pstBufInfo->bCrcCheck == TRUE)
            {
                u32FrameSize += 16*2; //CRC SIZE 16BYTE
            }
        }
        else
        {
            u32FrameSize = pstBufInfo->stFrameData.u32BufSize;
        }
    }
    else
    {
        printf("%s eBufType %d  not support in demo\n", __func__, pstBufInfo->eBufType);
    }

    return u32FrameSize;
}

MI_S32 ST_WriteFrameData2File(char *pFilePath, FILE **fp, MI_SYS_BufInfo_t *pstBufInfo, MI_U16 *pu16DumpNum, MI_BOOL bRecover)//pFilePath=/path/name
{
    MI_U32 u32OneFrameSize = 0;
    u32OneFrameSize = ST_CalculateFrameSize(pstBufInfo);
    if(MAX_U32_VALUE == u32OneFrameSize || u32OneFrameSize > pstBufInfo->stFrameData.u32BufSize)
    {
        printf("%s fail to write file %s, u32OneFrameSize %u, stFrameData.u32BufSize %u\n",
         __func__, pFilePath, u32OneFrameSize, pstBufInfo->stFrameData.u32BufSize);
        return -1;
    }

    return ST_WriteFile(pFilePath, fp, (MI_U8*)pstBufInfo->stFrameData.pVirAddr[0], u32OneFrameSize, pu16DumpNum, bRecover);
}

void * ST_PutInputDataThread(void * args)
{
    MI_SYS_ChnPort_t stChnInput;
    MI_SYS_BUF_HANDLE hHandle = 0;
    MI_SYS_BufConf_t stBufConf;
    MI_SYS_BufInfo_t stBufInfo;
    ST_InputFile_Attr_t *pstInputFileattr  = (ST_InputFile_Attr_t *)(args);
    MI_S32 s32Ret;
    MI_U32 offset_size = 0;
    int src_fd;
    time_t stTime = 0;
    char src_file[256];
    MI_U32 u32FileSize=0;
    MI_PHY phyFileAddr=0;
    void *pFileAddr=NULL;
    MI_U32 u32OneFrameSize = 0;

    memset(&stChnInput, 0x0, sizeof(MI_SYS_ChnPort_t));
    memset(&stBufConf, 0x0, sizeof(MI_SYS_BufConf_t));
    memset(&stBufInfo, 0x0, sizeof(MI_SYS_BufInfo_t));

    stChnInput.eModId = pstInputFileattr->stModuleInfo.eModId;
    stChnInput.u32DevId = pstInputFileattr->stModuleInfo.u32DevId;
    stChnInput.u32ChnId = pstInputFileattr->stModuleInfo.u32ChnId;
    stChnInput.u32PortId = pstInputFileattr->stModuleInfo.u32PortId;

    printf("[%s]module %d, dev %d, chn %d, port %d start get input  buffer\n", __FUNCTION__, stChnInput.eModId,stChnInput.u32DevId,stChnInput.u32ChnId,stChnInput.u32PortId);
    stBufConf.eBufType = E_MI_SYS_BUFDATA_FRAME;
    stBufConf.u64TargetPts = time(&stTime);
    stBufConf.bCrcCheck = pstInputFileattr->bCrcCheck;
    stBufConf.stFrameCfg.eFormat = pstInputFileattr->ePixelFormat;
    stBufConf.stFrameCfg.eFrameScanMode = E_MI_SYS_FRAME_SCAN_MODE_PROGRESSIVE;
    stBufConf.stFrameCfg.eCompressMode = pstInputFileattr->eCompress;
    stBufConf.stFrameCfg.u16Width = pstInputFileattr->u32Width;
    stBufConf.stFrameCfg.u16Height = pstInputFileattr->u32Height;
    sprintf(src_file,"%s",pstInputFileattr->InputFilePath);
    s32Ret = vpe_OpenSourceFile(src_file, &src_fd);
    if(s32Ret < 0)
    {
        UTStatus = UT_CASE_FAIL;
        DBG_ERR("open file %s fail!\n", src_file);
        return NULL;
    }

    u32FileSize=file_size(src_file);
    printf("get file %s size %d \n", src_file, u32FileSize);
    if(u32FileSize == 0)
    {
        close(src_fd);
        UTStatus = UT_CASE_FAIL;
        DBG_ERR("src file %s get file size fail !\n", src_file);
        return NULL;
    }

    MI_SYS_MMA_Alloc(0, (MI_U8 *)"mma_heap_name0", u32FileSize, &phyFileAddr);
    if(phyFileAddr == 0)
    {
        close(src_fd);
        UTStatus = UT_CASE_FAIL;
        DBG_ERR("get mma size fail \n");
        return NULL;
    }

    MI_SYS_Mmap(phyFileAddr, u32FileSize, &pFileAddr, FALSE);
    if(pFileAddr == NULL)
    {
        close(src_fd);
        UTStatus = UT_CASE_FAIL;
        DBG_ERR("mmap fail \n");
        MI_SYS_MMA_Free(0, phyFileAddr);
        return NULL;
    }

    if(1 != vpe_GetOneFrame(src_fd, (char *)pFileAddr, u32FileSize))
    {
        close(src_fd);
        UTStatus = UT_CASE_FAIL;
        DBG_ERR("read file size fail \n");
        return NULL;
    }

    while (!bExit)
    {
        if(MI_SUCCESS == MI_SYS_ChnInputPortGetBuf(&stChnInput,&stBufConf,&stBufInfo,&hHandle,0))
        {
            u32OneFrameSize = ST_CalculateFrameSize(&stBufInfo);

            //start user put int buffer
            //printf("file size %d, buffer size %d, offset size %d \n", u32FileSize, stBufInfo.stFrameData.u32BufSize, offset_size);
            if(u32FileSize >= u32OneFrameSize && offset_size + stBufInfo.stFrameData.u32BufSize >= u32OneFrameSize)
            {
                MI_SYS_MemcpyPa(0, stBufInfo.stFrameData.phyAddr[0], phyFileAddr+offset_size, u32OneFrameSize);
                //memcpy(stBufInfo.stFrameData.pVirAddr[0], (MI_U8 *)pFileAddr+offset_size, stBufInfo.stFrameData.u32BufSize);
                offset_size += u32OneFrameSize;

                if(pstInputFileattr->stContentCropWindow.u16Width!=0 && pstInputFileattr->stContentCropWindow.u16Height!=0)
                {
                    stBufInfo.stFrameData.stContentCropWindow.u16X = pstInputFileattr->stContentCropWindow.u16X;
                    stBufInfo.stFrameData.stContentCropWindow.u16Y = pstInputFileattr->stContentCropWindow.u16Y;
                    stBufInfo.stFrameData.stContentCropWindow.u16Width = pstInputFileattr->stContentCropWindow.u16Width;
                    stBufInfo.stFrameData.stContentCropWindow.u16Height = pstInputFileattr->stContentCropWindow.u16Height;
                }

                s32Ret = MI_SYS_ChnInputPortPutBuf(hHandle,&stBufInfo,FALSE);

                if(u32FileSize < offset_size + u32OneFrameSize)
                    offset_size = 0;
            }
            else
            {
                if(u32FileSize < u32OneFrameSize || stBufInfo.stFrameData.u32BufSize < u32OneFrameSize)
                {
                    UTStatus = UT_CASE_FAIL;
                    DBG_ERR("file %s size small than config width %d, height %d, pixel %d \n", src_file, stBufConf.stFrameCfg.u16Width, stBufConf.stFrameCfg.u16Height, stBufConf.stFrameCfg.eFormat);
                    DBG_ERR("u32FileSize %u,  stBufInfo.stFrameData.u32BufSize %u, u32OneFrameSize = %u\n", u32FileSize, stBufInfo.stFrameData.u32BufSize, u32OneFrameSize);
                }
                offset_size=0;
                s32Ret = MI_SYS_ChnInputPortPutBuf(hHandle,&stBufInfo,TRUE);
            }

        }
        if(pstInputFileattr->u32SleepMs == 0)
            usleep(THREAD_SLEEP_TIME_US);
        else
            usleep(pstInputFileattr->u32SleepMs*1000);
    }

    MI_SYS_Munmap(pFileAddr, u32FileSize);
    MI_SYS_MMA_Free(0, phyFileAddr);
    close(src_fd);
    return NULL;
}

MI_S32 ST_DoChangeStretchBuff(char *sFilePath, MI_SYS_PixelFormat_e eInputPixel, MI_SYS_WindowSize_t stInputWinSize, MI_U32 u32InputStride,
    MI_SYS_WindowRect_t stCropWin, MI_SYS_WindowSize_t stOutputWinSize, MI_SYS_PixelFormat_e eOutputPixel, char *sFileOutputPath)
{
    MI_S32 s32Ret = MI_SUCCESS;
#if MI_SCL_SUPPORT

    char sDestFilePath[128];
    FILE *fp = NULL;
    MI_U16 u16DumpBufferNum=1;
    int src_fd = 0;
    MI_U32  u32SrcSize=0, u32DestSize=0;
    struct stat statbuff;
    MI_PHY  inputphyaddr =0, outputaddr=0;
    void *pviraddr = NULL, *pviroutaddr=NULL;
    time_t stTime = 0;
    MI_U32 u32SrcAddrOffset[2]={0};
    MI_U32 u32DstAddrOffset[2]={0};
    MI_U32 u32DestStride[3] ={0};
    MI_U16 u16ValidStride[3] = {0};

    if(stat(sFilePath, &statbuff) < 0)
    {
        UTStatus = UT_CASE_FAIL;
        ST_ERR("Bb table file not exit!\n");
        s32Ret = -1;
        goto EXIT;
    }
    else
    {
        if (statbuff.st_size == 0)
        {
            UTStatus = UT_CASE_FAIL;
            ST_ERR("File size is zero!\n");
            s32Ret = -1;
            goto EXIT;

        }
        u32SrcSize = statbuff.st_size;
    }

    if(vpe_OpenSourceFile(sFilePath, &src_fd) < 0)
    {
        printf("open file fail!\n");
        s32Ret = -1;
        goto EXIT;

    }

    printf("input pixel %d, stride %d, rect(%d,%d), output pixel %d, crop(%d,%d,%d,%d) rect(%d,%d) \n", eInputPixel, u32InputStride, stInputWinSize.u16Width, stInputWinSize.u16Height,
        eOutputPixel,stCropWin.u16X, stCropWin.u16Y, stCropWin.u16Width, stCropWin.u16Height, stOutputWinSize.u16Width, stOutputWinSize.u16Height);

    if(eInputPixel == E_MI_SYS_PIXEL_FRAME_YUV422_YUYV
        || eInputPixel == E_MI_SYS_PIXEL_FRAME_YUV422_UYVY
        || eInputPixel == E_MI_SYS_PIXEL_FRAME_YUV422_YVYU
        || eInputPixel == E_MI_SYS_PIXEL_FRAME_YUV422_VYUY)
    {
        u32SrcSize = u32InputStride * stInputWinSize.u16Height;
        u16ValidStride[0] = stInputWinSize.u16Width*2;
        u32SrcAddrOffset[0] = 0;
    }
    else if(eInputPixel == E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420
        || eInputPixel == E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420_NV21
           )
    {
        u32SrcSize = u32InputStride * stInputWinSize.u16Height*3/2;
        u16ValidStride[0] = stInputWinSize.u16Width;
        u16ValidStride[1] = stInputWinSize.u16Width;
        u32SrcAddrOffset[0] = u32InputStride*stInputWinSize.u16Height;
    }
    else if(eInputPixel == E_MI_SYS_PIXEL_FRAME_ARGB8888
        || eInputPixel == E_MI_SYS_PIXEL_FRAME_ABGR8888
        || eInputPixel == E_MI_SYS_PIXEL_FRAME_BGRA8888)
    {
        u32SrcSize = u32InputStride * stInputWinSize.u16Height;
        u16ValidStride[0] = stInputWinSize.u16Width*4;
        u32SrcAddrOffset[0] = 0;
    }
    else if(eInputPixel == E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_422)
    {
        u32SrcSize = u32InputStride*stInputWinSize.u16Height*2;
        u16ValidStride[0] = stInputWinSize.u16Width;
        u16ValidStride[1] = stInputWinSize.u16Width;
        u32SrcAddrOffset[0] = u32InputStride*stInputWinSize.u16Height;
    }
    else if(eInputPixel == E_MI_SYS_PIXEL_FRAME_YUV420_PLANAR)
    {
        u32SrcSize = u32InputStride * stInputWinSize.u16Height*3/2;
        u16ValidStride[0] = stInputWinSize.u16Width;
        u16ValidStride[1] = stInputWinSize.u16Width/2;
        u16ValidStride[2] = stInputWinSize.u16Width/2;
        u32SrcAddrOffset[0] = u32InputStride*stInputWinSize.u16Height;
        u32SrcAddrOffset[1] = u32SrcAddrOffset[0]+u32InputStride/2*stInputWinSize.u16Height/2;
    }
    else if(eInputPixel == E_MI_SYS_PIXEL_FRAME_YUV422_PLANAR)
    {
        u32SrcSize = u32InputStride * stInputWinSize.u16Height*2;
        u16ValidStride[0] = stInputWinSize.u16Width;
        u16ValidStride[1] = stInputWinSize.u16Width/2;
        u16ValidStride[2] = stInputWinSize.u16Width/2;
        u32SrcAddrOffset[0] = u32InputStride*stInputWinSize.u16Height;
        u32SrcAddrOffset[1] = u32SrcAddrOffset[0]+u32InputStride/2*stInputWinSize.u16Height;
    }

    stOutputWinSize.u16Width = ALIGN_UP(stOutputWinSize.u16Width,8);
    stOutputWinSize.u16Height = ALIGN_UP(stOutputWinSize.u16Height,2);
    if(eOutputPixel == E_MI_SYS_PIXEL_FRAME_YUV422_YUYV
        || eOutputPixel == E_MI_SYS_PIXEL_FRAME_YUV422_UYVY
        || eOutputPixel == E_MI_SYS_PIXEL_FRAME_YUV422_YVYU
        || eOutputPixel == E_MI_SYS_PIXEL_FRAME_YUV422_VYUY)
    {
        u32DestStride[0] = ALIGN_UP(stOutputWinSize.u16Width, 16)*2;
        u32DestSize = u32DestStride[0]*stOutputWinSize.u16Height;
        u32DstAddrOffset[0]=0;
    }
    else if(eOutputPixel == E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420
        || eOutputPixel == E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420_NV21
        )
    {
        u32DestStride[0] = ALIGN_UP(stOutputWinSize.u16Width, 16);
        u32DestStride[1] = ALIGN_UP(stOutputWinSize.u16Width, 16);
        u32DestSize = u32DestStride[0]*stOutputWinSize.u16Height*3/2;
        u32DstAddrOffset[0] = u32DestStride[0]*stOutputWinSize.u16Height;
    }
    else if(eOutputPixel == E_MI_SYS_PIXEL_FRAME_ARGB8888
        || eOutputPixel == E_MI_SYS_PIXEL_FRAME_ABGR8888
        || eOutputPixel == E_MI_SYS_PIXEL_FRAME_BGRA8888)
    {
        u32DestStride[0] = ALIGN_UP(stOutputWinSize.u16Width, 16)*4;
        u32DestSize = u32DestStride[0]*stOutputWinSize.u16Height;
        u32DstAddrOffset[0]=0;
    }
    else if(eOutputPixel == E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_422)
    {
        u32DestStride[0] = ALIGN_UP(stOutputWinSize.u16Width, 32);
        u32DestSize = u32DestStride[0]*stOutputWinSize.u16Height*3/2;
        u32DstAddrOffset[0]=u32DestStride[0]*stOutputWinSize.u16Height;
    }
    else if(eOutputPixel == E_MI_SYS_PIXEL_FRAME_YUV420_PLANAR)
    {
        u32DestStride[0] = ALIGN_UP(stOutputWinSize.u16Width, 32);
        u32DestSize = u32DestStride[0] * stOutputWinSize.u16Height*3/2;
        u32DestStride[1] = u32DestStride[0]/2;
        u32DestStride[2] = u32DestStride[0]/2;
        u32DstAddrOffset[0] = u32DestStride[0]*stOutputWinSize.u16Height;
        u32DstAddrOffset[1] = u32DstAddrOffset[0]+u32DestStride[1]*stOutputWinSize.u16Height/2;
    }
    else if(eOutputPixel == E_MI_SYS_PIXEL_FRAME_YUV422_PLANAR)
    {
        u32DestStride[0] = ALIGN_UP(stOutputWinSize.u16Width, 32);
        u32DestSize = u32DestStride[0] * stOutputWinSize.u16Height*2;
        u32DestStride[1] = u32DestStride[0]/2;
        u32DestStride[2] = u32DestStride[0]/2;
        u32DstAddrOffset[0] = u32DestStride[0]*stOutputWinSize.u16Height;
        u32DstAddrOffset[1] = u32DstAddrOffset[0]+u32DestStride[1]*stOutputWinSize.u16Height;
    }

    ExecFuncResult(MI_SYS_MMA_Alloc(0, (MI_U8*)"mma_heap_name0", u32SrcSize, &inputphyaddr), s32Ret);
    ExecFuncResult(MI_SYS_Mmap(inputphyaddr, u32SrcSize, &pviraddr, FALSE), s32Ret);
    ExecFuncResult(MI_SYS_MMA_Alloc(0, (MI_U8*)"mma_heap_name0", u32DestSize, &outputaddr), s32Ret);
    ExecFuncResult(MI_SYS_Mmap(outputaddr, u32DestSize, &pviroutaddr, FALSE), s32Ret);

    //if(1 != vpe_GetOneFrame(src_fd, (char *)pviraddr, u32SrcSize))
    if(1 != vpe_GetOneFrameByStride(src_fd, (char *)pviraddr, u16ValidStride[0], u32InputStride, u32SrcSize))
    {
        UTStatus = UT_CASE_FAIL;
        DBG_ERR("read %s size %d fail \n", sFilePath, u32SrcSize);
        s32Ret = -1;
        goto EXIT;
    }

    MI_SCL_DevAttr_t stDevAttr;
    stDevAttr.u32NeedUseHWOutPortMask = ST_SCL_USER_STRETCHBUFF_PORTID;

    ExecFuncResult(MI_SCL_CreateDevice(ST_SCL_USER_STRETCHBUFF_DEVID, &stDevAttr), s32Ret);

    MI_SCL_DirectBuf_t  stSrcBuff;
    memset(&stSrcBuff, 0x0, sizeof(MI_SCL_DirectBuf_t));
    stSrcBuff.u32Width = stInputWinSize.u16Width;
    stSrcBuff.u32Height = stInputWinSize.u16Height;
    stSrcBuff.ePixelFormat = eInputPixel;
    stSrcBuff.phyAddr[0] = inputphyaddr;
    stSrcBuff.phyAddr[1] = inputphyaddr+u32SrcAddrOffset[0];
    //stSrcBuff.phyAddr[2] = inputphyaddr+u32SrcAddrOffset[1];
    stSrcBuff.u32BuffSize = u32SrcSize;
    stSrcBuff.u32Stride[0] = u16ValidStride[0];
    stSrcBuff.u32Stride[1] = u16ValidStride[1];
    //stSrcBuff.u32Stride[2] = u16ValidStride[2];

    printf("src width %d, stride %d, size %d \n", stInputWinSize.u16Width, stSrcBuff.u32Stride[0], stSrcBuff.u32BuffSize);

    MI_SCL_DirectBuf_t  stDestBuff;
    memset(&stDestBuff, 0x0, sizeof(MI_SCL_DirectBuf_t));
    stDestBuff.u32Width = stOutputWinSize.u16Width;
    stDestBuff.u32Height = stOutputWinSize.u16Height;
    stDestBuff.ePixelFormat = eOutputPixel;
    stDestBuff.phyAddr[0] = outputaddr;
    stDestBuff.phyAddr[1] = outputaddr+u32DstAddrOffset[0];
    //stDestBuff.phyAddr[2] = outputaddr+u32DstAddrOffset[1];
    stDestBuff.u32BuffSize = u32DestSize;
    stDestBuff.u32Stride[0]=u32DestStride[0];
    stDestBuff.u32Stride[1]=u32DestStride[1];
    //stDestBuff.u32Stride[2]=u32DestStride[2];

    MI_SYS_WindowRect_t stoutputCrop;
    memset(&stoutputCrop, 0x0, sizeof(MI_SYS_WindowRect_t));
    stoutputCrop.u16X = stCropWin.u16X;
    stoutputCrop.u16Y = stCropWin.u16Y;
    stoutputCrop.u16Width = stCropWin.u16Width;
    stoutputCrop.u16Height = stCropWin.u16Height;

    printf("dest width %d, stride %d, size %d \n", stOutputWinSize.u16Width, u32DestStride[0], stDestBuff.u32BuffSize);

    ExecFuncResult(MI_SCL_StretchBuf(&stSrcBuff, &stoutputCrop, &stDestBuff, E_MI_SCL_FILTER_TYPE_AUTO), s32Ret);

    ExecFuncResult(MI_SCL_DestroyDevice(ST_SCL_USER_STRETCHBUFF_DEVID), s32Ret);

    sprintf(sDestFilePath, "%s/stretch_%dx%d_pixel%d_%ld.yuv", sFileOutputPath, stOutputWinSize.u16Width, stOutputWinSize.u16Height, eOutputPixel, time(&stTime));

    ST_WriteFile(sDestFilePath, &fp, (MI_U8 *)pviroutaddr, u32DestSize, &u16DumpBufferNum,FALSE);
    printf("write file %s success \n",sDestFilePath);

EXIT:
    if(src_fd)
    {
        close(src_fd);
    }
    MI_SYS_Munmap(pviraddr, u32SrcSize);
    MI_SYS_Munmap(pviroutaddr, u32DestSize);

    MI_SYS_MMA_Free(0,inputphyaddr);
    MI_SYS_MMA_Free(0,outputaddr);
#endif
    return s32Ret;
}

void ST_DumpIni(dictionary * d, FILE * f)
{
    int     i;
    int     nsec ;
    int     fno ;
    if((d == NULL) || (f == NULL)) return ;

    fno  = fileno(f);
    nsec = iniparser_getnsec(d);
    if(nsec < 1)
    {
        int offset = 0;
        /* No section in file: dump all keys as they are */
        for(i = 0 ; i < d->size ; i++)
        {
            if(d->key[i] == NULL)
                continue ;
            if(':' == d->key[i][0])
            {
                offset = 1;
            }
            fprintf(f, "%s = %s\n", &d->key[i][offset], d->val[i]);
        }
        fsync(fno);
        return ;
    }

    printf("%s not support\n", __func__);
    return ;
}


MI_S32 s32ST_StatisticsMD5Value(MI_U8 *pVirAddr, MI_U32 u32BuffSize, ST_MD5Info_t *pstSaveMd5Value)
{
    MI_U8  i=0, j=0;
    MD5_CTX stMD5Ctx;
    MI_U8 au8MD5Value[16];
    MI_BOOL bMD5Same=FALSE;
    MI_U8 u8Md5InfoEmptyIndex=0xff;
    MI_S32 s32Ret=-1;

    MD5Init(&stMD5Ctx);
    MD5Update(&stMD5Ctx, pVirAddr, u32BuffSize);
    MD5Final(&stMD5Ctx, au8MD5Value);
    for(j=0; j<ST_MAX_MD5_VALUE_NUM; j++)
    {
        for(i=0; i<16; i++)
        {
            if(au8MD5Value[i] != pstSaveMd5Value[j].u8MD5ExpectValue[i])
            {
                bMD5Same=FALSE;
                break;
            }
            else
            {
                bMD5Same=TRUE;
            }
        }

        if(bMD5Same == TRUE)
        {
            break;
        }

        if(pstSaveMd5Value[j].bUsed == FALSE) //all no same, save index from last one
        {
            u8Md5InfoEmptyIndex=j;
        }
    }

    if(bMD5Same == FALSE)
    {
        if(u8Md5InfoEmptyIndex < ST_MAX_MD5_VALUE_NUM)
        {
            memcpy(pstSaveMd5Value[u8Md5InfoEmptyIndex].u8MD5ExpectValue, au8MD5Value, sizeof(MI_U8)*16);
            pstSaveMd5Value[u8Md5InfoEmptyIndex].bUsed = TRUE;
            DBG_INFO("save index %d ",u8Md5InfoEmptyIndex);
        }
        else
        {
            UTStatus = UT_CASE_FAIL;
            DBG_ERR("MD5 Vaule have full new value \n");
        }

        DBG_INFO("MD5 Vaule ");
        for(i=0; i<16; i++)
        {
            printf("%02x",au8MD5Value[i]);
        }
        printf("\n");
        s32Ret = MI_SUCCESS;
    }

    return s32Ret;
}

MI_BOOL ST_MD5ValueIsTheSame(MI_U8 *pu8MD5Value, ST_MD5Info_t *pstMd5ExpectValue)
{
    MI_BOOL bMD5Same=FALSE;
    MI_U8  i=0, j=0;

    for(j=0; j<ST_MAX_MD5_VALUE_NUM; j++)
    {
        if(pstMd5ExpectValue[j].bUsed == FALSE)
            continue;

        for(i=0; i<16; i++)
        {
            if(pu8MD5Value[i] != pstMd5ExpectValue[j].u8MD5ExpectValue[i])
            {
                bMD5Same=FALSE;
                break;
            }
            else
            {
                bMD5Same=TRUE;
            }
        }

        if(bMD5Same == TRUE)
        {
            break;
        }
    }

    return bMD5Same;
}

MI_S32 s32ST_CheckMD5Value(MI_U8 *pVirAddr, MI_U32 u32BuffSize, ST_MD5Info_t *pstMd5ExpectValue)
{
    MI_U8  i=0, j=0;
    MD5_CTX stMD5Ctx;
    MI_U8 au8MD5Value[16];
    MI_BOOL bMD5Same=FALSE;
    MI_S32 s32Ret=MI_SUCCESS;

    MD5Init(&stMD5Ctx);
    MD5Update(&stMD5Ctx, pVirAddr, u32BuffSize);
    MD5Final(&stMD5Ctx, au8MD5Value);

    for(j=0; j<ST_MAX_MD5_VALUE_NUM; j++)
    {
        if(pstMd5ExpectValue[j].bUsed == FALSE)
            continue;

        for(i=0; i<16; i++)
        {
            if(au8MD5Value[i] != pstMd5ExpectValue[j].u8MD5ExpectValue[i])
            {
                bMD5Same=FALSE;
                break;
            }
            else
            {
                bMD5Same=TRUE;
            }
        }

        if(bMD5Same == TRUE)
        {
            break;
        }
    }

    if(bMD5Same == FALSE)
    {
        UTStatus = UT_CASE_FAIL;
        DBG_ERR("MD5 value incorrect ");
        for(i=0; i<16; i++)
        {
            printf("%02x",au8MD5Value[i]);
        }
        printf("\n");
        s32Ret = -1;
    }
    else
    {
        DBG_INFO("md5 same with index %d \n", j);
    }

    return s32Ret;
}

MI_S32 ST_AddMD5Value(char *pIniPath, char *pKey, MI_U8 *pu8MD5ExpectValue, ST_MD5Info_t *pstMd5ExpectValue)
{
    FILE *fp = NULL;
    char *pstring = NULL;
    char u8Md5Str[40];
    MI_U32 i = 0, j = 0;
    MI_BOOL bMD5Same=FALSE;
    dictionary *pstDict = NULL;

    pstring = (char*)malloc(sizeof(char)*STATICS_MD5_TOTAL_CHARNUM);
    if(pstring == NULL)
    {
        DBG_ERR("md5 malloc error\n");
        return -1;
    }

    pthread_mutex_lock(&gIniMd5Mutex);
    bMD5Same = ST_MD5ValueIsTheSame(pu8MD5ExpectValue, pstMd5ExpectValue);
    if(bMD5Same)
    {
        goto EXIT;
    }

    for(j = 0; j < ST_MAX_MD5_VALUE_NUM; j++)
    {
        if(pstMd5ExpectValue[j].bUsed == FALSE)
        {
            memcpy(pstMd5ExpectValue[j].u8MD5ExpectValue,  pu8MD5ExpectValue, sizeof(MI_U8)*16);
            pstMd5ExpectValue[j].bUsed = TRUE;
            break;
        }
    }

    if( j >= ST_MAX_MD5_VALUE_NUM ){
        DBG_ERR("Exceeds the maximum length of MD5: %d\n", STATICS_MD5_TOTAL_CHARNUM);
        for(i=0; i<16; i++)
        {
           printf("%02x",pu8MD5ExpectValue[i]);
        }
        printf("\n");
        goto EXIT;
    }

    pstDict = iniparser_load(pIniPath);
    if(pstDict == NULL)
    {
        printf("%s iniparser_load failed. pIniPath %s\n", __func__, pIniPath);
        goto EXIT;
    }

    fp = fopen(pIniPath, "w");
    if(fp == NULL)
    {
        printf("%s file open %s failed\n", __func__, pIniPath);
        goto EXIT;
    }

    strcpy(pstring, "\"{");
    for(j = 0; j < ST_MAX_MD5_VALUE_NUM; j++)
    {
        if(pstMd5ExpectValue[j].bUsed == TRUE)
        {
            for(i=0; i<16; i++)
            {
                int8_to_string(pstMd5ExpectValue[j].u8MD5ExpectValue[i], (MI_U8*)&u8Md5Str[i << 1]);
            }
            u8Md5Str[32] = '\0';
            strcat(pstring, u8Md5Str);
            strcat(pstring, ",\\\n\t");
        }
    }
    strcat(pstring, "}\"");

    printf("set ini key %s, string %s\n", pKey, pstring);
    //printf("----------pstring sizeof = %d\n",strlen(pstring));
    iniparser_setstring(pstDict, pKey, pstring);
    ST_DumpIni(pstDict, fp);
    fclose(fp);
    iniparser_freedict(pstDict);
    fp = NULL;
    pstDict=NULL;

EXIT:
    if(fp != NULL)
    {
        fclose(fp);
    }
    if(pstDict != NULL)
    {
        iniparser_freedict(pstDict);
    }
    free(pstring);
    pthread_mutex_unlock(&gIniMd5Mutex);

    return MI_SUCCESS;
}

MI_S32 ST_MD5Action2(MI_U8 *pu8MD5Value, ST_Md5Attr_t *pstMd5Attr, ST_Md5Action_e eMd5Action)
{
    MI_U8  i=0;
    MI_BOOL bMD5Same=FALSE;
    MI_S32 s32Ret=MI_SUCCESS;

    for(i=0; i<16; i++)
    {
        int8_to_string(pu8MD5Value[i], &pstMd5Attr->Md5ValueString[i << 1]);
    }

    if(eMd5Action > E_ST_MD5_ACTION_NONE)
    {
        if(E_ST_MD5_ACTION_RESET == eMd5Action)
        {
            if(E_ST_MD5_ACTION_RESET == pstMd5Attr->eMd5Action)
            {
                pstMd5Attr->eMd5Action = E_ST_MD5_ACTION_ADD;
            }
            memset(pstMd5Attr->stMD5ExpectValue, 0, sizeof(ST_MD5Info_t) * ST_MAX_MD5_VALUE_NUM);
            eMd5Action = E_ST_MD5_ACTION_ADD;
        }
    }
    else
    {
        return MI_SUCCESS;
    }

    if(E_ST_MD5_ACTION_CHECK == eMd5Action)
    {
        bMD5Same = ST_MD5ValueIsTheSame(pu8MD5Value, pstMd5Attr->stMD5ExpectValue);
        if(bMD5Same == FALSE)
        {
            DBG_ERR("%s MD5 value incorrect \n", pstMd5Attr->key);
            s32Ret = -1;
        }
        else
        {
            DBG_INFO("%s md5 is the same \n", pstMd5Attr->key);
        }

        printf("%s\n", pstMd5Attr->Md5ValueString);
    }
    else
    {
        s32Ret = ST_AddMD5Value((char*)pstMd5Attr->pu8IniPath, (char*)pstMd5Attr->key, pu8MD5Value, pstMd5Attr->stMD5ExpectValue);
    }

    return s32Ret;
}


MI_U32 update_MD5(MD5_CTX *pstMD5Ctx, MI_SYS_BufInfo_t *pstBuffInfo)
{
    MI_U16 u16ValidStride[3] = {0};
    MI_U16 height[3] = {0};
    MI_U32 u32Offset =0;
    MI_U32 i = 0, j = 0;
    MI_SYS_PixelFormat_e inputpixel = pstBuffInfo->stFrameData.ePixelFormat;

    switch(inputpixel)
        {
            case E_MI_SYS_PIXEL_FRAME_YUV422_YUYV:
            case E_MI_SYS_PIXEL_FRAME_YUV422_UYVY:
            case E_MI_SYS_PIXEL_FRAME_YUV422_YVYU:
            case E_MI_SYS_PIXEL_FRAME_YUV422_VYUY:
            {
                u16ValidStride[0] = pstBuffInfo->stFrameData.u16Width*2;
                height[0] = pstBuffInfo->stFrameData.u16Height;
                break;
            }
            case E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420:
            case E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420_NV21:
            {
                u16ValidStride[0] = pstBuffInfo->stFrameData.u16Width;
                u16ValidStride[1] = pstBuffInfo->stFrameData.u16Width;
                height[0] = pstBuffInfo->stFrameData.u16Height;
                height[1] = pstBuffInfo->stFrameData.u16Height/2;
                break;
            }
            case E_MI_SYS_PIXEL_FRAME_ARGB8888:
            case E_MI_SYS_PIXEL_FRAME_ABGR8888:
            case E_MI_SYS_PIXEL_FRAME_BGRA8888:
            {
                u16ValidStride[0] = pstBuffInfo->stFrameData.u16Width*4;
                height[0] = pstBuffInfo->stFrameData.u16Height;
                break;
            }
            case E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_422:
            {
                u16ValidStride[0] = pstBuffInfo->stFrameData.u16Width;
                u16ValidStride[1] = pstBuffInfo->stFrameData.u16Width;
                height[0] = pstBuffInfo->stFrameData.u16Height;
                height[1] = pstBuffInfo->stFrameData.u16Height;
                break;
            }
            case E_MI_SYS_PIXEL_FRAME_YUV420_PLANAR:
            {
                u16ValidStride[0] = pstBuffInfo->stFrameData.u16Width;
                u16ValidStride[1] = pstBuffInfo->stFrameData.u16Width/2;
                u16ValidStride[2] = pstBuffInfo->stFrameData.u16Width/2;
                height[0] = pstBuffInfo->stFrameData.u16Height;
                height[1] = pstBuffInfo->stFrameData.u16Height/2;
                height[2] = pstBuffInfo->stFrameData.u16Height/2;
                break;
            }
            case E_MI_SYS_PIXEL_FRAME_YUV422_PLANAR:
            {
                u16ValidStride[0] = pstBuffInfo->stFrameData.u16Width;
                u16ValidStride[1] = pstBuffInfo->stFrameData.u16Width/2;
                u16ValidStride[2] = pstBuffInfo->stFrameData.u16Width/2;
                height[0] = pstBuffInfo->stFrameData.u16Height;
                height[1] = pstBuffInfo->stFrameData.u16Height;
                height[2] = pstBuffInfo->stFrameData.u16Height;
                break;
            }
            default:
            {
                DBG_ERR(" pixel format error \n");
                return -1;
            }
        }

    if(pstBuffInfo->bCrcCheck == TRUE
        || pstBuffInfo->stFrameData.eCompressMode > E_MI_SYS_COMPRESS_MODE_NONE)
    {
        MD5Update(pstMD5Ctx, (MI_U8 *)pstBuffInfo->stFrameData.pVirAddr[0], pstBuffInfo->stFrameData.u32BufSize);
    }
    else
    {
        for(j=0; j<3; j++)
        {
            if(pstBuffInfo->stFrameData.pVirAddr[j] != NULL)
            {
                u32Offset=0;
                for(i=0;i<height[j];i++){
                    MD5Update(pstMD5Ctx, (MI_U8 *)pstBuffInfo->stFrameData.pVirAddr[j]+u32Offset, u16ValidStride[j]);
                    u32Offset += pstBuffInfo->stFrameData.u32Stride[j];
                }
            }
        }
    }

    return 0;
}


MI_S32 ST_MD5Action(MI_SYS_BufInfo_t *pstBuffInfo, ST_Md5Attr_t *pstMd5Attr, ST_Md5Action_e eMd5Action)
{
    MD5_CTX stMD5Ctx;
    MI_U8 au8MD5Value[16];

    MD5Init(&stMD5Ctx);
    update_MD5(&stMD5Ctx, pstBuffInfo);
    MD5Final(&stMD5Ctx, au8MD5Value);
    return ST_MD5Action2(au8MD5Value, pstMd5Attr, eMd5Action);
}

MI_S32 ST_WriteFbdFile(char *pFilePath, FILE **Converfp, MI_SYS_BufInfo_t *pstBufInfo, MI_U16 *pu16DumpNum)
{
    MI_S32 s32Ret = MI_SUCCESS;
    MI_U32 u32MallocSize = pstBufInfo->stFrameData.u16Width*pstBufInfo->stFrameData.u16Height*2;
    char *ConverBuffer=(char*)malloc(u32MallocSize);
    MI_SYS_DataPrecision_e ePrecision = (MI_SYS_DataPrecision_e)((pstBufInfo->stFrameData.ePixelFormat - E_MI_SYS_PIXEL_FRAME_RGB_BAYER_BASE) / E_MI_SYS_PIXEL_BAYERID_MAX);

    if(ConverBuffer == NULL)
    {
        DBG_ERR("ConverBuffer malloc falied, no memory \n");
        return -1;
    }

    if(pstBufInfo->stFrameData.eCompressMode == E_MI_SYS_COMPRESS_MODE_TO_8BIT)
    {
#if MI_ISP_SUPPORT
        FBD_Execute((unsigned char *)pstBufInfo->stFrameData.pVirAddr[0], (unsigned short *)ConverBuffer, pstBufInfo->stFrameData.u16Width, pstBufInfo->stFrameData.u16Height);
#endif
    }
    else
    {
        ConvertRawImageTo16Bits(pstBufInfo->stFrameData.pVirAddr[0], ConverBuffer, pstBufInfo->stFrameData.u16Width, pstBufInfo->stFrameData.u16Height, ePrecision, false);
    }

    DBG_INFO("=============begin write file %s, id %d \n", pFilePath, *pu16DumpNum);
    if(MI_SUCCESS != ST_WriteFile(pFilePath, Converfp, (MI_U8 *)ConverBuffer, u32MallocSize, pu16DumpNum,FALSE))
    {
        DBG_ERR("write file err \n");
        s32Ret = -1;
        goto EXIT;
    }


EXIT:

    free(ConverBuffer);
    ConverBuffer = NULL;

    return s32Ret;
}

MI_S32 ST_GetModuleOutputDataOneFile(ST_DumpFileInfo_t *pstDumpFile,MI_U16 u16NodeId)
{
    MI_SYS_BufInfo_t stBufInfo;
    MI_SYS_BUF_HANDLE hHandle;
    FILE *fp = NULL;
    FILE *Converfp = NULL;
    MI_S32 s32Ret = MI_SUCCESS;
    MI_S32 s32Fd = 0;
    fd_set read_fds;
    struct timeval TimeoutVal;
    ST_DumpFileInfo_t *pstDumpFileInfo = pstDumpFile;
    MI_SYS_ChnPort_t stChnPort;
    char FilePath[256];
    char FileConverPath[512];
    MI_U16 u16DumpBuffNum =  pstDumpFileInfo->u16FileCnt;
    MI_U16 u16FbdDumpBuffNum = u16DumpBuffNum;
    ST_OutputFile_Attr_t *pstOutputFileAttr = (ST_OutputFile_Attr_t *)pstDumpFile->poutputFileAttr;
#if ST_NEED_STATICS_MD5_VALUE
    ST_MD5Info_t stMD5Info[ST_MAX_MD5_VALUE_NUM];
    memset(stMD5Info, 0x0, sizeof(ST_MD5Info_t)*ST_MAX_MD5_VALUE_NUM);
#endif
    memset(&stChnPort, 0x0, sizeof(MI_SYS_ChnPort_t));
    stChnPort.eModId = pstDumpFileInfo->stChnPort.eModId;
    stChnPort.u32DevId = pstDumpFileInfo->stChnPort.u32DevId;
    stChnPort.u32ChnId = pstDumpFileInfo->stChnPort.u32ChnId;
    stChnPort.u32PortId = pstDumpFileInfo->stChnPort.u32PortId;

    pthread_mutex_lock(&pstOutputFileAttr->Portmutex);

    s32Ret = MI_SYS_GetFd(&stChnPort,&s32Fd);
    if(s32Ret != MI_SUCCESS)
    {
        UTStatus = UT_CASE_FAIL;
        DBG_ERR("MI_SYS_GetFd error %d\n",s32Ret);
        s32Ret = UT_CASE_FAIL;
        goto UNLOCK_EXIT;
    }
    while(u16DumpBuffNum > 0)
    {
        FD_ZERO(&read_fds);
        FD_SET(s32Fd,&read_fds);
        TimeoutVal.tv_sec = 1;
        TimeoutVal.tv_usec = 0;
        s32Ret = select(s32Fd + 1, &read_fds, NULL, NULL, &TimeoutVal);

        if(s32Ret < 0)
        {
            ST_ERR("select fail\n");
            usleep(THREAD_SLEEP_TIME_US);
            continue;
        }
        else if(s32Ret == 0)
        {
            usleep(THREAD_SLEEP_TIME_US);
            continue;
        }
        else
        {
            if(FD_ISSET(s32Fd,&read_fds))
            {
                if (MI_SUCCESS == MI_SYS_ChnOutputPortGetBuf(&stChnPort, &stBufInfo, &hHandle))
                {
                    char TempFilePath[192];
                    if(stBufInfo.bCrcCheck == TRUE)
                    {
                        time_t stTime = 0;
                        sprintf(TempFilePath, "%s/moudle%dDev%dChn%dPort%dNode%d_%dx%d_Pixel%d_compress%d_CRC_%ld",pstDumpFileInfo->FilePath, stChnPort.eModId, stChnPort.u32DevId, stChnPort.u32ChnId, stChnPort.u32PortId,
                                        u16NodeId,stBufInfo.stFrameData.u16Width,stBufInfo.stFrameData.u16Height,stBufInfo.stFrameData.ePixelFormat, stBufInfo.stFrameData.eCompressMode, time(&stTime));
                    }
                    else
                    {
                        sprintf(TempFilePath, "%s/moudle%dDev%dChn%dPort%dNode%d_%dx%d_Pixel%d_compress%d",pstDumpFileInfo->FilePath, stChnPort.eModId, stChnPort.u32DevId, stChnPort.u32ChnId, stChnPort.u32PortId,
                                        u16NodeId,stBufInfo.stFrameData.u16Width,stBufInfo.stFrameData.u16Height,stBufInfo.stFrameData.ePixelFormat, stBufInfo.stFrameData.eCompressMode);
                    }

                    if(pstOutputFileAttr->stMd5Attr.eMd5Action > E_ST_MD5_ACTION_NONE)
                    {
                        s32Ret =ST_MD5Action(&stBufInfo, &pstOutputFileAttr->stMd5Attr, pstOutputFileAttr->stMd5Attr.eMd5Action);
                        //s32Ret =s32ST_CheckMD5Value((MI_U8 *)stBufInfo.stFrameData.pVirAddr[0], stBufInfo.stFrameData.u32BufSize, pstDumpFileInfo->stMD5ExpectValue);
                        if(s32Ret != MI_SUCCESS)
                        {
                            char Md5FailFilePath[356];
                            FILE *Md5Failfp = NULL;
                            MI_U16 u16DumpBufferNum =1;
                            UTStatus = UT_CASE_FAIL;
                            DBG_ERR("(%d-%d-%d-%d) MD5 not correct \n", stChnPort.eModId, stChnPort.u32DevId, stChnPort.u32ChnId,stChnPort.u32PortId);

                            sprintf(Md5FailFilePath, "%s_md5fail_value%s.yuv", TempFilePath, pstOutputFileAttr->stMd5Attr.Md5ValueString);
                            if(MI_SUCCESS != ST_WriteFrameData2File(Md5FailFilePath, &Md5Failfp, &stBufInfo, &u16DumpBufferNum,TRUE))
                            {
                                DBG_ERR("write file err \n");
                                s32Ret = UT_CASE_FAIL;
                                goto UNLOCK_EXIT;
                            }
                        }
                    }

                    sprintf(FilePath, "%s.yuv",TempFilePath);
                    printf("=======begin writ port %d file id %d, file path %s, bufsize %d, stride %d, height %d\n", stChnPort.u32PortId, u16DumpBuffNum, FilePath,
                            stBufInfo.stFrameData.u32BufSize,stBufInfo.stFrameData.u32Stride[0], stBufInfo.stFrameData.u16Height);

                    if(MI_SUCCESS != ST_WriteFrameData2File(FilePath, &fp, &stBufInfo, &u16DumpBuffNum,FALSE))
                    {
                        DBG_ERR("write file err \n");

                        if(MI_SUCCESS != MI_SYS_ChnOutputPortPutBuf(hHandle))
                        {
                            UTStatus = UT_CASE_FAIL;
                            DBG_ERR("(%d-%d-%d-%d) put buf error \n", stChnPort.eModId, stChnPort.u32DevId, stChnPort.u32ChnId,stChnPort.u32PortId);
                        }
                        s32Ret = UT_CASE_FAIL;
                        goto UNLOCK_EXIT;
                    }

                    if(stChnPort.eModId == E_MI_MODULE_ID_VIF
                                && stBufInfo.stFrameData.ePixelFormat > E_MI_SYS_PIXEL_FRAME_RGB_BAYER_BASE
                                && stBufInfo.stFrameData.ePixelFormat < E_MI_SYS_PIXEL_FRAME_RGB_BAYER_NUM)
                    {
                        if(pstOutputFileAttr->bNeedFbd == TRUE)
                        {
                            sprintf(FileConverPath, "%s_conver", FilePath);
                            ST_WriteFbdFile(FileConverPath, &Converfp, &stBufInfo, &u16FbdDumpBuffNum);
                        }
                    }

                    if(MI_SUCCESS != MI_SYS_ChnOutputPortPutBuf(hHandle))
                    {
                        UTStatus = UT_CASE_FAIL;
                        DBG_ERR("(%d-%d-%d-%d) put buf error \n", stChnPort.eModId, stChnPort.u32DevId, stChnPort.u32ChnId,stChnPort.u32PortId);
                    }
                }
            }
        }
    }

    s32Ret = MI_SUCCESS;

UNLOCK_EXIT:
    pthread_mutex_unlock(&pstOutputFileAttr->Portmutex);

    return s32Ret;
}

void * ST_GetOutputDataThread(void * args)
{
    MI_SYS_BufInfo_t stBufInfo;
    MI_SYS_BUF_HANDLE hHandle;
    ST_OutputFile_Attr_t *pstOutFileAttr = ((ST_OutputFile_Attr_t *)(args));
    FILE *Converfp = NULL;
    MI_S32 s32Ret = MI_SUCCESS;
    MI_S32 s32Fd = 0;
    fd_set read_fds;
    struct timeval TimeoutVal;
    MI_SYS_ChnPort_t stChnPort;
    ST_IspChannelAttr_t  *pstIspChnAttr = NULL;
    time_t stTime = 0;
    char TempFilePath[140];
    char FilePath[180];
    char FileConverPath[512];
    MI_U32 u32FbdDumpBufferNum=0;
    MI_BOOL bFirstDump=TRUE;
#if ST_NEED_STATICS_MD5_VALUE
    ST_MD5Info_t stMD5Info[ST_MAX_MD5_VALUE_NUM];
    memset(stMD5Info, 0x0, sizeof(ST_MD5Info_t)*ST_MAX_MD5_VALUE_NUM);
#endif
    memset(&stChnPort, 0x0, sizeof(MI_SYS_ChnPort_t));
    stChnPort.eModId = pstOutFileAttr->stModuleInfo.eModId;
    stChnPort.u32DevId = pstOutFileAttr->stModuleInfo.u32DevId;
    stChnPort.u32ChnId = pstOutFileAttr->stModuleInfo.u32ChnId;
    stChnPort.u32PortId = pstOutFileAttr->stModuleInfo.u32PortId;
    s32Ret = MI_SYS_GetFd(&stChnPort,&s32Fd);
    if(s32Ret != MI_SUCCESS)
    {
        UTStatus = UT_CASE_FAIL;
        ST_ERR("MI_SYS_GetFd error %d\n",s32Ret);
        return NULL;
    }
    if(stChnPort.eModId == E_MI_MODULE_ID_ISP)
    {
        pstIspChnAttr = &gstIspModule.stIspDevAttr[stChnPort.u32DevId].stIspChnlAttr[stChnPort.u32ChnId];
    }

    while (!pstOutFileAttr->bThreadExit)
    {
        FD_ZERO(&read_fds);
        FD_SET(s32Fd,&read_fds);
        TimeoutVal.tv_sec = 1;
        TimeoutVal.tv_usec = 0;
        s32Ret = select(s32Fd + 1, &read_fds, NULL, NULL, &TimeoutVal);

        if(s32Ret < 0)
        {
            ST_ERR("select fail\n");
            usleep(THREAD_SLEEP_TIME_US);
            continue;
        }
        else if(s32Ret == 0)
        {
            if(pstOutFileAttr->u16UserDepth !=0)
            {
               //ST_ERR("time out\n");
            }
            usleep(THREAD_SLEEP_TIME_US);
            continue;
        }
        else
        {
            if((pstIspChnAttr != NULL)
                && (pstIspChnAttr->stIspSkipFarme.u32SikpFrameCnt > 0))
            {
                struct timeval  stEndTime;
                MI_U64 u64NowInterval,u64RightInterval,u64ErrInterval;
                gettimeofday(&stEndTime, NULL);

                pthread_mutex_lock(&pstIspChnAttr->stIspSkipFarme.SkipMutex);
                u64NowInterval = GET_DIFF_TIME_US(pstIspChnAttr->stIspSkipFarme.stSatrtTime,stEndTime);
                if(pstIspChnAttr->stIspInPortAttr[0].stBindParam.u32DstFrmrate == 0)
                {
                     DBG_INFO("ISP DoSetIspSkipFrame only support Bind type\n");
                }
                u64RightInterval = (1000000/pstIspChnAttr->stIspInPortAttr[0].stBindParam.u32DstFrmrate)*pstIspChnAttr->stIspSkipFarme.u32SikpFrameCnt;
                u64ErrInterval = (1000000/pstIspChnAttr->stIspInPortAttr[0].stBindParam.u32DstFrmrate)*(pstIspChnAttr->stIspSkipFarme.u32SikpFrameCnt+1);

                if(u64NowInterval >= u64RightInterval && u64NowInterval < u64ErrInterval)
                {
                    DBG_INFO("ISP Dev %d,Chn %d,\nNowInterval : %llu/us,\nStart time %lu/us\nEnd time %lu/us,\nInterval[%llu~%llu)/us\n",stChnPort.u32DevId, stChnPort.u32ChnId,
                    u64NowInterval,(pstIspChnAttr->stIspSkipFarme.stSatrtTime.tv_sec * 1000000 + pstIspChnAttr->stIspSkipFarme.stSatrtTime.tv_usec),
                    (stEndTime.tv_sec * 1000000 + stEndTime.tv_usec),u64RightInterval,u64ErrInterval);
                }
                else
                {
                    UTStatus = UT_CASE_FAIL;
                    DBG_ERR("ISP Dev %d,Chn %d,\nNowInterval : %llu/us,\nStart time %lu/us\nEnd time %lu/us,\nInterval[%llu~%llu)/us\n",stChnPort.u32DevId, stChnPort.u32ChnId,
                    u64NowInterval,(pstIspChnAttr->stIspSkipFarme.stSatrtTime.tv_sec * 1000000 + pstIspChnAttr->stIspSkipFarme.stSatrtTime.tv_usec),
                    (stEndTime.tv_sec * 1000000 + stEndTime.tv_usec),u64RightInterval,u64ErrInterval);
                }
                pstIspChnAttr->stIspSkipFarme.u32SikpFrameCnt = 0;
                pthread_mutex_unlock(&pstIspChnAttr->stIspSkipFarme.SkipMutex);
            }
            else if(FD_ISSET(s32Fd,&read_fds))
            {
                pthread_mutex_lock(&pstOutFileAttr->Portmutex);

                if(pstOutFileAttr->u16UserDepth >0)//vpe realtime bind get output map viraddr fail
                {
                    if (MI_SUCCESS == MI_SYS_ChnOutputPortGetBuf(&stChnPort, &stBufInfo, &hHandle))
                    {
                        sprintf(TempFilePath, "%s/moudle%dDev%dChn%dPort%d_%dx%d_Pixel%d", pstOutFileAttr->FilePath, stChnPort.eModId, stChnPort.u32DevId, stChnPort.u32ChnId, stChnPort.u32PortId,
                            stBufInfo.stFrameData.u16Width,stBufInfo.stFrameData.u16Height,stBufInfo.stFrameData.ePixelFormat);

                        if(pstOutFileAttr->stMd5Attr.eMd5Action > E_ST_MD5_ACTION_NONE)
                        {
                            s32Ret =ST_MD5Action(&stBufInfo, &pstOutFileAttr->stMd5Attr, pstOutFileAttr->stMd5Attr.eMd5Action);
                            //s32Ret =ST_MD5Action((MI_U8 *)stBufInfo.stFrameData.pVirAddr[0], stBufInfo.stFrameData.u32BufSize,
                            //&pstOutFileAttr->stMd5Attr, pstOutFileAttr->stMd5Attr.eMd5Action);
                            //s32Ret =s32ST_CheckMD5Value((MI_U8 *)stBufInfo.stFrameData.pVirAddr[0], stBufInfo.stFrameData.u32BufSize, pstOutFileAttr->stMD5ExpectValue);
                            if(s32Ret != MI_SUCCESS)
                            {
                                UTStatus = UT_CASE_FAIL;
                                DBG_ERR("(%d-%d-%d-%d) MD5 not correct \n", stChnPort.eModId, stChnPort.u32DevId, stChnPort.u32ChnId,stChnPort.u32PortId);
                                {
                                    char Md5FailFilePath[256];
                                    FILE *Md5Failfp = NULL;
                                    MI_U16 u16DumpBufferNum =1;
                                    sprintf(Md5FailFilePath, "%s_md5fail_value%s.yuv", TempFilePath, pstOutFileAttr->stMd5Attr.Md5ValueString);
                                    if(MI_SUCCESS != ST_WriteFrameData2File(Md5FailFilePath, &Md5Failfp, &stBufInfo, &u16DumpBufferNum,TRUE))
                                    {
                                        DBG_ERR("write file err \n");
                                        pthread_mutex_unlock(&pstOutFileAttr->Portmutex);
                                        return NULL;
                                    }
                                }
                            }
                        }

                        if(pstOutFileAttr->s32DumpBuffNum > 0)
                        {
                            if(bFirstDump == TRUE)
                            {
                                sprintf(FilePath, "%s_%ld.yuv", TempFilePath, time(&stTime));
                                u32FbdDumpBufferNum = pstOutFileAttr->s32DumpBuffNum;
                                bFirstDump = FALSE;
                            }
                            //printf("get out success \n");

                            printf("=======begin writ port %d file id %d, file path %s, bufsize %d, stride %d, height %d\n", stChnPort.u32PortId, pstOutFileAttr->s32DumpBuffNum, FilePath,
                                stBufInfo.stFrameData.u32BufSize,stBufInfo.stFrameData.u32Stride[0], stBufInfo.stFrameData.u16Height);

                            if(stChnPort.eModId == E_MI_MODULE_ID_VIF
                                && stBufInfo.stFrameData.ePixelFormat > E_MI_SYS_PIXEL_FRAME_RGB_BAYER_BASE
                                && stBufInfo.stFrameData.ePixelFormat < E_MI_SYS_PIXEL_FRAME_RGB_BAYER_NUM)
                            {
                                if(pstOutFileAttr->bNeedFbd == TRUE)
                                {
                                    sprintf(FileConverPath, "%s_conver.yuv", TempFilePath);
                                    ST_WriteFbdFile(FileConverPath, &Converfp, &stBufInfo, (MI_U16 *)&u32FbdDumpBufferNum);
                                }
                            }

                            if(MI_SUCCESS != ST_WriteFrameData2File(FilePath, &pstOutFileAttr->fp, &stBufInfo, (MI_U16 *)&pstOutFileAttr->s32DumpBuffNum,FALSE))
                            {
                                DBG_ERR("write file err \n");
                                if(MI_SUCCESS != MI_SYS_ChnOutputPortPutBuf(hHandle))
                                {
                                    UTStatus = UT_CASE_FAIL;
                                    DBG_ERR("(%d-%d-%d-%d) put buf error \n", stChnPort.eModId, stChnPort.u32DevId, stChnPort.u32ChnId,stChnPort.u32PortId);
                                }
                                pthread_mutex_unlock(&pstOutFileAttr->Portmutex);
                                return NULL;
                            }

                            if(bFirstDump == FALSE && pstOutFileAttr->s32DumpBuffNum == 0)
                            {
                                bFirstDump = TRUE;
                            }
                        }
                        //printf("begin release out \n");
                        if(MI_SUCCESS != MI_SYS_ChnOutputPortPutBuf(hHandle))
                        {
                            UTStatus = UT_CASE_FAIL;
                            DBG_ERR("(%d-%d-%d-%d) put buf error \n", stChnPort.eModId, stChnPort.u32DevId, stChnPort.u32ChnId,stChnPort.u32PortId);
                        }
                        pstOutFileAttr->u32FinishCnt++;
                        //printf("end release out \n");
                    }
                }

                pthread_mutex_unlock(&pstOutFileAttr->Portmutex);
            }
        }
    }

    MI_SYS_CloseFd(s32Fd);

    return NULL;
}
MI_BOOL ST_DoSetIqBin(MI_ISP_DEV IspDev, MI_ISP_CHANNEL Ispchn, char *pConfigPath)
{
    MI_BOOL bRet = MI_SUCCESS;
#if 1

    MI_ISP_IQ_ParamInitInfoType_t status;
    MI_U8  u8ispreadycnt = 0;
    if (strlen(pConfigPath) == 0)
    {
        printf("IQ Bin File path NULL!\n");
        bRet = -1;
        goto EXIT;
    }

    do
    {
        if(u8ispreadycnt > 100)
        {
            printf("%s:%d, isp ready time out \n", __FUNCTION__, __LINE__);
            u8ispreadycnt = 0;
            break;
        }

        ExecFuncResult(MI_ISP_IQ_GetParaInitStatus(IspDev, Ispchn, &status), bRet);
        if(status.stParaAPI.bFlag != 1)
        {
            usleep(300*1000);
            u8ispreadycnt++;
            continue;
        }

        u8ispreadycnt = 0;

        printf("loading api bin...path:%s\n",pConfigPath);
        ExecFuncResult(MI_ISP_ApiCmdLoadBinFile(IspDev, Ispchn, (char *)pConfigPath, 1234), bRet);

        usleep(THREAD_SLEEP_TIME_US);
    }while(!status.stParaAPI.bFlag);

EXIT:
#endif
    return bRet;
}

void *ST_IQthread(void * args)
{
#if MI_ISPIQ_SUPPORT
    MI_ISP_DEV IspDevId = 0;
    MI_ISP_CHANNEL IspChnId = 0;
    char IqApiBinFilePath[ST_MAX_ISP_CHN_NUM][128];

    memset(IqApiBinFilePath, 0, sizeof(char)*128*ST_MAX_ISP_CHN_NUM);
    MI_IQSERVER_Open();
    for(int i=0;i<4;i++)
    {
	 if(i == 0)
	  ST_DoSetIqBin(0, i, (char *)IQ_FILE_PATH);
	 else
	  ST_DoSetIqBin(0, i, (char *)IQ_FILE1_PATH);
	}
    while (!bExit)
    {
        for(IspDevId=0; IspDevId<ST_MAX_ISP_DEV_NUM; IspDevId++)
        {
            ST_IspDevAttr_t *pstIspDevAttr= &gstIspModule.stIspDevAttr[IspDevId];
            if(pstIspDevAttr->bUsed == TRUE && pstIspDevAttr->bCreate == TRUE)
            {
                for(IspChnId=0; IspChnId<ST_MAX_ISP_CHN_NUM; IspChnId++)
                {
                    ST_IspChannelAttr_t  *pstIspChnAttr = &pstIspDevAttr->stIspChnlAttr[IspChnId];
                    if(pstIspChnAttr->bUsed == TRUE && pstIspChnAttr->bCreate == TRUE)
                    {
                        MI_ISP_IQ_Nr3dType_t stNR3D;
                        MI_ISP_IQ_ParamInitInfoType_t status;
                        MI_ISP_ChnParam_t stIspChnParam;
                        MI_SNR_PlaneInfo_t stPlaneInfo;
                        //char IqApiBinFilePath_tmp[128];
                        char * IqApiBinFilePath_tmp;
                        MI_S32 s32ret = MI_SUCCESS;

                        memset(&stNR3D,0x0,sizeof(MI_ISP_IQ_Nr3dType_t));
                        memset(&status,0x0,sizeof(MI_ISP_IQ_ParamInitInfoType_t));
                        memset(&stIspChnParam,0x0,sizeof(MI_ISP_ChnParam_t));
                        memset(&stPlaneInfo,0x0,sizeof(MI_SNR_PlaneInfo_t));
                        //memset(IqApiBinFilePath_tmp, 0, sizeof(char)*128);

                        s32ret = MI_ISP_IQ_GetParaInitStatus(IspDevId,IspChnId, &status);
                        if((s32ret == MI_SUCCESS && status.stParaAPI.bFlag != 1) || (s32ret != MI_SUCCESS))
                        {
                        	
                            continue;
                        }

                        s32ret = MI_ISP_IQ_GetNr3d(IspDevId, IspChnId, &stNR3D);
                        if(s32ret == MI_SUCCESS && stNR3D.bEnable == E_SS_IQ_FALSE)
                        {
                            stNR3D.bEnable = E_SS_IQ_TRUE;
                            printf("SET ISP dev %d, chn %d,NR3D status:%d, type:%d\n",IspDevId, IspChnId,stNR3D.bEnable, stNR3D.enOpType);

                            if(MI_SUCCESS != MI_ISP_IQ_SetNr3d(IspDevId, IspChnId, &stNR3D))
                            {
                                DBG_ERR("isp dev%d chn%d set NR3D failed!! \n",IspDevId, IspChnId);
                            }
                        }

                        MI_ISP_GetChnParam(IspDevId, IspChnId, &stIspChnParam);
                        if(pstIspChnAttr->eBindMiSnrPadId < ST_MAX_SENSOR_NUM && bUseUserSensor == FALSE)
                        {
                            MI_SNR_GetPlaneInfo(pstIspChnAttr->eBindMiSnrPadId, 0, &stPlaneInfo);

                          // sprintf(IqApiBinFilePath_tmp, "/config/iqfile/%s_api.bin", stPlaneInfo.s8SensorName);
                        }

                        //if(0!=strcmp(IqApiBinFilePath[IspChnId], &IqApiBinFilePath_tmp))
                        //{
                          //  memcpy(IqApiBinFilePath[IspChnId], &IqApiBinFilePath_tmp, sizeof(char)*128);
                           // printf("loading api bin...path:%s\n",IqApiBinFilePath[IspChnId]);
                            //MI_ISP_ApiCmdLoadBinFile(IspDevId, IspChnId, (char *)IqApiBinFilePath[IspChnId], 1234);
                            //MI_ISP_ApiCmdLoadBinFile(IspDevId, IspChnId, (char *)IqApiBinFilePath_tmp, 1234);
                        //}
                        
                    }


                }
            }
        }
        usleep(THREAD_SLEEP_TIME_US*10);
    }
#endif
    return  NULL;
}

/*
MI_S32 ST_CreatDivpChn(MI_DIVP_CHN DivpChn, MI_U32 u32MaxWidth, MI_U32 u32MaxHeight)
{
    MI_DIVP_ChnAttr_t stDivpChnAttr;
    MI_DIVP_OutputPortAttr_t stDivpPortAttr;
    ST_DivpAttr_t *pstDivpChnAttr = &gstDivpAttr;
    memset(&stDivpChnAttr, 0x0, sizeof(MI_DIVP_ChnAttr_t));
    memset(&stDivpPortAttr, 0x0, sizeof(MI_DIVP_OutputPortAttr_t));

    stDivpChnAttr.u32MaxWidth = u32MaxWidth;
    stDivpChnAttr.u32MaxHeight = u32MaxHeight;
    stDivpChnAttr.stCropRect.u16X = pstDivpChnAttr->stDivpPort.stPortCrop.u16X;
    stDivpChnAttr.stCropRect.u16Y = pstDivpChnAttr->stDivpPort.stPortCrop.u16Y;
    stDivpChnAttr.stCropRect.u16Width = pstDivpChnAttr->stDivpPort.stPortCrop.u16Width;
    stDivpChnAttr.stCropRect.u16Height = pstDivpChnAttr->stDivpPort.stPortCrop.u16Height;
    stDivpChnAttr.bVerMirror = pstDivpChnAttr->stDivpPort.bFlip;
    stDivpChnAttr.bHorMirror = pstDivpChnAttr->stDivpPort.bMirror;
    STCHECKRESULT(MI_DIVP_CreateChn(DivpChn, &stDivpChnAttr));

    stDivpPortAttr.u32Width = pstDivpChnAttr->stDivpPort.stPortSize.u16Width;
    stDivpPortAttr.u32Height = pstDivpChnAttr->stDivpPort.stPortSize.u16Height;
    stDivpPortAttr.ePixelFormat = pstDivpChnAttr->stDivpPort.ePixelFormat;
    STCHECKRESULT(MI_DIVP_SetOutputPortAttr(DivpChn, &stDivpPortAttr));

    STCHECKRESULT(MI_DIVP_StartChn(DivpChn));

    return 0;
}

MI_S32 ST_DestroyDivpChn(MI_DIVP_CHN DivpChn)
{
    STCHECKRESULT(MI_DIVP_StopChn(DivpChn));
    STCHECKRESULT(MI_DIVP_DestroyChn(DivpChn));

    return 0;
}
*/

MI_S32  ST_GetRotAfterCropRect(MI_SYS_WindowSize_t stInputSrcSize, MI_SYS_WindowRect_t stOrigPortCrop, MI_SYS_Rotate_e eRot, MI_BOOL bMirror, MI_BOOL bFlip, MI_SYS_WindowRect_t *pstWinRect)
{
    if((eRot == E_MI_SYS_ROTATE_90 && bMirror == FALSE && bFlip == FALSE)//rot 90
        ||(eRot == E_MI_SYS_ROTATE_270 && bMirror == TRUE && bFlip == TRUE)
        )
    {
        pstWinRect->u16X = stInputSrcSize.u16Height-stOrigPortCrop.u16Y-stOrigPortCrop.u16Height;
        pstWinRect->u16Y = stOrigPortCrop.u16X;
        pstWinRect->u16Width = stOrigPortCrop.u16Height;
        pstWinRect->u16Height = stOrigPortCrop.u16Width;
    }
    else if((eRot == E_MI_SYS_ROTATE_180 && bMirror == FALSE && bFlip == FALSE)//rot 180
        ||(eRot == E_MI_SYS_ROTATE_NONE && bMirror == TRUE && bFlip == TRUE)
        )
    {
        pstWinRect->u16X = stInputSrcSize.u16Width-stOrigPortCrop.u16X-stOrigPortCrop.u16Width;
        pstWinRect->u16Y = stInputSrcSize.u16Height-stOrigPortCrop.u16Y-stOrigPortCrop.u16Height;
        pstWinRect->u16Width = stOrigPortCrop.u16Width;
        pstWinRect->u16Height = stOrigPortCrop.u16Height;
    }
    else if((eRot == E_MI_SYS_ROTATE_270 && bMirror == FALSE && bFlip == FALSE)//rot 270
        ||(eRot == E_MI_SYS_ROTATE_90 && bMirror == TRUE && bFlip == TRUE)
        )
    {
        pstWinRect->u16X = stOrigPortCrop.u16Y;
        pstWinRect->u16Y = stInputSrcSize.u16Width-stOrigPortCrop.u16X-stOrigPortCrop.u16Width;
        pstWinRect->u16Width = stOrigPortCrop.u16Height;
        pstWinRect->u16Height = stOrigPortCrop.u16Width;
    }
    else if((eRot == E_MI_SYS_ROTATE_NONE && bMirror == TRUE && bFlip == FALSE)//mirror
        ||(eRot == E_MI_SYS_ROTATE_180 && bMirror == FALSE && bFlip == TRUE)
        )
    {
        pstWinRect->u16X =  stInputSrcSize.u16Width-stOrigPortCrop.u16X-stOrigPortCrop.u16Width;
        pstWinRect->u16Y = stOrigPortCrop.u16Y;
        pstWinRect->u16Width = stOrigPortCrop.u16Width;
        pstWinRect->u16Height = stOrigPortCrop.u16Height;
    }
    else if((eRot == E_MI_SYS_ROTATE_NONE && bMirror == FALSE && bFlip == TRUE)//flip
        ||(eRot == E_MI_SYS_ROTATE_180 && bMirror == TRUE && bFlip == FALSE)
        )
    {
        pstWinRect->u16X = stOrigPortCrop.u16X;
        pstWinRect->u16Y = stInputSrcSize.u16Height-stOrigPortCrop.u16Y-stOrigPortCrop.u16Height;
        pstWinRect->u16Width = stOrigPortCrop.u16Width;
        pstWinRect->u16Height = stOrigPortCrop.u16Height;
    }
    else if((eRot == E_MI_SYS_ROTATE_90 && bMirror == TRUE && bFlip == FALSE)//90+mirror
        ||(eRot == E_MI_SYS_ROTATE_270 && bMirror == FALSE && bFlip == TRUE)
        )
    {
        pstWinRect->u16X = stOrigPortCrop.u16X;;
        pstWinRect->u16Y = stOrigPortCrop.u16Y;
        pstWinRect->u16Width = stOrigPortCrop.u16Height;
        pstWinRect->u16Height = stOrigPortCrop.u16Width;
    }
    else if((eRot == E_MI_SYS_ROTATE_90 && bMirror == FALSE && bFlip == TRUE)//270+mirror
        ||(eRot == E_MI_SYS_ROTATE_270 && bMirror == TRUE && bFlip == FALSE)
        )
    {
        pstWinRect->u16X = stInputSrcSize.u16Height-stOrigPortCrop.u16Y-stOrigPortCrop.u16Height;
        pstWinRect->u16Y = stInputSrcSize.u16Width-stOrigPortCrop.u16X-stOrigPortCrop.u16Width;
        pstWinRect->u16Width = stOrigPortCrop.u16Height;
        pstWinRect->u16Height = stOrigPortCrop.u16Width;
    }
    else
    {
        pstWinRect->u16X = stOrigPortCrop.u16X;
        pstWinRect->u16Y = stOrigPortCrop.u16Y;
        pstWinRect->u16Width = stOrigPortCrop.u16Width;
        pstWinRect->u16Height = stOrigPortCrop.u16Height;
    }

    return 0;
}

MI_BOOL ST_GetIspOutputPortRect(MI_SYS_ChnPort_t *pstIspChnPort,MI_SYS_WindowRect_t *pIspOutPutRect)
{
    MI_BOOL bRet = MI_SUCCESS;
#if MI_ISP_SUPPORT

    MI_ISP_DEV IspDevId = pstIspChnPort->u32DevId;
    MI_ISP_CHANNEL IspChnId = pstIspChnPort->u32ChnId;
    MI_ISP_PORT IspOutPutPortId = pstIspChnPort->u32PortId;
    ST_IspDevAttr_t *pstIspDevAttr= &gstIspModule.stIspDevAttr[IspDevId];
    ST_IspChannelAttr_t  *pstIspChnAttr = &pstIspDevAttr->stIspChnlAttr[IspChnId];
    ST_PortAttr_t *pstIspOutputAttr = &pstIspChnAttr->stIspOutPortAttr[IspOutPutPortId];

    MI_SYS_WindowSize_t stInputSrcSize;
    MI_SYS_WindowRect_t stInputCropWin;
    memset(&stInputSrcSize, 0x0, sizeof(MI_SYS_WindowSize_t));
    memset(&stInputCropWin, 0x0, sizeof(MI_SYS_WindowRect_t));

    ExecFuncResult(MI_ISP_GetInputPortCrop(IspDevId, IspChnId, &stInputCropWin), bRet);
    if(stInputCropWin.u16Height == 0 || stInputCropWin.u16Width == 0)
    {
        if(pstIspChnAttr->stIspInPortAttr[0].stBindParam.stChnPort.eModId == E_MI_MODULE_ID_VIF)
        {
#if MI_VIF_SUPPORT
            MI_SYS_ChnPort_t *pstChnPort=&pstIspChnAttr->stIspInPortAttr[0].stBindParam.stChnPort;
            MI_VIF_OutputPortAttr_t  stOutputAttr;
            memset(&stOutputAttr, 0x0, sizeof(MI_VIF_OutputPortAttr_t));

            ExecFuncResult(MI_VIF_GetOutputPortAttr(pstChnPort->u32DevId, pstChnPort->u32PortId, &stOutputAttr), bRet);
            stInputSrcSize.u16Width = stOutputAttr.stDestSize.u16Width;
            stInputSrcSize.u16Height = stOutputAttr.stDestSize.u16Height;
#endif
        }
        else
        {
            stInputSrcSize.u16Width = pstIspChnAttr->stIspInPortAttr[0].stInputFileAttr.u32Width;
            stInputSrcSize.u16Height = pstIspChnAttr->stIspInPortAttr[0].stInputFileAttr.u32Height;
        }
    }
    else
    {
        stInputSrcSize.u16Width = stInputCropWin.u16Width;
        stInputSrcSize.u16Height = stInputCropWin.u16Height;
    }

    if(pstIspOutputAttr->bUsed == TRUE)
    {
        MI_SYS_WindowRect_t  stIspOutputCrop;
        MI_ISP_ChnParam_t stChnParam;
        memset(&stIspOutputCrop, 0x0, sizeof(MI_SYS_WindowRect_t));
        memset(&stChnParam, 0x0, sizeof(MI_ISP_ChnParam_t));

        ExecFuncResult(MI_ISP_GetChnParam(IspDevId,IspChnId,&stChnParam), bRet);

        if(pstIspOutputAttr->stOrigPortCrop.u16Height == 0
           || pstIspOutputAttr->stOrigPortCrop.u16Width == 0)
        {
            stIspOutputCrop.u16X = 0;
            stIspOutputCrop.u16Y = 0;
            stIspOutputCrop.u16Height = stInputSrcSize.u16Height;
            stIspOutputCrop.u16Width = stInputSrcSize.u16Width;
        }
        else
        {
            memcpy(&stIspOutputCrop, &pstIspOutputAttr->stOrigPortCrop, sizeof(MI_SYS_WindowRect_t));
        }
        ST_GetRotAfterCropRect(stInputSrcSize, stIspOutputCrop,
            stChnParam.eRot, stChnParam.bMirror, stChnParam.bFlip, pIspOutPutRect);
    }

EXIT:
#endif
    return bRet;
}

MI_S32 ST_JpdModuleInit(MI_U32 u32JpdChn)
{
    MI_S32 s32Ret = MI_SUCCESS;
#if MI_JPD_SUPPORT
    MI_U32 u32JpdDevId=0;
    MI_JPD_ChnCreatConf_t stChnCreatConf;
    ST_JpdChnAttr_t *pstJpdChnAttr = &gstJpdModeAttr.stJpdChnAttr[u32JpdChn];
    memset(&stChnCreatConf, 0x0, sizeof(MI_JPD_ChnCreatConf_t));

    stChnCreatConf.u32MaxPicWidth   = pstJpdChnAttr->u32MaxW;
    stChnCreatConf.u32MaxPicHeight  = pstJpdChnAttr->u32MaxH;
    stChnCreatConf.u32StreamBufSize = pstJpdChnAttr->u32FrameBuffSize;

    ExecFuncResult(MI_JPD_CreateChn(u32JpdDevId, u32JpdChn, &stChnCreatConf), s32Ret);
    ExecFuncResult(MI_JPD_StartChn(u32JpdDevId,u32JpdChn), s32Ret);

    MI_SYS_ChnPort_t stChnPort;
    memset(&stChnPort, 0x0, sizeof(MI_SYS_ChnPort_t));

    stChnPort.eModId = E_MI_MODULE_ID_JPD;
    stChnPort.u32DevId = 0;
    stChnPort.u32ChnId = u32JpdChn;
    stChnPort.u32PortId = 0;

    //MI_SYS_SetChnOutputPortDepth(0, &stChnPort, 1, 4);

    printf("jpd channel %d create put data thread\n",u32JpdChn);
    pthread_create(&pstJpdChnAttr->pPutDatathread, NULL, ST_PutJpdInputDataThread,(void*)pstJpdChnAttr);
    usleep(THREAD_SLEEP_TIME_US);

EXIT:
#endif
    return s32Ret;
}

MI_S32 ST_VencModuleInit(MI_U32 u32VencChn)
{
    MI_S32 s32Ret = MI_SUCCESS;
#if MI_VENC_SUPPORT

    ST_VencAttr_t *pstStreamAttr = &gstVencattr[u32VencChn];
    MI_U32 u32BindDevId =pstStreamAttr->stVencInBindParam.stChnPort.u32DevId;
    MI_U32 u32BindChn=pstStreamAttr->stVencInBindParam.stChnPort.u32ChnId;
    MI_U32 u32BindPort=pstStreamAttr->stVencInBindParam.stChnPort.u32PortId;
    MI_ModuleId_e eBindModule = pstStreamAttr->stVencInBindParam.stChnPort.eModId;

    MI_U32 u32VenBitRate = 0;
    MI_U32 u32MaxWidth =0, u32MaxHeight =0;
    MI_SYS_Rotate_e eRotateType = E_MI_SYS_ROTATE_NUM;
    MI_VENC_DEV DevId = pstStreamAttr->DevId;
    MI_VENC_CHN ChnId = pstStreamAttr->vencChn;
    if(E_MI_MODULE_ID_SCL == eBindModule)
    {
#if MI_SCL_SUPPORT
        ST_SclChannelAttr_t *pstSclChnAttr = &gstSclModule.stSclDevAttr[u32BindDevId].stSclChnlAttr[u32BindChn];
        MI_SCL_OutPortParam_t stSclOutputParam;
        MI_SCL_ChnParam_t  stSclChnParam;
        MI_ISP_ChnParam_t stIspChnParam;
        memset(&stSclOutputParam, 0x0, sizeof(MI_SCL_OutPortParam_t));
        memset(&stSclChnParam, 0x0, sizeof(MI_SCL_ChnParam_t));
        memset(&stIspChnParam, 0x0, sizeof(MI_ISP_ChnParam_t));

        ExecFuncResult(MI_SCL_GetOutputPortParam((MI_SCL_DEV)u32BindDevId, u32BindChn,u32BindPort,&stSclOutputParam), s32Ret);
        ExecFuncResult(MI_SCL_GetChnParam((MI_SCL_DEV)u32BindDevId,u32BindChn,&stSclChnParam), s32Ret);
#if MI_ISP_SUPPORT
        if(pstSclChnAttr->stSclInPortAttr[0].stBindParam.stChnPort.eModId == E_MI_MODULE_ID_ISP)
        {
            ExecFuncResult(MI_ISP_GetChnParam(pstSclChnAttr->stSclInPortAttr[0].stBindParam.stChnPort.u32DevId,
                pstSclChnAttr->stSclInPortAttr[0].stBindParam.stChnPort.u32ChnId,&stIspChnParam), s32Ret);
        }
#endif
        pstStreamAttr->u32Width = ALIGN_UP(stSclOutputParam.stSCLOutputSize.u16Width,8);
        pstStreamAttr->u32Height = stSclOutputParam.stSCLOutputSize.u16Height;

        if((stSclChnParam.eRot == E_MI_SYS_ROTATE_90 || stSclChnParam.eRot == E_MI_SYS_ROTATE_270)
            ^ (stIspChnParam.eRot == E_MI_SYS_ROTATE_90 || stIspChnParam.eRot == E_MI_SYS_ROTATE_270))
        {
            eRotateType = E_MI_SYS_ROTATE_90;
        }
        else
            eRotateType = E_MI_SYS_ROTATE_NONE;

        if((pstStreamAttr->stVencInBindParam.eBindType == E_MI_SYS_BIND_TYPE_HW_RING
            || pstStreamAttr->stVencInBindParam.eBindType == E_MI_SYS_BIND_TYPE_REALTIME)
            )
        {
            pstStreamAttr->s32DumpBuffNum = pstSclChnAttr->stSclOutPortAttr[u32BindPort].stoutFileAttr.s32DumpBuffNum;
            memcpy(pstStreamAttr->FilePath, pstSclChnAttr->stSclOutPortAttr[u32BindPort].stoutFileAttr.FilePath, sizeof(pstSclChnAttr->stSclOutPortAttr[u32BindPort].stoutFileAttr.FilePath));
            //pstStreamAttr->bNeedCheckMd5 = pstSclChnAttr->stSclOutPortAttr[u32BindPort].stoutFileAttr.bNeedCheckMd5;
            //memcpy(pstStreamAttr->u8MD5ExpectValue, pstSclChnAttr->stSclOutPortAttr[u32BindPort].stoutFileAttr.stMD5ExpectValue[0].u8MD5ExpectValue, sizeof(MI_U8)*16);
            memcpy(&pstStreamAttr->stMd5Attr, &pstSclChnAttr->stSclOutPortAttr[u32BindPort].stoutFileAttr.stMd5Attr,
             sizeof(ST_Md5Attr_t));
            pstSclChnAttr->stSclOutPortAttr[u32BindPort].stoutFileAttr.s32DumpBuffNum =0;
        }
#endif
    }
    else if(E_MI_MODULE_ID_ISP == eBindModule)
    {
#if MI_ISP_SUPPORT
        MI_SYS_WindowRect_t stCropRect;
        MI_ISP_ChnParam_t  stIspChnParam;
        memset(&stCropRect, 0x0, sizeof(MI_SYS_WindowRect_t));
        memset(&stIspChnParam, 0x0, sizeof(MI_ISP_ChnParam_t));

        ST_GetIspOutputPortRect(&pstStreamAttr->stVencInBindParam.stChnPort,&stCropRect);
        ExecFuncResult(MI_ISP_GetChnParam(u32BindDevId,u32BindChn,&stIspChnParam), s32Ret);

        pstStreamAttr->u32Width = stCropRect.u16Width;
        pstStreamAttr->u32Height = stCropRect.u16Height;
        eRotateType = stIspChnParam.eRot;
#endif
    }

    if(pstStreamAttr->eType == E_MI_VENC_MODTYPE_JPEGE)
    {
        if(eRotateType == E_MI_SYS_ROTATE_90  || eRotateType == E_MI_SYS_ROTATE_270)
        {
            u32MaxWidth = u16JpegMaxH;
            u32MaxHeight = u16JpegMaxW;
        }
        else
        {
            u32MaxWidth = u16JpegMaxW;
            u32MaxHeight = u16JpegMaxH;
        }
    }
    else
    {
        if(eRotateType == E_MI_SYS_ROTATE_90  || eRotateType == E_MI_SYS_ROTATE_270)
        {
            u32MaxWidth = u16VencMaxH;
            u32MaxHeight = u16VencMaxW;
        }
        else
        {
            u32MaxWidth = u16VencMaxW;
            u32MaxHeight = u16VencMaxH;
        }
    }

    u32VenBitRate = ((pstStreamAttr->u32Width * pstStreamAttr->u32Height + 500000)/1000000)*1024*1024;
    if(u32VenBitRate == 0)
    {
        u32VenBitRate = 2*1024*1024;
    }

    DBG_INFO("bindParam(%d,%d,%d,%d), Dev %d chn %d, pichwidth %d, height %d, :q %d, MaxHeight %d bitrate %d, fps %d \n",
        eBindModule, u32BindDevId,u32BindChn, u32BindPort,DevId,
        ChnId,pstStreamAttr->u32Width, pstStreamAttr->u32Height, u32MaxWidth, u32MaxHeight, u32VenBitRate, pstStreamAttr->u32Fps);


	MI_VENC_InitParam_t stInitParam;
	memset(&stInitParam, 0, sizeof(MI_VENC_InitParam_t));
	stInitParam.u32MaxWidth = u32MaxWidth;
	stInitParam.u32MaxHeight = u32MaxHeight;
	ExecFuncResult(MI_VENC_CreateDev(DevId, &stInitParam),s32Ret);

    //step3
    MI_VENC_ChnAttr_t stChnAttr;
    memset(&stChnAttr, 0, sizeof(MI_VENC_ChnAttr_t));
    if(pstStreamAttr->eType == E_MI_VENC_MODTYPE_H264E)
    {
        stChnAttr.stVeAttr.stAttrH264e.u32PicWidth = pstStreamAttr->u32Width;
        stChnAttr.stVeAttr.stAttrH264e.u32PicHeight = pstStreamAttr->u32Height;
        stChnAttr.stVeAttr.stAttrH264e.u32MaxPicWidth = u32MaxWidth;
        stChnAttr.stVeAttr.stAttrH264e.u32BFrameNum = 2;
        stChnAttr.stVeAttr.stAttrH264e.bByFrame = TRUE;
        stChnAttr.stVeAttr.stAttrH264e.u32MaxPicHeight = u32MaxHeight;
		stChnAttr.stVeAttr.stAttrH264e.u32BufSize = u32MaxWidth * u32MaxHeight * 1.5;
        /*stChnAttr.stRcAttr.eRcMode = E_MI_VENC_RC_MODE_H264CBR;
        stChnAttr.stRcAttr.stAttrH264Cbr.u32BitRate = u32VenBitRate;
        stChnAttr.stRcAttr.stAttrH264Cbr.u32FluctuateLevel = 0;
        stChnAttr.stRcAttr.stAttrH264Cbr.u32Gop = 30;
        stChnAttr.stRcAttr.stAttrH264Cbr.u32SrcFrmRateNum = pstStreamAttr->u32Fps;
        stChnAttr.stRcAttr.stAttrH264Cbr.u32SrcFrmRateDen = 1;
        stChnAttr.stRcAttr.stAttrH264Cbr.u32StatTime = 0;*/
        stChnAttr.stRcAttr.eRcMode = E_MI_VENC_RC_MODE_H264FIXQP;
        stChnAttr.stRcAttr.stAttrH264FixQp.u32Gop = 1;
        stChnAttr.stRcAttr.stAttrH264FixQp.u32IQp = 30;
        stChnAttr.stRcAttr.stAttrH264FixQp.u32PQp = 30;
        stChnAttr.stRcAttr.stAttrH264FixQp.u32SrcFrmRateNum = pstStreamAttr->u32Fps;
        stChnAttr.stRcAttr.stAttrH264FixQp.u32SrcFrmRateDen = 1;
    }
    else if(pstStreamAttr->eType == E_MI_VENC_MODTYPE_H265E)
    {
        stChnAttr.stVeAttr.stAttrH265e.u32PicWidth = pstStreamAttr->u32Width;
        stChnAttr.stVeAttr.stAttrH265e.u32PicHeight = pstStreamAttr->u32Height;
        stChnAttr.stVeAttr.stAttrH265e.u32MaxPicWidth = u32MaxWidth;
        stChnAttr.stVeAttr.stAttrH265e.u32MaxPicHeight = u32MaxHeight;
        stChnAttr.stVeAttr.stAttrH265e.bByFrame = TRUE;

        /*stChnAttr.stRcAttr.eRcMode = E_MI_VENC_RC_MODE_H265CBR;
        stChnAttr.stRcAttr.stAttrH265Cbr.u32BitRate = u32VenBitRate;
        stChnAttr.stRcAttr.stAttrH265Cbr.u32SrcFrmRateNum = pstStreamAttr->u32Fps;
        stChnAttr.stRcAttr.stAttrH265Cbr.u32SrcFrmRateDen = 1;
        stChnAttr.stRcAttr.stAttrH265Cbr.u32Gop = 30;
        stChnAttr.stRcAttr.stAttrH265Cbr.u32FluctuateLevel = 0;
        stChnAttr.stRcAttr.stAttrH265Cbr.u32StatTime = 0;*/

        stChnAttr.stRcAttr.eRcMode = E_MI_VENC_RC_MODE_H265FIXQP;
        stChnAttr.stRcAttr.stAttrH265FixQp.u32Gop = 1;
        stChnAttr.stRcAttr.stAttrH265FixQp.u32IQp = bIQp;
        stChnAttr.stRcAttr.stAttrH265FixQp.u32PQp = bPQp;
        stChnAttr.stRcAttr.stAttrH265FixQp.u32SrcFrmRateNum = pstStreamAttr->u32Fps;
        stChnAttr.stRcAttr.stAttrH265FixQp.u32SrcFrmRateDen = 1;
    }
    else if(pstStreamAttr->eType == E_MI_VENC_MODTYPE_JPEGE)
    {
        stChnAttr.stVeAttr.eType = E_MI_VENC_MODTYPE_JPEGE;
        stChnAttr.stVeAttr.stAttrJpeg.u32PicWidth = pstStreamAttr->u32Width;
        stChnAttr.stVeAttr.stAttrJpeg.u32PicHeight = pstStreamAttr->u32Height;
        stChnAttr.stVeAttr.stAttrJpeg.u32MaxPicWidth = u32MaxWidth;
        stChnAttr.stVeAttr.stAttrJpeg.u32MaxPicHeight = u32MaxHeight;

        stChnAttr.stRcAttr.eRcMode = E_MI_VENC_RC_MODE_MJPEGFIXQP;
        stChnAttr.stRcAttr.stAttrMjpegFixQp.u32Qfactor = 50;
        stChnAttr.stRcAttr.stAttrMjpegFixQp.u32SrcFrmRateNum = pstStreamAttr->u32Fps;
        stChnAttr.stRcAttr.stAttrMjpegFixQp.u32SrcFrmRateDen = 1;
        DevId = (pstStreamAttr->DevId == 0) ? 8 : pstStreamAttr->DevId;
    }
    stChnAttr.stVeAttr.eType = pstStreamAttr->eType;
    ExecFuncResult(ST_Venc_CreateChannel(DevId, ChnId, &stChnAttr), s32Ret);

    MI_VENC_InputSourceConfig_t stVencSourceCfg;
    memset(&stVencSourceCfg, 0, sizeof(MI_VENC_InputSourceConfig_t));
    if(pstStreamAttr->stVencInBindParam.eBindType == E_MI_SYS_BIND_TYPE_HW_RING)
    {
        if(pstStreamAttr->stVencInBindParam.u32BindParam == 0 || pstStreamAttr->stVencInBindParam.u32BindParam==pstStreamAttr->u32Height)
        {
            pstStreamAttr->stVencInBindParam.u32BindParam = pstStreamAttr->u32Height;
            stVencSourceCfg.eInputSrcBufferMode = E_MI_VENC_INPUT_MODE_RING_ONE_FRM;
        }
        else if(pstStreamAttr->stVencInBindParam.u32BindParam==pstStreamAttr->u32Height/2)
        {
            stVencSourceCfg.eInputSrcBufferMode = E_MI_VENC_INPUT_MODE_RING_HALF_FRM;
        }
        else
        {
            UTStatus = UT_CASE_FAIL;
            DBG_ERR("venc dev%d chn%d bindtype %d bindparam %d stream height %d err \n", DevId, ChnId, pstStreamAttr->stVencInBindParam.eBindType, pstStreamAttr->stVencInBindParam.u32BindParam,
                pstStreamAttr->u32Height);
        }
    }
    else
    {
        stVencSourceCfg.eInputSrcBufferMode = E_MI_VENC_INPUT_MODE_NORMAL_FRMBASE;
    }
    ExecFuncResult(MI_VENC_SetInputSourceConfig(DevId, ChnId, &stVencSourceCfg), s32Ret);

    ExecFuncResult(ST_Venc_StartChannel(DevId, ChnId), s32Ret);
    pstStreamAttr->bCreate = TRUE;

    ST_Sys_BindInfo_T stBindInfo;
    memset(&stBindInfo, 0x0, sizeof(ST_Sys_BindInfo_T));

    //step4
    /*
    MI_U32 u32DevId = -1;
    ExecFunc(MI_VENC_GetChnDevid(u32VencChn, &u32DevId), MI_SUCCESS);*/
    stBindInfo.stSrcChnPort.eModId = eBindModule;
    stBindInfo.stSrcChnPort.u32DevId = u32BindDevId;
    stBindInfo.stSrcChnPort.u32ChnId = u32BindChn;
    stBindInfo.stSrcChnPort.u32PortId = u32BindPort;

    stBindInfo.stDstChnPort.eModId = E_MI_MODULE_ID_VENC;
    stBindInfo.stDstChnPort.u32DevId = DevId;
    stBindInfo.stDstChnPort.u32ChnId = ChnId;
    stBindInfo.stDstChnPort.u32PortId = 0;

    stBindInfo.u32SrcFrmrate = 30;
    stBindInfo.u32DstFrmrate = 30;
    stBindInfo.eBindType = pstStreamAttr->stVencInBindParam.eBindType;
    stBindInfo.u32BindParam = pstStreamAttr->stVencInBindParam.u32BindParam;
    ExecFuncResult(ST_Sys_Bind_List(&stBindInfo), s32Ret);

EXIT:
#endif
    return s32Ret;
}

MI_S32 ST_VencModuleUnInit(MI_U32 u32VencChn)
{
    MI_S32 s32Ret = MI_SUCCESS;
#if MI_VENC_SUPPORT

    ST_VencAttr_t *pstStreamAttr = &gstVencattr[u32VencChn];
    MI_U32 u32VencChnId = pstStreamAttr->vencChn;
    MI_U32 u32VencDevId = pstStreamAttr->DevId;
    MI_U32 u32BindChn= pstStreamAttr->stVencInBindParam.stChnPort.u32ChnId;
    MI_U32 u32BindPort= pstStreamAttr->stVencInBindParam.stChnPort.u32PortId;
    MI_ModuleId_e eBindModule = pstStreamAttr->stVencInBindParam.stChnPort.eModId;
    MI_U32 u32BindDevId =pstStreamAttr->stVencInBindParam.stChnPort.u32DevId;

    ST_Sys_BindInfo_T stBindInfo;
    memset(&stBindInfo, 0x0, sizeof(ST_Sys_BindInfo_T));
    /*MI_U32 u32DevId = -1;
    ExecFunc(MI_VENC_GetChnDevid(u32vencChn, &u32DevId), MI_SUCCESS);*/

    stBindInfo.stSrcChnPort.eModId = eBindModule;
    stBindInfo.stSrcChnPort.u32DevId = u32BindDevId;
    stBindInfo.stSrcChnPort.u32ChnId = u32BindChn;
    stBindInfo.stSrcChnPort.u32PortId = u32BindPort;

    stBindInfo.stDstChnPort.eModId = E_MI_MODULE_ID_VENC;
    stBindInfo.stDstChnPort.u32DevId = u32VencDevId;
    stBindInfo.stDstChnPort.u32ChnId = u32VencChnId;
    stBindInfo.stDstChnPort.u32PortId = 0;

    stBindInfo.u32SrcFrmrate = 30;
    stBindInfo.u32DstFrmrate = 30;
    ExecFuncResult(ST_Sys_UnBind_List(&stBindInfo), s32Ret);
    ExecFuncResult(ST_Venc_StopChannel(u32VencDevId, u32VencChnId), s32Ret);
    ExecFuncResult(ST_Venc_DestoryChannel(u32VencDevId,u32VencChnId), s32Ret);
	ExecFuncResult(MI_VENC_DestroyDev(u32VencDevId),s32Ret);

    pstStreamAttr->bCreate = FALSE;

EXIT:
#endif
    return s32Ret;
}

MI_S32 ST_SensorModuleInit(MI_SNR_PADID eSnrPad)
{
    MI_S32 s32Ret = MI_SUCCESS;
#if MI_VIF_SUPPORT
    MI_SNR_PADID eSnrPadId = eSnrPad;
    MI_SNR_PADInfo_t  stPad0Info;
    MI_SNR_PlaneInfo_t stSnrPlane0Info;
    MI_U32 u32ResCount =0;
    MI_U8 u8ResIndex =0;
    MI_U8 u8ChocieRes =gstSensorAttr[eSnrPad].u8ResIndex;
    MI_S32 s32Input =0;
    MI_SNR_Res_t stRes;
    ST_Sensor_Attr_t *pstSensorAttr = NULL;
    memset(&stRes, 0x0, sizeof(MI_SNR_Res_t));
    memset(&stPad0Info, 0x0, sizeof(MI_SNR_PADInfo_t));
    memset(&stSnrPlane0Info, 0x0, sizeof(MI_SNR_PlaneInfo_t));

    pstSensorAttr = &gstSensorAttr[eSnrPadId];

    /************************************************
    Step2:  init Sensor
    *************************************************/
    if(pstSensorAttr->bPlaneMode ==TRUE)
    {
        ExecFuncResult(MI_SNR_SetPlaneMode(eSnrPad, FALSE), s32Ret);
    }
    else
    {
        ExecFuncResult(MI_SNR_SetPlaneMode(eSnrPad, TRUE), s32Ret);
    }

    ExecFuncResult(MI_SNR_QueryResCount(eSnrPadId, &u32ResCount), s32Ret);
    for(u8ResIndex=0; u8ResIndex < u32ResCount; u8ResIndex++)
    {
        ExecFuncResult(MI_SNR_GetRes(eSnrPadId, u8ResIndex, &stRes), s32Ret);
        printf("index %d, Crop(%d,%d,%d,%d), outputsize(%d,%d), maxfps %d, minfps %d, ResDesc %s\n",
        u8ResIndex,
        stRes.stCropRect.u16X, stRes.stCropRect.u16Y, stRes.stCropRect.u16Width,stRes.stCropRect.u16Height,
        stRes.stOutputSize.u16Width, stRes.stOutputSize.u16Height,
        stRes.u32MaxFps,stRes.u32MinFps,
        stRes.strResDesc);
    }
    if(u8ChocieRes >= u32ResCount && u8ChocieRes != 0xff)
    {
        UTStatus = UT_CASE_FAIL;
        printf("res set err  %d > =cnt %d\n", u8ChocieRes, u32ResCount);
        s32Ret = -1;
        goto EXIT;
    }
    else if(u8ChocieRes == 0xff)
    {
        printf("choice which resolution use, cnt %d\n", u32ResCount);
        do
        {
            scanf("%d", &s32Input);
            u8ChocieRes = (MI_U8)s32Input;
            ST_Flush();
            ExecFuncResult(MI_SNR_QueryResCount(eSnrPadId, &u32ResCount), s32Ret);
            if(u8ChocieRes >= u32ResCount)
            {
                UTStatus = UT_CASE_FAIL;
                printf("choice err res %d > =cnt %d\n", u8ChocieRes, u32ResCount);
            }
        }while(u8ChocieRes >= u32ResCount);
        printf("You select %d res\n", u8ChocieRes);
    }
    printf("Rest %d\n", u8ChocieRes);

    ExecFuncResult(MI_SNR_SetRes(eSnrPadId,u8ChocieRes), s32Ret);

    ExecFuncResult(MI_SNR_Enable(eSnrPadId), s32Ret);

EXIT:
#endif
    return s32Ret;
}

//chipindex 0,1,2,3
//BT656id   0,1,2,3
//slaveaddr 0x60,0x62,0x64.0x66
//sensorpad 0,2,1,3
//vifgroup  0,1,2,3
MI_S32 ST_TransSnrPadIdToDH9931Id(MI_SNR_PADID eSnrPad, MI_U8 *pu8DH9931Id)
{
#if USER_SENSOR_SUPPORT
    switch(eSnrPad)
    {
#if ((defined CONFIG_SIGMASTAR_CHIP_M6) && (CONFIG_SIGMASTAR_CHIP_M6 == 1))
        case 0:
            *pu8DH9931Id=0;
            break;
        case 2:
            *pu8DH9931Id=1;
            break;
#elif ((defined CONFIG_SIGMASTAR_CHIP_I7) && (CONFIG_SIGMASTAR_CHIP_I7 == 1))
        case 0:
        case 1:
        case 2:
        case 3:
            *pu8DH9931Id=0;
            break;
        case 4:
        case 5:
        case 6:
        case 7:
            *pu8DH9931Id=1;
            break;
#elif ((defined CONFIG_SIGMASTAR_CHIP_M6P) && (CONFIG_SIGMASTAR_CHIP_M6P == 1))
        case 0:
        case 1:
        case 2:
        case 3:
            *pu8DH9931Id=0;
            break;
#endif
        default:
            DBG_ERR("sensor padid %d err \n",eSnrPad);
            UTStatus = UT_CASE_FAIL;
            return -1;
    }
#endif
    return 0;
}

MI_S32 ST_UserSensorModuleInit(MI_SNR_PADID eSnrPad)
{
    MI_S32 s32Ret = MI_SUCCESS;
#if USER_SENSOR_SUPPORT
    MI_U8 u8ChipIndex=0;
    MI_SNR_PADInfo_t  stPad0Info;
    MI_SNR_PlaneInfo_t stSnrPlane0Info;
    MI_U32 u32ResCount =0;
    MI_U8 u8ResIndex =0;
    MI_S32 s32Input =0;
    cus_camsensor_res stRes;
    ST_Sensor_Attr_t *pstSensorAttr = &gstSensorAttr[eSnrPad];
    MI_U8 u8ChocieRes =pstSensorAttr->u8ResIndex;

    memset(&stRes, 0x0, sizeof(cus_camsensor_res));
    memset(&stPad0Info, 0x0, sizeof(MI_SNR_PADInfo_t));
    memset(&stSnrPlane0Info, 0x0, sizeof(MI_SNR_PlaneInfo_t));

    if(gstSensorAttr[eSnrPad].ADIndex == 0xFF)
    {
        ExecFuncResult(ST_TransSnrPadIdToDH9931Id(eSnrPad, &u8ChipIndex), s32Ret);
    }
    else
    {
        u8ChipIndex = gstSensorAttr[eSnrPad].ADIndex;
    }

    /************************************************
    Step2:  init Sensor
    *************************************************/
    DHC_DH9931_Init(u8ChipIndex);
    Cus_SetInterface(u8ChipIndex,pstSensorAttr->eUserSensorIntf);

    ExecFuncResult(Cus_GetVideoResNum(u8ChipIndex, &u32ResCount), s32Ret);
    for(u8ResIndex=0; u8ResIndex < u32ResCount; u8ResIndex++)
    {
        ExecFuncResult(Cus_GetVideoRes(u8ChipIndex, u8ResIndex, &stRes), s32Ret);
        printf("index %d, Crop(%d,%d,%d,%d), outputsize(%d,%d), maxfps %d, minfps %d, ResDesc %s\n",
        u8ResIndex,
        stRes.stCropRect.u16X, stRes.stCropRect.u16Y, stRes.stCropRect.u16Width,stRes.stCropRect.u16Height,
        stRes.stOutputSize.u16Width, stRes.stOutputSize.u16Height,
        stRes.u32MaxFps,stRes.u32MinFps,
        stRes.strResDesc);
    }
    if(u8ChocieRes >= u32ResCount && u8ChocieRes != 0xff)
    {
        UTStatus = UT_CASE_FAIL;
        printf("res set err  %d > =cnt %d\n", u8ChocieRes, u32ResCount);
        s32Ret = -1;
        goto EXIT;
    }
    else if(u8ChocieRes == 0xff)
    {
        printf("choice which resolution use, cnt %d\n", u32ResCount);
        do
        {
            scanf("%d", &s32Input);
            u8ChocieRes = (MI_U8)s32Input;
            ST_Flush();
            ExecFuncResult(Cus_GetVideoResNum(u8ChipIndex, &u32ResCount), s32Ret);
            if(u8ChocieRes >= u32ResCount)
            {
                UTStatus = UT_CASE_FAIL;
                printf("choice err res %d > =cnt %d\n", u8ChocieRes, u32ResCount);
            }
        }while(u8ChocieRes >= u32ResCount);
        printf("You select %d res\n", u8ChocieRes);
    }
    printf("Rest %d\n", u8ChocieRes);

    ExecFuncResult(Cus_SetVideoRes(u8ChipIndex,u8ChocieRes), s32Ret);
EXIT:
#endif
    return s32Ret;
}

MI_S32 ST_VifModuleInit(MI_VIF_GROUP GroupId)
{
    MI_S32 s32Ret = MI_SUCCESS;
#if MI_VIF_SUPPORT
    MI_VIF_DEV vifDev =0;
    MI_U8 vifDevIdPerGroup = 0;
    MI_VIF_PORT vifPort = 0;
    ST_VifModAttr_t *pstVifModAttr = &gstVifModule;
    ST_VifGroupAttr_t *pstVifGroupAttr = &gstVifModule.stVifGroupAttr[GroupId];
    MI_SNR_PADID eSnrPadId = (MI_SNR_PADID)pstVifGroupAttr->stBindSensor.eSensorPadID;
    MI_U32 u32PlaneId = pstVifGroupAttr->stBindSensor.u32PlaneID[0];

    MI_SNR_PADInfo_t  stPad0Info;
    MI_SNR_PlaneInfo_t stSnrPlane0Info;
    memset(&stPad0Info, 0x0, sizeof(MI_SNR_PADInfo_t));
    memset(&stSnrPlane0Info, 0x0, sizeof(MI_SNR_PlaneInfo_t));

   // MI_U8 i=0;

    /************************************************
    Step3:  init VIF
    *************************************************/
    /*
    MI_VIF_Dev2SnrPadMuxCfg_t stVifDev[4];
    stVifDev[0].eSensorPadID = 0;
    stVifDev[0].u32PlaneID = 0xff;
    stVifDev[1].eSensorPadID = E_MI_VIF_SNRPAD_ID_2;
    stVifDev[1].u32PlaneID = 0xff;
    stVifDev[2].eSensorPadID = E_MI_VIF_SNRPAD_ID_1;
    stVifDev[2].u32PlaneID = 0xff;
    stVifDev[3].eSensorPadID = E_MI_VIF_SNRPAD_ID_3;
    stVifDev[3].u32PlaneID = 0xff;
    MI_VIF_SetDev2SnrPadMux(stVifDev, 4);*/

    MI_VIF_GroupAttr_t stGroupAttr;
    memset(&stGroupAttr, 0x0, sizeof(MI_VIF_GroupAttr_t));

    if(pstVifGroupAttr->bUsed == TRUE
       && pstVifGroupAttr->bCreate == FALSE)
    {
        if(bUseUserSensor == FALSE)
        {
            ExecFuncResult(MI_SNR_GetPadInfo(eSnrPadId, &stPad0Info), s32Ret);
            ExecFuncResult(MI_SNR_GetPlaneInfo(eSnrPadId, u32PlaneId, &stSnrPlane0Info), s32Ret);
        }
        else
        {
#if USER_SENSOR_SUPPORT
            MI_U8 u8ChipIndex=0;
            ExecFuncResult(ST_TransSnrPadIdToDH9931Id(eSnrPadId, &u8ChipIndex), s32Ret);
            ExecFuncResult(Cus_GetSnrPadInfo(u8ChipIndex, &stPad0Info), s32Ret);
            ExecFuncResult(Cus_GetSnrPlaneInfo(u8ChipIndex, u32PlaneId, &stSnrPlane0Info), s32Ret);
#endif
        }
        pstVifGroupAttr->bCreate = TRUE;

        if(pstVifGroupAttr->eHDRType > E_MI_VIF_HDR_TYPE_OFF)
        {
            pstVifGroupAttr->eHDRType = (MI_VIF_HDRType_e)stPad0Info.eHDRMode;
        }

        stGroupAttr.eIntfMode = (MI_VIF_IntfMode_e)stPad0Info.eIntfMode;
        stGroupAttr.eWorkMode = pstVifGroupAttr->eWorkMode;
        stGroupAttr.eHDRType = pstVifGroupAttr->eHDRType;
        if(stGroupAttr.eIntfMode == E_MI_VIF_MODE_BT656 || stGroupAttr.eIntfMode == E_MI_VIF_MODE_BT1120_STANDARD)
            stGroupAttr.eClkEdge = (MI_VIF_ClkEdge_e)stPad0Info.unIntfAttr.stBt656Attr.eClkEdge;
        else
            stGroupAttr.eClkEdge = E_MI_VIF_CLK_EDGE_DOUBLE;

        ExecFuncResult(MI_VIF_CreateDevGroup(GroupId, &stGroupAttr), s32Ret);
    }

    for(vifDevIdPerGroup=0; vifDevIdPerGroup< ST_MAX_VIF_DEV_PERGROUP; vifDevIdPerGroup++)
    {
        ST_VifDevAttr_t *pstVifDevAttr = &pstVifGroupAttr->stVifDevAttr[vifDevIdPerGroup];
        if(pstVifDevAttr->bUsed == TRUE
            && pstVifDevAttr->bCreate == FALSE)
        {
            MI_VIF_DevAttr_t stVifDevAttr;
            memset(&stVifDevAttr, 0x0, sizeof(MI_VIF_DevAttr_t));

            vifDev = GroupId*ST_MAX_VIF_DEV_PERGROUP+vifDevIdPerGroup;
            if(pstVifDevAttr->stVifDevAttr.stInputRect.u16Width == 0 || pstVifDevAttr->stVifDevAttr.stInputRect.u16Height == 0)
            {
                stVifDevAttr.stInputRect.u16X = stSnrPlane0Info.stCapRect.u16X;
                stVifDevAttr.stInputRect.u16Y = stSnrPlane0Info.stCapRect.u16Y;
                stVifDevAttr.stInputRect.u16Width = stSnrPlane0Info.stCapRect.u16Width;
                stVifDevAttr.stInputRect.u16Height = stSnrPlane0Info.stCapRect.u16Height;
            }
            else
            {
                stVifDevAttr.stInputRect.u16X = pstVifDevAttr->stVifDevAttr.stInputRect.u16X;
                stVifDevAttr.stInputRect.u16Y = pstVifDevAttr->stVifDevAttr.stInputRect.u16Y;
                stVifDevAttr.stInputRect.u16Width = pstVifDevAttr->stVifDevAttr.stInputRect.u16Width;
                stVifDevAttr.stInputRect.u16Height = pstVifDevAttr->stVifDevAttr.stInputRect.u16Height;
            }
            stVifDevAttr.bEnH2T1PMode = pstVifDevAttr->stVifDevAttr.bEnH2T1PMode;
            stVifDevAttr.eField =pstVifDevAttr->stVifDevAttr.eField;
            if(pstVifDevAttr->stVifDevAttr.eInputPixel >= E_MI_SYS_PIXEL_FRAME_FORMAT_MAX)
            {
                if(stSnrPlane0Info.eBayerId >= E_MI_SYS_PIXEL_BAYERID_MAX)
                {
                    pstVifDevAttr->stVifDevAttr.eInputPixel = stSnrPlane0Info.ePixel;
                }
                else
                    pstVifDevAttr->stVifDevAttr.eInputPixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(stSnrPlane0Info.ePixPrecision, stSnrPlane0Info.eBayerId);
            }
            stVifDevAttr.eInputPixel = pstVifDevAttr->stVifDevAttr.eInputPixel;

            printf("setchnportattr (%d,%d,%d,%d) \n", stVifDevAttr.stInputRect.u16X, stVifDevAttr.stInputRect.u16Y, stVifDevAttr.stInputRect.u16Width, stVifDevAttr.stInputRect.u16Height);
            ExecFuncResult(MI_VIF_SetDevAttr(vifDev, &stVifDevAttr), s32Ret);
            ExecFuncResult(MI_VIF_EnableDev(vifDev), s32Ret);

            //memcpy(&pstVifDevAttr->stVifDevAttr, &stVifDevAttr, sizeof(MI_VIF_DevAttr_t));
            //pstVifDevAttr->stVifDevAttr save user set param, if memcpy, second will not use sensor res

            pstVifDevAttr->bCreate = TRUE;

            pthread_mutex_init(&pstVifDevAttr->Devmutex, NULL);
        }

        for(vifPort=0; vifPort< ST_MAX_VIF_OUTPORT_NUM; vifPort++)
        {
            ST_VifPortAttr_t *pstVifPortAttr = &pstVifDevAttr->stVifOutPortAttr[vifPort];
            if(pstVifPortAttr->bUsed == TRUE
                && pstVifPortAttr->bCreate== FALSE)
            {
                MI_VIF_OutputPortAttr_t stVifPortInfo;
                memset(&stVifPortInfo, 0, sizeof(MI_VIF_OutputPortAttr_t));

                if(pstVifPortAttr->stCapRect.u16Width == 0
                    || pstVifPortAttr->stCapRect.u16Height == 0)
                {
                    pstVifPortAttr->stCapRect.u16X = stSnrPlane0Info.stCapRect.u16X;
                    pstVifPortAttr->stCapRect.u16Y = stSnrPlane0Info.stCapRect.u16Y;
                    pstVifPortAttr->stCapRect.u16Width = stSnrPlane0Info.stCapRect.u16Width;
                    pstVifPortAttr->stCapRect.u16Height = stSnrPlane0Info.stCapRect.u16Height;
                }

                if(pstVifPortAttr->stDestSize.u16Width == 0
                    || pstVifPortAttr->stDestSize.u16Height ==0)
                {
                    pstVifPortAttr->stDestSize.u16Width = stSnrPlane0Info.stCapRect.u16Width;
                    pstVifPortAttr->stDestSize.u16Height = stSnrPlane0Info.stCapRect.u16Height;
                }

                if(pstVifPortAttr->ePixFormat >= E_MI_SYS_PIXEL_FRAME_FORMAT_MAX)
                {
                    pstVifPortAttr->ePixFormat = pstVifDevAttr->stVifDevAttr.eInputPixel;
                }

                if(pstVifDevAttr->stVifDevAttr.bEnH2T1PMode ==TRUE)
                {
                    pstVifPortAttr->stCapRect.u16Width = pstVifPortAttr->stCapRect.u16Width/2;
                    pstVifPortAttr->stDestSize.u16Width = pstVifPortAttr->stDestSize.u16Width/2;
                }

                stVifPortInfo.stCapRect.u16X = pstVifPortAttr->stCapRect.u16X;
                stVifPortInfo.stCapRect.u16Y = pstVifPortAttr->stCapRect.u16Y;
                stVifPortInfo.stCapRect.u16Width =  pstVifPortAttr->stCapRect.u16Width;
                stVifPortInfo.stCapRect.u16Height = pstVifPortAttr->stCapRect.u16Height;
                stVifPortInfo.stDestSize.u16Width = pstVifPortAttr->stDestSize.u16Width;
                stVifPortInfo.stDestSize.u16Height = pstVifPortAttr->stDestSize.u16Height;
                printf("sensor bayerid %d, bit mode %d \n", stSnrPlane0Info.eBayerId, stSnrPlane0Info.ePixPrecision);
                stVifPortInfo.ePixFormat = pstVifPortAttr->ePixFormat;
                //stVifPortInfo.u32FrameModeLineCount for lowlantancy mode
                stVifPortInfo.eFrameRate = pstVifPortAttr->eFrameRate;
                stVifPortInfo.eCompressMode = pstVifPortAttr->eCompressMode;

                ExecFuncResult(MI_VIF_SetOutputPortAttr(vifDev, vifPort, &stVifPortInfo), s32Ret);
                ExecFuncResult(MI_VIF_EnableOutputPort(vifDev, vifPort), s32Ret);

                pstVifPortAttr->bCreate = TRUE;

                MI_SYS_ChnPort_t stChnPort;
                memset(&stChnPort, 0x0, sizeof(MI_SYS_ChnPort_t));

                stChnPort.eModId=E_MI_MODULE_ID_VIF;
                stChnPort.u32DevId=vifDev;
                stChnPort.u32ChnId=0;
                stChnPort.u32PortId=vifPort;

                ExecFuncResult(MI_SYS_SetChnOutputPortDepth(0, &stChnPort, pstVifPortAttr->stoutFileAttr.u16UserDepth, pstVifPortAttr->stoutFileAttr.u16Depth), s32Ret);

                memcpy(&pstVifPortAttr->stoutFileAttr.stModuleInfo,&stChnPort,sizeof(MI_SYS_ChnPort_t));
                pstVifPortAttr->stoutFileAttr.bThreadExit = FALSE;
                pthread_mutex_init(&pstVifPortAttr->stoutFileAttr.Portmutex, NULL);
                pthread_create(&pstVifPortAttr->stoutFileAttr.pGetDatathread, NULL, ST_GetOutputDataThread, (void *)(&pstVifPortAttr->stoutFileAttr));
            }
        }
    }

    if(GroupId == ST_MAX_VIF_GROUP_NUM-1
        && pstVifModAttr->stRandomParamAttr.bParamRandomTest == TRUE
        && pstVifModAttr->stRandomParamAttr.pRandomParamthread == 0)
    {
        pthread_create(&pstVifModAttr->stRandomParamAttr.pRandomParamthread, NULL, ST_VifRandomParamTestThread, (void *)(&pstVifModAttr->stVifParamSave));
        printf("  vif random thread %lu  init \n", pstVifModAttr->stRandomParamAttr.pRandomParamthread);
    }

EXIT:
#endif
    return s32Ret;
}

MI_S32 ST_TransMISnrPadToMIIspBindSensorId(MI_SNR_PADID eMiSnrPadId, MI_ISP_BindSnrId_e *peMiIspSnrBindId)
{
    switch(eMiSnrPadId)
    {
        case 0:
            *peMiIspSnrBindId = E_MI_ISP_SENSOR0;
            break;
        case 1:
            *peMiIspSnrBindId = E_MI_ISP_SENSOR1;
            break;
        case 2:
            *peMiIspSnrBindId = E_MI_ISP_SENSOR2;
            break;
        case 3:
            *peMiIspSnrBindId = E_MI_ISP_SENSOR3;
            break;
        case 4:
            *peMiIspSnrBindId = E_MI_ISP_SENSOR4;
            break;
        case 5:
            *peMiIspSnrBindId = E_MI_ISP_SENSOR5;
            break;
        case 6:
            *peMiIspSnrBindId = E_MI_ISP_SENSOR6;
            break;
        case 7:
            *peMiIspSnrBindId = E_MI_ISP_SENSOR7;
            break;
        default:
            *peMiIspSnrBindId = E_MI_ISP_SENSOR0;
            printf("[%s]%d snrPad%d fail \n", __FUNCTION__, __LINE__, eMiSnrPadId);
            break;
    }

    return MI_SUCCESS;
}

MI_S32 ST_IspModuleInit(MI_ISP_DEV IspDevId)
{
    MI_S32 s32Ret = MI_SUCCESS;
#if MI_ISP_SUPPORT

    ST_IspModeAttr_t *pstIspModeAttr = &gstIspModule;
    ST_IspDevAttr_t *pstIspDevAttr= &gstIspModule.stIspDevAttr[IspDevId];
    MI_ISP_CHANNEL IspChnId = 0;
    MI_ISP_PORT  IspOutPortId =0;

    if(pstIspDevAttr->bUsed == TRUE
        && pstIspDevAttr->bCreate == FALSE)
    {
        MI_ISP_DevAttr_t stCreateDevAttr;
        memset(&stCreateDevAttr, 0x0, sizeof(MI_ISP_DevAttr_t));
        
		if(TEST_ISP_MULTI_DEV == 1)
		{
			stCreateDevAttr.u32DevStitchMask = E_MI_ISP_DEVICEMASK_ID0|E_MI_ISP_DEVICEMASK_ID1; 
			ExecFuncResult(MI_ISP_CreateDevice(MI_ISP_CREATE_MULTI_DEV(IspDevId), &stCreateDevAttr), s32Ret); 
		}
		else{
			stCreateDevAttr.u32DevStitchMask = (0x1<<IspDevId);
            ExecFuncResult(MI_ISP_CreateDevice(IspDevId, &stCreateDevAttr), s32Ret);
		}
        pstIspDevAttr->bCreate = TRUE;
    }

    for(IspChnId=0; IspChnId<ST_MAX_ISP_CHN_NUM; IspChnId++)
    {
        ST_IspChannelAttr_t  *pstIspChnAttr = &pstIspDevAttr->stIspChnlAttr[IspChnId];
        if(pstIspChnAttr->bUsed == TRUE
            && pstIspChnAttr->bCreate == FALSE)
        {
            MI_ISP_ChannelAttr_t  stIspChnAttr;
            MI_SNR_PADInfo_t  stPad0Info;
            MI_SNR_PlaneInfo_t stSnrPlane0Info;

            memset(&stPad0Info, 0x0, sizeof(MI_SNR_PADInfo_t));
            memset(&stSnrPlane0Info, 0x0, sizeof(MI_SNR_PlaneInfo_t));
            memset(&stIspChnAttr, 0x0, sizeof(MI_ISP_ChannelAttr_t));

            if(pstIspChnAttr->eBindMiSnrPadId < ST_MAX_SENSOR_NUM)
            {
#if MI_VIF_SUPPORT
                if(bUseUserSensor ==FALSE)
                {
                    ExecFuncResult(MI_SNR_GetPadInfo(pstIspChnAttr->eBindMiSnrPadId, &stPad0Info), s32Ret);
                    ExecFuncResult(MI_SNR_GetPlaneInfo(pstIspChnAttr->eBindMiSnrPadId, 0, &stSnrPlane0Info), s32Ret);
                }
                else
                {
#if USER_SENSOR_SUPPORT
                    MI_U8 u8ChipIndex=0;
                    ExecFuncResult(ST_TransSnrPadIdToDH9931Id(pstIspChnAttr->eBindMiSnrPadId, &u8ChipIndex), s32Ret);
                    ExecFuncResult(Cus_GetSnrPadInfo(u8ChipIndex, &stPad0Info), s32Ret);
                    ExecFuncResult(Cus_GetSnrPlaneInfo(u8ChipIndex, 0, &stSnrPlane0Info), s32Ret);
#endif
                }
#endif
            }

            if(stPad0Info.eIntfMode == E_MI_SNR_MODE_MIPI
                && stSnrPlane0Info.eBayerId < E_MI_SYS_PIXEL_BAYERID_MAX)
            {
                ST_TransMISnrPadToMIIspBindSensorId(pstIspChnAttr->eBindMiSnrPadId, (MI_ISP_BindSnrId_e *)&stIspChnAttr.u32SensorBindId);
            }
            else
            {
                stIspChnAttr.u32SensorBindId = E_MI_ISP_SENSOR_INVALID;
            }
			stIspChnAttr.u32Sync3AType = E_MI_ISP_SYNC3A_AE|E_MI_ISP_SYNC3A_AWB|E_MI_ISP_SYNC3A_IQ|E_MI_ISP_SYNC3A_1ST_SNR_ONLY;
            ExecFuncResult(MI_ISP_CreateChannel(IspDevId, IspChnId, &stIspChnAttr), s32Ret);
            memcpy(&pstIspChnAttr->stIspChnAttr, &stIspChnAttr, sizeof(MI_ISP_ChannelAttr_t));
            pstIspChnAttr->bCreate = TRUE;

            if(pstIspChnAttr->stIspInPortAttr[0].stInputCropWin.u16Width !=0
                && pstIspChnAttr->stIspInPortAttr[0].stInputCropWin.u16Height !=0)
            {
                ExecFuncResult(MI_ISP_SetInputPortCrop(IspDevId, IspChnId, &pstIspChnAttr->stIspInPortAttr[0].stInputCropWin), s32Ret);
            }

            if(pstIspChnAttr->stIspChnParam.eHDRType > E_MI_ISP_HDR_TYPE_OFF)
            {
                pstIspChnAttr->stIspChnParam.eHDRType = (MI_ISP_HDRType_e)stPad0Info.eHDRMode;
            }
			pstIspChnAttr->stIspChnParam.e3DNRLevel = E_MI_ISP_3DNR_LEVEL2;

            ExecFuncResult(MI_ISP_SetChnParam(IspDevId, IspChnId, &pstIspChnAttr->stIspChnParam), s32Ret);

            //ai isp setting
            if(pstIspChnAttr->stIspCustSegAttr.eMode > E_MI_ISP_CUST_SEG_MODE_NONE &&
                pstIspChnAttr->stIspCustSegAttr.eMode < E_MI_ISP_CUST_SEG_MODE_MAX)
            {
                MI_S32 ret = MI_SUCCESS;
#if MI_IPU_SUPPORT
                if(pstIspChnAttr->bUseIpu)
                {
                    ret = ST_Ipu_Init(&pstIspChnAttr->u32IpuChn, pstIspChnAttr->aIpuImagePath);
                    STCHECKRESULT(ret);
                    pstIspChnAttr->stAiIspThreadAttr.bUseIpu = pstIspChnAttr->bUseIpu;
                    pstIspChnAttr->stAiIspThreadAttr.u32IpuChn = pstIspChnAttr->u32IpuChn;
                }
                else
#endif
                {
                    pstIspChnAttr->stAiIspThreadAttr.bUseIpu = FALSE;
                }

                ret =MI_ISP_SetCustSegAttr(IspDevId, IspChnId, &pstIspChnAttr->stIspCustSegAttr);
                STCHECKRESULT(ret);
                if(MI_SUCCESS == ret)
                {
                    //customer thread
                    pstIspChnAttr->stAiIspThreadAttr.bExit = FALSE;
                    pstIspChnAttr->stAiIspThreadAttr.u8DevId = IspDevId;
                    pstIspChnAttr->stAiIspThreadAttr.u8ChnId = IspChnId;
                    pthread_create(&pstIspChnAttr->stAiIspThreadAttr.pThreadHandle, NULL, ST_AiIspThread, (void *)(&pstIspChnAttr->stAiIspThreadAttr));
                }
            }
			if(TEST_ISP_MULTI_DEV == 1)
			{
			    STCHECKRESULT(MI_ISP_SetChnOverlapAttr(IspDevId, IspChnId,E_MI_ISP_OVERLAP_256));

			}
            ExecFuncResult(MI_ISP_StartChannel(IspDevId, IspChnId), s32Ret);

            pstIspChnAttr->stIspSkipFarme.u32SikpFrameCnt = 0;
            pthread_mutex_init(&pstIspChnAttr->stIspSkipFarme.SkipMutex, NULL);

            for(IspOutPortId=0; IspOutPortId<ST_MAX_ISP_OUTPORT_NUM; IspOutPortId++)
            {
                ST_PortAttr_t *pstIspOutputAttr = &pstIspChnAttr->stIspOutPortAttr[IspOutPortId];
                if(pstIspOutputAttr->bUsed == TRUE)
                {
                    MI_ISP_OutPortParam_t  stIspOutputParam;
                    MI_SYS_ChnPort_t stChnPort;
                    memset(&stIspOutputParam, 0x0, sizeof(MI_ISP_OutPortParam_t));
                    memset(&stChnPort, 0x0, sizeof(MI_SYS_ChnPort_t));
                    stChnPort.eModId = E_MI_MODULE_ID_ISP;
                    stChnPort.u32DevId = IspDevId;
                    stChnPort.u32ChnId = IspChnId;
                    stChnPort.u32PortId = IspOutPortId;

                    ST_GetIspOutputPortRect(&stChnPort,&pstIspOutputAttr->stPortCrop);

                    if(pstIspOutputAttr->stOrigPortCrop.u16Height == 0
                       || pstIspOutputAttr->stOrigPortCrop.u16Width == 0)
                    {
                        memcpy(&stIspOutputParam.stCropRect,&pstIspOutputAttr->stOrigPortCrop,sizeof(MI_SYS_WindowRect_t));
                    }
                    else
                    {
                        memcpy(&stIspOutputParam.stCropRect,&pstIspOutputAttr->stPortCrop,sizeof(MI_SYS_WindowRect_t));
                    }
                    stIspOutputParam.ePixelFormat = pstIspOutputAttr->ePixelFormat;
                    stIspOutputParam.eCompressMode = pstIspOutputAttr->eCompressMode;

                    ExecFuncResult(MI_ISP_SetOutputPortParam(IspDevId, IspChnId, IspOutPortId, &stIspOutputParam), s32Ret);

                    ExecFuncResult(MI_ISP_EnableOutputPort(IspDevId, IspChnId, IspOutPortId), s32Ret);

                    ExecFuncResult(MI_SYS_SetChnOutputPortDepth(0, &stChnPort , pstIspOutputAttr->stoutFileAttr.u16UserDepth, pstIspOutputAttr->stoutFileAttr.u16Depth), s32Ret);
                    printf("isp module Dev%d, chn%d, port%d depth(%d,%d)\n", stChnPort.u32DevId, stChnPort.u32ChnId, stChnPort.u32PortId,
                        pstIspOutputAttr->stoutFileAttr.u16UserDepth, pstIspOutputAttr->stoutFileAttr.u16Depth);

                    memcpy(&pstIspOutputAttr->stoutFileAttr.stModuleInfo,&stChnPort,sizeof(MI_SYS_ChnPort_t));
                    pstIspOutputAttr->stoutFileAttr.bThreadExit = FALSE;
                    pthread_mutex_init(&pstIspOutputAttr->stoutFileAttr.Portmutex, NULL);
                    pthread_create(&pstIspOutputAttr->stoutFileAttr.pGetDatathread, NULL, ST_GetOutputDataThread, (void *)(&pstIspOutputAttr->stoutFileAttr));
                }
            }

            if(pstIspChnAttr->stIspInPortAttr[0].bUsed == TRUE)
            {
                if(pstIspChnAttr->stIspInPortAttr[0].stBindParam.stChnPort.eModId < E_MI_MODULE_ID_MAX)
                {
                    ST_Sys_BindInfo_T stBindInfo;
                    memset(&stBindInfo, 0x0, sizeof(ST_Sys_BindInfo_T));

                    stBindInfo.stSrcChnPort.eModId = pstIspChnAttr->stIspInPortAttr[0].stBindParam.stChnPort.eModId;
                    stBindInfo.stSrcChnPort.u32DevId = pstIspChnAttr->stIspInPortAttr[0].stBindParam.stChnPort.u32DevId;
                    stBindInfo.stSrcChnPort.u32ChnId = pstIspChnAttr->stIspInPortAttr[0].stBindParam.stChnPort.u32ChnId;
                    stBindInfo.stSrcChnPort.u32PortId = pstIspChnAttr->stIspInPortAttr[0].stBindParam.stChnPort.u32PortId;

                    stBindInfo.stDstChnPort.eModId = E_MI_MODULE_ID_ISP;
                    stBindInfo.stDstChnPort.u32DevId = IspDevId;
                    stBindInfo.stDstChnPort.u32ChnId = IspChnId;
                    stBindInfo.stDstChnPort.u32PortId = 0;

                    stBindInfo.u32SrcFrmrate = pstIspChnAttr->stIspInPortAttr[0].stBindParam.u32SrcFrmrate;
                    stBindInfo.u32DstFrmrate = pstIspChnAttr->stIspInPortAttr[0].stBindParam.u32DstFrmrate;
                    stBindInfo.eBindType = pstIspChnAttr->stIspInPortAttr[0].stBindParam.eBindType;

                    ExecFuncResult(ST_Sys_Bind_List(&stBindInfo), s32Ret);
                }
                else
                {
                    ST_InPortAttr_t  *pstIspInputPort = &pstIspChnAttr->stIspInPortAttr[0];
                    pstIspInputPort->stInputFileAttr.stModuleInfo.eModId = E_MI_MODULE_ID_ISP;
                    pstIspInputPort->stInputFileAttr.stModuleInfo.u32DevId = IspDevId;
                    pstIspInputPort->stInputFileAttr.stModuleInfo.u32ChnId = IspChnId;
                    pstIspInputPort->stInputFileAttr.stModuleInfo.u32PortId = 0;

                    pthread_create(&pstIspInputPort->stInputFileAttr.pPutDatathread, NULL, ST_PutInputDataThread, (void *)(&pstIspInputPort->stInputFileAttr));
                }
            }
        }
    }

     if(IspDevId == ST_MAX_ISP_DEV_NUM-1
        && pstIspModeAttr->stRandomParamAttr.bParamRandomTest == TRUE
        && pstIspModeAttr->stRandomParamAttr.pRandomParamthread == 0)
    {
        pthread_create(&pstIspModeAttr->stRandomParamAttr.pRandomParamthread, NULL, ST_IspRandomParamTestThread, (void *)(&pstIspModeAttr->stIspParamSave));
        printf("isp random thread %lu  init \n", pstIspModeAttr->stRandomParamAttr.pRandomParamthread);
    }

EXIT:
#endif
    return s32Ret;
}

MI_S32 ST_IspModuleUnInit(MI_ISP_DEV IspDevId)
{
    MI_S32 s32Ret = MI_SUCCESS;
#if MI_ISP_SUPPORT

    ST_IspDevAttr_t *pstIspDevAttr= &gstIspModule.stIspDevAttr[IspDevId];
    MI_ISP_CHANNEL IspChnId = 0;
    MI_ISP_PORT  IspOutPortId =0;
    ST_IspModeAttr_t *pstIspModeAttr = &gstIspModule;

    if(pstIspModeAttr->stRandomParamAttr.pRandomParamthread != 0)
    {
        void *retarg = NULL;
        pthread_cancel(pstIspModeAttr->stRandomParamAttr.pRandomParamthread);
        pthread_join(pstIspModeAttr->stRandomParamAttr.pRandomParamthread, &retarg);
        printf("isp random thread %lu  uninit \n", pstIspModeAttr->stRandomParamAttr.pRandomParamthread);
        pstIspModeAttr->stRandomParamAttr.pRandomParamthread = 0;
    }

    for(IspChnId=0; IspChnId<ST_MAX_ISP_CHN_NUM; IspChnId++)
    {
        ST_IspChannelAttr_t  *pstIspChnAttr = &pstIspDevAttr->stIspChnlAttr[IspChnId];
        if(pstIspChnAttr->bUsed == TRUE
            && pstIspChnAttr->bCreate == TRUE)
        {
            MI_ISP_ChannelAttr_t  stIspChnAttr;
            memset(&stIspChnAttr, 0x0, sizeof(MI_ISP_ChannelAttr_t));

            //ai isp setting
            if(0 != pstIspChnAttr->stAiIspThreadAttr.pThreadHandle)
            {
                pstIspChnAttr->stAiIspThreadAttr.bExit = TRUE;
                pthread_join(pstIspChnAttr->stAiIspThreadAttr.pThreadHandle, NULL);
                pstIspChnAttr->stAiIspThreadAttr.pThreadHandle = 0;
            }

#if MI_IPU_SUPPORT
            if(pstIspChnAttr->bUseIpu)
            {
                STCHECKRESULT(ST_Ipu_DeInit(pstIspChnAttr->u32IpuChn));
                pstIspChnAttr->u32IpuChn = 0;
            }
#endif

            if(pstIspChnAttr->stIspInPortAttr[0].bUsed == TRUE)
            {
                if(pstIspChnAttr->stIspInPortAttr[0].stBindParam.stChnPort.eModId < E_MI_MODULE_ID_MAX)
                {
                    ST_Sys_BindInfo_T stBindInfo;
                    memset(&stBindInfo, 0x0, sizeof(ST_Sys_BindInfo_T));
                    stBindInfo.stSrcChnPort.eModId = pstIspChnAttr->stIspInPortAttr[0].stBindParam.stChnPort.eModId;
                    stBindInfo.stSrcChnPort.u32DevId = pstIspChnAttr->stIspInPortAttr[0].stBindParam.stChnPort.u32DevId;
                    stBindInfo.stSrcChnPort.u32ChnId = pstIspChnAttr->stIspInPortAttr[0].stBindParam.stChnPort.u32ChnId;
                    stBindInfo.stSrcChnPort.u32PortId = pstIspChnAttr->stIspInPortAttr[0].stBindParam.stChnPort.u32PortId;

                    stBindInfo.stDstChnPort.eModId = E_MI_MODULE_ID_ISP;
                    stBindInfo.stDstChnPort.u32DevId = IspDevId;
                    stBindInfo.stDstChnPort.u32ChnId = IspChnId;
                    stBindInfo.stDstChnPort.u32PortId = 0;
                    stBindInfo.u32SrcFrmrate = 30;
                    stBindInfo.u32DstFrmrate = 30;
                    ExecFuncResult(ST_Sys_UnBind_List(&stBindInfo), s32Ret);
                }
                else
                {
                    void *retarg = NULL;
                    ST_InPortAttr_t  *pstIspInputPort = &pstIspChnAttr->stIspInPortAttr[0];

                    pthread_cancel(pstIspInputPort->stInputFileAttr.pPutDatathread);
                    pthread_join(pstIspInputPort->stInputFileAttr.pPutDatathread, &retarg);
                }
            }

            for(IspOutPortId=0; IspOutPortId<ST_MAX_ISP_OUTPORT_NUM; IspOutPortId++)
            {
                ST_PortAttr_t *pstIspOutputAttr = &pstIspChnAttr->stIspOutPortAttr[IspOutPortId];
                if(pstIspOutputAttr->bUsed == TRUE)
                {
                    void *retarg = NULL;
                    pstIspOutputAttr->stoutFileAttr.bThreadExit = TRUE;
                    //pthread_cancel(pstIspOutputAttr->stoutFileAttr.pGetDatathread);
                    pthread_join(pstIspOutputAttr->stoutFileAttr.pGetDatathread, &retarg);
                    pthread_mutex_destroy(&pstIspOutputAttr->stoutFileAttr.Portmutex);

                    ExecFuncResult(MI_ISP_DisableOutputPort(IspDevId, IspChnId, IspOutPortId), s32Ret);
                    pstIspOutputAttr->bUsed = FALSE;
                }
            }

            ExecFuncResult(MI_ISP_StopChannel(IspDevId, IspChnId), s32Ret);
            ExecFuncResult(MI_ISP_DestroyChannel(IspDevId, IspChnId), s32Ret);
            pstIspChnAttr->bCreate = FALSE;
            pstIspChnAttr->stIspSkipFarme.u32SikpFrameCnt = 0;
            pthread_mutex_destroy(&pstIspChnAttr->stIspSkipFarme.SkipMutex);
        }
    }

    if(pstIspDevAttr->bUsed == TRUE
        && pstIspDevAttr->bCreate == TRUE)
    {
        ExecFuncResult(MI_ISP_DestoryDevice(IspDevId), s32Ret);
        pstIspDevAttr->bCreate = FALSE;
    }

EXIT:
#endif
    return s32Ret;
}


MI_S32 ST_SclCreateChn(MI_SCL_DEV SclDevId, MI_SCL_CHANNEL SclChnId)
{
    MI_S32 s32Ret = MI_SUCCESS;
#if MI_SCL_SUPPORT

    ST_SclDevAttr_t *pstSclDevAttr= &gstSclModule.stSclDevAttr[SclDevId];
    ST_SclChannelAttr_t  *pstSclChnAttr = &pstSclDevAttr->stSclChnlAttr[SclChnId];
    MI_SCL_PORT  SclOutPortId =0;
    MI_SYS_Rotate_e eRot=E_MI_SYS_ROTATE_NONE;
    MI_BOOL bMirror =FALSE, bFlip=FALSE;
    MI_SYS_WindowSize_t  stSrcWinSize;
    memset(&stSrcWinSize, 0x0, sizeof(MI_SYS_WindowSize_t));

    if(pstSclChnAttr->bUsed == TRUE
        && pstSclChnAttr->bCreate == FALSE)
    {
        MI_SCL_ChannelAttr_t  stSclChnAttr;
        memset(&stSclChnAttr, 0x0, sizeof(MI_SCL_ChannelAttr_t));

        ExecFuncResult(MI_SCL_CreateChannel((MI_SCL_DEV)SclDevId, SclChnId, &stSclChnAttr), s32Ret);
        pstSclChnAttr->bCreate = TRUE;

        if(pstSclChnAttr->stSclInPortAttr[0].stBindParam.stChnPort.eModId == E_MI_MODULE_ID_ISP)
        {
            MI_SYS_ChnPort_t  *pstChnPort = &pstSclChnAttr->stSclInPortAttr[0].stBindParam.stChnPort;
            MI_ISP_ChnParam_t stChnParam;
            ST_PortAttr_t *pstIspOutputAttr = &gstIspModule.stIspDevAttr[pstChnPort->u32DevId].stIspChnlAttr[pstChnPort->u32ChnId].stIspOutPortAttr[pstChnPort->u32PortId];
            memset(&stChnParam, 0x0, sizeof(MI_ISP_ChnParam_t));
#if MI_ISP_SUPPORT
            ExecFuncResult(MI_ISP_GetChnParam(pstChnPort->u32DevId,pstChnPort->u32ChnId, &stChnParam), s32Ret);
#endif
            eRot = stChnParam.eRot;
            bMirror = stChnParam.bMirror;
            bFlip = stChnParam.bFlip;
            if(eRot == E_MI_SYS_ROTATE_90 || eRot == E_MI_SYS_ROTATE_270)//isp output before rot
            {
                stSrcWinSize.u16Width = pstIspOutputAttr->stPortCrop.u16Height;
                stSrcWinSize.u16Height = pstIspOutputAttr->stPortCrop.u16Width;
            }
            else
            {
                stSrcWinSize.u16Width = pstIspOutputAttr->stPortCrop.u16Width;
                stSrcWinSize.u16Height = pstIspOutputAttr->stPortCrop.u16Height;
            }
        }
        else
        {
            stSrcWinSize.u16Width = pstSclChnAttr->stSclInPortAttr[0].stInputFileAttr.u32Width;
            stSrcWinSize.u16Height = pstSclChnAttr->stSclInPortAttr[0].stInputFileAttr.u32Height;
        }

        if(pstSclChnAttr->stSclInPortAttr[0].stOrigInputCropWin.u16Width !=0
            && pstSclChnAttr->stSclInPortAttr[0].stOrigInputCropWin.u16Height !=0)//org is before rotation relative output
        {
            ST_GetRotAfterCropRect(stSrcWinSize, pstSclChnAttr->stSclInPortAttr[0].stOrigInputCropWin, eRot, bMirror, bFlip, &pstSclChnAttr->stSclInPortAttr[0].stInputCropWin);

            ExecFuncResult(MI_SCL_SetInputPortCrop((MI_SCL_DEV)SclDevId, SclChnId, &pstSclChnAttr->stSclInPortAttr[0].stInputCropWin), s32Ret);

            if(eRot == E_MI_SYS_ROTATE_90 || eRot == E_MI_SYS_ROTATE_270)
            {
                stSrcWinSize.u16Width = pstSclChnAttr->stSclInPortAttr[0].stInputCropWin.u16Height;
                stSrcWinSize.u16Height = pstSclChnAttr->stSclInPortAttr[0].stInputCropWin.u16Width;
            }
            else
            {
                stSrcWinSize.u16Width = pstSclChnAttr->stSclInPortAttr[0].stInputCropWin.u16Width;
                stSrcWinSize.u16Height = pstSclChnAttr->stSclInPortAttr[0].stInputCropWin.u16Height;
            }
        }
        else
        {
            pstSclChnAttr->stSclInPortAttr[0].stInputCropWin.u16X = 0;
            pstSclChnAttr->stSclInPortAttr[0].stInputCropWin.u16Y = 0;
            pstSclChnAttr->stSclInPortAttr[0].stInputCropWin.u16Width = stSrcWinSize.u16Width;
            pstSclChnAttr->stSclInPortAttr[0].stInputCropWin.u16Height = stSrcWinSize.u16Height;
        }

        MI_SCL_ChnParam_t  stSclChnParam;
        memset(&stSclChnParam, 0x0, sizeof(MI_SCL_ChnParam_t));
        stSclChnParam.eRot = pstSclChnAttr->eRotate;
        ExecFuncResult(MI_SCL_SetChnParam((MI_SCL_DEV)SclDevId, SclChnId, &stSclChnParam), s32Ret);

        ExecFuncResult(MI_SCL_StartChannel((MI_SCL_DEV)SclDevId, SclChnId), s32Ret);

        for(SclOutPortId=0; SclOutPortId<ST_MAX_SCL_OUTPORT_NUM; SclOutPortId++)
        {
            ST_PortAttr_t *pstSclOutputAttr = &pstSclChnAttr->stSclOutPortAttr[SclOutPortId];
            if(pstSclOutputAttr->bUsed == TRUE
                && pstSclOutputAttr->bEnable == FALSE)
            {
                MI_SCL_OutPortParam_t  stSclOutputParam;
                memset(&stSclOutputParam, 0x0, sizeof(MI_SCL_OutPortParam_t));

                ST_GetRotAfterCropRect(stSrcWinSize, pstSclOutputAttr->stOrigPortCrop, eRot, bMirror, bFlip, &pstSclOutputAttr->stPortCrop);
                memcpy(&stSclOutputParam.stSCLOutCropRect, &pstSclOutputAttr->stPortCrop, sizeof(MI_SYS_WindowRect_t));
                memcpy(&pstSclOutputAttr->stPortSize, &pstSclOutputAttr->stOrigPortSize, sizeof(MI_SYS_WindowSize_t));

                if((stSclChnParam.eRot == E_MI_SYS_ROTATE_90 || stSclChnParam.eRot == E_MI_SYS_ROTATE_270)
                    ^ (eRot==E_MI_SYS_ROTATE_90 || eRot == E_MI_SYS_ROTATE_270))
                {
                    pstSclOutputAttr->stPortSize.u16Width = pstSclOutputAttr->stOrigPortSize.u16Height;
                    pstSclOutputAttr->stPortSize.u16Height = pstSclOutputAttr->stOrigPortSize.u16Width;
                }
                memcpy(&stSclOutputParam.stSCLOutputSize, &pstSclOutputAttr->stPortSize, sizeof(MI_SYS_WindowSize_t));
                stSclOutputParam.ePixelFormat = pstSclOutputAttr->ePixelFormat;
                stSclOutputParam.eCompressMode = pstSclOutputAttr->eCompressMode;
                stSclOutputParam.bMirror = pstSclOutputAttr->bMirror;
                stSclOutputParam.bFlip = pstSclOutputAttr->bFlip;

                ExecFuncResult(MI_SCL_SetOutputPortParam((MI_SCL_DEV)SclDevId, SclChnId, SclOutPortId, &stSclOutputParam), s32Ret);

                ExecFuncResult(MI_SCL_EnableOutputPort((MI_SCL_DEV)SclDevId, SclChnId, SclOutPortId), s32Ret);
                pstSclOutputAttr->bEnable = TRUE;

                MI_SYS_ChnPort_t stChnPort;
                stChnPort.eModId = E_MI_MODULE_ID_SCL;
                stChnPort.u32DevId = SclDevId;
                stChnPort.u32ChnId = SclChnId;
                stChnPort.u32PortId = SclOutPortId;

                ExecFuncResult(MI_SYS_SetChnOutputPortDepth(0, &stChnPort , pstSclOutputAttr->stoutFileAttr.u16UserDepth, pstSclOutputAttr->stoutFileAttr.u16Depth), s32Ret);
                printf("scl module Dev%d, chn%d, port%d depth(%d,%d)\n", stChnPort.u32DevId, stChnPort.u32ChnId, stChnPort.u32PortId,
                    pstSclOutputAttr->stoutFileAttr.u16UserDepth, pstSclOutputAttr->stoutFileAttr.u16Depth);

                memcpy(&pstSclOutputAttr->stoutFileAttr.stModuleInfo,&stChnPort,sizeof(MI_SYS_ChnPort_t));
                pstSclOutputAttr->stoutFileAttr.bThreadExit = FALSE;
                pthread_mutex_init(&pstSclOutputAttr->stoutFileAttr.Portmutex, NULL);
                pthread_create(&pstSclOutputAttr->stoutFileAttr.pGetDatathread, NULL, ST_GetOutputDataThread, (void *)(&pstSclOutputAttr->stoutFileAttr));
            }
        }
    }

EXIT:
#endif
    return s32Ret;
}

MI_S32 ST_SclDestroyChn(MI_SCL_DEV SclDevId, MI_SCL_CHANNEL SclChnId)
{
    MI_S32 s32Ret = MI_SUCCESS;
#if MI_SCL_SUPPORT

    MI_SCL_PORT  SclOutPortId =0;
    ST_SclDevAttr_t *pstSclDevAttr= &gstSclModule.stSclDevAttr[SclDevId];
    ST_SclChannelAttr_t  *pstSclChnAttr = &pstSclDevAttr->stSclChnlAttr[SclChnId];
    if(pstSclChnAttr->bUsed == TRUE
        && pstSclChnAttr->bCreate == TRUE)
    {
        for(SclOutPortId=0; SclOutPortId<ST_MAX_SCL_OUTPORT_NUM; SclOutPortId++)
        {
            ST_PortAttr_t *pstSclOutputAttr = &pstSclChnAttr->stSclOutPortAttr[SclOutPortId];
            if(pstSclOutputAttr->bUsed == TRUE
               && pstSclOutputAttr->bEnable == TRUE
              )
            {
                void *retarg = NULL;
                pstSclOutputAttr->stoutFileAttr.bThreadExit = TRUE;
                //pthread_cancel(pstSclOutputAttr->stoutFileAttr.pGetDatathread);
                pthread_join(pstSclOutputAttr->stoutFileAttr.pGetDatathread, &retarg);
                pthread_mutex_destroy(&pstSclOutputAttr->stoutFileAttr.Portmutex);

                ExecFuncResult(MI_SCL_DisableOutputPort((MI_SCL_DEV)SclDevId, SclChnId, SclOutPortId), s32Ret);
                pstSclOutputAttr->bEnable = FALSE;
            }
        }

        ExecFuncResult(MI_SCL_StopChannel((MI_SCL_DEV)SclDevId, SclChnId), s32Ret);
        ExecFuncResult(MI_SCL_DestroyChannel((MI_SCL_DEV)SclDevId, SclChnId), s32Ret);
        pstSclChnAttr->bCreate = FALSE;
    }

EXIT:
#endif
    return s32Ret;
}

MI_S32 ST_SclModuleInit(MI_SCL_DEV SclDevId)
{
    MI_S32 s32Ret = MI_SUCCESS;
#if MI_SCL_SUPPORT

    ST_SclModeAttr_t *pstSclModeAttr =  &gstSclModule;
    ST_SclDevAttr_t *pstSclDevAttr= &gstSclModule.stSclDevAttr[SclDevId];
    MI_SCL_CHANNEL SclChnId = 0;

    if(pstSclDevAttr->bUsed == TRUE
        && pstSclDevAttr->bCreate == FALSE)
    {
        MI_SCL_DevAttr_t stCreateDevAttr;
        memset(&stCreateDevAttr, 0x0, sizeof(MI_SCL_DevAttr_t));
        stCreateDevAttr.u32NeedUseHWOutPortMask = pstSclDevAttr->u32UseHwSclMask;

        ExecFuncResult(MI_SCL_CreateDevice((MI_SCL_DEV)SclDevId, &stCreateDevAttr), s32Ret);
        pstSclDevAttr->bCreate = TRUE;
    }

    for(SclChnId=0; SclChnId<ST_MAX_SCL_CHN_NUM; SclChnId++)
    {
        ST_SclChannelAttr_t  *pstSclChnAttr = &pstSclDevAttr->stSclChnlAttr[SclChnId];
        ExecFuncResult(ST_SclCreateChn(SclDevId, SclChnId), s32Ret);

        if(pstSclChnAttr->stSclInPortAttr[0].bUsed == TRUE)
        {
            if(pstSclChnAttr->stSclInPortAttr[0].stBindParam.stChnPort.eModId < E_MI_MODULE_ID_MAX)
            {
                ST_Sys_BindInfo_T stBindInfo;
                memset(&stBindInfo, 0x0, sizeof(ST_Sys_BindInfo_T));

                stBindInfo.stSrcChnPort.eModId = pstSclChnAttr->stSclInPortAttr[0].stBindParam.stChnPort.eModId;
                stBindInfo.stSrcChnPort.u32DevId = pstSclChnAttr->stSclInPortAttr[0].stBindParam.stChnPort.u32DevId;
                stBindInfo.stSrcChnPort.u32ChnId = pstSclChnAttr->stSclInPortAttr[0].stBindParam.stChnPort.u32ChnId;
                stBindInfo.stSrcChnPort.u32PortId = pstSclChnAttr->stSclInPortAttr[0].stBindParam.stChnPort.u32PortId;

                stBindInfo.stDstChnPort.eModId = E_MI_MODULE_ID_SCL;
                stBindInfo.stDstChnPort.u32DevId = SclDevId;
                stBindInfo.stDstChnPort.u32ChnId = SclChnId;
                stBindInfo.stDstChnPort.u32PortId = 0;

                stBindInfo.u32SrcFrmrate = pstSclChnAttr->stSclInPortAttr[0].stBindParam.u32SrcFrmrate;
                stBindInfo.u32DstFrmrate = pstSclChnAttr->stSclInPortAttr[0].stBindParam.u32DstFrmrate;
                stBindInfo.eBindType = pstSclChnAttr->stSclInPortAttr[0].stBindParam.eBindType;

                ExecFuncResult(ST_Sys_Bind_List(&stBindInfo), s32Ret);
            }
            else
            {
                ST_InPortAttr_t  *pstSclInputPort = &pstSclChnAttr->stSclInPortAttr[0];
                pstSclInputPort->stInputFileAttr.stModuleInfo.eModId = E_MI_MODULE_ID_SCL;
                pstSclInputPort->stInputFileAttr.stModuleInfo.u32DevId = SclDevId;
                pstSclInputPort->stInputFileAttr.stModuleInfo.u32ChnId = SclChnId;
                pstSclInputPort->stInputFileAttr.stModuleInfo.u32PortId = 0;

                pthread_create(&pstSclInputPort->stInputFileAttr.pPutDatathread, NULL, ST_PutInputDataThread, (void *)(&pstSclInputPort->stInputFileAttr));
            }
        }
    }

    if(SclDevId == ST_MAX_SCL_DEV_NUM-1
        &&pstSclModeAttr->stRandomParamAttr.bParamRandomTest == TRUE
        && pstSclModeAttr->stRandomParamAttr.pRandomParamthread == 0)
    {
        pthread_create(&pstSclModeAttr->stRandomParamAttr.pRandomParamthread, NULL, ST_SclRandomParamTestThread, (void *)(&pstSclModeAttr->stSclParamSave));
        printf("scl random thread %lu  init \n", pstSclModeAttr->stRandomParamAttr.pRandomParamthread);
    }

EXIT:
#endif
    return s32Ret;
}

MI_S32 ST_SclModuleUnInit(MI_SCL_DEV SclDevId)
{
    MI_S32 s32Ret = MI_SUCCESS;
#if MI_SCL_SUPPORT

    ST_SclDevAttr_t *pstSclDevAttr= &gstSclModule.stSclDevAttr[SclDevId];
    MI_SCL_CHANNEL SclChnId = 0;
    ST_SclModeAttr_t *pstSclModeAttr = &gstSclModule;

    if(pstSclModeAttr->stRandomParamAttr.pRandomParamthread != 0)
    {
        void *retarg = NULL;
        pthread_cancel(pstSclModeAttr->stRandomParamAttr.pRandomParamthread);
        pthread_join(pstSclModeAttr->stRandomParamAttr.pRandomParamthread, &retarg);
        printf(" scl random thread %lu uninit \n", pstSclModeAttr->stRandomParamAttr.pRandomParamthread);
        pstSclModeAttr->stRandomParamAttr.pRandomParamthread = 0;
    }

    for(SclChnId=0; SclChnId<ST_MAX_SCL_CHN_NUM; SclChnId++)
    {
        ST_SclChannelAttr_t  *pstSclChnAttr = &pstSclDevAttr->stSclChnlAttr[SclChnId];

        if(pstSclChnAttr->stSclInPortAttr[0].bUsed == TRUE)
        {
            if(pstSclChnAttr->stSclInPortAttr[0].stBindParam.stChnPort.eModId < E_MI_MODULE_ID_MAX)
            {
                ST_Sys_BindInfo_T stBindInfo;
                memset(&stBindInfo, 0x0, sizeof(ST_Sys_BindInfo_T));
                stBindInfo.stSrcChnPort.eModId = pstSclChnAttr->stSclInPortAttr[0].stBindParam.stChnPort.eModId;
                stBindInfo.stSrcChnPort.u32DevId = pstSclChnAttr->stSclInPortAttr[0].stBindParam.stChnPort.u32DevId;
                stBindInfo.stSrcChnPort.u32ChnId = pstSclChnAttr->stSclInPortAttr[0].stBindParam.stChnPort.u32ChnId;
                stBindInfo.stSrcChnPort.u32PortId = pstSclChnAttr->stSclInPortAttr[0].stBindParam.stChnPort.u32PortId;

                stBindInfo.stDstChnPort.eModId = E_MI_MODULE_ID_SCL;
                stBindInfo.stDstChnPort.u32DevId = SclDevId;
                stBindInfo.stDstChnPort.u32ChnId = SclChnId;
                stBindInfo.stDstChnPort.u32PortId = 0;
                stBindInfo.u32SrcFrmrate = 30;
                stBindInfo.u32DstFrmrate = 30;
                ExecFuncResult(ST_Sys_UnBind_List(&stBindInfo), s32Ret);
            }
            else
            {
                void *retarg = NULL;
                ST_InPortAttr_t  *pstSclInputPort = &pstSclChnAttr->stSclInPortAttr[0];
                pthread_cancel(pstSclInputPort->stInputFileAttr.pPutDatathread);
                pthread_join(pstSclInputPort->stInputFileAttr.pPutDatathread, &retarg);
            }
            ExecFuncResult(ST_SclDestroyChn(SclDevId, SclChnId), s32Ret);
        }
    }

    if(pstSclDevAttr->bUsed == TRUE
        && pstSclDevAttr->bCreate == TRUE)
    {
        ExecFuncResult(MI_SCL_DestroyDevice((MI_SCL_DEV)SclDevId), s32Ret);
        pstSclDevAttr->bCreate = FALSE;
    }

EXIT:
#endif
    return s32Ret;
}

MI_S32 ST_SysModuleInit()
{
    ST_Sys_Init();
    return MI_SUCCESS;
}
MI_S32 ST_BaseModuleInit()
{
    MI_U8 eSnrPad = 0;
    MI_U32 u32VifGroup = 0;
    MI_ISP_DEV IspDevId =0;
    MI_U32 SclDevId =0;
    MI_U32 u32VencChn = 0;
    MI_DISP_DEV s32DispDev =0;
    MI_U32 u32JpdChn=0;
    MI_S32 s32Ret = MI_SUCCESS;

    ExecFuncResult(ST_SysModuleInit(), s32Ret);

    for(eSnrPad=0; eSnrPad<ST_MAX_SENSOR_NUM; eSnrPad++)
    {
        ST_Sensor_Attr_t *pstSensorAttr = &gstSensorAttr[eSnrPad];
        if(pstSensorAttr->bUsed == TRUE && pstSensorAttr->bCreate == FALSE)
        {
            pstSensorAttr->bCreate = TRUE;
            if(bUseUserSensor == FALSE)
            {
                ExecFuncResult(ST_SensorModuleInit((MI_SNR_PADID)eSnrPad), s32Ret);
            }
            else
            {
                ExecFuncResult(ST_UserSensorModuleInit((MI_SNR_PADID)eSnrPad), s32Ret);
            }
        }
    }
#if USER_SENSOR_SUPPORT
    if(bUseUserSensor == TRUE)
        ExecFuncResult(Cus_SnrEnable(), s32Ret);
#endif
    for(u32VifGroup=0; u32VifGroup<ST_MAX_VIF_GROUP_NUM; u32VifGroup++)
    {
        ExecFuncResult(ST_VifModuleInit((MI_VIF_GROUP)u32VifGroup), s32Ret);
    }

    for(u32JpdChn=0; u32JpdChn<ST_MAX_JPD_CHN_NUM; u32JpdChn++)
    {
        ST_JpdChnAttr_t *pstJpdChnAttr = &gstJpdModeAttr.stJpdChnAttr[u32JpdChn];
        if(pstJpdChnAttr->bUsed==TRUE &&  pstJpdChnAttr->bCreate == FALSE)
        {
            ExecFuncResult(ST_JpdModuleInit(u32JpdChn), s32Ret);
            pstJpdChnAttr->bCreate = TRUE;
        }
    }

    for(IspDevId=0; IspDevId<ST_MAX_ISP_DEV_NUM; IspDevId++)
    {
        ExecFuncResult(ST_IspModuleInit(IspDevId), s32Ret);
    }

    for(SclDevId=0; SclDevId<ST_MAX_SCL_DEV_NUM; SclDevId++)
    {
        ExecFuncResult(ST_SclModuleInit((MI_SCL_DEV)SclDevId), s32Ret);
    }

    for(u32VencChn=0; u32VencChn < ST_MAX_VENC_NUM; u32VencChn++)
    {
        ST_VencAttr_t*pstVencChnattr = &gstVencattr[u32VencChn];
        if(TRUE == pstVencChnattr->bUsed && FALSE == pstVencChnattr->bCreate)
        {
            ExecFuncResult(ST_VencModuleInit(u32VencChn), s32Ret);
        }
    }

    for(s32DispDev=0; s32DispDev < ST_MAX_DISP_DEV_NUM; s32DispDev++)
    {
        ST_DispDevAttr_t    *pstDispDevAttr = &gstDispModule.stDispDevAttr[s32DispDev];
        if(TRUE == pstDispDevAttr->bUsed && FALSE == pstDispDevAttr->bCreate)
        {
            ExecFuncResult(ST_DispModuleInit(s32DispDev), s32Ret);
        }
    }

EXIT:
    return s32Ret;
}

MI_S32 ST_VifModuleUnInit(MI_VIF_GROUP GroupId)
{
    MI_S32 s32Ret = MI_SUCCESS;
#if MI_VIF_SUPPORT
    MI_VIF_DEV vifDev = 0;
    MI_U32 vifDevIdPerGroup =0;
    MI_VIF_PORT vifPort=0;
    ST_VifGroupAttr_t  *pstVifGroupAttr = &gstVifModule.stVifGroupAttr[GroupId];
    ST_VifModAttr_t *pstVifModAttr = &gstVifModule;

    if(pstVifModAttr->stRandomParamAttr.pRandomParamthread != 0)
    {
        void *retarg = NULL;
        pthread_cancel(pstVifModAttr->stRandomParamAttr.pRandomParamthread);
        pthread_join(pstVifModAttr->stRandomParamAttr.pRandomParamthread, &retarg);
        printf("vif random thread %lu  uninit \n", pstVifModAttr->stRandomParamAttr.pRandomParamthread);
        pstVifModAttr->stRandomParamAttr.pRandomParamthread = 0;
    }

    for(vifDevIdPerGroup=ST_MAX_VIF_DEV_PERGROUP; vifDevIdPerGroup>0; vifDevIdPerGroup--)
    {
        ST_VifDevAttr_t *pstDevAttr = &pstVifGroupAttr->stVifDevAttr[vifDevIdPerGroup-1];
        vifDev = GroupId*ST_MAX_VIF_DEV_PERGROUP+vifDevIdPerGroup-1;

        for(vifPort=0; vifPort< ST_MAX_VIF_OUTPORT_NUM; vifPort++)
        {
            ST_VifPortAttr_t *pstVifOutputPortAttr = &pstDevAttr->stVifOutPortAttr[vifPort];
            if(pstVifOutputPortAttr->bUsed == TRUE
                && pstVifOutputPortAttr->bCreate== TRUE
                )
            {
                void *retarg = NULL;
                pstVifOutputPortAttr->stoutFileAttr.bThreadExit = TRUE;
                //pthread_cancel(pstVifOutputPortAttr->stoutFileAttr.pGetDatathread);
                pthread_join(pstVifOutputPortAttr->stoutFileAttr.pGetDatathread, &retarg);
                pthread_mutex_destroy(&pstVifOutputPortAttr->stoutFileAttr.Portmutex);

                ExecFuncResult(MI_VIF_DisableOutputPort(vifDev, vifPort), s32Ret);
                pstVifOutputPortAttr->bCreate = FALSE;
            }
        }

        if(pstDevAttr->bUsed == TRUE
           && pstDevAttr->bCreate ==TRUE)
        {
            ExecFuncResult(MI_VIF_DisableDev(vifDev), s32Ret);
            pstDevAttr->bCreate=FALSE;
            pthread_mutex_destroy(&pstDevAttr->Devmutex);
        }
    }

    if(pstVifGroupAttr->bCreate == TRUE)
    {
        ExecFuncResult(MI_VIF_DestroyDevGroup(GroupId), s32Ret);
        pstVifGroupAttr->bCreate = FALSE;
    }

EXIT:
#endif
    return s32Ret;
}
MI_S32 ST_SensorModuleUnInit(MI_SNR_PADID eSnrPad)
{
    MI_S32 s32Ret = MI_SUCCESS;
#if MI_VIF_SUPPORT
    MI_SNR_PADID eSnrPadId = eSnrPad;

   ExecFuncResult(MI_SNR_Disable(eSnrPadId), s32Ret);

EXIT:
#endif
    return s32Ret;
}

MI_S32 ST_UserSensorModuleUnInit(MI_SNR_PADID eSnrPad)
{
    MI_S32 s32Ret = MI_SUCCESS;
#if USER_SENSOR_SUPPORT
    MI_U8 u8ChipIndex=0;
    ExecFuncResult(ST_TransSnrPadIdToDH9931Id(eSnrPad, &u8ChipIndex), s32Ret);

    ExecFuncResult(Cus_SnrDisable(u8ChipIndex), s32Ret);
EXIT:
#endif
    return s32Ret;
}

MI_S32 ST_SysModuleUnInit()
{
    MI_S32 s32Ret = MI_SUCCESS;

    ExecFuncResult(MI_SYS_Exit(0),s32Ret);

EXIT:
    return s32Ret;
}

MI_S32 ST_JpdModuleUnInit(MI_U32 u32JpdChn)
{
    MI_S32 s32Ret = MI_SUCCESS;
#if MI_JPD_SUPPORT
    void *retarg = NULL;
    MI_U32 u32JpdDevId=0;
    ST_JpdChnAttr_t *pstJpdChnAttr = &gstJpdModeAttr.stJpdChnAttr[u32JpdChn];

    pthread_cancel(pstJpdChnAttr->pPutDatathread);
    pthread_join(pstJpdChnAttr->pPutDatathread, &retarg);

    ExecFuncResult(MI_JPD_StopChn(u32JpdDevId,u32JpdChn), s32Ret);
    ExecFuncResult(MI_JPD_DestroyChn(u32JpdDevId,u32JpdChn), s32Ret);

EXIT:
#endif
    return s32Ret;
}

MI_S32 ST_BaseModuleUnInit()
{
    MI_S32 s32Ret = MI_SUCCESS;
    int i = 0;
    MI_U32 u32VifGroup=0;
    MI_U8 eSnrPad = 0;
    MI_ISP_DEV IspDevId =0;
    MI_U32 SclDevId =0;
    for(i=0;i<ST_MAX_DISP_DEV_NUM;i++)
    {
        ST_DispDevAttr_t    *pstDispDevAttr = &gstDispModule.stDispDevAttr[i];
        if(TRUE == pstDispDevAttr->bUsed && TRUE == pstDispDevAttr->bCreate)
        {
            ExecFuncResult(ST_DispModuleUnInit(i), s32Ret);
        }
    }
    for(i=0;i<ST_MAX_VENC_NUM;i++)
    {
        ST_VencAttr_t*pstVencChnattr = &gstVencattr[i];
        if(TRUE == pstVencChnattr->bUsed && TRUE == pstVencChnattr->bCreate)
        {
            ExecFuncResult(ST_VencModuleUnInit(i), s32Ret);
        }
    }

    for(SclDevId=0; SclDevId<ST_MAX_SCL_DEV_NUM; SclDevId++)
    {
        ExecFuncResult(ST_SclModuleUnInit((MI_SCL_DEV)SclDevId), s32Ret);
    }

    for(IspDevId=0; IspDevId<ST_MAX_ISP_DEV_NUM; IspDevId++)
    {
        ExecFuncResult(ST_IspModuleUnInit(IspDevId), s32Ret);
    }

    for(i=0; i<ST_MAX_JPD_CHN_NUM; i++)
    {
        ST_JpdChnAttr_t *pstJpdChnattr = &gstJpdModeAttr.stJpdChnAttr[i];
        if(pstJpdChnattr->bUsed == TRUE && pstJpdChnattr->bCreate == TRUE)
        {
            pstJpdChnattr->bCreate = FALSE;
            ExecFuncResult(ST_JpdModuleUnInit(i), s32Ret);
        }
    }

    for(u32VifGroup=0; u32VifGroup<ST_MAX_VIF_GROUP_NUM; u32VifGroup++)
    {
        ExecFuncResult(ST_VifModuleUnInit((MI_VIF_GROUP)u32VifGroup), s32Ret);
    }

    for(eSnrPad=0; eSnrPad<ST_MAX_SENSOR_NUM; eSnrPad++)
    {
        ST_Sensor_Attr_t *pstSensorAttr = &gstSensorAttr[eSnrPad];
        if(pstSensorAttr->bUsed == TRUE && pstSensorAttr->bCreate == TRUE)
        {
            pstSensorAttr->bCreate = FALSE;
            if(bUseUserSensor == FALSE)
            {
                ExecFuncResult(ST_SensorModuleUnInit((MI_SNR_PADID)eSnrPad), s32Ret);
            }
            else
            {
                ExecFuncResult(ST_UserSensorModuleUnInit((MI_SNR_PADID)eSnrPad), s32Ret);
            }
        }
    }

    ExecFuncResult(ST_SysModuleUnInit(), s32Ret);

EXIT:
    return s32Ret;
}

MI_S32 ST_IniSetVencParam(MI_U32 u32VencId,MI_VENC_ModType_e  eEncType, ST_BindParam_t *pstVencInBindParam, MI_U32 u32Fps)
{
    if(eEncType > E_MI_VENC_MODTYPE_VENC
        && eEncType < E_MI_VENC_MODTYPE_MAX)
    {
        MI_U16 u16VencChn = 0;
        ST_VencAttr_t  *pstVencAttr = NULL;
        for(u16VencChn=0; u16VencChn<ST_MAX_VENC_NUM; u16VencChn++)
        {
            pstVencAttr = &gstVencattr[u16VencChn];
            if(pstVencAttr->bUsed == FALSE)
                break;
        }

        if(u16VencChn == ST_MAX_VENC_NUM)
        {
            printf("[%s]%d find venc chn fail max %d all use \n", __FUNCTION__,__LINE__, ST_MAX_VENC_NUM);
            return -1;
        }

        pstVencAttr->bUsed = TRUE;
        pstVencAttr->eType = eEncType;
        pstVencAttr->vencChn = u16VencChn%ST_MAX_VENC_CHN_NUM_PERDEV;
        pstVencAttr->u32Fps = u32Fps;
        pstVencAttr->DevId = u32VencId;
        //sprintf(pstVencAttr->szStreamName, "video%d",u16VencChn);

        memcpy(&pstVencAttr->stVencInBindParam, pstVencInBindParam, sizeof(ST_BindParam_t));
    }
    else
        printf("encode type %d not support \n", eEncType);

    return MI_SUCCESS;
}

MI_SYS_PixelFormat_e ST_GetIniPixel(char *pPixeString)
{
    MI_SYS_PixelFormat_e ePixel = E_MI_SYS_PIXEL_FRAME_FORMAT_MAX;
    printf("GetIniPixel:  %s\n",pPixeString);
    //ERR
    if(!strcmp(pPixeString, "ERR"))
    {
        UTStatus = UT_CASE_FAIL;
        DBG_ERR("GetIniPixel error\n");
        goto EXIT;
    }
    if(!strcmp(pPixeString, "MAX"))
    {
        goto EXIT;
    }

    //YUV
    if(!strcmp(pPixeString, "YUV420SPNV12"))
    {
        ePixel = E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420;
        goto EXIT;
    }
    if(!strcmp(pPixeString, "YUV420SPNV21"))
    {
        ePixel = E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420_NV21;
        goto EXIT;
    }
    if(!strcmp(pPixeString, "YUV420P"))
    {
        ePixel = E_MI_SYS_PIXEL_FRAME_YUV420_PLANAR;
        goto EXIT;
    }
    if(!strcmp(pPixeString, "YUV422YUYV"))
    {
        ePixel = E_MI_SYS_PIXEL_FRAME_YUV422_YUYV;
        goto EXIT;
    }
    if(!strcmp(pPixeString, "YUV422UYVY"))
    {
        ePixel = E_MI_SYS_PIXEL_FRAME_YUV422_UYVY;
        goto EXIT;
    }
    if(!strcmp(pPixeString, "YUV422YVYU"))
    {
        ePixel = E_MI_SYS_PIXEL_FRAME_YUV422_YVYU;
        goto EXIT;
    }
    if(!strcmp(pPixeString, "YUV422VYUY"))
    {
        ePixel = E_MI_SYS_PIXEL_FRAME_YUV422_VYUY;
        goto EXIT;
    }
    if(!strcmp(pPixeString, "YUV422P"))
    {
        ePixel = E_MI_SYS_PIXEL_FRAME_YUV422_PLANAR;
        goto EXIT;
    }
    if(!strcmp(pPixeString, "YUV422SP"))
    {
        ePixel = E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_422;
        goto EXIT;
    }

    //ARGB
    if(!strcmp(pPixeString, "ARGB8888"))
    {
        ePixel = E_MI_SYS_PIXEL_FRAME_ARGB8888;
        goto EXIT;
    }
    if(!strcmp(pPixeString, "ABGR8888"))
    {
        ePixel = E_MI_SYS_PIXEL_FRAME_ABGR8888;
        goto EXIT;
    }
    if(!strcmp(pPixeString, "BGRA8888"))
    {
        ePixel = E_MI_SYS_PIXEL_FRAME_BGRA8888;
        goto EXIT;
    }
    if(!strcmp(pPixeString, "RGB101010"))
    {
        ePixel = E_MI_SYS_PIXEL_FRAME_RGB101010;
        goto EXIT;
    }

    //Bayer
    if(!strcmp(pPixeString, "RG8B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_8BPP,E_MI_SYS_PIXEL_BAYERID_RG);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "GR8B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_8BPP,E_MI_SYS_PIXEL_BAYERID_GR);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "BG8B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_8BPP,E_MI_SYS_PIXEL_BAYERID_BG);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "GB8B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_8BPP,E_MI_SYS_PIXEL_BAYERID_GB);
        goto EXIT;
    }

    if(!strcmp(pPixeString, "RG10B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_10BPP,E_MI_SYS_PIXEL_BAYERID_RG);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "GR10B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_10BPP,E_MI_SYS_PIXEL_BAYERID_GR);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "BG10B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_10BPP,E_MI_SYS_PIXEL_BAYERID_BG);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "GB10B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_10BPP,E_MI_SYS_PIXEL_BAYERID_GB);
        goto EXIT;
    }

    if(!strcmp(pPixeString, "RG12B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_12BPP,E_MI_SYS_PIXEL_BAYERID_RG);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "GR12B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_12BPP,E_MI_SYS_PIXEL_BAYERID_GR);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "BG12B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_12BPP,E_MI_SYS_PIXEL_BAYERID_BG);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "GB12B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_12BPP,E_MI_SYS_PIXEL_BAYERID_GB);
        goto EXIT;
    }

    if(!strcmp(pPixeString, "RG14B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_14BPP,E_MI_SYS_PIXEL_BAYERID_RG);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "GR14B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_14BPP,E_MI_SYS_PIXEL_BAYERID_GR);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "BG14B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_14BPP,E_MI_SYS_PIXEL_BAYERID_BG);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "GB14B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_14BPP,E_MI_SYS_PIXEL_BAYERID_GB);
        goto EXIT;
    }

    if(!strcmp(pPixeString, "RG16B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_16BPP,E_MI_SYS_PIXEL_BAYERID_RG);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "GR16B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_16BPP,E_MI_SYS_PIXEL_BAYERID_GR);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "BG16B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_16BPP,E_MI_SYS_PIXEL_BAYERID_BG);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "GB16B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_16BPP,E_MI_SYS_PIXEL_BAYERID_GB);
        goto EXIT;
    }

    //IR8BPP
    if(!strcmp(pPixeString, "R08B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_8BPP,E_MI_SYS_PIXEL_RGBIR_R0);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "G08B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_8BPP,E_MI_SYS_PIXEL_RGBIR_G0);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "B08B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_8BPP,E_MI_SYS_PIXEL_RGBIR_B0);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "G18B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_8BPP,E_MI_SYS_PIXEL_RGBIR_G1);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "G28B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_8BPP,E_MI_SYS_PIXEL_RGBIR_G2);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "I08B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_8BPP,E_MI_SYS_PIXEL_RGBIR_I0);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "G38B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_8BPP,E_MI_SYS_PIXEL_RGBIR_G3);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "I18B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_8BPP,E_MI_SYS_PIXEL_RGBIR_I1);
        goto EXIT;
    }

    //IR10BPP
    if(!strcmp(pPixeString, "R010B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_10BPP,E_MI_SYS_PIXEL_RGBIR_R0);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "G010B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_10BPP,E_MI_SYS_PIXEL_RGBIR_G0);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "B010B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_10BPP,E_MI_SYS_PIXEL_RGBIR_B0);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "G110B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_10BPP,E_MI_SYS_PIXEL_RGBIR_G1);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "G210B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_10BPP,E_MI_SYS_PIXEL_RGBIR_G2);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "I010B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_10BPP,E_MI_SYS_PIXEL_RGBIR_I0);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "G310B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_10BPP,E_MI_SYS_PIXEL_RGBIR_G3);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "I110B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_10BPP,E_MI_SYS_PIXEL_RGBIR_I1);
        goto EXIT;
    }

    //IR12BPP
    if(!strcmp(pPixeString, "R012B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_12BPP,E_MI_SYS_PIXEL_RGBIR_R0);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "G012B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_12BPP,E_MI_SYS_PIXEL_RGBIR_G0);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "B012B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_12BPP,E_MI_SYS_PIXEL_RGBIR_B0);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "G112B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_12BPP,E_MI_SYS_PIXEL_RGBIR_G1);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "G212B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_12BPP,E_MI_SYS_PIXEL_RGBIR_G2);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "I012B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_12BPP,E_MI_SYS_PIXEL_RGBIR_I0);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "G312B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_12BPP,E_MI_SYS_PIXEL_RGBIR_G3);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "I112B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_12BPP,E_MI_SYS_PIXEL_RGBIR_I1);
        goto EXIT;
    }

    //IR14BPP
    if(!strcmp(pPixeString, "R014B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_14BPP,E_MI_SYS_PIXEL_RGBIR_R0);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "G014B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_14BPP,E_MI_SYS_PIXEL_RGBIR_G0);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "B014B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_14BPP,E_MI_SYS_PIXEL_RGBIR_B0);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "G114B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_14BPP,E_MI_SYS_PIXEL_RGBIR_G1);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "G214B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_14BPP,E_MI_SYS_PIXEL_RGBIR_G2);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "I014B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_14BPP,E_MI_SYS_PIXEL_RGBIR_I0);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "G314B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_14BPP,E_MI_SYS_PIXEL_RGBIR_G3);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "I114B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_14BPP,E_MI_SYS_PIXEL_RGBIR_I1);
        goto EXIT;
    }

    //IR16BPP
    if(!strcmp(pPixeString, "R016B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_16BPP,E_MI_SYS_PIXEL_RGBIR_R0);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "G016B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_16BPP,E_MI_SYS_PIXEL_RGBIR_G0);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "B016B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_16BPP,E_MI_SYS_PIXEL_RGBIR_B0);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "G116B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_16BPP,E_MI_SYS_PIXEL_RGBIR_G1);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "G216B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_16BPP,E_MI_SYS_PIXEL_RGBIR_G2);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "I016B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_16BPP,E_MI_SYS_PIXEL_RGBIR_I0);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "G316B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_16BPP,E_MI_SYS_PIXEL_RGBIR_G3);
        goto EXIT;
    }
    if(!strcmp(pPixeString, "I116B"))
    {
        ePixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_16BPP,E_MI_SYS_PIXEL_RGBIR_I1);
        goto EXIT;
    }

EXIT:
    return ePixel;
}

MI_S32 ST_ParserIni(char *pIniPath)
{
    MI_S32 s32Ret = MI_SUCCESS;
    MI_SNR_PADID eSnrPad = 0;
    ST_Sensor_Attr_t *pstSensorAttr = NULL;

    MI_U32  u32IspDevId =0;
    MI_U32  u32IspChnId =0;
    ST_IspDevAttr_t *pstIspDevAttr = NULL;
    ST_IspChannelAttr_t *pstIspChnAttr = NULL;

    ST_VifDevAttr_t *pstVifDevAttr = NULL;
    ST_VifGroupAttr_t *pstVifGroupAttr = NULL;
    ST_VifPortAttr_t *pstVifPort = NULL;
    MI_U8 u8VifGroupId =0;
    MI_U32 u32VifDev = 0;
    MI_VIF_PORT vifPort = 0;
    MI_U8 vifDevPerGroup = 0;

    MI_U32 u32JpdChn = 0;
    MI_BOOL bJpdUse=FALSE;
    MI_SYS_BindType_e eJpdBindType=E_MI_SYS_BIND_TYPE_FRAME_BASE;
    char *string= NULL;

    char SclDevString[][64] = {"SclIspRealtime0","SclRdma0","SclYUVRealtime0","SclRotRdma","SclIspRealtime1","SclRdma2","SclYUVRealtime1"};
    MI_U32  u32SclDevId =0;
    MI_U32  u32SclSrcId =0;

    dictionary *pstDict = iniparser_load(pIniPath);

    if(pstDict == NULL)
        return -1;

    printf("pstDict %p \n", pstDict);
	

    
    u32TimeOutSExit = (MI_U32)iniparser_getint(pstDict, ":ExitTimeOutS", 0);

    if(E_ST_MD5_ACTION_NONE == gMd5Action)
    {
        string = iniparser_getstring(pstDict, ":Md5Action", (char *)"ERR");
        if(!strcmp(string, "reset"))
        {
            gMd5Action = E_ST_MD5_ACTION_RESET;
        }
        else if(!strcmp(string, "add"))
        {
            gMd5Action = E_ST_MD5_ACTION_ADD;
        }
        else
        {
            gMd5Action = E_ST_MD5_ACTION_NONE;
        }
        printf("gMd5Action %d\n", gMd5Action);
    }

    eSnrPad = (MI_SNR_PADID)iniparser_getint(pstDict, ":SensorPad", -1);
    if(eSnrPad < ST_MAX_SENSOR_NUM)
    {
        pstSensorAttr = &gstSensorAttr[eSnrPad];
        pstSensorAttr->u8ResIndex = iniparser_getint(pstDict, ":ResIndex", -1);
        pstSensorAttr->ADIndex = iniparser_getint(pstDict, ":ADIndex",0xFF);
        pstSensorAttr->bUsed = TRUE;
		
		bUseDisp |= iniparser_getint(pstDict, ":testUseDisp",0);
		btest_1080 |= iniparser_getint(pstDict, ":test1080",0);
		TEST_ISP_MULTI_DEV |= iniparser_getint(pstDict, ":test_MULTI_DEV",0);

        bUseUserSensor |= iniparser_getint(pstDict, ":usersensor",0);
        bDetectAD |= iniparser_getint(pstDict, ":DetectAD",0);
        if(bDetectAD !=0)
            bUseUserSensor=TRUE;

        string = iniparser_getstring(pstDict, ":userintface", (char *)"BT656");
        if(!strcmp(string, "BT656"))
        {
            pstSensorAttr->eUserSensorIntf = CUS_SENIF_BUS_BT656;
        }
        else if(!strcmp(string, "BT1120"))
        {
            pstSensorAttr->eUserSensorIntf = CUS_SENIF_BUS_BT1120;
        }
        else
        {
            printf("userSensorIntface parse %s err, now only support BT656/BT1120, default use BT656 \n", string);
            pstSensorAttr->eUserSensorIntf = CUS_SENIF_BUS_BT656;
        }

        u32VifDev = iniparser_getint(pstDict, ":VifDev", 0);
        if(u32VifDev >= ST_MAX_VIF_DEV_NUM)
        {
            UTStatus = UT_CASE_FAIL;
            printf("VIF u32VifDev %d err > max %d \n", u32VifDev, ST_MAX_VIF_DEV_NUM);
            s32Ret = -1;
            goto EXIT;
        }

        u8VifGroupId = u32VifDev/ST_MAX_VIF_DEV_PERGROUP;
        vifDevPerGroup = u32VifDev%ST_MAX_VIF_DEV_PERGROUP;
        if(u8VifGroupId >= ST_MAX_VIF_GROUP_NUM)
        {
            UTStatus = UT_CASE_FAIL;
            printf("VIF u8VifGroupId %d err > max %d \n", u8VifGroupId, ST_MAX_VIF_GROUP_NUM);
            s32Ret = -1;
            goto EXIT;
        }

        printf(">>>>>>>>>>>>>>>dev %d groupid %d \n",u32VifDev, u8VifGroupId);
        pstVifGroupAttr = &gstVifModule.stVifGroupAttr[u8VifGroupId];
        //got vif channel can find the binded SensorPad
        pstVifGroupAttr->bUsed = pstSensorAttr->bUsed;
        pstVifGroupAttr->eHDRType=(MI_VIF_HDRType_e)iniparser_getint(pstDict, ":HDR", 0);
        vifPort = (MI_VIF_PORT)iniparser_getint(pstDict, ":VifOutPortId", ST_MAX_VIF_OUTPORT_NUM);
        vifPort = (vifPort==ST_MAX_VIF_OUTPORT_NUM)?0: vifPort;
        if(vifPort>= ST_MAX_VIF_OUTPORT_NUM)
        {
            UTStatus = UT_CASE_FAIL;
            printf("VIF port %d err > max %d \n", vifPort, ST_MAX_VIF_OUTPORT_NUM);
            s32Ret = -1;
            goto EXIT;
        }

        pstVifGroupAttr->stBindSensor.eSensorPadID = (MI_VIF_SNRPad_e)eSnrPad;
        memset(&pstVifGroupAttr->stBindSensor.u32PlaneID, 0x0, sizeof(MI_U32)*MI_VIF_MAX_GROUP_DEV_CNT);

        pstVifDevAttr = &pstVifGroupAttr->stVifDevAttr[vifDevPerGroup];
        pstVifPort = &pstVifDevAttr->stVifOutPortAttr[vifPort];
        pstVifDevAttr->bUsed = pstVifGroupAttr->bUsed;
        pstVifPort->bUsed = pstVifDevAttr->bUsed;
        pstVifDevAttr->stVifDevAttr.bEnH2T1PMode= (MI_BOOL)iniparser_getint(pstDict, ":H2T1P", 0);
        pstVifDevAttr->stVifDevAttr.eField = (MI_SYS_FieldType_e)iniparser_getint(pstDict, ":Field", 0);
        pstVifDevAttr->stVifDevAttr.stInputRect.u16X = iniparser_getint(pstDict,":VifInputRectX",0);
        pstVifDevAttr->stVifDevAttr.stInputRect.u16Y = iniparser_getint(pstDict,":VifInputRectY",0);
        pstVifDevAttr->stVifDevAttr.stInputRect.u16Width = iniparser_getint(pstDict,":VifInputRectW",0);
        pstVifDevAttr->stVifDevAttr.stInputRect.u16Height = iniparser_getint(pstDict,":VifInputRectH",0);

        string = iniparser_getstring(pstDict, ":VifDevpixel", (char *)"MAX");
        pstVifDevAttr->stVifDevAttr.eInputPixel = ST_GetIniPixel(string);

        string = iniparser_getstring(pstDict, ":RunMode", (char *)"ERR");
        if(!strcmp(string, "1Multi"))
        {
            pstVifGroupAttr->eWorkMode = E_MI_VIF_WORK_MODE_1MULTIPLEX;
        }
        else if(!strcmp(string, "2Multi"))
        {
            pstVifGroupAttr->eWorkMode = E_MI_VIF_WORK_MODE_2MULTIPLEX;
        }
        else if(!strcmp(string, "4Multi"))
        {
            pstVifGroupAttr->eWorkMode = E_MI_VIF_WORK_MODE_4MULTIPLEX;
        }
        else
        {
            printf("runmode parse %s err, please use RealTime/FrameMode/1Multi/2Multi/4Multi. default use RealTime \n", string);
            pstVifGroupAttr->eWorkMode = E_MI_VIF_WORK_MODE_1MULTIPLEX;
        }

        printf("VifDev %d, H2T1P %d, field %d Vifpixel %d\n", u32VifDev, pstVifDevAttr->stVifDevAttr.bEnH2T1PMode, pstVifDevAttr->stVifDevAttr.eField,pstVifDevAttr->stVifDevAttr.eInputPixel);

        string = iniparser_getstring(pstDict, ":Vifpixel", (char *)"MAX");
        pstVifPort->ePixFormat = ST_GetIniPixel(string);
        pstVifPort->stCapRect.u16X = iniparser_getint(pstDict,":VifPortCropX",0);
        pstVifPort->stCapRect.u16Y = iniparser_getint(pstDict,":VifPortCropY",0);
        pstVifPort->stCapRect.u16Width = iniparser_getint(pstDict,":VifPortCropW",0);
        pstVifPort->stCapRect.u16Height = iniparser_getint(pstDict,":VifPortCropH",0);

        pstVifPort->stDestSize.u16Width = iniparser_getint(pstDict,":VifPortW",0);
        pstVifPort->stDestSize.u16Height = iniparser_getint(pstDict,":VifPortH",0);

        pstVifPort->eFrameRate = (MI_VIF_FrameRate_e)iniparser_getint(pstDict,":VifFr",E_MI_VIF_FRAMERATE_FULL);
        pstVifPort->eCompressMode = (MI_SYS_CompressMode_e)iniparser_getint(pstDict,":VifCompress",E_MI_SYS_COMPRESS_MODE_NONE);
        pstVifPort->stoutFileAttr.u16UserDepth = iniparser_getint(pstDict, ":VifPortUserDepth", 0);
        pstVifPort->stoutFileAttr.u16Depth = iniparser_getint(pstDict, ":VifPortDepth", 4);
        pstVifPort->stoutFileAttr.bNeedFbd = iniparser_getint(pstDict, ":NeedFbd", 1);

        pstVifPort->stoutFileAttr.s32DumpBuffNum = iniparser_getint(pstDict, ":VifPortDumpBuffNum", 0);
        string = iniparser_getstring(pstDict, ":VifPortOutPutPath", (char *)"NULL");
        if(!strcmp((const char *)string, (const char *)"NULL"))
        {
            printf("OutPutPath NULL \n");
        }
        else
        {
            sprintf(pstVifPort->stoutFileAttr.FilePath, "%s/", string);
            printf("OutPutFile_Path:%s \n",pstVifPort->stoutFileAttr.FilePath);

            /***********************************************
            Mkdir path here to avoid running at the same time
            when Multi-threading and Multi-ports dumpfile
            ************************************************/
            s32Ret = ST_CheckMkdirOutFile(pstVifPort->stoutFileAttr.FilePath);
            if(s32Ret != MI_SUCCESS)
            {
                goto EXIT;
            }
        }

        pstSensorAttr->u32BindVifDev = u32VifDev;
        if(E_MI_VIF_HDR_TYPE_OFF== pstVifGroupAttr->eHDRType
          || E_MI_VIF_HDR_TYPE_EMBEDDED == pstVifGroupAttr->eHDRType
          || E_MI_VIF_HDR_TYPE_LI== pstVifGroupAttr->eHDRType)
        {
            pstSensorAttr->bPlaneMode = TRUE;
        }

        INIT_LIST_HEAD(&pstVifPort->head.pos);
    }

    u32JpdChn = iniparser_getint(pstDict,":JpdChn",0xffff);
    if(u32JpdChn < ST_MAX_JPD_CHN_NUM)
    {
        ST_JpdChnAttr_t *pstJpdChnAttr = &gstJpdModeAttr.stJpdChnAttr[u32JpdChn];
        bJpdUse = TRUE;
        string = iniparser_getstring(pstDict, ":JpdFilePath", (char *)"ERR");
        if(!strcmp((const char *)string, (const char *)"NULL"))
        {
            //printf("YuvFilePath NULL \n");
        }
        else if(!strcmp((const char *)string, (const char *)"ERR"))
        {
            UTStatus = UT_CASE_FAIL;
            printf("kerword JpdFilePath use err \n");
        }
        else
        {
            MI_U16  u16size = strlen(string);
            memcpy(pstJpdChnAttr->InputFilePath, string, u16size);
            printf("pstInputFileAttr:%s \n",pstJpdChnAttr->InputFilePath);
            pstJpdChnAttr->u32ChnId = u32JpdChn;
            pstJpdChnAttr->bUsed = TRUE;

            struct stat statbuff;
            memset(&statbuff, 0, sizeof(struct stat));
            if(stat(pstJpdChnAttr->InputFilePath, &statbuff) < 0)
            {
                UTStatus = UT_CASE_FAIL;
                ST_ERR("JPD file not exit!\n");
                s32Ret = -1;
                goto EXIT;
            }
            else
            {
                if (statbuff.st_size == 0)
                {
                    UTStatus = UT_CASE_FAIL;
                    ST_ERR("File size is zero!\n");
                    s32Ret = -1;
                    goto EXIT;
                }
                pstJpdChnAttr->u32FrameBuffSize = statbuff.st_size;
            }

            eJpdBindType =(MI_SYS_BindType_e)iniparser_getint(pstDict, ":JpdBindType", E_MI_SYS_BIND_TYPE_FRAME_BASE);
        }

        pstJpdChnAttr->u32FrameRate= iniparser_getint(pstDict,":JpdFr",30);
        pstJpdChnAttr->u32MaxW = 3840;
        pstJpdChnAttr->u32MaxH = 2160;

        INIT_LIST_HEAD(&pstJpdChnAttr->head.pos);
    }

    u32IspDevId = iniparser_getint(pstDict,":IspDevId",ST_MAX_ISP_DEV_NUM);
    if(u32IspDevId < ST_MAX_ISP_DEV_NUM)
    {
        pstIspDevAttr = &gstIspModule.stIspDevAttr[u32IspDevId];
        pstIspDevAttr->bUsed = TRUE;

        u32IspChnId = iniparser_getint(pstDict,":IspChnId",ST_MAX_ISP_CHN_NUM);
        if(u32IspChnId == ST_MAX_ISP_CHN_NUM)
        {
            for(u32IspChnId =0; u32IspChnId < ST_MAX_ISP_CHN_NUM; u32IspChnId++)
            {
                pstIspChnAttr = &pstIspDevAttr->stIspChnlAttr[u32IspChnId];
                if(pstIspChnAttr->bUsed == FALSE)
                {
                    break;
                }
            }
        }

        if(u32IspChnId == ST_MAX_ISP_CHN_NUM)
        {
            printf("[%s]%d find ispchn fail max %d all use \n", __FUNCTION__,__LINE__, ST_MAX_ISP_CHN_NUM);
            s32Ret = -1;
            goto EXIT;
        }
        pstIspChnAttr = &pstIspDevAttr->stIspChnlAttr[u32IspChnId];

        pstIspChnAttr->u8ChnId = u32IspChnId;
        pstIspChnAttr->bUsed = TRUE;
        pstIspChnAttr->stIspInPortAttr[0].bUsed = TRUE;

        pstIspChnAttr->stIspInPortAttr[0].stInputCropWin.u16X = iniparser_getint(pstDict, ":IspInCropX", 0);
        pstIspChnAttr->stIspInPortAttr[0].stInputCropWin.u16Y = iniparser_getint(pstDict, ":IspInCropY", 0);
        pstIspChnAttr->stIspInPortAttr[0].stInputCropWin.u16Width = iniparser_getint(pstDict, ":IspInCropW", 0);
        pstIspChnAttr->stIspInPortAttr[0].stInputCropWin.u16Height = iniparser_getint(pstDict, ":IspInCropH", 0);

        string = iniparser_getstring(pstDict, ":IspInFilePath", (char *)"NULL");
        if(!strcmp((const char *)string, (const char *)"NULL"))
        {
            MI_S32 s32IspInputFrameRate = iniparser_getint(pstDict, ":IspInFr",0xff);

            string = iniparser_getstring(pstDict, ":IspbindType", (char *)"NULL");
            if(!strcmp((const char *)string, (const char *)"Realtime"))
            {
                pstIspChnAttr->stIspInPortAttr[0].stBindParam.eBindType = E_MI_SYS_BIND_TYPE_REALTIME;
            }
            else if(!strcmp((const char *)string, (const char *)"Frame"))
            {
                pstIspChnAttr->stIspInPortAttr[0].stBindParam.eBindType = E_MI_SYS_BIND_TYPE_FRAME_BASE;
            }
            else
            {
                UTStatus = UT_CASE_FAIL;
                printf("[%s]%d isp input bindtype err %s \n", __FUNCTION__,__LINE__, string);
                s32Ret = -1;
                goto EXIT;
            }

            pstIspChnAttr->stIspInPortAttr[0].stBindParam.stChnPort.eModId = E_MI_MODULE_ID_VIF;
            pstIspChnAttr->stIspInPortAttr[0].stBindParam.stChnPort.u32DevId = u32VifDev;
            pstIspChnAttr->stIspInPortAttr[0].stBindParam.stChnPort.u32ChnId = 0;
            pstIspChnAttr->stIspInPortAttr[0].stBindParam.stChnPort.u32PortId = vifPort;
            pstIspChnAttr->stIspInPortAttr[0].stBindParam.u32SrcFrmrate = 30;
            pstIspChnAttr->stIspInPortAttr[0].stBindParam.u32DstFrmrate = s32IspInputFrameRate;
        }
        else
        {
            MI_U16  u16size = strlen(string);
            memcpy(pstIspChnAttr->stIspInPortAttr[0].stInputFileAttr.InputFilePath, string, u16size);
            printf("pstInputFileAttr:%s \n",pstIspChnAttr->stIspInPortAttr[0].stInputFileAttr.InputFilePath);

            pstIspChnAttr->stIspInPortAttr[0].stInputFileAttr.u32Width = iniparser_getint(pstDict, ":IspInFileW", 0);
            pstIspChnAttr->stIspInPortAttr[0].stInputFileAttr.u32Height = iniparser_getint(pstDict, ":IspInFileH", 0);
            string = iniparser_getstring(pstDict, ":IspInFilePixel", (char *)"ERR");
            pstIspChnAttr->stIspInPortAttr[0].stInputFileAttr.ePixelFormat = ST_GetIniPixel(string);
            pstIspChnAttr->stIspInPortAttr[0].stInputFileAttr.eCompress = (MI_SYS_CompressMode_e)iniparser_getint(pstDict, ":IspInCompress", 0);
            pstIspChnAttr->stIspInPortAttr[0].stInputFileAttr.bCrcCheck = (MI_BOOL)iniparser_getint(pstDict, ":IspInCrc", 0);
            pstIspChnAttr->stIspInPortAttr[0].stInputFileAttr.u32SleepMs = iniparser_getint(pstDict, ":IspInFileSleepMs", 0);

            pstIspChnAttr->stIspInPortAttr[0].stInputFileAttr.stContentCropWindow.u16X = iniparser_getint(pstDict, ":IspContentCropX", 0);
            pstIspChnAttr->stIspInPortAttr[0].stInputFileAttr.stContentCropWindow.u16Y = iniparser_getint(pstDict, ":IspContentCropY", 0);
            pstIspChnAttr->stIspInPortAttr[0].stInputFileAttr.stContentCropWindow.u16Width = iniparser_getint(pstDict, ":IspContentCropW", 0);
            pstIspChnAttr->stIspInPortAttr[0].stInputFileAttr.stContentCropWindow.u16Height = iniparser_getint(pstDict, ":IspContentCropH", 0);

            pstIspChnAttr->stIspInPortAttr[0].stBindParam.stChnPort.eModId = E_MI_MODULE_ID_MAX;
        }

        if(pstVifGroupAttr !=NULL)
            pstIspChnAttr->stIspChnParam.eHDRType = (MI_ISP_HDRType_e)pstVifGroupAttr->eHDRType;
        else
            pstIspChnAttr->stIspChnParam.eHDRType = E_MI_ISP_HDR_TYPE_OFF;

        pstIspChnAttr->stIspChnParam.e3DNRLevel = (MI_ISP_3DNR_Level_e)iniparser_getint(pstDict, ":NRLevel", 0);
        pstIspChnAttr->stIspChnParam.eRot = (MI_SYS_Rotate_e)iniparser_getint(pstDict, ":IspRotation", 0);
        pstIspChnAttr->stIspChnParam.bMirror = (MI_SYS_Rotate_e)iniparser_getint(pstDict, ":IspMirror", 0);
        pstIspChnAttr->stIspChnParam.bFlip = (MI_SYS_Rotate_e)iniparser_getint(pstDict, ":IspFlip", 0);
        pstIspChnAttr->eBindMiSnrPadId = eSnrPad;
        string = iniparser_getstring(pstDict, ":IqBinPath", (char *)"ERR");
        if(!strcmp((const char *)string, (const char *)"NULL"))
        {
            printf("IQ Bin Path NULL \n");
        }
        else
        {
            MI_U16 u16size = strlen(string);
            memcpy(pstIspChnAttr->IqCfgbin_Path,string,u16size);
            printf("IqBinPath:%s \n",pstIspChnAttr->IqCfgbin_Path);
        }

        pstIspChnAttr->stIspChnParam.bY2bEnable = iniparser_getint(pstDict, ":IspYuv2Bayer", 0);

        pstIspChnAttr->stIspCustSegAttr.eMode = (MI_ISP_CustSegMode_e)iniparser_getint(pstDict, ":IspCustMode", 0);
        if(pstIspChnAttr->stIspCustSegAttr.eMode > E_MI_ISP_CUST_SEG_MODE_NONE &&
           pstIspChnAttr->stIspCustSegAttr.eMode < E_MI_ISP_CUST_SEG_MODE_MAX)
        {
            //ai isp open
            pstIspChnAttr->stIspCustSegAttr.eFrom = (MI_ISP_InternalSeg_e)iniparser_getint(pstDict, ":IspCustFrom", 0);
            pstIspChnAttr->stIspCustSegAttr.eTo = (MI_ISP_InternalSeg_e)iniparser_getint(pstDict, ":IspCustTo", 0);
            string = iniparser_getstring(pstDict, ":IspCustInPixel", (char *)"ERR");
            pstIspChnAttr->stIspCustSegAttr.stInputParam.ePixelFormat = ST_GetIniPixel(string);
            string = iniparser_getstring(pstDict, ":IspCustOutPixel", (char *)"ERR");
            pstIspChnAttr->stIspCustSegAttr.stOutputParam.ePixelFormat = ST_GetIniPixel(string);

#if MI_IPU_SUPPORT
            string = iniparser_getstring(pstDict, ":IspIpuImagePath", (char *)"NULL");
            if(!strcmp((const char *)string, (const char *)"NULL"))
            {
                printf("IspIpuImagePath NULL \n");
            }
            else
            {
                MI_U16 u16size = strlen(string);
                pstIspChnAttr->bUseIpu = TRUE;
                memcpy(pstIspChnAttr->aIpuImagePath,string,u16size);
                printf("IspIpuImagePath:%s \n",pstIspChnAttr->aIpuImagePath);
            }
#endif
        }

        pstIspChnAttr->stIspOutPortAttr[0].bUsed = iniparser_getint(pstDict, ":Ispport0Use", 0);
        pstIspChnAttr->stIspOutPortAttr[0].stoutFileAttr.u16Depth = 4;
        pstIspChnAttr->stIspOutPortAttr[1].bUsed = iniparser_getint(pstDict, ":Ispport1Use", 0);
        pstIspChnAttr->stIspOutPortAttr[2].bUsed = iniparser_getint(pstDict, ":Ispport2Use", 0);

        if(pstIspChnAttr->stIspOutPortAttr[0].bUsed == TRUE)
        {
            INIT_LIST_HEAD(&pstIspChnAttr->stIspOutPortAttr[0].head.pos);
        }

        if(pstIspChnAttr->stIspOutPortAttr[1].bUsed == TRUE)
        {
            MI_VENC_ModType_e  eEncType = E_MI_VENC_MODTYPE_VENC;
            pstIspChnAttr->stIspOutPortAttr[1].stOrigPortCrop.u16X = iniparser_getint(pstDict, ":Ispport1PortCropX", 0);
            pstIspChnAttr->stIspOutPortAttr[1].stOrigPortCrop.u16Y = iniparser_getint(pstDict, ":Ispport1PortCropY", 0);
            pstIspChnAttr->stIspOutPortAttr[1].stOrigPortCrop.u16Width = iniparser_getint(pstDict, ":Ispport1PortCropW", 0);
            pstIspChnAttr->stIspOutPortAttr[1].stOrigPortCrop.u16Height = iniparser_getint(pstDict, ":Ispport1PortCropH", 0);
            string = iniparser_getstring(pstDict, ":Ispport1Pixel", (char *)"ERR");
            pstIspChnAttr->stIspOutPortAttr[1].ePixelFormat = ST_GetIniPixel(string);
            pstIspChnAttr->stIspOutPortAttr[1].eCompressMode = (MI_SYS_CompressMode_e)iniparser_getint(pstDict, ":Ispport1Compress", 0);

            pstIspChnAttr->stIspOutPortAttr[1].stoutFileAttr.u16UserDepth = iniparser_getint(pstDict, ":Ispport1userdepth", 0);
            pstIspChnAttr->stIspOutPortAttr[1].stoutFileAttr.u16Depth = iniparser_getint(pstDict, ":Ispport1depth", 4);

            pstIspChnAttr->stIspOutPortAttr[1].stoutFileAttr.s32DumpBuffNum = iniparser_getint(pstDict, ":Ispport1DumpBuffNum", 0);

            INIT_LIST_HEAD(&pstIspChnAttr->stIspOutPortAttr[1].head.pos);

            string = iniparser_getstring(pstDict, ":Ispport1OutPutPath", (char *)"NULL");
            if(!strcmp((const char *)string, (const char *)"NULL"))
            {
                printf("OutPutPath NULL \n");
            }
            else
            {
                sprintf(pstIspChnAttr->stIspOutPortAttr[1].stoutFileAttr.FilePath, "%s/", string);
                printf("OutPutFile_Path:%s \n",pstIspChnAttr->stIspOutPortAttr[1].stoutFileAttr.FilePath);

                /***********************************************
                Mkdir path here to avoid running at the same time
                when Multi-threading and Multi-ports dumpfile
                ************************************************/
                s32Ret = ST_CheckMkdirOutFile(pstIspChnAttr->stIspOutPortAttr[1].stoutFileAttr.FilePath);
                if(s32Ret != MI_SUCCESS)
                {
                    goto EXIT;
                }
            }

            eEncType = (MI_VENC_ModType_e)iniparser_getint(pstDict, ":Ispport1EncodeType", E_MI_VENC_MODTYPE_VENC);
            MI_U32 u32EncFps = iniparser_getint(pstDict, ":Ispport1EncodeFps", 30);
            MI_U32 u32BindVencId = iniparser_getint(pstDict, ":Ispport1BindVencId", 0);
            ST_BindParam_t stVencInBindParam;
            stVencInBindParam.eBindType = E_MI_SYS_BIND_TYPE_FRAME_BASE;
            stVencInBindParam.stChnPort.eModId = E_MI_MODULE_ID_ISP;
            stVencInBindParam.stChnPort.u32DevId = u32IspDevId;
            stVencInBindParam.stChnPort.u32ChnId = u32IspChnId;
            stVencInBindParam.stChnPort.u32PortId = 1;
            stVencInBindParam.u32SrcFrmrate = 30;
            stVencInBindParam.u32DstFrmrate = 30;
            //ST_IniSetVencParam(u32BindVencId,eEncType, &stVencInBindParam, u32EncFps);

            char **array_str_holder=NULL;
            int array_cnt=0;
            char *holder = NULL;

            pstIspChnAttr->stIspOutPortAttr[1].stoutFileAttr.stMd5Attr.pu8IniPath = (MI_U8*)pIniPath;
            strcpy(pstIspChnAttr->stIspOutPortAttr[1].stoutFileAttr.stMd5Attr.key, ":Ispport1Md5");
            pstIspChnAttr->stIspOutPortAttr[1].stoutFileAttr.stMd5Attr.eMd5Action = gMd5Action;
            string = iniparser_getstring(pstDict, ":Ispport1Md5", NULL);
            if(string != NULL)
            {
                printf(">>>>>>>>>>>Ispport1Md5 string %s \n", string);
                holder=iniparser_parser_string_to_array(string, &array_str_holder, &array_cnt);
                if(holder != NULL)
                {
                    MI_U8 i=0, j=0;
                    //pstIspChnAttr->stIspOutPortAttr[1].stoutFileAttr.bNeedCheckMd5 = TRUE;
                    pstIspChnAttr->stIspOutPortAttr[1].stoutFileAttr.stMd5Attr.eMd5Action = gMd5Action > E_ST_MD5_ACTION_NONE ?
                                                                                        gMd5Action : E_ST_MD5_ACTION_CHECK;
                    for(i=0; i<array_cnt; i++)
                    {
                        MI_U16  u16size = strlen(array_str_holder[i]);
                        printf("MD5 size %d, value %s\n", u16size, array_str_holder[i]);
                        for(j=0; j<16; j++)
                        {
                            pstIspChnAttr->stIspOutPortAttr[1].stoutFileAttr.stMd5Attr.stMD5ExpectValue[i].u8MD5ExpectValue[j]=string_to_int8((MI_U8 *)&array_str_holder[i][j<<1]);
                        }
                        pstIspChnAttr->stIspOutPortAttr[1].stoutFileAttr.stMd5Attr.stMD5ExpectValue[i].bUsed = TRUE;

                        for(j=0; j<16; j++)
                        {
                            printf("%02x", pstIspChnAttr->stIspOutPortAttr[1].stoutFileAttr.stMd5Attr.stMD5ExpectValue[i].u8MD5ExpectValue[j]);
                        }
                        printf("\n");
                    }

                    free(array_str_holder);
                    array_str_holder = NULL;
                    free(holder);
                    holder = NULL;
                }
            }
        }

        if(pstIspChnAttr->stIspOutPortAttr[2].bUsed == TRUE)
        {
            MI_VENC_ModType_e  eEncType = E_MI_VENC_MODTYPE_VENC;
            pstIspChnAttr->stIspOutPortAttr[2].stoutFileAttr.u16UserDepth = iniparser_getint(pstDict, ":Ispport2userdepth", 0);
            pstIspChnAttr->stIspOutPortAttr[2].stoutFileAttr.u16Depth = iniparser_getint(pstDict, ":Ispport2depth", 4);

            eEncType = (MI_VENC_ModType_e)iniparser_getint(pstDict, ":Ispport2EncodeType", E_MI_VENC_MODTYPE_VENC);
            MI_U32 u32EncFps = iniparser_getint(pstDict, ":Ispport2EncodeFps", 30);
            MI_U32 u32BindVencId = iniparser_getint(pstDict, ":Ispport2BindVencId", 0);

            INIT_LIST_HEAD(&pstIspChnAttr->stIspOutPortAttr[2].head.pos);

            pstIspChnAttr->stIspOutPortAttr[2].stoutFileAttr.s32DumpBuffNum = iniparser_getint(pstDict, ":Ispport2DumpBuffNum", 0);
            string = iniparser_getstring(pstDict, ":Ispport2OutPutPath", (char *)"NULL");
            if(!strcmp((const char *)string, (const char *)"NULL"))
            {
                printf("OutPutPath NULL \n");
            }
            else
            {
                sprintf(pstIspChnAttr->stIspOutPortAttr[2].stoutFileAttr.FilePath, "%s/", string);

                printf("OutPutFile_Path:%s \n",pstIspChnAttr->stIspOutPortAttr[2].stoutFileAttr.FilePath);

                /***********************************************
                Mkdir path here to avoid running at the same time
                when Multi-threading and Multi-ports dumpfile
                ************************************************/
                s32Ret = ST_CheckMkdirOutFile(pstIspChnAttr->stIspOutPortAttr[2].stoutFileAttr.FilePath);
                if(s32Ret != MI_SUCCESS)
                {
                    goto EXIT;
                }
            }

            ST_BindParam_t stVencInBindParam;
            stVencInBindParam.eBindType = E_MI_SYS_BIND_TYPE_FRAME_BASE;
            stVencInBindParam.stChnPort.eModId = E_MI_MODULE_ID_ISP;
            stVencInBindParam.stChnPort.u32DevId = u32IspDevId;
            stVencInBindParam.stChnPort.u32ChnId = u32IspChnId;
            stVencInBindParam.stChnPort.u32PortId = 2;
            stVencInBindParam.u32SrcFrmrate = 30;
            stVencInBindParam.u32DstFrmrate = 30;
            //ST_IniSetVencParam(u32BindVencId,eEncType, &stVencInBindParam, u32EncFps);
        }
    }

    for(u32SclSrcId=0; u32SclSrcId<ST_MAX_SCL_DEV_NUM; u32SclSrcId++)
    {
        char PortString[128] = {0};
        MI_U32 u32SclDevUseSclId=0;
        ST_SclDevAttr_t *pstSclDevAttr = NULL;
        ST_SCL_SourceSelect_e  eSrcSelect = E_ST_SCL_SRC_FROM_INVALID;

        sprintf(PortString, ":%sUseSclId", SclDevString[u32SclSrcId]);
        u32SclDevUseSclId = iniparser_getint(pstDict, PortString, 0);

        if(!strcmp((const char *)SclDevString[u32SclSrcId], (const char *)"SclIspRealtime0"))
        {
            u32SclDevId =0;
            eSrcSelect = E_ST_SCL_SRC_FROM_ISP_REALTIME;
        }
        else if(!strcmp((const char *)SclDevString[u32SclSrcId], (const char *)"SclRdma0"))
        {
            u32SclDevId =1;
            eSrcSelect = E_ST_SCL_SRC_FROM_RDMA0;
        }
        else if(!strcmp((const char *)SclDevString[u32SclSrcId], (const char *)"SclYUVRealtime0"))
        {
            u32SclDevId =2;
            eSrcSelect = E_ST_SCL_SRC_FROM_YUV_REALTIME;
        }
        /*else if(!strcmp((const char *)SclDevString[u32SclSrcId], (const char *)"SclRdma1"))
        {
            u32SclDevId =3;
            eSrcSelect = E_ST_SCL_SRC_FROM_RDMA1;
        }*/
        else if(!strcmp((const char *)SclDevString[u32SclSrcId], (const char *)"SclRotRdma"))
        {
            u32SclDevId =3;
            eSrcSelect = E_ST_SCL_SRC_FROM_RDMA_ROT;
        }
        else if(!strcmp((const char *)SclDevString[u32SclSrcId], (const char *)"SclIspRealtime1"))
        {
            u32SclDevId =4;
            eSrcSelect = E_ST_SCL_SRC_FROM_ISP_REALTIME1;
        }
        else if(!strcmp((const char *)SclDevString[u32SclSrcId], (const char *)"SclRdma2"))
        {
            u32SclDevId =5;
            eSrcSelect = E_ST_SCL_SRC_FROM_RDMA2;
        }
        else if(!strcmp((const char *)SclDevString[u32SclSrcId], (const char *)"SclYUVRealtime1"))
        {
            u32SclDevId =6;
            eSrcSelect = E_ST_SCL_SRC_FROM_YUV_REALTIME1;
        }

        if(u32SclDevUseSclId >0)
        {
            MI_U32  u32SclChnId =0;
            MI_U32 u32SclId =0;
            MI_U8 u8PortId =0;
            ST_SclChannelAttr_t *pstSclChnAttr = NULL;
            pstSclDevAttr = &gstSclModule.stSclDevAttr[u32SclDevId];
            pstSclDevAttr->bUsed = TRUE;
            pstSclDevAttr->u32UseHwSclMask = u32SclDevUseSclId;
            pstSclDevAttr->u8DevId = u32SclDevId;

            for(u32SclId=0; u32SclId<ST_MAX_SCL_HWSCLID_NUM; u32SclId++)
            {
                if(u32SclDevUseSclId & (E_MI_SCL_HWSCL0 << u32SclId))
                {
                    pstSclDevAttr->u8HwSclPortId[u32SclId]=u8PortId;
                    u8PortId++;
                }
            }

            u32SclChnId = iniparser_getint(pstDict,":SclChnId",ST_MAX_SCL_CHN_NUM);
            if(u32SclChnId == ST_MAX_SCL_CHN_NUM)
            {
                for(u32SclChnId =0; u32SclChnId < ST_MAX_SCL_CHN_NUM; u32SclChnId++)
                {
                    pstSclChnAttr = &pstSclDevAttr->stSclChnlAttr[u32SclChnId];
                    if(pstSclChnAttr->bUsed == FALSE)
                    {
                        break;
                    }
                }
            }

            if(u32SclChnId == ST_MAX_SCL_CHN_NUM)
            {
                printf("[%s]%d find ispchn fail max %d all use \n", __FUNCTION__,__LINE__, ST_MAX_ISP_CHN_NUM);
                s32Ret = -1;
                goto EXIT;
            }
            pstSclChnAttr = &pstSclDevAttr->stSclChnlAttr[u32SclChnId];

            pstSclChnAttr->bUsed = TRUE;
            pstSclChnAttr->u8ChnId = u32SclChnId;
            pstSclChnAttr->stSclInPortAttr[0].bUsed = TRUE;
            pstSclChnAttr->eSclSrcSelect = eSrcSelect;

            DBG_INFO("srcselect %d >>>>>>>>>>>>>>>\n", pstSclChnAttr->eSclSrcSelect);
            if(pstSclChnAttr->eSclSrcSelect == E_ST_SCL_SRC_FROM_RDMA_ROT)
            {
                pstSclChnAttr->eRotate = (MI_SYS_Rotate_e)iniparser_getint(pstDict, ":SclRot", 0);
                DBG_INFO("scl rot %d >>>>>>>>>>>>>>>\n",  pstSclChnAttr->eRotate);
            }

            sprintf(PortString, ":%sInFilePath", SclDevString[u32SclDevId]);
            string = iniparser_getstring(pstDict, PortString, (char *)"NULL");
            if(!strcmp((const char *)string, (const char *)"NULL"))
            {
                if(pstSclChnAttr->eSclSrcSelect == E_ST_SCL_SRC_FROM_RDMA0
                    || pstSclChnAttr->eSclSrcSelect == E_ST_SCL_SRC_FROM_RDMA_ROT
                    || pstSclChnAttr->eSclSrcSelect == E_ST_SCL_SRC_FROM_RDMA2)
                {
                    if(bJpdUse == TRUE)
                    {
                        pstSclChnAttr->stSclInPortAttr[0].stBindParam.stChnPort.eModId = E_MI_MODULE_ID_JPD;
                        pstSclChnAttr->stSclInPortAttr[0].stBindParam.stChnPort.u32DevId = 0;
                        pstSclChnAttr->stSclInPortAttr[0].stBindParam.stChnPort.u32ChnId = u32JpdChn;
                        pstSclChnAttr->stSclInPortAttr[0].stBindParam.stChnPort.u32PortId = 0;
                        pstSclChnAttr->stSclInPortAttr[0].stBindParam.u32SrcFrmrate = 30;
                        pstSclChnAttr->stSclInPortAttr[0].stBindParam.u32DstFrmrate = 30;
                        pstSclChnAttr->stSclInPortAttr[0].stBindParam.eBindType = eJpdBindType;
                    }
                    else
                    {
                        pstSclChnAttr->stSclInPortAttr[0].stBindParam.stChnPort.eModId = E_MI_MODULE_ID_ISP;
                        pstSclChnAttr->stSclInPortAttr[0].stBindParam.stChnPort.u32DevId = u32IspDevId;
                        pstSclChnAttr->stSclInPortAttr[0].stBindParam.stChnPort.u32ChnId = u32IspChnId;
                        pstSclChnAttr->stSclInPortAttr[0].stBindParam.stChnPort.u32PortId = 1;
                        pstSclChnAttr->stSclInPortAttr[0].stBindParam.u32SrcFrmrate = 30;
                        pstSclChnAttr->stSclInPortAttr[0].stBindParam.u32DstFrmrate = 30;
                        pstSclChnAttr->stSclInPortAttr[0].stBindParam.eBindType = E_MI_SYS_BIND_TYPE_FRAME_BASE;
                    }
                }
                else if(pstSclChnAttr->eSclSrcSelect == E_ST_SCL_SRC_FROM_ISP_REALTIME
                      || pstSclChnAttr->eSclSrcSelect == E_ST_SCL_SRC_FROM_ISP_REALTIME1)
                {
                    pstSclChnAttr->stSclInPortAttr[0].stBindParam.stChnPort.eModId = E_MI_MODULE_ID_ISP;
                    pstSclChnAttr->stSclInPortAttr[0].stBindParam.stChnPort.u32DevId = u32IspDevId;
                    pstSclChnAttr->stSclInPortAttr[0].stBindParam.stChnPort.u32ChnId = u32IspChnId;
                    pstSclChnAttr->stSclInPortAttr[0].stBindParam.stChnPort.u32PortId = 0;
                    pstSclChnAttr->stSclInPortAttr[0].stBindParam.u32SrcFrmrate = 30;
                    pstSclChnAttr->stSclInPortAttr[0].stBindParam.u32DstFrmrate = 30;
                    pstSclChnAttr->stSclInPortAttr[0].stBindParam.eBindType = E_MI_SYS_BIND_TYPE_REALTIME;
                }
                else if(pstSclChnAttr->eSclSrcSelect == E_ST_SCL_SRC_FROM_YUV_REALTIME
                      || pstSclChnAttr->eSclSrcSelect == E_ST_SCL_SRC_FROM_YUV_REALTIME1)
                {
                    pstSclChnAttr->stSclInPortAttr[0].stBindParam.stChnPort.eModId = E_MI_MODULE_ID_VIF;
                    pstSclChnAttr->stSclInPortAttr[0].stBindParam.stChnPort.u32DevId = u32VifDev;
                    pstSclChnAttr->stSclInPortAttr[0].stBindParam.stChnPort.u32ChnId = 0;
                    pstSclChnAttr->stSclInPortAttr[0].stBindParam.stChnPort.u32PortId = vifPort;
                    pstSclChnAttr->stSclInPortAttr[0].stBindParam.u32SrcFrmrate = 30;
                    pstSclChnAttr->stSclInPortAttr[0].stBindParam.u32DstFrmrate = 30;
                    pstSclChnAttr->stSclInPortAttr[0].stBindParam.eBindType = E_MI_SYS_BIND_TYPE_REALTIME;
                }
                else
                {
                    UTStatus = UT_CASE_FAIL;
                    printf("[%s]%d scl source err %d \n", __FUNCTION__,__LINE__, pstSclChnAttr->eSclSrcSelect);
                    s32Ret = -1;
                    goto EXIT;
                }

            }
            else
            {
                MI_U16  u16size = strlen(string);
                memcpy(pstSclChnAttr->stSclInPortAttr[0].stInputFileAttr.InputFilePath, string, u16size);
                printf("pstInputFileAttr:%s \n",pstSclChnAttr->stSclInPortAttr[0].stInputFileAttr.InputFilePath);

                sprintf(PortString, ":%sInFileW", SclDevString[u32SclDevId]);
                pstSclChnAttr->stSclInPortAttr[0].stInputFileAttr.u32Width = iniparser_getint(pstDict, PortString, 0);
                sprintf(PortString, ":%sInFileH", SclDevString[u32SclDevId]);
                pstSclChnAttr->stSclInPortAttr[0].stInputFileAttr.u32Height = iniparser_getint(pstDict, PortString, 0);
                sprintf(PortString, ":%sInFilePixel", SclDevString[u32SclDevId]);
                string = iniparser_getstring(pstDict, PortString, (char *)"ERR");
                pstSclChnAttr->stSclInPortAttr[0].stInputFileAttr.ePixelFormat = ST_GetIniPixel(string);
                sprintf(PortString, ":%sInFileSleepMs", SclDevString[u32SclDevId]);
                pstSclChnAttr->stSclInPortAttr[0].stInputFileAttr.u32SleepMs = iniparser_getint(pstDict, PortString, 0);
                sprintf(PortString, ":%sInCompress", SclDevString[u32SclDevId]);
                pstSclChnAttr->stSclInPortAttr[0].stInputFileAttr.eCompress = (MI_SYS_CompressMode_e)iniparser_getint(pstDict, PortString, 0);
                sprintf(PortString, ":%sInCrc", SclDevString[u32SclDevId]);
                pstSclChnAttr->stSclInPortAttr[0].stInputFileAttr.bCrcCheck = (MI_BOOL)iniparser_getint(pstDict, PortString, 0);

                sprintf(PortString, ":%sContentCropX", SclDevString[u32SclDevId]);
                pstSclChnAttr->stSclInPortAttr[0].stInputFileAttr.stContentCropWindow.u16X = iniparser_getint(pstDict, PortString, 0);
                sprintf(PortString, ":%sContentCropY", SclDevString[u32SclDevId]);
                pstSclChnAttr->stSclInPortAttr[0].stInputFileAttr.stContentCropWindow.u16Y = iniparser_getint(pstDict, PortString, 0);
                sprintf(PortString, ":%sContentCropW", SclDevString[u32SclDevId]);
                pstSclChnAttr->stSclInPortAttr[0].stInputFileAttr.stContentCropWindow.u16Width = iniparser_getint(pstDict, PortString, 0);
                sprintf(PortString, ":%sContentCropH", SclDevString[u32SclDevId]);
                pstSclChnAttr->stSclInPortAttr[0].stInputFileAttr.stContentCropWindow.u16Height = iniparser_getint(pstDict, PortString, 0);

                pstSclChnAttr->stSclInPortAttr[0].stBindParam.stChnPort.eModId = E_MI_MODULE_ID_MAX;
            }

            if(u32SclDevId==1 || u32SclDevId==3)
            {
                sprintf(PortString, ":%sCropX", SclDevString[u32SclDevId]);
                pstSclChnAttr->stSclInPortAttr[0].stOrigInputCropWin.u16X = iniparser_getint(pstDict, PortString, 0);
                sprintf(PortString, ":%sCropY", SclDevString[u32SclDevId]);
                pstSclChnAttr->stSclInPortAttr[0].stOrigInputCropWin.u16Y = iniparser_getint(pstDict, PortString, 0);
                sprintf(PortString, ":%sCropW", SclDevString[u32SclDevId]);
                pstSclChnAttr->stSclInPortAttr[0].stOrigInputCropWin.u16Width = iniparser_getint(pstDict, PortString, 0);
                sprintf(PortString, ":%sCropH", SclDevString[u32SclDevId]);
                pstSclChnAttr->stSclInPortAttr[0].stOrigInputCropWin.u16Height = iniparser_getint(pstDict, PortString, 0);
            }

            for(u32SclId=0;u32SclId<ST_MAX_SCL_HWSCLID_NUM; u32SclId++)
            {
                if(u32SclDevUseSclId & 0x1<<u32SclId)
                {
                    MI_U32 bUse =0;
                    MI_U32  u32SclOutPortId=0;
                    sprintf(PortString, ":Scl%dUse", u32SclId);
                    bUse = iniparser_getint(pstDict, PortString, 0);
                    if(bUse == 1)
                    {
                        ST_PortAttr_t  *pstSclOutPortAttr = NULL;
                        u32SclOutPortId = pstSclDevAttr->u8HwSclPortId[u32SclId];
                        if(u32SclOutPortId >= ST_MAX_SCL_OUTPORT_NUM)
                        {
                            printf("[%s]%d find scl outputportid fail max %d all use \n", __FUNCTION__,__LINE__, ST_MAX_ISP_CHN_NUM);
                            s32Ret = -1;
                            goto EXIT;
                        }

                        pstSclOutPortAttr = &pstSclChnAttr->stSclOutPortAttr[u32SclOutPortId];
                        pstSclOutPortAttr->bUsed = bUse;

                        INIT_LIST_HEAD(&pstSclOutPortAttr->head.pos);

                        sprintf(PortString, ":Scl%dPortCropX", u32SclId);
                        pstSclOutPortAttr->stOrigPortCrop.u16X = iniparser_getint(pstDict, PortString, 0);
                        sprintf(PortString, ":Scl%dPortCropY", u32SclId);
                        pstSclOutPortAttr->stOrigPortCrop.u16Y = iniparser_getint(pstDict, PortString, 0);
                        sprintf(PortString, ":Scl%dPortCropW", u32SclId);
                        pstSclOutPortAttr->stOrigPortCrop.u16Width = iniparser_getint(pstDict, PortString, 0);
                        sprintf(PortString, ":Scl%dPortCropH", u32SclId);
                        pstSclOutPortAttr->stOrigPortCrop.u16Height = iniparser_getint(pstDict, PortString, 0);

                        sprintf(PortString, ":Scl%dmirror", u32SclId);
                        pstSclOutPortAttr->bMirror = iniparser_getint(pstDict, PortString, 0);
                        sprintf(PortString, ":Scl%dflip", u32SclId);
                        pstSclOutPortAttr->bFlip = iniparser_getint(pstDict, PortString, 0);

                        sprintf(PortString, ":Scl%dW", u32SclId);
                        pstSclOutPortAttr->stOrigPortSize.u16Width = iniparser_getint(pstDict, PortString, 0);
                        sprintf(PortString, ":Scl%dH", u32SclId);
                        pstSclOutPortAttr->stOrigPortSize.u16Height = iniparser_getint(pstDict, PortString, 0);

                        sprintf(PortString, ":Scl%dPixel", u32SclId);
                        string = iniparser_getstring(pstDict, PortString, (char *)"ERR");
                        pstSclOutPortAttr->ePixelFormat = ST_GetIniPixel(string);

                        sprintf(PortString, ":Scl%dCompress", u32SclId);
                        pstSclOutPortAttr->eCompressMode = (MI_SYS_CompressMode_e)iniparser_getint(pstDict, PortString, 0);

                        sprintf(PortString, ":Scl%duserdepth", u32SclId);
                        pstSclOutPortAttr->stoutFileAttr.u16UserDepth = (MI_U16)iniparser_getint(pstDict, PortString, 0);

                        sprintf(PortString, ":Scl%ddepth", u32SclId);
                        pstSclOutPortAttr->stoutFileAttr.u16Depth = (MI_U16)iniparser_getint(pstDict, PortString, 4);

                        sprintf(PortString, ":Scl%dDumpBuffNum", u32SclId);
                        pstSclOutPortAttr->stoutFileAttr.s32DumpBuffNum = (MI_S32)iniparser_getint(pstDict, PortString, 0);

                        sprintf(PortString, ":Scl%dOutPutPath", u32SclId);
                        string = iniparser_getstring(pstDict, PortString, (char *)"NULL");

                        printf("i %d, string:%s \n",u32SclId,string);
                        if(!strcmp((const char *)string, (const char *)"NULL"))
                        {
                            printf("OutPutPath NULL \n");
                        }
                        else
                        {
                            sprintf(pstSclOutPortAttr->stoutFileAttr.FilePath, "%s/", string);

                            printf("scl OutPutFile_Path:%s \n",pstSclOutPortAttr->stoutFileAttr.FilePath);

                            /***********************************************
                            Mkdir path here to avoid running at the same time
                            when Multi-threading and Multi-ports dumpfile
                            ************************************************/
                            s32Ret = ST_CheckMkdirOutFile(pstSclOutPortAttr->stoutFileAttr.FilePath);
                            if(s32Ret != MI_SUCCESS)
                            {
                                goto EXIT;
                            }
                        }

                        sprintf(PortString, ":Scl%dBindMod", u32SclId);
                        string = iniparser_getstring(pstDict, PortString, (char *)"VENC");

                        MI_SYS_BindType_e  eBindType = E_MI_SYS_BIND_TYPE_FRAME_BASE;
                        MI_U32  u32BindParam=0;
                        sprintf(PortString, ":Scl%dBindtype", u32SclId);
                        eBindType = (MI_SYS_BindType_e)iniparser_getint(pstDict, PortString, E_MI_SYS_BIND_TYPE_FRAME_BASE);

                        sprintf(PortString, ":Scl%dBindParam", u32SclId);
                        u32BindParam = (MI_U32)iniparser_getint(pstDict, PortString, 0);

                        if(!strcmp((const char *)string, (const char *)"DISP"))
                        {
                            gbPreviewByDisp = iniparser_getint(pstDict, ":UseDisp", 0);
                            if(TRUE == gbPreviewByDisp)
                            {
                                MI_U32 u32Dev = 0;
                                MI_U32 u32Layer = 0;
                                ST_DispDevAttr_t *pstDispDevAttr = &gstDispModule.stDispDevAttr[u32Dev];
                                ST_DispLayerAttr_t *pstDispLayerAttr = &pstDispDevAttr->stDispLayerAttr[u32Layer];
                                MI_U32 u32Port = pstDispLayerAttr->u16PortUseCount;
                                ST_DispPortAttr_t *pstDispPortAttr = &pstDispLayerAttr->stDispPortAttr[u32Port];

                                pstDispPortAttr->stBindInfo.stChnPort.u32DevId= u32SclDevId;
                                pstDispPortAttr->stBindInfo.stChnPort.u32ChnId= u32SclChnId;
                                pstDispPortAttr->stBindInfo.stChnPort.u32PortId = u32SclOutPortId;
                                pstDispPortAttr->stBindInfo.stChnPort.eModId = E_MI_MODULE_ID_SCL;
                                pstDispPortAttr->bUsed = true;
                                pstDispLayerAttr->u16PortUseCount++;
                                pstDispLayerAttr->bUsed = true;
                                pstDispDevAttr->bUsed = true;

                                sprintf(PortString, ":DispDev%dIntfType", u32Dev);
                                pstDispDevAttr->eIntfType= (MI_DISP_Interface_e)iniparser_getint(pstDict, PortString, 9);
                                if(E_MI_DISP_INTF_TTL == pstDispDevAttr->eIntfType)
                                {
                                    if(E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420 != pstSclOutPortAttr->ePixelFormat)
                                    {
                                        UTStatus = UT_CASE_FAIL;
                                        DBG_ERR("Because DispDev%dIntfType = TTL(9) , so port%dPixel force equal to semiplanar_420(11)",u32Dev,u32SclId);
                                        pstSclOutPortAttr->ePixelFormat = E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420;
                                    }
                                }
                                ST_DispParserIni(pstDict);
                            }
                        }
                        else if(!strcmp((const char *)string, (const char *)"VENC"))
                        {
                            MI_VENC_ModType_e  eEncType = E_MI_VENC_MODTYPE_VENC;
                            sprintf(PortString, ":Scl%dEncodeType", u32SclId);
                            eEncType = (MI_VENC_ModType_e)iniparser_getint(pstDict, PortString, E_MI_VENC_MODTYPE_VENC);
							if(eEncType == E_MI_VENC_MODTYPE_H265E)
							{
							sprintf(PortString, ":Scl%dEncode265IQp", u32SclId);
							bIQp =iniparser_getint(pstDict,PortString,0);
							sprintf(PortString, ":Scl%dEncode265PQp", u32SclId);
							bPQp =iniparser_getint(pstDict,PortString,0);
							}
                            sprintf(PortString, ":Scl%dEncodeFps", u32SclId);
                            MI_U32 u32EncFps = iniparser_getint(pstDict,PortString, 30);

                            sprintf(PortString, ":Scl%dBindVencId", u32SclId);
                            MI_U32 u32BindVencId = iniparser_getint(pstDict, PortString, 0);

                            ST_BindParam_t stVencInBindParam;
                            stVencInBindParam.eBindType = eBindType;
                            stVencInBindParam.u32BindParam = u32BindParam;
                            stVencInBindParam.stChnPort.eModId = E_MI_MODULE_ID_SCL;
                            stVencInBindParam.stChnPort.u32DevId = u32SclDevId;
                            stVencInBindParam.stChnPort.u32ChnId = u32SclChnId;
                            stVencInBindParam.stChnPort.u32PortId = u32SclOutPortId;
                            stVencInBindParam.u32SrcFrmrate = 30;
                            stVencInBindParam.u32DstFrmrate = 30;
                            //ST_IniSetVencParam(u32BindVencId,eEncType, &stVencInBindParam, u32EncFps);
                        }

                        char **array_str_holder=NULL;
                        int array_cnt=0;
                        char *holder = NULL;

                        pstSclOutPortAttr->stoutFileAttr.stMd5Attr.pu8IniPath = (MI_U8*)pIniPath;
                        sprintf(pstSclOutPortAttr->stoutFileAttr.stMd5Attr.key, ":Scl%dMd5", u32SclId);
                        sprintf(PortString, ":Scl%dMd5", u32SclId);
                        pstSclOutPortAttr->stoutFileAttr.stMd5Attr.eMd5Action = gMd5Action;
                        string = iniparser_getstring(pstDict, PortString, NULL);
                        if(string != NULL)
                        {
                            printf(">>>>>>>>>>>%s string %s \n",PortString, string);
                            holder=iniparser_parser_string_to_array(string, &array_str_holder, &array_cnt);
                            if(holder != NULL)
                            {
                                MI_U8 i=0, j=0;
                                //pstSclOutPortAttr->stoutFileAttr.bNeedCheckMd5 = TRUE;
                                pstSclOutPortAttr->stoutFileAttr.stMd5Attr.eMd5Action = gMd5Action > E_ST_MD5_ACTION_NONE ?
                                                                                        gMd5Action : E_ST_MD5_ACTION_CHECK;
                                for(i=0; i<array_cnt; i++)
                                {
                                    MI_U16  u16size = strlen(array_str_holder[i]);
                                    printf("MD5 size %d, value %s\n", u16size, array_str_holder[i]);
                                    for(j=0; j<16; j++)
                                    {
                                        pstSclOutPortAttr->stoutFileAttr.stMd5Attr.stMD5ExpectValue[i].u8MD5ExpectValue[j]=string_to_int8((MI_U8 *)&array_str_holder[i][j<<1]);
                                    }
                                    pstSclOutPortAttr->stoutFileAttr.stMd5Attr.stMD5ExpectValue[i].bUsed = TRUE;

                                    for(j=0; j<16; j++)
                                    {
                                        printf("%02x", pstSclOutPortAttr->stoutFileAttr.stMd5Attr.stMD5ExpectValue[i].u8MD5ExpectValue[j]);
                                    }
                                    printf("\n");
                                }

                                free(array_str_holder);
                                array_str_holder = NULL;
                                free(holder);
                                holder = NULL;
                            }
                        }
                    }
                }
            }
        }
    }
	

EXIT:

    iniparser_freedict(pstDict);

    return s32Ret;
}

int  ST_DynamicChangeFuncListSort(void *priv, struct list_head *a, struct list_head *b)
{
    ST_DynamicFuncParam_t *pstDynamicFuncA = NULL, *pstDynamicFuncB=NULL;
    pstDynamicFuncA = container_of(a, typeof(*pstDynamicFuncA), stDynamicFuncListNode);
    pstDynamicFuncB = container_of(b, typeof(*pstDynamicFuncB), stDynamicFuncListNode);

    if(pstDynamicFuncA->u16Id < pstDynamicFuncB->u16Id)
    {
        return 0;  //small first do it
    }
    else
    {
        return 1;
    }
}

MI_S32 ST_ParseDynamicTestIni(char *pIniPath)
{
    char **array_str_holder=NULL;
    int array_cnt=0;
    char *holder = NULL;
    MI_U32 i=0,u32MaxId=0,u32Sleep=0;
    struct list_head  stTempListHead;
    char *string= NULL;
    MI_SYS_ChnPort_t stDumpFileChnPort;
    memset(&stDumpFileChnPort, 0x0, sizeof(MI_SYS_ChnPort_t));

    dictionary *pstDict = iniparser_load(pIniPath);

    if(pstDict == NULL)
        return -1;

    printf("pstDict %p \n", pstDict);

    INIT_LIST_HEAD(&stTempListHead);

    MI_U32 u32RandomCnt = iniparser_getint(pstDict, ":random", 0);
    gstDynamicTest.u32ListRepeatCnt = iniparser_getint(pstDict, ":loop", 1);
    gstDynamicTest.s32DynamicDumpCnt = iniparser_getint(pstDict, ":dynamicdumpcnt", 0x7FFFFFFF);
    DBG_INFO("random cnt %d, loop cnt %d, dynamicdumpcnt %d\n", u32RandomCnt,gstDynamicTest.u32ListRepeatCnt, gstDynamicTest.s32DynamicDumpCnt);

    string = iniparser_getstring(pstDict, ":dumpfile", NULL);
    if(string != NULL)
    {
        MI_U8 u8ParamCnt=6;
        printf(">>>>>>>>>>>dumpfile string %s \n", string);
        holder=iniparser_parser_string_to_array(string, &array_str_holder, &array_cnt);
        if(holder != NULL)
        {
            if((array_cnt % u8ParamCnt) != 0)
            {
                UTStatus = UT_CASE_FAIL;
                DBG_ERR("changeisprot need param cnt %d, input cnt %d err, module/dev/chn/port/cnt/path \n",u8ParamCnt, array_cnt);
            }
            else
            {
                MI_U8  u8GroupParamCnt=array_cnt/u8ParamCnt;
                DBG_INFO("group param cnt %d\n", u8GroupParamCnt);
                for(i=0; i<u8GroupParamCnt; i++)
                {
                    ST_DumpFileInfo_t  *pstDumpFile= (ST_DumpFileInfo_t *)malloc(sizeof(ST_DumpFileInfo_t));
                    memset(pstDumpFile, 0x0, sizeof(ST_DumpFileInfo_t));

                    pstDumpFile->stChnPort.eModId = (MI_ModuleId_e)strtoul(array_str_holder[0+i*u8ParamCnt], 0, 0);
                    pstDumpFile->stChnPort.u32DevId = (MI_U32)strtoul(array_str_holder[1+i*u8ParamCnt], 0, 0);
                    pstDumpFile->stChnPort.u32ChnId = (MI_U32)strtoul(array_str_holder[2+i*u8ParamCnt], 0, 0);
                    pstDumpFile->stChnPort.u32PortId = (MI_U32)strtoul(array_str_holder[3+i*u8ParamCnt], 0, 0);
                    pstDumpFile->u16FileCnt = (MI_U16)strtoul(array_str_holder[4+i*u8ParamCnt], 0, 0);

                    MI_U16  u16size = strlen(array_str_holder[5+i*u8ParamCnt]);
                    memcpy(pstDumpFile->FilePath, array_str_holder[5+i*u8ParamCnt], u16size);

                    if( pstDumpFile->stChnPort.eModId == E_MI_MODULE_ID_VIF)
                    {
                        ST_VifPortAttr_t *pstVifPortAttr = NULL;
                        MI_U16 u16VifGroupId=pstDumpFile->stChnPort.u32DevId/ST_MAX_VIF_DEV_PERGROUP;
                        MI_U16 u16VifDevIdInGroup=pstDumpFile->stChnPort.u32DevId%ST_MAX_VIF_DEV_PERGROUP;
                        pstVifPortAttr = &gstVifModule.stVifGroupAttr[u16VifGroupId].stVifDevAttr[u16VifDevIdInGroup].stVifOutPortAttr[pstDumpFile->stChnPort.u32PortId];
                        pstDumpFile->poutputFileAttr = (void *)&pstVifPortAttr->stoutFileAttr;
                    }
                    else if( pstDumpFile->stChnPort.eModId == E_MI_MODULE_ID_ISP)
                    {
                        ST_PortAttr_t *pstIspPortAttr = NULL;
                        pstIspPortAttr = &gstIspModule.stIspDevAttr[pstDumpFile->stChnPort.u32DevId].stIspChnlAttr[pstDumpFile->stChnPort.u32ChnId].stIspOutPortAttr[pstDumpFile->stChnPort.u32PortId];
                        pstDumpFile->poutputFileAttr = (void *)&pstIspPortAttr->stoutFileAttr;
                    }
                    else if( pstDumpFile->stChnPort.eModId == E_MI_MODULE_ID_SCL)
                    {
                        ST_PortAttr_t *pstSclPortAttr = NULL;
                        pstSclPortAttr = &gstSclModule.stSclDevAttr[pstDumpFile->stChnPort.u32DevId].stSclChnlAttr[pstDumpFile->stChnPort.u32ChnId].stSclOutPortAttr[pstDumpFile->stChnPort.u32PortId];
                        pstDumpFile->poutputFileAttr = (void *)&pstSclPortAttr->stoutFileAttr;
                    }
                    DBG_INFO("dumpfile param moduleid %d, Dev %d, chn %d, port %d, cnt %d, filepath %s \n", pstDumpFile->stChnPort.eModId,pstDumpFile->stChnPort.u32DevId,
                        pstDumpFile->stChnPort.u32ChnId,pstDumpFile->stChnPort.u32PortId,pstDumpFile->u16FileCnt,pstDumpFile->FilePath);

                    list_add_tail(&pstDumpFile->stDumpFileListNode, &gstDynamicTest.stDumpFileListHead);
                }
            }

            free(array_str_holder);
            array_str_holder = NULL;
            free(holder);
            holder = NULL;
        }
        else
        {
            UTStatus = UT_CASE_FAIL;
            DBG_ERR("string to array fail \n");
        }
    }

    u32Sleep = (MI_U32)iniparser_getint(pstDict, ":changeisprotsleep", 1);
    string = iniparser_getstring(pstDict, ":changeisprot", NULL);
    if(string != NULL)
    {
        MI_U8 u8ParamCnt=7;
        printf(">>>>>>>>>>>changeisprot string %s \n", string);
        holder=iniparser_parser_string_to_array(string, &array_str_holder, &array_cnt);
        if(holder != NULL)
        {
            if((array_cnt % u8ParamCnt) != 0)
            {
                UTStatus = UT_CASE_FAIL;
                DBG_ERR("changeisprot need param cnt %d, input cnt %d err, id/cnt/devid/chnid/rot/mirror/flip \n",u8ParamCnt, array_cnt);
            }
            else
            {
                MI_U8  u8GroupParamCnt=array_cnt/u8ParamCnt;
                DBG_INFO("group param cnt %d\n", u8GroupParamCnt);
                for(i=0; i<u8GroupParamCnt; i++)
                {
                    ST_DynamicFuncParam_t  *pstDynamicChangeIspRot= (ST_DynamicFuncParam_t *)malloc(sizeof(ST_DynamicFuncParam_t));
                    MI_U16 u16Rot=0;
                    memset(pstDynamicChangeIspRot, 0x0, sizeof(ST_DynamicFuncParam_t));

                    pstDynamicChangeIspRot->eFuncType=E_DYNAMIC_CHANGE_ISPROT;
                    pstDynamicChangeIspRot->u16Id = (MI_U16)strtoul(array_str_holder[0+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIspRot->u32Cnt = (MI_U32)strtoul(array_str_holder[1+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIspRot->stChangeIspRot.DevId = (MI_ISP_DEV)strtoul(array_str_holder[2+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIspRot->stChangeIspRot.ChnId = (MI_ISP_CHANNEL)strtoul(array_str_holder[3+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIspRot->u32Sleep = u32Sleep;
                    u16Rot = (MI_SYS_Rotate_e)strtoul(array_str_holder[4+i*u8ParamCnt], 0, 0);
                    switch(u16Rot)
                    {
                        case 0:
                            pstDynamicChangeIspRot->stChangeIspRot.eRot = E_MI_SYS_ROTATE_NONE;
                            break;
                        case 90:
                            pstDynamicChangeIspRot->stChangeIspRot.eRot = E_MI_SYS_ROTATE_90;
                            break;
                        case 180:
                            pstDynamicChangeIspRot->stChangeIspRot.eRot = E_MI_SYS_ROTATE_180;
                            break;
                        case 270:
                            pstDynamicChangeIspRot->stChangeIspRot.eRot = E_MI_SYS_ROTATE_270;
                            break;
                        default:
                            DBG_ERR("rot param %d err, only 0,90,180,270 \n", u16Rot);
                            UTStatus = UT_CASE_FAIL;
                            break;
                    }
                    pstDynamicChangeIspRot->stChangeIspRot.bMirror = (MI_BOOL)strtoul(array_str_holder[5+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIspRot->stChangeIspRot.bFlip = (MI_BOOL)strtoul(array_str_holder[6+i*u8ParamCnt], 0, 0);

                    if(u32MaxId < pstDynamicChangeIspRot->u16Id)
                        u32MaxId = pstDynamicChangeIspRot->u16Id;

                    DBG_INFO("changeisprot param id %d, cnt %d, Dev %d, chn %d, rot %d, mirror %d, flip %d sleep %d\n", pstDynamicChangeIspRot->u16Id, pstDynamicChangeIspRot->u32Cnt,
                        pstDynamicChangeIspRot->stChangeIspRot.DevId, pstDynamicChangeIspRot->stChangeIspRot.ChnId,pstDynamicChangeIspRot->stChangeIspRot.eRot,pstDynamicChangeIspRot->stChangeIspRot.bMirror,pstDynamicChangeIspRot->stChangeIspRot.bFlip,pstDynamicChangeIspRot->u32Sleep);

                    if(u32RandomCnt ==0)
                        list_add_tail(&pstDynamicChangeIspRot->stDynamicFuncListNode, &gstDynamicTest.stDynamicFuncListHead);
                    else
                        list_add_tail(&pstDynamicChangeIspRot->stDynamicFuncListNode, &stTempListHead);
                }
            }

            free(array_str_holder);
            array_str_holder = NULL;
            free(holder);
            holder = NULL;
        }
        else
        {
            UTStatus = UT_CASE_FAIL;
            DBG_ERR("string to array fail \n");
        }
    }

    u32Sleep = (MI_U32)iniparser_getint(pstDict, ":changehdrressleep", 1);
    string = iniparser_getstring(pstDict, ":changehdrres", NULL);
    if(string != NULL)
    {
        MI_U8 u8ParamCnt=5;
        printf(">>>>>>>>>>>changehdrres string %s \n", string);
        holder=iniparser_parser_string_to_array(string, &array_str_holder, &array_cnt);
        if(holder !=NULL)
        {
            if(array_cnt%u8ParamCnt !=0)
            {
                UTStatus = UT_CASE_FAIL;
                DBG_ERR("changehdrres need param cnt %d, input cnt %d err, id/cnt/groupid/bHDR/ResIdx \n",u8ParamCnt, array_cnt);
            }
            else
            {
                MI_U8  u8GroupParamCnt=array_cnt/u8ParamCnt;
                DBG_INFO("group param cnt %d\n", u8GroupParamCnt);
                for(i=0; i<u8GroupParamCnt; i++)
                {
                    ST_DynamicFuncParam_t  *pstDynamicChangeHdrRes= (ST_DynamicFuncParam_t *)malloc(sizeof(ST_DynamicFuncParam_t));
                    memset(pstDynamicChangeHdrRes, 0x0, sizeof(ST_DynamicFuncParam_t));

                    pstDynamicChangeHdrRes->eFuncType=E_DYNAMIC_CHANGE_HDRRES;
                    pstDynamicChangeHdrRes->u16Id = (MI_U16)strtoul(array_str_holder[0+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeHdrRes->u32Cnt = (MI_U32)strtoul(array_str_holder[1+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeHdrRes->stChangeHdrResParam.GroupId = (MI_VIF_GROUP)strtoul(array_str_holder[2+i*u8ParamCnt], 0,0);
                    pstDynamicChangeHdrRes->stChangeHdrResParam.bUseHdr = (MI_BOOL)strtoul(array_str_holder[3+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeHdrRes->stChangeHdrResParam.u8ResIndex = (MI_U8)strtoul(array_str_holder[4+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeHdrRes->u32Sleep = u32Sleep;

                    if(u32MaxId < pstDynamicChangeHdrRes->u16Id)
                        u32MaxId = pstDynamicChangeHdrRes->u16Id;

                    DBG_INFO("changehdrres param id %d, cnt %d, group %d, busehdr %d, resindx %d sleep %d\n", pstDynamicChangeHdrRes->u16Id, pstDynamicChangeHdrRes->u32Cnt,
                        pstDynamicChangeHdrRes->stChangeHdrResParam.GroupId, pstDynamicChangeHdrRes->stChangeHdrResParam.bUseHdr,pstDynamicChangeHdrRes->stChangeHdrResParam.u8ResIndex,pstDynamicChangeHdrRes->u32Sleep);

                    if(u32RandomCnt ==0)
                        list_add_tail(&pstDynamicChangeHdrRes->stDynamicFuncListNode, &gstDynamicTest.stDynamicFuncListHead);
                    else
                        list_add_tail(&pstDynamicChangeHdrRes->stDynamicFuncListNode, &stTempListHead);
                }
            }

            free(array_str_holder);
            array_str_holder = NULL;
            free(holder);
            holder = NULL;
        }
        else
        {
            UTStatus = UT_CASE_FAIL;
            DBG_ERR("string to array fail \n");
        }
    }

    u32Sleep = (MI_U32)iniparser_getint(pstDict, ":changevifgroupattrsleep", 1);
    string = iniparser_getstring(pstDict, ":changevifgroupattr", NULL);
    if(string != NULL)
    {
        MI_U8 u8ParamCnt=4;
        printf(">>>>>>>>>>>changevifgroupattr string %s \n", string);
        holder=iniparser_parser_string_to_array(string, &array_str_holder, &array_cnt);
        if(holder !=NULL)
        {
            if(array_cnt%u8ParamCnt !=0)
            {
                UTStatus = UT_CASE_FAIL;
                DBG_ERR("changevifgroupattr need param cnt %d, input cnt %d err, id/cnt/GroupId/InputPixel \n",u8ParamCnt, array_cnt);
            }
            else
            {
                MI_U8  u8GroupParamCnt=array_cnt/u8ParamCnt;
                DBG_INFO("group param cnt %d\n", u8GroupParamCnt);
                for(i=0; i<u8GroupParamCnt; i++)
                {
                    ST_DynamicFuncParam_t  *pstDynamicChangeVifGroupAttrRes= (ST_DynamicFuncParam_t *)malloc(sizeof(ST_DynamicFuncParam_t));
                    ST_DynamicChangeVifGroupAttr_t *pstChangeVifGroupAttr = &pstDynamicChangeVifGroupAttrRes->stChangeVifGroupAttr;
                    memset(pstDynamicChangeVifGroupAttrRes, 0x0, sizeof(ST_DynamicFuncParam_t));

                    pstDynamicChangeVifGroupAttrRes->eFuncType=E_DYNAMIC_CHANGE_VIFGROUPATTR;
                    pstDynamicChangeVifGroupAttrRes->u16Id = (MI_U16)strtoul(array_str_holder[0+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeVifGroupAttrRes->u32Cnt = (MI_U32)strtoul(array_str_holder[1+i*u8ParamCnt], 0, 0);
                    pstChangeVifGroupAttr->GroupId = (MI_VIF_GROUP)strtoul(array_str_holder[2+i*u8ParamCnt], 0,0);
                    pstChangeVifGroupAttr->eInputPixel = ST_GetIniPixel(array_str_holder[3+i*u8ParamCnt]);
                    pstDynamicChangeVifGroupAttrRes->u32Sleep = u32Sleep;

                    if(u32MaxId < pstDynamicChangeVifGroupAttrRes->u16Id)
                        u32MaxId = pstDynamicChangeVifGroupAttrRes->u16Id;

                    DBG_INFO("changevifgroupattr param id %d, cnt %d,GroupId %d, InputPixel %d, sleep %d\n",
                    pstDynamicChangeVifGroupAttrRes->u16Id,pstDynamicChangeVifGroupAttrRes->u32Cnt,pstChangeVifGroupAttr->GroupId,
                    pstChangeVifGroupAttr->eInputPixel, pstDynamicChangeVifGroupAttrRes->u32Sleep);

                    if(u32RandomCnt ==0)
                        list_add_tail(&pstDynamicChangeVifGroupAttrRes->stDynamicFuncListNode, &gstDynamicTest.stDynamicFuncListHead);
                    else
                        list_add_tail(&pstDynamicChangeVifGroupAttrRes->stDynamicFuncListNode, &stTempListHead);
                }
            }

            free(array_str_holder);
            array_str_holder = NULL;
            free(holder);
            holder = NULL;
        }
        else
        {
            UTStatus = UT_CASE_FAIL;
            DBG_ERR("string to array fail \n");
        }
    }

    u32Sleep = (MI_U32)iniparser_getint(pstDict, ":changevifdevattrsleep", 1);
    string = iniparser_getstring(pstDict, ":changevifdevattr", NULL);
    if(string != NULL)
    {
        MI_U8 u8ParamCnt=10;
        printf(">>>>>>>>>>>changevifdevattr string %s \n", string);
        holder=iniparser_parser_string_to_array(string, &array_str_holder, &array_cnt);
        if(holder !=NULL)
        {
            if(array_cnt%u8ParamCnt !=0)
            {
                UTStatus = UT_CASE_FAIL;
                DBG_ERR("changevifdevattr need param cnt %d, input cnt %d err, id/cnt/Dev/InputPixel/InputRectX/InputRectY/InputRectW/InputRectH/Field/EnH2T1PMode \n",u8ParamCnt, array_cnt);
            }
            else
            {
                MI_U8  u8GroupParamCnt=array_cnt/u8ParamCnt;
                DBG_INFO("group param cnt %d\n", u8GroupParamCnt);
                for(i=0; i<u8GroupParamCnt; i++)
                {
                    ST_DynamicFuncParam_t  *pstDynamicChangeVifDevAttrRes= (ST_DynamicFuncParam_t *)malloc(sizeof(ST_DynamicFuncParam_t));
                    ST_DynamicChangeVifDevAttr_t *pstChangeVifDevAttr = &pstDynamicChangeVifDevAttrRes->stChangeVifDevAttr;
                    MI_VIF_DevAttr_t *pstDevAttr = &pstChangeVifDevAttr->stDevAttr;
                    memset(pstDynamicChangeVifDevAttrRes, 0x0, sizeof(ST_DynamicFuncParam_t));

                    pstDynamicChangeVifDevAttrRes->eFuncType=E_DYNAMIC_CHANGE_VIFDEVATTR;
                    pstDynamicChangeVifDevAttrRes->u16Id = (MI_U16)strtoul(array_str_holder[0+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeVifDevAttrRes->u32Cnt = (MI_U32)strtoul(array_str_holder[1+i*u8ParamCnt], 0, 0);
                    pstChangeVifDevAttr->DevId = (MI_VIF_DEV)strtoul(array_str_holder[2+i*u8ParamCnt], 0,0);
                    pstDevAttr->eInputPixel = ST_GetIniPixel(array_str_holder[3+i*u8ParamCnt]);
                    pstDevAttr->stInputRect.u16X = (MI_U16)strtoul(array_str_holder[4+i*u8ParamCnt], 0, 0);
                    pstDevAttr->stInputRect.u16Y = (MI_U16)strtoul(array_str_holder[5+i*u8ParamCnt], 0, 0);
                    pstDevAttr->stInputRect.u16Width = (MI_U16)strtoul(array_str_holder[6+i*u8ParamCnt], 0, 0);
                    pstDevAttr->stInputRect.u16Height = (MI_U16)strtoul(array_str_holder[7+i*u8ParamCnt], 0, 0);
                    pstDevAttr->eField = (MI_SYS_FieldType_e)strtoul(array_str_holder[8+i*u8ParamCnt], 0, 0);
                    pstDevAttr->bEnH2T1PMode = (MI_BOOL)strtoul(array_str_holder[9+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeVifDevAttrRes->u32Sleep = u32Sleep;

                    if(u32MaxId < pstDynamicChangeVifDevAttrRes->u16Id)
                        u32MaxId = pstDynamicChangeVifDevAttrRes->u16Id;

                    DBG_INFO("changevifdevattr param id %d, cnt %d,Dev %d, InputPixel %d,InputRect(%d,%d,%d,%d), Field %d, EnH2T1PMode %d , sleep %d\n",
                    pstDynamicChangeVifDevAttrRes->u16Id,pstDynamicChangeVifDevAttrRes->u32Cnt,pstChangeVifDevAttr->DevId,pstDevAttr->eInputPixel,
                    pstDevAttr->stInputRect.u16X,pstDevAttr->stInputRect.u16Y,pstDevAttr->stInputRect.u16Width,pstDevAttr->stInputRect.u16Height,
                    pstDevAttr->eField,pstDevAttr->bEnH2T1PMode,pstDynamicChangeVifDevAttrRes->u32Sleep);

                    if(u32RandomCnt ==0)
                        list_add_tail(&pstDynamicChangeVifDevAttrRes->stDynamicFuncListNode, &gstDynamicTest.stDynamicFuncListHead);
                    else
                        list_add_tail(&pstDynamicChangeVifDevAttrRes->stDynamicFuncListNode, &stTempListHead);
                }
            }

            free(array_str_holder);
            array_str_holder = NULL;
            free(holder);
            holder = NULL;
        }
        else
        {
            UTStatus = UT_CASE_FAIL;
            DBG_ERR("string to array fail \n");
        }
    }

    u32Sleep = (MI_U32)iniparser_getint(pstDict, ":changevifoutputattrsleep", 1);
    string = iniparser_getstring(pstDict, ":changevifoutputattr", NULL);
    if(string != NULL)
    {
        MI_U8 u8ParamCnt=13;
        printf(">>>>>>>>>>>changevifoutputattr string %s \n", string);
        holder=iniparser_parser_string_to_array(string, &array_str_holder, &array_cnt);
        if(holder !=NULL)
        {
            if(array_cnt%u8ParamCnt !=0)
            {
                UTStatus = UT_CASE_FAIL;
                DBG_ERR("changevifoutputattr need param cnt %d, input cnt %d err, id/cnt/DevId/PortId/CapRectX/CapRectY/CapRectW/CapRectH/DestSizeW/DestSizeH/PixFormat/FrameRate/Compress \n",u8ParamCnt, array_cnt);
            }
            else
            {
                MI_U8  u8GroupParamCnt=array_cnt/u8ParamCnt;
                DBG_INFO("group param cnt %d\n", u8GroupParamCnt);
                for(i=0; i<u8GroupParamCnt; i++)
                {
                    ST_DynamicFuncParam_t  *pstDynamicChangeVifOutPutAttrRes= (ST_DynamicFuncParam_t *)malloc(sizeof(ST_DynamicFuncParam_t));
                    ST_DynamicChangeVifOutPutAttr_t *pstChangeVifOutPutAttr = &pstDynamicChangeVifOutPutAttrRes->stChangeVifOutPutAttr;
                    MI_VIF_OutputPortAttr_t *pstOutPutAttr = &pstChangeVifOutPutAttr->stOutPutAttr;
                    memset(pstDynamicChangeVifOutPutAttrRes, 0x0, sizeof(ST_DynamicFuncParam_t));

                    pstDynamicChangeVifOutPutAttrRes->eFuncType=E_DYNAMIC_CHANGE_VIFOUTPORTATTR;
                    pstDynamicChangeVifOutPutAttrRes->u16Id = (MI_U16)strtoul(array_str_holder[0+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeVifOutPutAttrRes->u32Cnt = (MI_U32)strtoul(array_str_holder[1+i*u8ParamCnt], 0, 0);
                    pstChangeVifOutPutAttr->DevId = (MI_VIF_DEV)strtoul(array_str_holder[2+i*u8ParamCnt], 0,0);
                    pstChangeVifOutPutAttr->PortId = (MI_VIF_PORT)strtoul(array_str_holder[3+i*u8ParamCnt], 0,0);
                    pstOutPutAttr->stCapRect.u16X = (MI_U16)strtoul(array_str_holder[4+i*u8ParamCnt], 0, 0);
                    pstOutPutAttr->stCapRect.u16Y = (MI_U16)strtoul(array_str_holder[5+i*u8ParamCnt], 0, 0);
                    pstOutPutAttr->stCapRect.u16Width = (MI_U16)strtoul(array_str_holder[6+i*u8ParamCnt], 0, 0);
                    pstOutPutAttr->stCapRect.u16Height = (MI_U16)strtoul(array_str_holder[7+i*u8ParamCnt], 0, 0);
                    pstOutPutAttr->stDestSize.u16Width = (MI_U16)strtoul(array_str_holder[8+i*u8ParamCnt], 0, 0);
                    pstOutPutAttr->stDestSize.u16Height = (MI_U16)strtoul(array_str_holder[9+i*u8ParamCnt], 0, 0);
                    pstOutPutAttr->ePixFormat = ST_GetIniPixel(array_str_holder[10+i*u8ParamCnt]);
                    pstOutPutAttr->eFrameRate = (MI_VIF_FrameRate_e)strtoul(array_str_holder[11+i*u8ParamCnt], 0, 0);
                    pstOutPutAttr->eCompressMode = (MI_SYS_CompressMode_e)strtoul(array_str_holder[12+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeVifOutPutAttrRes->u32Sleep = u32Sleep;

                    if(u32MaxId < pstDynamicChangeVifOutPutAttrRes->u16Id)
                        u32MaxId = pstDynamicChangeVifOutPutAttrRes->u16Id;

                    DBG_INFO("changevifoutputattr param id %d, cnt %d, DevId %d,portId %d,CapRect(%d,%d,%d,%d), DestSize(%d, %d), PixFormat %d ,FrameRate %d,Compress %d, sleep %d\n",
                    pstDynamicChangeVifOutPutAttrRes->u16Id,pstDynamicChangeVifOutPutAttrRes->u32Cnt,pstChangeVifOutPutAttr->DevId,pstChangeVifOutPutAttr->PortId,
                    pstOutPutAttr->stCapRect.u16X,pstOutPutAttr->stCapRect.u16Y,pstOutPutAttr->stCapRect.u16Width,pstOutPutAttr->stCapRect.u16Height,pstOutPutAttr->stDestSize.u16Width,
                    pstOutPutAttr->stDestSize.u16Height,pstOutPutAttr->ePixFormat,pstOutPutAttr->eFrameRate,pstOutPutAttr->eCompressMode,pstDynamicChangeVifOutPutAttrRes->u32Sleep);

                    if(u32RandomCnt ==0)
                        list_add_tail(&pstDynamicChangeVifOutPutAttrRes->stDynamicFuncListNode, &gstDynamicTest.stDynamicFuncListHead);
                    else
                        list_add_tail(&pstDynamicChangeVifOutPutAttrRes->stDynamicFuncListNode, &stTempListHead);
                }
            }

            free(array_str_holder);
            array_str_holder = NULL;
            free(holder);
            holder = NULL;
        }
        else
        {
            UTStatus = UT_CASE_FAIL;
            DBG_ERR("string to array fail \n");
        }
    }

    u32Sleep = (MI_U32)iniparser_getint(pstDict, ":changevifoutFRCsleep", 1);
    string = iniparser_getstring(pstDict, ":changevifoutFRC", NULL);
    if(string != NULL)
    {
        MI_U8 u8ParamCnt=5;
        printf(">>>>>>>>>>>changevifoutFRC string %s \n", string);
        holder=iniparser_parser_string_to_array(string, &array_str_holder, &array_cnt);
        if(holder !=NULL)
        {
            if(array_cnt%u8ParamCnt !=0)
            {
                UTStatus = UT_CASE_FAIL;
                DBG_ERR("changevifoutFRC need param cnt %d, input cnt %d err, id/cnt/DevId/PortId/FrameRate \n",u8ParamCnt, array_cnt);
            }
            else
            {
                MI_U8  u8GroupParamCnt=array_cnt/u8ParamCnt;
                DBG_INFO("group param cnt %d\n", u8GroupParamCnt);
                for(i=0; i<u8GroupParamCnt; i++)
                {
                    ST_DynamicFuncParam_t  *pstDynamicChangeVifOutFRCRes= (ST_DynamicFuncParam_t *)malloc(sizeof(ST_DynamicFuncParam_t));
                    ST_DynamicChangeVifOutFRC_t *pstChangeVifOutOutFRC = &pstDynamicChangeVifOutFRCRes->stChangeVifOutOutFRC;
                    memset(pstDynamicChangeVifOutFRCRes, 0x0, sizeof(ST_DynamicFuncParam_t));

                    pstDynamicChangeVifOutFRCRes->eFuncType=E_DYNAMIC_CHANGE_VIFOUTFRC;
                    pstDynamicChangeVifOutFRCRes->u16Id = (MI_U16)strtoul(array_str_holder[0+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeVifOutFRCRes->u32Cnt = (MI_U32)strtoul(array_str_holder[1+i*u8ParamCnt], 0, 0);
                    pstChangeVifOutOutFRC->DevId = (MI_VIF_DEV)strtoul(array_str_holder[2+i*u8ParamCnt], 0,0);
                    pstChangeVifOutOutFRC->PortId = (MI_VIF_PORT)strtoul(array_str_holder[3+i*u8ParamCnt], 0,0);
                    pstChangeVifOutOutFRC->u32FrameRate = (MI_U32)strtoul(array_str_holder[4+i*u8ParamCnt], 0,0);
                    pstDynamicChangeVifOutFRCRes->u32Sleep = u32Sleep;

                    if(u32MaxId < pstDynamicChangeVifOutFRCRes->u16Id)
                        u32MaxId = pstDynamicChangeVifOutFRCRes->u16Id;

                    DBG_INFO("changevifoutFRC param id %d, cnt %d,DevId %d, PortId %d, FrameRate %d, sleep %d,\n",
                    pstDynamicChangeVifOutFRCRes->u16Id,pstDynamicChangeVifOutFRCRes->u32Cnt,pstChangeVifOutOutFRC->DevId,
                    pstChangeVifOutOutFRC->PortId,pstChangeVifOutOutFRC->u32FrameRate,pstDynamicChangeVifOutFRCRes->u32Sleep);

                    if(u32RandomCnt ==0)
                        list_add_tail(&pstDynamicChangeVifOutFRCRes->stDynamicFuncListNode, &gstDynamicTest.stDynamicFuncListHead);
                    else
                        list_add_tail(&pstDynamicChangeVifOutFRCRes->stDynamicFuncListNode, &stTempListHead);
                }
            }

            free(array_str_holder);
            array_str_holder = NULL;
            free(holder);
            holder = NULL;
        }
        else
        {
            UTStatus = UT_CASE_FAIL;
            DBG_ERR("string to array fail \n");
        }
    }

    u32Sleep = (MI_U32)iniparser_getint(pstDict, ":changevifoutputonoffsleep", 1);
    string = iniparser_getstring(pstDict, ":changevifoutputonoff", NULL);
    if(string != NULL)
    {
        MI_U8 u8ParamCnt=5;
        printf(">>>>>>>>>>>changevifoutputonoff string %s \n", string);
        holder=iniparser_parser_string_to_array(string, &array_str_holder, &array_cnt);
        if(holder !=NULL)
        {
            if(array_cnt%u8ParamCnt !=0)
            {
                UTStatus = UT_CASE_FAIL;
                DBG_ERR("changevifoutputonoff need param cnt %d, input cnt %d err, id/cnt/devid/portid/bOn \n",u8ParamCnt, array_cnt);
            }
            else
            {
                MI_U8  u8GroupParamCnt=array_cnt/u8ParamCnt;
                DBG_INFO("group param cnt %d\n", u8GroupParamCnt);
                for(i=0; i<u8GroupParamCnt; i++)
                {
                    ST_DynamicFuncParam_t  *pstDynamicChangeVifOutPutOnOffRes= (ST_DynamicFuncParam_t *)malloc(sizeof(ST_DynamicFuncParam_t));
                    ST_DynamicChangeVifOutPutOnOff_t *pstChangeVifOutPutOnOff = &pstDynamicChangeVifOutPutOnOffRes->stChangeVifOutPutOnOff;
                    memset(pstDynamicChangeVifOutPutOnOffRes, 0x0, sizeof(ST_DynamicFuncParam_t));

                    pstDynamicChangeVifOutPutOnOffRes->eFuncType=E_DYNAMIC_CHANGE_VIFOUTPORTONOFF;
                    pstDynamicChangeVifOutPutOnOffRes->u16Id = (MI_U16)strtoul(array_str_holder[0+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeVifOutPutOnOffRes->u32Cnt = (MI_U32)strtoul(array_str_holder[1+i*u8ParamCnt], 0, 0);
                    pstChangeVifOutPutOnOff->DevId = (MI_VIF_DEV)strtoul(array_str_holder[2+i*u8ParamCnt], 0,0);
                    pstChangeVifOutPutOnOff->PortId = (MI_VIF_PORT)strtoul(array_str_holder[3+i*u8ParamCnt], 0,0);
                    pstChangeVifOutPutOnOff->bOn = (MI_U32)strtoul(array_str_holder[4+i*u8ParamCnt], 0,0);
                    pstDynamicChangeVifOutPutOnOffRes->u32Sleep = u32Sleep;

                    if(u32MaxId < pstDynamicChangeVifOutPutOnOffRes->u16Id)
                        u32MaxId = pstDynamicChangeVifOutPutOnOffRes->u16Id;

                    DBG_INFO("changevifoutputonoff param id %d, cnt %d, dev %d, port %d, bOn %d, sleep %d,\n",
                    pstDynamicChangeVifOutPutOnOffRes->u16Id,pstDynamicChangeVifOutPutOnOffRes->u32Cnt,pstChangeVifOutPutOnOff->DevId,
                    pstChangeVifOutPutOnOff->PortId,pstChangeVifOutPutOnOff->bOn,pstDynamicChangeVifOutPutOnOffRes->u32Sleep);

                    if(u32RandomCnt ==0)
                        list_add_tail(&pstDynamicChangeVifOutPutOnOffRes->stDynamicFuncListNode, &gstDynamicTest.stDynamicFuncListHead);
                    else
                        list_add_tail(&pstDynamicChangeVifOutPutOnOffRes->stDynamicFuncListNode, &stTempListHead);
                }
            }

            free(array_str_holder);
            array_str_holder = NULL;
            free(holder);
            holder = NULL;
        }
        else
        {
            UTStatus = UT_CASE_FAIL;
            DBG_ERR("string to array fail \n");
        }
    }

    u32Sleep = (MI_U32)iniparser_getint(pstDict, ":changeVifDevOnOffsleep", 1);
    string = iniparser_getstring(pstDict, ":changeVifDevOnOff", NULL);
    if(string != NULL)
    {
        MI_U8 u8ParamCnt=4;
        printf(">>>>>>>>>>>VifDevOnOff string %s \n", string);
        holder=iniparser_parser_string_to_array(string, &array_str_holder, &array_cnt);
        if(holder != NULL)
        {
            if((array_cnt % u8ParamCnt) != 0)
            {
                UTStatus = UT_CASE_FAIL;
                DBG_ERR("VifDevOnOff need param cnt %d, input cnt %d err, id/cnt/devid/bOn \n",u8ParamCnt, array_cnt);
            }
            else
            {
                MI_U8  u8GroupParamCnt=array_cnt/u8ParamCnt;
                DBG_INFO("group param cnt %d\n", u8GroupParamCnt);
                for(i=0; i<u8GroupParamCnt; i++)
                {
                    ST_DynamicFuncParam_t  *pstVifDevOnOff = (ST_DynamicFuncParam_t *)malloc(sizeof(ST_DynamicFuncParam_t));
                    memset(pstVifDevOnOff, 0x0, sizeof(ST_DynamicFuncParam_t));

                    pstVifDevOnOff->eFuncType = E_DYNAMIC_CHANGE_VIFDEVONOFF;
                    pstVifDevOnOff->u16Id = (MI_U16)strtoul(array_str_holder[0+i*u8ParamCnt], 0, 0);
                    pstVifDevOnOff->u32Cnt = (MI_U32)strtoul(array_str_holder[1+i*u8ParamCnt], 0, 0);
                    pstVifDevOnOff->stChangeVifDevOnOff.DevId = (MI_U32)strtoul(array_str_holder[2+i*u8ParamCnt], 0, 0);
                    pstVifDevOnOff->stChangeVifDevOnOff.bOn = (MI_BOOL)strtoul(array_str_holder[3+i*u8ParamCnt], 0, 0);
                    pstVifDevOnOff->u32Sleep = u32Sleep;

                    if(u32MaxId < pstVifDevOnOff->u16Id)
                    {
                        u32MaxId = pstVifDevOnOff->u16Id;
                    }

                    DBG_INFO("DevOnOff id %d,cnt %d, devid%d bOn %d\n",pstVifDevOnOff->u16Id,pstVifDevOnOff->u32Cnt,
                        pstVifDevOnOff->stChangeVifDevOnOff.DevId, pstVifDevOnOff->stChangeVifDevOnOff.bOn);

                    if(u32RandomCnt ==0)
                        list_add_tail(&pstVifDevOnOff->stDynamicFuncListNode, &gstDynamicTest.stDynamicFuncListHead);
                    else
                        list_add_tail(&pstVifDevOnOff->stDynamicFuncListNode, &stTempListHead);
                }
            }

            free(array_str_holder);
            array_str_holder = NULL;
            free(holder);
            holder = NULL;
        }
        else
        {
            UTStatus = UT_CASE_FAIL;
            DBG_ERR("string to array fail \n");
        }
    }

    u32Sleep = (MI_U32)iniparser_getint(pstDict, ":changecropstepsleep", 1);
    string = iniparser_getstring(pstDict, ":changecropstep", NULL);
    if(string != NULL)
    {
        MI_U8 u8ParamCnt=9;
        printf(">>>>>>>>>>>changecropstep string %s \n", string);
        holder=iniparser_parser_string_to_array(string, &array_str_holder, &array_cnt);
        if(holder !=NULL)
        {
            if(array_cnt%u8ParamCnt !=0)
            {
                UTStatus = UT_CASE_FAIL;
                DBG_ERR("changecropstep need param cnt %d, input cnt %d err, id/cnt/position/dev/chn/port/maxw/maxh/filepath \n",u8ParamCnt, array_cnt);
            }
            else
            {
                MI_U8  u8GroupParamCnt=array_cnt/u8ParamCnt;
                DBG_INFO("group param cnt %d\n", u8GroupParamCnt);
                for(i=0; i<u8GroupParamCnt; i++)
                {
                    ST_DynamicFuncParam_t  *pstDynamicChangeCropStep= (ST_DynamicFuncParam_t *)malloc(sizeof(ST_DynamicFuncParam_t));
                    memset(pstDynamicChangeCropStep, 0x0, sizeof(ST_DynamicFuncParam_t));

                    pstDynamicChangeCropStep->eFuncType=E_DYNAMIC_CHANGE_CROPSTEPSIZE;
                    pstDynamicChangeCropStep->u16Id = (MI_U16)strtoul(array_str_holder[0+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeCropStep->u32Cnt = (MI_U32)strtoul(array_str_holder[1+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeCropStep->stChangeCropStep.eCropPosition = (ST_CropPosition_e)strtoul(array_str_holder[2+i*u8ParamCnt], 0,0);
                    pstDynamicChangeCropStep->stChangeCropStep.stChnPort.u32DevId = (MI_U32)strtoul(array_str_holder[3+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeCropStep->stChangeCropStep.stChnPort.u32ChnId = (MI_U32)strtoul(array_str_holder[4+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeCropStep->stChangeCropStep.stChnPort.u32PortId = (MI_U32)strtoul(array_str_holder[5+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeCropStep->stChangeCropStep.stMaxWin.u16Width = (MI_U16)strtoul(array_str_holder[6+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeCropStep->stChangeCropStep.stMaxWin.u16Height = (MI_U16)strtoul(array_str_holder[7+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeCropStep->u32Sleep = u32Sleep;

                    MI_U16  u16size = strlen(array_str_holder[8+i*u8ParamCnt]);
                    memcpy(pstDynamicChangeCropStep->stChangeCropStep.FilePath, array_str_holder[8+i*u8ParamCnt], u16size);

                    switch(pstDynamicChangeCropStep->stChangeCropStep.eCropPosition)
                    {
                        case E_MI_VIF_DEV_CROP:
                        case E_MI_VIF_OUTPUTPORT_CROP:
                            pstDynamicChangeCropStep->stChangeCropStep.stChnPort.eModId = E_MI_MODULE_ID_VIF;
                            break;
                        case E_MI_ISP_INPUTPORT_CROP:
                        case E_MI_ISP_OUTPUTPORT_CROP:
                            pstDynamicChangeCropStep->stChangeCropStep.stChnPort.eModId = E_MI_MODULE_ID_ISP;
                            break;
                        case E_MI_SCL_INPUTPORT_CROP:
                        case E_MI_SCL_OUTPUTPORT_CROP:
                            pstDynamicChangeCropStep->stChangeCropStep.stChnPort.eModId = E_MI_MODULE_ID_SCL;
                            break;
                        default:
                            DBG_ERR("cropposition %d err \n", pstDynamicChangeCropStep->stChangeCropStep.eCropPosition);
                            UTStatus = UT_CASE_FAIL;
                            break;
                    }

                    if(u32MaxId < pstDynamicChangeCropStep->u16Id)
                        u32MaxId = pstDynamicChangeCropStep->u16Id;

                    DBG_INFO("changecropstep param id %d, cnt %d, position %d, dev %d, chn %d, port %d, maxw %d, maxh %d, file path %s sleep %d\n", pstDynamicChangeCropStep->u16Id, pstDynamicChangeCropStep->u32Cnt,
                        pstDynamicChangeCropStep->stChangeCropStep.eCropPosition, pstDynamicChangeCropStep->stChangeCropStep.stChnPort.u32DevId, pstDynamicChangeCropStep->stChangeCropStep.stChnPort.u32ChnId,
                        pstDynamicChangeCropStep->stChangeCropStep.stChnPort.u32PortId, pstDynamicChangeCropStep->stChangeCropStep.stMaxWin.u16Width, pstDynamicChangeCropStep->stChangeCropStep.stMaxWin.u16Height, pstDynamicChangeCropStep->stChangeCropStep.FilePath, pstDynamicChangeCropStep->u32Sleep);

                    if(u32RandomCnt ==0)
                        list_add_tail(&pstDynamicChangeCropStep->stDynamicFuncListNode, &gstDynamicTest.stDynamicFuncListHead);
                    else
                        list_add_tail(&pstDynamicChangeCropStep->stDynamicFuncListNode, &stTempListHead);
                }
            }

            free(array_str_holder);
            array_str_holder = NULL;
            free(holder);
            holder = NULL;
        }
        else
        {
            UTStatus = UT_CASE_FAIL;
            DBG_ERR("string to array fail \n");
        }
    }

    u32Sleep = (MI_U32)iniparser_getint(pstDict, ":changeisp3dnrsleep", 1);
    string = iniparser_getstring(pstDict, ":changeisp3dnr", NULL);
    if(string != NULL)
    {
        MI_U8 u8ParamCnt=5;
        printf(">>>>>>>>>>>changeisp3dnr string %s \n", string);
        holder=iniparser_parser_string_to_array(string, &array_str_holder, &array_cnt);
        if(holder !=NULL)
        {
            if(array_cnt%u8ParamCnt !=0)
            {
                UTStatus = UT_CASE_FAIL;
                DBG_ERR("changeisp3dnr need param cnt %d, input cnt %d err, id/cnt/devid/chnid/3dnr \n",u8ParamCnt, array_cnt);
            }
            else
            {
                MI_U8  u8GroupParamCnt=array_cnt/u8ParamCnt;
                DBG_INFO("group param cnt %d\n", u8GroupParamCnt);
                for(i=0; i<u8GroupParamCnt; i++)
                {
                    ST_DynamicFuncParam_t  *pstDynamicChangeIsp3dnr= (ST_DynamicFuncParam_t *)malloc(sizeof(ST_DynamicFuncParam_t));
                    memset(pstDynamicChangeIsp3dnr, 0x0, sizeof(ST_DynamicFuncParam_t));

                    pstDynamicChangeIsp3dnr->eFuncType=E_DYNAMIC_CHANGE_ISP3DNRLEVEL;
                    pstDynamicChangeIsp3dnr->u16Id = (MI_U16)strtoul(array_str_holder[0+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIsp3dnr->u32Cnt = (MI_U32)strtoul(array_str_holder[1+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIsp3dnr->stChangeIsp3dnr.DevId = (MI_U32)strtoul(array_str_holder[2+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIsp3dnr->stChangeIsp3dnr.ChnId = (MI_U32)strtoul(array_str_holder[3+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIsp3dnr->stChangeIsp3dnr.e3dnrlevel = (MI_ISP_3DNR_Level_e)strtoul(array_str_holder[4+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIsp3dnr->u32Sleep = u32Sleep;

                    if(u32MaxId < pstDynamicChangeIsp3dnr->u16Id)
                        u32MaxId = pstDynamicChangeIsp3dnr->u16Id;

                    DBG_INFO("changeisp3dnr param id %d, cnt %d, dev %d, chn %d, 3dnrlevel %d sleep %d\n", pstDynamicChangeIsp3dnr->u16Id, pstDynamicChangeIsp3dnr->u32Cnt,
                        pstDynamicChangeIsp3dnr->stChangeIsp3dnr.DevId, pstDynamicChangeIsp3dnr->stChangeIsp3dnr.ChnId,pstDynamicChangeIsp3dnr->stChangeIsp3dnr.e3dnrlevel,pstDynamicChangeIsp3dnr->u32Sleep);

                    if(u32RandomCnt ==0)
                        list_add_tail(&pstDynamicChangeIsp3dnr->stDynamicFuncListNode, &gstDynamicTest.stDynamicFuncListHead);
                    else
                        list_add_tail(&pstDynamicChangeIsp3dnr->stDynamicFuncListNode, &stTempListHead);
                }
            }

            free(array_str_holder);
            array_str_holder = NULL;
            free(holder);
            holder = NULL;
        }
        else
        {
            UTStatus = UT_CASE_FAIL;
            DBG_ERR("string to array fail \n");
        }
    }

    u32Sleep = (MI_U32)iniparser_getint(pstDict, ":changeispinputcropsleep", 1);
    string = iniparser_getstring(pstDict, ":changeispinputcrop", NULL);
    if(string != NULL)
    {
        MI_U8 u8ParamCnt=8;
        printf(">>>>>>>>>>>changeispinputcrop string %s \n", string);
        holder=iniparser_parser_string_to_array(string, &array_str_holder, &array_cnt);
        if(holder !=NULL)
        {
            if(array_cnt%u8ParamCnt !=0)
            {
                UTStatus = UT_CASE_FAIL;
                DBG_ERR("changeispinputcrop need param cnt %d, input cnt %d err, id/cnt/devid/chnid/crop x/y/w/h \n",u8ParamCnt, array_cnt);
            }
            else
            {
                MI_U8  u8GroupParamCnt=array_cnt/u8ParamCnt;
                DBG_INFO("group param cnt %d\n", u8GroupParamCnt);
                for(i=0; i<u8GroupParamCnt; i++)
                {
                    ST_DynamicFuncParam_t  *pstDynamicChangeIspInputCrop= (ST_DynamicFuncParam_t *)malloc(sizeof(ST_DynamicFuncParam_t));
                    memset(pstDynamicChangeIspInputCrop, 0x0, sizeof(ST_DynamicFuncParam_t));

                    pstDynamicChangeIspInputCrop->eFuncType=E_DYNAMIC_CHANGE_ISPINPUTCROP;
                    pstDynamicChangeIspInputCrop->u16Id = (MI_U16)strtoul(array_str_holder[0+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIspInputCrop->u32Cnt = (MI_U32)strtoul(array_str_holder[1+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIspInputCrop->stChangeIspInputCrop.DevId = (MI_U32)strtoul(array_str_holder[2+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIspInputCrop->stChangeIspInputCrop.ChnId = (MI_U32)strtoul(array_str_holder[3+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIspInputCrop->stChangeIspInputCrop.stCropInfo.u16X = (MI_U16)strtoul(array_str_holder[4+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIspInputCrop->stChangeIspInputCrop.stCropInfo.u16Y = (MI_U16)strtoul(array_str_holder[5+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIspInputCrop->stChangeIspInputCrop.stCropInfo.u16Width = (MI_U16)strtoul(array_str_holder[6+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIspInputCrop->stChangeIspInputCrop.stCropInfo.u16Height = (MI_U16)strtoul(array_str_holder[7+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIspInputCrop->u32Sleep = u32Sleep;

                    if(u32MaxId < pstDynamicChangeIspInputCrop->u16Id)
                        u32MaxId = pstDynamicChangeIspInputCrop->u16Id;

                    DBG_INFO("changeisp3dnr param id %d, cnt %d, dev %d, chn %d, crop (%d,%d,%d,%d), sleep %d\n", pstDynamicChangeIspInputCrop->u16Id, pstDynamicChangeIspInputCrop->u32Cnt,
                        pstDynamicChangeIspInputCrop->stChangeIspInputCrop.DevId, pstDynamicChangeIspInputCrop->stChangeIspInputCrop.ChnId,
                        pstDynamicChangeIspInputCrop->stChangeIspInputCrop.stCropInfo.u16X,pstDynamicChangeIspInputCrop->stChangeIspInputCrop.stCropInfo.u16Y,
                        pstDynamicChangeIspInputCrop->stChangeIspInputCrop.stCropInfo.u16Width,pstDynamicChangeIspInputCrop->stChangeIspInputCrop.stCropInfo.u16Height,pstDynamicChangeIspInputCrop->u32Sleep);

                    if(u32RandomCnt ==0)
                        list_add_tail(&pstDynamicChangeIspInputCrop->stDynamicFuncListNode, &gstDynamicTest.stDynamicFuncListHead);
                    else
                        list_add_tail(&pstDynamicChangeIspInputCrop->stDynamicFuncListNode, &stTempListHead);
                }
            }

            free(array_str_holder);
            array_str_holder = NULL;
            free(holder);
            holder = NULL;
        }
        else
        {
            UTStatus = UT_CASE_FAIL;
            DBG_ERR("string to array fail \n");
        }
    }

    u32Sleep = (MI_U32)iniparser_getint(pstDict, ":changeispoutputparamsleep", 1);
    string = iniparser_getstring(pstDict, ":changeispoutputparam", NULL);
    if(string != NULL)
    {
        MI_U8 u8ParamCnt=11;
        printf(">>>>>>>>>>>changeispoutputparam string %s \n", string);
        holder=iniparser_parser_string_to_array(string, &array_str_holder, &array_cnt);
        if(holder !=NULL)
        {
            if(array_cnt%u8ParamCnt !=0)
            {
                UTStatus = UT_CASE_FAIL;
                DBG_ERR("changeispoutputparam need param cnt %d, input cnt %d err, id/cnt/devid/chnid/portid/pixel/crop x/y/w/h/compress \n",u8ParamCnt, array_cnt);
            }
            else
            {
                MI_U8  u8GroupParamCnt=array_cnt/u8ParamCnt;
                DBG_INFO("group param cnt %d\n", u8GroupParamCnt);
                for(i=0; i<u8GroupParamCnt; i++)
                {
                    ST_DynamicFuncParam_t  *pstDynamicChangeIspOutputParam= (ST_DynamicFuncParam_t *)malloc(sizeof(ST_DynamicFuncParam_t));
                    memset(pstDynamicChangeIspOutputParam, 0x0, sizeof(ST_DynamicFuncParam_t));

                    pstDynamicChangeIspOutputParam->eFuncType=E_DYNAMIC_CHANGE_ISPOUTPARAM;
                    pstDynamicChangeIspOutputParam->u16Id = (MI_U16)strtoul(array_str_holder[0+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIspOutputParam->u32Cnt = (MI_U32)strtoul(array_str_holder[1+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIspOutputParam->stChangeIspOutputParam.DevId = (MI_U32)strtoul(array_str_holder[2+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIspOutputParam->stChangeIspOutputParam.ChnId = (MI_U32)strtoul(array_str_holder[3+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIspOutputParam->stChangeIspOutputParam.PortId = (MI_U32)strtoul(array_str_holder[4+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIspOutputParam->stChangeIspOutputParam.stOutPortParam.ePixelFormat = ST_GetIniPixel(array_str_holder[5+i*u8ParamCnt]);
                    pstDynamicChangeIspOutputParam->stChangeIspOutputParam.stOutPortParam.stCropRect.u16X = (MI_U16)strtoul(array_str_holder[6+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIspOutputParam->stChangeIspOutputParam.stOutPortParam.stCropRect.u16Y = (MI_U16)strtoul(array_str_holder[7+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIspOutputParam->stChangeIspOutputParam.stOutPortParam.stCropRect.u16Width = (MI_U16)strtoul(array_str_holder[8+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIspOutputParam->stChangeIspOutputParam.stOutPortParam.stCropRect.u16Height = (MI_U16)strtoul(array_str_holder[9+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIspOutputParam->stChangeIspOutputParam.stOutPortParam.eCompressMode = (MI_SYS_CompressMode_e)strtoul(array_str_holder[10+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIspOutputParam->u32Sleep = u32Sleep;

                    if(u32MaxId < pstDynamicChangeIspOutputParam->u16Id)
                        u32MaxId = pstDynamicChangeIspOutputParam->u16Id;

                    DBG_INFO("changeispoutputparam param id %d, cnt %d, dev %d, chn %d, port %d, pixel %d, crop (%d,%d,%d,%d), compress %d,sleep %d\n", pstDynamicChangeIspOutputParam->u16Id, pstDynamicChangeIspOutputParam->u32Cnt,
                        pstDynamicChangeIspOutputParam->stChangeIspOutputParam.DevId, pstDynamicChangeIspOutputParam->stChangeIspOutputParam.ChnId,
                        pstDynamicChangeIspOutputParam->stChangeIspOutputParam.PortId, pstDynamicChangeIspOutputParam->stChangeIspOutputParam.stOutPortParam.ePixelFormat,
                        pstDynamicChangeIspOutputParam->stChangeIspOutputParam.stOutPortParam.stCropRect.u16X, pstDynamicChangeIspOutputParam->stChangeIspOutputParam.stOutPortParam.stCropRect.u16Y,
                        pstDynamicChangeIspOutputParam->stChangeIspOutputParam.stOutPortParam.stCropRect.u16Width, pstDynamicChangeIspOutputParam->stChangeIspOutputParam.stOutPortParam.stCropRect.u16Height,
                        pstDynamicChangeIspOutputParam->stChangeIspOutputParam.stOutPortParam.eCompressMode,pstDynamicChangeIspOutputParam->u32Sleep);

                    if(u32RandomCnt ==0)
                        list_add_tail(&pstDynamicChangeIspOutputParam->stDynamicFuncListNode, &gstDynamicTest.stDynamicFuncListHead);
                    else
                        list_add_tail(&pstDynamicChangeIspOutputParam->stDynamicFuncListNode, &stTempListHead);
                }
            }

            free(array_str_holder);
            array_str_holder = NULL;
            free(holder);
            holder = NULL;
        }
        else
        {
            UTStatus = UT_CASE_FAIL;
            DBG_ERR("string to array fail \n");
        }
    }

    u32Sleep = (MI_U32)iniparser_getint(pstDict, ":changeispzoomsleep", 1);
    string = iniparser_getstring(pstDict, ":changeispzoom", NULL);
    if(string != NULL)
    {
        MI_U8 u8ParamCnt=17;
        printf(">>>>>>>>>>>changeispzoom string %s \n", string);
        holder=iniparser_parser_string_to_array(string, &array_str_holder, &array_cnt);
        if(holder !=NULL)
        {
            if(array_cnt%u8ParamCnt !=0)
            {
                UTStatus = UT_CASE_FAIL;
                DBG_ERR("changeispzoom need param cnt %d, input cnt %d err, id/cnt/devid/chnid/rev/start x,y,w,h/dest x,y,w,h/step x,y,w,h \n",u8ParamCnt, array_cnt);
            }
            else
            {
                MI_U8  u8GroupParamCnt=array_cnt/u8ParamCnt;
                DBG_INFO("group param cnt %d\n", u8GroupParamCnt);
                for(i=0; i<u8GroupParamCnt; i++)
                {
                    ST_DynamicFuncParam_t  *pstDynamicChangeIspZoom= (ST_DynamicFuncParam_t *)malloc(sizeof(ST_DynamicFuncParam_t));
                    memset(pstDynamicChangeIspZoom, 0x0, sizeof(ST_DynamicFuncParam_t));

                    pstDynamicChangeIspZoom->eFuncType=E_DYNAMIC_CHANGE_ISPZOOM;
                    pstDynamicChangeIspZoom->u16Id = (MI_U16)strtoul(array_str_holder[0+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIspZoom->u32Cnt = (MI_U32)strtoul(array_str_holder[1+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIspZoom->stChangeIspZoom.DevId = (MI_U32)strtoul(array_str_holder[2+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIspZoom->stChangeIspZoom.ChnId = (MI_U32)strtoul(array_str_holder[3+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIspZoom->stChangeIspZoom.bRev = (MI_U32)strtoul(array_str_holder[4+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIspZoom->stChangeIspZoom.stStart.u16X = (MI_U32)strtoul(array_str_holder[5+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIspZoom->stChangeIspZoom.stStart.u16Y = (MI_U32)strtoul(array_str_holder[6+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIspZoom->stChangeIspZoom.stStart.u16Width = (MI_U32)strtoul(array_str_holder[7+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIspZoom->stChangeIspZoom.stStart.u16Height = (MI_U32)strtoul(array_str_holder[8+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIspZoom->stChangeIspZoom.stDest.u16X = (MI_U32)strtoul(array_str_holder[9+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIspZoom->stChangeIspZoom.stDest.u16Y = (MI_U32)strtoul(array_str_holder[10+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIspZoom->stChangeIspZoom.stDest.u16Width = (MI_U32)strtoul(array_str_holder[11+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIspZoom->stChangeIspZoom.stDest.u16Height = (MI_U32)strtoul(array_str_holder[12+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIspZoom->stChangeIspZoom.stStepWH.u16X = (MI_U32)strtoul(array_str_holder[13+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIspZoom->stChangeIspZoom.stStepWH.u16Y = (MI_U32)strtoul(array_str_holder[14+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIspZoom->stChangeIspZoom.stStepWH.u16Width = (MI_U32)strtoul(array_str_holder[15+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIspZoom->stChangeIspZoom.stStepWH.u16Height = (MI_U32)strtoul(array_str_holder[16+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIspZoom->u32Sleep = u32Sleep;

                    if(u32MaxId < pstDynamicChangeIspZoom->u16Id)
                        u32MaxId = pstDynamicChangeIspZoom->u16Id;

                    DBG_INFO("changeispzoom param id %d, cnt %d, dev %d, chn %d, reverse %d, start(%d,%d,%d,%d), dest(%d,%d,%d,%d), step(%d,%d,%d,%d) sleep %d\n",
                        pstDynamicChangeIspZoom->u16Id, pstDynamicChangeIspZoom->u32Cnt,pstDynamicChangeIspZoom->stChangeIspZoom.DevId,
                        pstDynamicChangeIspZoom->stChangeIspZoom.ChnId,pstDynamicChangeIspZoom->stChangeIspZoom.bRev,
                        pstDynamicChangeIspZoom->stChangeIspZoom.stStart.u16X, pstDynamicChangeIspZoom->stChangeIspZoom.stStart.u16Y,
                        pstDynamicChangeIspZoom->stChangeIspZoom.stStart.u16Width, pstDynamicChangeIspZoom->stChangeIspZoom.stStart.u16Height,
                        pstDynamicChangeIspZoom->stChangeIspZoom.stDest.u16X, pstDynamicChangeIspZoom->stChangeIspZoom.stDest.u16Y,
                        pstDynamicChangeIspZoom->stChangeIspZoom.stDest.u16Width, pstDynamicChangeIspZoom->stChangeIspZoom.stDest.u16Height,
                        pstDynamicChangeIspZoom->stChangeIspZoom.stStepWH.u16X, pstDynamicChangeIspZoom->stChangeIspZoom.stStepWH.u16Y,
                        pstDynamicChangeIspZoom->stChangeIspZoom.stStepWH.u16Width, pstDynamicChangeIspZoom->stChangeIspZoom.stStepWH.u16Height,
                        pstDynamicChangeIspZoom->u32Sleep);

                    if(u32RandomCnt ==0)
                        list_add_tail(&pstDynamicChangeIspZoom->stDynamicFuncListNode, &gstDynamicTest.stDynamicFuncListHead);
                    else
                        list_add_tail(&pstDynamicChangeIspZoom->stDynamicFuncListNode, &stTempListHead);
                }
            }

            free(array_str_holder);
            array_str_holder = NULL;
            free(holder);
            holder = NULL;
        }
        else
        {
            UTStatus = UT_CASE_FAIL;
            DBG_ERR("string to array fail \n");
        }
    }

    u32Sleep = (MI_U32)iniparser_getint(pstDict, ":changeispskipframesleep", 1);
    string = iniparser_getstring(pstDict, ":changeispskipframe", NULL);
    if(string != NULL)
    {
        MI_U8 u8ParamCnt=5;
        printf(">>>>>>>>>>>changeispskipframe string %s \n", string);
        holder=iniparser_parser_string_to_array(string, &array_str_holder, &array_cnt);
        if(holder !=NULL)
        {
            if(array_cnt%u8ParamCnt !=0)
            {
                UTStatus = UT_CASE_FAIL;
                DBG_ERR("changeispskipframe need param cnt %d, input cnt %d err, id/cnt/devid/chnid/FrameNum \n",u8ParamCnt, array_cnt);
            }
            else
            {
                MI_U8  u8GroupParamCnt=array_cnt/u8ParamCnt;
                DBG_INFO("group param cnt %d\n", u8GroupParamCnt);
                for(i=0; i<u8GroupParamCnt; i++)
                {
                    ST_DynamicFuncParam_t  *pstDynamicChangeIspSkipFrame= (ST_DynamicFuncParam_t *)malloc(sizeof(ST_DynamicFuncParam_t));
                    memset(pstDynamicChangeIspSkipFrame, 0x0, sizeof(ST_DynamicFuncParam_t));

                    pstDynamicChangeIspSkipFrame->eFuncType=E_DYNAMIC_CHANGE_ISPSKIPFRAME;
                    pstDynamicChangeIspSkipFrame->u16Id = (MI_U16)strtoul(array_str_holder[0+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIspSkipFrame->u32Cnt = (MI_U32)strtoul(array_str_holder[1+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIspSkipFrame->stChangeIspSkipFrame.DevId = (MI_ISP_DEV)strtoul(array_str_holder[2+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIspSkipFrame->stChangeIspSkipFrame.ChnId = (MI_ISP_CHANNEL)strtoul(array_str_holder[3+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIspSkipFrame->stChangeIspSkipFrame.u32FrameNum = (MI_U32)strtoul(array_str_holder[4+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeIspSkipFrame->u32Sleep = u32Sleep;

                    if(u32MaxId < pstDynamicChangeIspSkipFrame->u16Id)
                        u32MaxId = pstDynamicChangeIspSkipFrame->u16Id;

                    DBG_INFO("changeispskipframe param id %d, cnt %d, dev %d, chn %d,FrameNum %d sleep %d\n",pstDynamicChangeIspSkipFrame->u16Id, pstDynamicChangeIspSkipFrame->u32Cnt,
                    pstDynamicChangeIspSkipFrame->stChangeIspSkipFrame.DevId,pstDynamicChangeIspSkipFrame->stChangeIspSkipFrame.ChnId,pstDynamicChangeIspSkipFrame->stChangeIspSkipFrame.u32FrameNum,
                    pstDynamicChangeIspSkipFrame->u32Sleep);

                    if(u32RandomCnt ==0)
                        list_add_tail(&pstDynamicChangeIspSkipFrame->stDynamicFuncListNode, &gstDynamicTest.stDynamicFuncListHead);
                    else
                        list_add_tail(&pstDynamicChangeIspSkipFrame->stDynamicFuncListNode, &stTempListHead);
                }
            }

            free(array_str_holder);
            array_str_holder = NULL;
            free(holder);
            holder = NULL;
        }
        else
        {
            UTStatus = UT_CASE_FAIL;
            DBG_ERR("string to array fail \n");
        }
    }

    u32Sleep = (MI_U32)iniparser_getint(pstDict, ":changeoutputsizestepsleep", 1);
    string = iniparser_getstring(pstDict, ":changeoutputsizestep", NULL);
    if(string != NULL)
    {
        MI_U8 u8ParamCnt=9;
        printf(">>>>>>>>>>>changeoutputsizestep string %s \n", string);
        holder=iniparser_parser_string_to_array(string, &array_str_holder, &array_cnt);
        if(holder !=NULL)
        {
            if(array_cnt%u8ParamCnt !=0)
            {
                UTStatus = UT_CASE_FAIL;
                DBG_ERR("changeoutputsizestep need param cnt %d, input cnt %d err, id/cnt/ModId/devid/chnid/portid/maxw/maxh/filepath \n",u8ParamCnt, array_cnt);
            }
            else
            {
                MI_U8  u8GroupParamCnt=array_cnt/u8ParamCnt;
                DBG_INFO("group param cnt %d\n", u8GroupParamCnt);
                for(i=0; i<u8GroupParamCnt; i++)
                {
                    ST_DynamicFuncParam_t  *pstDynamicChangeOutputSizeStep= (ST_DynamicFuncParam_t *)malloc(sizeof(ST_DynamicFuncParam_t));
                    memset(pstDynamicChangeOutputSizeStep, 0x0, sizeof(ST_DynamicFuncParam_t));

                    pstDynamicChangeOutputSizeStep->eFuncType=E_DYNAMIC_CHANGE_OUTPUTSTEPSIZE;
                    pstDynamicChangeOutputSizeStep->u16Id = (MI_U16)strtoul(array_str_holder[0+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeOutputSizeStep->u32Cnt = (MI_U32)strtoul(array_str_holder[1+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeOutputSizeStep->stChangeOutputSizeStep.stChnPort.eModId = (MI_ModuleId_e)strtoul(array_str_holder[2+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeOutputSizeStep->stChangeOutputSizeStep.stChnPort.u32DevId = (MI_U32)strtoul(array_str_holder[3+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeOutputSizeStep->stChangeOutputSizeStep.stChnPort.u32ChnId = (MI_U32)strtoul(array_str_holder[4+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeOutputSizeStep->stChangeOutputSizeStep.stChnPort.u32PortId = (MI_U32)strtoul(array_str_holder[5+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeOutputSizeStep->stChangeOutputSizeStep.stMaxWin.u16Width = (MI_U16)strtoul(array_str_holder[6+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeOutputSizeStep->stChangeOutputSizeStep.stMaxWin.u16Height = (MI_U16)strtoul(array_str_holder[7+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeOutputSizeStep->u32Sleep = u32Sleep;

                    MI_U16  u16size = strlen(array_str_holder[8+i*u8ParamCnt]);
                    memcpy(pstDynamicChangeOutputSizeStep->stChangeOutputSizeStep.FilePath, array_str_holder[8+i*u8ParamCnt], u16size);

                    if(u32MaxId < pstDynamicChangeOutputSizeStep->u16Id)
                        u32MaxId = pstDynamicChangeOutputSizeStep->u16Id;

                    DBG_INFO("changeoutputsizestep param id %d, cnt %d, ModId %d, dev %d, chn %d, port %d, maxw %d, maxh %d, file %s sleep %d\n", pstDynamicChangeOutputSizeStep->u16Id, pstDynamicChangeOutputSizeStep->u32Cnt,
                        pstDynamicChangeOutputSizeStep->stChangeOutputSizeStep.stChnPort.eModId,pstDynamicChangeOutputSizeStep->stChangeOutputSizeStep.stChnPort.u32DevId, pstDynamicChangeOutputSizeStep->stChangeOutputSizeStep.stChnPort.u32ChnId,
                        pstDynamicChangeOutputSizeStep->stChangeOutputSizeStep.stChnPort.u32PortId, pstDynamicChangeOutputSizeStep->stChangeOutputSizeStep.stMaxWin.u16Width, pstDynamicChangeOutputSizeStep->stChangeOutputSizeStep.stMaxWin.u16Height,
                        pstDynamicChangeOutputSizeStep->stChangeOutputSizeStep.FilePath,pstDynamicChangeOutputSizeStep->u32Sleep);

                    if(u32RandomCnt ==0)
                        list_add_tail(&pstDynamicChangeOutputSizeStep->stDynamicFuncListNode, &gstDynamicTest.stDynamicFuncListHead);
                    else
                        list_add_tail(&pstDynamicChangeOutputSizeStep->stDynamicFuncListNode, &stTempListHead);
                }
            }

            free(array_str_holder);
            array_str_holder = NULL;
            free(holder);
            holder = NULL;
        }
        else
        {
            UTStatus = UT_CASE_FAIL;
            DBG_ERR("string to array fail \n");
        }
    }

    u32Sleep = (MI_U32)iniparser_getint(pstDict, ":stretchbuffsleep", 1);
    string = iniparser_getstring(pstDict, ":stretchbuff", NULL);
    if(string != NULL)
    {
        MI_U8 u8ParamCnt=15;
        printf(">>>>>>>>>>>stretchbuff string %s \n", string);
        holder=iniparser_parser_string_to_array(string, &array_str_holder, &array_cnt);
        if(holder != NULL)
        {
            if((array_cnt % u8ParamCnt) != 0)
            {
                UTStatus = UT_CASE_FAIL;
                DBG_ERR("stretchbuff need param cnt %d, input cnt %d err, id/cnt/file/pixel/inw/inh/instride/cropx/cropy/cropw/croph/outw/outh/outpixel \n",u8ParamCnt, array_cnt);
            }
            else
            {
                MI_U8  u8GroupParamCnt=array_cnt/u8ParamCnt;
                DBG_INFO("group param cnt %d\n", u8GroupParamCnt);
                for(i=0; i<u8GroupParamCnt; i++)
                {
                    ST_DynamicFuncParam_t  *pstStretchBuff= (ST_DynamicFuncParam_t *)malloc(sizeof(ST_DynamicFuncParam_t));
                    memset(pstStretchBuff, 0x0, sizeof(ST_DynamicFuncParam_t));

                    pstStretchBuff->eFuncType = E_DYNAMIC_CHANGE_SCLSTRETCHBUFF;
                    pstStretchBuff->u16Id = (MI_U16)strtoul(array_str_holder[0+i*u8ParamCnt], 0, 0);
                    pstStretchBuff->u32Cnt = (MI_U32)strtoul(array_str_holder[1+i*u8ParamCnt], 0, 0);

                    MI_U16  u16size = strlen(array_str_holder[2+i*u8ParamCnt]);
                    memcpy(pstStretchBuff->stChangeStretchBuff.sFileInputPath, array_str_holder[2+i*u8ParamCnt], u16size);
                    pstStretchBuff->stChangeStretchBuff.eInputPixel = ST_GetIniPixel(array_str_holder[3+i*u8ParamCnt]);
                    pstStretchBuff->stChangeStretchBuff.stInputWinSize.u16Width = (MI_U16)strtoul(array_str_holder[4+i*u8ParamCnt], 0, 0);
                    pstStretchBuff->stChangeStretchBuff.stInputWinSize.u16Height = (MI_U16)strtoul(array_str_holder[5+i*u8ParamCnt], 0, 0);
                    pstStretchBuff->stChangeStretchBuff.u32InputStride = (MI_U32)strtoul(array_str_holder[6+i*u8ParamCnt], 0, 0);
                    pstStretchBuff->stChangeStretchBuff.stCropWin.u16X = (MI_U16)strtoul(array_str_holder[7+i*u8ParamCnt], 0, 0);
                    pstStretchBuff->stChangeStretchBuff.stCropWin.u16Y = (MI_U16)strtoul(array_str_holder[8+i*u8ParamCnt], 0, 0);
                    pstStretchBuff->stChangeStretchBuff.stCropWin.u16Width = (MI_U16)strtoul(array_str_holder[9+i*u8ParamCnt], 0, 0);
                    pstStretchBuff->stChangeStretchBuff.stCropWin.u16Height = (MI_U16)strtoul(array_str_holder[10+i*u8ParamCnt], 0, 0);
                    pstStretchBuff->stChangeStretchBuff.stOutputWinSize.u16Width = (MI_U16)strtoul(array_str_holder[11+i*u8ParamCnt], 0, 0);
                    pstStretchBuff->stChangeStretchBuff.stOutputWinSize.u16Height = (MI_U16)strtoul(array_str_holder[12+i*u8ParamCnt], 0, 0);
                    pstStretchBuff->stChangeStretchBuff.eOutputPixel = ST_GetIniPixel(array_str_holder[13+i*u8ParamCnt]);

                    u16size = strlen(array_str_holder[14+i*u8ParamCnt]);
                    memcpy(pstStretchBuff->stChangeStretchBuff.sFileOutputPath, array_str_holder[14+i*u8ParamCnt], u16size);
                    pstStretchBuff->u32Sleep = u32Sleep;

                    if(u32MaxId < pstStretchBuff->u16Id)
                    {
                        u32MaxId = pstStretchBuff->u16Id;
                    }

                    DBG_INFO("stretch buffer id %d,cnt %d,file %s,pixel%d,insize(%dx%d),instride %d,crop(%d,%d,%d,%d),outsize(%dx%d),outpixel%d,outputpath %s\n",pstStretchBuff->u16Id,pstStretchBuff->u32Cnt,
                        pstStretchBuff->stChangeStretchBuff.sFileInputPath,pstStretchBuff->stChangeStretchBuff.eInputPixel,
                        pstStretchBuff->stChangeStretchBuff.stInputWinSize.u16Width,pstStretchBuff->stChangeStretchBuff.stInputWinSize.u16Height,pstStretchBuff->stChangeStretchBuff.u32InputStride,
                        pstStretchBuff->stChangeStretchBuff.stCropWin.u16X,pstStretchBuff->stChangeStretchBuff.stCropWin.u16Y,pstStretchBuff->stChangeStretchBuff.stCropWin.u16Width,pstStretchBuff->stChangeStretchBuff.stCropWin.u16Height,
                        pstStretchBuff->stChangeStretchBuff.stOutputWinSize.u16Width,pstStretchBuff->stChangeStretchBuff.stOutputWinSize.u16Width,pstStretchBuff->stChangeStretchBuff.eOutputPixel,pstStretchBuff->stChangeStretchBuff.sFileOutputPath);

                        if(u32RandomCnt ==0)
                            list_add_tail(&pstStretchBuff->stDynamicFuncListNode, &gstDynamicTest.stDynamicFuncListHead);
                        else
                            list_add_tail(&pstStretchBuff->stDynamicFuncListNode, &stTempListHead);
                }
            }

            free(array_str_holder);
            array_str_holder = NULL;
            free(holder);
            holder = NULL;
        }
        else
        {
            UTStatus = UT_CASE_FAIL;
            DBG_ERR("string to array fail \n");
        }
    }

    u32Sleep = (MI_U32)iniparser_getint(pstDict, ":changescloutparamsleep", 1);
    string = iniparser_getstring(pstDict, ":changescloutparam", NULL);
    if(string != NULL)
    {
        MI_U8 u8ParamCnt=15;
        printf(">>>>>>>>>>>scloutparam string %s \n", string);
        holder=iniparser_parser_string_to_array(string, &array_str_holder, &array_cnt);
        if(holder != NULL)
        {
            if((array_cnt % u8ParamCnt) != 0)
            {
                UTStatus = UT_CASE_FAIL;
                DBG_ERR("changescloutparam need param cnt %d, input cnt %d err, id/cnt/devid/chnid/portid/mirror/flip/cropx/cropy/cropw/croph/outw/outh/outpixel/compress \n",u8ParamCnt, array_cnt);
            }
            else
            {
                MI_U8  u8GroupParamCnt=array_cnt/u8ParamCnt;
                DBG_INFO("group param cnt %d\n", u8GroupParamCnt);
                for(i=0; i<u8GroupParamCnt; i++)
                {
                    ST_DynamicFuncParam_t  *pstSclOutParam= (ST_DynamicFuncParam_t *)malloc(sizeof(ST_DynamicFuncParam_t));
                    memset(pstSclOutParam, 0x0, sizeof(ST_DynamicFuncParam_t));

                    pstSclOutParam->eFuncType = E_DYNAMIC_CHANGE_SCLOUTPUTPARAM;
                    pstSclOutParam->u16Id = (MI_U16)strtoul(array_str_holder[0+i*u8ParamCnt], 0, 0);
                    pstSclOutParam->u32Cnt = (MI_U32)strtoul(array_str_holder[1+i*u8ParamCnt], 0, 0);
                    pstSclOutParam->stChangeSclOutputParam.DevId = (MI_SYS_PixelFormat_e)strtoul(array_str_holder[2+i*u8ParamCnt], 0, 0);
                    pstSclOutParam->stChangeSclOutputParam.ChnId = (MI_U16)strtoul(array_str_holder[3+i*u8ParamCnt], 0, 0);
                    pstSclOutParam->stChangeSclOutputParam.PortId = (MI_U16)strtoul(array_str_holder[4+i*u8ParamCnt], 0, 0);
                    pstSclOutParam->stChangeSclOutputParam.stOutputParam.bMirror = (MI_U32)strtoul(array_str_holder[5+i*u8ParamCnt], 0, 0);
                    pstSclOutParam->stChangeSclOutputParam.stOutputParam.bFlip = (MI_U16)strtoul(array_str_holder[6+i*u8ParamCnt], 0, 0);
                    pstSclOutParam->stChangeSclOutputParam.stOutputParam.stSCLOutCropRect.u16X = (MI_U16)strtoul(array_str_holder[7+i*u8ParamCnt], 0, 0);
                    pstSclOutParam->stChangeSclOutputParam.stOutputParam.stSCLOutCropRect.u16Y = (MI_U16)strtoul(array_str_holder[8+i*u8ParamCnt], 0, 0);
                    pstSclOutParam->stChangeSclOutputParam.stOutputParam.stSCLOutCropRect.u16Width = (MI_U16)strtoul(array_str_holder[9+i*u8ParamCnt], 0, 0);
                    pstSclOutParam->stChangeSclOutputParam.stOutputParam.stSCLOutCropRect.u16Height = (MI_U16)strtoul(array_str_holder[10+i*u8ParamCnt], 0, 0);
                    pstSclOutParam->stChangeSclOutputParam.stOutputParam.stSCLOutputSize.u16Width = (MI_U16)strtoul(array_str_holder[11+i*u8ParamCnt], 0, 0);
                    pstSclOutParam->stChangeSclOutputParam.stOutputParam.stSCLOutputSize.u16Height = (MI_U16)strtoul(array_str_holder[12+i*u8ParamCnt], 0, 0);
                    pstSclOutParam->stChangeSclOutputParam.stOutputParam.ePixelFormat = ST_GetIniPixel(array_str_holder[13+i*u8ParamCnt]);
                    pstSclOutParam->stChangeSclOutputParam.stOutputParam.eCompressMode = (MI_SYS_CompressMode_e)strtoul(array_str_holder[14+i*u8ParamCnt], 0, 0);
                    pstSclOutParam->u32Sleep = u32Sleep;

                    if(u32MaxId < pstSclOutParam->u16Id)
                    {
                        u32MaxId = pstSclOutParam->u16Id;
                    }

                    DBG_INFO("changescloutparam id %d,cnt %d,devid%d chnid%d portid%d mirror%d flip%d,crop(%d,%d,%d,%d),outsize(%dx%d),outpixel%d,compress%d\n",pstSclOutParam->u16Id,pstSclOutParam->u32Cnt,
                        pstSclOutParam->stChangeSclOutputParam.DevId, pstSclOutParam->stChangeSclOutputParam.ChnId, pstSclOutParam->stChangeSclOutputParam.PortId, pstSclOutParam->stChangeSclOutputParam.stOutputParam.bMirror,
                        pstSclOutParam->stChangeSclOutputParam.stOutputParam.bFlip, pstSclOutParam->stChangeSclOutputParam.stOutputParam.stSCLOutCropRect.u16X,pstSclOutParam->stChangeSclOutputParam.stOutputParam.stSCLOutCropRect.u16Y,
                        pstSclOutParam->stChangeSclOutputParam.stOutputParam.stSCLOutCropRect.u16Width,pstSclOutParam->stChangeSclOutputParam.stOutputParam.stSCLOutCropRect.u16Height,
                        pstSclOutParam->stChangeSclOutputParam.stOutputParam.stSCLOutputSize.u16Width, pstSclOutParam->stChangeSclOutputParam.stOutputParam.stSCLOutputSize.u16Height,
                        pstSclOutParam->stChangeSclOutputParam.stOutputParam.ePixelFormat,pstSclOutParam->stChangeSclOutputParam.stOutputParam.eCompressMode);

                    if(u32RandomCnt ==0)
                        list_add_tail(&pstSclOutParam->stDynamicFuncListNode, &gstDynamicTest.stDynamicFuncListHead);
                    else
                        list_add_tail(&pstSclOutParam->stDynamicFuncListNode, &stTempListHead);
                }
            }

            free(array_str_holder);
            array_str_holder = NULL;
            free(holder);
            holder = NULL;
        }
        else
        {
            UTStatus = UT_CASE_FAIL;
            DBG_ERR("string to array fail \n");
        }
    }

    u32Sleep = (MI_U32)iniparser_getint(pstDict, ":changeHwResetsleep", 1);
    string = iniparser_getstring(pstDict, ":changeHwReset", NULL);
    if(string != NULL)
    {
        MI_U8 u8ParamCnt=6;
        printf(">>>>>>>>>>>HwReset string %s \n", string);
        holder=iniparser_parser_string_to_array(string, &array_str_holder, &array_cnt);
        if(holder != NULL)
        {
            if((array_cnt % u8ParamCnt) != 0)
            {
                UTStatus = UT_CASE_FAIL;
                DBG_ERR("HwReset need param cnt %d, input cnt %d err, id/cnt/modid/devid/chnid/pos \n",u8ParamCnt, array_cnt);
            }
            else
            {
                MI_U8  u8GroupParamCnt=array_cnt/u8ParamCnt;
                DBG_INFO("group param cnt %d\n", u8GroupParamCnt);
                for(i=0; i<u8GroupParamCnt; i++)
                {
                    ST_DynamicFuncParam_t  *pstHwReset = (ST_DynamicFuncParam_t *)malloc(sizeof(ST_DynamicFuncParam_t));
                    memset(pstHwReset, 0x0, sizeof(ST_DynamicFuncParam_t));

                    pstHwReset->eFuncType = E_DYNAMIC_CHANGE_HWRESET;
                    pstHwReset->u16Id = (MI_U16)strtoul(array_str_holder[0+i*u8ParamCnt], 0, 0);
                    pstHwReset->u32Cnt = (MI_U32)strtoul(array_str_holder[1+i*u8ParamCnt], 0, 0);
                    pstHwReset->stChangeHwReset.eModId = (MI_ModuleId_e)strtoul(array_str_holder[2+i*u8ParamCnt], 0, 0);
                    pstHwReset->stChangeHwReset.DevId = (MI_U32)strtoul(array_str_holder[3+i*u8ParamCnt], 0, 0);
                    pstHwReset->stChangeHwReset.ChnId = (MI_U32)strtoul(array_str_holder[4+i*u8ParamCnt], 0, 0);
                    pstHwReset->stChangeHwReset.u8Pos = (MI_U8)strtoul(array_str_holder[5+i*u8ParamCnt], 0, 0);
                    pstHwReset->u32Sleep = u32Sleep;

                    if(u32MaxId < pstHwReset->u16Id)
                    {
                        u32MaxId = pstHwReset->u16Id;
                    }

                    DBG_INFO("HwReset id %d,cnt %d, eModId%d devid%d chnid%d Pos%d\n",pstHwReset->u16Id,pstHwReset->u32Cnt,
                        pstHwReset->stChangeHwReset.eModId, pstHwReset->stChangeHwReset.DevId, pstHwReset->stChangeHwReset.ChnId,
                        pstHwReset->stChangeHwReset.u8Pos);

                    if(u32RandomCnt ==0)
                        list_add_tail(&pstHwReset->stDynamicFuncListNode, &gstDynamicTest.stDynamicFuncListHead);
                    else
                        list_add_tail(&pstHwReset->stDynamicFuncListNode, &stTempListHead);
                }
            }

            free(array_str_holder);
            array_str_holder = NULL;
            free(holder);
            holder = NULL;
        }
        else
        {
            UTStatus = UT_CASE_FAIL;
            DBG_ERR("string to array fail \n");
        }
    }

    u32Sleep = (MI_U32)iniparser_getint(pstDict, ":changesclrotatesleep", 1);
    string = iniparser_getstring(pstDict, ":changeSclRot", 0);
    if(string != NULL)
    {
        MI_U8 u8ParamCnt=5;
        printf(">>>>>>>>>>>changesclrotate string %s \n", string);
        holder=iniparser_parser_string_to_array(string, &array_str_holder, &array_cnt);
        if(holder !=NULL)
        {
            if(array_cnt%u8ParamCnt !=0)
            {
                UTStatus = UT_CASE_FAIL;
                DBG_ERR("changesclrotate : cnt %d, input cnt %d err, id/cnt/devid/chnid/rot  \n",u8ParamCnt, array_cnt);
            }
            else
            {
                MI_U8  u8GroupParamCnt=array_cnt/u8ParamCnt;
                for(i=0; i<u8GroupParamCnt; i++)
                {
                    ST_DynamicFuncParam_t  *pstDynamicChangeSclRotate = (ST_DynamicFuncParam_t *)malloc(sizeof(ST_DynamicFuncParam_t));
                    memset(pstDynamicChangeSclRotate, 0x0, sizeof(ST_DynamicFuncParam_t));

                    pstDynamicChangeSclRotate->eFuncType=E_DYNAMIC_CHANGE_SCLROTATE;
                    pstDynamicChangeSclRotate->u16Id = (MI_U16)strtoul(array_str_holder[0+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeSclRotate->u32Cnt = (MI_U32)strtoul(array_str_holder[1+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeSclRotate->stChangeSclRotate.DevId = (MI_U32)strtoul(array_str_holder[2+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeSclRotate->stChangeSclRotate.ChnId = (MI_U32)strtoul(array_str_holder[3+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeSclRotate->stChangeSclRotate.eRot = (MI_SYS_Rotate_e)strtoul(array_str_holder[4+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeSclRotate->u32Sleep=u32Sleep;
                    DBG_INFO("change scl rot %d >>>>>>>>>>>>>>>\n",  pstDynamicChangeSclRotate->stChangeSclRotate.eRot);
                    DBG_INFO("change scl rotate id %d, cnt %d, dev %d, chn %d, rot %d, sleep %d\n", pstDynamicChangeSclRotate->u16Id, pstDynamicChangeSclRotate->u32Cnt,
                        pstDynamicChangeSclRotate->stChangeSclRotate.DevId, pstDynamicChangeSclRotate->stChangeSclRotate.ChnId,
                        pstDynamicChangeSclRotate->stChangeSclRotate.eRot, pstDynamicChangeSclRotate->u32Sleep);

                    if(u32MaxId < pstDynamicChangeSclRotate->u16Id)
                    {
                        u32MaxId = pstDynamicChangeSclRotate->u16Id;
                    }

                    if(u32RandomCnt ==0)
                        list_add_tail(&pstDynamicChangeSclRotate->stDynamicFuncListNode, &gstDynamicTest.stDynamicFuncListHead);
                    else
                        list_add_tail(&pstDynamicChangeSclRotate->stDynamicFuncListNode, &stTempListHead);
                }
            }

            free(array_str_holder);
            array_str_holder = NULL;
            free(holder);
            holder = NULL;
        }
        else
        {
            UTStatus = UT_CASE_FAIL;
            DBG_ERR("string to array fail \n");
        }
    }

    u32Sleep = (MI_U32)iniparser_getint(pstDict, ":changesclinputcropsleep", 1);
    string = iniparser_getstring(pstDict, ":changesclinputcrop", 0);
    if(string != NULL)
    {
        MI_U8 u8ParamCnt=8;
        printf(">>>>>>>>>>>changesclinputcrop string %s \n", string);
        holder=iniparser_parser_string_to_array(string, &array_str_holder, &array_cnt);
        if(holder !=NULL)
        {
            if(array_cnt%u8ParamCnt !=0)
            {
                UTStatus = UT_CASE_FAIL;
                DBG_ERR("changesclinputcrop : cnt %d, input cnt %d err, id/cnt/devid/chnid/rot  \n",u8ParamCnt, array_cnt);
            }
            else
            {
                MI_U8  u8GroupParamCnt=array_cnt/u8ParamCnt;
                for(i=0; i<u8GroupParamCnt; i++)
                {
                    ST_DynamicFuncParam_t  *pstDynamicChangeSclInputCrop = (ST_DynamicFuncParam_t *)malloc(sizeof(ST_DynamicFuncParam_t));
                    memset(pstDynamicChangeSclInputCrop, 0x0, sizeof(ST_DynamicFuncParam_t));

                    pstDynamicChangeSclInputCrop->eFuncType=E_DYNAMIC_CHANGE_SCLINPUTCROP;
                    pstDynamicChangeSclInputCrop->u16Id = (MI_U16)strtoul(array_str_holder[0+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeSclInputCrop->u32Cnt = (MI_U32)strtoul(array_str_holder[1+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeSclInputCrop->stChangeIspInputCrop.DevId = (MI_U32)strtoul(array_str_holder[2+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeSclInputCrop->stChangeIspInputCrop.ChnId = (MI_U32)strtoul(array_str_holder[3+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeSclInputCrop->stChangeSclInputCrop.stCropInfo.u16X = (MI_U32)strtoul(array_str_holder[4+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeSclInputCrop->stChangeSclInputCrop.stCropInfo.u16Y = (MI_U32)strtoul(array_str_holder[5+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeSclInputCrop->stChangeSclInputCrop.stCropInfo.u16Width = (MI_U32)strtoul(array_str_holder[6+i*u8ParamCnt], 0, 0);
                    pstDynamicChangeSclInputCrop->stChangeSclInputCrop.stCropInfo.u16Height = (MI_U32)strtoul(array_str_holder[7+i*u8ParamCnt], 0, 0);

                    pstDynamicChangeSclInputCrop->u32Sleep=u32Sleep;
                    DBG_INFO("change scl input crop id %d, cnt %d, x %d, y %d, width %d, height %d, sleep %d\n", pstDynamicChangeSclInputCrop->u16Id, pstDynamicChangeSclInputCrop->u32Cnt,
                        pstDynamicChangeSclInputCrop->stChangeSclInputCrop.stCropInfo.u16X, pstDynamicChangeSclInputCrop->stChangeSclInputCrop.stCropInfo.u16Y,
                        pstDynamicChangeSclInputCrop->stChangeSclInputCrop.stCropInfo.u16Width, pstDynamicChangeSclInputCrop->stChangeSclInputCrop.stCropInfo.u16Height, pstDynamicChangeSclInputCrop->u32Sleep);

                    if(u32MaxId < pstDynamicChangeSclInputCrop->u16Id)
                    {
                        u32MaxId = pstDynamicChangeSclInputCrop->u16Id;
                    }

                    if(u32RandomCnt ==0)
                        list_add_tail(&pstDynamicChangeSclInputCrop->stDynamicFuncListNode, &gstDynamicTest.stDynamicFuncListHead);
                    else
                        list_add_tail(&pstDynamicChangeSclInputCrop->stDynamicFuncListNode, &stTempListHead);
                }
            }

            free(array_str_holder);
            array_str_holder = NULL;
            free(holder);
            holder = NULL;
        }
        else
        {
            UTStatus = UT_CASE_FAIL;
            DBG_ERR("string to array fail \n");
        }
    }

    u32Sleep = (MI_U32)iniparser_getint(pstDict, ":destroycreatechnlsleep", 1);
    string = iniparser_getstring(pstDict, ":destroycreatechnl", 0);
    if(string != NULL)
    {
        MI_U8 u8ParamCnt=4;
        printf(">>>>>>>>>>>changesclinputcrop string %s \n", string);
        holder=iniparser_parser_string_to_array(string, &array_str_holder, &array_cnt);
        if(holder !=NULL)
        {
            if(array_cnt%u8ParamCnt !=0)
            {
                UTStatus = UT_CASE_FAIL;
                DBG_ERR("destroycreatechnl : cnt %d, input cnt %d err, id/cnt/module/devid  \n",u8ParamCnt, array_cnt);
            }
            else
            {
                MI_U8  u8GroupParamCnt=array_cnt/u8ParamCnt;
                for(i=0; i<u8GroupParamCnt; i++)
                {
                    ST_DynamicFuncParam_t  *pstDynamicDestroyCreateChannel = (ST_DynamicFuncParam_t *)malloc(sizeof(ST_DynamicFuncParam_t));
                    memset(pstDynamicDestroyCreateChannel, 0x0, sizeof(ST_DynamicFuncParam_t));

                    pstDynamicDestroyCreateChannel->eFuncType=E_DYNAMIC_DESTROYCREATE_CHANNEL;
                    pstDynamicDestroyCreateChannel->u16Id = (MI_U16)strtoul(array_str_holder[0+i*u8ParamCnt], 0, 0);
                    pstDynamicDestroyCreateChannel->u32Cnt = (MI_U32)strtoul(array_str_holder[1+i*u8ParamCnt], 0, 0);
                    pstDynamicDestroyCreateChannel->stDestroyCreateChannel.eModId = (MI_ModuleId_e)strtoul(array_str_holder[2+i*u8ParamCnt], 0, 0);
                    pstDynamicDestroyCreateChannel->stDestroyCreateChannel.DevId = (MI_U32)strtoul(array_str_holder[3+i*u8ParamCnt], 0, 0);

                    pstDynamicDestroyCreateChannel->u32Sleep=u32Sleep;
                    DBG_INFO("destroycreatechnl id %d, cnt %d, moduleid %d, dev %d, sleep %d\n", pstDynamicDestroyCreateChannel->u16Id, pstDynamicDestroyCreateChannel->u32Cnt,
                        pstDynamicDestroyCreateChannel->stDestroyCreateChannel.eModId,pstDynamicDestroyCreateChannel->stDestroyCreateChannel.DevId, pstDynamicDestroyCreateChannel->u32Sleep);

                    if(u32MaxId < pstDynamicDestroyCreateChannel->u16Id)
                    {
                        u32MaxId = pstDynamicDestroyCreateChannel->u16Id;
                    }

                    if(u32RandomCnt ==0)
                        list_add_tail(&pstDynamicDestroyCreateChannel->stDynamicFuncListNode, &gstDynamicTest.stDynamicFuncListHead);
                    else
                        list_add_tail(&pstDynamicDestroyCreateChannel->stDynamicFuncListNode, &stTempListHead);
                }
            }

            free(array_str_holder);
            array_str_holder = NULL;
            free(holder);
            holder = NULL;
        }
        else
        {
            UTStatus = UT_CASE_FAIL;
            DBG_ERR("string to array fail \n");
        }
    }

    u32Sleep = (MI_U32)iniparser_getint(pstDict, ":changeCRCOnoffsleep", 1);
    string = iniparser_getstring(pstDict, ":changeCRCOnof", NULL);
    if(string != NULL)
    {
        MI_U8 u8ParamCnt=7;
        printf(">>>>>>>>>>>CRCOnof string %s \n", string);
        holder=iniparser_parser_string_to_array(string, &array_str_holder, &array_cnt);
        if(holder != NULL)
        {
            if((array_cnt % u8ParamCnt) != 0)
            {
                UTStatus = UT_CASE_FAIL;
                DBG_ERR("CRCOnof need param cnt %d, input cnt %d err, id/cnt/moduleid/devid/chnid/portid/bOn \n",u8ParamCnt, array_cnt);
            }
            else
            {
                MI_U8  u8GroupParamCnt=array_cnt/u8ParamCnt;
                DBG_INFO("group param cnt %d\n", u8GroupParamCnt);
                for(i=0; i<u8GroupParamCnt; i++)
                {
                    ST_DynamicFuncParam_t  *pstCRCOnoff = (ST_DynamicFuncParam_t *)malloc(sizeof(ST_DynamicFuncParam_t));
                    memset(pstCRCOnoff, 0x0, sizeof(ST_DynamicFuncParam_t));

                    pstCRCOnoff->eFuncType = E_DYNAMIC_CHANGE_CRCONOFF;
                    pstCRCOnoff->u16Id = (MI_U16)strtoul(array_str_holder[0+i*u8ParamCnt], 0, 0);
                    pstCRCOnoff->u32Cnt = (MI_U32)strtoul(array_str_holder[1+i*u8ParamCnt], 0, 0);
                    pstCRCOnoff->stChangeCRCOnoff.eModuleId = (MI_ModuleId_e)strtoul(array_str_holder[2+i*u8ParamCnt], 0, 0);
                    pstCRCOnoff->stChangeCRCOnoff.u32DevId = (MI_U32)strtoul(array_str_holder[3+i*u8ParamCnt], 0, 0);
                    pstCRCOnoff->stChangeCRCOnoff.u32ChnId = (MI_U32)strtoul(array_str_holder[4+i*u8ParamCnt], 0, 0);
                    pstCRCOnoff->stChangeCRCOnoff.u32PortId = (MI_U32)strtoul(array_str_holder[5+i*u8ParamCnt], 0, 0);
                    pstCRCOnoff->stChangeCRCOnoff.bOn = (MI_BOOL)strtoul(array_str_holder[6+i*u8ParamCnt], 0, 0);
                    pstCRCOnoff->u32Sleep = u32Sleep;

                    if(u32MaxId < pstCRCOnoff->u16Id)
                    {
                        u32MaxId = pstCRCOnoff->u16Id;
                    }

                    DBG_INFO("CRCOnof id %d,cnt %d, moduleid%d devid%d chnid%d portid%d bOn %d\n",pstCRCOnoff->u16Id,
                        pstCRCOnoff->u32Cnt, pstCRCOnoff->stChangeCRCOnoff.eModuleId, pstCRCOnoff->stChangeCRCOnoff.u32DevId,
                        pstCRCOnoff->stChangeCRCOnoff.u32ChnId, pstCRCOnoff->stChangeCRCOnoff.u32PortId, pstCRCOnoff->stChangeCRCOnoff.bOn);

                    if(u32RandomCnt ==0)
                        list_add_tail(&pstCRCOnoff->stDynamicFuncListNode, &gstDynamicTest.stDynamicFuncListHead);
                    else
                        list_add_tail(&pstCRCOnoff->stDynamicFuncListNode, &stTempListHead);
                }
            }

            free(array_str_holder);
            array_str_holder = NULL;
            free(holder);
            holder = NULL;
        }
        else
        {
            UTStatus = UT_CASE_FAIL;
            DBG_ERR("string to array fail \n");
        }
    }

    u32Sleep = (MI_U32)iniparser_getint(pstDict, ":changeChnPortOnOffsleep", 1);
    string = iniparser_getstring(pstDict, ":changeChnPortOnOff", NULL);
    if(string != NULL)
    {
        MI_U8 u8ParamCnt=8;
        printf(">>>>>>>>>>>changeChnPortOnOff string %s \n", string);
        holder=iniparser_parser_string_to_array(string, &array_str_holder, &array_cnt);
        if(holder != NULL)
        {
            if((array_cnt % u8ParamCnt) != 0)
            {
                UTStatus = UT_CASE_FAIL;
                DBG_ERR("changeChnPortOnOff need param cnt %d, input cnt %d err, id/cnt/moduleid/devid/chnid/portid/position/status \n",u8ParamCnt, array_cnt);
            }
            else
            {
                MI_U8  u8GroupParamCnt=array_cnt/u8ParamCnt;
                DBG_INFO("group param cnt %d\n", u8GroupParamCnt);
                for(i=0; i<u8GroupParamCnt; i++)
                {
                    ST_DynamicFuncParam_t  *pstChnPortOnOff = (ST_DynamicFuncParam_t *)malloc(sizeof(ST_DynamicFuncParam_t));
                    memset(pstChnPortOnOff, 0x0, sizeof(ST_DynamicFuncParam_t));

                    pstChnPortOnOff->eFuncType = E_DYNAMIC_CHANGE_CHNPORTONOFF;
                    pstChnPortOnOff->u16Id = (MI_U16)strtoul(array_str_holder[0+i*u8ParamCnt], 0, 0);
                    pstChnPortOnOff->u32Cnt = (MI_U32)strtoul(array_str_holder[1+i*u8ParamCnt], 0, 0);
                    pstChnPortOnOff->stChangeChnPortOnOff.stChnPort.eModId = (MI_ModuleId_e)strtoul(array_str_holder[2+i*u8ParamCnt], 0, 0);
                    pstChnPortOnOff->stChangeChnPortOnOff.stChnPort.u32DevId = (MI_U32)strtoul(array_str_holder[3+i*u8ParamCnt], 0, 0);
                    pstChnPortOnOff->stChangeChnPortOnOff.stChnPort.u32ChnId = (MI_U32)strtoul(array_str_holder[4+i*u8ParamCnt], 0, 0);
                    pstChnPortOnOff->stChangeChnPortOnOff.stChnPort.u32PortId = (MI_U32)strtoul(array_str_holder[5+i*u8ParamCnt], 0, 0);
                    pstChnPortOnOff->stChangeChnPortOnOff.ePosition = (ST_DynamicChnPortPosition_e)strtoul(array_str_holder[6+i*u8ParamCnt], 0, 0);
                    pstChnPortOnOff->stChangeChnPortOnOff.bStatus = (MI_BOOL)strtoul(array_str_holder[7+i*u8ParamCnt], 0, 0);
                    pstChnPortOnOff->u32Sleep = u32Sleep;

                    if(u32MaxId < pstChnPortOnOff->u16Id)
                    {
                        u32MaxId = pstChnPortOnOff->u16Id;
                    }

                    DBG_INFO("changeChnPortOnOff id %d,cnt %d, moduleid%d devid%d chnid%d portid%d position%d status%d\n",
                        pstChnPortOnOff->u16Id,pstChnPortOnOff->u32Cnt,
                        pstChnPortOnOff->stChangeChnPortOnOff.stChnPort.eModId,
                        pstChnPortOnOff->stChangeChnPortOnOff.stChnPort.u32DevId,
                        pstChnPortOnOff->stChangeChnPortOnOff.stChnPort.u32ChnId,
                        pstChnPortOnOff->stChangeChnPortOnOff.stChnPort.u32PortId,
                        pstChnPortOnOff->stChangeChnPortOnOff.ePosition,
                        pstChnPortOnOff->stChangeChnPortOnOff.bStatus);

                    if(u32RandomCnt ==0)
                        list_add_tail(&pstChnPortOnOff->stDynamicFuncListNode, &gstDynamicTest.stDynamicFuncListHead);
                    else
                        list_add_tail(&pstChnPortOnOff->stDynamicFuncListNode, &stTempListHead);
                }
            }

            free(array_str_holder);
            array_str_holder = NULL;
            free(holder);
            holder = NULL;
        }
        else
        {
            UTStatus = UT_CASE_FAIL;
            DBG_ERR("string to array fail \n");
        }
    }

    if(u32RandomCnt ==0)
    {
         ST_List_Sort(NULL, &gstDynamicTest.stDynamicFuncListHead, ST_DynamicChangeFuncListSort);
        gstDynamicTest.u32ListTotalNodeCnt = u32MaxId;
    }
    else
    {
        struct list_head *pos = NULL;
        struct list_head *n = NULL;
        gstDynamicTest.u32ListTotalNodeCnt = u32RandomCnt;

        for(i=0; i<u32RandomCnt;)
        {
            int a=rand();
            a=a%(u32MaxId+1);

            list_for_each_safe(pos, n, &stTempListHead)
            {
                ST_DynamicFuncParam_t *pstTempDynamicFunc = NULL;
                pstTempDynamicFunc = container_of(pos, typeof(*pstTempDynamicFunc), stDynamicFuncListNode);

                if(pstTempDynamicFunc->u16Id == a)
                {
                    ST_DynamicFuncParam_t  *pstDynamicFunc= (ST_DynamicFuncParam_t *)malloc(sizeof(ST_DynamicFuncParam_t));
                    memcpy(pstDynamicFunc, pstTempDynamicFunc, sizeof(ST_DynamicFuncParam_t));
                    list_add_tail(&pstDynamicFunc->stDynamicFuncListNode, &gstDynamicTest.stDynamicFuncListHead);
                    i++;
                    break;
                }
            }
        }

        list_for_each_safe(pos, n, &stTempListHead)
        {
            ST_DynamicFuncParam_t *pstTempDynamicFunc = NULL;
            pstTempDynamicFunc = container_of(pos, typeof(*pstTempDynamicFunc), stDynamicFuncListNode);

            list_del(&pstTempDynamicFunc->stDynamicFuncListNode);
            free(pstTempDynamicFunc);
        }
    }

    iniparser_freedict(pstDict);
    return MI_SUCCESS;
}

MI_BOOL ST_DoChangeIspRotate(MI_ISP_DEV IspDev, MI_ISP_CHANNEL IspChn, MI_SYS_Rotate_e eRot, MI_BOOL bMirror, MI_BOOL bFlip)
{
    MI_BOOL bRet = MI_SUCCESS;
#if (MI_VIF_SUPPORT && MI_ISP_SUPPORT && MI_SCL_SUPPORT && MI_VENC_SUPPORT)

    MI_VIF_DEV VifDevId = 0;
    MI_VIF_PORT VifPortId = 0;
    MI_ISP_PORT  IspOutPortId = 0;
    MI_SCL_DEV SclDevId = 0;
    MI_SCL_CHANNEL SclChnId = 0;
    MI_SCL_PORT SclOutPortId = 0;
    MI_VENC_CHN VencChnId = 0;
    MI_SYS_WindowSize_t stInputSrcSize;
    ST_IspChannelAttr_t  *pstIspChnAttr = NULL;
    ST_PortAttr_t *pstIspOutputAttr = NULL;
    ST_SclChannelAttr_t *pstSclChnAttr = NULL;
    ST_VencAttr_t  *pstVencAttr = NULL;

    if(IspDev >= ST_MAX_ISP_DEV_NUM || IspChn >= ST_MAX_ISP_CHN_NUM)
    {
        printf("IspDev %d, IspChn %d > max dev %d chn %d \n", IspDev, IspChn, ST_MAX_ISP_DEV_NUM, ST_MAX_ISP_CHN_NUM);
        bRet = -1;
        goto EXIT;
    }

    pstIspChnAttr = &gstIspModule.stIspDevAttr[IspDev].stIspChnlAttr[IspChn];
    if(pstIspChnAttr->bCreate != TRUE || pstIspChnAttr->bUsed != TRUE)
    {
        printf("ISP channel:%d not working \n",IspChn);
        bRet = -1;
        goto EXIT;
    }

    VifDevId = pstIspChnAttr->stIspInPortAttr[0].stBindParam.stChnPort.u32DevId;
    VifPortId = pstIspChnAttr->stIspInPortAttr[0].stBindParam.stChnPort.u32PortId;

    for(IspOutPortId=0; IspOutPortId<ST_MAX_ISP_OUTPORT_NUM; IspOutPortId++)
    {
        ST_Output_BindParam_List_t *isp_list_node = NULL, *isp_node_safe = NULL;
        ST_Output_BindParam_List_t *scl_list_node = NULL, *scl_node_safe = NULL;

        pstIspOutputAttr = &pstIspChnAttr->stIspOutPortAttr[IspOutPortId];
        if(pstIspOutputAttr->bUsed != TRUE)
            continue;

        /************************************************
        Step1:
        Stop Venc (Because rot will change preview resolution)
        *************************************************/
        list_for_each_entry_safe(isp_list_node, isp_node_safe, &pstIspOutputAttr->head.pos, pos)
        {
            if(isp_list_node->stBindInfo.stDstChnPort.eModId == E_MI_MODULE_ID_SCL)
            {
                SclDevId = isp_list_node->stBindInfo.stDstChnPort.u32DevId;
                SclChnId = isp_list_node->stBindInfo.stDstChnPort.u32ChnId;
                pstSclChnAttr = &gstSclModule.stSclDevAttr[SclDevId].stSclChnlAttr[SclChnId];

                if(pstSclChnAttr->bUsed != TRUE || pstSclChnAttr->bCreate != TRUE)
                    continue;

                for(SclOutPortId=0; SclOutPortId<ST_MAX_SCL_OUTPORT_NUM; SclOutPortId++)
                {
                    ST_PortAttr_t *pstSclOutputAttr = &pstSclChnAttr->stSclOutPortAttr[SclOutPortId];

                    if(pstSclOutputAttr->bUsed != TRUE || pstSclOutputAttr->bEnable != TRUE)
                        continue;

                    list_for_each_entry_safe(scl_list_node, scl_node_safe, &pstSclOutputAttr->head.pos, pos)
                    {
                        if(scl_list_node->stBindInfo.stDstChnPort.eModId == E_MI_MODULE_ID_VENC)
                        {
                            VencChnId = scl_list_node->stBindInfo.stDstChnPort.u32ChnId;
                            pstVencAttr = &gstVencattr[VencChnId];

                            if(pstVencAttr->bUsed == TRUE && pstVencAttr->bCreate == TRUE)
                                ExecFuncResult(ST_VencModuleUnInit(VencChnId), bRet);
                        }
                    }
                }


                /************************************************
                Step2: stop SCL
                *************************************************/
                ExecFuncResult(MI_SCL_StopChannel((MI_SCL_DEV)SclDevId, SclChnId), bRet);
            }
            else if(isp_list_node->stBindInfo.stDstChnPort.eModId == E_MI_MODULE_ID_VENC)
            {
                VencChnId = isp_list_node->stBindInfo.stDstChnPort.u32ChnId;
                pstVencAttr = &gstVencattr[VencChnId];

                if(pstVencAttr->bUsed == TRUE && pstVencAttr->bCreate == TRUE)
                    ExecFuncResult(ST_VencModuleUnInit(VencChnId), bRet);
            }
        }
    }


    /************************************************
    Step3: Stop ISP (Wait driver all buffer done)
    *************************************************/
    ExecFuncResult(MI_ISP_StopChannel(IspDev, IspChn), bRet);


    /************************************************
    Step4: Disable Vif Port(Realtime mode Change Rot Will Change Isp Cfg, need stop push vif stream)
    *************************************************/
    if(pstIspChnAttr->stIspInPortAttr[0].stBindParam.stChnPort.eModId == E_MI_MODULE_ID_VIF &&
        pstIspChnAttr->stIspInPortAttr[0].stBindParam.eBindType == E_MI_SYS_BIND_TYPE_REALTIME)
        ExecFuncResult(MI_VIF_DisableOutputPort(VifDevId, VifPortId), bRet);


    /**************************************************************
    Step5: Set ISP Rot/Mirror/Flip/3dnrlevel, update ISP param
    ***************************************************************/
    ExecFuncResult(MI_ISP_GetChnParam(IspDev, IspChn, &pstIspChnAttr->stIspChnParam), bRet);
    pstIspChnAttr->stIspChnParam.eRot = eRot;
    pstIspChnAttr->stIspChnParam.bFlip = bFlip;
    pstIspChnAttr->stIspChnParam.bMirror = bMirror;
    ExecFuncResult(MI_ISP_SetChnParam(IspDev, IspChn, &pstIspChnAttr->stIspChnParam), bRet);

    memset(&stInputSrcSize, 0x0, sizeof(MI_SYS_WindowSize_t));

    //update isp output crop
    for(IspOutPortId = 0;IspOutPortId < ST_MAX_ISP_OUTPORT_NUM;IspOutPortId++)
    {
        MI_SYS_ChnPort_t stChnPort;
        MI_ISP_OutPortParam_t  stIspOutputParam;

        pstIspOutputAttr = &pstIspChnAttr->stIspOutPortAttr[IspOutPortId];
        if(pstIspOutputAttr->bUsed != TRUE)
            continue;

        memset(&stChnPort, 0x0, sizeof(MI_SYS_ChnPort_t));
        memset(&stIspOutputParam, 0x0, sizeof(MI_ISP_OutPortParam_t));

        stChnPort.eModId = E_MI_MODULE_ID_ISP;
        stChnPort.u32DevId = IspDev;
        stChnPort.u32ChnId = IspChn;
        stChnPort.u32PortId = IspOutPortId;

        //recalculate isp output port crop after rotation
        ExecFuncResult(ST_GetIspOutputPortRect(&stChnPort,&pstIspOutputAttr->stPortCrop), bRet);

        ExecFuncResult(MI_ISP_GetOutputPortParam(IspDev, IspChn, IspOutPortId, &stIspOutputParam), bRet);
        if(pstIspOutputAttr->stOrigPortCrop.u16Height == 0
           || pstIspOutputAttr->stOrigPortCrop.u16Width == 0)
        {
            memcpy(&stIspOutputParam.stCropRect,&pstIspOutputAttr->stOrigPortCrop,sizeof(MI_SYS_WindowRect_t));
        }
        else
        {
            memcpy(&stIspOutputParam.stCropRect,&pstIspOutputAttr->stPortCrop,sizeof(MI_SYS_WindowRect_t));
        }
        ExecFuncResult(MI_ISP_SetOutputPortParam(IspDev, IspChn, IspOutPortId, &stIspOutputParam), bRet);
    }


    /************************************************
    Step6: Start Vif
    *************************************************/
    if(pstIspChnAttr->stIspInPortAttr[0].stBindParam.stChnPort.eModId == E_MI_MODULE_ID_VIF &&
        pstIspChnAttr->stIspInPortAttr[0].stBindParam.eBindType == E_MI_SYS_BIND_TYPE_REALTIME)
        ExecFuncResult(MI_VIF_EnableOutputPort(VifDevId, VifPortId), bRet);


    /************************************************
    Step7: Start ISP
    *************************************************/
    ExecFuncResult(MI_ISP_StartChannel(IspDev, IspChn), bRet);


    /************************************************
    Step8: Start SCL, update SCL param
    *************************************************/
    for(IspOutPortId=0; IspOutPortId<ST_MAX_ISP_OUTPORT_NUM; IspOutPortId++)
    {
        ST_Output_BindParam_List_t *isp_list_node = NULL;

        pstIspOutputAttr = &pstIspChnAttr->stIspOutPortAttr[IspOutPortId];
        if(pstIspOutputAttr->bUsed != TRUE)
            continue;

        list_for_each_entry(isp_list_node, &pstIspOutputAttr->head.pos, pos)
        {
            if(isp_list_node->stBindInfo.stDstChnPort.eModId == E_MI_MODULE_ID_SCL)
            {
                MI_SYS_WindowSize_t stSCLInputSrcSize;

                SclDevId = isp_list_node->stBindInfo.stDstChnPort.u32DevId;
                SclChnId = isp_list_node->stBindInfo.stDstChnPort.u32ChnId;
                pstSclChnAttr = &gstSclModule.stSclDevAttr[SclDevId].stSclChnlAttr[SclChnId];

                if(pstSclChnAttr->bUsed != TRUE || pstSclChnAttr->bCreate != TRUE)
                    continue;

                memset(&stSCLInputSrcSize, 0x0, sizeof(MI_SYS_WindowSize_t));
                if(pstIspChnAttr->stIspChnParam.eRot == E_MI_SYS_ROTATE_90 || pstIspChnAttr->stIspChnParam.eRot == E_MI_SYS_ROTATE_270)
                {
                    stSCLInputSrcSize.u16Width = pstIspOutputAttr->stPortCrop.u16Height;
                    stSCLInputSrcSize.u16Height = pstIspOutputAttr->stPortCrop.u16Width;
                }
                else
                {
                    stSCLInputSrcSize.u16Width = pstIspOutputAttr->stPortCrop.u16Width;
                    stSCLInputSrcSize.u16Height = pstIspOutputAttr->stPortCrop.u16Height;
                }

                //update scl input crop
                if(pstSclChnAttr->stSclInPortAttr[0].stOrigInputCropWin.u16Width !=0
                    && pstSclChnAttr->stSclInPortAttr[0].stOrigInputCropWin.u16Height !=0)
                {
                    ST_GetRotAfterCropRect(stSCLInputSrcSize, pstSclChnAttr->stSclInPortAttr[0].stOrigInputCropWin,
                        pstIspChnAttr->stIspChnParam.eRot, pstIspChnAttr->stIspChnParam.bMirror, pstIspChnAttr->stIspChnParam.bFlip, &pstSclChnAttr->stSclInPortAttr[0].stInputCropWin);

                    ExecFuncResult(MI_SCL_SetInputPortCrop((MI_SCL_DEV)SclDevId, SclChnId, &pstSclChnAttr->stSclInPortAttr[0].stInputCropWin), bRet);

                    if(pstIspChnAttr->stIspChnParam.eRot == E_MI_SYS_ROTATE_90 || pstIspChnAttr->stIspChnParam.eRot == E_MI_SYS_ROTATE_270)
                    {
                        stSCLInputSrcSize.u16Width = pstSclChnAttr->stSclInPortAttr[0].stInputCropWin.u16Height;
                        stSCLInputSrcSize.u16Height = pstSclChnAttr->stSclInPortAttr[0].stInputCropWin.u16Width;
                    }
                    else
                    {
                        stSCLInputSrcSize.u16Width = pstSclChnAttr->stSclInPortAttr[0].stInputCropWin.u16Width;
                        stSCLInputSrcSize.u16Height = pstSclChnAttr->stSclInPortAttr[0].stInputCropWin.u16Height;
                    }
                }

                //update scl output crop
                for(SclOutPortId=0; SclOutPortId<ST_MAX_SCL_OUTPORT_NUM; SclOutPortId++)
                {
                    MI_SCL_OutPortParam_t  stSclOutputParam;
                    ST_PortAttr_t *pstSclOutputAttr = &pstSclChnAttr->stSclOutPortAttr[SclOutPortId];

                    if(pstSclOutputAttr->bUsed != TRUE || pstSclOutputAttr->bEnable != TRUE)
                        continue;

                    ST_GetRotAfterCropRect(stSCLInputSrcSize, pstSclOutputAttr->stOrigPortCrop,
                        pstIspChnAttr->stIspChnParam.eRot, pstIspChnAttr->stIspChnParam.bMirror, pstIspChnAttr->stIspChnParam.bFlip, &pstSclOutputAttr->stPortCrop);

                    //only scl dev3 support rotation, and not support crop/scaling when do rotation
                    if((pstSclChnAttr->eRotate == E_MI_SYS_ROTATE_90 || pstSclChnAttr->eRotate == E_MI_SYS_ROTATE_270)
                        ^ (pstIspChnAttr->stIspChnParam.eRot == E_MI_SYS_ROTATE_90 || pstIspChnAttr->stIspChnParam.eRot == E_MI_SYS_ROTATE_270))
                    {
                        pstSclOutputAttr->stPortSize.u16Width = pstSclOutputAttr->stOrigPortSize.u16Height;
                        pstSclOutputAttr->stPortSize.u16Height = pstSclOutputAttr->stOrigPortSize.u16Width;
                    }
                    else
                    {
                        memcpy(&pstSclOutputAttr->stPortSize, &pstSclOutputAttr->stOrigPortSize, sizeof(MI_SYS_WindowSize_t));
                    }

                    memset(&stSclOutputParam, 0x0, sizeof(MI_SCL_OutPortParam_t));
                    ExecFuncResult(MI_SCL_GetOutputPortParam((MI_SCL_DEV)SclDevId, SclChnId, SclOutPortId, &stSclOutputParam), bRet);
                    memcpy(&stSclOutputParam.stSCLOutCropRect, &pstSclOutputAttr->stPortCrop, sizeof(MI_SYS_WindowRect_t));
                    memcpy(&stSclOutputParam.stSCLOutputSize, &pstSclOutputAttr->stPortSize, sizeof(MI_SYS_WindowSize_t));
                    ExecFuncResult(MI_SCL_SetOutputPortParam((MI_SCL_DEV)SclDevId, SclChnId, SclOutPortId, &stSclOutputParam), bRet);
                }

                ExecFuncResult(MI_SCL_StartChannel((MI_SCL_DEV)SclDevId, SclChnId), bRet);
            }
        }
    }

    /************************************************
    Step9: Start Venc
    *************************************************/
    MI_U16 u16VencChn;
    for(u16VencChn=0; u16VencChn<ST_MAX_VENC_NUM; u16VencChn++)
    {
        ST_VencAttr_t  *pstVencAttr = &gstVencattr[u16VencChn];
        if(pstVencAttr->bUsed == TRUE && pstVencAttr->bCreate == FALSE)
        {
            ExecFuncResult(ST_VencModuleInit(u16VencChn), bRet);
        }
    }

EXIT:
#endif
    return bRet;
}

MI_BOOL ST_DoChangeSclRotate(MI_SCL_DEV SclDev, MI_SCL_CHANNEL SclChn, MI_SYS_Rotate_e eSclRot)
{
    MI_BOOL bRet = MI_SUCCESS;
#if MI_SCL_SUPPORT

    ST_Output_BindParam_List_t *scl_list_node = NULL, *scl_node_safe = NULL;
    MI_SCL_PORT SclOutPortId = 0;
    MI_VENC_CHN VencChnId = 0;
    MI_SYS_Rotate_e eIspRot = E_MI_SYS_ROTATE_NONE;
    ST_SclChannelAttr_t *pstSclChnattr = &gstSclModule.stSclDevAttr[SclDev].stSclChnlAttr[SclChn];
    ST_VencAttr_t  *pstVencAttr = NULL;

    if(eSclRot >= E_MI_SYS_ROTATE_NUM)
    {
        printf("rot %d >= max %d \n", eSclRot, E_MI_SYS_ROTATE_NUM);
        bRet = -1;
        goto EXIT;
    }

    if(pstSclChnattr->bCreate != TRUE || pstSclChnattr->bUsed != TRUE)
    {
        printf("SCL dev%d chn:%d is not working \n", SclDev, SclChn);
        bRet = -1;
        goto EXIT;
    }

    /************************************************
    Step1: Stop Venc
    *************************************************/
    for(SclOutPortId=0; SclOutPortId<ST_MAX_SCL_OUTPORT_NUM; SclOutPortId++)
    {
        ST_PortAttr_t *pstSclOutputAttr = &pstSclChnattr->stSclOutPortAttr[SclOutPortId];

        if(pstSclOutputAttr->bUsed != TRUE || pstSclOutputAttr->bEnable != TRUE)
            continue;

        list_for_each_entry_safe(scl_list_node, scl_node_safe, &pstSclOutputAttr->head.pos, pos)
        {
            if(scl_list_node->stBindInfo.stDstChnPort.eModId == E_MI_MODULE_ID_VENC)
            {
                VencChnId = scl_list_node->stBindInfo.stDstChnPort.u32ChnId;
                pstVencAttr = &gstVencattr[VencChnId];

                if(pstVencAttr->bUsed == TRUE && pstVencAttr->bCreate == TRUE)
                    ExecFuncResult(ST_VencModuleUnInit(VencChnId), bRet);
            }
        }
    }


    /************************************************
    Step2: Stop SCL
    *************************************************/
    ExecFuncResult(MI_SCL_StopChannel(SclDev, SclChn), bRet);


    /************************************************
    Step3: update SCL chn param
    *************************************************/
    MI_SCL_ChnParam_t  stSclChnParam;
    memset(&stSclChnParam, 0x0, sizeof(MI_SCL_ChnParam_t));
    ExecFuncResult(MI_SCL_GetChnParam(SclDev, SclChn, &stSclChnParam), bRet);
    stSclChnParam.eRot = eSclRot;
    ExecFuncResult(MI_SCL_SetChnParam(SclDev, SclChn, &stSclChnParam), bRet);

    pstSclChnattr->eRotate = eSclRot;

    if(pstSclChnattr->stSclInPortAttr[0].stBindParam.stChnPort.eModId == E_MI_MODULE_ID_ISP)
    {
        MI_ISP_DEV IspDev = pstSclChnattr->stSclInPortAttr[0].stBindParam.stChnPort.u32DevId;
        MI_ISP_CHANNEL IspChn = pstSclChnattr->stSclInPortAttr[0].stBindParam.stChnPort.u32ChnId;
        ST_IspChannelAttr_t *pstIspChnAttr = &gstIspModule.stIspDevAttr[IspDev].stIspChnlAttr[IspChn];

        eIspRot = pstIspChnAttr->stIspChnParam.eRot;
    }


    /************************************************
    Step4: update SCL output port param
    *************************************************/
    for(SclOutPortId=0; SclOutPortId<ST_MAX_SCL_OUTPORT_NUM; SclOutPortId++)
    {
        MI_SCL_OutPortParam_t  stSclOutputParam;
        ST_PortAttr_t *pstSclOutputAttr = &pstSclChnattr->stSclOutPortAttr[SclOutPortId];

        if(pstSclOutputAttr->bUsed != TRUE || pstSclOutputAttr->bEnable != TRUE)
            continue;

        if((eSclRot == E_MI_SYS_ROTATE_90 || eSclRot == E_MI_SYS_ROTATE_270)
            ^ (eIspRot == E_MI_SYS_ROTATE_90 || eIspRot == E_MI_SYS_ROTATE_270))
        {
            pstSclOutputAttr->stPortSize.u16Width = pstSclOutputAttr->stOrigPortSize.u16Height;
            pstSclOutputAttr->stPortSize.u16Height = pstSclOutputAttr->stOrigPortSize.u16Width;
        }
        else
        {
            memcpy(&pstSclOutputAttr->stPortSize, &pstSclOutputAttr->stOrigPortSize, sizeof(MI_SYS_WindowSize_t));
        }

        memset(&stSclOutputParam, 0x0, sizeof(MI_SCL_OutPortParam_t));
        ExecFuncResult(MI_SCL_GetOutputPortParam(SclDev, SclChn, SclOutPortId, &stSclOutputParam), bRet);
        memcpy(&stSclOutputParam.stSCLOutCropRect, &pstSclOutputAttr->stPortCrop, sizeof(MI_SYS_WindowRect_t));
        memcpy(&stSclOutputParam.stSCLOutputSize, &pstSclOutputAttr->stPortSize, sizeof(MI_SYS_WindowSize_t));
        ExecFuncResult(MI_SCL_SetOutputPortParam(SclDev, SclChn, SclOutPortId, &stSclOutputParam), bRet);
    }


    /************************************************
    Step5: Start SCL
    *************************************************/
    ExecFuncResult(MI_SCL_StartChannel(SclDev, SclChn), bRet);


    /************************************************
    Step6: Start Venc
    *************************************************/
    MI_U16 u16VencChn;
    for(u16VencChn=0; u16VencChn<ST_MAX_VENC_NUM; u16VencChn++)
    {
        ST_VencAttr_t  *pstVencAttr = &gstVencattr[u16VencChn];
        if(pstVencAttr->bUsed == TRUE && pstVencAttr->bCreate == FALSE)
        {
            ExecFuncResult(ST_VencModuleInit(u16VencChn), bRet);
        }
    }

EXIT:
#endif
    return bRet;
}

MI_BOOL ST_DoChangeSclInputCrop(MI_SCL_DEV DevId, MI_SCL_CHANNEL ChnId,MI_SYS_WindowRect_t *pstCropInfo)
{
    MI_BOOL bRet = MI_SUCCESS;
#if MI_SCL_SUPPORT

    MI_SYS_WindowRect_t stOrgSclInputRect;
    memset(&stOrgSclInputRect, 0x0, sizeof(MI_SYS_WindowRect_t));

    ExecFuncResult(MI_SCL_GetInputPortCrop(DevId,ChnId,&stOrgSclInputRect), bRet);

    DBG_INFO("SCL[%d-%d] orginputcrop(%d,%d,%d,%d),Destinputcrop(%d,%d,%d,%d)\n",DevId,ChnId,
    stOrgSclInputRect.u16X,stOrgSclInputRect.u16Y,stOrgSclInputRect.u16Width,stOrgSclInputRect.u16Height,
    pstCropInfo->u16X,pstCropInfo->u16Y,pstCropInfo->u16Width,pstCropInfo->u16Height);

    memcpy(&stOrgSclInputRect,pstCropInfo,sizeof(MI_SYS_WindowRect_t));
    ExecFuncResult(MI_SCL_SetInputPortCrop(DevId,ChnId,&stOrgSclInputRect), bRet);

EXIT:
#endif
    return bRet;
}

MI_BOOL ST_DoChangeSclOutPutParam(MI_SCL_DEV DevId, MI_SCL_CHANNEL ChnId, MI_SCL_PORT PortId, MI_SCL_OutPortParam_t *pstOutputParam)
{
    MI_BOOL bRet = MI_SUCCESS;
#if MI_SCL_SUPPORT

    ST_Sys_BindInfo_T stBindInfo;
    MI_SCL_OutPortParam_t stSclOutputParam;
    ST_SclChannelAttr_t *pstSclChnattr = NULL;
    ST_PortAttr_t *pstSclPortAttr = NULL;
    memset(&stBindInfo, 0x0, sizeof(ST_Sys_BindInfo_T));
    memset(&stSclOutputParam, 0x0, sizeof(MI_SCL_OutPortParam_t));

    if((DevId<ST_MAX_SCL_DEV_NUM && ChnId <ST_MAX_SCL_CHN_NUM && PortId<ST_MAX_SCL_OUTPORT_NUM) != TRUE)
    {
        UTStatus = UT_CASE_FAIL;
        DBG_ERR("Input Dev %d, Chn %d, port %d invalid \n", DevId, ChnId, PortId);
        bRet = -1;
        goto EXIT;
    }

    pstSclChnattr = &gstSclModule.stSclDevAttr[DevId].stSclChnlAttr[ChnId];
    pstSclPortAttr = &pstSclChnattr->stSclOutPortAttr[PortId];

    ExecFuncResult(MI_SCL_GetOutputPortParam(DevId, ChnId, PortId, &stSclOutputParam), bRet);

    /************************************************
    Step2: disable Vpe port
    *************************************************/
    ExecFuncResult(MI_SCL_DisableOutputPort(DevId, ChnId, PortId), bRet);

    /************************************************
    Step3: Set Port Mode,add default value.
           At dynamic, We can change arbitrary param on the org param,
           such as only change compress :
           org(outsize 1920,1080 crop 50,50,1280,720 pixel 11 compress 0)
           ini(outsize 0,0 crop 0,0,0,0 pixel MAX compress 5)
    *************************************************/
    pstSclPortAttr->bMirror = pstOutputParam->bMirror;
    pstSclPortAttr->bFlip = pstOutputParam->bFlip;
    if(pstOutputParam->stSCLOutputSize.u16Width != 0 && pstOutputParam->stSCLOutputSize.u16Height != 0)
    {
        memcpy(&pstSclPortAttr->stOrigPortSize, &pstOutputParam->stSCLOutputSize, sizeof(MI_SYS_WindowSize_t));
        memcpy(&pstSclPortAttr->stPortSize, &pstOutputParam->stSCLOutputSize, sizeof(MI_SYS_WindowSize_t));
        memcpy(&stSclOutputParam.stSCLOutputSize, &pstOutputParam->stSCLOutputSize, sizeof(MI_SYS_WindowSize_t));
    }
    if(pstOutputParam->stSCLOutCropRect.u16Width != 0 && pstOutputParam->stSCLOutCropRect.u16Height != 0)
    {
        memcpy(&pstSclPortAttr->stOrigPortCrop, &pstOutputParam->stSCLOutCropRect, sizeof(MI_SYS_WindowRect_t));
        memcpy(&pstSclPortAttr->stPortCrop, &pstOutputParam->stSCLOutCropRect, sizeof(MI_SYS_WindowRect_t));
        memcpy(&stSclOutputParam.stSCLOutCropRect, &pstOutputParam->stSCLOutCropRect, sizeof(MI_SYS_WindowRect_t));
    }

    if(pstOutputParam->ePixelFormat != E_MI_SYS_PIXEL_FRAME_FORMAT_MAX)
    {
        pstSclPortAttr->ePixelFormat = pstOutputParam->ePixelFormat;
        stSclOutputParam.ePixelFormat = pstOutputParam->ePixelFormat;
    }
    if(pstOutputParam->eCompressMode != E_MI_SYS_COMPRESS_MODE_BUTT)
    {
        pstSclPortAttr->eCompressMode = pstOutputParam->eCompressMode;
        stSclOutputParam.eCompressMode = pstOutputParam->eCompressMode;
    }

    pstSclPortAttr->bUsed = TRUE;

    ExecFuncResult(MI_SCL_SetOutputPortParam(DevId, ChnId, PortId, &stSclOutputParam), bRet);
    ExecFuncResult(MI_SCL_EnableOutputPort(DevId, ChnId, PortId), bRet);

EXIT:
#endif
    return bRet;
}

MI_BOOL ST_DoSetChnZoom(MI_U32 u32ChnNum)
{
#if 0
    float r = 16.0/9;
    MI_U16 ystep = 8;
    MI_U16 xstep =ALIGN_UP((MI_U16)(r*ystep), 2);
    int oriW = 0, oriH = 0;
    MI_SYS_WindowRect_t stCropInfo;
    MI_VIF_ChnPortAttr_t stVifPortInfo;
    MI_U32 u32SleepTimeUs = 0;
    MI_U32 u32Fps =0;
    MI_S32 s32ZoomPosition = 0;
    MI_S32 s32PortZoom = 0;
    MI_BOOL bZoomDone = TRUE;
    memset(&stVifPortInfo, 0x0, sizeof(MI_VIF_ChnPortAttr_t));
    memset(&stCropInfo, 0, sizeof(MI_SYS_WindowRect_t));

    MI_ISP_CHANNEL  VpeChn = 0;
    MI_U8 u8VpeInPortId =0;
    MI_VIF_CHN  VifChn = 0;
    MI_VIF_CHN  VifPort = 0;
    MI_VIF_DEV  VifDev = 0;
    MI_SNR_PADID eSnrPad = E_MI_SNR_PAD_ID_0;
    if(u32ChnNum > 1)
    {
        printf("select channel id:");
        scanf("%d", &VpeChn);
        ST_Flush();

    }
    else
    {
        for(VpeChn=0; VpeChn<ST_MAX_VPECHN_NUM; VpeChn++)
        {
            ST_VpeChannelAttr_t *pstVpeChnattr = &gstVpeChnattr[VpeChn];
            if(pstVpeChnattr->bUsed==TRUE &&  pstVpeChnattr->bCreate == TRUE)
            {
                printf("use vpe chn %d \n", VpeChn);
                break;
            }
        }
    }

    if(VpeChn >= ST_MAX_VPECHN_NUM)
    {
        printf("VpeChn %d > max %d \n", VpeChn, ST_MAX_VPECHN_NUM);
        return 0;
    }
    ST_VpeChannelAttr_t *pstVpeChnAttr = &gstVpeChnattr[VpeChn];
    VifChn = pstVpeChnAttr->stInputPortAttr[u8VpeInPortId].stBindParam.stChnPort.u32ChnId;
    VifDev = pstVpeChnAttr->stInputPortAttr[u8VpeInPortId].stBindParam.stChnPort.u32DevId;
    VifPort = pstVpeChnAttr->stInputPortAttr[u8VpeInPortId].stBindParam.stChnPort.u32PortId;
    ST_VifDevAttr_t *pstVifChnAttr = &gstVifModule.stVifDevAttr[VifDev];
    eSnrPad = (MI_SNR_PADID)pstVifChnAttr->stBindSensor.eSensorPadID;
    ST_Sensor_Attr_t *pstSensorAttr = &gstSensorAttr[eSnrPad];

    MI_VIF_GetChnPortAttr(VifChn,0,&stVifPortInfo);
    oriW = stVifPortInfo.stCapRect.u16Width;
    oriH = stVifPortInfo.stCapRect.u16Height;
    if(pstSensorAttr->bUsed == false)
    {
        oriW = pstVpeChnAttr->stInputFileAttr.u32Width;
        oriH = pstVpeChnAttr->stInputFileAttr.u32Height;
    }
    else
        MI_SNR_GetFps(eSnrPad, &u32Fps);

    if(u32Fps ==0)
    {
        UTStatus = UT_CASE_FAIL;
        printf("Get fps %d err, use default 30\n", u32Fps);
        u32Fps = 30;
    }

    u32SleepTimeUs = 1000000/u32Fps;
    printf("fps %d, sleeptime %d \n", u32Fps, u32SleepTimeUs);

    printf("set zoom position: 1.vif, 2.vpe isp dma, 3.vpe scl pre-crop");
    scanf("%d", &s32ZoomPosition);
    ST_Flush();

    if(s32ZoomPosition == 3)
    {
        printf("select which port zoom: 0:port0, 1:port1, 2:port2, 3: all port \n");
        scanf("%d", &s32PortZoom);
        ST_Flush();
    }

    while(1)
    {
        if(bZoomDone == TRUE)
        {
            stCropInfo.u16X += xstep;
            stCropInfo.u16Y += ystep;
            stCropInfo.u16Width = oriW - (2 * stCropInfo.u16X);
            stCropInfo.u16Height = oriH -(2 * stCropInfo.u16Y);

            stCropInfo.u16Width = ALIGN_UP(stCropInfo.u16Width, 2);
            stCropInfo.u16Height = ALIGN_UP(stCropInfo.u16Height, 2);

            if(stCropInfo.u16Width < 660 || stCropInfo.u16Height < 360)
            {
                bZoomDone = FALSE;
            }
        }
        else
        {
            stCropInfo.u16X -= xstep;
            stCropInfo.u16Y -= ystep;
            stCropInfo.u16Width = oriW - (2 * stCropInfo.u16X);
            stCropInfo.u16Height = oriH -(2 * stCropInfo.u16Y);

            stCropInfo.u16Width = ALIGN_UP(stCropInfo.u16Width, 2);
            stCropInfo.u16Height = ALIGN_UP(stCropInfo.u16Height, 2);

            if(stCropInfo.u16Width > oriW || stCropInfo.u16Height > oriH)
            {
                break;
            }
        }

        if(s32ZoomPosition == 1)
        {
            MI_VIF_ChnPortAttr_t stChnPortAttr;
            ExecFunc(MI_VIF_GetChnPortAttr(VifChn, VifPort, &stChnPortAttr), MI_SUCCESS);

            memcpy(&stChnPortAttr.stCapRect, &stCropInfo, sizeof(MI_SYS_WindowRect_t));

            stChnPortAttr.stDestSize.u16Width = stCropInfo.u16Width;
            stChnPortAttr.stDestSize.u16Height = stCropInfo.u16Height;

            ExecFunc(MI_VIF_SetChnPortAttr(VifChn, VifPort, &stChnPortAttr), MI_SUCCESS);
        }
        else if(s32ZoomPosition == 2)
        {
            STCHECKRESULT(MI_VPE_SetChannelCrop(VpeChn,  &stCropInfo));
            STCHECKRESULT(MI_VPE_GetChannelCrop(VpeChn,  &stCropInfo));
        }
        else if(s32ZoomPosition == 3)
        {
            if(s32PortZoom == 3)
            {
                MI_U8 i=0;
                for(i=0; i<ST_MAX_PORT_NUM;i++)
                {
                    if(i== ST_VPE_VIR_PORTID || i==3 || i==4)
                        continue;

                    ST_PortAttr_t *pstVpePortAttr = &pstVpeChnAttr->stVpePortAttr[i];

                    if(pstVpePortAttr->bUsed == TRUE)
                        MI_VPE_SetPortCrop(VpeChn, i, &stCropInfo);
                }
            }
            else
                MI_VPE_SetPortCrop(VpeChn, s32PortZoom, &stCropInfo);
        }
        printf("after crop down x:%d y:%d w:%d h:%d\n", stCropInfo.u16X, stCropInfo.u16Y, stCropInfo.u16Width, stCropInfo.u16Height);

        //ST_Flush();

        usleep(u32SleepTimeUs);
    }
#endif
    return 0;
}
#if 0

MI_BOOL ST_DoSetIqBin(MI_ISP_DEV IspDevId, MI_ISP_CHANNEL IspChnId, char *pConfigPath)
{
    CUS3A_ALGO_STATUS_t stCus3aStatus;
    memset(&stCus3aStatus, 0, sizeof(CUS3A_ALGO_STATUS_t));

    MI_U8  u8ispreadycnt = 0;
    if (strlen(pConfigPath) == 0)
    {
        ST_ERR("IQ Bin File path is NULL!\n");
        return FALSE;
    }

    while(1)
    {
        if(u8ispreadycnt > 100)
        {
            ST_ERR("ISP ready time out!\n");
            u8ispreadycnt = 0;
            break;
        }

        CUS3A_GetAlgoStatus((CUS3A_ISP_DEV_e)IspDevId,(CUS3A_ISP_CH_e)IspChnId, &stCus3aStatus);

        if((stCus3aStatus.Ae == E_ALGO_STATUS_RUNNING) && (stCus3aStatus.Awb == E_ALGO_STATUS_RUNNING))
        {
            ST_DBG("Ready to load IQ bin :%s u8ispreadycnt:%d\n",pConfigPath, u8ispreadycnt);
            MI_ISP_ApiCmdLoadBinFile(IspDevId,IspChnId, (char *)pConfigPath, 1234);

            usleep(10*1000);

            u8ispreadycnt = 0;
            break;

        }
        else
        {
            usleep(10*1000);
            u8ispreadycnt++;
        }

    }

    //MI_ISP_AE_FLICKER_TYPE_e Flicker= SS_AE_FLICKER_TYPE_50HZ;
    //MI_ISP_AE_SetFlicker(IspDevId,IspChnId, &Flicker);

//    ST_DBG("MI_ISP_AE_SetFlicker:%d !\n", (int)Flicker);
    return 0;
}
#else

#endif
//source file over 300 pictures maybe cause fbd file show abnormal
MI_BOOL ST_FbdFile(MI_U32 u32Width,MI_U32 u32Height,char *srcfile)
{
#if MI_ISP_SUPPORT

    struct stat statbuf;
    MI_U32 u32PicCount = 0;
    MI_U32 i = 0;
    MI_U64 u64filesize = 0;
    char destfile[256];

    if(stat(srcfile, &statbuf) < 0)
    {
        printf("can't get file size \n");
        return -1;
    }
    else
        u64filesize = statbuf.st_size;

    sprintf(destfile, "%s_fbd", srcfile);

    MI_U32 u32FbcBuffsize=u32Width*u32Height;
    MI_U32 u32FbdBuffsize=u32Width*u32Height*2;

    char *fbcbuffer=(char*)malloc(u32FbcBuffsize);
    char *fbdbuffer=(char*)malloc(u32FbdBuffsize);
    if(fbcbuffer == NULL || fbdbuffer == NULL)
    {
        if(fbcbuffer)
        {
            free(fbcbuffer);
        }

        if(fbdbuffer)
        {
            free(fbdbuffer);
        }
        printf("malloc falied, no memory \n");
        return -1;
    }

    FILE *srcfp = fopen(srcfile,"rb");
    FILE *destfp = fopen(destfile, "wb");
    if(srcfp == NULL || destfp == NULL)
    {
        printf("open file:%s,%s failed \n",srcfile,destfile);
        if(srcfp)
        {
            fclose(srcfp);
        }

        if(destfp)
        {
            fclose(destfp);
        }
        free(fbcbuffer);
        free(fbdbuffer);
        return -1;
    }

    u32PicCount = u64filesize/u32FbcBuffsize;

    for(i = 0; i < u32PicCount; i++)
    {
        fread(fbcbuffer, u32FbcBuffsize, 1, srcfp);
        FBD_Execute((unsigned char *)fbcbuffer, (unsigned short *)fbdbuffer, u32Width, u32Height);
        fwrite(fbdbuffer, u32FbdBuffsize, 1, destfp);
    }

    free(fbcbuffer);
    free(fbdbuffer);
    fclose(destfp);
    fclose(srcfp);
    printf("Fbd done , file %s \n", destfile);
#endif
    return 0;
}

MI_BOOL ST_DoChangeVifOutputFrc(MI_VIF_DEV VifDevId, MI_VIF_PORT VifPortId,MI_U32 u32FrameRate)
{
    MI_BOOL bRet = MI_SUCCESS;
#if MI_VIF_SUPPORT

    MI_VIF_GROUP  u32VifGroupId=0;
    MI_U32 u32VifDevIdInGroup=0;
    ST_Sys_BindInfo_T stBindInfo;
    ST_VifPortAttr_t  *pstVifPortInfo= NULL;
    ST_Output_BindParam_List_t *VifOutput_list_node = NULL, *VifOutput_node_safe = NULL;

    if(VifDevId >= ST_MAX_VIF_DEV_NUM || VifPortId >= ST_MAX_VIF_OUTPORT_NUM)
    {
        printf("vif Dev %d, port%d over range\n",VifDevId, VifPortId);
        bRet = -1;
        goto EXIT;
    }

    u32VifGroupId=VifDevId/ST_MAX_VIF_DEV_PERGROUP;
    u32VifDevIdInGroup = VifDevId%ST_MAX_VIF_DEV_PERGROUP;
    pstVifPortInfo= &gstVifModule.stVifGroupAttr[u32VifGroupId].stVifDevAttr[u32VifDevIdInGroup].stVifOutPortAttr[VifPortId];

    list_for_each_entry_safe(VifOutput_list_node, VifOutput_node_safe, &pstVifPortInfo->head.pos, pos)
    {
        if(VifOutput_list_node == NULL)
            break;

        memset(&stBindInfo, 0x0, sizeof(ST_Sys_BindInfo_T));
        stBindInfo.stSrcChnPort.eModId = VifOutput_list_node->stBindInfo.stSrcChnPort.eModId;
        stBindInfo.stSrcChnPort.u32DevId = VifOutput_list_node->stBindInfo.stSrcChnPort.u32DevId;
        stBindInfo.stSrcChnPort.u32ChnId = VifOutput_list_node->stBindInfo.stSrcChnPort.u32ChnId;
        stBindInfo.stSrcChnPort.u32PortId =VifOutput_list_node->stBindInfo.stSrcChnPort.u32PortId;

        stBindInfo.stDstChnPort.eModId = VifOutput_list_node->stBindInfo.stDstChnPort.eModId;
        stBindInfo.stDstChnPort.u32DevId = VifOutput_list_node->stBindInfo.stDstChnPort.u32DevId;
        stBindInfo.stDstChnPort.u32ChnId = VifOutput_list_node->stBindInfo.stDstChnPort.u32ChnId;
        stBindInfo.stDstChnPort.u32PortId = VifOutput_list_node->stBindInfo.stDstChnPort.u32PortId;

        stBindInfo.u32SrcFrmrate = 30;
        stBindInfo.u32DstFrmrate = 30;
        stBindInfo.eBindType = VifOutput_list_node->stBindInfo.eBindType;
        ExecFuncResult(ST_Sys_UnBind(&stBindInfo), bRet);

        stBindInfo.u32SrcFrmrate = 30;
        stBindInfo.u32DstFrmrate = u32FrameRate;
        ExecFuncResult(ST_Sys_Bind(&stBindInfo), bRet);
    }

EXIT:
#endif
    return bRet;
}
//port0 only support pixel change
//port1 support crop scaling down, out pixel only 12bit bayer
MI_BOOL  ST_DoChangeVifPortAttr(MI_VIF_DEV VifDevId, MI_VIF_PORT VifPortId, MI_VIF_OutputPortAttr_t *pstVifPortAttr)
{
    MI_BOOL bRet = MI_SUCCESS;
#if MI_VIF_SUPPORT

    ST_Output_BindParam_List_t *pHead = NULL, *pListPos = NULL, *pNext = NULL;
    MI_VIF_OutputPortAttr_t stVifPortAttr;
    MI_U8 u8VifGroupId = VifDevId/ST_MAX_VIF_DEV_PERGROUP;
    MI_U8 vifDevPerGroup = VifDevId%ST_MAX_VIF_DEV_PERGROUP;
    ST_VifPortAttr_t *pstVifPort = &gstVifModule.stVifGroupAttr[u8VifGroupId].stVifDevAttr[vifDevPerGroup].stVifOutPortAttr[VifPortId];

    pthread_mutex_lock(&gstVifModule.stVifGroupAttr[u8VifGroupId].stVifDevAttr[vifDevPerGroup].Devmutex);

    if(pstVifPort->bCreate != TRUE || pstVifPort->bUsed != TRUE)
    {
        UTStatus = UT_CASE_FAIL;
        DBG_ERR("vif dev%d port%d is not working \n", VifDevId, VifPortId);
        bRet = -1;
        goto EXIT;
    }
    memset(&stVifPortAttr, 0x0, sizeof(MI_VIF_OutputPortAttr_t));

#if MI_ISP_SUPPORT
    pHead = &pstVifPort->head;
    list_for_each_entry_safe(pListPos,pNext,&pHead->pos,pos)
    {
        MI_ISP_DEV DevId = pListPos->stBindInfo.stDstChnPort.u32DevId;
        MI_ISP_CHANNEL ChnId = pListPos->stBindInfo.stDstChnPort.u32ChnId;
        if(pListPos->stBindInfo.eBindType == E_MI_SYS_BIND_TYPE_REALTIME)
        {
            ExecFuncResult(MI_ISP_StopChannel(DevId,ChnId), bRet);
            ExecFuncResult(ST_Sys_UnBind(&pListPos->stBindInfo), bRet);
        }
    }
#endif

    ExecFuncResult(MI_VIF_DisableOutputPort(VifDevId, VifPortId), bRet);
    ExecFuncResult(MI_VIF_GetOutputPortAttr(VifDevId, VifPortId, &stVifPortAttr), bRet);

    if(pstVifPortAttr->stCapRect.u16X != 0
        || pstVifPortAttr->stCapRect.u16Y != 0
        || pstVifPortAttr->stCapRect.u16Width != 0
        || pstVifPortAttr->stCapRect.u16Height != 0)
    {
        if(VifPortId == 1)
            memcpy(&stVifPortAttr.stCapRect,&pstVifPortAttr->stCapRect,sizeof(MI_SYS_WindowRect_t));
        else
        {
            DBG_ERR("Vif only support port1 crop\n");
            UTStatus = UT_CASE_FAIL;
        }
    }
    if(pstVifPortAttr->stDestSize.u16Width != 0
        || pstVifPortAttr->stDestSize.u16Height != 0)
    {
        if(VifPortId == 1)
            memcpy(&stVifPortAttr.stDestSize,&pstVifPortAttr->stDestSize,sizeof(MI_SYS_WindowSize_t));
        else
        {
            UTStatus = UT_CASE_FAIL;
            DBG_ERR("Vif only support port1 scaling down\n");
        }
    }
    if(E_MI_SYS_PIXEL_FRAME_FORMAT_MAX != pstVifPortAttr->ePixFormat)
        stVifPortAttr.ePixFormat = pstVifPortAttr->ePixFormat;
    if(E_MI_VIF_FRAMERATE_MAX != pstVifPortAttr->eFrameRate)
        stVifPortAttr.eFrameRate = pstVifPortAttr->eFrameRate;
    if(E_MI_SYS_COMPRESS_MODE_BUTT != pstVifPortAttr->eCompressMode)
        stVifPortAttr.eCompressMode = pstVifPortAttr->eCompressMode;

    ExecFuncResult(MI_VIF_SetOutputPortAttr(VifDevId, VifPortId, &stVifPortAttr), bRet);
    ExecFuncResult(MI_VIF_EnableOutputPort(VifDevId, VifPortId), bRet);

#if MI_ISP_SUPPORT
    pHead = &pstVifPort->head;
    list_for_each_entry_safe(pListPos,pNext,&pHead->pos,pos)
    {
        MI_ISP_DEV IspDevId = pListPos->stBindInfo.stDstChnPort.u32DevId;
        MI_ISP_CHANNEL IspChnId = pListPos->stBindInfo.stDstChnPort.u32ChnId;
        if(pListPos->stBindInfo.eBindType == E_MI_SYS_BIND_TYPE_REALTIME)
        {
            ExecFuncResult(ST_Sys_Bind(&pListPos->stBindInfo), bRet);
            ExecFuncResult(MI_ISP_StartChannel(IspDevId,IspChnId), bRet);
        }
    }
#endif
EXIT:
    pthread_mutex_unlock(&gstVifModule.stVifGroupAttr[u8VifGroupId].stVifDevAttr[vifDevPerGroup].Devmutex);
#endif
    return bRet;
}

MI_BOOL ST_ConvertTo16Bitsbayer(MI_U32 u32Width,MI_U32 u32Height,MI_SYS_PixelFormat_e ePixel,char *srcfile)
{
    struct stat statbuf;
    MI_U32 i = 0;
    MI_U32 stride = 0, u32PicCount = 0;
    MI_U64  u64filesize = 0;
    MI_U32 u32OrigBuffsize = 0, u32DestBuffsize = 0;
    MI_SYS_DataPrecision_e ePrecision = E_MI_SYS_DATA_PRECISION_MAX;
    char destfile[256];
    FILE *srcfp = NULL, *destfp = NULL;
    char *Oribuffer = NULL, *Destbuffer = NULL;

    if(ePixel <= E_MI_SYS_PIXEL_FRAME_RGB_BAYER_BASE
        || ePixel >= E_MI_SYS_PIXEL_FRAME_RGB_BAYER_NUM)
    {
        printf("the file pixel%d is not bayer \n",ePixel);
        return -1;
    }

    if(stat(srcfile, &statbuf) < 0)
    {
        printf("can't get file size \n");
        return -1;
    }
    else
        u64filesize = statbuf.st_size;

    sprintf(destfile, "%s_To16bpp", srcfile);

    srcfp = fopen(srcfile,"rb");
    destfp = fopen(destfile, "wb");
    if(srcfp == NULL || destfp == NULL)
    {
        if(srcfp)
        {
            fclose(srcfp);
        }

        if(destfp)
        {
            fclose(destfp);
        }
        printf("open file:%s,%s failed \n",srcfile,destfile);
        return -1;
    }

    ePrecision = (MI_SYS_DataPrecision_e)((ePixel - E_MI_SYS_PIXEL_FRAME_RGB_BAYER_BASE) / E_MI_SYS_PIXEL_BAYERID_MAX);
    ST_GetBayerStride(ePrecision, u32Width, &stride);

    u32OrigBuffsize=stride*u32Height;
    u32DestBuffsize=u32Width*u32Height*2;

    u32PicCount = u64filesize/u32OrigBuffsize;

    Oribuffer = (char*)malloc(u32OrigBuffsize);
    Destbuffer = (char*)malloc(u32DestBuffsize);
    if(Oribuffer == NULL || Destbuffer == NULL)
    {
        printf("malloc falied, no memory \n");

        if(Oribuffer)
        {
            free(Oribuffer);
        }

        if(Destbuffer)
        {
            free(Destbuffer);
        }

        fclose(destfp);
        fclose(srcfp);
        return -1;
    }

    printf("=======begin convert T16Bit, orgfile path %s, T16Bitfile patch %s, ePixel %d ,stride %d, Width %d, height %d\n",
    srcfile,destfile,ePixel,stride,u32Width,u32Height);
    printf("PicCount %d,current Count 0000",u32PicCount);
    for(i = 0; i < u32PicCount; i++)
    {
        fread(Oribuffer, u32OrigBuffsize, 1, srcfp);
        ConvertRawImageTo16Bits(Oribuffer, Destbuffer, u32Width, u32Height, ePrecision, false);
        fwrite(Destbuffer, u32DestBuffsize, 1, destfp);
        printf("\b\b\b\b%4d",i);
        fflush(stdout);
        //printf("u32filesize %llu u32OrigBuffsize %d u32PicCount %d i %d\n",u64filesize,u32OrigBuffsize,u32PicCount,i);
    }
    printf("\n");

    free(Oribuffer);
    free(Destbuffer);
    fclose(destfp);
    fclose(srcfp);
    printf("=======end  convert file path %s \n", destfile);

    return 0;
}

MI_S32 ST_DumpOutputToFile(MI_SYS_BufInfo_t* pstBufInfo, char* pFilePath, ST_DumpOutputStatus_e estatus)
{
    static FILE *fp = NULL;
    struct timeval starttime = {0}, endtime = {0};
    if(estatus == E_ST_DUMP_STATUS_BEGIN)
    {
        fp = fopen(pFilePath ,"wb");
        if(fp == NULL)
        {
            printf("open file %s fail \n",pFilePath);
            return -1;
        }
        printf("begin dump, file:%s \n",pFilePath);
    }
    printf("begin write \n");
    gettimeofday(&starttime,NULL);

    fwrite(pstBufInfo->stFrameData.pVirAddr[0], pstBufInfo->stFrameData.u32BufSize, 1, fp);
    gettimeofday(&endtime,NULL);
    printf("write file done ----- wxh:(%dx%d) pixel %d cost:%lums \n",pstBufInfo->stFrameData.u16Width,pstBufInfo->stFrameData.u16Height,pstBufInfo->stFrameData.ePixelFormat,(endtime.tv_sec-starttime.tv_sec)*1000+(endtime.tv_usec-starttime.tv_usec)/1000);

    if(estatus == E_ST_DUMP_STATUS_END)
    {
        fclose(fp);
        printf("finish dump, release all, file:%s  \n",pFilePath);
    }

    return MI_SUCCESS;
}

MI_S32 ST_CopyOutputToDestFile(MI_SYS_BufInfo_t* pstBufInfo, MI_SYS_WindowSize_t *pstMaxWin, char* pFilePath, ST_DumpOutputStatus_e estatus, MI_SYS_ChnPort_t *pstChnPort)
{
    MI_S32 s32Ret = MI_SUCCESS;
    static FILE *fp = NULL;
    static MI_U32 u32DestBufSize = 0;
    static void *pDestVirAddr[MAX_ADDRESS_COUNT] = {NULL};
    static MI_PHY DestPhyAddr = 0;
    static MI_U32 u32DestStride[MAX_ADDRESS_COUNT] = {0}, u32DestHeight[MAX_ADDRESS_COUNT] = {0};

    MI_U16 u16DestW = pstMaxWin->u16Width;
    MI_U16 u16DestH = pstMaxWin->u16Height;
    MI_U8  i = 0;
    MI_U16 u16line = 0;
    MI_U32 u32WriteStride = 0, u32WriteHeight = 0;
    MI_U32 u32SourceBufW[MAX_ADDRESS_COUNT] = {0}, u32SourceBufH[MAX_ADDRESS_COUNT] = {0};
    struct timeval starttime = {0}, endtime = {0};

    if(estatus == E_ST_DUMP_STATUS_BEGIN)
    {
        fp = fopen(pFilePath ,"wb");
        if(fp == NULL)
        {
            printf("open file %s fail \n",pFilePath);
            s32Ret = -1;
            goto EXIT;
        }
        printf("begin dump, file:%s \n",pFilePath);

        DestPhyAddr = 0;
        memset(pDestVirAddr, 0x0, sizeof(void *)*MAX_ADDRESS_COUNT);
        memset(u32DestStride, 0x0, sizeof(MI_U16)*MAX_ADDRESS_COUNT);
        memset(u32DestHeight, 0x0, sizeof(MI_U16)*MAX_ADDRESS_COUNT);

        switch(pstBufInfo->stFrameData.ePixelFormat)
        {
            case E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420:
            {
                u32DestStride[0] = u16DestW;
                u32DestStride[1] = u16DestW;
                u32DestHeight[0] = u16DestH;
                u32DestHeight[1] = u16DestH/2;
                u32DestBufSize = u16DestW*u16DestH*3/2;
                break;
            }
            case E_MI_SYS_PIXEL_FRAME_YUV422_YUYV:
            case E_MI_SYS_PIXEL_FRAME_YUV422_UYVY:
            case E_MI_SYS_PIXEL_FRAME_YUV422_YVYU:
            case E_MI_SYS_PIXEL_FRAME_YUV422_VYUY:
            {
                u32DestStride[0] = u16DestW*2;
                u32DestHeight[0] = u16DestH;
                u32DestBufSize = u16DestH*u16DestW*2;
                break;
            }
            default:
            {
                if(pstBufInfo->stFrameData.ePixelFormat > E_MI_SYS_PIXEL_FRAME_RGB_BAYER_BASE
                    && pstBufInfo->stFrameData.ePixelFormat < E_MI_SYS_PIXEL_FRAME_RGB_BAYER_NUM)
                {
                    MI_SYS_DataPrecision_e ePrecision = (MI_SYS_DataPrecision_e)((pstBufInfo->stFrameData.ePixelFormat - E_MI_SYS_PIXEL_FRAME_RGB_BAYER_BASE) / E_MI_SYS_PIXEL_BAYERID_MAX);

                    if(pstBufInfo->stFrameData.eCompressMode == E_MI_SYS_COMPRESS_MODE_TO_8BIT)
                    {
                        u32DestStride[0] = u16DestW;
                    }
                    else
                    {
                        ST_GetBayerStride(ePrecision, u16DestW, &u32DestStride[0]);
                    }
                    u32DestHeight[0] = u16DestH;
                    u32DestBufSize = u32DestHeight[0]*u32DestStride[0];
                }
                else
                {
                    printf("not support this format:%d \n",pstBufInfo->stFrameData.ePixelFormat);
                    fclose(fp);
                    s32Ret = -1;
                    goto EXIT;
                }
                break;
            }
        }
        printf("image bufsize:%x stride0:%d height0:%d height1:%d\n",u32DestBufSize,u32DestStride[0],u32DestHeight[0],u32DestHeight[1]);

        ExecFuncResult(MI_SYS_MMA_Alloc(0, (MI_U8*)"mma_heap_name0", u32DestBufSize, &DestPhyAddr), s32Ret);
        if(MI_SUCCESS != MI_SYS_Mmap(DestPhyAddr, u32DestBufSize, &pDestVirAddr[0], FALSE))
        {
            printf("buf mmap failed !! \n");
            MI_SYS_MMA_Free(0,DestPhyAddr);
            fclose(fp);
            s32Ret = -1;
            goto EXIT;
        }

        pDestVirAddr[1] = u32DestHeight[1] == 0 ? NULL : (char *)pDestVirAddr[0] + u32DestStride[0]*u32DestHeight[0];
        pDestVirAddr[2] = u32DestHeight[2] == 0 ? NULL : (char *)pDestVirAddr[1] + u32DestStride[1]*u32DestHeight[1];
    }

    printf("begin write \n");
    gettimeofday(&starttime,NULL);
    switch(pstBufInfo->stFrameData.ePixelFormat)
    {
        case E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420:
        {
            memset(pDestVirAddr[0], 0x0, u32DestStride[0]*u32DestHeight[0]);
            memset(pDestVirAddr[1], 0x80, u32DestStride[1]*u32DestHeight[1]);

            u32SourceBufH[0] = (pstBufInfo->stFrameData.phyAddr[1]-pstBufInfo->stFrameData.phyAddr[0])/pstBufInfo->stFrameData.u32Stride[0];
            u32SourceBufH[1] = u32SourceBufH[0]/2;
            u32SourceBufW[0] = pstBufInfo->stFrameData.u32Stride[0];
            u32SourceBufW[1] = pstBufInfo->stFrameData.u32Stride[1];
            break;
        }
        case E_MI_SYS_PIXEL_FRAME_YUV422_YUYV:
        case E_MI_SYS_PIXEL_FRAME_YUV422_UYVY:
        case E_MI_SYS_PIXEL_FRAME_YUV422_YVYU:
        case E_MI_SYS_PIXEL_FRAME_YUV422_VYUY:
        {
            memset(pDestVirAddr[0], 0x0, u32DestBufSize);

            u32SourceBufH[0] = pstBufInfo->stFrameData.u32BufSize/pstBufInfo->stFrameData.u32Stride[0];
            u32SourceBufW[0] = pstBufInfo->stFrameData.u32Stride[0];
            break;
        }
        default:
        {
            if(pstBufInfo->stFrameData.ePixelFormat > E_MI_SYS_PIXEL_FRAME_RGB_BAYER_BASE
                    && pstBufInfo->stFrameData.ePixelFormat < E_MI_SYS_PIXEL_FRAME_RGB_BAYER_NUM)
            {
                memset(pDestVirAddr[0], 0x0, u32DestBufSize);

                u32SourceBufH[0] = pstBufInfo->stFrameData.u32BufSize/pstBufInfo->stFrameData.u32Stride[0];
                u32SourceBufW[0] = pstBufInfo->stFrameData.u32Stride[0];
            }
            else
            {
                printf("not support this format:%d \n",pstBufInfo->stFrameData.ePixelFormat);
                s32Ret = -1;
                goto EXIT;

            }
        }
    }

    for(i = 0; i < MAX_ADDRESS_COUNT; i++)
    {
        u32WriteHeight = u32SourceBufH[i] > u32DestHeight[i] ? u32DestHeight[i] : u32SourceBufH[i];
        u32WriteStride = u32SourceBufW[i] > u32DestStride[i] ? u32DestStride[i] : u32SourceBufW[i];

        for(u16line = 0; u16line < u32WriteHeight; u16line++)
        {
            memcpy((char *)pDestVirAddr[i]+u16line*u32DestStride[i], (char *)pstBufInfo->stFrameData.pVirAddr[i]+u16line*u32SourceBufW[i], u32WriteStride);
        }

        //printf("bufcnt:%u, wStride:%u, wHeight:%u \n",i,u32WriteStride,u32WriteHeight);
    }

    fwrite(pDestVirAddr[0],u32DestBufSize,1,fp);

    gettimeofday(&endtime,NULL);
    printf("write file done ----- wxh:(%dx%d) size:%u cost:%lums \n",u16DestW,u16DestH,u32DestBufSize,(endtime.tv_sec-starttime.tv_sec)*1000+(endtime.tv_usec-starttime.tv_usec)/1000);

    if(estatus == E_ST_DUMP_STATUS_END)
    {
        ExecFuncResult(MI_SYS_Munmap(pDestVirAddr[0], u32DestBufSize), s32Ret);
        ExecFuncResult(MI_SYS_MMA_Free(0,DestPhyAddr), s32Ret);
        fclose(fp);

        fp = NULL;
        DestPhyAddr = 0;
        pDestVirAddr[0] = NULL;
        pDestVirAddr[1] = NULL;
        pDestVirAddr[2] = NULL;

        printf("finish dump, release all, file:%s  \n",pFilePath);

        if(pstChnPort->eModId == E_MI_MODULE_ID_VIF)
        {
            ST_VifPortAttr_t *pstVifPortAttr = NULL;
            MI_U16 u16VifGroupId=pstChnPort->u32DevId/ST_MAX_VIF_DEV_PERGROUP;
            MI_U16 u16VifDevIdInGroup=pstChnPort->u32DevId%ST_MAX_VIF_DEV_PERGROUP;
            pstVifPortAttr = &gstVifModule.stVifGroupAttr[u16VifGroupId].stVifDevAttr[u16VifDevIdInGroup].stVifOutPortAttr[pstChnPort->u32PortId];
            if(pstVifPortAttr->stoutFileAttr.bNeedFbd == TRUE)
            {
                if(pstBufInfo->stFrameData.eCompressMode == E_MI_SYS_COMPRESS_MODE_TO_8BIT)
                {
                    ST_FbdFile(u16DestW,u16DestH,pFilePath);
                }
                else
                {
                    ST_ConvertTo16Bitsbayer(u16DestW,u16DestH,pstBufInfo->stFrameData.ePixelFormat,pFilePath);
                }
            }
        }
    }

EXIT:
    return s32Ret;
}

MI_BOOL ST_GetOutputSizeStepInfo(MI_SYS_ChnPort_t *pstChnPort, ST_StepTestInfo_t *pstStepTestInfo)
{
    MI_S32 s32Ret        = MI_SUCCESS;
    MI_ModuleId_e eModId = pstChnPort->eModId;
    MI_U32 u32DevId      = pstChnPort->u32DevId;
    MI_U32 u32ChnId      = pstChnPort->u32ChnId;
    MI_U32 u32Portid     = pstChnPort->u32PortId;
    MI_VIF_OutputPortAttr_t stOrgVifOutputAttr;
    MI_SCL_OutPortParam_t   stSclOutputParam;
    ST_PortAttr_t  *pstSclOutputAttr = NULL;

    if(eModId == E_MI_MODULE_ID_VIF)
    {
        if((u32DevId<ST_MAX_VIF_DEV_NUM && u32ChnId<1 && u32Portid == 1) == FALSE || ST_MAX_VIF_OUTPORT_NUM <= 1)
        {
            UTStatus = UT_CASE_FAIL;
            DBG_ERR("(%d,%d,%d,%d) param over range(%d,%d,%d), only port1 support scaling down \n", eModId, u32DevId, u32ChnId, u32Portid, ST_MAX_VIF_DEV_NUM, 1, ST_MAX_VIF_OUTPORT_NUM);
            s32Ret = -1;
            goto EXIT;
        }
        MI_U32 u32VifGroupId = u32DevId/ST_MAX_VIF_DEV_PERGROUP;
        MI_U32 u32VifDevIdInGroup = u32DevId%ST_MAX_VIF_DEV_PERGROUP;
        ST_VifPortAttr_t  *pstVifPortAttr=&gstVifModule.stVifGroupAttr[u32VifGroupId].stVifDevAttr[u32VifDevIdInGroup].stVifOutPortAttr[u32Portid];
        if(pstVifPortAttr->bUsed == FALSE)
        {
            UTStatus = UT_CASE_FAIL;
            DBG_ERR("Vif dev:%d, chn:%d, port:%d not enable \n", u32DevId, u32ChnId, u32Portid);
            s32Ret = -1;
            goto EXIT;
        }

        memset(&stOrgVifOutputAttr, 0x0, sizeof(MI_VIF_OutputPortAttr_t));
        ExecFuncResult(MI_VIF_GetOutputPortAttr(u32DevId,u32Portid, &stOrgVifOutputAttr), s32Ret);

        if(stOrgVifOutputAttr.stDestSize.u16Width < stOrgVifOutputAttr.stCapRect.u16Width
            ||stOrgVifOutputAttr.stDestSize.u16Height < stOrgVifOutputAttr.stCapRect.u16Height)
        {
            UTStatus = UT_CASE_FAIL;
            if(stOrgVifOutputAttr.stDestSize.u16Width > ST_VIF_OUTPORT1_SCALING_LIMIT_W)
            {
                DBG_ERR("vif port1 output width %d > scaling limit w %d \n", stOrgVifOutputAttr.stDestSize.u16Width, ST_VIF_OUTPORT1_SCALING_LIMIT_W);
                s32Ret = -1;
                goto EXIT;
            }
        }

        pstStepTestInfo->stStepSize.u16Width    = 2;
        pstStepTestInfo->stStepSize.u16Height   = 2;
        pstStepTestInfo->stLimitSize.u16Width   = 16;
        pstStepTestInfo->stLimitSize.u16Height  = 16;
        pstStepTestInfo->stOutputSize.u16Width  = stOrgVifOutputAttr.stDestSize.u16Width;
        pstStepTestInfo->stOutputSize.u16Height = stOrgVifOutputAttr.stDestSize.u16Height;
        pstStepTestInfo->ePixelFormat   = stOrgVifOutputAttr.ePixFormat;
        pstStepTestInfo->pstoutFileAttr = &pstVifPortAttr->stoutFileAttr;
    }
    else if(eModId == E_MI_MODULE_ID_SCL)
    {
        if(u32DevId >= ST_MAX_SCL_DEV_NUM || u32ChnId >= ST_MAX_SCL_CHN_NUM || u32Portid >= ST_MAX_SCL_OUTPORT_NUM)
        {
            printf("DevId %d chn %d portid %d > max %d %d %d\n", u32DevId, u32ChnId, u32Portid, ST_MAX_SCL_DEV_NUM, ST_MAX_SCL_CHN_NUM, ST_MAX_SCL_OUTPORT_NUM);
            s32Ret = -1;
            goto EXIT;
        }

        pstSclOutputAttr = &gstSclModule.stSclDevAttr[u32DevId].stSclChnlAttr[u32ChnId].stSclOutPortAttr[u32Portid];
        if(pstSclOutputAttr->bEnable == FALSE)
        {
            printf("SCl dev:%d, chn:%d, port:%d not enable \n", u32DevId, u32ChnId, u32Portid);
            s32Ret = -1;
            goto EXIT;
        }

        memset(&stSclOutputParam, 0x0, sizeof(MI_SCL_OutPortParam_t));
        ExecFuncResult(MI_SCL_GetOutputPortParam(u32DevId, u32ChnId, u32Portid, &stSclOutputParam), s32Ret);

        pstStepTestInfo->stStepSize.u16Width    = 8;
        pstStepTestInfo->stStepSize.u16Height   = 2;
        pstStepTestInfo->stLimitSize.u16Width   = 16;
        pstStepTestInfo->stLimitSize.u16Height  = 16;
        pstStepTestInfo->stOutputSize.u16Width  = stSclOutputParam.stSCLOutputSize.u16Width;
        pstStepTestInfo->stOutputSize.u16Height = stSclOutputParam.stSCLOutputSize.u16Height;
        pstStepTestInfo->ePixelFormat   = stSclOutputParam.ePixelFormat;
        pstStepTestInfo->pstoutFileAttr = &pstSclOutputAttr->stoutFileAttr;

        //eMd5Action = pstSclOutputAttr->stoutFileAttr.stMd5Attr.eMd5Action;
        //pstSclOutputAttr->stoutFileAttr.stMd5Attr.eMd5Action = E_ST_MD5_ACTION_NONE;
        //bNeedCheckMd5 = pstSclOutputAttr->stoutFileAttr.bNeedCheckMd5;
        //pstSclOutputAttr->stoutFileAttr.bNeedCheckMd5 = FALSE;
        //pstoutFileAttr = &pstSclOutputAttr->stoutFileAttr;
        //memcpy(u8ExpectMd5Value, pstSclOutputAttr->stoutFileAttr.stMD5ExpectValue[0].u8MD5ExpectValue, sizeof(MI_U8)*16);
    }

    printf("module %d OutStepWH(%d,%d) orgin weight:%d height:%d PixelFormat:%d\n",eModId,
            pstStepTestInfo->stStepSize.u16Width, pstStepTestInfo->stStepSize.u16Height,
            pstStepTestInfo->stOutputSize.u16Width, pstStepTestInfo->stOutputSize.u16Height, pstStepTestInfo->ePixelFormat);

    ExecFuncResult(MI_SYS_SetChnOutputPortDepth(0, pstChnPort, 2, 4), s32Ret);

EXIT:
    return s32Ret;
}

MI_BOOL ST_DoChangeOutputSizeStep(MI_SYS_ChnPort_t *pstChnPort, MI_SYS_WindowSize_t *pstMaxWin, char *pFilePath)
{
#if (MI_VIF_SUPPORT && MI_SCL_SUPPORT)
    char    FilePath[256] = {0};
    MI_S32 s32Ret = MI_SUCCESS;
    MI_S32 s32Fd = 0;
    fd_set read_fds;
    time_t stTime = 0;
    struct timeval TimeoutVal;
    MI_U8 i=0;

    MI_BOOL bout = FALSE;
    //pthread_mutex_t *pOutmutex = NULL;
    MI_U16 u16outputW = 0, u16outputH = 0;
    MI_U16 u16SaveoutputW = 0, u16SaveoutputH = 0;
    MI_SYS_PixelFormat_e ePixelFormat = E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420;
    ST_DumpOutputStatus_e eStatus = E_ST_DUMP_STATUS_BEGIN;
    //MI_SYS_ChnPort_t stChnPort;
    MI_SYS_BufInfo_t stBufInfo;
    MI_SYS_BUF_HANDLE hHandle;
    //MI_SCL_OutPortParam_t  stSclOutputParam;
    //MI_VIF_OutputPortAttr_t stOrgVifOutputAttr;
    ST_OutputFile_Attr_t  *pstoutFileAttr = NULL;

    MI_BOOL bNeedDumpFile = TRUE;
    MD5_CTX stMD5Ctx;
    MI_U8 au8MD5Value[16];
    //MI_BOOL bMd5Same=TRUE;
    //MI_U8 u8ExpectMd5Value[16];
    //MI_BOOL bNeedCheckMd5= FALSE;
    ST_Md5Action_e eMd5Action = E_ST_MD5_ACTION_NONE;

    ST_StepTestInfo_t stStepTestInfo;
    MI_U32 OUTPUT_WEIGHT_STEP = 0;
    MI_U32 OUTPUT_HEIGHT_STEP = 0;
    MI_U32 OUTPUT_LIMITW = 0;
    MI_U32 OUTPUT_LIMITH = 0;
    MI_ModuleId_e eModId = pstChnPort->eModId;
    MI_U32 u32DevId      = pstChnPort->u32DevId;
    MI_U32 u32ChnId      = pstChnPort->u32ChnId;
    MI_U32 u32Portid     = pstChnPort->u32PortId;

    memset(&stStepTestInfo, 0x0, sizeof(ST_StepTestInfo_t));
    if(MI_SUCCESS != ST_GetOutputSizeStepInfo(pstChnPort, &stStepTestInfo))
    {
        ST_ERR("ST_GetOutputSizeStepInfo error\n");
        return -1;
    }

    OUTPUT_WEIGHT_STEP = stStepTestInfo.stStepSize.u16Width;
    OUTPUT_HEIGHT_STEP = stStepTestInfo.stStepSize.u16Height;
    OUTPUT_LIMITW      = stStepTestInfo.stLimitSize.u16Width;
    OUTPUT_LIMITH      = stStepTestInfo.stLimitSize.u16Height;
    u16outputW         = stStepTestInfo.stOutputSize.u16Width;
    u16outputH         = stStepTestInfo.stOutputSize.u16Height;
    pstoutFileAttr     = stStepTestInfo.pstoutFileAttr;

    MD5Init(&stMD5Ctx);
    if(pstoutFileAttr->stMd5Attr.eMd5Action > E_ST_MD5_ACTION_NONE)
    {
       eMd5Action = pstoutFileAttr->stMd5Attr.eMd5Action;
       pstoutFileAttr->stMd5Attr.eMd5Action = E_ST_MD5_ACTION_NONE;
       //memcpy(u8ExpectMd5Value, pstOutFileAttr->stMD5ExpectValue[0].u8MD5ExpectValue, sizeof(MI_U8)*16);
    }

    printf("module %d OutStepWH(%d,%d) orgin weight:%d height:%d PixelFormat:%d \n",eModId,OUTPUT_WEIGHT_STEP,OUTPUT_HEIGHT_STEP,u16outputW,u16outputH,ePixelFormat);

    s32Ret = MI_SYS_GetFd(pstChnPort,&s32Fd);
    if(s32Ret != MI_SUCCESS)
    {
        UTStatus = UT_CASE_FAIL;
        ST_ERR("MI_SYS_GetFd error %d\n",s32Ret);
        return -1;
    }

    u16SaveoutputW = u16outputW;
    u16SaveoutputH = u16outputH;

    if(ST_CheckMkdirOutFile(pFilePath) != MI_SUCCESS)
    {
        bNeedDumpFile = FALSE;
    }

    sprintf(FilePath, "%s/%sdev%dchn%dport%d_%dx%d_pixel%d_%ld.yuv", pFilePath, ((eModId == E_MI_MODULE_ID_VIF) ? "VIF":"SCL"),
    u32DevId,u32ChnId,u32Portid,pstMaxWin->u16Width,pstMaxWin->u16Height,ePixelFormat, time(&stTime));

    pthread_mutex_lock(&pstoutFileAttr->Portmutex);

    while(!bout)
    {
        FD_ZERO(&read_fds);
        FD_SET(s32Fd,&read_fds);
        TimeoutVal.tv_sec = 1;
        TimeoutVal.tv_usec = 0;
        s32Ret = select(s32Fd + 1, &read_fds, NULL, NULL, &TimeoutVal);

        if(s32Ret < 0)
        {
            ST_ERR("select fail\n");
            usleep(THREAD_SLEEP_TIME_US);
            continue;
        }
        else if(s32Ret == 0)
        {
            usleep(THREAD_SLEEP_TIME_US);
            continue;
        }
        else
        {
            if(!FD_ISSET(s32Fd,&read_fds))
                continue;

            memset(&stBufInfo, 0x0, sizeof(MI_SYS_BufInfo_t));
            if (MI_SUCCESS == MI_SYS_ChnOutputPortGetBuf(pstChnPort, &stBufInfo, &hHandle))
            {
                if(u16outputW <= OUTPUT_LIMITW && u16outputH <= OUTPUT_LIMITH)
                    bout = TRUE;

                if(u16outputW > OUTPUT_LIMITW)
                    u16outputW -= OUTPUT_WEIGHT_STEP;

                if(u16outputH > OUTPUT_LIMITH)
                    u16outputH -= OUTPUT_HEIGHT_STEP;

                if(bout == TRUE)
                {
                    u16outputW = u16SaveoutputW;
                    u16outputH = u16SaveoutputH;
                }

                printf("%s set weight:%u, Height:%u \n",eModId == E_MI_MODULE_ID_VIF ? "VIF":"SCL",u16outputW,u16outputH);

                if(eModId == E_MI_MODULE_ID_VIF)
                {
                    MI_VIF_OutputPortAttr_t  stVifOutputAtt;
                    memset(&stVifOutputAtt, 0x0, sizeof(MI_VIF_OutputPortAttr_t));

                    ExecFuncResult(MI_VIF_GetOutputPortAttr(u32DevId, u32Portid, &stVifOutputAtt), s32Ret);

                    stVifOutputAtt.stDestSize.u16Width = u16outputW;
                    stVifOutputAtt.stDestSize.u16Height = u16outputH;

                    ExecFuncResult(ST_DoChangeVifPortAttr(u32DevId, u32Portid, &stVifOutputAtt), s32Ret);
                }
                else if(eModId == E_MI_MODULE_ID_SCL)
                {
                    MI_SCL_OutPortParam_t  stSclOutputParam;
                    memset(&stSclOutputParam, 0x0, sizeof(MI_SCL_OutPortParam_t));

                    ExecFuncResult(MI_SCL_GetOutputPortParam(u32DevId, u32ChnId, u32Portid, &stSclOutputParam), s32Ret);

                    stSclOutputParam.stSCLOutputSize.u16Width = u16outputW;
                    stSclOutputParam.stSCLOutputSize.u16Height = u16outputH;

                    ExecFuncResult(ST_DoChangeSclOutPutParam(u32DevId, u32ChnId, u32Portid, &stSclOutputParam), s32Ret);
                }

                if(bout == TRUE)
                    eStatus = E_ST_DUMP_STATUS_END;

                if(bNeedDumpFile == TRUE)
                {
                    if(MI_SUCCESS != ST_CopyOutputToDestFile(&stBufInfo, pstMaxWin, FilePath, eStatus, pstChnPort))
                        break;
                }

                if(eStatus == E_ST_DUMP_STATUS_BEGIN)
                    eStatus = E_ST_DUMP_STATUS_RUNNING;

                MD5Update(&stMD5Ctx, (MI_U8 *)stBufInfo.stFrameData.pVirAddr[0], stBufInfo.stFrameData.u32BufSize);

                ExecFuncResult(MI_SYS_ChnOutputPortPutBuf(hHandle), s32Ret);
            }
        }
    }

    MD5Final(&stMD5Ctx, au8MD5Value);

    DBG_INFO("MD5 Vaule ");
    for(i=0; i<16; i++)
    {
        printf("%02x",au8MD5Value[i]);
    }
    printf("\n");

    if(eMd5Action > E_ST_MD5_ACTION_NONE)
    {
        ST_MD5Action2(au8MD5Value, &pstoutFileAttr->stMd5Attr, eMd5Action);
    }

    s32Ret = MI_SUCCESS;

EXIT:
    pthread_mutex_unlock(&pstoutFileAttr->Portmutex);
#endif
    return s32Ret;
}

MI_S32 ST_GetCropStepRect(MI_SYS_WindowRect_t *pstOrgRect, MI_SYS_WindowRect_t *pstRect, MI_SYS_WindowSize_t *pstStepWin, MI_SYS_WindowSize_t *pstLimitWin)
{
    static MI_BOOL bWidthStepDone=FALSE;
    static MI_BOOL bHeightStepDone=FALSE;
    MI_BOOL bout =FALSE;

    if((pstRect->u16Width - pstStepWin->u16Width) >= pstLimitWin->u16Width)
    {
        pstRect->u16X += pstStepWin->u16Width;
        pstRect->u16Width -=pstStepWin->u16Width;
    }
    else
    {
        bWidthStepDone=TRUE;
        pstRect->u16X =pstOrgRect->u16X;
        pstRect->u16Width =pstOrgRect->u16Width;
    }

    if((pstRect->u16Height - pstStepWin->u16Height) >= pstLimitWin->u16Height)
    {
        pstRect->u16Y +=pstStepWin->u16Height;
        pstRect->u16Height -=pstStepWin->u16Height;
    }
    else
    {
        bHeightStepDone=TRUE;
        pstRect->u16Y =pstOrgRect->u16Y;
        pstRect->u16Height =pstOrgRect->u16Height;
    }

    if(bWidthStepDone == TRUE && bHeightStepDone==TRUE)
    {
        bout=TRUE;
        bWidthStepDone = FALSE;
        bHeightStepDone =FALSE;

        pstRect->u16X =pstOrgRect->u16X;
        pstRect->u16Width =pstOrgRect->u16Width;
        pstRect->u16Y =pstOrgRect->u16Y;
        pstRect->u16Height =pstOrgRect->u16Height;
    }

    DBG_INFO("Get Cropstep(%d,%d,%d,%d), bout %d\n", pstRect->u16X, pstRect->u16Y, pstRect->u16Width, pstRect->u16Height, bout);
    return bout;
}
MI_BOOL ST_DoChangeIspInputCrop(MI_ISP_DEV DevId, MI_ISP_CHANNEL ChnId,MI_SYS_WindowRect_t *pstCropInfo);
MI_BOOL ST_DoChangeIspOutputParam(MI_ISP_DEV DevId, MI_ISP_CHANNEL ChnId,MI_ISP_PORT IspOutPortId,MI_ISP_OutPortParam_t *pstIspOutputParam);
MI_BOOL ST_DoChangeVifDevAttr(MI_VIF_DEV VifDev,MI_VIF_DevAttr_t *pstDestDevAtt);

MI_BOOL ST_GetCropSizeStepInfo(ST_CropPosition_e eCropPosition, MI_SYS_ChnPort_t *pstChnPort, ST_StepTestInfo_t *pstStepTestInfo)
{
    MI_S32  s32Ret     = MI_SUCCESS;
    MI_U8   u8RetryCnt = 200;
    MI_U32  u32VifGroupId      = 0;
    MI_U32  u32VifDevIdInGroup = 0;

    MI_VIF_DevAttr_t        stOrgVifDevAttr;
    MI_VIF_OutputPortAttr_t stOrgVifOutputAttr;
    MI_SYS_WindowRect_t     stOrgIspInputCropRect;
    MI_ISP_OutPortParam_t   stOrgIspOutputParam;
    MI_SYS_WindowRect_t     stOrgSclInputCropRect;
    MI_SCL_OutPortParam_t   stOrgSclOutputParam;

    if(eCropPosition==E_MI_VIF_DEV_CROP || eCropPosition==E_MI_VIF_OUTPUTPORT_CROP)
    {
        memset(&stOrgVifDevAttr, 0x0, sizeof(MI_VIF_DevAttr_t));
        memset(&stOrgVifOutputAttr, 0x0, sizeof(MI_VIF_OutputPortAttr_t));

        if((pstChnPort->u32DevId<ST_MAX_VIF_DEV_NUM && pstChnPort->u32ChnId<1 && pstChnPort->u32PortId<ST_MAX_VIF_OUTPORT_NUM) == FALSE)
        {
            UTStatus = UT_CASE_FAIL;
            DBG_ERR("(%d,%d,%d,%d) param over range(%d,%d,%d) \n", pstChnPort->eModId, pstChnPort->u32DevId, pstChnPort->u32ChnId, pstChnPort->u32PortId, ST_MAX_VIF_DEV_NUM, 1, ST_MAX_VIF_OUTPORT_NUM);
            return -1;
        }
        u32VifGroupId = pstChnPort->u32DevId/ST_MAX_VIF_DEV_PERGROUP;
        u32VifDevIdInGroup = pstChnPort->u32DevId%ST_MAX_VIF_DEV_PERGROUP;
        //eDestFileModule = E_MI_MODULE_ID_VIF;
        ST_VifPortAttr_t  *pstVifPortAttr=&gstVifModule.stVifGroupAttr[u32VifGroupId].stVifDevAttr[u32VifDevIdInGroup].stVifOutPortAttr[pstChnPort->u32PortId];

        if(pstVifPortAttr->bUsed == FALSE)
        {
            UTStatus = UT_CASE_FAIL;
            DBG_ERR("Vif dev:%d, chn:%d, port:%d not enable \n", pstChnPort->u32DevId, pstChnPort->u32ChnId, pstChnPort->u32PortId);
            return -1;
        }
        pstStepTestInfo->stStepSize.u16Width   = 2;
        pstStepTestInfo->stStepSize.u16Height  = 2;
        pstStepTestInfo->stLimitSize.u16Width  = 16;
        pstStepTestInfo->stLimitSize.u16Height = 16;
        ExecFuncResult(MI_VIF_GetOutputPortAttr(pstChnPort->u32DevId,pstChnPort->u32PortId, &stOrgVifOutputAttr), s32Ret);

        if(eCropPosition==E_MI_VIF_DEV_CROP)
        {
            while(1)
            {
                ExecFuncResult(MI_VIF_GetDevAttr(pstChnPort->u32DevId, &stOrgVifDevAttr), s32Ret);
                if(stOrgVifDevAttr.stInputRect.u16Width > pstStepTestInfo->stLimitSize.u16Width
                    && stOrgVifDevAttr.stInputRect.u16Height > pstStepTestInfo->stLimitSize.u16Height)
                {
                    memcpy(&pstStepTestInfo->stOrgRect, &stOrgVifDevAttr.stInputRect, sizeof(MI_SYS_WindowRect_t));
                    break;
                }
                else
                    u8RetryCnt--;

                if(u8RetryCnt==0)
                {
                    DBG_INFO("Vif dev:%d, chn:%d, port:%d DevInputRect(%d,%d) Smaller than u16CropLimitWH(%d,%d)\n",
                        pstChnPort->u32DevId, pstChnPort->u32ChnId, pstChnPort->u32PortId,stOrgVifDevAttr.stInputRect.u16Width,stOrgVifDevAttr.stInputRect.u16Height,
                        pstStepTestInfo->stLimitSize.u16Width,pstStepTestInfo->stLimitSize.u16Height);
                    u8RetryCnt = 200;
                    //return -1;
                }
                usleep(20*1000);
            }
        }
        else
        {
            while(1)
            {
                ExecFuncResult(MI_VIF_GetOutputPortAttr(pstChnPort->u32DevId,pstChnPort->u32PortId, &stOrgVifOutputAttr), s32Ret);
                if(stOrgVifOutputAttr.stCapRect.u16Width > pstStepTestInfo->stLimitSize.u16Width
                    && stOrgVifOutputAttr.stCapRect.u16Height > pstStepTestInfo->stLimitSize.u16Height)
                {
                    memcpy(&pstStepTestInfo->stOrgRect, &stOrgVifOutputAttr.stCapRect, sizeof(MI_SYS_WindowRect_t));
                    break;
                }
                else
                    u8RetryCnt--;

                if(u8RetryCnt==0)
                {
                    DBG_INFO("Vif dev:%d, chn:%d, port:%d DevInputRect(%d,%d) Smaller than u16CropLimitWH(%d,%d)\n",
                        pstChnPort->u32DevId, pstChnPort->u32ChnId, pstChnPort->u32PortId,stOrgVifOutputAttr.stCapRect.u16Width,stOrgVifOutputAttr.stCapRect.u16Height,
                        pstStepTestInfo->stLimitSize.u16Width,pstStepTestInfo->stLimitSize.u16Height);
                    u8RetryCnt = 200;
                    //return -1;
                }
                usleep(20*1000);
            }
        }

        pstStepTestInfo->stOutputSize.u16Width = pstStepTestInfo->stMaxWin.u16Width;
        pstStepTestInfo->stOutputSize.u16Height = pstStepTestInfo->stMaxWin.u16Height;
        pstStepTestInfo->ePixelFormat = stOrgVifOutputAttr.ePixFormat;
        pstStepTestInfo->pstoutFileAttr = &pstVifPortAttr->stoutFileAttr;
    }
    else if(eCropPosition==E_MI_ISP_INPUTPORT_CROP || eCropPosition==E_MI_ISP_OUTPUTPORT_CROP)
    {
        memset(&stOrgIspInputCropRect, 0x0, sizeof(MI_SYS_WindowRect_t));
        memset(&stOrgIspOutputParam, 0x0, sizeof(MI_ISP_OutPortParam_t));

        if((pstChnPort->u32DevId<ST_MAX_ISP_DEV_NUM && pstChnPort->u32ChnId<ST_MAX_ISP_CHN_NUM && pstChnPort->u32PortId<ST_MAX_ISP_OUTPORT_NUM) ==FALSE)
        {
            UTStatus = UT_CASE_FAIL;
            DBG_ERR("(%d,%d,%d,%d) param over range(%d,%d,%d) \n", pstChnPort->eModId, pstChnPort->u32DevId, pstChnPort->u32ChnId, pstChnPort->u32PortId, ST_MAX_ISP_DEV_NUM, ST_MAX_ISP_CHN_NUM, ST_MAX_ISP_OUTPORT_NUM);
            return -1;
        }
        ST_PortAttr_t  *pstIspOutputAttr = &gstIspModule.stIspDevAttr[pstChnPort->u32DevId].stIspChnlAttr[pstChnPort->u32ChnId].stIspOutPortAttr[pstChnPort->u32PortId];

        if(pstIspOutputAttr->bUsed == FALSE)
        {
            UTStatus = UT_CASE_FAIL;
            DBG_ERR("ISP dev:%d, chn:%d, port:%d not enable \n", pstChnPort->u32DevId, pstChnPort->u32ChnId, pstChnPort->u32PortId);
            return -1;
        }

        pstStepTestInfo->stStepSize.u16Width =16;//bayer format=2
        pstStepTestInfo->stStepSize.u16Height =2;
        //eDestFileModule = E_MI_MODULE_ID_ISP;

        if(eCropPosition==E_MI_ISP_INPUTPORT_CROP)//AWB LimitWH(60,40) && Histo LimitWH(120,5)
        {
            pstStepTestInfo->stLimitSize.u16Width = 120;
            pstStepTestInfo->stLimitSize.u16Height = 40;
            while(1)
            {
                ExecFuncResult(MI_ISP_GetInputPortCrop(pstChnPort->u32DevId, pstChnPort->u32ChnId, &stOrgIspInputCropRect), s32Ret);
                if(stOrgIspInputCropRect.u16Width > pstStepTestInfo->stLimitSize.u16Width
                    && stOrgIspInputCropRect.u16Height > pstStepTestInfo->stLimitSize.u16Height)
                {
                    memcpy(&pstStepTestInfo->stOrgRect, &stOrgIspInputCropRect, sizeof(MI_SYS_WindowRect_t));
                    break;
                }
                else
                    u8RetryCnt--;

                if(u8RetryCnt==0)
                {
                    DBG_INFO("isp dev:%d, chn:%d, port:%d inputwh(%d,%d) Smaller than u16CropLimitWH(%d,%d)\n",
                    pstChnPort->u32DevId, pstChnPort->u32ChnId, pstChnPort->u32PortId,stOrgIspInputCropRect.u16Width,stOrgIspInputCropRect.u16Height,
                    pstStepTestInfo->stLimitSize.u16Width,pstStepTestInfo->stLimitSize.u16Height);
                    u8RetryCnt = 200;
                    //return -1;
                }
                usleep(20*1000);
            }
        }
        else
        {
            pstStepTestInfo->stLimitSize.u16Width = 16;
            pstStepTestInfo->stLimitSize.u16Height = 16;
            while(1)
            {
                ExecFuncResult(MI_ISP_GetOutputPortParam(pstChnPort->u32DevId, pstChnPort->u32ChnId, pstChnPort->u32PortId, &stOrgIspOutputParam), s32Ret);
                if(stOrgIspOutputParam.stCropRect.u16Width > pstStepTestInfo->stLimitSize.u16Width
                    && stOrgIspOutputParam.stCropRect.u16Height > pstStepTestInfo->stLimitSize.u16Height)
                {
                    memcpy(&pstStepTestInfo->stOrgRect, &stOrgIspOutputParam.stCropRect, sizeof(MI_SYS_WindowRect_t));
                    break;
                }
                else
                    u8RetryCnt--;

                if(u8RetryCnt==0)
                {
                    DBG_INFO("isp dev:%d, chn:%d, port:%d outcrop(%d,%d) Smaller than u16CropLimitWH(%d,%d)\n",
                    pstChnPort->u32DevId, pstChnPort->u32ChnId, pstChnPort->u32PortId,stOrgIspOutputParam.stCropRect.u16Width,stOrgIspOutputParam.stCropRect.u16Height,pstStepTestInfo->stLimitSize.u16Width,pstStepTestInfo->stLimitSize.u16Height);
                    u8RetryCnt = 200;
                    //return -1;
                }
                usleep(20*1000);
            }
        }

        ExecFuncResult(MI_ISP_GetOutputPortParam(pstChnPort->u32DevId, pstChnPort->u32ChnId, pstChnPort->u32PortId, &stOrgIspOutputParam), s32Ret);
        pstStepTestInfo->stOutputSize.u16Width  = pstStepTestInfo->stMaxWin.u16Width;
        pstStepTestInfo->stOutputSize.u16Height = pstStepTestInfo->stMaxWin.u16Height;
        pstStepTestInfo->ePixelFormat   = stOrgIspOutputParam.ePixelFormat;
        pstStepTestInfo->pstoutFileAttr = &pstIspOutputAttr->stoutFileAttr;
    }
    else if(eCropPosition==E_MI_SCL_INPUTPORT_CROP || eCropPosition==E_MI_SCL_OUTPUTPORT_CROP)
    {
        memset(&stOrgSclInputCropRect, 0x0, sizeof(MI_SYS_WindowRect_t));
        memset(&stOrgSclOutputParam, 0x0, sizeof(MI_SCL_OutPortParam_t));

        if((pstChnPort->u32DevId<ST_MAX_SCL_DEV_NUM && pstChnPort->u32ChnId<ST_MAX_SCL_CHN_NUM && pstChnPort->u32PortId<ST_MAX_SCL_OUTPORT_NUM) ==FALSE)
        {
            UTStatus = UT_CASE_FAIL;
            DBG_ERR("(%d,%d,%d,%d) param over range(%d,%d,%d) \n", pstChnPort->eModId, pstChnPort->u32DevId, pstChnPort->u32ChnId, pstChnPort->u32PortId, ST_MAX_SCL_DEV_NUM, ST_MAX_SCL_CHN_NUM, ST_MAX_SCL_OUTPORT_NUM);
            return -1;
        }
        ST_PortAttr_t  *pstSclOutputAttr = &gstSclModule.stSclDevAttr[pstChnPort->u32DevId].stSclChnlAttr[pstChnPort->u32ChnId].stSclOutPortAttr[pstChnPort->u32PortId];

        if(pstSclOutputAttr->bEnable == FALSE)
        {
            UTStatus = UT_CASE_FAIL;
            DBG_ERR("SCl dev:%d, chn:%d, port:%d not enable \n", pstChnPort->u32DevId, pstChnPort->u32ChnId, pstChnPort->u32PortId);
            return -1;
        }

        pstStepTestInfo->stLimitSize.u16Width = 16;
        pstStepTestInfo->stLimitSize.u16Height = 16;
        if(eCropPosition==E_MI_SCL_INPUTPORT_CROP)
        {
            pstStepTestInfo->stStepSize.u16Width =16;
            pstStepTestInfo->stStepSize.u16Height =2;

            while(1)
            {
                ExecFuncResult(MI_SCL_GetInputPortCrop(pstChnPort->u32DevId, pstChnPort->u32ChnId, &stOrgSclInputCropRect), s32Ret);
                if(stOrgSclInputCropRect.u16Width > pstStepTestInfo->stLimitSize.u16Width
                    && stOrgSclInputCropRect.u16Height > pstStepTestInfo->stLimitSize.u16Height)
                {
                    memcpy(&pstStepTestInfo->stOrgRect, &stOrgSclInputCropRect, sizeof(MI_SYS_WindowRect_t));
                    break;
                }
                else
                    u8RetryCnt--;

                if(u8RetryCnt==0)
                {
                    DBG_INFO("scl dev:%d, chn:%d, port:%d inputwh(%d,%d) Smaller than u16CropLimitWH(%d,%d)\n",
                    pstChnPort->u32DevId, pstChnPort->u32ChnId, pstChnPort->u32PortId,stOrgSclInputCropRect.u16Width,stOrgSclInputCropRect.u16Height,
                    pstStepTestInfo->stLimitSize.u16Width,pstStepTestInfo->stLimitSize.u16Height);
                    u8RetryCnt = 200;
                    //return -1;
                }
                usleep(20*1000);
            }
        }
        else
        {
            pstStepTestInfo->stStepSize.u16Width =2;
            pstStepTestInfo->stStepSize.u16Height =2;

            while(1)
            {
                ExecFuncResult(MI_SCL_GetOutputPortParam(pstChnPort->u32DevId, pstChnPort->u32ChnId, pstChnPort->u32PortId, &stOrgSclOutputParam), s32Ret);
                if(stOrgSclOutputParam.stSCLOutCropRect.u16Width > pstStepTestInfo->stLimitSize.u16Width
                    && stOrgSclOutputParam.stSCLOutCropRect.u16Height > pstStepTestInfo->stLimitSize.u16Height)
                {
                    memcpy(&pstStepTestInfo->stOrgRect, &stOrgSclOutputParam.stSCLOutCropRect, sizeof(MI_SYS_WindowRect_t));
                    break;
                }
                else
                    u8RetryCnt--;

                if(u8RetryCnt==0)
                {
                    DBG_INFO("scl dev:%d, chn:%d, port:%d orgoutcrop(%d,%d) Smaller than u16CropLimitWH(%d,%d)\n",
                    pstChnPort->u32DevId, pstChnPort->u32ChnId, pstChnPort->u32PortId,stOrgSclOutputParam.stSCLOutCropRect.u16Width,stOrgSclOutputParam.stSCLOutCropRect.u16Height,
                    pstStepTestInfo->stLimitSize.u16Width,pstStepTestInfo->stLimitSize.u16Height);
                    u8RetryCnt = 200;
                    //return -1;
                }
                usleep(20*1000);
            }
        }

        ExecFuncResult(MI_SCL_GetOutputPortParam(pstChnPort->u32DevId, pstChnPort->u32ChnId, pstChnPort->u32PortId, &stOrgSclOutputParam), s32Ret);
        pstStepTestInfo->stOutputSize.u16Width  = stOrgSclOutputParam.stSCLOutputSize.u16Width;
        pstStepTestInfo->stOutputSize.u16Height = stOrgSclOutputParam.stSCLOutputSize.u16Height;

        pstStepTestInfo->ePixelFormat = stOrgSclOutputParam.ePixelFormat;
        pstStepTestInfo->pstoutFileAttr = &pstSclOutputAttr->stoutFileAttr;
    }
    else
    {
        UTStatus = UT_CASE_FAIL;
        DBG_ERR("eCropPosition %d over range \n", eCropPosition);
        s32Ret = -1;
        goto EXIT;
    }

    printf("eCropPosition %d OrgRect(%d,%d,%d,%d) CropStepWH(%d,%d) LimitSizeWH(%d,%d) OutputSizeWH(%d,%d) PixelFormat:%d \n",eCropPosition,
        pstStepTestInfo->stOrgRect.u16X, pstStepTestInfo->stOrgRect.u16Y,
        pstStepTestInfo->stOrgRect.u16Width, pstStepTestInfo->stOrgRect.u16Height,
        pstStepTestInfo->stStepSize.u16Width, pstStepTestInfo->stStepSize.u16Height,
        pstStepTestInfo->stLimitSize.u16Width, pstStepTestInfo->stLimitSize.u16Height,
        pstStepTestInfo->stOutputSize.u16Width, pstStepTestInfo->stOutputSize.u16Height, pstStepTestInfo->ePixelFormat);

EXIT:
    return s32Ret;
}

MI_BOOL ST_DoChangeCropSizeStepTest(ST_CropPosition_e eCropPosition, MI_SYS_ChnPort_t *pstChnPort, MI_SYS_WindowSize_t *pstMaxWin, char *sFilePath)
{
#if (MI_VIF_SUPPORT && MI_ISP_SUPPORT && MI_SCL_SUPPORT)
    //MI_U32  u32VifGroupId=0;
    //MI_U32  u32VifDevIdInGroup=0;
    char  FilePath[256] = {0};
    MI_U8 i=0;

    MI_S32 s32Ret = MI_SUCCESS;
    MI_S32 s32Fd = 0;
    fd_set read_fds;
    time_t stTime = 0;
    struct timeval TimeoutVal;
    //MI_U8  u8RetryCnt=200;

    MI_BOOL bNeedDumpFile = TRUE;
    MD5_CTX stMD5Ctx;
    MI_U8 au8MD5Value[16];
    //MI_BOOL bMd5Same=TRUE;
    //MI_U8 u8ExpectMd5Value[16];
    //MI_BOOL bNeedCheckMd5= FALSE;
    ST_Md5Action_e eMd5Action = E_ST_MD5_ACTION_NONE;
    //MI_ModuleId_e eDestFileModule = E_MI_MODULE_ID_MAX;
    MI_BOOL bout = FALSE;
    MI_U16 u16outputW = 0, u16outputH = 0;
    MI_SYS_PixelFormat_e ePixelFormat = E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420;
    ST_DumpOutputStatus_e eStatus = E_ST_DUMP_STATUS_BEGIN;
    MI_SYS_BufInfo_t stBufInfo;
    MI_SYS_BUF_HANDLE hHandle;
    ST_OutputFile_Attr_t *pstOutFileAttr = NULL;
    MI_SYS_WindowRect_t  stOrgRect;
    MI_SYS_WindowSize_t  stCropStepSize;
    MI_SYS_WindowSize_t  stCropLimitSize;
    ST_StepTestInfo_t    stStepTestInfo;

    memset(&stOrgRect, 0x0, sizeof(MI_SYS_WindowSize_t));
    memset(&stCropStepSize,  0x0, sizeof(MI_SYS_WindowSize_t));
    memset(&stCropLimitSize, 0x0, sizeof(MI_SYS_WindowSize_t));
    memset(&stStepTestInfo,  0x0, sizeof(ST_StepTestInfo_t));

    memcpy(&stStepTestInfo.stMaxWin, pstMaxWin, sizeof(MI_SYS_WindowSize_t));
    if(MI_SUCCESS != ST_GetCropSizeStepInfo(eCropPosition, pstChnPort, &stStepTestInfo))
    {
        ST_ERR("ST_GetOutputSizeStepInfo error\n");
        return -1;
    }
    memcpy(&stOrgRect, &stStepTestInfo.stOrgRect, sizeof(MI_SYS_WindowRect_t));
    memcpy(&stCropStepSize, &stStepTestInfo.stStepSize, sizeof(MI_SYS_WindowSize_t));
    memcpy(&stCropLimitSize, &stStepTestInfo.stLimitSize, sizeof(MI_SYS_WindowSize_t));
    u16outputW     = stStepTestInfo.stOutputSize.u16Width;
    u16outputH     = stStepTestInfo.stOutputSize.u16Height;
    ePixelFormat   = stStepTestInfo.ePixelFormat;
    pstOutFileAttr = stStepTestInfo.pstoutFileAttr;

    MD5Init(&stMD5Ctx);

    while(pstOutFileAttr->u32FinishCnt == 0)
    {
        usleep(THREAD_SLEEP_TIME_US);
    }

    if(pstOutFileAttr->stMd5Attr.eMd5Action > E_ST_MD5_ACTION_NONE)
    {
        eMd5Action = pstOutFileAttr->stMd5Attr.eMd5Action;
        pstOutFileAttr->stMd5Attr.eMd5Action = E_ST_MD5_ACTION_NONE;
        //memcpy(u8ExpectMd5Value, pstOutFileAttr->stMD5ExpectValue[0].u8MD5ExpectValue, sizeof(MI_U8)*16);
    }

    s32Ret = MI_SYS_GetFd(pstChnPort,&s32Fd);
    if(s32Ret != MI_SUCCESS)
    {
        UTStatus = UT_CASE_FAIL;
        ST_ERR("MI_SYS_GetFd error %d\n",s32Ret);
        return -1;
    }

    printf("weight:%d height:%d PixelFormat:%d \n",u16outputW,u16outputH,ePixelFormat);

    if(ST_CheckMkdirOutFile(sFilePath) != MI_SUCCESS)
    {
        bNeedDumpFile = FALSE;
    }

    sprintf(FilePath, "%s/%sdev%dchn%dport%d_dest%dx%d_pixel%d_%ld.yuv", sFilePath,((pstChnPort->eModId==E_MI_MODULE_ID_VIF)? "VIF":
                                                                                (pstChnPort->eModId==E_MI_MODULE_ID_ISP)?"ISP":"SCL"),
    pstChnPort->u32DevId,pstChnPort->u32ChnId,pstChnPort->u32PortId,u16outputW,u16outputH,ePixelFormat, time(&stTime));

    pthread_mutex_lock(&pstOutFileAttr->Portmutex);

    while(!bout)
    {
        FD_ZERO(&read_fds);
        FD_SET(s32Fd,&read_fds);
        TimeoutVal.tv_sec = 1;
        TimeoutVal.tv_usec = 0;
        s32Ret = select(s32Fd + 1, &read_fds, NULL, NULL, &TimeoutVal);

        if(s32Ret < 0)
        {
            ST_ERR("select fail\n");
            usleep(THREAD_SLEEP_TIME_US);
            continue;
        }
        else if(s32Ret == 0)
        {
            usleep(THREAD_SLEEP_TIME_US);
            continue;
        }
        else
        {
            if(!FD_ISSET(s32Fd,&read_fds))
                continue;

            memset(&stBufInfo, 0x0, sizeof(MI_SYS_BufInfo_t));
            if (MI_SUCCESS == MI_SYS_ChnOutputPortGetBuf(pstChnPort, &stBufInfo, &hHandle))
            {
                printf("get buffer success \n");
                if(eCropPosition ==E_MI_VIF_DEV_CROP)
                {
                    MI_VIF_DevAttr_t  stVifDevAttr;
                    memset(&stVifDevAttr, 0x0, sizeof(MI_VIF_DevAttr_t));

                    ExecFuncResult(MI_VIF_GetDevAttr(pstChnPort->u32DevId, &stVifDevAttr), s32Ret);
                    bout =ST_GetCropStepRect(&stOrgRect, &stVifDevAttr.stInputRect, &stCropStepSize, &stCropLimitSize);
                    ExecFuncResult(ST_DoChangeVifDevAttr(pstChnPort->u32DevId, &stVifDevAttr), s32Ret);
                    //usleep(1000*1000*2);
                }
                else if(eCropPosition ==E_MI_VIF_OUTPUTPORT_CROP)
                {
                    MI_VIF_OutputPortAttr_t  stVifOutputAtt;
                    memset(&stVifOutputAtt, 0x0, sizeof(MI_VIF_OutputPortAttr_t));

                    ExecFuncResult(MI_VIF_GetOutputPortAttr(pstChnPort->u32DevId, pstChnPort->u32PortId, &stVifOutputAtt), s32Ret);
                    bout =ST_GetCropStepRect(&stOrgRect, &stVifOutputAtt.stCapRect, &stCropStepSize, &stCropLimitSize);
                    stVifOutputAtt.stDestSize.u16Width = stVifOutputAtt.stCapRect.u16Width;
                    stVifOutputAtt.stDestSize.u16Height = stVifOutputAtt.stCapRect.u16Height;
                    ExecFuncResult(ST_DoChangeVifPortAttr(pstChnPort->u32DevId, pstChnPort->u32PortId, &stVifOutputAtt), s32Ret);
                    //usleep(1000*1000*2);
                }
                else if(eCropPosition ==E_MI_ISP_INPUTPORT_CROP)
                {
                    MI_SYS_WindowRect_t  stIspInputCrop;
                    memset(&stIspInputCrop, 0x0, sizeof(MI_SYS_WindowRect_t));

                    ExecFuncResult(MI_ISP_GetInputPortCrop(pstChnPort->u32DevId, pstChnPort->u32ChnId, &stIspInputCrop), s32Ret);
                    bout =ST_GetCropStepRect(&stOrgRect, &stIspInputCrop, &stCropStepSize, &stCropLimitSize);
                    ExecFuncResult(ST_DoChangeIspInputCrop(pstChnPort->u32DevId, pstChnPort->u32ChnId, &stIspInputCrop), s32Ret);
                }
                else if(eCropPosition ==E_MI_ISP_OUTPUTPORT_CROP)
                {
                    MI_ISP_OutPortParam_t  stIspOutputParam;
                    memset(&stIspOutputParam, 0x0, sizeof(MI_ISP_OutPortParam_t));

                    ExecFuncResult(MI_ISP_GetOutputPortParam(pstChnPort->u32DevId, pstChnPort->u32ChnId, pstChnPort->u32PortId, &stIspOutputParam), s32Ret);
                    bout =ST_GetCropStepRect(&stOrgRect, &stIspOutputParam.stCropRect, &stCropStepSize, &stCropLimitSize);
                    ExecFuncResult(ST_DoChangeIspOutputParam(pstChnPort->u32DevId, pstChnPort->u32ChnId, pstChnPort->u32PortId, &stIspOutputParam), s32Ret);
                }
                else if(eCropPosition ==E_MI_SCL_INPUTPORT_CROP)
                {
                    MI_SYS_WindowRect_t  stSclInputRect;
                    memset(&stSclInputRect, 0x0, sizeof(MI_SYS_WindowRect_t));

                    ExecFuncResult(MI_SCL_GetInputPortCrop(pstChnPort->u32DevId, pstChnPort->u32ChnId, &stSclInputRect), s32Ret);
                    bout =ST_GetCropStepRect(&stOrgRect, &stSclInputRect, &stCropStepSize, &stCropLimitSize);
                    ExecFuncResult(ST_DoChangeSclInputCrop(pstChnPort->u32DevId, pstChnPort->u32ChnId, &stSclInputRect), s32Ret);
                }
                else if(eCropPosition ==E_MI_SCL_OUTPUTPORT_CROP)
                {
                    MI_SCL_OutPortParam_t  stSclOutputParam;
                    memset(&stSclOutputParam, 0x0, sizeof(MI_SCL_OutPortParam_t));

                    ExecFuncResult(MI_SCL_GetOutputPortParam(pstChnPort->u32DevId, pstChnPort->u32ChnId, pstChnPort->u32PortId, &stSclOutputParam), s32Ret);
                    bout =ST_GetCropStepRect(&stOrgRect, &stSclOutputParam.stSCLOutCropRect, &stCropStepSize, &stCropLimitSize);
                    ExecFuncResult(ST_DoChangeSclOutPutParam(pstChnPort->u32DevId, pstChnPort->u32ChnId,pstChnPort->u32PortId, &stSclOutputParam), s32Ret);
                }

                if(bout == TRUE)
                {
                    eStatus = E_ST_DUMP_STATUS_END;
                }

                if(bNeedDumpFile == TRUE)
                {
                    if(eCropPosition==E_MI_SCL_INPUTPORT_CROP || eCropPosition==E_MI_SCL_OUTPUTPORT_CROP)
                    {
                        if(MI_SUCCESS != ST_DumpOutputToFile(&stBufInfo, FilePath, eStatus))
                            break;
                    }
                    else
                    {
                        if(MI_SUCCESS != ST_CopyOutputToDestFile(&stBufInfo, pstMaxWin, FilePath, eStatus, pstChnPort))
                            break;
                    }
                }

                if(eStatus == E_ST_DUMP_STATUS_BEGIN)
                    eStatus = E_ST_DUMP_STATUS_RUNNING;

                MD5Update(&stMD5Ctx, (MI_U8 *)stBufInfo.stFrameData.pVirAddr[0], stBufInfo.stFrameData.u32BufSize);

                ExecFuncResult(MI_SYS_ChnOutputPortPutBuf(hHandle), s32Ret);

                printf("put buf done \n");

            }
        }
    }

    MD5Final(&stMD5Ctx, au8MD5Value);

    DBG_INFO("MD5 Vaule ");
    for(i=0; i<16; i++)
    {
        printf("%02x",au8MD5Value[i]);
    }
    printf("\n");

    if(eMd5Action > E_ST_MD5_ACTION_NONE)
    {
        ST_MD5Action2(au8MD5Value, &pstOutFileAttr->stMd5Attr, eMd5Action);
    }

    s32Ret = MI_SUCCESS;

EXIT:
    pthread_mutex_unlock(&pstOutFileAttr->Portmutex);
#endif
    return s32Ret;
}

static __attribute__((unused)) MI_BOOL ST_CreatZoomTable
(MI_ISP_ZoomTable_t *const pZoomTable,const MI_SYS_WindowRect_t *pstStart,const MI_SYS_WindowRect_t *pstDest,
const MI_SYS_WindowRect_t * pstStepWH,const MI_U32 u32Num,const MI_U8 u8SensorID)
{
    MI_BOOL bRet = MI_SUCCESS;

    if(pZoomTable == NULL)
        return -1;

    pZoomTable->pVirTableAddr = (MI_ISP_ZoomEntry_t *)malloc(sizeof(MI_ISP_ZoomEntry_t) * u32Num);
    if(pZoomTable->pVirTableAddr == NULL)
        return -1;

    MI_U32 i = 0;
    MI_U16 u16CropW = pstStart->u16Width;
    MI_U16 u16CropH = pstStart->u16Height;
    MI_U16 u16CropX = pstStart->u16X;
    MI_U16 u16CropY = pstStart->u16Y;
    for(i = 0; i < u32Num; i++)
    {
        pZoomTable->pVirTableAddr[i].stCropWin.u16Width = u16CropW;
        pZoomTable->pVirTableAddr[i].stCropWin.u16Height= u16CropH;
        pZoomTable->pVirTableAddr[i].stCropWin.u16X = u16CropX;
        pZoomTable->pVirTableAddr[i].stCropWin.u16Y = u16CropY;
        printf("[%d-%d][%d-%d-%d-%d]\n",u32Num,i,u16CropX,u16CropY,u16CropW,u16CropH);

        if(u16CropW != pstDest->u16Width)
            u16CropW = u16CropW > pstDest->u16Width ?(u16CropW - pstStepWH->u16Width) :
                                                        (u16CropW + pstStepWH->u16Width);
        if(u16CropH != pstDest->u16Height)
            u16CropH = u16CropH > pstDest->u16Height ?(u16CropH - pstStepWH->u16Height) :
                                                        (u16CropH + pstStepWH->u16Height);
        if(u16CropX != pstDest->u16X)
            u16CropX = u16CropX > pstDest->u16X ?(u16CropX - pstStepWH->u16X) :
                                                        (u16CropX + pstStepWH->u16X);
        if(u16CropY != pstDest->u16Y)
            u16CropY = u16CropY > pstDest->u16Y ?(u16CropY - pstStepWH->u16Y) :
                                                        (u16CropY + pstStepWH->u16Y);

        pZoomTable->pVirTableAddr[i].u8ZoomSensorId = u8SensorID;
    }
    pZoomTable->u32EntryNum = i;
    return bRet;

}

static __attribute__((unused)) MI_BOOL ST_DestroyZoomTable(MI_ISP_ZoomTable_t * pZoomTable)
{
    MI_BOOL bRet = MI_SUCCESS;

    free(pZoomTable->pVirTableAddr);
    pZoomTable->pVirTableAddr = NULL;

    return bRet;
}

MI_BOOL ST_DoSetIspZoom(MI_ISP_DEV DevId,MI_ISP_CHANNEL ChnId,MI_BOOL bRev,MI_SYS_WindowRect_t *pstStart,MI_SYS_WindowRect_t *pstDest,MI_SYS_WindowRect_t *pstStepWH)
{
    MI_BOOL bRet = MI_SUCCESS;
#if MI_ISP_SUPPORT

    MI_U32 u32Num = 0;
    MI_U32 u32XComp = 0,u32YComp = 0,u32WComp = 0,u32HComp = 0;
    MI_U32 u32WADV = 0,u32HADV = 0;

    MI_ISP_ZoomTable_t stZoomTable;
    MI_ISP_ZoomAttr_t stZoomAttr;
    MI_ISP_ZoomAttr_t stZoomAttrInfo;
    MI_SYS_WindowRect_t stOrgRect;
    MI_SYS_WindowRect_t stIspInCropInfo;
    MI_SYS_WindowSize_t stIspInputSrcSize;
    MI_ISP_ChnParam_t stIspChnParam;
    memset(&stZoomTable, 0x0, sizeof(MI_ISP_ZoomTable_t));
    memset(&stOrgRect, 0x0, sizeof(MI_SYS_WindowRect_t));
    memset(&stIspInCropInfo, 0x0, sizeof(MI_SYS_WindowRect_t));
    memset(&stIspInputSrcSize, 0x0, sizeof(MI_SYS_WindowSize_t));
    memset(&stIspChnParam, 0x0, sizeof(MI_ISP_ChnParam_t));

    if(pstStepWH->u16X == 0 || pstStepWH->u16Y == 0
        || pstStepWH->u16Width == 0 || pstStepWH->u16Height == 0)
    {
        pstStepWH->u16X = 2;
        pstStepWH->u16Y = 2;
        pstStepWH->u16Width = 2;
        pstStepWH->u16Height = 2;
    }

    ExecFuncResult(MI_ISP_GetInputPortCrop(DevId,ChnId,&stIspInCropInfo), bRet);
    ExecFuncResult(MI_ISP_GetChnParam(DevId,ChnId,&stIspChnParam), bRet);
    stIspInputSrcSize.u16Width = stIspInCropInfo.u16Width;
    stIspInputSrcSize.u16Height = stIspInCropInfo.u16Height;
    //SCL SrcSize default use isp outputport0
    memcpy(&stOrgRect,pstStart,sizeof(MI_SYS_WindowRect_t));
    ST_GetRotAfterCropRect(stIspInputSrcSize,stOrgRect,stIspChnParam.eRot,stIspChnParam.bMirror,stIspChnParam.bFlip,pstStart);
    memcpy(&stOrgRect,pstDest,sizeof(MI_SYS_WindowRect_t));
    ST_GetRotAfterCropRect(stIspInputSrcSize,stOrgRect,stIspChnParam.eRot,stIspChnParam.bMirror,stIspChnParam.bFlip,pstDest);

    if(stIspChnParam.eRot == E_MI_SYS_ROTATE_90
    || stIspChnParam.eRot == E_MI_SYS_ROTATE_270)
    {
        SWAP(pstStepWH->u16Width, pstStepWH->u16Height);
        SWAP(pstStepWH->u16X, pstStepWH->u16Y);
    }

    u32XComp = (pstStart->u16X > pstDest->u16X ? (pstStart->u16X - pstDest->u16X) :
                                            (pstDest->u16X - pstStart->u16X))/pstStepWH->u16X;
    u32YComp = (pstStart->u16Y > pstDest->u16Y ? (pstStart->u16Y - pstDest->u16Y) :
                                            (pstDest->u16Y - pstStart->u16Y))/pstStepWH->u16Y;
    u32WADV = pstStart->u16Width > pstDest->u16Width ? (pstStart->u16Width - pstDest->u16Width) :
                                                    (pstDest->u16Width - pstStart->u16Width);
    u32HADV = pstStart->u16Height > pstDest->u16Height ? (pstStart->u16Height - pstDest->u16Height) :
                                                        (pstDest->u16Height - pstStart->u16Height);

    u32WComp = u32WADV/pstStepWH->u16Width;
    u32HComp = u32HADV/pstStepWH->u16Height;
    u32Num = MAX(u32XComp,MAX(u32YComp,MAX(u32WComp,u32HComp)));


    printf("[%d][%d-%d-%d-%d]\n",u32Num,u32WADV,u32HADV,u32XComp,u32YComp);
    printf("pstStart[%d-%d-%d-%d]\n",pstStart->u16X,pstStart->u16Y,pstStart->u16Width,pstStart->u16Height);
    printf("pstDest[%d-%d-%d-%d]\n",pstDest->u16X,pstDest->u16Y,pstDest->u16Width,pstDest->u16Height);
    printf("pstStepWH[%d-%d-%d-%d]\n",pstStepWH->u16X,pstStepWH->u16Y,pstStepWH->u16Width,pstStepWH->u16Height);

    ExecFuncResult(ST_CreatZoomTable(&stZoomTable,pstStart,pstDest,pstStepWH,u32Num,0), bRet);

    memset(&stZoomAttr, 0x0, sizeof(MI_ISP_ZoomAttr_t));
    stZoomAttr.u32FromEntryIndex = 0;
    stZoomAttr.u32ToEntryIndex = stZoomTable.u32EntryNum-1;
    stZoomAttr.u32CurEntryIndex = stZoomAttr.u32FromEntryIndex;

    ExecFuncResult(MI_ISP_LoadPortZoomTable(DevId,ChnId,&stZoomTable), bRet);
    ExecFuncResult(MI_ISP_StartPortZoom(DevId,ChnId,&stZoomAttr), bRet);

    memset(&stZoomAttrInfo, 0x0, sizeof(MI_ISP_ZoomAttr_t));
    while(stZoomAttrInfo.u32CurEntryIndex != stZoomAttr.u32ToEntryIndex)//wait zoom done
    {
        ExecFuncResult(MI_ISP_GetPortCurZoomAttr(DevId,ChnId,&stZoomAttrInfo), bRet);
        sleep(1);
    }

    if(bRev)
    {
        memset(&stZoomAttr, 0x0, sizeof(MI_ISP_ZoomAttr_t));
        stZoomAttr.u32FromEntryIndex = stZoomTable.u32EntryNum-1;
        stZoomAttr.u32ToEntryIndex = 0;
        stZoomAttr.u32CurEntryIndex = stZoomAttr.u32FromEntryIndex;

        ExecFuncResult(MI_ISP_StopPortZoom(DevId,ChnId), bRet);
        ExecFuncResult(MI_ISP_StartPortZoom(DevId,ChnId,&stZoomAttr), bRet);

        while(stZoomAttrInfo.u32CurEntryIndex != stZoomAttr.u32ToEntryIndex)//wait zoom done
        {
            ExecFuncResult(MI_ISP_GetPortCurZoomAttr(DevId,ChnId,&stZoomAttrInfo), bRet);
            sleep(1);
        }
    }

    ExecFuncResult(MI_ISP_StopPortZoom(DevId,ChnId), bRet);
    ST_DestroyZoomTable(&stZoomTable);

EXIT:
#endif
    return bRet;
}

MI_BOOL ST_DoSetIspSkipFrame(MI_ISP_DEV IspDev,MI_ISP_CHANNEL IspChn, MI_U32 u32FrameNum)
{
    MI_BOOL bRet = MI_SUCCESS;
#if MI_ISP_SUPPORT

    if(u32FrameNum)
    {
        ST_IspChannelAttr_t  *pstIspChnAttr = &gstIspModule.stIspDevAttr[IspDev].stIspChnlAttr[IspChn];

        pthread_mutex_lock(&pstIspChnAttr->stIspSkipFarme.SkipMutex);
        pstIspChnAttr->stIspSkipFarme.u32SikpFrameCnt = u32FrameNum;
        gettimeofday(&pstIspChnAttr->stIspSkipFarme.stSatrtTime, NULL);
        pthread_mutex_unlock(&pstIspChnAttr->stIspSkipFarme.SkipMutex);
        //DBG_INFO("stSatrtTime %lu\n",(pstIspChnAttr->stIspSkipFarme.stSatrtTime.tv_sec * 1000000 + pstIspChnAttr->stIspSkipFarme.stSatrtTime.tv_usec));
        ExecFuncResult(MI_ISP_SkipFrame(IspDev,IspChn,u32FrameNum), bRet);
    }
EXIT:
#endif
    return bRet;
}

//for vif input pixel change
MI_BOOL ST_DoChangeVifGroupAttr(MI_VIF_GROUP VifGroup, MI_SYS_PixelFormat_e eInputPixel)
{
#if MI_VIF_SUPPORT
    MI_VIF_PORT vifPort = 0;
    MI_VIF_DEV vifDevIdPerGroup = 0;

    ST_VifGroupAttr_t  *pstVifGroupAttr = &gstVifModule.stVifGroupAttr[VifGroup];

    if(pstVifGroupAttr->bCreate != TRUE || pstVifGroupAttr->bUsed != TRUE)
    {
        DBG_WRN("VIF Group:%d is not working \n", VifGroup);
        return MI_SUCCESS;
    }

    STCHECKRESULT(ST_VifModuleUnInit(VifGroup));

    for(vifDevIdPerGroup = 0; vifDevIdPerGroup < ST_MAX_VIF_DEV_PERGROUP; vifDevIdPerGroup++)
    {
        ST_VifDevAttr_t *pstDevAttr = &pstVifGroupAttr->stVifDevAttr[vifDevIdPerGroup];

        if(pstDevAttr->bUsed != TRUE)
            continue;

        pstDevAttr->stVifDevAttr.eInputPixel = eInputPixel;

        for(vifPort=0; vifPort< ST_MAX_VIF_OUTPORT_NUM; vifPort++)
        {
            ST_VifPortAttr_t *pstVifPortAttr = &pstDevAttr->stVifOutPortAttr[vifPort];
            if(pstVifPortAttr->bUsed != TRUE)
                continue;

            pstVifPortAttr->ePixFormat = pstDevAttr->stVifDevAttr.eInputPixel;
        }
    }

    STCHECKRESULT(ST_VifModuleInit(VifGroup));
#endif
    return MI_SUCCESS;
}

MI_BOOL ST_DoChangeVifDevAttr(MI_VIF_DEV VifDev,MI_VIF_DevAttr_t *pstDestDevAtt)
{
    MI_BOOL bRet = MI_SUCCESS;
#if MI_VIF_SUPPORT

    MI_U32 i = 0;
    MI_VIF_PORT vifPort = 0;
    MI_VIF_GROUP GroupId = VifDev/ST_MAX_VIF_DEV_PERGROUP;
    MI_U32 vifDevIdPerGroup = VifDev%ST_MAX_VIF_DEV_PERGROUP;

    ST_VifGroupAttr_t  *pstVifGroupAttr = &gstVifModule.stVifGroupAttr[GroupId];
    ST_VifDevAttr_t *pstDevAttr = &pstVifGroupAttr->stVifDevAttr[vifDevIdPerGroup];
    MI_VIF_DevAttr_t stSrcDevAttr;
    MI_VIF_GroupAttr_t stGroupAttr;
    memset(&stSrcDevAttr, 0x0, sizeof(MI_VIF_DevAttr_t));
    memset(&stGroupAttr, 0x0, sizeof(MI_VIF_GroupAttr_t));

    pthread_mutex_lock(&pstDevAttr->Devmutex);

    /************************************************
    Step1: stop ISP ==>realtime
    *************************************************/
    for(i = 0; i < ST_MAX_VIF_OUTPORT_NUM; i++)
    {
        ST_VifPortAttr_t *pstVifPortAttr = &pstDevAttr->stVifOutPortAttr[i];
        if(pstVifPortAttr->bCreate == TRUE && pstVifPortAttr->bUsed == TRUE)
        {
            ST_Output_BindParam_List_t *pHead = NULL, *pListPos = NULL, *pNext = NULL;
            pHead = &pstVifPortAttr->head;
            list_for_each_entry_safe(pListPos,pNext,&pHead->pos,pos)
            {
#if MI_ISP_SUPPORT
                MI_ISP_DEV DevId = pListPos->stBindInfo.stDstChnPort.u32DevId;
                MI_ISP_CHANNEL ChnId = pListPos->stBindInfo.stDstChnPort.u32ChnId;
                if(pListPos->stBindInfo.eBindType == E_MI_SYS_BIND_TYPE_REALTIME)
                {
                    ExecFuncResult(MI_ISP_StopChannel(DevId,ChnId), bRet);
                    ExecFuncResult(ST_Sys_UnBind(&pListPos->stBindInfo), bRet);
                }
#endif
            }
        }
    }

    /************************************************
    Step1: Set Vif DevAttr
    *************************************************/
    ExecFuncResult(MI_VIF_GetDevGroupAttr(GroupId,&stGroupAttr), bRet);
    ExecFuncResult(MI_VIF_GetDevAttr(VifDev,&stSrcDevAttr), bRet);

    if(pstDestDevAtt->stInputRect.u16Height != 0
        ||pstDestDevAtt->stInputRect.u16Width != 0
        ||pstDestDevAtt->stInputRect.u16X != 0
        ||pstDestDevAtt->stInputRect.u16Y != 0)
    {
        if(stGroupAttr.eIntfMode == E_MI_VIF_MODE_MIPI || stGroupAttr.eIntfMode == E_MI_VIF_MODE_LVDS ||stGroupAttr.eIntfMode == E_MI_VIF_MODE_DIGITAL_CAMERA)
            memcpy(&stSrcDevAttr.stInputRect,&pstDestDevAtt->stInputRect,sizeof(MI_SYS_WindowRect_t));
        else
        {
            UTStatus = UT_CASE_FAIL;
            DBG_ERR("Dev crop only support MIPI\n");
        }
    }
    stSrcDevAttr.bEnH2T1PMode = pstDestDevAtt->bEnH2T1PMode;
    if(E_MI_SYS_FIELDTYPE_NUM != pstDestDevAtt->eField)//if MAX ,use org
        stSrcDevAttr.eField = pstDestDevAtt->eField;
    if(E_MI_SYS_PIXEL_FRAME_FORMAT_MAX != pstDestDevAtt->eInputPixel)//if MAX ,use org
        stSrcDevAttr.eInputPixel = pstDestDevAtt->eInputPixel;

    /************************************************
    Step2: Disable Vif dev
    *************************************************/
    for(vifPort=0; vifPort< ST_MAX_VIF_OUTPORT_NUM; vifPort++)
    {
        ST_VifPortAttr_t *pstVifOutputPortAttr = &pstDevAttr->stVifOutPortAttr[vifPort];
        if(pstVifOutputPortAttr->bUsed == TRUE && pstVifOutputPortAttr->bCreate== TRUE)
        {
            ExecFuncResult(MI_VIF_DisableOutputPort(VifDev, vifPort), bRet);
            pstVifOutputPortAttr->bCreate = FALSE;
        }
    }
    ExecFuncResult(MI_VIF_DisableDev(VifDev), bRet);
    pstDevAttr->bCreate=FALSE;

    /************************************************
    Step3: Enable Vif dev
    *************************************************/
    ExecFuncResult(MI_VIF_SetDevAttr(VifDev,&stSrcDevAttr), bRet);
    ExecFuncResult(MI_VIF_EnableDev(VifDev), bRet);
    pstDevAttr->bCreate=TRUE;

    for(vifPort=0; vifPort< ST_MAX_VIF_OUTPORT_NUM; vifPort++)
    {
        ST_VifPortAttr_t *pstVifOutputPortAttr = &pstDevAttr->stVifOutPortAttr[vifPort];
        if(pstVifOutputPortAttr->bUsed == TRUE && pstVifOutputPortAttr->bCreate== FALSE)
        {

            MI_VIF_OutputPortAttr_t  stVifOutPortAttr;
            memset(&stVifOutPortAttr, 0x0, sizeof(MI_VIF_OutputPortAttr_t));
            ExecFuncResult(MI_VIF_GetOutputPortAttr(VifDev, vifPort, &stVifOutPortAttr), bRet);

            stVifOutPortAttr.stCapRect.u16X = 0;
            stVifOutPortAttr.stCapRect.u16Y = 0;
            stVifOutPortAttr.stCapRect.u16Width = stSrcDevAttr.stInputRect.u16Width;
            stVifOutPortAttr.stCapRect.u16Height = stSrcDevAttr.stInputRect.u16Height;
            stVifOutPortAttr.stDestSize.u16Width = stSrcDevAttr.stInputRect.u16Width;
            stVifOutPortAttr.stDestSize.u16Height = stSrcDevAttr.stInputRect.u16Height;

            if(stVifOutPortAttr.ePixFormat == E_MI_SYS_PIXEL_FRAME_YUV422_YUYV
                || stVifOutPortAttr.ePixFormat == E_MI_SYS_PIXEL_FRAME_YUV422_UYVY
                || stVifOutPortAttr.ePixFormat == E_MI_SYS_PIXEL_FRAME_YUV422_YVYU
                || stVifOutPortAttr.ePixFormat == E_MI_SYS_PIXEL_FRAME_YUV422_VYUY)
            {
                stVifOutPortAttr.ePixFormat = stSrcDevAttr.eInputPixel;
            }

            if(stSrcDevAttr.bEnH2T1PMode == TRUE)
            {
                stVifOutPortAttr.stCapRect.u16Width = stVifOutPortAttr.stCapRect.u16Width/2;
                stVifOutPortAttr.stDestSize.u16Width = stVifOutPortAttr.stDestSize.u16Width/2;
            }

            ExecFuncResult(MI_VIF_SetOutputPortAttr(VifDev, vifPort, &stVifOutPortAttr), bRet);
            ExecFuncResult(MI_VIF_EnableOutputPort(VifDev, vifPort), bRet);
            pstVifOutputPortAttr->bCreate = TRUE;
        }
    }

    /************************************************
    Step7: ISP Start
    *************************************************/
    for(i = 0; i < ST_MAX_VIF_OUTPORT_NUM; i++)
    {
        ST_VifPortAttr_t *pstVifPortAttr = &pstDevAttr->stVifOutPortAttr[i];
        if(pstVifPortAttr->bCreate == TRUE && pstVifPortAttr->bUsed == TRUE)
        {
            ST_Output_BindParam_List_t *pHead = NULL, *pListPos = NULL, *pNext = NULL;
            pHead = &pstVifPortAttr->head;
            list_for_each_entry_safe(pListPos,pNext,&pHead->pos,pos)
            {
#if MI_ISP_SUPPORT
                MI_ISP_DEV IspDevId = pListPos->stBindInfo.stDstChnPort.u32DevId;
                MI_ISP_CHANNEL IspChnId = pListPos->stBindInfo.stDstChnPort.u32ChnId;
                if(pListPos->stBindInfo.eBindType == E_MI_SYS_BIND_TYPE_REALTIME)
                {
                    ExecFuncResult(ST_Sys_Bind(&pListPos->stBindInfo), bRet);
                    ExecFuncResult(MI_ISP_StartChannel(IspDevId,IspChnId), bRet);
                }
#endif
            }
        }
    }

EXIT:
    pthread_mutex_unlock(&pstDevAttr->Devmutex);
#endif
    return bRet;
}

MI_BOOL ST_DoChangeIspOutputParam(MI_ISP_DEV DevId, MI_ISP_CHANNEL ChnId,MI_ISP_PORT IspOutPortId,MI_ISP_OutPortParam_t *pstIspOutputParam)
{
    MI_BOOL bRet = MI_SUCCESS;
#if MI_ISP_SUPPORT

    MI_ISP_OutPortParam_t stOrgIspOutputParam;
    memset(&stOrgIspOutputParam, 0x0, sizeof(MI_ISP_OutPortParam_t));

    ExecFuncResult(MI_ISP_GetOutputPortParam(DevId,ChnId,IspOutPortId,&stOrgIspOutputParam), bRet);

    if(IspOutPortId == 1)
    {
        if(pstIspOutputParam->stCropRect.u16Width != 0 && pstIspOutputParam->stCropRect.u16Height != 0)
        {
            memcpy(&stOrgIspOutputParam.stCropRect,&pstIspOutputParam->stCropRect,sizeof(MI_SYS_WindowRect_t));
        }
        if(pstIspOutputParam->eCompressMode != E_MI_SYS_COMPRESS_MODE_BUTT)
        {
            stOrgIspOutputParam.eCompressMode = pstIspOutputParam->eCompressMode;
        }
        if(pstIspOutputParam->ePixelFormat != E_MI_SYS_PIXEL_FRAME_FORMAT_MAX)
        {
            stOrgIspOutputParam.ePixelFormat = pstIspOutputParam->ePixelFormat;
        }
    }
    else
    {
        DBG_WRN("Isp dev%d chn%d port%d not support changge output param\n",DevId,ChnId,IspOutPortId);
        return 0;
    }

    ExecFuncResult(MI_ISP_DisableOutputPort(DevId, ChnId,IspOutPortId), bRet);

    ExecFuncResult(MI_ISP_SetOutputPortParam(DevId,ChnId,IspOutPortId,&stOrgIspOutputParam), bRet);

    ExecFuncResult(MI_ISP_EnableOutputPort(DevId, ChnId,IspOutPortId), bRet);
EXIT:
#endif
    return bRet;
}

MI_BOOL ST_DoChangeIspInputCrop(MI_ISP_DEV DevId, MI_ISP_CHANNEL ChnId,MI_SYS_WindowRect_t *pstCropInfo)
{
    MI_BOOL bRet = MI_SUCCESS;
#if MI_ISP_SUPPORT

    MI_SYS_WindowRect_t stOrgIspInputRect;
    memset(&stOrgIspInputRect, 0x0, sizeof(MI_SYS_WindowRect_t));

    ExecFuncResult(MI_ISP_GetInputPortCrop(DevId,ChnId,&stOrgIspInputRect), bRet);

    DBG_INFO("ISP[%d-%d] orginputcrop(%d,%d,%d,%d),Destinputcrop(%d,%d,%d,%d)\n",DevId,ChnId,
    stOrgIspInputRect.u16X,stOrgIspInputRect.u16Y,stOrgIspInputRect.u16Width,stOrgIspInputRect.u16Height,
    pstCropInfo->u16X,pstCropInfo->u16Y,pstCropInfo->u16Width,pstCropInfo->u16Height);

    memcpy(&stOrgIspInputRect,pstCropInfo,sizeof(MI_SYS_WindowRect_t));
    ExecFuncResult(MI_ISP_SetInputPortCrop(DevId,ChnId,&stOrgIspInputRect), bRet);

EXIT:
#endif
    return bRet;
}

MI_BOOL ST_DoChangeIsp3dnr(MI_ISP_DEV IspDev, MI_ISP_CHANNEL IspChn, MI_ISP_3DNR_Level_e e3dnrlevel)
{
    MI_BOOL bRet = MI_SUCCESS;
#if MI_ISP_SUPPORT

    ST_IspChannelAttr_t  *pstIspChnAttr = NULL;

    if(IspDev >= ST_MAX_ISP_DEV_NUM || IspChn >= ST_MAX_ISP_CHN_NUM)
    {
        printf("IspDev %d, IspChn %d > max dev %d chn %d \n", IspDev, IspChn, ST_MAX_ISP_DEV_NUM, ST_MAX_ISP_CHN_NUM);
        return -1;
    }

    if(e3dnrlevel >= E_MI_ISP_3DNR_LEVEL_NUM)
    {
        printf(" 3dnrlevel %d is invalid \n", e3dnrlevel);
        return -1;
    }

    pstIspChnAttr = &gstIspModule.stIspDevAttr[IspDev].stIspChnlAttr[IspChn];
    if(pstIspChnAttr->bCreate != TRUE || pstIspChnAttr->bUsed != TRUE)
    {
        printf("ISP channel:%d not working \n",IspChn);
        return -1;
    }

    ExecFuncResult(MI_ISP_StopChannel(IspDev, IspChn), bRet);

    ExecFuncResult(MI_ISP_GetChnParam(IspDev, IspChn, &pstIspChnAttr->stIspChnParam), bRet);
    pstIspChnAttr->stIspChnParam.e3DNRLevel = e3dnrlevel;
    ExecFuncResult(MI_ISP_SetChnParam(IspDev, IspChn, &pstIspChnAttr->stIspChnParam), bRet);

    ExecFuncResult(MI_ISP_StartChannel(IspDev, IspChn), bRet);

EXIT:
#endif
    return bRet;
}

MI_BOOL ST_DoChangeHdrRes(MI_VIF_GROUP GroupId,MI_BOOL bUsedHdr, MI_U8 u8ResIndex)
{
    MI_BOOL bRet = MI_SUCCESS;
#if (MI_VIF_SUPPORT && MI_ISP_SUPPORT)

    MI_BOOL bSetPlaneMode = FALSE;
    MI_U8 i = 0,u8ChocieRes = 0;
    MI_U32 u32ResCount = 0;
    MI_U32 u32DevIdInGroup = 0;
    ST_VifGroupAttr_t *pstVifGroupAttr = &gstVifModule.stVifGroupAttr[GroupId];
    ST_VifDevAttr_t *pstVifDevAttr = &pstVifGroupAttr->stVifDevAttr[u32DevIdInGroup];
    MI_SNR_PADID eSnrPadId = (MI_SNR_PADID)pstVifGroupAttr->stBindSensor.eSensorPadID;
    MI_VIF_HDRType_e eSelectHDR = E_MI_VIF_HDR_TYPE_OFF;
    MI_SNR_Res_t stRes;

    MI_SNR_PADInfo_t  stPad0Info;
    memset(&stPad0Info, 0x0, sizeof(MI_SNR_PADInfo_t));

    /************************************************
    Step1: ISP Stop ==> wait driver all buffer done
    *************************************************/
    for(i = 0; i < ST_MAX_VIF_OUTPORT_NUM; i++)
    {
        ST_VifPortAttr_t *pstVifPortAttr = &pstVifDevAttr->stVifOutPortAttr[i];
        if(pstVifPortAttr->bCreate == TRUE && pstVifPortAttr->bUsed == TRUE)
        {
            ST_Output_BindParam_List_t *pHead = NULL, *pListPos = NULL, *pNext = NULL;
            pHead = &pstVifPortAttr->head;
            list_for_each_entry_safe(pListPos,pNext,&pHead->pos,pos)
            {
                MI_ISP_DEV DevId = pListPos->stBindInfo.stDstChnPort.u32DevId;
                MI_ISP_CHANNEL ChnId = pListPos->stBindInfo.stDstChnPort.u32ChnId;

                ExecFuncResult(MI_ISP_StopChannel(DevId,ChnId), bRet);
                ExecFuncResult(ST_Sys_UnBind(&pListPos->stBindInfo), bRet);
            }
        }
    }
    /************************************************
    Step2:  Destory VIF
    *************************************************/
    ExecFuncResult(ST_VifModuleUnInit(GroupId), bRet);

    /************************************************
    Step3:  Destory Sensor
    *************************************************/
    ExecFuncResult(MI_SNR_Disable(eSnrPadId), bRet);

    /************************************************
    Step4: Choice Hdr Type
    *************************************************/
    if(bUsedHdr == TRUE)
    {
        ExecFuncResult(MI_SNR_GetPadInfo(eSnrPadId, &stPad0Info), bRet);
        bSetPlaneMode = TRUE;
        eSelectHDR = (MI_VIF_HDRType_e)stPad0Info.eHDRMode;
    }
    /************************************************
    Step5:  Init Sensor
    *************************************************/
    ExecFuncResult(MI_SNR_SetPlaneMode(eSnrPadId, bSetPlaneMode), bRet);

    ExecFuncResult(MI_SNR_QueryResCount(eSnrPadId, &u32ResCount), bRet);
    for(i=0; i < u32ResCount; i++)
    {
        memset(&stRes, 0x0, sizeof(MI_SNR_Res_t));
        ExecFuncResult(MI_SNR_GetRes(eSnrPadId, i, &stRes), bRet);
        printf("index %d, Crop(%d,%d,%d,%d), outputsize(%d,%d), maxfps %d, minfps %d, ResDesc %s\n",i,
        stRes.stCropRect.u16X, stRes.stCropRect.u16Y, stRes.stCropRect.u16Width,stRes.stCropRect.u16Height,
        stRes.stOutputSize.u16Width, stRes.stOutputSize.u16Height,
        stRes.u32MaxFps,stRes.u32MinFps,
        stRes.strResDesc);
    }

    if(u8ResIndex> u32ResCount)
    {
        do
        {
            MI_S32 s32ScanfTemp = 0;

            scanf("%d", &s32ScanfTemp);
            ST_Flush();
            u8ChocieRes = (MI_U8)s32ScanfTemp;
            ExecFuncResult(MI_SNR_QueryResCount(eSnrPadId, &u32ResCount), bRet);
            if(u8ChocieRes >= u32ResCount)
            {
                UTStatus = UT_CASE_FAIL;
                printf("choice err res %d > =cnt %d,Please Reenter\n", u8ChocieRes, u32ResCount);
            }
        }while(u8ChocieRes >= u32ResCount);
    }
    else
    {
        u8ChocieRes = u8ResIndex;
    }
    printf("You select %d res\n", u8ChocieRes);

    memset(&stRes, 0x0, sizeof(MI_SNR_Res_t));
    ExecFuncResult(MI_SNR_GetRes(eSnrPadId, u8ChocieRes, &stRes), bRet);

    ExecFuncResult(MI_SNR_SetRes(eSnrPadId,u8ChocieRes), bRet);
    ExecFuncResult(MI_SNR_Enable(eSnrPadId), bRet);

    /************************************************
    Step6:  init VIF
    *************************************************/
    for(i = 0; i < ST_MAX_VIF_OUTPORT_NUM; i++)
    {
        ST_VifPortAttr_t *pstVifPortAttr = &pstVifDevAttr->stVifOutPortAttr[i];
        memset(&pstVifPortAttr->stCapRect, 0x0, sizeof(MI_SYS_WindowRect_t));
        memset(&pstVifPortAttr->stDestSize, 0x0, sizeof(MI_SYS_WindowSize_t));
        if(i == 0)
            pstVifPortAttr->ePixFormat = E_MI_SYS_PIXEL_FRAME_FORMAT_MAX; //use sensor param
    }
    pstVifDevAttr->stVifDevAttr.eInputPixel = E_MI_SYS_PIXEL_FRAME_FORMAT_MAX; //use sensor param
    pstVifGroupAttr->eHDRType = eSelectHDR;
    printf("eSelectHDR %d %d\n",pstVifGroupAttr->eHDRType,eSelectHDR);
    ExecFuncResult(ST_VifModuleInit(GroupId), bRet);

    /************************************************
    Step7: ISP Start
    *************************************************/
    for(i = 0; i < ST_MAX_VIF_OUTPORT_NUM; i++)
    {
        ST_VifPortAttr_t *pstVifPortAttr = &pstVifDevAttr->stVifOutPortAttr[i];
        if(pstVifPortAttr->bCreate == TRUE && pstVifPortAttr->bUsed == TRUE)
        {
            ST_Output_BindParam_List_t *pVifHead = NULL, *pVifListPos = NULL, *pVifNext = NULL;
            MI_VIF_OutputPortAttr_t  stOutputAttr;
            memset(&stOutputAttr, 0x0, sizeof(MI_VIF_OutputPortAttr_t));

            pVifHead = &pstVifPortAttr->head;
            list_for_each_entry_safe(pVifListPos,pVifNext,&pVifHead->pos,pos)
            {
                ST_Output_BindParam_List_t *pIspHead = NULL, *pIspListPos = NULL, *pIspNext = NULL;
                MI_ISP_DEV IspDevId = pVifListPos->stBindInfo.stDstChnPort.u32DevId;
                MI_ISP_CHANNEL IspChnId = pVifListPos->stBindInfo.stDstChnPort.u32ChnId;
                MI_ISP_PORT IspPort = 0;

                MI_ISP_ChnParam_t stIspChnParam;
                memset(&stIspChnParam, 0x0, sizeof(MI_ISP_ChnParam_t));

                ExecFuncResult(ST_Sys_Bind(&pVifListPos->stBindInfo), bRet);

                ExecFuncResult(MI_ISP_GetChnParam(IspDevId,IspChnId, &stIspChnParam), bRet);
                stIspChnParam.eHDRType = (MI_ISP_HDRType_e)eSelectHDR;
                ExecFuncResult(MI_ISP_SetChnParam(IspDevId,IspChnId, &stIspChnParam), bRet);

                for(IspPort = 0; IspPort < ST_MAX_ISP_OUTPORT_NUM; IspPort++)
                {
                    ST_PortAttr_t *pstIspPortAttr= &gstIspModule.stIspDevAttr[IspDevId].stIspChnlAttr[IspChnId].stIspOutPortAttr[IspPort];
                    if(pstIspPortAttr->bUsed == TRUE)
                    {
                        pIspHead = &pstIspPortAttr->head;
                        list_for_each_entry_safe(pIspListPos,pIspNext,&pIspHead->pos,pos)
                        {
                            MI_SCL_DEV SclDevId = pIspListPos->stBindInfo.stDstChnPort.u32DevId;
                            MI_SCL_CHANNEL SclChnId = pIspListPos->stBindInfo.stDstChnPort.u32ChnId;
                            MI_SCL_PORT SclPortId = 0;//default only set scl port0
                            ST_PortAttr_t *pstSclPortAttr= &gstSclModule.stSclDevAttr[SclDevId].stSclChnlAttr[SclChnId].stSclOutPortAttr[SclPortId];

                            MI_SCL_OutPortParam_t stOutputParam;
                            memset(&stOutputParam, 0x0, sizeof(MI_SCL_OutPortParam_t));
                            if(pstSclPortAttr->bUsed == TRUE && pstSclPortAttr->bEnable == TRUE)
                            {
#if MI_SCL_SUPPORT
                                ExecFuncResult(MI_SCL_GetOutputPortParam(SclDevId, SclChnId, SclPortId, &stOutputParam), bRet);
#endif
                                memcpy(&stOutputParam.stSCLOutCropRect,&stRes.stCropRect,sizeof(MI_SYS_WindowRect_t));
                                stOutputParam.stSCLOutputSize.u16Width = stRes.stCropRect.u16Width;
                                stOutputParam.stSCLOutputSize.u16Height = stRes.stCropRect.u16Height;
                                ExecFuncResult(ST_DoChangeSclOutPutParam(SclDevId, SclChnId, SclPortId, &stOutputParam), bRet);
                            }
                        }
                    }
                }
                ExecFuncResult(MI_ISP_StartChannel(IspDevId,IspChnId), bRet);
            }
        }
    }

EXIT:
#endif
    return bRet;
}
MI_BOOL ST_DoChangeVifOutPutOnOff(MI_VIF_DEV DevId, MI_VIF_PORT PortId, MI_BOOL bOn)
{
    MI_BOOL bRet = MI_SUCCESS;
#if MI_VIF_SUPPORT
    MI_VIF_GROUP GroupId = DevId/ST_MAX_VIF_DEV_PERGROUP;
    MI_U32 vifDevIdPerGroup = DevId%ST_MAX_VIF_DEV_PERGROUP;
    ST_VifDevAttr_t *pstDevAttr = &gstVifModule.stVifGroupAttr[GroupId].stVifDevAttr[vifDevIdPerGroup];
    ST_VifPortAttr_t *pstVifOutputPortAttr = &pstDevAttr->stVifOutPortAttr[PortId];

    if(pstDevAttr->bCreate != TRUE || pstDevAttr->bUsed != TRUE)
    {
        return -1;
    }

    pthread_mutex_lock(&pstDevAttr->Devmutex);
    if(bOn == FALSE)
    {
        if(pstVifOutputPortAttr->bUsed == TRUE && pstVifOutputPortAttr->bCreate == TRUE)
        {
            printf("dev:%d port:%d disable\n",DevId, PortId);
            ExecFuncResult(MI_VIF_DisableOutputPort(DevId, PortId), bRet);
            pstVifOutputPortAttr->bCreate = FALSE;
        }
    }
    else
    {
        if(pstVifOutputPortAttr->bUsed == TRUE && pstVifOutputPortAttr->bCreate == FALSE)
        {
            printf("dev:%d port:%d enable\n",DevId, PortId);
            ExecFuncResult(MI_VIF_EnableOutputPort(DevId, PortId), bRet);
            pstVifOutputPortAttr->bCreate = TRUE;
        }
    }
EXIT:
    pthread_mutex_unlock(&pstDevAttr->Devmutex);
#endif
    return bRet;
}

MI_BOOL ST_DoChangeVifDevOnOff(MI_U32 DevId, MI_BOOL bOn)
{
    MI_BOOL bRet = MI_SUCCESS;
#if MI_VIF_SUPPORT

    MI_VIF_GROUP GroupId = DevId/ST_MAX_VIF_DEV_PERGROUP;
    MI_U32 vifDevIdPerGroup = DevId%ST_MAX_VIF_DEV_PERGROUP;
    ST_VifGroupAttr_t *pstVifGroupAttr = &gstVifModule.stVifGroupAttr[GroupId];
    ST_VifDevAttr_t *pstDevAttr = &pstVifGroupAttr->stVifDevAttr[vifDevIdPerGroup];
    MI_U32 u32vifport = 0;
    ST_VifPortAttr_t *pstVifOutputPortAttr = NULL;

    pthread_mutex_lock(&pstDevAttr->Devmutex);
    if(bOn == FALSE)
    {
        if(pstDevAttr->bCreate == TRUE && pstDevAttr->bUsed == TRUE)
        {
            printf("dev:%d disable \n",DevId);
            for(u32vifport = 0; u32vifport < ST_MAX_VIF_OUTPORT_NUM; u32vifport++)
            {
                pstVifOutputPortAttr = &pstDevAttr->stVifOutPortAttr[u32vifport];
                if(pstVifOutputPortAttr->bCreate == TRUE && pstVifOutputPortAttr->bUsed == TRUE)
                {
                    ExecFuncResult(MI_VIF_DisableOutputPort(DevId, u32vifport), bRet);
                    pstVifOutputPortAttr->bCreate = FALSE;
                }
            }
            ExecFuncResult(MI_VIF_DisableDev(DevId), bRet);
            pstDevAttr->bCreate = FALSE;
        }
     }
     else
     {
        if(pstDevAttr->bCreate == FALSE && pstDevAttr->bUsed == TRUE)
        {
            printf("dev:%d enable \n",DevId);
            ExecFuncResult(MI_VIF_EnableDev(DevId), bRet);
            pstDevAttr->bCreate = TRUE;

            for(u32vifport = 0; u32vifport < ST_MAX_VIF_OUTPORT_NUM; u32vifport++)
            {
                pstVifOutputPortAttr = &pstDevAttr->stVifOutPortAttr[u32vifport];
                if(pstVifOutputPortAttr->bCreate == FALSE && pstVifOutputPortAttr->bUsed == TRUE)
                {
                    ExecFuncResult(MI_VIF_EnableOutputPort(DevId, u32vifport), bRet);
                    pstVifOutputPortAttr->bCreate = TRUE;
                }
            }
        }
    }
EXIT:
    pthread_mutex_unlock(&pstDevAttr->Devmutex);
#endif
    return bRet;
}

MI_S32 ST_DoChangeChnPortOnOff(MI_SYS_ChnPort_t *pstChnPort,ST_DynamicChnPortPosition_e ePosition,MI_BOOL bstatus)
{
    MI_S32 s32Result = MI_SUCCESS;

    if(pstChnPort->eModId == E_MI_MODULE_ID_ISP)
    {
#if MI_ISP_SUPPORT
        ST_IspChannelAttr_t  *pstIspChnAttr = &gstIspModule.stIspDevAttr[pstChnPort->u32DevId].stIspChnlAttr[pstChnPort->u32ChnId];

        if(ePosition == E_CHNPORT_POSITION_CHN)
        {
            if(bstatus == TRUE)
            {
                if(pstIspChnAttr->bCreate == FALSE && pstIspChnAttr->bUsed == TRUE)
                {
                    STCHECKRESULT(MI_ISP_StartChannel(pstChnPort->u32DevId,pstChnPort->u32ChnId));
                    pstIspChnAttr->bCreate = TRUE;

                }
            }
            else
            {
                if(pstIspChnAttr->bCreate == TRUE && pstIspChnAttr->bUsed == TRUE)
                {
                    STCHECKRESULT(MI_ISP_StopChannel(pstChnPort->u32DevId,pstChnPort->u32ChnId));
                    pstIspChnAttr->bCreate = FALSE;
                }
            }
        }
        else if(ePosition == E_CHNPORT_POSITION_PORT)
        {
            ST_PortAttr_t *pstIspOutputAttr = &pstIspChnAttr->stIspOutPortAttr[pstChnPort->u32PortId];

            if(bstatus == TRUE)
            {
                if(pstIspOutputAttr->bUsed == TRUE && pstIspOutputAttr->bEnable == FALSE)
                {
                    STCHECKRESULT(MI_ISP_EnableOutputPort(pstChnPort->u32DevId,pstChnPort->u32ChnId,pstChnPort->u32PortId));
                    pstIspOutputAttr->bEnable = TRUE;
                }
            }
            else
            {
                if(pstIspOutputAttr->bUsed == TRUE && pstIspOutputAttr->bEnable == TRUE)
                {
                    STCHECKRESULT(MI_ISP_DisableOutputPort(pstChnPort->u32DevId,pstChnPort->u32ChnId,pstChnPort->u32PortId));
                    pstIspOutputAttr->bEnable = FALSE;
                }
            }
        }

#endif
    }
    else if(pstChnPort->eModId == E_MI_MODULE_ID_SCL)
    {
#if MI_SCL_SUPPORT

        ST_SclChannelAttr_t *pstSclChnattr = &gstSclModule.stSclDevAttr[pstChnPort->u32DevId].stSclChnlAttr[pstChnPort->u32ChnId];

        if(ePosition == E_CHNPORT_POSITION_CHN)
        {
            if(bstatus == TRUE)
            {
                if(pstSclChnattr->bCreate == FALSE && pstSclChnattr->bUsed == TRUE)
                {
                    STCHECKRESULT(MI_SCL_StartChannel(pstChnPort->u32DevId,pstChnPort->u32ChnId));
                    pstSclChnattr->bCreate = TRUE;

                }
            }
            else
            {
                if(pstSclChnattr->bCreate == TRUE && pstSclChnattr->bUsed == TRUE)
                {
                    STCHECKRESULT(MI_SCL_StopChannel(pstChnPort->u32DevId,pstChnPort->u32ChnId));
                    pstSclChnattr->bCreate = FALSE;
                }
            }
        }
        else if(ePosition == E_CHNPORT_POSITION_PORT)
        {
            ST_PortAttr_t *pstSclOutputAttr = &pstSclChnattr->stSclOutPortAttr[pstChnPort->u32PortId];

            if(bstatus == TRUE)
            {
                if(pstSclOutputAttr->bUsed == TRUE && pstSclOutputAttr->bEnable == FALSE)
                {
                    STCHECKRESULT(MI_SCL_EnableOutputPort(pstChnPort->u32DevId,pstChnPort->u32ChnId,pstChnPort->u32PortId));
                    pstSclOutputAttr->bEnable = TRUE;
                }
            }
            else
            {
                if(pstSclOutputAttr->bUsed == TRUE && pstSclOutputAttr->bEnable == TRUE)
                {
                    STCHECKRESULT(MI_SCL_DisableOutputPort(pstChnPort->u32DevId,pstChnPort->u32ChnId,pstChnPort->u32PortId));
                    pstSclOutputAttr->bEnable = FALSE;
                }
            }
        }
#endif
    }
    else
    {
        DBG_ERR("module %d not support \n", pstChnPort->eModId);
        s32Result = -1;
        UTStatus = UT_CASE_FAIL;
    }

    return s32Result;
}

MI_BOOL ST_DoHwReset(MI_ModuleId_e eModuleId, MI_U32 u32DevId, MI_U32 u32ChnId, MI_U32 u32pos)
{
    __attribute__((unused)) char cmd[256];

    if(eModuleId == E_MI_MODULE_ID_VIF)
    {
#if MI_VIF_SUPPORT
        MI_VIF_GROUP GroupId = u32DevId/ST_MAX_VIF_DEV_PERGROUP;
        MI_U32 vifDevIdPerGroup = u32DevId%ST_MAX_VIF_DEV_PERGROUP;
        ST_VifGroupAttr_t  *pstVifGroupAttr = &gstVifModule.stVifGroupAttr[GroupId];
        ST_VifDevAttr_t *pstDevAttr = &pstVifGroupAttr->stVifDevAttr[vifDevIdPerGroup];

        if(pstDevAttr->bCreate == TRUE && pstDevAttr->bUsed == TRUE)
        {
            memset(cmd, 0x0, sizeof(char)*256);
            sprintf(cmd, "echo resethw %hhu %hhu > /proc/mi_modules/mi_vif/mi_vif%hhu\n",u32DevId, u32pos, u32DevId);
            system(cmd);
            printf("cmd:%s",cmd);
        }
#endif
    }
    else if(eModuleId == E_MI_MODULE_ID_ISP)
        /*1.use sys cmdq reset, isp/scl set busehwreset=true
          2.set scl precrop active triger cmdq err
        */
    {
#if MI_ISP_SUPPORT

        memset(cmd, 0x0, sizeof(char)*256);
        sprintf(cmd, "echo setprecrop 0 0 0 0 2 2 > /proc/mi_modules/mi_scl/mi_scl%hhu\n",u32DevId);
        system(cmd);
        printf("cmd:%s",cmd);
        usleep(100*THREAD_SLEEP_TIME_US);

        memset(cmd, 0x0, sizeof(char)*256);
        sprintf(cmd, "echo setprecrop 0 0 0 0 1920 1080 > /proc/mi_modules/mi_scl/mi_scl%hhu\n", u32DevId);
        system(cmd);
        printf("cmd:%s",cmd);
        usleep(100*THREAD_SLEEP_TIME_US);
#endif
    }
    else if(eModuleId == E_MI_MODULE_ID_SCL)
    {
#if MI_SCL_SUPPORT
        memset(cmd, 0x0, sizeof(char)*256);
        sprintf(cmd, "echo resethw 0 > /proc/mi_modules/mi_scl/mi_scl%hhu\n", u32DevId);
        system(cmd);
        printf("cmd:%s",cmd);
        usleep(100*THREAD_SLEEP_TIME_US);
#endif
    }

    return 0;
}

MI_BOOL ST_DoCrcOnOff(MI_ModuleId_e eModuleId, MI_U32 u32DevId, MI_U32 u32ChnId, MI_U32 u32PortId, MI_BOOL bOn)
{
    __attribute__((unused)) char cmd[256];

    if(eModuleId == E_MI_MODULE_ID_VIF)
    {
#if MI_VIF_SUPPORT
        MI_VIF_GROUP GroupId = u32DevId/ST_MAX_VIF_DEV_PERGROUP;
        MI_U32 vifDevIdPerGroup = u32DevId%ST_MAX_VIF_DEV_PERGROUP;
        ST_VifPortAttr_t *pstVifOutputPortAttr = &gstVifModule.stVifGroupAttr[GroupId].stVifDevAttr[vifDevIdPerGroup].stVifOutPortAttr[u32PortId];

        if(pstVifOutputPortAttr->bCreate != TRUE || pstVifOutputPortAttr->bUsed != TRUE)
        {
            DBG_ERR("vif dev%d port%d is not working \n", u32DevId, u32PortId);
            return -1;
        }

        memset(cmd, 0x0, sizeof(char)*256);
        sprintf(cmd, "echo crc %hhu %hhu > /proc/mi_modules/mi_vif/mi_vif%hhu\n",u32DevId, bOn, u32DevId);
        system(cmd);
        printf("cmd:%s",cmd);
#endif
    }
    else if(eModuleId == E_MI_MODULE_ID_ISP)
    {
#if MI_ISP_SUPPORT

        ST_IspChannelAttr_t  *pstIspChnAttr = &gstIspModule.stIspDevAttr[u32DevId].stIspChnlAttr[u32ChnId];
        ST_PortAttr_t *pstIspOutputAttr = &pstIspChnAttr->stIspOutPortAttr[u32PortId];

        if(pstIspChnAttr->bUsed != TRUE || pstIspOutputAttr->bUsed != TRUE)
        {
            DBG_ERR("isp dev%d port%d is not working \n", u32DevId, u32PortId);
            return -1;
        }
        memset(cmd, 0x0, sizeof(char)*256);
        sprintf(cmd, "echo crcmode %hhu %hhu > /proc/mi_modules/mi_isp/mi_isp%hhu\n",u32ChnId, bOn, u32DevId);
        system(cmd);
        printf("cmd:%s",cmd);

#endif
    }
    else if(eModuleId == E_MI_MODULE_ID_SCL)
    {
#if MI_SCL_SUPPORT

        ST_SclChannelAttr_t *pstSclChnattr = &gstSclModule.stSclDevAttr[u32DevId].stSclChnlAttr[u32ChnId];
        ST_PortAttr_t *pstSclOutputAttr = &pstSclChnattr->stSclOutPortAttr[u32PortId];

        if(pstSclChnattr->bUsed != TRUE || pstSclOutputAttr->bEnable != TRUE)
        {
            DBG_ERR("scl dev%d port%d is not working \n", u32DevId, u32PortId);
            return -1;
        }

        memset(cmd, 0x0, sizeof(char)*256);
        sprintf(cmd, "echo crcmode %hhu %hhu %hhu > /proc/mi_modules/mi_scl/mi_scl%hhu\n",
                    u32ChnId,u32PortId, bOn, u32DevId);
        system(cmd);
        printf("cmd:%s",cmd);

#endif
    }

    return 0;
}

MI_S32 ST_DoDestroyCreateIspChannel(MI_ISP_DEV IspDevId)
{
    MI_U8 u8IspDevId=IspDevId;
    MI_U8 u8ChnId=0, u8PortId=0;
    MI_S32 s32Ret = MI_SUCCESS;

    for(u8ChnId=0; u8ChnId< ST_MAX_ISP_CHN_NUM; u8ChnId++)
    {
        ST_IspChannelAttr_t *pstIspChnAttr = &gstIspModule.stIspDevAttr[u8IspDevId].stIspChnlAttr[u8ChnId];
        if(pstIspChnAttr->bUsed == TRUE)
        {
            MI_SYS_WindowRect_t stIspInputCrop;
            memset(&stIspInputCrop, 0x0, sizeof(MI_SYS_WindowRect_t));

            ST_Sys_BindInfo_T stBindInfo;
            memset(&stBindInfo, 0x0, sizeof(ST_Sys_BindInfo_T));
            stBindInfo.stSrcChnPort.eModId = pstIspChnAttr->stIspInPortAttr[0].stBindParam.stChnPort.eModId;
            stBindInfo.stSrcChnPort.u32DevId = pstIspChnAttr->stIspInPortAttr[0].stBindParam.stChnPort.u32DevId;
            stBindInfo.stSrcChnPort.u32ChnId = pstIspChnAttr->stIspInPortAttr[0].stBindParam.stChnPort.u32ChnId;
            stBindInfo.stSrcChnPort.u32PortId = pstIspChnAttr->stIspInPortAttr[0].stBindParam.stChnPort.u32PortId;

            stBindInfo.stDstChnPort.eModId = E_MI_MODULE_ID_ISP;
            stBindInfo.stDstChnPort.u32DevId = u8IspDevId;
            stBindInfo.stDstChnPort.u32ChnId = u8ChnId;
            stBindInfo.stDstChnPort.u32PortId = 0;
            stBindInfo.u32SrcFrmrate = 30;
            stBindInfo.u32DstFrmrate = 30;
            ExecFuncResult(ST_Sys_UnBind(&stBindInfo), s32Ret);

            for(u8PortId=0; u8PortId<ST_MAX_ISP_OUTPORT_NUM; u8PortId++)
            {
                ST_PortAttr_t *pstIspOutPortAttr = &pstIspChnAttr->stIspOutPortAttr[u8PortId];
                if(pstIspOutPortAttr->bUsed == TRUE)
                {
                    ST_Output_BindParam_List_t *pIspHead = NULL, *pIspListPos = NULL, *pIspNext = NULL;
                    pIspHead = &pstIspOutPortAttr->head;
                    list_for_each_entry_safe(pIspListPos,pIspNext,&pIspHead->pos,pos)
                    {
                        ExecFuncResult(ST_Sys_UnBind(&pIspListPos->stBindInfo), s32Ret);
                    }
                    ExecFuncResult(MI_ISP_DisableOutputPort(u8IspDevId, u8ChnId, u8PortId), s32Ret);
                }
            }

            ExecFuncResult(MI_ISP_GetInputPortCrop(u8IspDevId, u8ChnId, &stIspInputCrop), s32Ret);

            ExecFuncResult(MI_ISP_StopChannel(u8IspDevId, u8ChnId), s32Ret);
            ExecFuncResult(MI_ISP_DestroyChannel(u8IspDevId, u8ChnId), s32Ret);

            usleep(THREAD_SLEEP_TIME_US*10);

            ExecFuncResult(MI_ISP_CreateChannel(u8IspDevId, u8ChnId, &pstIspChnAttr->stIspChnAttr), s32Ret);
            ExecFuncResult(MI_ISP_SetInputPortCrop(u8IspDevId, u8ChnId, &stIspInputCrop), s32Ret);
            ExecFuncResult(MI_ISP_SetChnParam(u8IspDevId, u8ChnId, &pstIspChnAttr->stIspChnParam), s32Ret);
            ExecFuncResult(MI_ISP_StartChannel(u8IspDevId, u8ChnId), s32Ret);

            memset(&stBindInfo, 0x0, sizeof(ST_Sys_BindInfo_T));
            stBindInfo.stSrcChnPort.eModId = pstIspChnAttr->stIspInPortAttr[0].stBindParam.stChnPort.eModId;
            stBindInfo.stSrcChnPort.u32DevId = pstIspChnAttr->stIspInPortAttr[0].stBindParam.stChnPort.u32DevId;
            stBindInfo.stSrcChnPort.u32ChnId = pstIspChnAttr->stIspInPortAttr[0].stBindParam.stChnPort.u32ChnId;
            stBindInfo.stSrcChnPort.u32PortId = pstIspChnAttr->stIspInPortAttr[0].stBindParam.stChnPort.u32PortId;

            stBindInfo.stDstChnPort.eModId = E_MI_MODULE_ID_ISP;
            stBindInfo.stDstChnPort.u32DevId = u8IspDevId;
            stBindInfo.stDstChnPort.u32ChnId = u8ChnId;
            stBindInfo.stDstChnPort.u32PortId = 0;

            stBindInfo.u32SrcFrmrate = pstIspChnAttr->stIspInPortAttr[0].stBindParam.u32SrcFrmrate;
            stBindInfo.u32DstFrmrate = pstIspChnAttr->stIspInPortAttr[0].stBindParam.u32DstFrmrate;
            stBindInfo.eBindType = pstIspChnAttr->stIspInPortAttr[0].stBindParam.eBindType;
            ExecFuncResult(ST_Sys_Bind(&stBindInfo), s32Ret);

            for(u8PortId=0; u8PortId<ST_MAX_ISP_OUTPORT_NUM; u8PortId++)
            {
                ST_PortAttr_t *pstIspOutPortAttr = &pstIspChnAttr->stIspOutPortAttr[u8PortId];
                if(pstIspOutPortAttr->bUsed == TRUE)
                {
                    ST_Output_BindParam_List_t *pIspHead = NULL, *pIspListPos = NULL, *pIspNext = NULL;
                    MI_ISP_OutPortParam_t stIspOutputParam;
                    memset(&stIspOutputParam, 0x0, sizeof(MI_ISP_OutPortParam_t));

                    stIspOutputParam.ePixelFormat = pstIspOutPortAttr->ePixelFormat;

                    ExecFuncResult(MI_ISP_SetOutputPortParam(u8IspDevId, u8ChnId, u8PortId, &stIspOutputParam), s32Ret);
                    ExecFuncResult(MI_ISP_EnableOutputPort(u8IspDevId, u8ChnId, u8PortId), s32Ret);

                     pIspHead = &pstIspOutPortAttr->head;
                     list_for_each_entry_safe(pIspListPos,pIspNext,&pIspHead->pos,pos)
                     {
                         ExecFuncResult(ST_Sys_Bind(&pIspListPos->stBindInfo), s32Ret);
                     }
                }
            }
        }
    }

EXIT:
    return s32Ret;
}

MI_S32 ST_DoDestroyCreateSclChannel(MI_SCL_DEV DevId)
{
    MI_U8 u8SclDevId=DevId;
    MI_U8 u8ChnId=0, u8PortId=0;
    MI_S32 s32Ret = MI_SUCCESS;

    for(u8ChnId=0; u8ChnId< ST_MAX_SCL_CHN_NUM; u8ChnId++)
    {
        ST_SclChannelAttr_t *pstSclChnAttr = &gstSclModule.stSclDevAttr[u8SclDevId].stSclChnlAttr[u8ChnId];
        MI_SCL_OutPortParam_t stOutputParam[ST_MAX_SCL_OUTPORT_NUM];
        memset(stOutputParam, 0x0, sizeof(MI_SCL_OutPortParam_t)*ST_MAX_SCL_OUTPORT_NUM);

        if(pstSclChnAttr->bUsed == TRUE)
        {
            MI_SYS_WindowRect_t stSclInputCrop;
            MI_SCL_ChannelAttr_t stSclChnAttr;
            MI_SCL_ChnParam_t stChnParam;
            memset(&stSclChnAttr, 0x0, sizeof(MI_SCL_ChannelAttr_t));
            memset(&stSclInputCrop, 0x0, sizeof(MI_SYS_WindowRect_t));
            memset(&stChnParam, 0x0, sizeof(MI_SCL_ChnParam_t));

            for(u8PortId=0; u8PortId<ST_MAX_SCL_OUTPORT_NUM; u8PortId++)
            {
                ST_PortAttr_t *pstSclOutPortAttr = &pstSclChnAttr->stSclOutPortAttr[u8PortId];
                if(pstSclOutPortAttr->bUsed == TRUE)
                {
                    ExecFuncResult(MI_SCL_GetOutputPortParam(u8SclDevId, u8ChnId, u8PortId, &stOutputParam[u8PortId]), s32Ret);
                    ExecFuncResult(MI_SCL_DisableOutputPort(u8SclDevId, u8ChnId, u8PortId), s32Ret);
                }
            }

            ExecFuncResult(MI_SCL_GetInputPortCrop(u8SclDevId, u8ChnId, &stSclInputCrop), s32Ret);
            ExecFuncResult(MI_SCL_GetChnParam(u8SclDevId, u8ChnId, &stChnParam), s32Ret);

            ExecFuncResult(MI_SCL_StopChannel(u8SclDevId, u8ChnId), s32Ret);
            ExecFuncResult(MI_SCL_DestroyChannel(u8SclDevId, u8ChnId), s32Ret);

            usleep(THREAD_SLEEP_TIME_US*10);

            ExecFuncResult(MI_SCL_CreateChannel(u8SclDevId, u8ChnId, &stSclChnAttr), s32Ret);
            if(u8SclDevId ==1 || u8SclDevId==3)
            {
                ExecFuncResult(MI_SCL_SetInputPortCrop(u8SclDevId, u8ChnId, &stSclInputCrop), s32Ret);
            }
            ExecFuncResult(MI_SCL_SetChnParam(u8SclDevId, u8ChnId, &stChnParam), s32Ret);
            ExecFuncResult(MI_SCL_StartChannel(u8SclDevId, u8ChnId), s32Ret);

            for(u8PortId=0; u8PortId<ST_MAX_ISP_OUTPORT_NUM; u8PortId++)
            {
                ST_PortAttr_t *pstSclOutPortAttr = &pstSclChnAttr->stSclOutPortAttr[u8PortId];
                if(pstSclOutPortAttr->bUsed == TRUE)
                {
                    ExecFuncResult(MI_SCL_SetOutputPortParam(u8SclDevId, u8ChnId, u8PortId, &stOutputParam[u8PortId]), s32Ret);
                    ExecFuncResult(MI_SCL_EnableOutputPort(u8SclDevId, u8ChnId, u8PortId), s32Ret);
                }
            }
        }
    }

EXIT:
    return s32Ret;
}

MI_S32 ST_DoDynamicFunc(ST_DynamicFuncParam_t *pstDynamicFunc, MI_BOOL bdump)
{
    MI_S32 s32Ret = MI_SUCCESS;
    MI_U32 i=0;

    for(i=0; i<pstDynamicFunc->u32Cnt; i++)
    {
        switch(pstDynamicFunc->eFuncType)
        {
            case E_DYNAMIC_CHANGE_HDRRES:
                {
                    ST_DynamicChangeHdrRes_t  *pstDynamicChangeHdrRes = &pstDynamicFunc->stChangeHdrResParam;
                    DBG_INFO("ST_DoChangeHdrRes Group %d, useHdr %d, ResIdx %d \n", pstDynamicChangeHdrRes->GroupId, pstDynamicChangeHdrRes->bUseHdr, pstDynamicChangeHdrRes->u8ResIndex);
                    ExecFuncResult(ST_DoChangeHdrRes(pstDynamicChangeHdrRes->GroupId, pstDynamicChangeHdrRes->bUseHdr, pstDynamicChangeHdrRes->u8ResIndex), s32Ret);
                    usleep(1000*pstDynamicFunc->u32Sleep);
                    break;
                }
            case E_DYNAMIC_CHANGE_VIFGROUPATTR:
                {
                    ST_DynamicChangeVifGroupAttr_t  *pstDynamicChangeVifGroupAttr = &pstDynamicFunc->stChangeVifGroupAttr;
                    DBG_INFO("changevifgroupattr GroupId %d, InputPixel %d, sleep %d\n", pstDynamicChangeVifGroupAttr->GroupId,
                    pstDynamicChangeVifGroupAttr->eInputPixel, pstDynamicFunc->u32Sleep);
                    ExecFuncResult(ST_DoChangeVifGroupAttr(pstDynamicChangeVifGroupAttr->GroupId, pstDynamicChangeVifGroupAttr->eInputPixel), s32Ret);
                    usleep(1000*pstDynamicFunc->u32Sleep);
                    break;
                }
            case E_DYNAMIC_CHANGE_VIFDEVATTR:
                {
                    ST_DynamicChangeVifDevAttr_t  *pstDynamicChangeVifDevAttr = &pstDynamicFunc->stChangeVifDevAttr;
                    MI_VIF_DevAttr_t *pstDevAttr = &pstDynamicChangeVifDevAttr->stDevAttr;
                    DBG_INFO("ST_DoChangeVifDevAttr Dev %d, eInputPixel %d , stInputRect(%d,%d,%d,%d), eField %d, bEnH2T1PMode %d ,u32Sleep %d\n", pstDynamicChangeVifDevAttr->DevId,pstDevAttr->eInputPixel,
                    pstDevAttr->stInputRect.u16X,pstDevAttr->stInputRect.u16Y,pstDevAttr->stInputRect.u16Width,pstDevAttr->stInputRect.u16Height,pstDevAttr->eField,pstDevAttr->bEnH2T1PMode,pstDynamicFunc->u32Sleep);
                    ExecFuncResult(ST_DoChangeVifDevAttr(pstDynamicChangeVifDevAttr->DevId, &pstDynamicChangeVifDevAttr->stDevAttr), s32Ret);
                    usleep(1000*pstDynamicFunc->u32Sleep);
                    break;
                }
            case E_DYNAMIC_CHANGE_VIFOUTPORTATTR:
                {
                    ST_DynamicChangeVifOutPutAttr_t  *pstDynamicChangeVifOutPutAttr = &pstDynamicFunc->stChangeVifOutPutAttr;
                    MI_VIF_OutputPortAttr_t *pstOutPutAttr = &pstDynamicChangeVifOutPutAttr->stOutPutAttr;
                    DBG_INFO("ST_DoChangeVifPortAttr Dev %d,PortId %d stCapRect(%d,%d,%d,%d), stDestSize(%d,%d), ePixFormat %d ,eFrameRate %d, eCompress %d, u32Sleep %d\n", pstDynamicChangeVifOutPutAttr->DevId,pstDynamicChangeVifOutPutAttr->PortId,
                    pstOutPutAttr->stCapRect.u16X,pstOutPutAttr->stCapRect.u16Y,pstOutPutAttr->stCapRect.u16Width,pstOutPutAttr->stCapRect.u16Height,pstOutPutAttr->stDestSize.u16Width,
                    pstOutPutAttr->stDestSize.u16Height,pstOutPutAttr->ePixFormat,pstOutPutAttr->eFrameRate,pstOutPutAttr->eCompressMode,pstDynamicFunc->u32Sleep);
                    ExecFuncResult(ST_DoChangeVifPortAttr(pstDynamicChangeVifOutPutAttr->DevId, pstDynamicChangeVifOutPutAttr->PortId,&pstDynamicChangeVifOutPutAttr->stOutPutAttr), s32Ret);
                    usleep(1000*pstDynamicFunc->u32Sleep);
                    break;
                }
            case E_DYNAMIC_CHANGE_VIFOUTFRC:
                {
                    ST_DynamicChangeVifOutFRC_t  *pstDynamicChangeVifOutFRC = &pstDynamicFunc->stChangeVifOutOutFRC;
                    DBG_INFO("ST_DoChangeVifOutputFrc Dev %d,PortId %d u32FrameRate %d ,u32Sleep %d\n", pstDynamicChangeVifOutFRC->DevId,pstDynamicChangeVifOutFRC->PortId,
                    pstDynamicChangeVifOutFRC->u32FrameRate,pstDynamicFunc->u32Sleep);
                    ExecFuncResult(ST_DoChangeVifOutputFrc(pstDynamicChangeVifOutFRC->DevId, pstDynamicChangeVifOutFRC->PortId,pstDynamicChangeVifOutFRC->u32FrameRate), s32Ret);
                    usleep(1000*pstDynamicFunc->u32Sleep);
                    break;
                }
            case E_DYNAMIC_CHANGE_VIFOUTPORTONOFF:
                {
                    ST_DynamicChangeVifOutPutOnOff_t  *pstDynamicChangeVifOutPutOnOff = &pstDynamicFunc->stChangeVifOutPutOnOff;
                    DBG_INFO("ST_DoChangeVifOutPutOnOff dev %d, port %d, bOn %d,u32Sleep %d\n",pstDynamicChangeVifOutPutOnOff->DevId, pstDynamicChangeVifOutPutOnOff->PortId, pstDynamicChangeVifOutPutOnOff->bOn,pstDynamicFunc->u32Sleep);
                    ExecFuncResult(ST_DoChangeVifOutPutOnOff(pstDynamicChangeVifOutPutOnOff->DevId, pstDynamicChangeVifOutPutOnOff->PortId ,pstDynamicChangeVifOutPutOnOff->bOn), s32Ret);
                    usleep(1000*pstDynamicFunc->u32Sleep);
                    break;
                }
            case E_DYNAMIC_CHANGE_VIFDEVONOFF:
                {
                    ST_DynamicChangeVifDevOnOff_t  *pstDynamicChangeVifDevOnOff = &pstDynamicFunc->stChangeVifDevOnOff;
                    DBG_INFO("ST_DoChangeVifDevOnOff DevId %d, bOn %d, u32Sleep %d\n",pstDynamicChangeVifDevOnOff->DevId, pstDynamicChangeVifDevOnOff->bOn, pstDynamicFunc->u32Sleep);
                    ExecFuncResult(ST_DoChangeVifDevOnOff(pstDynamicChangeVifDevOnOff->DevId, pstDynamicChangeVifDevOnOff->bOn), s32Ret);
                    usleep(1000*pstDynamicFunc->u32Sleep);
                    break;
                }
            case E_DYNAMIC_CHANGE_ISPROT:
                {
                    ST_DynamicChangeIspRot_t  *pstDynamicChangeIspRot = &pstDynamicFunc->stChangeIspRot;
                    DBG_INFO("ST_DoChangeIspRotate Dev %d, chn %d, rot %d, mirror %d, flip %d \n", pstDynamicChangeIspRot->DevId, pstDynamicChangeIspRot->ChnId, pstDynamicChangeIspRot->eRot,pstDynamicChangeIspRot->bMirror,pstDynamicChangeIspRot->bFlip);
                    ExecFuncResult(ST_DoChangeIspRotate(pstDynamicChangeIspRot->DevId, pstDynamicChangeIspRot->ChnId, pstDynamicChangeIspRot->eRot,pstDynamicChangeIspRot->bMirror,pstDynamicChangeIspRot->bFlip), s32Ret);
                    usleep(1000*pstDynamicFunc->u32Sleep);
                    break;
                }
            case E_DYNAMIC_CHANGE_ISP3DNRLEVEL:
                {
                    ST_DynamicChangeIsp3dnr_t  *pstDynamicChangeIsp3dnr = &pstDynamicFunc->stChangeIsp3dnr;
                    DBG_INFO("ST_DoChangeIsp3dnr Dev %d, chn %d, 3dnrlevel %d \n", pstDynamicChangeIsp3dnr->DevId, pstDynamicChangeIsp3dnr->ChnId, pstDynamicChangeIsp3dnr->e3dnrlevel);
                    ExecFuncResult(ST_DoChangeIsp3dnr(pstDynamicChangeIsp3dnr->DevId, pstDynamicChangeIsp3dnr->ChnId, pstDynamicChangeIsp3dnr->e3dnrlevel), s32Ret);
                    usleep(1000*pstDynamicFunc->u32Sleep);
                    break;
                }
            case E_DYNAMIC_CHANGE_ISPINPUTCROP:
                {
                    ST_DynamicChangeIspInputCrop_t  *pstDynamicChangeIspInputCrop = &pstDynamicFunc->stChangeIspInputCrop;
                    DBG_INFO("ST_DoChangeIspInputCrop Dev %d, chn %d, crop (%d,%d,%d,%d) \n", pstDynamicChangeIspInputCrop->DevId, pstDynamicChangeIspInputCrop->ChnId,
                        pstDynamicChangeIspInputCrop->stCropInfo.u16X,pstDynamicChangeIspInputCrop->stCropInfo.u16Y,
                        pstDynamicChangeIspInputCrop->stCropInfo.u16Width,pstDynamicChangeIspInputCrop->stCropInfo.u16Height);
                    ExecFuncResult(ST_DoChangeIspInputCrop(pstDynamicChangeIspInputCrop->DevId, pstDynamicChangeIspInputCrop->ChnId, &pstDynamicChangeIspInputCrop->stCropInfo), s32Ret);
                    usleep(1000*pstDynamicFunc->u32Sleep);
                    break;
                }
            case E_DYNAMIC_CHANGE_ISPOUTPARAM:
                {
                    ST_DynamicChangeIspOutputParam_t  *pstDynamicChangeIspOutputParam = &pstDynamicFunc->stChangeIspOutputParam;
                    DBG_INFO("ST_DoChangeIsp3dnr Dev %d, chn %d, port %d, pixel %d, crop (%d,%d,%d,%d) compress %d\n", pstDynamicChangeIspOutputParam->DevId, pstDynamicChangeIspOutputParam->ChnId,
                        pstDynamicChangeIspOutputParam->PortId, pstDynamicChangeIspOutputParam->stOutPortParam.ePixelFormat,
                        pstDynamicChangeIspOutputParam->stOutPortParam.stCropRect.u16X, pstDynamicChangeIspOutputParam->stOutPortParam.stCropRect.u16Y,
                        pstDynamicChangeIspOutputParam->stOutPortParam.stCropRect.u16Width, pstDynamicChangeIspOutputParam->stOutPortParam.stCropRect.u16Height,
                        pstDynamicChangeIspOutputParam->stOutPortParam.eCompressMode);
                    ExecFuncResult(ST_DoChangeIspOutputParam(pstDynamicChangeIspOutputParam->DevId, pstDynamicChangeIspOutputParam->ChnId, pstDynamicChangeIspOutputParam->PortId, &pstDynamicChangeIspOutputParam->stOutPortParam), s32Ret);
                    usleep(1000*pstDynamicFunc->u32Sleep);
                    break;
                }
            case E_DYNAMIC_CHANGE_ISPZOOM:
                {
                    ST_DynamicChangeIspZoom_t  *pstDynamicChangeIspZoom = &pstDynamicFunc->stChangeIspZoom;
                    DBG_INFO("ST_DoSetIspZoom dev %d, chn %d, revrse %d, start(%d,%d,%d,%d), dest(%d,%d,%d,%d), step(%d,%d,%d,%d) \n",
                        pstDynamicChangeIspZoom->DevId,pstDynamicChangeIspZoom->ChnId,pstDynamicChangeIspZoom->bRev,
                        pstDynamicChangeIspZoom->stStart.u16X, pstDynamicChangeIspZoom->stStart.u16Y,pstDynamicChangeIspZoom->stStart.u16Width, pstDynamicChangeIspZoom->stStart.u16Height,
                        pstDynamicChangeIspZoom->stDest.u16X, pstDynamicChangeIspZoom->stDest.u16Y,pstDynamicChangeIspZoom->stDest.u16Width, pstDynamicChangeIspZoom->stDest.u16Height,
                        pstDynamicChangeIspZoom->stStepWH.u16X, pstDynamicChangeIspZoom->stStepWH.u16Y,pstDynamicChangeIspZoom->stStepWH.u16Width, pstDynamicChangeIspZoom->stStepWH.u16Height);
                    ExecFuncResult(ST_DoSetIspZoom(pstDynamicChangeIspZoom->DevId, pstDynamicChangeIspZoom->ChnId, pstDynamicChangeIspZoom->bRev, &pstDynamicChangeIspZoom->stStart, &pstDynamicChangeIspZoom->stDest, &pstDynamicChangeIspZoom->stStepWH), s32Ret);
                    usleep(1000*pstDynamicFunc->u32Sleep);
                    break;
                }
            case E_DYNAMIC_CHANGE_ISPSKIPFRAME:
                {
                    ST_DynamicChangeIspSkipFrame_t  *pstDynamicChangeIspSkipFrame = &pstDynamicFunc->stChangeIspSkipFrame;
                    DBG_INFO("ST_DoSetIspSkipFrame dev %d, chn %d, FrameNum %d \n",pstDynamicChangeIspSkipFrame->DevId,pstDynamicChangeIspSkipFrame->ChnId,pstDynamicChangeIspSkipFrame->u32FrameNum);
                    ExecFuncResult(ST_DoSetIspSkipFrame(pstDynamicChangeIspSkipFrame->DevId, pstDynamicChangeIspSkipFrame->ChnId, pstDynamicChangeIspSkipFrame->u32FrameNum), s32Ret);
                    usleep(1000*pstDynamicFunc->u32Sleep);
                    break;
                }
            case E_DYNAMIC_CHANGE_OUTPUTSTEPSIZE:
                {
                    ST_DynamicChangOutputSizeStep_t  *pstDynamicChangeOutputSizeStep = &pstDynamicFunc->stChangeOutputSizeStep;
                    DBG_INFO("ST_DoChangeOutputSizeStep module %d dev %d, chn %d, port %d, maxw %d, maxh %d, file %s \n",pstDynamicChangeOutputSizeStep->stChnPort.eModId,
                        pstDynamicChangeOutputSizeStep->stChnPort.u32DevId, pstDynamicChangeOutputSizeStep->stChnPort.u32ChnId,pstDynamicChangeOutputSizeStep->stChnPort.u32PortId,
                        pstDynamicChangeOutputSizeStep->stMaxWin.u16Width, pstDynamicChangeOutputSizeStep->stMaxWin.u16Height, pstDynamicChangeOutputSizeStep->FilePath);
                    ExecFuncResult(ST_DoChangeOutputSizeStep(&pstDynamicChangeOutputSizeStep->stChnPort, &pstDynamicChangeOutputSizeStep->stMaxWin, pstDynamicChangeOutputSizeStep->FilePath), s32Ret);
                    usleep(1000*pstDynamicFunc->u32Sleep);
                    break;
                }
            case E_DYNAMIC_CHANGE_CROPSTEPSIZE:
                {
                    ST_DynamicChangeCropStep_t  *pstDynamicChangeCropStep = &pstDynamicFunc->stChangeCropStep;
                    DBG_INFO("ST_DoChangeCropSizeStepTest position %d,(%d,%d,%d,%d), maxw %d, maxh %d, filepath %s\n", pstDynamicChangeCropStep->eCropPosition,
                        pstDynamicChangeCropStep->stChnPort.eModId, pstDynamicChangeCropStep->stChnPort.u32DevId, pstDynamicChangeCropStep->stChnPort.u32ChnId,
                        pstDynamicChangeCropStep->stChnPort.u32PortId, pstDynamicChangeCropStep->stMaxWin.u16Width, pstDynamicChangeCropStep->stMaxWin.u16Height, pstDynamicChangeCropStep->FilePath);
                    ExecFuncResult(ST_DoChangeCropSizeStepTest(pstDynamicChangeCropStep->eCropPosition, &pstDynamicChangeCropStep->stChnPort, &pstDynamicChangeCropStep->stMaxWin, pstDynamicChangeCropStep->FilePath), s32Ret);
                    usleep(1000*pstDynamicFunc->u32Sleep);
                    break;
                }
            case E_DYNAMIC_CHANGE_SCLSTRETCHBUFF:
                {
                    ST_DynamicChangeStretchBuff_t  *pstDynamicChangeStretchBuff = &pstDynamicFunc->stChangeStretchBuff;
                    DBG_INFO("stretch buffer file %s,pixel%d,insize(%dx%d),instride %d,crop(%d,%d,%d,%d),outsize(%dx%d),outpixel%d, outputpath %s\n",
                        pstDynamicChangeStretchBuff->sFileInputPath,pstDynamicChangeStretchBuff->eInputPixel,
                        pstDynamicChangeStretchBuff->stInputWinSize.u16Width,pstDynamicChangeStretchBuff->stInputWinSize.u16Height,pstDynamicChangeStretchBuff->u32InputStride,
                        pstDynamicChangeStretchBuff->stCropWin.u16X,pstDynamicChangeStretchBuff->stCropWin.u16Y,pstDynamicChangeStretchBuff->stCropWin.u16Width,pstDynamicChangeStretchBuff->stCropWin.u16Height,
                        pstDynamicChangeStretchBuff->stOutputWinSize.u16Width,pstDynamicChangeStretchBuff->stOutputWinSize.u16Width,pstDynamicChangeStretchBuff->eOutputPixel,pstDynamicChangeStretchBuff->sFileOutputPath);
                    ExecFuncResult(ST_DoChangeStretchBuff(pstDynamicChangeStretchBuff->sFileInputPath, pstDynamicChangeStretchBuff->eInputPixel, pstDynamicChangeStretchBuff->stInputWinSize, pstDynamicChangeStretchBuff->u32InputStride,
                        pstDynamicChangeStretchBuff->stCropWin, pstDynamicChangeStretchBuff->stOutputWinSize, pstDynamicChangeStretchBuff->eOutputPixel,pstDynamicChangeStretchBuff->sFileOutputPath), s32Ret);

                    usleep(1000*pstDynamicFunc->u32Sleep);
                    break;
                }
            case E_DYNAMIC_CHANGE_SCLOUTPUTPARAM:
                {
                    ST_DynamicChangeSclOutputParam_t  *pstSclOutParam = &pstDynamicFunc->stChangeSclOutputParam;
                    DBG_INFO("changescloutparam devid%d chnid%d portid%d mirror%d flip%d,crop(%d,%d,%d,%d),outsize(%dx%d),outpixel%d,compress %d\n",
                        pstSclOutParam->DevId, pstSclOutParam->ChnId, pstSclOutParam->PortId, pstSclOutParam->stOutputParam.bMirror,
                        pstSclOutParam->stOutputParam.bFlip, pstSclOutParam->stOutputParam.stSCLOutCropRect.u16X,pstSclOutParam->stOutputParam.stSCLOutCropRect.u16Y,
                        pstSclOutParam->stOutputParam.stSCLOutCropRect.u16Width,pstSclOutParam->stOutputParam.stSCLOutCropRect.u16Height,
                        pstSclOutParam->stOutputParam.stSCLOutputSize.u16Width, pstSclOutParam->stOutputParam.stSCLOutputSize.u16Height,
                        pstSclOutParam->stOutputParam.ePixelFormat,pstSclOutParam->stOutputParam.eCompressMode);

                    ExecFuncResult(ST_DoChangeSclOutPutParam(pstSclOutParam->DevId, pstSclOutParam->ChnId, pstSclOutParam->PortId, &pstSclOutParam->stOutputParam), s32Ret);
                    usleep(1000*pstDynamicFunc->u32Sleep);
                    break;
                }
            case E_DYNAMIC_CHANGE_HWRESET:
                {
                    ST_DynamicChangeHwReset_t  *pstHwReset = &pstDynamicFunc->stChangeHwReset;
                    DBG_INFO("HwReset ModId%d devid%d chnid%d pos%d\n",
                        pstHwReset->eModId, pstHwReset->DevId, pstHwReset->ChnId, pstHwReset->u8Pos);

                    ExecFuncResult(ST_DoHwReset(pstHwReset->eModId, pstHwReset->DevId, pstHwReset->ChnId, pstHwReset->u8Pos), s32Ret);
                    usleep(1000*pstDynamicFunc->u32Sleep);
                    break;
                }
            case E_DYNAMIC_CHANGE_SCLROTATE:
                {
                    ST_DynamicChangeSclRotate_t  *pstSclRot = &pstDynamicFunc->stChangeSclRotate;
                    DBG_INFO("SclRotate devid%d chnid%d rot%d sleep%d \n",pstSclRot->DevId, pstSclRot->ChnId,pstSclRot->eRot,pstDynamicFunc->u32Sleep);

                    ExecFuncResult(ST_DoChangeSclRotate(pstSclRot->DevId,pstSclRot->ChnId,pstSclRot->eRot), s32Ret);
                    usleep(1000*pstDynamicFunc->u32Sleep);
                    break;
                }
            case E_DYNAMIC_CHANGE_SCLINPUTCROP:
                {
                    ST_DynamicChangeSclInputCrop_t *pstChangeSclInputCrop = &pstDynamicFunc->stChangeSclInputCrop;
                    DBG_INFO("SclInputCrop devid%d chnid%d \n",pstChangeSclInputCrop->DevId,pstChangeSclInputCrop->ChnId);

                    ExecFuncResult(ST_DoChangeSclInputCrop(pstChangeSclInputCrop->DevId,pstChangeSclInputCrop->ChnId,&pstChangeSclInputCrop->stCropInfo), s32Ret);
                    usleep(1000*pstDynamicFunc->u32Sleep);
                    break;
                }
            case E_DYNAMIC_DESTROYCREATE_CHANNEL:
                {
                    ST_DynamicDestroyCreateChannel_t *pstDestroyCreateChannel = &pstDynamicFunc->stDestroyCreateChannel;
                    DBG_INFO("DestroyCreateChannel module %d devid%d \n",pstDestroyCreateChannel->eModId,pstDestroyCreateChannel->DevId);

                    if(pstDestroyCreateChannel->eModId == E_MI_MODULE_ID_ISP)
                    {
                        ExecFuncResult(ST_DoDestroyCreateIspChannel(pstDestroyCreateChannel->DevId), s32Ret);
                    }
                    else if(pstDestroyCreateChannel->eModId == E_MI_MODULE_ID_SCL)
                    {
                        ExecFuncResult(ST_DoDestroyCreateSclChannel(pstDestroyCreateChannel->DevId), s32Ret);
                    }
                    usleep(1000*pstDynamicFunc->u32Sleep);
                    break;
                }
            case E_DYNAMIC_CHANGE_CRCONOFF:
                {
                    ST_DynamicChangeCRCOnoff_t  *pstCRCOnoff = &pstDynamicFunc->stChangeCRCOnoff;
                    DBG_INFO("CRCOnoff moduleid%d devid%d chnid%d portid%d bOn %d\n",
                        pstCRCOnoff->eModuleId, pstCRCOnoff->u32DevId, pstCRCOnoff->u32ChnId, pstCRCOnoff->u32PortId, pstCRCOnoff->bOn);

                    ExecFuncResult(ST_DoCrcOnOff(pstCRCOnoff->eModuleId, pstCRCOnoff->u32DevId, pstCRCOnoff->u32ChnId, pstCRCOnoff->u32PortId, pstCRCOnoff->bOn), s32Ret);
                    usleep(1000*pstDynamicFunc->u32Sleep);
                    break;
                }
            case E_DYNAMIC_CHANGE_CHNPORTONOFF:
                {
                    ST_DynamicChangeChnPortOnoff_t  *pstChnPortOnoff = &pstDynamicFunc->stChangeChnPortOnOff;
                    DBG_INFO("ChnPortOnoff moduleid%d devid%d chnid%d portid%d position%d bOn %d\n",
                        pstChnPortOnoff->stChnPort.eModId, pstChnPortOnoff->stChnPort.u32DevId, pstChnPortOnoff->stChnPort.u32ChnId, pstChnPortOnoff->stChnPort.u32PortId,
                        pstChnPortOnoff->ePosition,pstChnPortOnoff->bStatus);

                    ExecFuncResult(ST_DoChangeChnPortOnOff(&pstChnPortOnoff->stChnPort, pstChnPortOnoff->ePosition, pstChnPortOnoff->bStatus), s32Ret);
                    usleep(1000*pstDynamicFunc->u32Sleep);
                    break;
                }
            default:
                DBG_ERR("Func type %d, not found \n", pstDynamicFunc->eFuncType);
                UTStatus = UT_CASE_FAIL;
                break;
        }

        if((!list_empty(&gstDynamicTest.stDumpFileListHead)) && (bdump == TRUE))
        {
            struct list_head *pos = NULL;
            struct list_head *n = NULL;
            list_for_each_safe(pos, n, &gstDynamicTest.stDumpFileListHead)
            {
                ST_DumpFileInfo_t *pstDumpFile = NULL;
                pstDumpFile = container_of(pos, typeof(*pstDumpFile), stDumpFileListNode);

                if(pstDumpFile != NULL)
                {
                    //ST_GetModuleOutputData(&pstDumpFile->stChnPort, pstDumpFile->FilePath, pstDumpFile->u16FileCnt);
                    ST_GetModuleOutputDataOneFile(pstDumpFile,pstDynamicFunc->u16Id);
                    //usleep(1000*1000*1); select no need sleep
                }
                DBG_INFO("[%s]:%d \n", __FUNCTION__,__LINE__);
            }
        }
    }

EXIT:
    return s32Ret;
}

#if USER_SENSOR_SUPPORT
MI_U32 getSensorSize(DHC_DH9931_VIDEO_FMT_E eVideoFmt, MI_U32 *u32W, MI_U32 *u32H)
{
    switch (eVideoFmt)
    {
        case DHC_SD_NTSC_JM:
            *u32W = 960;
            *u32H = 480;
            break;
        case DHC_SD_PAL_BGHID:
            *u32W = 960;
            *u32H = 576;
            break;
        case DHC_CVI_1280x720_25HZ:
        case DHC_CVI_1280x720_30HZ:
        case DHC_CVI_1280x720_50HZ:
        case DHC_CVI_1280x720_60HZ:
        case DHC_AHD_1280x720_25HZ:
        case DHC_AHD_1280x720_30HZ:
        case DHC_AHD_1280x720_50HZ:
        case DHC_AHD_1280x720_60HZ:
        case DHC_TVI_1280x720_25HZ:
        case DHC_TVI_1280x720_30HZ:
        case DHC_TVI_1280x720_50HZ:
        case DHC_TVI_1280x720_60HZ:
        case DHC_TVI3_1280x720_25HZ:
        case DHC_TVI3_1280x720_30HZ:
            *u32W = 1280;
            *u32H = 720;
            break;
        case DHC_CVI_1920x1080_25HZ:
        case DHC_CVI_1920x1080_30HZ:
        case DHC_CVI_1920x1080_50HZ:
        case DHC_CVI_1920x1080_60HZ:
        case DHC_AHD_1920x1080_25HZ:
        case DHC_AHD_1920x1080_30HZ:
        case DHC_TVI_1920x1080_25HZ:
        case DHC_TVI_1920x1080_30HZ:
            *u32W = 1920;
            *u32H = 1080;
            break;
        case DHC_CVI_2560x1440_12HZ:
        case DHC_CVI_2560x1440_15HZ:
        case DHC_CVI_2560x1440_25HZ:
        case DHC_CVI_2560x1440_30HZ:
        case DHC_AHD_2560x1440_15HZ:
        case DHC_AHD_2560x1440_25HZ:
        case DHC_AHD_2560x1440_30HZ:
        case DHC_TVI_2560x1440_25HZ:
        case DHC_TVI_2560x1440_30HZ:
            *u32W = 2560;
            *u32H = 1440;
            break;
        case DHC_AHD_2592x1944_12HZ:
        case DHC_AHD_2592x1944_20HZ:
        case DHC_CVI_2592x1944_20HZ:
            *u32W = 2592;
            *u32H = 1944;
            break;
        case DHC_TVI_2560x1944_12HZ:
        case DHC_TVI_2560x1944_20HZ:
            *u32W = 2560;
            *u32H = 1944;
            break;
        case DHC_CVI_2560x1920_20HZ:
            *u32W = 2560;
            *u32H = 1920;
            break;
        case DHC_CVI_2304x1296_25HZ:
            *u32W = 2304;
            *u32H = 1296;
            break;
        case DHC_CVI_2048x1152_30HZ:
            *u32W = 2048;
            *u32H = 1152;
            break;
        case DHC_CVI_2048x1536_25HZ:
        case DHC_CVI_2048x1536_30HZ:
        case DHC_AHD_2048x1536_18HZ:
        case DHC_AHD_2048x1536_25HZ:
        case DHC_AHD_2048x1536_30HZ:
            *u32W = 2048;
            *u32H = 1536;
            break;
        case DHC_CVI_2880x1920_20HZ:
            *u32W = 2880;
            *u32H = 1920;
            break;
        case DHC_CVI_3072x1728_20HZ:
        case DHC_CVI_3072x1728_25HZ:
            *u32W = 3072;
            *u32H = 1728;
            break;
        case DHC_CVI_3840x2160_12HZ:
        case DHC_CVI_3840x2160_15HZ:
            *u32W = 3840;
            *u32H = 2160;
            break;
       case DHC_TVI_1920x1536_18HZ:
           *u32W = 1920;
           *u32H = 1536;
           break;
        default:
            printf("Unsupported format[%d]!\n", eVideoFmt);
            return -1;
    }

    if(bDetectLog == TRUE)
    {
        printf("eVideoFmt[%d], wxh[%dx%d]!\n", eVideoFmt, *u32W, *u32H);
    }

    return 0;
}

MI_S32 ST_ResetVif(MI_U32 u32DevId, MI_U32 u32SensorW, MI_U32 u32SensorH)
{
    MI_S32 s32Ret = MI_SUCCESS;
    MI_VIF_PORT vifPort = 0;
    MI_VIF_GROUP  GroupId = u32DevId/ST_MAX_VIF_DEV_PERGROUP;
    MI_VIF_DEV  DevIdPerGroup = u32DevId%ST_MAX_VIF_DEV_PERGROUP;

    pthread_mutex_lock(&gstVifModule.stVifGroupAttr[GroupId].stVifDevAttr[DevIdPerGroup].Devmutex);

    for(vifPort=0; vifPort< ST_MAX_VIF_OUTPORT_NUM; vifPort++)
    {
        ST_VifPortAttr_t *pstVifPortAttr = &gstVifModule.stVifGroupAttr[GroupId].stVifDevAttr[DevIdPerGroup].stVifOutPortAttr[vifPort];
        if(pstVifPortAttr->bUsed == TRUE
            && pstVifPortAttr->bCreate== TRUE
            )
        {
            ExecFuncResult(MI_VIF_DisableOutputPort(u32DevId, vifPort), s32Ret);
            pstVifPortAttr->bCreate = FALSE;
        }
    }
/*
    MI_S32 s32ScanfTemp = 0;
    printf("stop here \n");
    scanf("%d", &s32ScanfTemp);
    ST_Flush();
*/
    ExecFuncResult(MI_VIF_DisableDev(u32DevId), s32Ret);

    if(gstVifModule.stVifGroupAttr[GroupId].stVifDevAttr[DevIdPerGroup].bUsed == TRUE)
    {
        MI_VIF_DevAttr_t stVifDevAttr;
        memset(&stVifDevAttr, 0x0, sizeof(MI_VIF_DevAttr_t));
        ExecFuncResult(MI_VIF_GetDevAttr(u32DevId, &stVifDevAttr), s32Ret);

        stVifDevAttr.stInputRect.u16X = 0;
        stVifDevAttr.stInputRect.u16Y = 0;
        stVifDevAttr.stInputRect.u16Width = u32SensorW;
        stVifDevAttr.stInputRect.u16Height = u32SensorH;
        if((u32SensorW == 960 && u32SensorH == 480)||(u32SensorW == 960 && u32SensorH == 576))
        {
            stVifDevAttr.eField = E_MI_SYS_FIELDTYPE_BOTH;
        }
        else
        {
            stVifDevAttr.eField = E_MI_SYS_FIELDTYPE_NONE;
        }
        printf("setchnportattr (%d,%d,%d,%d) eField (%d)\n", stVifDevAttr.stInputRect.u16X, stVifDevAttr.stInputRect.u16Y, stVifDevAttr.stInputRect.u16Width, stVifDevAttr.stInputRect.u16Height,stVifDevAttr.eField);

        ExecFuncResult(MI_VIF_SetDevAttr(u32DevId, &stVifDevAttr), s32Ret);
        ExecFuncResult(MI_VIF_EnableDev(u32DevId), s32Ret);

        memcpy(&gstVifModule.stVifGroupAttr[GroupId].stVifDevAttr[DevIdPerGroup].stVifDevAttr, &stVifDevAttr, sizeof(MI_VIF_DevAttr_t));
        if(stVifDevAttr.bEnH2T1PMode == TRUE)
            u32SensorW =u32SensorW/2;
    }

    for(vifPort=0; vifPort< ST_MAX_VIF_OUTPORT_NUM; vifPort++)
    {
        ST_VifPortAttr_t *pstVifPortAttr = &gstVifModule.stVifGroupAttr[GroupId].stVifDevAttr[DevIdPerGroup].stVifOutPortAttr[vifPort];
        if(pstVifPortAttr->bUsed == TRUE
            && pstVifPortAttr->bCreate== FALSE)
        {
            MI_VIF_OutputPortAttr_t stVifPortInfo;
            memset(&stVifPortInfo, 0, sizeof(MI_VIF_OutputPortAttr_t));

            stVifPortInfo.stCapRect.u16X = 0;
            stVifPortInfo.stCapRect.u16Y = 0;
            stVifPortInfo.stCapRect.u16Width =  u32SensorW;
            stVifPortInfo.stCapRect.u16Height = u32SensorH;
            stVifPortInfo.stDestSize.u16Width = u32SensorW;
            stVifPortInfo.stDestSize.u16Height = u32SensorH;
            stVifPortInfo.ePixFormat = pstVifPortAttr->ePixFormat;
            stVifPortInfo.eCompressMode = pstVifPortAttr->eCompressMode;
            //stVifPortInfo.u32FrameModeLineCount for lowlantancy mode

            //if(stDevAttr.eIntfMode == E_MI_VIF_MODE_BT656)
            {
                stVifPortInfo.eFrameRate = E_MI_VIF_FRAMERATE_FULL;
            }
            ExecFuncResult(MI_VIF_SetOutputPortAttr(u32DevId, vifPort, &stVifPortInfo), s32Ret);
            ExecFuncResult(MI_VIF_EnableOutputPort(u32DevId, vifPort), s32Ret);

            pstVifPortAttr->bCreate = TRUE;
        }
    }

EXIT:
    pthread_mutex_unlock(&gstVifModule.stVifGroupAttr[GroupId].stVifDevAttr[DevIdPerGroup].Devmutex);
    return s32Ret;
}

MI_S32 ST_SetDH9931HalfMode(MI_U8 u8ChipIndex, MI_U8 u8Chn, MI_U32 *pu32SensorW)
{
    CUS_BT656_MULTIPLEX_e eMultiplex = CUS_BT656_WORK_MODE_MAX;

    Cus_GetCurMultiplex(u8ChipIndex, &eMultiplex);
    if(eMultiplex == CUS_BT656_WORK_MODE_MAX)
    {
        printf("dh9931 work mode %d wrong, not support \n", eMultiplex);
        return -1;
    }
    else if(eMultiplex == CUS_BT656_WORK_MODE_4MULTIPLEX)
    {
        if(*pu32SensorW == 1920)
        {
            Cus_SetHalfMode(u8ChipIndex, u8Chn, DHC_TRUE, DHC_HALF_MODE_74_25);
            *pu32SensorW = (*pu32SensorW)/2;
        }
        else if(*pu32SensorW == 2560)
        {
            Cus_SetHalfMode(u8ChipIndex, u8Chn, DHC_TRUE, DHC_HALF_MODE_148_5);
            *pu32SensorW = (*pu32SensorW)/2;
        }
        else
        {
            Cus_SetHalfMode(u8ChipIndex, u8Chn, DHC_FALSE, DHC_HALF_MODE_148_5);
        }
    }
    else if(eMultiplex == CUS_BT656_WORK_MODE_2MULTIPLEX)
    {
        if(*pu32SensorW == 2560)
        {
            Cus_SetHalfMode(u8ChipIndex, u8Chn, DHC_TRUE, DHC_HALF_MODE_148_5);
            *pu32SensorW = (*pu32SensorW)/2;
        }
        else
        {
            Cus_SetHalfMode(u8ChipIndex, u8Chn, DHC_FALSE, DHC_HALF_MODE_148_5);
        }
    }
    else
    {
        Cus_SetHalfMode(u8ChipIndex, u8Chn, DHC_FALSE, DHC_HALF_MODE_148_5);
    }

    return 0;
}

MI_S32 ST_DH9931RegSetting(MI_U8 u8ChipIndex, MI_U8 u8Chn, DHC_DH9931_VIDEO_FMT_E enVideoReportFormat)
{
    MI_U16 u16RegValue = 0;
    CUS_BT656_MULTIPLEX_e eMultiplex = CUS_BT656_WORK_MODE_MAX;

    Cus_GetCurMultiplex(u8ChipIndex, &eMultiplex);
    if(eMultiplex == CUS_BT656_WORK_MODE_MAX || u8Chn > 3)
    {
        printf("Failed, dh9931 chn:%d, work mode %d, not support \n", u8Chn, eMultiplex);
        return -1;
    }

    //fix 720p25 not output;
    //cannot do it in reset vif, beacuse There is no change in the detected state before and after fast plugging in and out of the power supply
    if(enVideoReportFormat == DHC_CVI_1280x720_25HZ
        || enVideoReportFormat == DHC_AHD_1280x720_25HZ
        || enVideoReportFormat == DHC_TVI_1280x720_25HZ
        )
    {
        Cus_SetDH9931Data(u8ChipIndex, 0x0506|(u8Chn<<12), 0x70);//small than 0x70 is ok
        //DBG_INFO("Dev %d 720p25 set encodedelay \n", u32VifDev);
    }

    //2multi/4multi, each channel clock should be the same. SD need use HD clk.
    if(eMultiplex != CUS_BT656_WORK_MODE_1MULTIPLEX)
    {
        if(enVideoReportFormat == DHC_SD_NTSC_JM
            || enVideoReportFormat == DHC_SD_PAL_BGHID)
        {
            //0x0507[5]: clk edge, 0 rising, 1 dual
            //0x0507[6]: set 1, SD res will use HD clk, and 0x050e[2:4] will be useful.
            //0x050e[2:4]: 0:74.25m 1:148.5m 2:37.125m 3:297m 4:144m 5:288m, only use by SD.

            Cus_SetDH9931Data(u8ChipIndex, 0x0507|(u8Chn<<12), 0x74);

            u16RegValue = (eMultiplex == CUS_BT656_WORK_MODE_2MULTIPLEX) ? (0x04|u8Chn) : (0x00|u8Chn);
            Cus_SetDH9931Data(u8ChipIndex, 0x050e|(u8Chn<<12), u16RegValue);
        }
        else
        {
            //restore clk setting for other resolution after switching from SD resolution
            //0x0507[3]: set half clk, such as 148.5m -> 74.25m
            //HD res is 148.5m when 2multi, 4multi need use 74.25m, 0x0507[3] should be 1.

            u16RegValue = (eMultiplex == CUS_BT656_WORK_MODE_2MULTIPLEX) ? 0x34 : 0x3C;
            Cus_SetDH9931Data(u8ChipIndex, 0x0507|(u8Chn<<12), u16RegValue);
        }
    }

    return 0;
}

MI_S32 ST_CheckVifNeedReset(MI_U8 u8DH9931Id, MI_U8 u8Chn, MI_U32 u32SensorWidth, MI_U32 u32SensorHeight)
{
    MI_U32 u32VifDev = u8Chn;
    MI_U32 u32VifGroup=0;

    for(u32VifGroup = 0; u32VifGroup < ST_MAX_VIF_GROUP_NUM; u32VifGroup++)
    {
        MI_U8 u8TmpChipIndex=0;
        MI_U32 u32VifModuleDevId = 0;
        MI_VIF_DevAttr_t stVifDevAttr;

        if(gstVifModule.stVifGroupAttr[u32VifGroup].bCreate == FALSE)
            continue;

        ST_TransSnrPadIdToDH9931Id(gstVifModule.stVifGroupAttr[u32VifGroup].stBindSensor.eSensorPadID, &u8TmpChipIndex);

        if(u8TmpChipIndex != u8DH9931Id)
            continue;

        if(gstVifModule.stVifGroupAttr[u32VifGroup].stVifDevAttr[u32VifDev].bUsed != TRUE)
            continue;

        u32VifModuleDevId = u32VifGroup*ST_MAX_VIF_DEV_PERGROUP+u32VifDev;
        memset(&stVifDevAttr, 0x0, sizeof(MI_VIF_DevAttr_t));
        MI_VIF_GetDevAttr(u32VifModuleDevId, &stVifDevAttr);

        //ST_ResetVif not support change multi mode.If 4multi mode change to 2multi mode,different config files must be load.
        if((0 != u32SensorWidth && 0 != u32SensorHeight)
            && (u32SensorWidth != stVifDevAttr.stInputRect.u16Width
                || u32SensorHeight!= stVifDevAttr.stInputRect.u16Height))
        {
#if ((defined CONFIG_SIGMASTAR_CHIP_M6) && (CONFIG_SIGMASTAR_CHIP_M6 == 1))
            Cus_SetDH9931Data(u8TmpChipIndex, 0x1802, 0x10);
#endif
            printf("dev[%d]: resolution changed from[%dx%d] to [%dx%d]\n", u32VifModuleDevId, stVifDevAttr.stInputRect.u16Width,
            stVifDevAttr.stInputRect.u16Height, u32SensorWidth, u32SensorHeight);

            if(MI_SUCCESS != ST_ResetVif(u32VifModuleDevId, u32SensorWidth, u32SensorHeight))
            {
                UTStatus = UT_CASE_FAIL;
                DBG_ERR("reset vif:%d failed !!! \n",u32VifModuleDevId);
            }
        }
    }

    return 0;
}

void *ST_Check9931Status(void *args)
{
#define ST_DH9931_NUM 2
    DHC_DH9931_DETECT_ATTR stDetectAttr;
    DHC_DH9931_VIDEO_STATUS_S stVideoStatus;
    DHC_U8 u8Chn = 0, u8ChipIndex = 0;
    MI_U32 u32SensorWidth = 0;
    MI_U32 u32SensorHeight = 0;
    MI_BOOL b9931chipUsed[ST_DH9931_NUM]={0};
    MI_SNR_PADID  snrPadId=0;

    for (snrPadId = 0 ; snrPadId < ST_MAX_SENSOR_NUM; snrPadId++)
    {
        if(gstSensorAttr[snrPadId].bCreate == TRUE)
        {
            ST_TransSnrPadIdToDH9931Id(snrPadId, &u8ChipIndex);
            if(u8ChipIndex < ST_DH9931_NUM)
            {
                b9931chipUsed[u8ChipIndex]=TRUE;
            }
        }
    }

    while(!bExit)
    {
        stDetectAttr.bEqAdd = DHC_TRUE;
        stDetectAttr.bEqAuto = DHC_TRUE;
        DHC_DH9931_SDK_DetectThr(&stDetectAttr);

        sleep(1);
        if(bDetectAD == FALSE)
        {
            continue;
        }

        for(u8ChipIndex = 0 ; u8ChipIndex < ST_DH9931_NUM; u8ChipIndex++)
        {
            CUS_BT656_MULTIPLEX_e eMultiplex = CUS_BT656_WORK_MODE_MAX;

            if(b9931chipUsed[u8ChipIndex]!=TRUE)
                continue;

            Cus_GetCurMultiplex(u8ChipIndex, &eMultiplex);

            for(u8Chn = 0 ; u8Chn < 4; u8Chn++)
            {
                Cus_GetVideoStatus(u8ChipIndex, u8Chn, &stVideoStatus);
                if(bDetectLog == TRUE)
                {
                    printf("ad[%d],chn[%d]: isLost[%d], VideoFormat[%d], ReportFormat[%d] \n\n",u8ChipIndex,u8Chn,stVideoStatus.enVideoLost,stVideoStatus.enVideoFormat,stVideoStatus.enVideoReportFormat);
                }

                if(eMultiplex == CUS_BT656_WORK_MODE_1MULTIPLEX)
                {
                    if(u8Chn!=3 && ((1 == stVideoStatus.enVideoLost)||(stVideoStatus.enVideoReportFormat == DHC_INVALID_FMT)))
                        continue;
                }

                if((1 == stVideoStatus.enVideoLost)||(stVideoStatus.enVideoReportFormat == DHC_INVALID_FMT))
                {
                    //Default set 720p when invalid format detected.
                    u32SensorWidth = 1280;
                    u32SensorHeight = 720;
                }
                else
                {
                    getSensorSize(stVideoStatus.enVideoReportFormat, &u32SensorWidth, &u32SensorHeight);
                }

                ST_SetDH9931HalfMode(u8ChipIndex, u8Chn, &u32SensorWidth);

                ST_DH9931RegSetting(u8ChipIndex, u8Chn, stVideoStatus.enVideoReportFormat);

                if(eMultiplex == CUS_BT656_WORK_MODE_1MULTIPLEX)
                {
                    ST_CheckVifNeedReset(u8ChipIndex, 0, u32SensorWidth, u32SensorHeight);
                    break;
                }
                else
                {
                    ST_CheckVifNeedReset(u8ChipIndex, u8Chn, u32SensorWidth, u32SensorHeight);
                }

            }
        }
        //sleep(2);
    }
    return NULL;
}
#endif

int ST_InitParam()
{
    gstSensorAttr[0].u32BindVifDev = 0;
    gstSensorAttr[1].u32BindVifDev = 2;
    gstSensorAttr[2].u32BindVifDev = 1;

    return 0;
}

void ST_WaitDumpFileDone()
{
    MI_U32 u32VifGroup = 0;
    MI_U8 vifDevIdPerGroup = 0;
    MI_VIF_PORT vifPort = 0;
    ST_VifDevAttr_t *pstVifDevAttr = NULL;
    ST_VifPortAttr_t *pstVifPortAttr=NULL;

    MI_ISP_DEV IspDevId =0;
    MI_ISP_CHANNEL IspChnId = 0;
    MI_ISP_PORT  IspOutPortId =0;
    ST_IspDevAttr_t *pstIspDevAttr= NULL;
    ST_IspChannelAttr_t  *pstIspChnAttr=NULL;
    ST_PortAttr_t *pstIspOutputAttr=NULL;

    MI_U32  u32SclDevId =0;
    MI_U32  u32SclChnId =0;
    MI_U32  u32SclPortId =0;
    ST_SclChannelAttr_t *pstSclChnAttr = NULL;
    ST_SclDevAttr_t *pstSclDevAttr = NULL;
    ST_PortAttr_t *pstSclPortAttr = NULL;


    //vif
    for(u32VifGroup=0; u32VifGroup<ST_MAX_VIF_GROUP_NUM; u32VifGroup++)
    {
        for(vifDevIdPerGroup=0; vifDevIdPerGroup< ST_MAX_VIF_DEV_PERGROUP; vifDevIdPerGroup++)
        {

            pstVifDevAttr = &gstVifModule.stVifGroupAttr[u32VifGroup].stVifDevAttr[vifDevIdPerGroup];
            if(pstVifDevAttr->bUsed == FALSE)
            {
                continue;
            }
            for(vifPort=0; vifPort< ST_MAX_VIF_OUTPORT_NUM; vifPort++)
            {
                pstVifPortAttr = &pstVifDevAttr->stVifOutPortAttr[vifPort];
                if(pstVifPortAttr->bUsed == FALSE)
                {
                    continue;
                }

                if(pstVifPortAttr->stoutFileAttr.s32DumpBuffNum < 0)
                {
                    pstVifPortAttr->stoutFileAttr.s32DumpBuffNum = -pstVifPortAttr->stoutFileAttr.s32DumpBuffNum;
                    DBG_INFO("vif devid %d port %d, dump %d\n", vifDevIdPerGroup+u32VifGroup*ST_MAX_VIF_DEV_PERGROUP, vifPort, pstVifPortAttr->stoutFileAttr.s32DumpBuffNum);
                }

                while(pstVifPortAttr->stoutFileAttr.s32DumpBuffNum > 0 || pstVifPortAttr->stoutFileAttr.fp != NULL)
                {
                    usleep(5000);
                }
                DBG_INFO("vif module: group %d devid %d port %d, wait dump finish!\n",u32VifGroup,vifDevIdPerGroup,vifPort);
            }
        }
    }

    //isp
    for(IspDevId=0; IspDevId<ST_MAX_ISP_DEV_NUM; IspDevId++)
    {
        pstIspDevAttr= &gstIspModule.stIspDevAttr[IspDevId];
        if(pstIspDevAttr->bUsed == FALSE)
        {
            continue;
        }

        for(IspChnId=0; IspChnId<ST_MAX_ISP_CHN_NUM; IspChnId++)
        {
            pstIspChnAttr = &pstIspDevAttr->stIspChnlAttr[IspChnId];
            if(pstIspChnAttr->bUsed == FALSE)
            {
                continue;
            }

            for(IspOutPortId=0; IspOutPortId<ST_MAX_ISP_OUTPORT_NUM; IspOutPortId++)
            {
                pstIspOutputAttr = &pstIspChnAttr->stIspOutPortAttr[IspOutPortId];
                if(pstIspOutputAttr->bUsed == FALSE)
                {
                    continue;
                }

                if(pstIspOutputAttr->stoutFileAttr.s32DumpBuffNum < 0)
                {
                    pstIspOutputAttr->stoutFileAttr.s32DumpBuffNum = -pstIspOutputAttr->stoutFileAttr.s32DumpBuffNum;
                    DBG_INFO("isp devid %d chn %d port %d, dump %d\n", IspDevId, IspChnId, IspOutPortId, pstIspOutputAttr->stoutFileAttr.s32DumpBuffNum);
                }

                while(pstIspOutputAttr->stoutFileAttr.s32DumpBuffNum > 0 || pstIspOutputAttr->stoutFileAttr.fp != NULL)
                {
                    usleep(5000);
                }
                DBG_INFO("isp module: devid %d chnid %d port %d, wait dump finish!\n",IspDevId,IspChnId,IspOutPortId);
            }
        }
    }

    //scl
    for(u32SclDevId=0; u32SclDevId<ST_MAX_SCL_DEV_NUM; u32SclDevId++)
    {
        pstSclDevAttr = &gstSclModule.stSclDevAttr[u32SclDevId];
        if(pstSclDevAttr->bUsed == FALSE)
        {
            continue;
        }

        for(u32SclChnId =0; u32SclChnId < ST_MAX_SCL_CHN_NUM; u32SclChnId++)
        {
            pstSclChnAttr = &pstSclDevAttr->stSclChnlAttr[u32SclChnId];
            if(pstSclChnAttr->bUsed == FALSE)
            {
                continue;
            }

            for(u32SclPortId=0;u32SclPortId<ST_MAX_SCL_OUTPORT_NUM;u32SclPortId++)
            {

                pstSclPortAttr = &pstSclChnAttr->stSclOutPortAttr[u32SclPortId];
                if( pstSclPortAttr->bUsed == FALSE)
                {
                    continue;
                }

                if(pstSclPortAttr->stoutFileAttr.s32DumpBuffNum < 0)
                {
                    pstSclPortAttr->stoutFileAttr.s32DumpBuffNum = -pstSclPortAttr->stoutFileAttr.s32DumpBuffNum;
                    DBG_INFO("scl devid %d chn %d port %d, dump %d\n", u32SclDevId, u32SclChnId, u32SclPortId, pstSclPortAttr->stoutFileAttr.s32DumpBuffNum);
                }

                while(pstSclPortAttr->stoutFileAttr.s32DumpBuffNum > 0 || pstSclPortAttr->stoutFileAttr.fp != NULL)
                {
                    usleep(5000);
                }
                DBG_INFO("slc module: devid %d chnid %d port %d, wait dump finish!\n",u32SclDevId,u32SclChnId,u32SclPortId);
            }
        }
    }

    return;
}
#ifndef ExecFuncResult
#define ExecFuncResult(_func_, _ret_) \
        do{ \
            _ret_ = _func_; \
            if (_ret_ != MI_SUCCESS) \
            { \
                printf("[%s %d]exec function failed, error:%x\n", __func__, __LINE__, _ret_); \
                goto EXIT; \
            } \
            else \
            { \
                printf("[%s %d]exec function pass\n", __func__, __LINE__); \
            } \
        } while(0)
#endif

#define info(fmt, args...)                                                     \
    ({                                                                              \
        do {                                                                        \
        printf(ASCII_COLOR_GREEN "[APP INFO]:%s[%d]: ", __FUNCTION__, __LINE__);    \
        printf(fmt, ##args);                                                        \
        printf(ASCII_COLOR_END);                                                    \
        } while (0);                                                                \
    })
#define debug(fmt, args...)                                                     \
    ({                                                                              \
        do {                                                                        \
        printf(ASCII_COLOR_YELLOW "[APP INFO]:%s[%d]: ", __FUNCTION__, __LINE__);    \
        printf(fmt, ##args);                                                        \
        printf(ASCII_COLOR_END);                                                    \
        } while (0);                                                                \
    })
#define err(fmt, args...)                                                      \
    ({                                                                              \
        do {                                                                        \
        printf(ASCII_COLOR_RED "[APP ERR ]:%s[%d]: ", __FUNCTION__, __LINE__);      \
        printf(fmt, ##args);                                                        \
        printf(ASCII_COLOR_END);                                                    \
        } while (0);                                                                \
    })

#define ExecFunc(_func_, _ret_)                                                \
  if (_func_ != _ret_) {                                                       \
    err("[%d]exec function failed\n", __LINE__);                            \
    return;                                                                 \
  } else {                                                                     \
    info("(%d)exec function pass\n", __LINE__);                              \
  }


MI_S32 SensorModuleInit(MI_SNR_PADID eSnrPad ,MI_U8 u8ChocieRes)
{

    MI_SNR_PADID eSnrPadId = eSnrPad;
    MI_SNR_PADInfo_t  stPad0Info;
    MI_SNR_PlaneInfo_t stSnrPlane0Info;
    MI_U32 u32ResCount =0;
    MI_U8 u8ResIndex =0;
    MI_S32 s32Input =0;
    MI_SNR_Res_t stRes;
    ST_Sensor_Attr_t *pstSensorAttr = NULL;
    memset(&stRes, 0x0, sizeof(MI_SNR_Res_t));
    memset(&stPad0Info, 0x0, sizeof(MI_SNR_PADInfo_t));
    memset(&stSnrPlane0Info, 0x0, sizeof(MI_SNR_PlaneInfo_t));

   STCHECKRESULT(MI_SNR_SetPlaneMode(eSnrPad, FALSE));
    
    STCHECKRESULT(MI_SNR_QueryResCount(eSnrPadId, &u32ResCount));
    for(u8ResIndex=0; u8ResIndex < u32ResCount; u8ResIndex++)
    {
        STCHECKRESULT(MI_SNR_GetRes(eSnrPadId, u8ResIndex, &stRes));
        printf("index %d, Crop(%d,%d,%d,%d), outputsize(%d,%d), maxfps %d, minfps %d, ResDesc %s\n",
        u8ResIndex,
        stRes.stCropRect.u16X, stRes.stCropRect.u16Y, stRes.stCropRect.u16Width,stRes.stCropRect.u16Height,
        stRes.stOutputSize.u16Width, stRes.stOutputSize.u16Height,
        stRes.u32MaxFps,stRes.u32MinFps,
        stRes.strResDesc);
    }

	
    if(u8ChocieRes >= u32ResCount && u8ChocieRes != 0xff)
    {
        printf("res set err  %d > =cnt %d\n", u8ChocieRes, u32ResCount);
        return -1;
    }
    else if(u8ChocieRes == 0xff)
    {
        printf("choice which resolution use, cnt %d\n", u32ResCount);
        do
        {
            scanf("%d", &s32Input);
            u8ChocieRes = (MI_U8)s32Input;
            ST_Flush();
            STCHECKRESULT(MI_SNR_QueryResCount(eSnrPadId, &u32ResCount));
            if(u8ChocieRes >= u32ResCount)
            {
                printf("choice err res %d > =cnt %d\n", u8ChocieRes, u32ResCount);
            }
        }while(u8ChocieRes >= u32ResCount);
        printf("You select %d res\n", u8ChocieRes);
    }
    printf("eSnrPad:%d Rest %d\n", eSnrPad, u8ChocieRes);

    STCHECKRESULT(MI_SNR_SetRes(eSnrPadId,u8ChocieRes));
    STCHECKRESULT(MI_SNR_Enable(eSnrPadId));

    return MI_SUCCESS;
}

MI_S32 VifModuleInit(MI_VIF_GROUP GroupId,MI_SNR_PADID eSnrPad,MI_SYS_PixelFormat_e InputPixel)
{
   MI_S32 s32Ret = MI_SUCCESS;

   MI_VIF_DEV vifDev =0;
   MI_U8 vifDevIdPerGroup = 0;
   MI_VIF_PORT vifPort = 0;

   MI_SNR_PADInfo_t  stPad0Info;
   MI_SNR_PlaneInfo_t stSnrPlane0Info;

   memset(&stPad0Info, 0x0, sizeof(MI_SNR_PADInfo_t));
   memset(&stSnrPlane0Info, 0x0, sizeof(MI_SNR_PlaneInfo_t));
   MI_VIF_GroupAttr_t stGroupAttr;
   memset(&stGroupAttr, 0x0, sizeof(MI_VIF_GroupAttr_t));

   //GetSnrInfo(&stPad0Info, &stSnrPlane0Info);
   ExecFuncResult(MI_SNR_GetPadInfo(eSnrPad, &stPad0Info), s32Ret);
   ExecFuncResult(MI_SNR_GetPlaneInfo(eSnrPad, 0, &stSnrPlane0Info), s32Ret);

   //E_MI_VIF_MODE_BT1120_STANDARD

   stGroupAttr.eIntfMode = E_MI_VIF_MODE_MIPI;
   //stGroupAttr.eIntfMode = E_MI_VIF_MODE_BT1120_STANDARD;
   stGroupAttr.eWorkMode = E_MI_VIF_WORK_MODE_1MULTIPLEX;
   stGroupAttr.eHDRType = E_MI_VIF_HDR_TYPE_OFF;
   //stGroupAttr.eClkEdge = E_MI_VIF_CLK_EDGE_SINGLE_UP;
   stGroupAttr.eClkEdge = E_MI_VIF_CLK_EDGE_DOUBLE;

   ExecFuncResult(MI_VIF_CreateDevGroup(GroupId, &stGroupAttr), s32Ret);

   vifDev = GroupId*MI_VIF_MAX_GROUP_DEV_CNT    +   vifDevIdPerGroup;	

   MI_VIF_DevAttr_t stVifDevAttr;
   memset(&stVifDevAttr, 0x0, sizeof(MI_VIF_DevAttr_t));


   stVifDevAttr.stInputRect.u16X = stSnrPlane0Info.stCapRect.u16X;
   stVifDevAttr.stInputRect.u16Y = stSnrPlane0Info.stCapRect.u16Y;
   stVifDevAttr.stInputRect.u16Width = stSnrPlane0Info.stCapRect.u16Width;
   stVifDevAttr.stInputRect.u16Height = stSnrPlane0Info.stCapRect.u16Height;
   stVifDevAttr.bEnH2T1PMode = 0;
   stVifDevAttr.eField = E_MI_SYS_FIELDTYPE_NONE;
   stVifDevAttr.eInputPixel = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(stSnrPlane0Info.ePixPrecision, stSnrPlane0Info.eBayerId);//InputPixel;

   ExecFuncResult(MI_VIF_SetDevAttr(vifDev, &stVifDevAttr), s32Ret);
   ExecFuncResult(MI_VIF_EnableDev(vifDev), s32Ret);


   MI_VIF_OutputPortAttr_t stVifPortInfo;
   memset(&stVifPortInfo, 0, sizeof(MI_VIF_OutputPortAttr_t));

   stVifPortInfo.stCapRect.u16X = stSnrPlane0Info.stCapRect.u16X;
   stVifPortInfo.stCapRect.u16Y = stSnrPlane0Info.stCapRect.u16Y;
   stVifPortInfo.stCapRect.u16Width 		  = stSnrPlane0Info.stCapRect.u16Width;
   stVifPortInfo.stCapRect.u16Height = stSnrPlane0Info.stCapRect.u16Height;
   stVifPortInfo.stDestSize.u16Width = stSnrPlane0Info.stCapRect.u16Width;
   stVifPortInfo.stDestSize.u16Height = stSnrPlane0Info.stCapRect.u16Height;
   stVifPortInfo.ePixFormat =(MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(stSnrPlane0Info.ePixPrecision, stSnrPlane0Info.eBayerId);//InputPixel;
   stVifPortInfo.eFrameRate = E_MI_VIF_FRAMERATE_FULL;
   stVifPortInfo.eCompressMode = E_MI_SYS_COMPRESS_MODE_NONE;

   ExecFuncResult(MI_VIF_SetOutputPortAttr(vifDev, 0, &stVifPortInfo), s32Ret);
   ExecFuncResult(MI_VIF_EnableOutputPort(vifDev, 0), s32Ret);

   MI_SYS_ChnPort_t stChnPort;
   memset(&stChnPort, 0x0, sizeof(MI_SYS_ChnPort_t));

   stChnPort.eModId=E_MI_MODULE_ID_VIF;
   stChnPort.u32DevId=vifDev;
   stChnPort.u32ChnId=0;
   stChnPort.u32PortId=vifPort;
   ExecFuncResult(MI_SYS_SetChnOutputPortDepth(0, &stChnPort, 0, 8), s32Ret);
   printf("VifModuleInit GroupId:%d vifDev:%d\n", GroupId, vifDev);


EXIT:

   return s32Ret;
}

MI_S32 VifModuleUnInit(MI_VIF_GROUP groupId)
{
    MI_VIF_DEV vifDev = 0;
    MI_VIF_PORT vifPort=0;
    MI_U16 vifDevIdPerGroup=0;
    for(vifDevIdPerGroup=0; vifDevIdPerGroup< ST_MAX_VIF_DEV_PERGROUP; vifDevIdPerGroup++)
    {
        vifDev = groupId*ST_MAX_VIF_DEV_PERGROUP+vifDevIdPerGroup;

        for(vifPort=0; vifPort< ST_MAX_VIF_OUTPORT_NUM; vifPort++)
        {
            STCHECKRESULT(MI_VIF_DisableOutputPort(vifDev, vifPort));
        }
        STCHECKRESULT(MI_VIF_DisableDev(vifDev));
    }

    STCHECKRESULT(MI_VIF_DestroyDevGroup(groupId));

    return MI_SUCCESS;
}


MI_S32 ST_IspChannelInit(MI_ISP_DEV IspDevId,MI_ISP_CHANNEL IspChnId,MI_SNR_PADID eMiSnrPadId)
{
	MI_ISP_ChannelAttr_t  stIspChnAttr;
    MI_ISP_PORT  IspOutPortId =1;
	MI_S32 s32Ret = MI_SUCCESS;

	
	memset(&stIspChnAttr, 0x0, sizeof(MI_ISP_ChannelAttr_t));


	ST_TransMISnrPadToMIIspBindSensorId(eMiSnrPadId, (MI_ISP_BindSnrId_e *)&stIspChnAttr.u32SensorBindId);


		
	stIspChnAttr.u32Sync3AType = E_MI_ISP_SYNC3A_AE|E_MI_ISP_SYNC3A_AWB|E_MI_ISP_SYNC3A_IQ|E_MI_ISP_SYNC3A_1ST_SNR_ONLY;

	ExecFuncResult(MI_ISP_CreateChannel(IspDevId, IspChnId, &stIspChnAttr), s32Ret);

	ExecFuncResult(MI_ISP_SetChnOverlapAttr(IspDevId, IspChnId,E_MI_ISP_OVERLAP_256),s32Ret);

	MI_ISP_ChnParam_t  stChnParam;
	memset(&stChnParam,0x0,sizeof(MI_ISP_ChnParam_t));
	stChnParam.e3DNRLevel = E_MI_ISP_3DNR_LEVEL2;

	ExecFuncResult(MI_ISP_SetChnParam(IspDevId, IspChnId, &stChnParam),s32Ret);

	ExecFuncResult(MI_ISP_StartChannel(IspDevId, IspChnId), s32Ret);

                
    MI_ISP_OutPortParam_t  stIspOutputParam;

    memset(&stIspOutputParam, 0x0, sizeof(MI_ISP_OutPortParam_t));

    stIspOutputParam.ePixelFormat = E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420;
    stIspOutputParam.eCompressMode = E_MI_SYS_COMPRESS_MODE_NONE;

    ExecFuncResult(MI_ISP_SetOutputPortParam(IspDevId, IspChnId, IspOutPortId, &stIspOutputParam), s32Ret);

    ExecFuncResult(MI_ISP_EnableOutputPort(IspDevId, IspChnId, IspOutPortId), s32Ret);


EXIT:
		return s32Ret;

}


MI_S32 IspModuleInit(MI_ISP_DEV IspDevId,MI_SNR_PADID eSnrPad[])
{
    MI_S32 s32Ret = MI_SUCCESS;
    MI_ISP_DevAttr_t stCreateDevAttr;
	MI_U32 u32MultiIspDev = MI_ISP_CREATE_MULTI_DEV(IspDevId);
    memset(&stCreateDevAttr, 0x0, sizeof(MI_ISP_DevAttr_t));
    //stCreateDevAttr.u32DevStitchMask = (0x1<<IspDevId);
	stCreateDevAttr.u32DevStitchMask = E_MI_ISP_DEVICEMASK_ID0|E_MI_ISP_DEVICEMASK_ID1; 
	//STCHECKRESULT(MI_ISP_CreateDevice(u32MultiIspDev, &stCreateDevAttr));  //MI_ISP_CREATE_MULTI_DEV use mullti device

    ExecFuncResult(MI_ISP_CreateDevice(u32MultiIspDev, &stCreateDevAttr), s32Ret);

	for(int i = 0; i < 4; i++)
	{
		ExecFuncResult(ST_IspChannelInit(IspDevId, i, eSnrPad[i]), s32Ret);
	}
         
EXIT:
    return s32Ret;
}

#define SCL_MAX_NUM 8
#define MAX_ROOM_NUM 4
SclModuleNode sclNode[SCL_MAX_NUM];


void construct_scl_module(void)
{ 
	MI_U32 u32Width;
	MI_U32 u32Height;

	if(btest_1080)
	{
	     u32Width  = 1920;
	     u32Height = 1080;
	}
	else
	{
	     u32Width  = 3840;
	     u32Height = 2160;
	}
	
    SclModuleInitInfo _SclInfo[MAX_ROOM_NUM][SCL_MAX_NUM] = 
    { 
        {   //              HWSclId             Dev Chn Port   OutW       OutH                   ePixelFormat             bCreateDev

			{E_MI_SCL_HWSCL0 | E_MI_SCL_HWSCL1 , 1,  0,  0,  1920, 1080, E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420, true},
			{E_MI_SCL_HWSCL0 | E_MI_SCL_HWSCL1 , 1,  1,  1,  1920, 1080, E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420, false},


			{E_MI_SCL_HWSCL4 | E_MI_SCL_HWSCL5 , 5,  0,  0,  1920, 1080, E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420, true},
			{E_MI_SCL_HWSCL4 | E_MI_SCL_HWSCL5 , 5,  1,  1,  1920, 1080, E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420, false},
			
			{E_MI_SCL_HWSCL8                   , 3,  0,  0,  u32Width, u32Height, E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420, true},


        },
    };

    for (auto i = 0; i < 5; i++)
    {
        info("--->initSclNode RoomID = %d, i = %d\n", RoomID, i);
        
        SclModuleNode::initSclNode(sclNode[i], _SclInfo[RoomID][i]);

        info("--->StInit i = %d [dev:%d, chn:%d, outPort:%d]\n", i, sclNode[i].u32DevId, sclNode[i].u32ChnId, sclNode[i].u32OutPort);

        sclNode[i].StInit();
    }

 



    info("---------------->construct_scl_module\n\n");
}

void construct_scl_rtsp_module(void)
{ 

	
    SclModuleInitInfo _SclInfo[3][SCL_MAX_NUM] = 
    { 
        {   //              HWSclId             Dev Chn Port   OutW       OutH                   ePixelFormat             bCreateDev

			{E_MI_SCL_HWSCL0 | E_MI_SCL_HWSCL1 , 1,  0,  0,  5472, 3078, E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420, true},
			{E_MI_SCL_HWSCL0 | E_MI_SCL_HWSCL1 , 1,  1,  1,  2592, 1944, E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420, false},


			{E_MI_SCL_HWSCL4 | E_MI_SCL_HWSCL5 , 5,  0,  0,  2592, 1944, E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420, true},
			{E_MI_SCL_HWSCL4 | E_MI_SCL_HWSCL5 , 5,  1,  1,  2592, 1944, E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420, false},
			
			//{E_MI_SCL_HWSCL8                   , 3,  0,  0,  u32Width, u32Height, E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420, true},


        },
    };

    for (auto i = 0; i < 4; i++)
    {
        info("--->initSclNode RoomID = %d, i = %d\n", RoomID, i);
        
        SclModuleNode::initSclNode(sclNode[i], _SclInfo[RoomID][i]);

        info("--->StInit i = %d [dev:%d, chn:%d, outPort:%d]\n", i, sclNode[i].u32DevId, sclNode[i].u32ChnId, sclNode[i].u32OutPort);

        sclNode[i].StInit();
    }

 		ST_Sys_BindInfo_T stBindInfo;
		memset(&stBindInfo, 0x0, sizeof(ST_Sys_BindInfo_T));
		MI_U16 u16SocId = 0;
		MI_SYS_ChnPort_t stSrcChnPort;
		MI_SYS_ChnPort_t stDstChnPort;
		//step4
		/*
		MI_U32 u32DevId = -1;
		ExecFunc(MI_VENC_GetChnDevid(u32VencChn, &u32DevId), MI_SUCCESS);*/
		stSrcChnPort.eModId = E_MI_MODULE_ID_ISP;
		stSrcChnPort.u32DevId = 0;
		stSrcChnPort.u32ChnId = 0;
		stSrcChnPort.u32PortId = 1;
	
		stDstChnPort.eModId = E_MI_MODULE_ID_SCL;
		stDstChnPort.u32DevId = 1;
		stDstChnPort.u32ChnId = 0;
		stDstChnPort.u32PortId = 0;
		MI_SYS_BindChnPort2(u16SocId, &stSrcChnPort, &stDstChnPort, 30, 30, E_MI_SYS_BIND_TYPE_FRAME_BASE, 0);

		stSrcChnPort.eModId = E_MI_MODULE_ID_ISP;
		stSrcChnPort.u32DevId = 0;
		stSrcChnPort.u32ChnId = 1;
		stSrcChnPort.u32PortId = 1;
		
		stDstChnPort.eModId = E_MI_MODULE_ID_SCL;
		stDstChnPort.u32DevId = 1;
		stDstChnPort.u32ChnId = 1;
		stDstChnPort.u32PortId = 0;
	
		MI_SYS_BindChnPort2(u16SocId, &stSrcChnPort, &stDstChnPort, 30, 30, E_MI_SYS_BIND_TYPE_FRAME_BASE, 0);

		stSrcChnPort.eModId = E_MI_MODULE_ID_ISP;
		stSrcChnPort.u32DevId = 0;
		stSrcChnPort.u32ChnId = 2;
		stSrcChnPort.u32PortId = 1;
		
		stDstChnPort.eModId = E_MI_MODULE_ID_SCL;
		stDstChnPort.u32DevId = 5;
		stDstChnPort.u32ChnId = 0;
		stDstChnPort.u32PortId = 0;
	

		MI_SYS_BindChnPort2(u16SocId, &stSrcChnPort, &stDstChnPort, 30, 30, E_MI_SYS_BIND_TYPE_FRAME_BASE, 0);

		stSrcChnPort.eModId = E_MI_MODULE_ID_ISP;
		stSrcChnPort.u32DevId = 0;
		stSrcChnPort.u32ChnId = 3;
		stSrcChnPort.u32PortId = 1;
		
		stDstChnPort.eModId = E_MI_MODULE_ID_SCL;
		stDstChnPort.u32DevId = 5;
		stDstChnPort.u32ChnId = 1;
		stDstChnPort.u32PortId = 0;

		MI_SYS_BindChnPort2(u16SocId, &stSrcChnPort, &stDstChnPort, 30, 30, E_MI_SYS_BIND_TYPE_FRAME_BASE, 0);




    info("---------------->construct_scl_rtsp_module\n\n");
}

MI_S32 St_Sys_Bind(    MI_ModuleId_e eSrcModId, MI_U32          u32SrcDevId,    MI_U32    u32SrcChnId, MI_U32        u32SrcPortId,
   						   MI_ModuleId_e eDstModId,        MI_U32  u32DstDevId, MI_U32        u32DstChnId,MI_U32        u32DstPortId)


{
		MI_S32 s32Ret = MI_SUCCESS;

		MI_SYS_ChnPort_t stSrcChnPort;
		MI_SYS_ChnPort_t stDstChnPort;

		
  		stSrcChnPort.eModId = eSrcModId;
	    stSrcChnPort.u32DevId = u32SrcDevId;
	    stSrcChnPort.u32ChnId = u32SrcChnId;
	    stSrcChnPort.u32PortId = u32SrcPortId;

	    stDstChnPort.eModId = eDstModId;
	    stDstChnPort.u32DevId = u32DstDevId;
	    stDstChnPort.u32ChnId = u32DstChnId;
	    stDstChnPort.u32PortId = u32DstPortId;

		MI_SYS_BindChnPort2(0, &stSrcChnPort, &stDstChnPort, 30, 30, E_MI_SYS_BIND_TYPE_FRAME_BASE, 0);

EXIT:
    return s32Ret;
}


int32_t BaseModuleInit(void)
{
		MI_S32 s32Ret = MI_SUCCESS;

		MI_S32 enable0 = 1;
		MI_S32 enable1 = 1;
		MI_S32 enable2 = 1;
		MI_S32 enable3 = 1;
		MI_U32 u32VifGroup[] = {0, 2, 4, 6};
	
		MI_SNR_PADID eIsp0SnrPad[] = {0, 1, 4, 5};
		//MI_SNR_PADID eIsp1SnrPad[] = {4, 5};
	
		char*env  =  getenv("REACH_VI");
	
		if(env)
		{
			sscanf(env, "%d %d %d %d", &enable0,&enable1,&enable2,&enable3);
		}
		
		printf("enable:%d %d %d %d	 env:%s\n", enable0,enable1,enable2,enable3,env);
	
	
		if(enable0)
		{
			SensorModuleInit(0,0);
			eIsp0SnrPad[0] = 0;
		}
	
		if(enable1)
		{
			SensorModuleInit(1,1);
			eIsp0SnrPad[1] = 1;
		}
	
		if(enable2)
		{
			SensorModuleInit(4,1);
			eIsp0SnrPad[2] = 4;
		}
	
		if(enable3)
		{
			SensorModuleInit(5,1);
			eIsp0SnrPad[3] = 5;
		}
	
	
		//初始化4路采集
	
		if(enable0)
		{
			ExecFuncResult(VifModuleInit((MI_VIF_GROUP)u32VifGroup[0],0,(MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_10BPP,E_MI_SYS_PIXEL_BAYERID_RG)), s32Ret);
		}
	
		if(enable1)
		{
			ExecFuncResult(VifModuleInit((MI_VIF_GROUP)u32VifGroup[1],1,(MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_10BPP,E_MI_SYS_PIXEL_BAYERID_RG)), s32Ret);
		}
	
		if(enable2)
		{
			ExecFuncResult(VifModuleInit((MI_VIF_GROUP)u32VifGroup[2],4,(MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_10BPP,E_MI_SYS_PIXEL_BAYERID_RG)), s32Ret);
		}
		
		if(enable3)
		{
			ExecFuncResult(VifModuleInit((MI_VIF_GROUP)u32VifGroup[3],5,(MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_10BPP,E_MI_SYS_PIXEL_BAYERID_RG)), s32Ret);
		}

		//初始化ISP vif8 vif24 使用规避直连scl帧率异常问题
		IspModuleInit(0, eIsp0SnrPad);
		//IspModuleInit(1, eIsp1SnrPad);

		if(enable0)
		{
			ExecFuncResult(St_Sys_Bind(E_MI_MODULE_ID_VIF, ST_MAX_VIF_DEV_PERGROUP*u32VifGroup[0], 0, 0,    E_MI_MODULE_ID_ISP, 0, 0, 0), s32Ret);
		}
	
		if(enable1)
		{
			ExecFuncResult(St_Sys_Bind(E_MI_MODULE_ID_VIF, ST_MAX_VIF_DEV_PERGROUP*u32VifGroup[1], 0, 0,    E_MI_MODULE_ID_ISP, 0, 1, 0), s32Ret);
		}
		
		if(enable2)
		{
			ExecFuncResult(St_Sys_Bind(E_MI_MODULE_ID_VIF, ST_MAX_VIF_DEV_PERGROUP*u32VifGroup[2], 0, 0,    E_MI_MODULE_ID_ISP, 0, 2, 0), s32Ret);
		}
	
		if(enable3)
		{
			ExecFuncResult(St_Sys_Bind(E_MI_MODULE_ID_VIF, ST_MAX_VIF_DEV_PERGROUP*u32VifGroup[3], 0, 0,    E_MI_MODULE_ID_ISP, 0, 3, 0), s32Ret);
		}


 EXIT:
				
 	return s32Ret;


}


static volatile int NeedDeinitVDISP = 0;
#define MAKE_YUYV_VALUE(y, u, v) ((y) << 24) | ((u) << 16) | ((y) << 8) | (v)
#define YUYV_BLACK              MAKE_YUYV_VALUE(0,128,128)
#define YUYV_WHITE              MAKE_YUYV_VALUE(255,128,128)
#define YUYV_RED                MAKE_YUYV_VALUE(76,84,255)
#define YUYV_GREEN              MAKE_YUYV_VALUE(149,43,21)
#define YUYV_BLUE               MAKE_YUYV_VALUE(29,225,107)

void construct_vdisp_module(void)
{
    typedef struct RECT {
        /*
        (x,y)_ _ _ _ _ _ _ | | | (h)
        |_ _ _(w)_ _ _|
        */
        short x;
        short y;
        short w;
        short h;
    } RECT;

    /*
     *初始化 vdisp 模块
     */
    if (MI_SUCCESS == MI_VDISP_Init())
    {
        NeedDeinitVDISP = 1;
    }

    MI_VDISP_InputChnAttr_t stInputChnAttr;
    MI_VDISP_OutputPortAttr_t stOutputPortAttr;
    MI_VDISP_DEV DevId = 0;
    RECT rect_0, rect_1, rect_2, rect_3;
    MI_VDISP_CHN Chn_0_Id = 16;
    MI_VDISP_CHN Chn_1_Id = 17;
    MI_VDISP_CHN Chn_2_Id = 18;
    MI_VDISP_CHN Chn_3_Id = 19;
    /*
     *打开一个 vdisp 虚拟设备,以便开始对这个设备进行配置
     */
    MI_VDISP_OpenDevice(DevId);

    /*
     *为了满足 vdisp 缩略图窗口的对齐要求，对 x 坐标做 16 向下对齐
     *设置 input channel 参数
     */
    stInputChnAttr.s32IsFreeRun = TRUE;
	stInputChnAttr.u32OutHeight = 1080;//rect_0.h;
	stInputChnAttr.u32OutWidth = 1920;//rect_0.w;
    stInputChnAttr.u32OutX = 0;//rect_0.x;
    stInputChnAttr.u32OutY = 0;//rect_0.y;
    /*
     *为 input channel VDISP_OVERLAYINPUTCHNID 设置参数
     */
    MI_VDISP_SetInputChannelAttr(DevId, Chn_0_Id, &stInputChnAttr);

    /*
     *为了满足 vdisp 缩略图窗口的对齐要求，对 x 坐标做 16 向下对齐
     *设置 input channel 参数
     */

    stInputChnAttr.s32IsFreeRun = TRUE;
    stInputChnAttr.u32OutHeight = 1080;//rect_0.h;
    stInputChnAttr.u32OutWidth = 1920;//rect_0.w;
    stInputChnAttr.u32OutX = 1920;//rect_0.x;
    stInputChnAttr.u32OutY = 0;//rect_0.y;

    /*
     *为 input channel 1 设置参数
     */
    MI_VDISP_SetInputChannelAttr(DevId, Chn_1_Id, &stInputChnAttr);

    /*
     *为了满足 vdisp 缩略图窗口的对齐要求，对 x 坐标做 16 向下对齐
     *设置 input channel 参数
     */

    stInputChnAttr.s32IsFreeRun = TRUE;
	stInputChnAttr.u32OutHeight = 1080;//rect_0.h;
	stInputChnAttr.u32OutWidth = 1920;//rect_0.w;
	stInputChnAttr.u32OutX = 0;//rect_0.x;
	stInputChnAttr.u32OutY = 1080;//rect_0.y;

    /*
     *为 input channel 2 设置参数
     */
    MI_VDISP_SetInputChannelAttr(DevId, Chn_2_Id, &stInputChnAttr);

    /*
     *为了满足 vdisp 缩略图窗口的对齐要求，对 x 坐标做 16 向下对齐
     *设置 input channel 参数
     */

    stInputChnAttr.s32IsFreeRun = TRUE;
	stInputChnAttr.u32OutHeight = 1080;//rect_0.h;
	stInputChnAttr.u32OutWidth = 1920;//rect_0.w;
	stInputChnAttr.u32OutX = 1920;//rect_0.x;
	stInputChnAttr.u32OutY = 1080;//rect_0.y;

    /*
     *为 input channel 2 设置参数
     */
    MI_VDISP_SetInputChannelAttr(DevId, Chn_3_Id, &stInputChnAttr);



    //设置输出的颜色格式
    stOutputPortAttr.ePixelFormat = E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420;
    //设置 vdisp 输出帧中未使用区域的涂黑颜色,YUV 颜色空间
    stOutputPortAttr.u32BgColor = YUYV_BLACK;
    //设置 vdisp 输出帧率
    stOutputPortAttr.u32FrmRate = 30;
    //设置 vdisp 输出帧的高
    stOutputPortAttr.u32Height = 2160;
    //设置 vdisp 输出帧的宽
    stOutputPortAttr.u32Width = 3840;
    //设置 vdisp 输出帧的最低 PTS
    stOutputPortAttr.u64pts = 0;
    /*
     *为 output port 0 设置参数
     */
    MI_VDISP_SetOutputPortAttr(DevId, 0, &stOutputPortAttr);
    /*
     *在设置完 input channel 参数之后，
     *激活对应的 channel，以供使用
     */
    MI_VDISP_EnableInputChannel(DevId, Chn_0_Id);
    MI_VDISP_EnableInputChannel(DevId, Chn_1_Id);
    MI_VDISP_EnableInputChannel(DevId, Chn_2_Id);
    MI_VDISP_EnableInputChannel(DevId, Chn_3_Id);
    /*
     *在 vdisp 的 input channel&output port 都配置完参数之后，
     *让 vdisp 的这个设备开始工作
     */
    MI_VDISP_StartDev(DevId);

    MI_U32 u32BufQueueDepth = 5;

    MI_SYS_ChnPort_t stChnPort;
    stChnPort.eModId = E_MI_MODULE_ID_VDISP;
    stChnPort.u32DevId = DevId;
    stChnPort.u32ChnId = 0;
    stChnPort.u32PortId = 0;

    MI_SYS_SetChnOutputPortDepth(0, &stChnPort, 0, u32BufQueueDepth);

    info("---------------->construct_vdisp_module\n\n");
}
volatile int gs32ReturnValue = 0;
volatile bool gbReturnCheck = true;

#define DISP_UT_RESULT_CHECK(func)  do{\
    int __ret__ = 0;\
    if (gbReturnCheck)\
    {\
        __ret__ = (func);\
        if (__ret__ != MI_SUCCESS)\
        {\
            info("err ret:%#x in %d for %s\n", __ret__, __LINE__, #func);\
            gs32ReturnValue = -1;\
        }\
    }\
}while(0)
static MI_S32 Hdmi_callback(MI_HDMI_DeviceId_e eHdmi, MI_HDMI_EventType_e Event, void* pEventParam, void* pUsrParam)
{
	switch (Event)
	{
	case E_MI_HDMI_EVENT_HOTPLUG:
		info("E_MI_HDMI_EVENT_HOTPLUG.\n");
		DISP_UT_RESULT_CHECK(MI_HDMI_Start(eHdmi));
		break;
	case E_MI_HDMI_EVENT_NO_PLUG:
		info("E_MI_HDMI_EVENT_NO_PLUG.\n");
		DISP_UT_RESULT_CHECK(MI_HDMI_Stop(eHdmi));
		break;
	default:
		info("Unsupport event.\n");
		break;
	}
	return MI_SUCCESS;
}



void ST_DefaultArgs()
{
    ST_VencAttr_t *pstStreamAttr = gstVencattr;

    pstStreamAttr[0].bEnable = TRUE;
    pstStreamAttr[0].szStreamName = SUB_STREAM0;
    pstStreamAttr[0].DevId = MI_VENC_DEV_ID_H264_H265_0;
    pstStreamAttr[0].vencChn = 0;
    pstStreamAttr[0].eType = E_MI_VENC_MODTYPE_H265E;
    pstStreamAttr[0].u32Height = 5472;
    pstStreamAttr[0].u32Width = 3078;    
    pstStreamAttr[0].stVencInBindParam.stChnPort.eModId = E_MI_MODULE_ID_SCL;
    pstStreamAttr[0].stVencInBindParam.stChnPort.u32DevId = 1;
    pstStreamAttr[0].stVencInBindParam.stChnPort.u32ChnId = 0;
    pstStreamAttr[0].stVencInBindParam.stChnPort.u32PortId = 0;
    pstStreamAttr[0].stVencInBindParam.eBindType = E_MI_SYS_BIND_TYPE_FRAME_BASE;
    pstStreamAttr[0].u32Fps = 30;

    pstStreamAttr[1].bEnable = TRUE;
    pstStreamAttr[1].szStreamName = SUB_STREAM1;
    pstStreamAttr[1].DevId = MI_VENC_DEV_ID_H264_H265_0;
    pstStreamAttr[1].vencChn = 1;
    pstStreamAttr[1].eType = E_MI_VENC_MODTYPE_H265E;
    pstStreamAttr[1].u32Height = 2592;
    pstStreamAttr[1].u32Width = 1944;    
    pstStreamAttr[1].stVencInBindParam.stChnPort.eModId = E_MI_MODULE_ID_SCL;
    pstStreamAttr[1].stVencInBindParam.stChnPort.u32DevId = 1;
    pstStreamAttr[1].stVencInBindParam.stChnPort.u32ChnId = 1;
    pstStreamAttr[1].stVencInBindParam.stChnPort.u32PortId = 1;
    pstStreamAttr[1].stVencInBindParam.eBindType = E_MI_SYS_BIND_TYPE_FRAME_BASE;
    pstStreamAttr[1].u32Fps = 30;

    pstStreamAttr[2].bEnable = TRUE;
    pstStreamAttr[2].szStreamName = SUB_STREAM2;
    pstStreamAttr[2].DevId = MI_VENC_DEV_ID_H264_H265_1;
    pstStreamAttr[2].vencChn = 0;
    pstStreamAttr[2].eType = E_MI_VENC_MODTYPE_H264E;
    pstStreamAttr[2].u32Height = 2592;
    pstStreamAttr[2].u32Width = 1944;    
    pstStreamAttr[2].stVencInBindParam.stChnPort.eModId = E_MI_MODULE_ID_SCL;
    pstStreamAttr[2].stVencInBindParam.stChnPort.u32DevId = 5;
    pstStreamAttr[2].stVencInBindParam.stChnPort.u32ChnId = 0;
    pstStreamAttr[2].stVencInBindParam.stChnPort.u32PortId = 0;
    pstStreamAttr[2].stVencInBindParam.eBindType = E_MI_SYS_BIND_TYPE_FRAME_BASE;
    pstStreamAttr[2].u32Fps = 30;

    pstStreamAttr[3].bEnable = TRUE;
    pstStreamAttr[3].szStreamName = SUB_STREAM3;
    pstStreamAttr[3].DevId = MI_VENC_DEV_ID_H264_H265_1;
    pstStreamAttr[3].vencChn = 1;
    pstStreamAttr[3].eType = E_MI_VENC_MODTYPE_H264E;
    pstStreamAttr[3].u32Height = 2592;
    pstStreamAttr[3].u32Width = 1944;    
    pstStreamAttr[3].stVencInBindParam.stChnPort.eModId = E_MI_MODULE_ID_SCL;
    pstStreamAttr[3].stVencInBindParam.stChnPort.u32DevId = 5;
    pstStreamAttr[3].stVencInBindParam.stChnPort.u32ChnId = 1;
    pstStreamAttr[3].stVencInBindParam.stChnPort.u32PortId = 1;
    pstStreamAttr[3].stVencInBindParam.eBindType = E_MI_SYS_BIND_TYPE_FRAME_BASE;
    pstStreamAttr[3].u32Fps = 30;
    
}


MI_S32 VencModuleInit(MI_U32 u32VencChn)
{
	   ST_VencAttr_t *pstStreamAttr = gstVencattr; 
		MI_U32 u32BindDevId =pstStreamAttr[u32VencChn].stVencInBindParam.stChnPort.u32DevId;
		MI_U32 u32BindChn=pstStreamAttr[u32VencChn].stVencInBindParam.stChnPort.u32ChnId;
		MI_U32 u32BindPort=pstStreamAttr[u32VencChn].stVencInBindParam.stChnPort.u32PortId;
		MI_ModuleId_e eBindModule = pstStreamAttr[u32VencChn].stVencInBindParam.stChnPort.eModId;
	    
		MI_U32 u32VenBitRate = 0;
		MI_U32 u32MaxWidth =0, u32MaxHeight =0;
		//MI_SYS_Rotate_e eRotateType = E_MI_SYS_ROTATE_NUM;	//后续优化代码开
		MI_VENC_DEV DevId =pstStreamAttr[u32VencChn].DevId;
		if(pstStreamAttr[u32VencChn].bEnable != TRUE)
		{
			
			printf("VNEC CHN%d stream is not open %d \n",u32VencChn,pstStreamAttr[u32VencChn].bEnable);
			return 0;
		}
		
		if(E_MI_MODULE_ID_SCL == eBindModule)
		{
			MI_SCL_OutPortParam_t stSclOutputParam;
	
			memset(&stSclOutputParam, 0x0, sizeof(MI_SCL_OutPortParam_t));
	
			STCHECKRESULT(MI_SCL_GetOutputPortParam((MI_SCL_DEV)u32BindDevId, u32BindChn,u32BindPort,&stSclOutputParam));
			
			pstStreamAttr[u32VencChn].u32Width = stSclOutputParam.stSCLOutputSize.u16Width;
			pstStreamAttr[u32VencChn].u32Height = stSclOutputParam.stSCLOutputSize.u16Height;
			
		}
		u32MaxWidth = u16VencMaxW;
		u32MaxHeight = u16VencMaxH;
		u32VenBitRate = ((pstStreamAttr[u32VencChn].u32Width * pstStreamAttr[u32VencChn].u32Height + 500000)/1000000)*1024*1024;
		if(u32VenBitRate == 0)
		{
			u32VenBitRate = 2*1024*1024;
		}
		
		printf("bindParam(%d,%d,%d,%d), chn %d, pichwidth %d, height %d, MaxWidth %d, MaxHeight %d bitrate %d, fps %d \n",
			eBindModule, u32BindDevId,u32BindChn, u32BindPort,
			u32VencChn,pstStreamAttr[u32VencChn].u32Width, pstStreamAttr[u32VencChn].u32Height, u32MaxWidth, u32MaxHeight, u32VenBitRate, pstStreamAttr[u32VencChn].u32Fps);


		if(pstStreamAttr[u32VencChn].vencChn == 0)
		{
		MI_VENC_InitParam_t stInitParam;
		memset(&stInitParam, 0, sizeof(MI_VENC_InitParam_t));
		stInitParam.u32MaxWidth = u32MaxWidth;
		stInitParam.u32MaxHeight = u32MaxHeight;
		STCHECKRESULT(MI_VENC_CreateDev(pstStreamAttr[u32VencChn].DevId, &stInitParam));
		}
	
		//step3
		MI_VENC_ChnAttr_t stChnAttr;
		memset(&stChnAttr, 0, sizeof(MI_VENC_ChnAttr_t));
		if(pstStreamAttr[u32VencChn].eType == E_MI_VENC_MODTYPE_H264E)
		{
			stChnAttr.stVeAttr.stAttrH264e.u32PicWidth = pstStreamAttr[u32VencChn].u32Width;
			stChnAttr.stVeAttr.stAttrH264e.u32PicHeight = pstStreamAttr[u32VencChn].u32Height;
			stChnAttr.stVeAttr.stAttrH264e.u32MaxPicWidth = u32MaxWidth;
			stChnAttr.stVeAttr.stAttrH264e.u32BFrameNum = 2;
			stChnAttr.stVeAttr.stAttrH264e.bByFrame = TRUE;
			stChnAttr.stVeAttr.stAttrH264e.u32MaxPicHeight = u32MaxHeight;
	/*
			stChnAttr.stRcAttr.eRcMode = E_MI_VENC_RC_MODE_H264CBR;
			stChnAttr.stRcAttr.stAttrH264Cbr.u32BitRate = u32VenBitRate;
			stChnAttr.stRcAttr.stAttrH264Cbr.u32FluctuateLevel = 0;
			stChnAttr.stRcAttr.stAttrH264Cbr.u32Gop = 30;
			stChnAttr.stRcAttr.stAttrH264Cbr.u32SrcFrmRateNum = pstStreamAttr[u32VencChn].u32Fps;
			stChnAttr.stRcAttr.stAttrH264Cbr.u32SrcFrmRateDen = 1;
			stChnAttr.stRcAttr.stAttrH264Cbr.u32StatTime = 0;*/
			stChnAttr.stRcAttr.eRcMode = E_MI_VENC_RC_MODE_H264FIXQP;
			stChnAttr.stRcAttr.stAttrH264FixQp.u32Gop = 1;
			stChnAttr.stRcAttr.stAttrH264FixQp.u32IQp = 25;
			stChnAttr.stRcAttr.stAttrH264FixQp.u32PQp = 25;
		    stChnAttr.stRcAttr.stAttrH264FixQp.u32SrcFrmRateNum = pstStreamAttr[u32VencChn].u32Fps;
            stChnAttr.stRcAttr.stAttrH264FixQp.u32SrcFrmRateDen = 1;
	
			//DevId =0;
		}
		else if(pstStreamAttr[u32VencChn].eType == E_MI_VENC_MODTYPE_H265E)
		{
			stChnAttr.stVeAttr.stAttrH265e.u32PicWidth = pstStreamAttr[u32VencChn].u32Width;
			stChnAttr.stVeAttr.stAttrH265e.u32PicHeight = pstStreamAttr[u32VencChn].u32Height;
			stChnAttr.stVeAttr.stAttrH265e.u32MaxPicWidth = u32MaxWidth;
			stChnAttr.stVeAttr.stAttrH265e.u32MaxPicHeight = u32MaxHeight;
			stChnAttr.stVeAttr.stAttrH265e.bByFrame = TRUE;
	/*
			stChnAttr.stRcAttr.eRcMode = E_MI_VENC_RC_MODE_H265CBR;
			stChnAttr.stRcAttr.stAttrH265Cbr.u32BitRate = u32VenBitRate;
			stChnAttr.stRcAttr.stAttrH265Cbr.u32SrcFrmRateNum = pstStreamAttr[u32VencChn].u32Fps;
			stChnAttr.stRcAttr.stAttrH265Cbr.u32SrcFrmRateDen = 1;
			stChnAttr.stRcAttr.stAttrH265Cbr.u32Gop = 30;
			stChnAttr.stRcAttr.stAttrH265Cbr.u32FluctuateLevel = 0;
			stChnAttr.stRcAttr.stAttrH265Cbr.u32StatTime = 0;
			*/
			stChnAttr.stRcAttr.eRcMode = E_MI_VENC_RC_MODE_H265FIXQP;
			stChnAttr.stRcAttr.stAttrH265FixQp.u32Gop = 1;
			if(pstStreamAttr[u32VencChn].vencChn == 0){
			stChnAttr.stRcAttr.stAttrH265FixQp.u32IQp = bIQp;
			stChnAttr.stRcAttr.stAttrH265FixQp.u32PQp = bPQp;
			}
			else
			{
			stChnAttr.stRcAttr.stAttrH265FixQp.u32IQp = 25;
			stChnAttr.stRcAttr.stAttrH265FixQp.u32PQp = 25;
			}
			stChnAttr.stRcAttr.stAttrH265FixQp.u32SrcFrmRateNum = pstStreamAttr[u32VencChn].u32Fps;
            stChnAttr.stRcAttr.stAttrH265FixQp.u32SrcFrmRateDen = 1;
			//DevId =0;
		}
		else if(pstStreamAttr[u32VencChn].eType == E_MI_VENC_MODTYPE_JPEGE)
		{
			stChnAttr.stVeAttr.eType = E_MI_VENC_MODTYPE_JPEGE;
			stChnAttr.stVeAttr.stAttrJpeg.u32PicWidth = pstStreamAttr[u32VencChn].u32Width;
			stChnAttr.stVeAttr.stAttrJpeg.u32PicHeight = pstStreamAttr[u32VencChn].u32Height;
			stChnAttr.stVeAttr.stAttrJpeg.u32MaxPicWidth = u32MaxWidth;
			stChnAttr.stVeAttr.stAttrJpeg.u32MaxPicHeight = u32MaxHeight;
	
			stChnAttr.stRcAttr.eRcMode = E_MI_VENC_RC_MODE_MJPEGFIXQP;
			stChnAttr.stRcAttr.stAttrMjpegFixQp.u32Qfactor = 50;
			stChnAttr.stRcAttr.stAttrMjpegFixQp.u32SrcFrmRateNum = pstStreamAttr[u32VencChn].u32Fps;
			stChnAttr.stRcAttr.stAttrMjpegCbr.u32SrcFrmRateDen = 1;
			//DevId =8;
		}
		stChnAttr.stVeAttr.eType = pstStreamAttr[u32VencChn].eType;
		STCHECKRESULT(ST_Venc_CreateChannel(pstStreamAttr[u32VencChn].DevId, pstStreamAttr[u32VencChn].vencChn, &stChnAttr));
		
		MI_VENC_InputSourceConfig_t stVencSourceCfg;
		if(pstStreamAttr[u32VencChn].stVencInBindParam.eBindType == E_MI_SYS_BIND_TYPE_HW_RING)
		{
			if(pstStreamAttr[u32VencChn].stVencInBindParam.u32BindParam == 0 || pstStreamAttr[u32VencChn].stVencInBindParam.u32BindParam==pstStreamAttr[u32VencChn].u32Height)
			{
				pstStreamAttr[u32VencChn].stVencInBindParam.u32BindParam = pstStreamAttr[u32VencChn].u32Height;
				stVencSourceCfg.eInputSrcBufferMode = E_MI_VENC_INPUT_MODE_RING_ONE_FRM;
			}
			else if(pstStreamAttr[u32VencChn].stVencInBindParam.u32BindParam==pstStreamAttr[u32VencChn].u32Height/2)
			{
				stVencSourceCfg.eInputSrcBufferMode = E_MI_VENC_INPUT_MODE_RING_HALF_FRM;
			}
			else
			{
				printf("venc dev%d chn%d bindtype %d bindparam %d stream height %d err \n", DevId, u32VencChn, pstStreamAttr[u32VencChn].stVencInBindParam.eBindType, pstStreamAttr[u32VencChn].stVencInBindParam.u32BindParam,
					pstStreamAttr[u32VencChn].u32Height);
			}
		}
		else
		{
			stVencSourceCfg.eInputSrcBufferMode = E_MI_VENC_INPUT_MODE_NORMAL_FRMBASE;
		}
		MI_VENC_SetInputSourceConfig(DevId, pstStreamAttr[u32VencChn].vencChn, &stVencSourceCfg);
	
		STCHECKRESULT(ST_Venc_StartChannel(DevId, pstStreamAttr[u32VencChn].vencChn));
	
		ST_Sys_BindInfo_T stBindInfo;
		memset(&stBindInfo, 0x0, sizeof(ST_Sys_BindInfo_T));
	
		//step4
		/*
		MI_U32 u32DevId = -1;
		ExecFunc(MI_VENC_GetChnDevid(u32VencChn, &u32DevId), MI_SUCCESS);*/
		stBindInfo.stSrcChnPort.eModId = eBindModule;
		stBindInfo.stSrcChnPort.u32DevId = u32BindDevId;
		stBindInfo.stSrcChnPort.u32ChnId = u32BindChn;
		stBindInfo.stSrcChnPort.u32PortId = u32BindPort;
	
		stBindInfo.stDstChnPort.eModId = E_MI_MODULE_ID_VENC;
		stBindInfo.stDstChnPort.u32DevId = DevId;
		stBindInfo.stDstChnPort.u32ChnId = pstStreamAttr[u32VencChn].vencChn;
		stBindInfo.stDstChnPort.u32PortId = 0;
	
		stBindInfo.u32SrcFrmrate = 30;
		stBindInfo.u32DstFrmrate = 15;
		stBindInfo.eBindType = pstStreamAttr[u32VencChn].stVencInBindParam.eBindType;
		//stBindInfo.u32BindParam = pstStreamAttr[u32VencChn].stVencInBindParam.u32BindParam;
		STCHECKRESULT(ST_Sys_Bind(&stBindInfo));
	
		printf("VENC Chn%d Start SUCCESS\n", pstStreamAttr[u32VencChn].vencChn);
		
		return MI_SUCCESS;

}

MI_S32 VencModuleDeInit(MI_U32 u32VencChn)
{
    
    ST_VencAttr_t *pstStreamAttr = gstVencattr;
	MI_U32 u32vencChn = pstStreamAttr[u32VencChn].vencChn;



    STCHECKRESULT(ST_Venc_StopChannel(pstStreamAttr[u32VencChn].DevId, u32vencChn));
    STCHECKRESULT(ST_Venc_DestoryChannel(pstStreamAttr[u32VencChn].DevId,u32vencChn));

    return MI_SUCCESS;
}


void construct_disp_module(MI_DISP_OutputTiming_e         IntfSync,MI_U32  Width,MI_U32 Height)
{
    MI_DISP_DEV         DispDev = 0;
    MI_DISP_LAYER       DispLayer = 0;
    MI_DISP_INPUTPORT   DispInport = 0;

    MI_DISP_PubAttr_t stDispPubAttr;
    memset(&stDispPubAttr, 0, sizeof(MI_DISP_PubAttr_t));
    stDispPubAttr.u32BgColor = YUYV_BLACK;
    stDispPubAttr.eIntfSync = IntfSync;
    stDispPubAttr.eIntfType = E_MI_DISP_INTF_HDMI;
    MI_DISP_SetPubAttr(DispDev, &stDispPubAttr);
    info("---->MI_DISP_SetPubAttr\n");

    MI_DISP_Enable(DispDev);
    info("---->MI_DISP_Enable\n");

    MI_DISP_VideoLayerAttr_t stLayerAttr;
    memset(&stLayerAttr, 0, sizeof(MI_DISP_VideoLayerAttr_t));
    stLayerAttr.stVidLayerSize.u16Width = Width;
    stLayerAttr.stVidLayerSize.u16Height = Height;
    stLayerAttr.stVidLayerDispWin.u16X = 0;
    stLayerAttr.stVidLayerDispWin.u16Y = 0;
    stLayerAttr.stVidLayerDispWin.u16Width = Width;
    stLayerAttr.stVidLayerDispWin.u16Height = Height;
    MI_DISP_BindVideoLayer(DispLayer, DispDev);
    info("---->MI_DISP_BindVideoLayer\n");

    MI_DISP_SetVideoLayerAttr(DispLayer, &stLayerAttr);
    info("---->MI_DISP_SetVideoLayerAttr\n");

    MI_DISP_EnableVideoLayer(DispLayer);
    info("---->MI_DISP_EnableVideoLayer\n");

    MI_DISP_InputPortAttr_t stInputPortAttr;

    stInputPortAttr.u16SrcWidth = Width;
    stInputPortAttr.u16SrcHeight = Height;
    stInputPortAttr.stDispWin.u16X = 0;
    stInputPortAttr.stDispWin.u16Y = 0;
    stInputPortAttr.stDispWin.u16Width = Width;
    stInputPortAttr.stDispWin.u16Height = Height;
    stInputPortAttr.eDecompressMode = E_MI_SYS_COMPRESS_MODE_NONE;
    MI_DISP_SetInputPortAttr(DispLayer, DispInport, &stInputPortAttr);
    info("---->MI_DISP_SetInputPortAttr\n");

    MI_DISP_EnableInputPort(DispLayer, DispInport);
    info("---->MI_DISP_EnableInputPort\n");

    info("---------------->construct_disp_module\n\n");
}


static MI_BOOL disp_ut_hdmi_init(MI_HDMI_TimingType_e eTimingType)
{
    MI_HDMI_TimingType_e eHdmiTiming = eTimingType;
    MI_HDMI_Attr_t stHdmiAttr;
    MI_HDMI_DeviceId_e eHdmiDefaultDevId = E_MI_HDMI_ID_0;
    MI_HDMI_InitParam_t stInitParam;

    memset(&stInitParam, 0, sizeof(MI_HDMI_InitParam_t));
    memset(&stHdmiAttr, 0, sizeof(stHdmiAttr));

    stInitParam.pCallBackArgs = NULL;
    stInitParam.pfnHdmiEventCallback = Hdmi_callback;
    DISP_UT_RESULT_CHECK(MI_HDMI_Init(&stInitParam));

    stHdmiAttr.stEnInfoFrame.bEnableAudInfoFrame = TRUE;
    stHdmiAttr.stEnInfoFrame.bEnableAviInfoFrame = TRUE;
    stHdmiAttr.stEnInfoFrame.bEnableSpdInfoFrame = TRUE;
    stHdmiAttr.stAudioAttr.bEnableAudio = TRUE;
    stHdmiAttr.stAudioAttr.bIsMultiChannel = 0;
    stHdmiAttr.stAudioAttr.eBitDepth = E_MI_HDMI_BIT_DEPTH_16;
    stHdmiAttr.stAudioAttr.eCodeType = E_MI_HDMI_ACODE_PCM;
    stHdmiAttr.stAudioAttr.eSampleRate = E_MI_HDMI_AUDIO_SAMPLERATE_48K;
    stHdmiAttr.stVideoAttr.bEnableVideo = TRUE;
    stHdmiAttr.stVideoAttr.eColorType = E_MI_HDMI_COLOR_TYPE_YCBCR444;//default color type
    //stHdmiAttr.stVideoAttr.eInColorType = E_MI_HDMI_COLOR_TYPE_YCBCR444;//default color type
    stHdmiAttr.stVideoAttr.eDeepColorMode = E_MI_HDMI_DEEP_COLOR_30BIT;
    stHdmiAttr.stVideoAttr.eTimingType = eTimingType;
    stHdmiAttr.stVideoAttr.eOutputMode = E_MI_HDMI_OUTPUT_MODE_HDMI;
    DISP_UT_RESULT_CHECK(MI_HDMI_Open(eHdmiDefaultDevId));
    DISP_UT_RESULT_CHECK(MI_HDMI_SetAttr(eHdmiDefaultDevId, &stHdmiAttr));
    DISP_UT_RESULT_CHECK(MI_HDMI_Start(eHdmiDefaultDevId));
    info("hdmi init as default!\n\n");

    info("---------------->disp_ut_hdmi_init\n\n");
    return MI_SUCCESS;
}

void Bind_isp_scl_vdisp_scl_disp(void) {
		MI_U16 u16SocId = 0;
	
	
		MI_SYS_BindAttr_t bindAttr;
		bindAttr.eBindType = E_MI_SYS_BIND_TYPE_FRAME_BASE;
	
		MI_SYS_ChnPort_t stSrcChnPort;
		MI_SYS_ChnPort_t stDstChnPort;
	

		stSrcChnPort.eModId = E_MI_MODULE_ID_ISP;
		stSrcChnPort.u32ChnId = 0;
		stSrcChnPort.u32DevId = 0;
		stSrcChnPort.u32PortId = 1;
		stDstChnPort.eModId = E_MI_MODULE_ID_SCL;
		stDstChnPort.u32ChnId = 0;
		stDstChnPort.u32DevId = 1;
		stDstChnPort.u32PortId = 0;
		MI_SYS_BindChnPort2(u16SocId, &stSrcChnPort, &stDstChnPort, 30, 30, E_MI_SYS_BIND_TYPE_FRAME_BASE, 0);
		debug("-->MI_SYS_BindChnPort isp%d[dev:%d chn:%d] -> scl%d [dev:%d chn:%d InPort:0]\n",
			0, 0, 0,0,
			0, 0);
	
	
		stSrcChnPort.eModId = E_MI_MODULE_ID_ISP;
		stSrcChnPort.u32ChnId = 1;
		stSrcChnPort.u32DevId = 0;
		stSrcChnPort.u32PortId = 1;
		stDstChnPort.eModId = E_MI_MODULE_ID_SCL;
		stDstChnPort.u32ChnId = 1;
		stDstChnPort.u32DevId = 1;
		stDstChnPort.u32PortId = 0;
		MI_SYS_BindChnPort2(u16SocId, &stSrcChnPort, &stDstChnPort, 30, 30, E_MI_SYS_BIND_TYPE_FRAME_BASE, 0);
	
		debug("-->MI_SYS_BindChnPort isp%d[dev:%d chn:%d] -> scl%d [dev:%d chn:%d InPort:0]\n",
			0, 0, 1,0,
			0, 1);
	
	
		stSrcChnPort.eModId = E_MI_MODULE_ID_ISP;
		stSrcChnPort.u32ChnId = 2;
		stSrcChnPort.u32DevId = 0;
		stSrcChnPort.u32PortId = 1;
		stDstChnPort.eModId = E_MI_MODULE_ID_SCL;
		stDstChnPort.u32ChnId = 0;
		stDstChnPort.u32DevId = 5;
		stDstChnPort.u32PortId = 0;
		MI_SYS_BindChnPort2(u16SocId, &stSrcChnPort, &stDstChnPort, 30, 30, E_MI_SYS_BIND_TYPE_FRAME_BASE, 0);
		debug("-->MI_SYS_BindChnPort isp%d[dev:%d chn:%d] -> scl%d [dev:%d chn:%d InPort:0]\n",
			1, 1, 0,4,
			4, 0);
	
		stSrcChnPort.eModId = E_MI_MODULE_ID_ISP;
		stSrcChnPort.u32ChnId = 3;
		stSrcChnPort.u32DevId = 0;
		stSrcChnPort.u32PortId = 1;
		stDstChnPort.eModId = E_MI_MODULE_ID_SCL;
		stDstChnPort.u32ChnId = 1;
		stDstChnPort.u32DevId = 5;
		stDstChnPort.u32PortId = 0;
		MI_SYS_BindChnPort2(u16SocId, &stSrcChnPort, &stDstChnPort, 30, 30, E_MI_SYS_BIND_TYPE_FRAME_BASE, 0);
	
		debug("-->MI_SYS_BindChnPort isp%d[dev:%d chn:%d] -> scl%d [dev:%d chn:%d InPort:0]\n",
			1, 1, 1,4,
			4, 1);
	


	
	
		

		stSrcChnPort.eModId = E_MI_MODULE_ID_SCL;
		stSrcChnPort.u32ChnId = 0;
		stSrcChnPort.u32DevId = 1;
		stSrcChnPort.u32PortId = 0;
		stDstChnPort.eModId = E_MI_MODULE_ID_VDISP;
		stDstChnPort.u32ChnId = 16;
		stDstChnPort.u32DevId = 0;
		stDstChnPort.u32PortId = 0;
	
		MI_SYS_BindChnPort2(u16SocId, &stSrcChnPort, &stDstChnPort, 30, 30, E_MI_SYS_BIND_TYPE_FRAME_BASE, 0);
		debug("-->MI_SYS_BindChnPort scl%d VeDev%d VeChn%d port%d-> disp%d VeDev%d VeChn%d\n", 0, 0, 0, 0, 0, 0, 0);
	
		stSrcChnPort.eModId = E_MI_MODULE_ID_SCL;
		stSrcChnPort.u32ChnId = 1;
		stSrcChnPort.u32DevId = 1;
		stSrcChnPort.u32PortId = 1;
		stDstChnPort.eModId = E_MI_MODULE_ID_VDISP;
		stDstChnPort.u32ChnId = 17;
		stDstChnPort.u32DevId = 0;
		stDstChnPort.u32PortId = 0;
	
		MI_SYS_BindChnPort2(u16SocId, &stSrcChnPort, &stDstChnPort, 30, 30, E_MI_SYS_BIND_TYPE_FRAME_BASE, 0);
		debug("-->MI_SYS_BindChnPort scl%d VeDev%d VeChn%d port%d-> disp%d VeDev%d VeChn%d\n", 0, 0, 1, 0, 0, 0, 1);
	
		stSrcChnPort.eModId = E_MI_MODULE_ID_SCL;
		stSrcChnPort.u32ChnId = 0;
		stSrcChnPort.u32DevId = 5;
		stSrcChnPort.u32PortId = 0;
		stDstChnPort.eModId = E_MI_MODULE_ID_VDISP;
		stDstChnPort.u32ChnId = 18;
		stDstChnPort.u32DevId = 0;
		stDstChnPort.u32PortId = 0;
	
		MI_SYS_BindChnPort2(u16SocId, &stSrcChnPort, &stDstChnPort, 30, 30, E_MI_SYS_BIND_TYPE_FRAME_BASE, 0);
		debug("-->MI_SYS_BindChnPort scl%d VeDev%d VeChn%d port%d-> disp%d VeDev%d VeChn%d\n", 4, 4, 0, 0, 0, 0, 2);
	
		stSrcChnPort.eModId = E_MI_MODULE_ID_SCL;
		stSrcChnPort.u32ChnId = 1;
		stSrcChnPort.u32DevId = 5;
		stSrcChnPort.u32PortId = 1;
		stDstChnPort.eModId = E_MI_MODULE_ID_VDISP;
		stDstChnPort.u32ChnId = 19;
		stDstChnPort.u32DevId = 0;
		stDstChnPort.u32PortId = 0;
	
		MI_SYS_BindChnPort2(u16SocId, &stSrcChnPort, &stDstChnPort, 30, 30, E_MI_SYS_BIND_TYPE_FRAME_BASE, 0);
		debug("-->MI_SYS_BindChnPort scl%d VeDev%d VeChn%d port%d-> disp%d VeDev%d VeChn%d\n", 4, 4, 1, 0, 0, 0, 3);



		stSrcChnPort.eModId = E_MI_MODULE_ID_VDISP;
		stSrcChnPort.u32ChnId = 0;
		stSrcChnPort.u32DevId = 0;
		stSrcChnPort.u32PortId = 0;
		stDstChnPort.eModId = E_MI_MODULE_ID_SCL;
		stDstChnPort.u32ChnId = 0;
		stDstChnPort.u32DevId = 3;
		stDstChnPort.u32PortId = 0;
		
		MI_SYS_BindChnPort2(u16SocId, &stSrcChnPort, &stDstChnPort, 30, 30, E_MI_SYS_BIND_TYPE_FRAME_BASE, 0);


		 stSrcChnPort.eModId = E_MI_MODULE_ID_SCL;
		 stSrcChnPort.u32ChnId = 0;
		 stSrcChnPort.u32DevId = 3;
		 stSrcChnPort.u32PortId = 0;
		 stDstChnPort.eModId = E_MI_MODULE_ID_DISP;
		 stDstChnPort.u32ChnId = 0;
		 stDstChnPort.u32DevId = 0;
		 stDstChnPort.u32PortId = 0;
	
		 MI_SYS_BindChnPort2(u16SocId, &stSrcChnPort, &stDstChnPort, 30, 30, E_MI_SYS_BIND_TYPE_FRAME_BASE, 0);
		 debug("-->MI_SYS_BindChnPort scl%d VeDev%d VeChn%d port%d-> disp%d VeDev%d VeChn%d\n", 0, 0, 0, 0, 0, 0, 0);
	


}

void unBind_isp_scl_vdisp_scl_disp(void) {
    MI_U16 u16SocId = 0;

    MI_SYS_ChnPort_t stSrcChnPort;
    MI_SYS_ChnPort_t stDstChnPort;    
    


		stSrcChnPort.eModId = E_MI_MODULE_ID_SCL;
		stSrcChnPort.u32ChnId = 0;
		stSrcChnPort.u32DevId = 3;
		stSrcChnPort.u32PortId = 0;
		stDstChnPort.eModId = E_MI_MODULE_ID_DISP;
		stDstChnPort.u32ChnId = 0;
		stDstChnPort.u32DevId = 0;
		stDstChnPort.u32PortId = 0;
		
		MI_SYS_UnBindChnPort(u16SocId, &stSrcChnPort, &stDstChnPort);

		stSrcChnPort.eModId = E_MI_MODULE_ID_VDISP;
		stSrcChnPort.u32ChnId = 0;
		stSrcChnPort.u32DevId = 0;
		stSrcChnPort.u32PortId = 0;
		stDstChnPort.eModId = E_MI_MODULE_ID_SCL;
		stDstChnPort.u32ChnId = 0;
		stDstChnPort.u32DevId = 3;
		stDstChnPort.u32PortId = 0;
		
		MI_SYS_UnBindChnPort(u16SocId, &stSrcChnPort, &stDstChnPort);

		stSrcChnPort.eModId = E_MI_MODULE_ID_SCL;
		stSrcChnPort.u32ChnId = 0;
		stSrcChnPort.u32DevId = 1;
		stSrcChnPort.u32PortId = 0;
		stDstChnPort.eModId = E_MI_MODULE_ID_VDISP;
		stDstChnPort.u32ChnId = 16;
		stDstChnPort.u32DevId = 0;
		stDstChnPort.u32PortId = 0;

		MI_SYS_UnBindChnPort(u16SocId, &stSrcChnPort, &stDstChnPort);

		stSrcChnPort.eModId = E_MI_MODULE_ID_SCL;
		stSrcChnPort.u32ChnId = 1;
		stSrcChnPort.u32DevId = 1;
		stSrcChnPort.u32PortId = 1;
		stDstChnPort.eModId = E_MI_MODULE_ID_VDISP;
		stDstChnPort.u32ChnId = 17;
		stDstChnPort.u32DevId = 0;
		stDstChnPort.u32PortId = 0;

		MI_SYS_UnBindChnPort(u16SocId, &stSrcChnPort, &stDstChnPort);

		stSrcChnPort.eModId = E_MI_MODULE_ID_SCL;
		stSrcChnPort.u32ChnId = 0;
		stSrcChnPort.u32DevId = 5;
		stSrcChnPort.u32PortId = 0;
		stDstChnPort.eModId = E_MI_MODULE_ID_VDISP;
		stDstChnPort.u32ChnId = 18;
		stDstChnPort.u32DevId = 0;
		stDstChnPort.u32PortId = 0;

		MI_SYS_UnBindChnPort(u16SocId, &stSrcChnPort, &stDstChnPort);

		stSrcChnPort.eModId = E_MI_MODULE_ID_SCL;
		stSrcChnPort.u32ChnId = 1;
		stSrcChnPort.u32DevId = 5;
		stSrcChnPort.u32PortId = 1;
		stDstChnPort.eModId = E_MI_MODULE_ID_VDISP;
		stDstChnPort.u32ChnId = 19;
		stDstChnPort.u32DevId = 0;
		stDstChnPort.u32PortId = 0;

		MI_SYS_UnBindChnPort(u16SocId, &stSrcChnPort, &stDstChnPort);






	stSrcChnPort.eModId = E_MI_MODULE_ID_ISP;
	stSrcChnPort.u32ChnId = 0;
	stSrcChnPort.u32DevId = 0;
	stSrcChnPort.u32PortId = 1;
	stDstChnPort.eModId = E_MI_MODULE_ID_SCL;
	stDstChnPort.u32ChnId = 0;
	stDstChnPort.u32DevId = 1;
	stDstChnPort.u32PortId = 0;
	MI_SYS_UnBindChnPort(u16SocId, &stSrcChnPort, &stDstChnPort);
	stSrcChnPort.eModId = E_MI_MODULE_ID_ISP;
	stSrcChnPort.u32ChnId = 1;
	stSrcChnPort.u32DevId = 0;
	stSrcChnPort.u32PortId = 1;
	stDstChnPort.eModId = E_MI_MODULE_ID_SCL;
	stDstChnPort.u32ChnId = 1;
	stDstChnPort.u32DevId = 1;
	stDstChnPort.u32PortId = 0;
	MI_SYS_UnBindChnPort(u16SocId, &stSrcChnPort, &stDstChnPort);
	
	stSrcChnPort.eModId = E_MI_MODULE_ID_ISP;
	stSrcChnPort.u32ChnId = 2;
	stSrcChnPort.u32DevId = 0;
	stSrcChnPort.u32PortId = 1;
	stDstChnPort.eModId = E_MI_MODULE_ID_SCL;
	stDstChnPort.u32ChnId = 0;
	stDstChnPort.u32DevId = 5;
	stDstChnPort.u32PortId = 0;
	MI_SYS_UnBindChnPort(u16SocId, &stSrcChnPort, &stDstChnPort);
	stSrcChnPort.eModId = E_MI_MODULE_ID_ISP;
	stSrcChnPort.u32ChnId = 3;
	stSrcChnPort.u32DevId = 0;
	stSrcChnPort.u32PortId = 1;
	stDstChnPort.eModId = E_MI_MODULE_ID_SCL;
	stDstChnPort.u32ChnId = 1;
	stDstChnPort.u32DevId = 5;
	stDstChnPort.u32PortId = 0;
	MI_SYS_UnBindChnPort(u16SocId, &stSrcChnPort, &stDstChnPort);

	
	stSrcChnPort.eModId = E_MI_MODULE_ID_VIF;
	stSrcChnPort.u32ChnId = 0;
	stSrcChnPort.u32DevId = 0;
	stSrcChnPort.u32PortId = 0;
	stDstChnPort.eModId = E_MI_MODULE_ID_ISP;
	stDstChnPort.u32ChnId = 0;
	stDstChnPort.u32DevId = 0;
	stDstChnPort.u32PortId = 0;
	MI_SYS_UnBindChnPort(u16SocId, &stSrcChnPort, &stDstChnPort);
	stSrcChnPort.eModId = E_MI_MODULE_ID_VIF;
	stSrcChnPort.u32ChnId = 0;
	stSrcChnPort.u32DevId = 8;
	stSrcChnPort.u32PortId = 0;
	stDstChnPort.eModId = E_MI_MODULE_ID_ISP;
	stDstChnPort.u32ChnId = 1;
	stDstChnPort.u32DevId = 0;
	stDstChnPort.u32PortId = 0;
	MI_SYS_UnBindChnPort(u16SocId, &stSrcChnPort, &stDstChnPort);
	stSrcChnPort.eModId = E_MI_MODULE_ID_VIF;
	stSrcChnPort.u32ChnId = 0;
	stSrcChnPort.u32DevId = 16;
	stSrcChnPort.u32PortId = 0;
	stDstChnPort.eModId = E_MI_MODULE_ID_ISP;
	stDstChnPort.u32ChnId = 2;
	stDstChnPort.u32DevId = 0;
	stDstChnPort.u32PortId = 0;
	MI_SYS_UnBindChnPort(u16SocId, &stSrcChnPort, &stDstChnPort);
	stSrcChnPort.eModId = E_MI_MODULE_ID_VIF;
	stSrcChnPort.u32ChnId = 0;
	stSrcChnPort.u32DevId = 24;
	stSrcChnPort.u32PortId = 0;
	stDstChnPort.eModId = E_MI_MODULE_ID_ISP;
	stDstChnPort.u32ChnId = 3;
	stDstChnPort.u32DevId = 0;
	stDstChnPort.u32PortId = 0;
	MI_SYS_UnBindChnPort(u16SocId, &stSrcChnPort, &stDstChnPort);


 
    info("---------------->unbind_disp_scl_isp_vif\n");
}

void unBind_isp_scl_venc(void) {

	MI_U16 u16SocId = 0;

	MI_SYS_ChnPort_t stSrcChnPort;
	MI_SYS_ChnPort_t stDstChnPort;	  
	stSrcChnPort.eModId = E_MI_MODULE_ID_SCL;
	stSrcChnPort.u32ChnId = 0;
	stSrcChnPort.u32DevId = 1;
	stSrcChnPort.u32PortId = 0;
	stDstChnPort.eModId = E_MI_MODULE_ID_VENC;
	stDstChnPort.u32ChnId = 0;
	stDstChnPort.u32DevId = 0;
	stDstChnPort.u32PortId = 0;
	MI_SYS_UnBindChnPort(u16SocId, &stSrcChnPort, &stDstChnPort);
	stSrcChnPort.eModId = E_MI_MODULE_ID_SCL;
	stSrcChnPort.u32ChnId = 1;
	stSrcChnPort.u32DevId = 1;
	stSrcChnPort.u32PortId = 1;
	stDstChnPort.eModId = E_MI_MODULE_ID_VENC;
	stDstChnPort.u32ChnId = 1;
	stDstChnPort.u32DevId = 0;
	stDstChnPort.u32PortId = 0;
	MI_SYS_UnBindChnPort(u16SocId, &stSrcChnPort, &stDstChnPort);

	stSrcChnPort.eModId = E_MI_MODULE_ID_SCL;
	stSrcChnPort.u32ChnId = 0;
	stSrcChnPort.u32DevId = 5;
	stSrcChnPort.u32PortId = 0;
	stDstChnPort.eModId = E_MI_MODULE_ID_VENC;
	stDstChnPort.u32ChnId = 0;
	stDstChnPort.u32DevId = 1;
	stDstChnPort.u32PortId = 0;
	MI_SYS_UnBindChnPort(u16SocId, &stSrcChnPort, &stDstChnPort);

	stSrcChnPort.eModId = E_MI_MODULE_ID_SCL;
	stSrcChnPort.u32ChnId = 1;
	stSrcChnPort.u32DevId = 5;
	stSrcChnPort.u32PortId = 1;
	stDstChnPort.eModId = E_MI_MODULE_ID_VENC;
	stDstChnPort.u32ChnId = 1;
	stDstChnPort.u32DevId = 1;
	stDstChnPort.u32PortId = 0;
	MI_SYS_UnBindChnPort(u16SocId, &stSrcChnPort, &stDstChnPort);



	stSrcChnPort.eModId = E_MI_MODULE_ID_ISP;
	stSrcChnPort.u32ChnId = 0;
	stSrcChnPort.u32DevId = 0;
	stSrcChnPort.u32PortId = 1;
	stDstChnPort.eModId = E_MI_MODULE_ID_SCL;
	stDstChnPort.u32ChnId = 0;
	stDstChnPort.u32DevId = 1;
	stDstChnPort.u32PortId = 0;
	MI_SYS_UnBindChnPort(u16SocId, &stSrcChnPort, &stDstChnPort);
	stSrcChnPort.eModId = E_MI_MODULE_ID_ISP;
	stSrcChnPort.u32ChnId = 1;
	stSrcChnPort.u32DevId = 0;
	stSrcChnPort.u32PortId = 1;
	stDstChnPort.eModId = E_MI_MODULE_ID_SCL;
	stDstChnPort.u32ChnId = 1;
	stDstChnPort.u32DevId = 1;
	stDstChnPort.u32PortId = 0;
	MI_SYS_UnBindChnPort(u16SocId, &stSrcChnPort, &stDstChnPort);

	stSrcChnPort.eModId = E_MI_MODULE_ID_ISP;
	stSrcChnPort.u32ChnId = 2;
	stSrcChnPort.u32DevId = 0;
	stSrcChnPort.u32PortId = 1;
	stDstChnPort.eModId = E_MI_MODULE_ID_SCL;
	stDstChnPort.u32ChnId = 0;
	stDstChnPort.u32DevId = 5;
	stDstChnPort.u32PortId = 0;
	MI_SYS_UnBindChnPort(u16SocId, &stSrcChnPort, &stDstChnPort);
	stSrcChnPort.eModId = E_MI_MODULE_ID_ISP;
	stSrcChnPort.u32ChnId = 3;
	stSrcChnPort.u32DevId = 0;
	stSrcChnPort.u32PortId = 1;
	stDstChnPort.eModId = E_MI_MODULE_ID_SCL;
	stDstChnPort.u32ChnId = 1;
	stDstChnPort.u32DevId = 5;
	stDstChnPort.u32PortId = 0;
	MI_SYS_UnBindChnPort(u16SocId, &stSrcChnPort, &stDstChnPort);


	stSrcChnPort.eModId = E_MI_MODULE_ID_VIF;
	stSrcChnPort.u32ChnId = 0;
	stSrcChnPort.u32DevId = 0;
	stSrcChnPort.u32PortId = 0;
	stDstChnPort.eModId = E_MI_MODULE_ID_ISP;
	stDstChnPort.u32ChnId = 0;
	stDstChnPort.u32DevId = 0;
	stDstChnPort.u32PortId = 0;
	MI_SYS_UnBindChnPort(u16SocId, &stSrcChnPort, &stDstChnPort);
	stSrcChnPort.eModId = E_MI_MODULE_ID_VIF;
	stSrcChnPort.u32ChnId = 0;
	stSrcChnPort.u32DevId = 8;
	stSrcChnPort.u32PortId = 0;
	stDstChnPort.eModId = E_MI_MODULE_ID_ISP;
	stDstChnPort.u32ChnId = 1;
	stDstChnPort.u32DevId = 0;
	stDstChnPort.u32PortId = 0;
	MI_SYS_UnBindChnPort(u16SocId, &stSrcChnPort, &stDstChnPort);
	stSrcChnPort.eModId = E_MI_MODULE_ID_VIF;
	stSrcChnPort.u32ChnId = 0;
	stSrcChnPort.u32DevId = 16;
	stSrcChnPort.u32PortId = 0;
	stDstChnPort.eModId = E_MI_MODULE_ID_ISP;
	stDstChnPort.u32ChnId = 2;
	stDstChnPort.u32DevId = 0;
	stDstChnPort.u32PortId = 0;
	MI_SYS_UnBindChnPort(u16SocId, &stSrcChnPort, &stDstChnPort);
	stSrcChnPort.eModId = E_MI_MODULE_ID_VIF;
	stSrcChnPort.u32ChnId = 0;
	stSrcChnPort.u32DevId = 24;
	stSrcChnPort.u32PortId = 0;
	stDstChnPort.eModId = E_MI_MODULE_ID_ISP;
	stDstChnPort.u32ChnId = 3;
	stDstChnPort.u32DevId = 0;
	stDstChnPort.u32PortId = 0;
	MI_SYS_UnBindChnPort(u16SocId, &stSrcChnPort, &stDstChnPort);


}

static MI_BOOL disp_ut_hdmi_deinit(void)
{
    MI_HDMI_DeviceId_e eHdmiDefaultDevId = E_MI_HDMI_ID_0;

    {
        DISP_UT_RESULT_CHECK(MI_HDMI_Stop(eHdmiDefaultDevId));
        DISP_UT_RESULT_CHECK(MI_HDMI_Close(eHdmiDefaultDevId));
        DISP_UT_RESULT_CHECK(MI_HDMI_DeInit());
    }


    info("---------------->disp_ut_hdmi_deinit\n");
    return MI_SUCCESS;
}

void destruct_disp_module(void)
{
    MI_DISP_DEV         DispDev = 0;
    MI_DISP_LAYER       DispLayer = 0;
    MI_DISP_INPUTPORT   DispInport = 0;

    MI_DISP_DisableInputPort(DispLayer, DispInport);
    MI_DISP_DisableVideoLayer(DispLayer);
    MI_DISP_UnBindVideoLayer(DispLayer, DispDev);
    MI_DISP_Disable(DispDev);

    info("---->destruct_disp_module\n");
}

void destruct_vdisp_module(void) {
    MI_VDISP_DEV DevId = 0;
    MI_VDISP_CHN Chn_0_Id = 16;
    MI_VDISP_CHN Chn_1_Id = 17;
    MI_VDISP_CHN Chn_2_Id = 18;
    MI_VDISP_CHN Chn_3_Id = 19;

    //pthread_join(thr3, NULL);

    /*
     * 先禁用已经打开 vdisp channel
     * <VDISP_OVERLAYINPUTCHNID,1,2>
     */
    MI_VDISP_DisableInputChannel(DevId, Chn_0_Id);
    MI_VDISP_DisableInputChannel(DevId, Chn_1_Id);
    MI_VDISP_DisableInputChannel(DevId, Chn_2_Id);
    MI_VDISP_DisableInputChannel(DevId, Chn_3_Id);

    /*
     *停止已经打开的 vdisp 设备
     */
    MI_VDISP_StopDev(DevId);
    /*
     *关闭已经打开的 vdisp 设备
     */
    MI_VDISP_CloseDevice(DevId);
    /*
     *退出 vdisp 模块
     */
    if (NeedDeinitVDISP)
    {
        MI_VDISP_Exit();
    }

    info("---->destruct_vdisp_module\n");
}
void destruct_scl_module(void)
{


    for (auto i = 0; i < 5; i++)
    {
        info("--->StDeinit i = %d [dev:%d, chn:%d, outPort:%d]\n", i, sclNode[i].u32DevId, sclNode[i].u32ChnId, sclNode[i].u32OutPort);
        sclNode[i].StDeinit();
    }

    info("---->destruct_scl_module\n");
}

void destruct_scl_rtsp_module(void)
{


    for (auto i = 0; i < 4; i++)
    {
        info("--->StDeinit i = %d [dev:%d, chn:%d, outPort:%d]\n", i, sclNode[i].u32DevId, sclNode[i].u32ChnId, sclNode[i].u32OutPort);
        sclNode[i].StDeinit();
    }

    info("---->destruct_scl_module\n");
}

MI_S32 IspModuleUnInit(MI_ISP_DEV IspDevId)
{
    MI_S32 s32Ret = MI_SUCCESS;
    MI_ISP_CHANNEL IspChnId = 0;
    MI_ISP_PORT  IspOutPortId =1;

	for(IspChnId=0; IspChnId<4; IspChnId++)
	{
    		ExecFuncResult(MI_ISP_DisableOutputPort(IspDevId, IspChnId,IspOutPortId), s32Ret);
	}
	
    for(IspChnId=0; IspChnId<4; IspChnId++)
    {
   
            ExecFuncResult(MI_ISP_StopChannel(IspDevId, IspChnId), s32Ret);
            ExecFuncResult(MI_ISP_DestroyChannel(IspDevId, IspChnId), s32Ret);

    }
    
    ExecFuncResult(MI_ISP_DestoryDevice(IspDevId), s32Ret);

EXIT:

    return s32Ret;
}

int main(int argc, char **argv)
{
    MI_U8  i=0;
    MI_S32 s32Ret = MI_SUCCESS;
#if USER_SENSOR_SUPPORT
    pthread_t ptCheck9931;
#endif
    struct timeval stTestBeginTime, stTestEndTime;
/*
    struct rlimit limit;
    limit.rlim_cur = RLIM_INFINITY;
    limit.rlim_max = RLIM_INFINITY;
    setrlimit(RLIMIT_CORE, &limit);
*/
    gettimeofday(&stTestBeginTime, NULL);
    ST_InitParam();

    DBG_INFO("dynamic change func use ini name dynamic_change.ini \n");

    pthread_mutex_init(&gIniMd5Mutex, NULL);
    gMd5Action = E_ST_MD5_ACTION_NONE;
    INIT_LIST_HEAD(&gstDynamicTest.stDynamicFuncListHead);
    INIT_LIST_HEAD(&gstDynamicTest.stDumpFileListHead);
    for(i=1; i< argc; i++)
    {
        printf("inipath %s \n", argv[i]);

        if(strstr(argv[i], "dynamic_change.ini"))
        {
            ExecFuncResult(ST_ParseDynamicTestIni(argv[i]), s32Ret);
        }
        else if(strstr(argv[i], "random_case.ini"))
        {
            ExecFuncResult(ST_ParseRandomTestIni(argv[i]), s32Ret);
        }
        else if(MI_SUCCESS != ST_ParserIni(argv[i]))
        {
            printf("parse init fail \n");
            return -1;
        }

        u32ChnNum++;
    }
	if(bUseDisp)
	{
	   MI_U32 u32VifGroup=0;
	   MI_U8 eSnrPad = 0;
	   MI_SYS_Init(0);
	   BaseModuleInit();
	   construct_scl_module();
	   construct_vdisp_module();
	   if(btest_1080)
	   {
	   construct_disp_module(E_MI_DISP_OUTPUT_1080P30,1920,1080);
	   disp_ut_hdmi_init(E_MI_HDMI_TIMING_1080_30P);
	   }
	   else
	   {
	   construct_disp_module(E_MI_DISP_OUTPUT_3840x2160_30,3840,2160);
	   disp_ut_hdmi_init(E_MI_HDMI_TIMING_4K2K_30P);
	   }
       Bind_isp_scl_vdisp_scl_disp();
#if 1
	   pthread_t pIQthread;
	   pthread_create(&pIQthread, NULL, ST_IQthread, NULL);
#endif


	   while(!bExit)
	   {
	   	   MI_U32 u32Select = 0xff;
		   printf("select 111: exit\n");
		   scanf("%d", &u32Select);
		   ST_Flush();
		   if(u32Select == 111)//change hdr res
		   {
	       bExit = TRUE;
		   }

	   }
	   usleep(THREAD_SLEEP_TIME_US);
	   unBind_isp_scl_vdisp_scl_disp();
	   disp_ut_hdmi_deinit();
       destruct_disp_module();
	   destruct_vdisp_module();
	   destruct_scl_module();
	   IspModuleUnInit(0);
	   for(u32VifGroup=0;u32VifGroup <ST_MAX_VIF_GROUP_NUM;u32VifGroup++){
	   ExecFuncResult(VifModuleUnInit((MI_VIF_GROUP)u32VifGroup), s32Ret);
	   }
	   for(eSnrPad=0; eSnrPad<ST_MAX_SENSOR_NUM; eSnrPad++){
	   ExecFuncResult(ST_SensorModuleUnInit((MI_SNR_PADID)eSnrPad), s32Ret);
	   }
	   MI_SYS_Exit(0);

	}
	else
	{
	int num;
    ExecFuncResult(ST_SaveValidParam(), s32Ret);
#if 0
    ExecFuncResult(ST_BaseModuleInit(), s32Ret);
#else
    MI_SYS_Init(0);
    ST_DefaultArgs();
	BaseModuleInit();
	construct_scl_rtsp_module();
	for(num=0;num<4;num++){
		VencModuleInit(num);
	}
#endif
    ST_RtspServerStart();


#if 1
        pthread_t pIQthread;
        pthread_create(&pIQthread, NULL, ST_IQthread, NULL);
#endif


    if(bDetectAD ==TRUE)
    {
#if USER_SENSOR_SUPPORT
        pthread_create(&ptCheck9931, NULL, ST_Check9931Status, NULL);
#endif
    }

    usleep(THREAD_SLEEP_TIME_US*50);

    if(!list_empty(&gstDynamicTest.stDynamicFuncListHead))
    {
        MI_U32 u32loopcnt = gstDynamicTest.u32ListRepeatCnt;
        MI_U32 u32cnt = 0;
        MI_BOOL bdump = FALSE;
        MI_S32 s32TotalCnt = u32loopcnt*(gstDynamicTest.u32ListTotalNodeCnt+1);
        MI_S32 s32BeginId = 0, s32EndId = s32TotalCnt-1, s32DynamicId = 0;
        if(gstDynamicTest.s32DynamicDumpCnt < 0)
        {
            s32BeginId = s32EndId + gstDynamicTest.s32DynamicDumpCnt + 1;
            if(s32BeginId < 0)
                s32BeginId = 0;
        }
        else
        {
            s32EndId = gstDynamicTest.s32DynamicDumpCnt - 1;
            if(s32EndId > s32TotalCnt-1)
                s32EndId = s32TotalCnt-1;
        }
        DBG_INFO("Dump file: dynamic id between %d~%d \n", s32BeginId, s32EndId);

        for(u32cnt = 0; u32cnt < u32loopcnt; u32cnt++)
        {
            MI_U32 u32Listcnt = gstDynamicTest.u32ListTotalNodeCnt;
            struct list_head *pos = NULL;
            struct list_head *n = NULL;
            list_for_each_safe(pos, n, &gstDynamicTest.stDynamicFuncListHead)
            {
                ST_DynamicFuncParam_t *pstDynamicFunc = NULL;
                pstDynamicFunc = container_of(pos, typeof(*pstDynamicFunc), stDynamicFuncListNode);

                DBG_INFO("[%s]:%d do dynamic ListRepeatCnt [%d-%d]  ListTotalNodeCnt %d\n", __FUNCTION__,__LINE__, u32loopcnt,u32cnt,u32Listcnt--);
                if(pstDynamicFunc != NULL)
                {
                    if(s32DynamicId >= s32BeginId && s32DynamicId <= s32EndId)
                    {
                        bdump = TRUE;
                    }
                    else
                    {
                        bdump = FALSE;
                    }
                    s32DynamicId++;

                    ExecFuncResult(ST_DoDynamicFunc(pstDynamicFunc, bdump), s32Ret);
                    if((u32cnt+1) == u32loopcnt)
                    {
                        list_del(&pstDynamicFunc->stDynamicFuncListNode);
                        free(pstDynamicFunc);
                    }
                }
            }
        }
    }

    if(!list_empty(&gstDynamicTest.stDumpFileListHead))
    {
        struct list_head *pos = NULL;
        struct list_head *n = NULL;
        list_for_each_safe(pos, n, &gstDynamicTest.stDumpFileListHead)
        {
            ST_DumpFileInfo_t *pstDumpFile = NULL;
            pstDumpFile = container_of(pos, typeof(*pstDumpFile), stDumpFileListNode);

            if(pstDumpFile != NULL)
            {
                list_del(&pstDumpFile->stDumpFileListNode);
                free(pstDumpFile);
            }
        }
    }

    for(i=0; i<ST_MAX_VENC_NUM;i++)
    {
        ST_VencAttr_t *pstStreamAttr = &gstVencattr[i];
        if(pstStreamAttr->bCreate ==TRUE
            && pstStreamAttr->bUsed==TRUE
            && pstStreamAttr->s32DumpBuffNum >0)
        {
           ExecFuncResult(ST_GetVencOut(i, pstStreamAttr->s32DumpBuffNum, pstStreamAttr->FilePath), s32Ret);
        }
    }

    if(u32TimeOutSExit >0)
    {
        usleep(1000*1000*u32TimeOutSExit);
        ST_WaitDumpFileDone();
        bExit=TRUE;
    }

    while(!bExit)
    {
        MI_U32 u32Select = 0xff;
        printf("select 0: VIF change Hdr res \n");
        printf("select 1: VIF change DevAttr\n");
        printf("select 2: VIF change Outputport attr\n");
        printf("select 3: VIF change Output FrameRate\n");
        printf("select 4: VIF change Output on/off\n");
        printf("select 5: ISP change Isp Rotate\n");
        printf("select 6: ISP change 3dnr Level\n");
        printf("select 7: ISP change Input Crop\n");
        printf("select 8: ISP change OutputParam\n");
        printf("select 9: ISP zoom\n");
        printf("select 10: ISP enable cus3a demo\n");
        printf("select 11: ISP set api bin\n");
        printf("select 12: ISP set skip frame\n");
        printf("select 13: SCL change rotate\n");
        printf("select 14: SCL change input Crop\n");
        printf("select 15: SCL change output param\n");
        printf("select 16: SCL stretch buffer\n");
        printf("select 17: Output step size test\n");
        printf("select 18: crop step size test\n");
        printf("select 19: Get Module output port buffer\n");
        printf("select 20: fbd file\n");
        printf("select 21: get venc out \n");
        printf("select 22: Convert bayer file to 16Bpp \n");
        printf("select 23: Set AD HalfMode \n");
        printf("select 24: hw reset \n");
        printf("select 25: VIF Dev switch OnOff \n");
        printf("select 26: DestroyCreate isp channel \n");
        printf("select 27: DestroyCreate scl channel \n");
        printf("select 28: CRC onoff \n");
        printf("select 29: Change chnport On/off \n");
        printf("select 30: VIF change GroupAttr(only for input pixel) \n");
        printf("select 31: Dh9931ControlMenu \n");
        printf("select 32: Dh9931ChangeSnrFormatRes \n");
        printf("select 33: Report 9931 Detect Value \n");
        printf("select 111: exit\n");

        scanf("%d", &u32Select);
        ST_Flush();
        if(u32Select == 0)//change hdr res
        {
            MI_VIF_GROUP GroupId = 0;
            MI_BOOL bUsedHdr = FALSE;
            MI_S32 s32ScanfTemp = 0;

            printf("select Group id:");
            scanf("%d", &s32ScanfTemp);
            ST_Flush();
            GroupId = (MI_VIF_GROUP)s32ScanfTemp;

            printf("select bUsedHdr:");
            scanf("%d", &s32ScanfTemp);
            ST_Flush();
            bUsedHdr = (MI_BOOL)s32ScanfTemp;

            ExecFuncResult(ST_DoChangeHdrRes(GroupId,bUsedHdr,-1), s32Ret);
        }
        else if(u32Select == 1) //change vif dev res
        {
#if MI_VIF_SUPPORT
            MI_S32 s32ScanfTemp = 0;
            MI_VIF_DEV VifDev = 0;
            MI_VIF_DevAttr_t stDevAttr;
            memset(&stDevAttr, 0x0, sizeof(MI_VIF_DevAttr_t));

            printf("VifDev:");
            scanf("%d", &s32ScanfTemp);
            ST_Flush();
            VifDev = (MI_VIF_DEV)s32ScanfTemp;

            ExecFuncResult(MI_VIF_GetDevAttr(VifDev,&stDevAttr), s32Ret);
            printf(" orgeField(%d)\n orgebEnH2T1PMode(%d)\n orgeInputPixel(%d)\n orgstInputRect(%d,%d,%d,%d)\n",
            stDevAttr.eField,stDevAttr.bEnH2T1PMode,stDevAttr.eInputPixel,
            stDevAttr.stInputRect.u16X,stDevAttr.stInputRect.u16Y,stDevAttr.stInputRect.u16Width,stDevAttr.stInputRect.u16Height);

            printf("eField:");
            scanf("%d",&s32ScanfTemp);
            ST_Flush();
            stDevAttr.eField = (MI_SYS_FieldType_e)s32ScanfTemp;

            printf("bEnH2T1PMode:");
            scanf("%d",&s32ScanfTemp);
            ST_Flush();
            stDevAttr.bEnH2T1PMode = (MI_BOOL)s32ScanfTemp;

            printf("stInputRect:\n");
            printf("x:");
            scanf("%d",&s32ScanfTemp);
            ST_Flush();
            stDevAttr.stInputRect.u16X = (MI_U16)s32ScanfTemp;

            printf("y:");
            scanf("%d",&s32ScanfTemp);
            ST_Flush();
            stDevAttr.stInputRect.u16Y = (MI_U16)s32ScanfTemp;

            printf("w:");
            scanf("%d",&s32ScanfTemp);
            ST_Flush();
            stDevAttr.stInputRect.u16Width = (MI_U16)s32ScanfTemp;

            printf("h:");
            scanf("%d",&s32ScanfTemp);
            ST_Flush();
            stDevAttr.stInputRect.u16Height = (MI_U16)s32ScanfTemp;

            printf("input, Field(%d)  bEnH2T1PMode(%d)\n stInputRect(%d,%d,%d,%d)\n",
            stDevAttr.eField,stDevAttr.bEnH2T1PMode,stDevAttr.stInputRect.u16X,
            stDevAttr.stInputRect.u16Y,stDevAttr.stInputRect.u16Width,stDevAttr.stInputRect.u16Height);

            ExecFuncResult(ST_DoChangeVifDevAttr(VifDev,&stDevAttr), s32Ret);
#endif
        }
        else if(u32Select == 2) //change vif port res
        {
#if MI_VIF_SUPPORT
            MI_S32 s32ScanfTemp = 0;
            MI_VIF_DEV VifDevId = 0;
            MI_VIF_PORT VifPortId = 0;
            MI_VIF_OutputPortAttr_t  stVifPortAttr;
            memset(&stVifPortAttr, 0x0, sizeof(MI_VIF_OutputPortAttr_t));

            printf("select dev id:");
            scanf("%d", &s32ScanfTemp);
            ST_Flush();
            VifDevId = (MI_VIF_DEV)s32ScanfTemp;

            printf("select port id:");
            scanf("%d", &s32ScanfTemp);
            ST_Flush();
            VifPortId = (MI_VIF_PORT)s32ScanfTemp;

            if(VifPortId == 1)
            {
                printf("crop x:");
                scanf("%d", &s32ScanfTemp);
                ST_Flush();
                stVifPortAttr.stCapRect.u16X = (MI_U16)s32ScanfTemp;

                printf("crop y:");
                scanf("%d", &s32ScanfTemp);
                ST_Flush();
                stVifPortAttr.stCapRect.u16Y = (MI_U16)s32ScanfTemp;

                printf("crop w:");
                scanf("%d", &s32ScanfTemp);
                ST_Flush();
                stVifPortAttr.stCapRect.u16Width = (MI_U16)s32ScanfTemp;

                printf("crop h:");
                scanf("%d", &s32ScanfTemp);
                ST_Flush();
                stVifPortAttr.stCapRect.u16Height = (MI_U16)s32ScanfTemp;

                printf("dest w:");
                scanf("%d", &s32ScanfTemp);
                ST_Flush();
                stVifPortAttr.stDestSize.u16Width = (MI_U16)s32ScanfTemp;

                printf("dest h:");
                scanf("%d", &s32ScanfTemp);
                ST_Flush();
                stVifPortAttr.stDestSize.u16Height = (MI_U16)s32ScanfTemp;
            }

            printf("pixel:");
            scanf("%d", &s32ScanfTemp);
            ST_Flush();
            stVifPortAttr.ePixFormat = (MI_SYS_PixelFormat_e)s32ScanfTemp;

            printf("FrameRate: 0 full , 1 half :");
            scanf("%d", &s32ScanfTemp);
            ST_Flush();
            stVifPortAttr.eFrameRate = (MI_VIF_FrameRate_e)s32ScanfTemp;

            printf("compress: (0:off 5:10to6 7:dsc)");
            scanf("%d", &s32ScanfTemp);
            ST_Flush();
            stVifPortAttr.eCompressMode = (MI_SYS_CompressMode_e)s32ScanfTemp;

            ExecFuncResult(ST_DoChangeVifPortAttr(VifDevId, VifPortId, &stVifPortAttr), s32Ret);
#endif
        }
        else if(u32Select == 3) // change vif output port frc
        {
            MI_S32 s32ScanfTemp = 0;
            MI_VIF_DEV VifDev = 0;
            MI_VIF_PORT VifPortId = 0;
            MI_U32 u32FrameRate = 0;

            printf("select Vif Dev id:");
            scanf("%d", &s32ScanfTemp);
            ST_Flush();
            VifDev = (MI_VIF_DEV)s32ScanfTemp;

            printf("select port id:");
            scanf("%d", &s32ScanfTemp);
            ST_Flush();
            VifPortId = (MI_VIF_PORT)s32ScanfTemp;

            printf("select FrameRate:");
            scanf("%d", &s32ScanfTemp);
            ST_Flush();
            u32FrameRate = (MI_U32)s32ScanfTemp;

            ExecFuncResult(ST_DoChangeVifOutputFrc(VifDev,VifPortId,u32FrameRate), s32Ret);
        }
        else if(u32Select == 4) // change vif output port on/off
        {
            MI_S32 s32ScanfTemp = 0;
            MI_VIF_DEV VifDev = 0;
            MI_VIF_PORT VifPortId = 0;
            MI_BOOL bOn = FALSE;

            printf("select Vif Dev id:");
            scanf("%d", &s32ScanfTemp);
            ST_Flush();
            VifDev = (MI_VIF_DEV)s32ScanfTemp;

            printf("select port id:");
            scanf("%d", &s32ScanfTemp);
            ST_Flush();
            VifPortId = (MI_VIF_PORT)s32ScanfTemp;

            printf("select Vif outputport on/off :");
            scanf("%d", &s32ScanfTemp);
            ST_Flush();
            bOn = (MI_BOOL)s32ScanfTemp;

            ExecFuncResult(ST_DoChangeVifOutPutOnOff(VifDev, VifPortId, bOn), s32Ret);
        }
        else if(u32Select == 5) // change isp rot
        {
            MI_S32 s32IspChn = 0;
            MI_S32 s32Rotation = 0;
            MI_S32 s32Mirror = 0;
            MI_S32 s32Flip = 0;

            printf("select ISP channel:");
            scanf("%d", &s32IspChn);
            ST_Flush();

            printf("rotation(0:0, 1:90, 2:180, 3:270):");
            scanf("%d", &s32Rotation);
            ST_Flush();

            printf("mirror 0: FALSE, 1:TRUE :");
            scanf("%d", &s32Mirror);
            ST_Flush();

            printf("Flip 0: FALSE, 1:TRUE :");
            scanf("%d", &s32Flip);
            ST_Flush();

            ExecFuncResult(ST_DoChangeIspRotate(0, (MI_ISP_CHANNEL)s32IspChn, (MI_SYS_Rotate_e)s32Rotation, s32Mirror, s32Flip), s32Ret);
        }
        else if(u32Select == 6) // change isp 3dnr
        {
            MI_S32 s32IspChn = 0;
            MI_S32 s32level = 0;

            printf("select ISP channel:");
            scanf("%d", &s32IspChn);
            ST_Flush();

            printf("3dnr level :");
            scanf("%d", &s32level);
            ST_Flush();
            ExecFuncResult(ST_DoChangeIsp3dnr(0, (MI_ISP_CHANNEL)s32IspChn, (MI_ISP_3DNR_Level_e)s32level), s32Ret);
        }
        else if(u32Select == 7) // change isp input crop
        {
            MI_S32 s32ScanfTemp = 0;
            MI_ISP_DEV DevId = 0;
            MI_ISP_CHANNEL ChnId = 0;
            MI_SYS_WindowRect_t stCropInfo;
            memset(&stCropInfo,0x0,sizeof(MI_SYS_WindowRect_t));

            printf("DevId:");
            scanf("%d", &s32ScanfTemp);
            ST_Flush();
            DevId = (MI_ISP_DEV)s32ScanfTemp;

            printf("ChnId:");
            scanf("%d", &s32ScanfTemp);
            ST_Flush();
            ChnId = (MI_ISP_CHANNEL)s32ScanfTemp;

            printf("stCropInfo:\n:");
            printf("x:");
            scanf("%d",&s32ScanfTemp);
            ST_Flush();
            stCropInfo.u16X = (MI_U16)s32ScanfTemp;

            printf("y:");
            scanf("%d",&s32ScanfTemp);
            ST_Flush();
            stCropInfo.u16Y = (MI_U16)s32ScanfTemp;

            printf("w:");
            scanf("%d",&s32ScanfTemp);
            ST_Flush();
            stCropInfo.u16Width = (MI_U16)s32ScanfTemp;

            printf("h:");
            scanf("%d",&s32ScanfTemp);
            ST_Flush();
            stCropInfo.u16Height = (MI_U16)s32ScanfTemp;

            printf("DevId(%d),ChnId(%d),stCropRect(%d,%d,%d,%d)\n", DevId,ChnId,
            stCropInfo.u16X,stCropInfo.u16Y,stCropInfo.u16Width,stCropInfo.u16Height);

            ExecFuncResult(ST_DoChangeIspInputCrop(DevId,ChnId,&stCropInfo), s32Ret);
        }
        else if(u32Select == 8) //change isp output param
        {
            MI_S32 s32ScanfTemp = 0;
            MI_ISP_DEV DevId = 0;
            MI_ISP_CHANNEL ChnId = 0;
            MI_ISP_PORT IspOutPortId = 0;
            MI_ISP_OutPortParam_t stIspOutputParam;
            memset(&stIspOutputParam,0x0,sizeof(MI_ISP_OutPortParam_t));

            printf("DevId:");
            scanf("%d", &s32ScanfTemp);
            ST_Flush();
            DevId = (MI_ISP_DEV)s32ScanfTemp;

            printf("ChnId:");
            scanf("%d", &s32ScanfTemp);
            ST_Flush();
            ChnId = (MI_ISP_CHANNEL)s32ScanfTemp;

            printf("IspOutPortId:");
            scanf("%d", &s32ScanfTemp);
            ST_Flush();
            IspOutPortId = (MI_ISP_PORT)s32ScanfTemp;

            printf("ePixelFormat:");
            scanf("%d", &s32ScanfTemp);
            ST_Flush();
            stIspOutputParam.ePixelFormat = (MI_SYS_PixelFormat_e)s32ScanfTemp;

            printf("eCompressMode:");
            scanf("%d", &s32ScanfTemp);
            ST_Flush();
            stIspOutputParam.eCompressMode = (MI_SYS_CompressMode_e)s32ScanfTemp;

            printf("stCropRect:\n:");
            printf("x:");
            scanf("%d",&s32ScanfTemp);
            ST_Flush();
            stIspOutputParam.stCropRect.u16X = (MI_U16)s32ScanfTemp;

            printf("y:");
            scanf("%d",&s32ScanfTemp);
            ST_Flush();
            stIspOutputParam.stCropRect.u16Y = (MI_U16)s32ScanfTemp;

            printf("w:");
            scanf("%d",&s32ScanfTemp);
            ST_Flush();
            stIspOutputParam.stCropRect.u16Width = (MI_U16)s32ScanfTemp;

            printf("h:");
            scanf("%d",&s32ScanfTemp);
            ST_Flush();
            stIspOutputParam.stCropRect.u16Height = (MI_U16)s32ScanfTemp;

            printf("DevId(%d),ChnId(%d),IspOutPortId(%d),stCropRect(%d,%d,%d,%d), pixel %d, compress %d\n", DevId,ChnId,IspOutPortId,
            stIspOutputParam.stCropRect.u16X,stIspOutputParam.stCropRect.u16Y,
            stIspOutputParam.stCropRect.u16Width,stIspOutputParam.stCropRect.u16Height,
            stIspOutputParam.ePixelFormat,stIspOutputParam.eCompressMode);

            ExecFuncResult(ST_DoChangeIspOutputParam(DevId,ChnId,IspOutPortId,&stIspOutputParam), s32Ret);
        }
        else if(u32Select == 9) //change isp zoom
        {
            MI_S32 s32ScanfTemp = 0;
            MI_ISP_DEV DevId = 0;
            MI_ISP_CHANNEL ChnId = 0;
            MI_BOOL bRev = FALSE;
            MI_SYS_WindowRect_t stStart;
            MI_SYS_WindowRect_t stDest;
            MI_SYS_WindowRect_t stStepWH;
            memset(&stStart, 0x0, sizeof(MI_SYS_WindowRect_t));
            memset(&stDest, 0x0, sizeof(MI_SYS_WindowRect_t));
            memset(&stStepWH, 0x0, sizeof(MI_SYS_WindowRect_t));

            printf("DevId:");
            scanf("%d", &s32ScanfTemp);
            ST_Flush();
            DevId = (MI_ISP_DEV)s32ScanfTemp;

            printf("ChnId:");
            scanf("%d", &s32ScanfTemp);
            ST_Flush();
            ChnId = (MI_ISP_CHANNEL)s32ScanfTemp;

            printf("breverse:");
            scanf("%d", &s32ScanfTemp);
            ST_Flush();
            bRev = (MI_BOOL)s32ScanfTemp;

            printf("stStart:\n:");
            printf("x:");
            scanf("%d",&s32ScanfTemp);
            ST_Flush();
            stStart.u16X = (MI_U16)s32ScanfTemp;

            printf("y:");
            scanf("%d",&s32ScanfTemp);
            ST_Flush();
            stStart.u16Y = (MI_U16)s32ScanfTemp;

            printf("w:");
            scanf("%d",&s32ScanfTemp);
            ST_Flush();
            stStart.u16Width = (MI_U16)s32ScanfTemp;

            printf("h:");
            scanf("%d",&s32ScanfTemp);
            ST_Flush();
            stStart.u16Height = (MI_U16)s32ScanfTemp;

            printf("stDest:\n:");
            printf("x:");
            scanf("%d",&s32ScanfTemp);
            ST_Flush();
            stDest.u16X = (MI_U16)s32ScanfTemp;

            printf("y:");
            scanf("%d",&s32ScanfTemp);
            ST_Flush();
            stDest.u16Y = (MI_U16)s32ScanfTemp;

            printf("w:");
            scanf("%d",&s32ScanfTemp);
            ST_Flush();
            stDest.u16Width = (MI_U16)s32ScanfTemp;

            printf("h:");
            scanf("%d",&s32ScanfTemp);
            ST_Flush();
            stDest.u16Height = (MI_U16)s32ScanfTemp;

            printf("stStepWH:\n:");
            printf("x:");
            scanf("%d",&s32ScanfTemp);
            ST_Flush();
            stStepWH.u16X = (MI_U16)s32ScanfTemp;

            printf("y:");
            scanf("%d",&s32ScanfTemp);
            ST_Flush();
            stStepWH.u16Y = (MI_U16)s32ScanfTemp;

            printf("w:");
            scanf("%d",&s32ScanfTemp);
            ST_Flush();
            stStepWH.u16Width = (MI_U16)s32ScanfTemp;

            printf("h:");
            scanf("%d",&s32ScanfTemp);
            ST_Flush();
            stStepWH.u16Height = (MI_U16)s32ScanfTemp;

            printf("DevId(%d),ChnId(%d),breverse(%d),stStart(%d,%d,%d,%d),stDest(%d,%d,%d,%d),stDest(%d,%d,%d,%d)\n", DevId,ChnId,bRev,
            stStart.u16X,stStart.u16Y,stStart.u16Width,stStart.u16Height,
            stDest.u16X,stDest.u16Y,stDest.u16Width,stDest.u16Height,
            stStepWH.u16X,stStepWH.u16Y,stStepWH.u16Width,stStepWH.u16Height);

            ExecFuncResult(ST_DoSetIspZoom(DevId,ChnId,bRev,&stStart,&stDest,&stStepWH), s32Ret);
        }
        else if(u32Select == 10) // isp customer 3A
        {
#if MI_ISPIQ_SUPPORT
            //ST_EnableCustomize3A();
#endif
        }
        else if(u32Select == 11)
        {
#if MI_ISPIQ_SUPPORT
            MI_S32 s32ScanfTemp = 0;
            MI_ISP_DEV DevId = 0;
            MI_ISP_CHANNEL ChnId = 0;
            char sFilePath[128];
            printf("DevId:");
            scanf("%d", &s32ScanfTemp);
            ST_Flush();
            DevId = (MI_ISP_DEV)s32ScanfTemp;

            printf("ChnId:");
            scanf("%d", &s32ScanfTemp);
            ST_Flush();
            ChnId = (MI_ISP_CHANNEL)s32ScanfTemp;

            printf("write API Bin file path:\n");
            scanf("%s", sFilePath);
            ST_Flush();
            bExit =ST_DoSetIqBin(DevId, ChnId, sFilePath);
#endif
        }
        else if(u32Select == 12)//isp set skip frame
        {
            MI_ISP_DEV IspDev = 0;
            MI_ISP_CHANNEL IspChn = 0;
            MI_U32 u32FrameNum = 0;
            MI_S32 s32ScanTemp=0;

            printf("select Dev id:");
            scanf("%d", &s32ScanTemp);
            ST_Flush();
            IspDev = s32ScanTemp;

            printf("select channel id:");
            scanf("%d", &s32ScanTemp);
            ST_Flush();
            IspChn = s32ScanTemp;

            printf("select skip frame cnt:");
            scanf("%d", &s32ScanTemp);
            ST_Flush();
            u32FrameNum = s32ScanTemp;

            ExecFuncResult(ST_DoSetIspSkipFrame(IspDev,IspChn,u32FrameNum), s32Ret);
        }
        else if(u32Select == 13)//SCL change rotate
        {
            MI_S32 s32SclChn = 0;
            MI_S32 s32Rotation = 0;

            printf("select SCL channel:");
            scanf("%d", &s32SclChn);
            ST_Flush();

            printf("rotation(0:0, 1:90, 2:180, 3:270):");
            scanf("%d", &s32Rotation);
            ST_Flush();

            ExecFuncResult(ST_DoChangeSclRotate(3, (MI_SCL_CHANNEL)s32SclChn, (MI_SYS_Rotate_e)s32Rotation), s32Ret);
        }
        else if(u32Select == 14) //chang scl input crop
        {
            MI_SCL_DEV DevId=0;
            MI_SCL_CHANNEL ChnId=0;
            MI_SYS_WindowRect_t stCropRect;
            memset(&stCropRect, 0x0, sizeof(MI_SYS_WindowRect_t));

            printf("select Dev id:");
            scanf("%d", &DevId);
            ST_Flush();

            printf("select channel id:");
            scanf("%d", &ChnId);
            ST_Flush();

            printf("crop x:");
            scanf("%d", (int *)&stCropRect.u16X);
            ST_Flush();

            printf("crop y:");
            scanf("%d", (int *)&stCropRect.u16Y);
            ST_Flush();

            printf("crop width:");
            scanf("%d", (int *)&stCropRect.u16Width);
            ST_Flush();

            printf("crop height:");
            scanf("%d", (int *)&stCropRect.u16Height);
            ST_Flush();

            ExecFuncResult(ST_DoChangeSclInputCrop(DevId, ChnId, &stCropRect), s32Ret);
        }
        else if(u32Select == 15) //change scl output param
        {
            MI_SCL_DEV DevId=0;
            MI_SCL_CHANNEL ChnId=0;
            MI_SCL_PORT PortId=0;
            MI_SCL_OutPortParam_t  stOutputParam;
            MI_S32 s32ScanTemp=0;
            memset(&stOutputParam, 0x0, sizeof(MI_SCL_OutPortParam_t));

            printf("select Dev id:");
            scanf("%d", &s32ScanTemp);
            ST_Flush();
            DevId = s32ScanTemp;

            printf("select channel id:");
            scanf("%d", &s32ScanTemp);
            ST_Flush();
            ChnId = s32ScanTemp;

            printf("select port id:");
            scanf("%d", &s32ScanTemp);
            ST_Flush();
            PortId = s32ScanTemp;

            printf("port %d bmirror:", PortId);
            scanf("%d", &s32ScanTemp);
            ST_Flush();
            stOutputParam.bMirror = s32ScanTemp;

            printf("port %d bflip:", PortId);
            scanf("%d", &s32ScanTemp);
            ST_Flush();
            stOutputParam.bFlip = s32ScanTemp;

            printf("port %d crop x:", PortId);
            scanf("%d", &s32ScanTemp);
            ST_Flush();
            stOutputParam.stSCLOutCropRect.u16X = s32ScanTemp;

            printf("port %d crop y:", PortId);
            scanf("%d", &s32ScanTemp);
            ST_Flush();
            stOutputParam.stSCLOutCropRect.u16Y = s32ScanTemp;

            printf("port %d crop width:", PortId);
            scanf("%d", &s32ScanTemp);
            ST_Flush();
            stOutputParam.stSCLOutCropRect.u16Width=s32ScanTemp;

            printf("port %d crop height:", PortId);
            scanf("%d", &s32ScanTemp);
            ST_Flush();
            stOutputParam.stSCLOutCropRect.u16Height=s32ScanTemp;

            printf("port %d out width:", PortId);
            scanf("%d", &s32ScanTemp);
            ST_Flush();
            stOutputParam.stSCLOutputSize.u16Width=s32ScanTemp;

            printf("port %d out height:", PortId);
            scanf("%d", &s32ScanTemp);
            ST_Flush();
            stOutputParam.stSCLOutputSize.u16Height=s32ScanTemp;

            printf("port %d out pixel:\n", PortId);
            printf("yuv422yuyv:0, yuv422uyvy:14, yuv422yvyu:15,yuv422vyuy:16,yuv422sp:10, yuv422planar:17\n");
            printf("yuv420nv12:11,yuv420nv21:12, yuv420planar:18 \n");
            printf("argb8888  :1, abgr8888  :2,  bgra8888:3, \n");
            scanf("%d",&s32ScanTemp);
            ST_Flush();
            stOutputParam.ePixelFormat=(MI_SYS_PixelFormat_e)s32ScanTemp;

            printf("port %d out compressmode 0(disable)/5(enable):", PortId);
            scanf("%d", &s32ScanTemp);
            ST_Flush();
            stOutputParam.eCompressMode=(MI_SYS_CompressMode_e)s32ScanTemp;

            ExecFuncResult(ST_DoChangeSclOutPutParam(DevId, ChnId, PortId, &stOutputParam), s32Ret);
        }
        else if(u32Select == 16) // scl stretch buffer
        {
            char sFileInputPath[128];
            char sFileOutputPath[128];
            MI_S32 s32SrcW=0, s32SrcH=0;
            MI_S32 s32PortCropX =0, s32PortCropY=0,s32PortCropW=0,s32PortCropH =0;
            MI_S32 s32DestW=0, s32DestH=0;
            MI_S32 s32DestPixel, s32InputPixel;
            MI_U32 u32SrcStride =0;

            printf("read file path:\n");
            scanf("%s", sFileInputPath);
            ST_Flush();

            printf("input pixel:");
            scanf("%d", &s32InputPixel);
            ST_Flush();

            printf("Input width:");
            scanf("%d", &s32SrcW);
            ST_Flush();

            printf("Input height:");
            scanf("%d", &s32SrcH);
            ST_Flush();

            printf("Input buff stride:");
            scanf("%d", &u32SrcStride);
            ST_Flush();

            printf("port crop x:");
            scanf("%d", &s32PortCropX);
            ST_Flush();

            printf("port crop y:");
            scanf("%d", &s32PortCropY);
            ST_Flush();

            printf("port crop width:");
            scanf("%d", &s32PortCropW);
            ST_Flush();

            printf("port crop height:");
            scanf("%d", &s32PortCropH);
            ST_Flush();

            printf("Dest width:");
            scanf("%d", &s32DestW);
            ST_Flush();

            printf("Dest height:");
            scanf("%d", &s32DestH);
            ST_Flush();

            printf("output pixel:");
            scanf("%d", &s32DestPixel);
            ST_Flush();

            printf("output file path:\n");
            scanf("%s", sFileOutputPath);
            ST_Flush();

            MI_SYS_WindowSize_t stInputWinSize;
            stInputWinSize.u16Width = s32SrcW;
            stInputWinSize.u16Height = s32SrcH;

            MI_SYS_WindowRect_t stCropWin;
            stCropWin.u16X = s32PortCropX;
            stCropWin.u16Y = s32PortCropY;
            stCropWin.u16Width = s32PortCropW;
            stCropWin.u16Height = s32PortCropH;

            MI_SYS_WindowSize_t stOutputWinSize;
            stOutputWinSize.u16Width = s32DestW;
            stOutputWinSize.u16Height = s32DestH;

            ExecFuncResult(ST_DoChangeStretchBuff(sFileInputPath, (MI_SYS_PixelFormat_e)s32InputPixel, stInputWinSize, u32SrcStride, stCropWin, stOutputWinSize, (MI_SYS_PixelFormat_e)s32DestPixel, sFileOutputPath), s32Ret);
        }
        else if(u32Select == 17) // output step size
        {
            MI_S32  s32ModuleId =0;
            MI_S32  s32DevId =0;
            MI_S32  s32ChnId =0;
            MI_S32  s32Portid = 0;
            MI_U32  u32MaxWidth  = 0;
            MI_U32  u32MaxHeight = 0;
            char    sFilePath[128];
            MI_SYS_ChnPort_t    stChnPort;
            MI_SYS_WindowSize_t stMaxWin;
            memset(&stChnPort, 0x0, sizeof(MI_SYS_ChnPort_t));
            memset(&stMaxWin,  0x0, sizeof(MI_SYS_WindowSize_t));

            printf("select module id vif:6 scl:34 :");
            scanf("%d", &s32ModuleId);
            ST_Flush();

            printf("select Dev id:");
            scanf("%d", &s32DevId);
            ST_Flush();

            printf("select channel id:");
            scanf("%d", &s32ChnId);
            ST_Flush();

            printf("select port id:");
            scanf("%d", &s32Portid);
            ST_Flush();

            printf("file MaxWidth:");
            scanf("%d", &u32MaxWidth);
            ST_Flush();

            printf("file MaxHeight:");
            scanf("%d", &u32MaxHeight);
            ST_Flush();

            printf("write file path:\n");
            scanf("%s", sFilePath);
            ST_Flush();

            stChnPort.eModId    = (MI_ModuleId_e)s32ModuleId;
            stChnPort.u32DevId  = s32DevId;
            stChnPort.u32ChnId  = s32ChnId;
            stChnPort.u32PortId = s32Portid;
            stMaxWin.u16Width   = u32MaxWidth;
            stMaxWin.u16Height  = u32MaxHeight;
            ExecFuncResult(ST_DoChangeOutputSizeStep(&stChnPort, &stMaxWin, sFilePath), s32Ret);
        }
        else if(u32Select == 18) // crop step size
        {
            MI_S32  s32DevId =0;
            MI_S32  s32ChnId =0;
            MI_S32  s32Portid = 0;
            MI_S32  s32Select = 0;
            MI_U32  u32MaxWidth  = 0;
            MI_U32  u32MaxHeight = 0;
            char    sFilePath[128];
            ST_CropPosition_e eCropPosition;
            MI_SYS_ChnPort_t stChnPort;
            MI_SYS_WindowSize_t stMaxWin;
            memset(&stChnPort, 0x0, sizeof(MI_SYS_ChnPort_t));
            memset(&stMaxWin,  0x0, sizeof(MI_SYS_WindowSize_t));

            printf("1.VIF Dev    crop (MIPI only)\n");
            printf("2.VIF Port   crop \n");
            printf("3.Isp Input  crop \n");
            printf("4.Isp Output crop \n");
            printf("5.Scl Input  crop \n");
            printf("6.Scl Output crop \n");
            scanf("%d", &s32Select);
            ST_Flush();

            printf("select Dev id:");
            scanf("%d", &s32DevId);
            ST_Flush();

            printf("select channel id:");
            scanf("%d", &s32ChnId);
            ST_Flush();

            printf("select Output port id:");
            scanf("%d", &s32Portid);
            ST_Flush();

            printf("file MaxWidth:");
            scanf("%d", &u32MaxWidth);
            ST_Flush();

            printf("file MaxHeight:");
            scanf("%d", &u32MaxHeight);
            ST_Flush();

            printf("write file path:\n");
            scanf("%s", sFilePath);
            ST_Flush();

            switch(s32Select)
            {
                case 1:
                    eCropPosition = E_MI_VIF_DEV_CROP;
                    stChnPort.eModId = E_MI_MODULE_ID_VIF;
                    break;
                case 2:
                    eCropPosition = E_MI_VIF_OUTPUTPORT_CROP;
                    stChnPort.eModId = E_MI_MODULE_ID_VIF;
                    break;
                case 3:
                    eCropPosition = E_MI_ISP_INPUTPORT_CROP;
                    stChnPort.eModId = E_MI_MODULE_ID_ISP;
                    break;
                case 4:
                    eCropPosition = E_MI_ISP_OUTPUTPORT_CROP;
                    stChnPort.eModId = E_MI_MODULE_ID_ISP;
                    break;
                case 5:
                    eCropPosition = E_MI_SCL_INPUTPORT_CROP;
                    stChnPort.eModId = E_MI_MODULE_ID_SCL;
                    break;
                case 6:
                    eCropPosition = E_MI_SCL_OUTPUTPORT_CROP;
                    stChnPort.eModId = E_MI_MODULE_ID_SCL;
                    break;
                default:
                    printf("crop type error %d\n", s32Select);
                    continue;
            }

            stChnPort.u32DevId = s32DevId;
            stChnPort.u32ChnId = s32ChnId;
            stChnPort.u32PortId = s32Portid;
            stMaxWin.u16Width  = u32MaxWidth;
            stMaxWin.u16Height = u32MaxHeight;
            ExecFuncResult(ST_DoChangeCropSizeStepTest(eCropPosition, &stChnPort, &stMaxWin, sFilePath), s32Ret);
        }
        else if(u32Select == 19)
        {
            char sFilePath[128];
            char ModuleNameString[64];
            MI_S32  s32Portid = 0;
            MI_S32  s32DevId =0;
            MI_S32  s32ChnId = 0;
            MI_S32  s32DumpBuffNum =0;
            MI_SYS_ChnPort_t stChnPort;
            memset(&stChnPort, 0x0, sizeof(MI_SYS_ChnPort_t));

            printf("select module id(vif/isp/scl):");
            scanf("%s",ModuleNameString);
            ST_Flush();

            printf("select Dev id:");
            scanf("%d", &s32DevId);
            ST_Flush();

            printf("select channel id:");
            scanf("%d", &s32ChnId);
            ST_Flush();

            printf("select port id:");
            scanf("%d", &s32Portid);
            ST_Flush();

            printf("Dump Buffer Num:");
            scanf("%d", &s32DumpBuffNum);
            ST_Flush();

            printf("write file path:\n");
            scanf("%s", sFilePath);
            ST_Flush();

            if(!strcmp(ModuleNameString, "vif"))
                stChnPort.eModId= E_MI_MODULE_ID_VIF;
            else if(!strcmp(ModuleNameString, "isp"))
                stChnPort.eModId= E_MI_MODULE_ID_ISP;
            else if(!strcmp(ModuleNameString, "scl"))
                stChnPort.eModId= E_MI_MODULE_ID_SCL;

            stChnPort.u32DevId = s32DevId;
            stChnPort.u32ChnId = s32ChnId;
            stChnPort.u32PortId = s32Portid;
            ExecFuncResult(ST_GetModuleOutputData(&stChnPort,sFilePath,s32DumpBuffNum), s32Ret);
        }
        else if(u32Select == 20)
        {
            MI_S32 s32ScanfTemp = 0;
            MI_U32 u32Width=0;
            MI_U32 u32Height=0;
            char srcfile[128] = {0};

            printf("file width:");
            scanf("%d", &s32ScanfTemp);
            ST_Flush();
            u32Width = (MI_U32)s32ScanfTemp;

            printf("file height:");
            scanf("%d", &s32ScanfTemp);
            ST_Flush();
            u32Height = (MI_U32)s32ScanfTemp;

            printf("file path:\n");
            scanf("%s", srcfile);
            ST_Flush();

            ExecFuncResult(ST_FbdFile(u32Width,u32Height,srcfile), s32Ret);
        }
        else if(u32Select == 21)
        {
            MI_S32  VencChn = 0;
            MI_S32 s32DumpBuffNum = 0;
            char sFilePath[128];
            printf("select venc chn id:");
            scanf("%d", &VencChn);
            ST_Flush();

            printf("Dump Buffer Num:");
            scanf("%d", &s32DumpBuffNum);
            ST_Flush();

            printf("write file path:\n");
            scanf("%s", sFilePath);
            ST_Flush();

            ExecFuncResult(ST_GetVencOut(VencChn, s32DumpBuffNum, sFilePath), s32Ret);
        }
        else if(u32Select == 22)
        {
            char sFilePath[128];
            MI_S32 s32ScanTemp=0;
            MI_U32 u32Width = 0,u32Height = 0;
            MI_SYS_PixelFormat_e ePixel = E_MI_SYS_PIXEL_FRAME_FORMAT_MAX;

            printf("file width:");
            scanf("%d", &s32ScanTemp);
            u32Width = (MI_U32)s32ScanTemp;
            ST_Flush();

            printf("file height:");
            scanf("%d", &s32ScanTemp);
            u32Height = (MI_U32)s32ScanTemp;
            ST_Flush();

            printf("file pixel:");
            scanf("%d", &s32ScanTemp);
            ST_Flush();
            ePixel = (MI_SYS_PixelFormat_e)s32ScanTemp;

            printf("file path:\n");
            scanf("%s", sFilePath);
            ST_Flush();

            printf("file path[%s] WH(%d,%d) pixel %d \n",sFilePath,u32Width,u32Height,ePixel);

            ExecFuncResult(ST_ConvertTo16Bitsbayer(u32Width,u32Height,ePixel,sFilePath), s32Ret);
        }
        else if(u32Select == 23)
        {
#if USER_SENSOR_SUPPORT
            MI_S32 s32ScanTemp=0;
            printf("UseDetectThread:");
            scanf("%d", &s32ScanTemp);
            bDetectAD = (MI_BOOL)s32ScanTemp;
            ST_Flush();

            if(bDetectAD == FALSE)
            {
                MI_U8 u8ChipIndex=0, u8AdChn=0;
                DHC_DH9931_VIDEO_STATUS_S stVideoStatus;
                MI_U32 u32SensorWidth=0, u32SensorHeight=0;
                MI_BOOL bUseADHalfMode=FALSE;
                MI_U32  u32VifGroup=0, u32VifDev=0, u32VifModuleDevId=0;
                MI_VIF_WorkMode_e  eVifWorkMode;

                printf("Ad Index:");
                scanf("%d", &s32ScanTemp);
                u8ChipIndex = (MI_U8)s32ScanTemp;
                ST_Flush();

                printf("Ad chn:");
                scanf("%d", &s32ScanTemp);
                u8AdChn = (MI_U8)s32ScanTemp;
                ST_Flush();

                printf("bUseHalf Mode:");
                scanf("%d", &s32ScanTemp);
                bUseADHalfMode = (MI_BOOL)s32ScanTemp;
                ST_Flush();

                DHC_DH9931_SDK_GetVideoStatus(u8ChipIndex, u8AdChn, &stVideoStatus);
                getSensorSize(stVideoStatus.enVideoReportFormat, &u32SensorWidth, &u32SensorHeight);

                if(bUseADHalfMode ==TRUE)
                {
                    DHC_DH9931_SDK_SetHalfMode(u8ChipIndex, u8AdChn, DHC_TRUE, DHC_HALF_MODE_148_5);
                    u32SensorWidth=u32SensorWidth/2;
                }
                else
                {
                    DHC_DH9931_SDK_SetHalfMode(u8ChipIndex, u8AdChn, DHC_FALSE, DHC_HALF_MODE_148_5);
                }

                u32VifGroup =u8ChipIndex;

                eVifWorkMode = gstVifModule.stVifGroupAttr[u32VifGroup].eWorkMode;
                if(eVifWorkMode == E_MI_VIF_WORK_MODE_1MULTIPLEX)
                {
                    u32VifDev=0;
                }
                else
                    u32VifDev = u8AdChn;

                u32VifModuleDevId = u32VifGroup*ST_MAX_VIF_DEV_PERGROUP+u32VifDev;
                ExecFuncResult(ST_ResetVif(u32VifModuleDevId, u32SensorWidth, u32SensorHeight), s32Ret);
            }
#endif
        }
        else if(u32Select == 24)
        {
            MI_S32 s32ScanTemp = 0;
            MI_U32 u32pos = 0xFF;
            MI_U32 u32DevId = 0, u32ChnId = 0;
            MI_ModuleId_e eModuleId = E_MI_MODULE_ID_MAX;
            char ModuleNameString[64];

            printf("select module id(vif/isp/scl):");
            scanf("%s",ModuleNameString);
            ST_Flush();

            if(!strcmp(ModuleNameString, "vif"))
                eModuleId = E_MI_MODULE_ID_VIF;
            else if(!strcmp(ModuleNameString, "isp"))
                eModuleId = E_MI_MODULE_ID_ISP;
            else if(!strcmp(ModuleNameString, "scl"))
                eModuleId = E_MI_MODULE_ID_SCL;

            printf("select Dev id:");
            scanf("%d", &s32ScanTemp);
            ST_Flush();
            u32DevId = (MI_U32)s32ScanTemp;

            printf("select Chn id:");
            scanf("%d", &s32ScanTemp);
            ST_Flush();
            u32ChnId = (MI_U32)s32ScanTemp;

            if(eModuleId == E_MI_MODULE_ID_VIF)
            {
                printf("select reset point, 0 FIFO, 1 wdma, 2 AFIFO_wdma_group, 3 FIFO_wdma_dev :");
                scanf("%d", &s32ScanTemp);
                ST_Flush();
                u32pos = (MI_U32)s32ScanTemp;
            }

            ExecFuncResult(ST_DoHwReset(eModuleId, u32DevId, u32ChnId, u32pos), s32Ret);
        }
        else if(u32Select == 25)
        {
            MI_S32 s32ScanTemp = 0;
            MI_U32 u32DevId = 0;
            MI_BOOL bOn = FALSE;

            printf("select Dev id:");
            scanf("%d", &s32ScanTemp);
            ST_Flush();
            u32DevId = (MI_U32)s32ScanTemp;

            printf("bOn:");
            scanf("%d", &s32ScanTemp);
            ST_Flush();
            bOn = (MI_BOOL)s32ScanTemp;

            ExecFuncResult(ST_DoChangeVifDevOnOff(u32DevId, bOn), s32Ret);
        }
        else if(u32Select == 26)
        {
            MI_S32 s32ScanTemp = 0;
            MI_U32 u32DevId = 0;

            printf("select Dev id:");
            scanf("%d", &s32ScanTemp);
            ST_Flush();
            u32DevId = (MI_U32)s32ScanTemp;
            STCHECKRESULT(ST_DoDestroyCreateIspChannel(u32DevId));
        }
        else if(u32Select == 27)
        {
            MI_S32 s32ScanTemp = 0;
            MI_U32 u32DevId = 0;

            printf("select Dev id:");
            scanf("%d", &s32ScanTemp);
            ST_Flush();
            u32DevId = (MI_U32)s32ScanTemp;
            ExecFuncResult(ST_DoDestroyCreateSclChannel(u32DevId), s32Ret);
        }
        else if(u32Select == 28)
        {
            MI_S32 s32ScanTemp = 0;
            MI_U32 u32DevId = 0, u32ChnId = 0, u32PortId = 0;
            MI_BOOL bOn = FALSE;
            MI_ModuleId_e eModuleId = E_MI_MODULE_ID_MAX;
            char ModuleNameString[64];

            printf("select module id(vif/isp/scl):");
            scanf("%s",ModuleNameString);
            ST_Flush();

            if(!strcmp(ModuleNameString, "vif"))
                eModuleId = E_MI_MODULE_ID_VIF;
            else if(!strcmp(ModuleNameString, "isp"))
                eModuleId = E_MI_MODULE_ID_ISP;
            else if(!strcmp(ModuleNameString, "scl"))
                eModuleId = E_MI_MODULE_ID_SCL;

            printf("select Dev id:");
            scanf("%d", &s32ScanTemp);
            ST_Flush();
            u32DevId = (MI_U32)s32ScanTemp;

            if(eModuleId != E_MI_MODULE_ID_VIF)
            {
                printf("select Chn id:");
                scanf("%d", &s32ScanTemp);
                ST_Flush();
                u32ChnId = (MI_U32)s32ScanTemp;
            }

            printf("select port id:");
            scanf("%d", &s32ScanTemp);
            ST_Flush();
            u32PortId = (MI_U32)s32ScanTemp;

            printf("on:");
            scanf("%d", &s32ScanTemp);
            ST_Flush();
            bOn = (MI_BOOL)s32ScanTemp;

            ExecFuncResult(ST_DoCrcOnOff(eModuleId, u32DevId, u32ChnId, u32PortId, bOn), s32Ret);
        }
        else if(u32Select == 29) //Change chnport On/off
        {
            MI_S32 s32ScanTemp = 0;
            MI_U32 u32count = 0;
            MI_BOOL bStatus = FALSE;
            MI_SYS_ChnPort_t stChnPort;
            ST_DynamicChnPortPosition_e ePosition = E_CHNPORT_POSITION_MAX;
            char ModuleNameString[64];
            memset(&stChnPort, 0x0, sizeof(MI_SYS_ChnPort_t));

            printf("select module id(vif/isp/scl):");
            scanf("%s",ModuleNameString);
            ST_Flush();

            if(!strcmp(ModuleNameString, "vif"))
            {
                stChnPort.eModId = E_MI_MODULE_ID_VIF;
            }
            else if(!strcmp(ModuleNameString, "isp"))
            {
                stChnPort.eModId = E_MI_MODULE_ID_ISP;
            }
            else if(!strcmp(ModuleNameString, "scl"))
            {
                stChnPort.eModId = E_MI_MODULE_ID_SCL;
            }
            else
            {
                DBG_WRN("not support module:%s\n",ModuleNameString);
                continue;
            }

            printf("select position chn(0)/port(1):");
            scanf("%d", &s32ScanTemp);
            ST_Flush();
            ePosition = (ST_DynamicChnPortPosition_e)s32ScanTemp;

            printf("select Dev id:");
            scanf("%d", &s32ScanTemp);
            ST_Flush();
            stChnPort.u32DevId = (MI_U32)s32ScanTemp;

            printf("select Chn id:");
            scanf("%d", &s32ScanTemp);
            ST_Flush();
            stChnPort.u32ChnId = (MI_U32)s32ScanTemp;

            if(ePosition == E_CHNPORT_POSITION_PORT)
            {
                printf("select port id:");
                scanf("%d", &s32ScanTemp);
                ST_Flush();
                stChnPort.u32PortId = (MI_U32)s32ScanTemp;
            }

            printf("Enter status: On(1)/Off(0)");
            scanf("%d", &s32ScanTemp);
            ST_Flush();
            bStatus = (MI_BOOL)s32ScanTemp;

            printf("On/Off count:");
            scanf("%d", &s32ScanTemp);
            ST_Flush();
            u32count = (MI_U32)s32ScanTemp;

            while(u32count--)
            {
                ExecFuncResult(ST_DoChangeChnPortOnOff(&stChnPort,ePosition,bStatus), s32Ret);
                bStatus = (bStatus == TRUE) ? FALSE : TRUE;
            }
        }
        else if(u32Select == 30)
        {
            MI_VIF_GROUP GroupId = 0;
            MI_SYS_PixelFormat_e eInputPixel = E_MI_SYS_PIXEL_FRAME_YUV422_UYVY;
            MI_S32 s32ScanfTemp = 0;

            printf("select Group id:");
            scanf("%d", &s32ScanfTemp);
            ST_Flush();
            GroupId = (MI_VIF_GROUP)s32ScanfTemp;

            printf("select input pixel:");
            scanf("%d", &s32ScanfTemp);
            ST_Flush();
            eInputPixel = (MI_SYS_PixelFormat_e)s32ScanfTemp;

            ExecFuncResult(ST_DoChangeVifGroupAttr(GroupId, eInputPixel), s32Ret);
        }
#if USER_SENSOR_SUPPORT
        else if(u32Select == 31)
        {
            MI_U8  u8ChipIndex =0;
            MI_U8  u8ChannelIndex =0;
            MI_S32 s32ScanfTemp = 0;
            printf("choice 9931 chip index \n");
            printf("0: 9931 index 0 \n");
            printf("1: 9931 index 1 \n");
            scanf("%d", &s32ScanfTemp);
            ST_Flush();
            u8ChipIndex = (MI_U8)s32ScanfTemp;

            printf("choice 9931 channel index\n");
            scanf("%d", &s32ScanfTemp);
            ST_Flush();
            u8ChannelIndex = (MI_U8)s32ScanfTemp;

            printf("select controlid:\n");
            printf("%d.enable\n", E_SEND_CMD_ENABLE);

            printf("%d.cvi up\n", E_SEND_CMD_CVI_UP);
            printf("%d.cvi down\n", E_SEND_CMD_CVI_DOWN);
            printf("%d.cvi left\n", E_SEND_CMD_CVI_LEFT);
            printf("%d.cvi right\n", E_SEND_CMD_CVI_RIGHT);
            printf("%d.cvi enter\n", E_SEND_CMD_CVI_ENTER);

            printf("8.ahd1080p  up\n");
            printf("9.ahd1080p  down\n");
            printf("10.ahd1080p left\n");
            printf("11.ahd1080p right\n");
            printf("12.ahd1080p enter\n");
            printf("13.ahd1080p add\n");

            printf("%d.ahd  up\n", E_SEND_CMD_AHD_UP);
            printf("%d.ahd  down\n", E_SEND_CMD_AHD_DOWN);
            printf("%d.ahd left\n", E_SEND_CMD_AHD_LEFT);
            printf("%d.ahd right\n", E_SEND_CMD_AHD_RIGHT);
            printf("%d.ahd enter\n", E_SEND_CMD_AHD_ENTER);
            //printf("19.ahd add\n");
            scanf("%d", &s32ScanfTemp);
            ST_Flush();

            ST_DH9931ControlSensorMenu(u8ChipIndex,u8ChannelIndex,(ST_DH9931OsdMenuCmd_e)s32ScanfTemp);
        }
        else if(u32Select == 32)
        {
            MI_U8  u8ChipIndex =0;
            MI_U8  u8ChannelIndex =0;
            DHC_DH9931_VIDEO_FMT_E  eChangeSnrFormatRes =DHC_INVALID_FMT;
            MI_S32 s32ScanfTemp = 0;
            ST_SensorType_e eSnrType;

            printf("choice 9931 chip index \n");
            printf("0: 9931 index 0 \n");
            printf("1: 9931 index 1 \n");
            scanf("%d", &s32ScanfTemp);
            ST_Flush();
            u8ChipIndex = (MI_U8)s32ScanfTemp;
            //u8ChipIndex=1;

            printf("choice 9931 channel index\n");
            scanf("%d", &s32ScanfTemp);
            ST_Flush();
            u8ChannelIndex = (MI_U8)s32ScanfTemp;
            //u8ChannelIndex=3;

            printf("choice sensor type:1(YOF), 2(DH)\n");
            scanf("%d", &s32ScanfTemp);
            ST_Flush();
            eSnrType = (ST_SensorType_e)s32ScanfTemp;
            //eSnrType=E_SENSOR_TYPE_DH;

            printf("choice change AHD sensor format:\n");
            printf("222:show sensorformat meun\n");
            printf("%d. CVI 720P25 \n", DHC_CVI_1280x720_25HZ);
            printf("%d. CVI 720P30 \n", DHC_CVI_1280x720_30HZ);
            printf("%d. CVI 1080P25 \n", DHC_CVI_1920x1080_25HZ);
            printf("%d. CVI 1080P30 \n", DHC_CVI_1920x1080_30HZ);
            printf("%d. AHD 720P25 \n", DHC_AHD_1280x720_25HZ);
            printf("%d. AHD 720P30 \n", DHC_AHD_1280x720_30HZ);
            printf("%d. AHD 1080P25 \n", DHC_AHD_1920x1080_25HZ);
            printf("%d. AHD 1080P30 \n", DHC_AHD_1920x1080_30HZ);
            scanf("%d", &s32ScanfTemp);
            ST_Flush();
            eChangeSnrFormatRes = (DHC_DH9931_VIDEO_FMT_E)s32ScanfTemp;
            /*
            switch(i)
            {
                case 0:
                    eChangeSnrFormatRes = DHC_CVI_1280x720_25HZ;
                    break;
                case 1:
                    eChangeSnrFormatRes = DHC_CVI_1280x720_30HZ;
                    break;
                case 2:
                    eChangeSnrFormatRes = DHC_CVI_1920x1080_25HZ;
                    break;
                case 3:
                    eChangeSnrFormatRes = DHC_CVI_1920x1080_30HZ;
                    break;
                default:
                    i=0;
                    eChangeSnrFormatRes = DHC_CVI_1920x1080_25HZ;
                    break;
            }

            printf("choice format %d \n", eChangeSnrFormatRes);
            i++;
            sleep(1);*/
            ST_DH9931ChoiceSnrFormatRes(u8ChipIndex, u8ChannelIndex, eChangeSnrFormatRes, eSnrType);
        }
#endif
        else if(u32Select == 33)
        {
            MI_S32 s32ScanfTemp = 0;
            printf("Need Report 9931 Detect Log:0: no print, 1:print");
            scanf("%d", &s32ScanfTemp);
            ST_Flush();
            bDetectLog = (MI_BOOL)s32ScanfTemp;
        }
        /*
        else if(u32Select == 10)
        {
            bExit =ST_DoSetChnZoom(u32ChnNum);
        }
        */
        else if(u32Select == 111)
        {
            bExit = TRUE;
        }

        usleep(THREAD_SLEEP_TIME_US);
    }

    usleep(THREAD_SLEEP_TIME_US);

    ST_RtspServerStop();

    if(bDetectAD ==TRUE)
    {
#if USER_SENSOR_SUPPORT
        void *retarg = NULL;
        pthread_cancel(ptCheck9931);
        pthread_join(ptCheck9931, &retarg);
#endif
    }
	MI_U32 u32VifGroup=0;
	MI_U8 eSnrPad = 0;

    //ExecFuncResult(ST_BaseModuleUnInit(), s32Ret);
    unBind_isp_scl_venc();
    for(num=0;num<4;num++){
	VencModuleDeInit(num);
	}
	destruct_scl_rtsp_module();
	IspModuleUnInit(0);
	for(u32VifGroup=0;u32VifGroup <ST_MAX_VIF_GROUP_NUM;u32VifGroup++){
	ExecFuncResult(VifModuleUnInit((MI_VIF_GROUP)u32VifGroup), s32Ret);
	}
	for(eSnrPad=0; eSnrPad<ST_MAX_SENSOR_NUM; eSnrPad++){
	ExecFuncResult(ST_SensorModuleUnInit((MI_SNR_PADID)eSnrPad), s32Ret);
	}
    ExecFuncResult(ST_SysModuleUnInit(), s32Ret);

    pthread_mutex_destroy(&gIniMd5Mutex);
    memset(&gIniMd5Mutex, 0x0, sizeof(gIniMd5Mutex));

    u32ChnNum=0;
    gbMutiVpeChnNum =0;

    memset(&gstSensorAttr, 0x0, sizeof(gstSensorAttr));
    memset(&gstVifModule, 0x0, sizeof(gstVifModule));
    memset(&gstVencattr, 0x0, sizeof(gstVencattr));
    memset(&gstIspModule, 0x0, sizeof(gstIspModule));
    memset(&gstSclModule, 0x0, sizeof(gstSclModule));

    gettimeofday(&stTestEndTime, NULL);

    DBG_INFO("================>case cmd %s cost time %ld s\n", argv[1], (stTestEndTime.tv_sec-stTestBeginTime.tv_sec));
	}
    s32Ret = UTStatus;
	
EXIT:
    return s32Ret;
}

