#ifndef __ST_VPE_DATATYPE_H_
#define __ST_VPE_DATATYPE_H_

#include "BasicUsageEnvironment.hh"
#include "liveMedia.hh"
#include "Live555RTSPServer.hh"
#include <string>

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/syscall.h>
#include <sys/resource.h>

#include "st_common.h"
#include "st_venc.h"
#include "dictionary.h"
#include "iniparser.h"
#include "md5.h"

//#include "mi_rgn.h"
#include "mi_sensor.h"
#include "mi_sensor_datatype.h"

#include "mi_vif.h"
//#include "mi_eptz.h"
//#include "mi_ldc.h"
//#include "mi_divp.h"
#include "mi_disp.h"
#include "mi_panel.h"
#include "mi_hdmi.h"
#include "mi_jpd.h"
#include "mi_isp.h"
#include "mi_scl.h"

#include "linux_list.h"

#if ((defined PROG_USER_SENSOR) && (PROG_USER_SENSOR == 1))
    #include "st_dh9931.h"
    #include "st_dh9931menuconfig.h"
    #define USER_SENSOR_SUPPORT (PROG_USER_SENSOR)
#else
    #define USER_SENSOR_SUPPORT (0)
#endif

#if ((defined PROG_VIF_ENABLE) && (PROG_VIF_ENABLE == 1))
    #define MI_VIF_SUPPORT (PROG_VIF_ENABLE)
#else
    #define MI_VIF_SUPPORT (0)
#endif

#if ((defined PROG_ISP_ENABLE) && (PROG_ISP_ENABLE == 1))
    #ifdef __cplusplus
        extern "C" {
    #endif
        extern int FBD_Execute(unsigned char *pSrc, unsigned short *pDst, int w, int h);
    #ifdef __cplusplus
        }
    #endif
    #define MI_ISP_SUPPORT (PROG_ISP_ENABLE)
#else
    #define MI_ISP_SUPPORT (0)
#endif

#if ((defined PROG_ISPIQ_ENABLE) && (PROG_ISPIQ_ENABLE == 1))
    #include "mi_isp_iq.h"
    #include "mi_isp_cus3a_api.h"
    #include "mi_iqserver.h"
    #include "st_cus3a.h"
    #define MI_ISPIQ_SUPPORT (PROG_ISPIQ_ENABLE)
#else
    #define MI_ISPIQ_SUPPORT (0)
#endif

#if ((defined PROG_IPU_ENABLE) && (PROG_IPU_ENABLE == 1))
    #include "st_ipu.h"
    #define MI_IPU_SUPPORT (PROG_IPU_ENABLE)
#else
    #define MI_IPU_SUPPORT (0)
#endif

#if ((defined PROG_SCL_ENABLE) && (PROG_SCL_ENABLE == 1))
    #define MI_SCL_SUPPORT (PROG_SCL_ENABLE)
#else
    #define MI_SCL_SUPPORT (0)
#endif

#if ((defined PROG_JPD_ENABLE) && (PROG_JPD_ENABLE == 1))
    #define MI_JPD_SUPPORT (PROG_JPD_ENABLE)
#else
    #define MI_JPD_SUPPORT (0)
#endif

#if ((defined PROG_VENC_ENABLE) && (PROG_VENC_ENABLE == 1))
    #define MI_VENC_SUPPORT (PROG_VENC_ENABLE)
#else
    #define MI_VENC_SUPPORT (0)
#endif

#if ((defined PROG_DISP_ENABLE) && (PROG_DISP_ENABLE == 1))
    #define MI_DISP_SUPPORT (PROG_DISP_ENABLE)
#else
    #define MI_DISP_SUPPORT (0)
#endif

using namespace std;

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

#define CHIP_USE (1)

#ifdef CHIP_USE
#define THREAD_SLEEP_TIME_US  (10*1000)
#elif defined FPGA_USE
#define THREAD_SLEEP_TIME_US  (1*1000)
#elif defined PZ1_USE
#define THREAD_SLEEP_TIME_US  (1)
#endif

#ifndef SWAP
#define SWAP(a, b)  do { (a) ^= (b); (b) ^= (a); (a) ^= (b); } while (0)
#endif

#if (defined CONFIG_SIGMASTAR_CHIP_I6E) && (CONFIG_SIGMASTAR_CHIP_I6E == 1)
    #define ST_MAX_SENSOR_NUM (3)

    #define ST_MAX_VIF_DEV_PERGROUP (4)
    #define ST_MAX_VIF_GROUP_NUM (3)
    #define ST_VIF_OUTPORT1_SCALING_LIMIT_W 0
    #define ST_MAX_VIF_OUTPORT_NUM (2)

    #define ST_MAX_ISP_DEV_NUM (1)
    #define ST_MAX_ISP_CHN_NUM (16)
    #define ST_MAX_ISP_INPORT_NUM (1)
    #define ST_MAX_ISP_OUTPORT_NUM (3)

    #define ST_MAX_SCL_CHN_NUM (16)
    #define ST_MAX_SCL_INPORT_NUM (1)
    #define ST_MAX_SCL_OUTPORT_NUM (6)
    #define ST_MAX_SCL_HWSCLID_NUM (6)
    #define ST_MAX_SCL_DEV_NUM (4)
    #define ST_SCL_USER_STRETCHBUFF_DEVID (255)
    #define ST_SCL_USER_STRETCHBUFF_PORTID E_MI_SCL_HWSCL_MAX

    #define ST_MAX_JPD_CHN_NUM (0)
    #define ST_MAX_VENC_CHN_NUM_PERDEV (16)
    #define ST_MAX_VENC_DEV_NUM (1)
    #define ST_MAX_VENC_NUM   (ST_MAX_VENC_CHN_NUM_PERDEV*ST_MAX_VENC_DEV_NUM)

    #define ST_MAX_DISP_DEV_NUM (1)
    #define ST_MAX_DISP_LAYER_NUM (1)
    #define ST_MAX_DISP_LAYER0_PORT_NUM (1)
    #define ST_MAX_DISP_LAYER1_PORT_NUM (0)
#elif ((defined CONFIG_SIGMASTAR_CHIP_M6) && (CONFIG_SIGMASTAR_CHIP_M6 == 1))
    #define ST_MAX_SENSOR_NUM (4)

    #define ST_MAX_VIF_DEV_PERGROUP (4)
    #define ST_MAX_VIF_GROUP_NUM (4)
    #define ST_VIF_OUTPORT1_SCALING_LIMIT_W 960   //vif port1 scaling down max width size 960
    #define ST_MAX_VIF_OUTPORT_NUM (2)

    #define ST_MAX_ISP_DEV_NUM (1)
    #define ST_MAX_ISP_CHN_NUM (16)
    #define ST_MAX_ISP_INPORT_NUM (1)
    #define ST_MAX_ISP_OUTPORT_NUM (3)

    #define ST_MAX_SCL_DEV_NUM (4)
    #define ST_MAX_SCL_CHN_NUM (16)
    #define ST_MAX_SCL_INPORT_NUM (1)
    #define ST_MAX_SCL_OUTPORT_NUM (6)
    #define ST_MAX_SCL_HWSCLID_NUM (6)
    #define ST_SCL_USER_STRETCHBUFF_DEVID (3)
    #define ST_SCL_USER_STRETCHBUFF_PORTID E_MI_SCL_HWSCL5

    #define ST_MAX_JPD_CHN_NUM (16)
    #define ST_MAX_VENC_CHN_NUM_PERDEV (16)
    #define ST_MAX_VENC_DEV_NUM (1)
    #define ST_MAX_VENC_NUM   (ST_MAX_VENC_CHN_NUM_PERDEV*ST_MAX_VENC_DEV_NUM)

    #define ST_MAX_DISP_DEV_NUM (2)
    #define ST_MAX_DISP_LAYER_NUM (2)
    #define ST_MAX_DISP_LAYER0_PORT_NUM (16)
    #define ST_MAX_DISP_LAYER1_PORT_NUM (1)

#elif ((defined CONFIG_SIGMASTAR_CHIP_P3) && CONFIG_SIGMASTAR_CHIP_P3 == 1)
    #define ST_MAX_SENSOR_NUM (2)

    #define ST_MAX_VIF_DEV_PERGROUP (4)
    #define ST_MAX_VIF_GROUP_NUM (2)
    #define ST_VIF_OUTPORT1_SCALING_LIMIT_W 0
    #define ST_MAX_VIF_OUTPORT_NUM (2)

    #define ST_MAX_ISP_DEV_NUM (1)
    #define ST_MAX_ISP_CHN_NUM (16)
    #define ST_MAX_ISP_INPORT_NUM (1)
    #define ST_MAX_ISP_OUTPORT_NUM (3)

    #define ST_MAX_SCL_DEV_NUM (4)
    #define ST_MAX_SCL_CHN_NUM (16)
    #define ST_MAX_SCL_INPORT_NUM (1)
    #define ST_MAX_SCL_OUTPORT_NUM (6)
    #define ST_MAX_SCL_HWSCLID_NUM (6)
    #define ST_SCL_USER_STRETCHBUFF_DEVID (255)
    #define ST_SCL_USER_STRETCHBUFF_PORTID E_MI_SCL_HWSCL_MAX

    #define ST_MAX_JPD_CHN_NUM (0)
    #define ST_MAX_VENC_CHN_NUM_PERDEV (16)
    #define ST_MAX_VENC_DEV_NUM (1)
    #define ST_MAX_VENC_NUM   (ST_MAX_VENC_CHN_NUM_PERDEV*ST_MAX_VENC_DEV_NUM)

    #define ST_MAX_DISP_DEV_NUM (1)
    #define ST_MAX_DISP_LAYER_NUM (1)
    #define ST_MAX_DISP_LAYER0_PORT_NUM (1)
    #define ST_MAX_DISP_LAYER1_PORT_NUM (0)
#elif ((defined CONFIG_SIGMASTAR_CHIP_I7) && (CONFIG_SIGMASTAR_CHIP_I7 == 1))
    #define ST_MAX_SENSOR_NUM (8)

    #define ST_MAX_VIF_DEV_PERGROUP (4)
    #define ST_MAX_VIF_GROUP_NUM (8)
    #define ST_MAX_VIF_OUTPORT_NUM (1)
    #define ST_VIF_OUTPORT1_SCALING_LIMIT_W 0

    #define ST_MAX_ISP_DEV_NUM (2)
    #define ST_MAX_ISP_CHN_NUM (32)
    #define ST_MAX_ISP_INPORT_NUM (1)
    #define ST_MAX_ISP_OUTPORT_NUM (3)

    #define ST_MAX_SCL_DEV_NUM (7)
    #define ST_MAX_SCL_CHN_NUM (32)
    #define ST_MAX_SCL_INPORT_NUM (1)
    #define ST_MAX_SCL_OUTPORT_NUM (9)
    #define ST_MAX_SCL_HWSCLID_NUM (9)
    #define ST_SCL_USER_STRETCHBUFF_DEVID (3)
    #define ST_SCL_USER_STRETCHBUFF_PORTID E_MI_SCL_HWSCL8

    #define ST_MAX_JPD_CHN_NUM (16)
    #define ST_MAX_VENC_CHN_NUM_PERDEV (32)
    #define ST_MAX_VENC_DEV_NUM (2)
    #define ST_MAX_VENC_NUM   (ST_MAX_VENC_CHN_NUM_PERDEV*ST_MAX_VENC_DEV_NUM)

    #define ST_MAX_DISP_DEV_NUM (2)
    #define ST_MAX_DISP_LAYER_NUM (2)
    #define ST_MAX_DISP_LAYER0_PORT_NUM (16)
    #define ST_MAX_DISP_LAYER1_PORT_NUM (1)
#elif ((defined CONFIG_SIGMASTAR_CHIP_M6P) && (CONFIG_SIGMASTAR_CHIP_M6P == 1))
    #define ST_MAX_SENSOR_NUM (4)

    #define ST_MAX_VIF_DEV_PERGROUP (4)
    #define ST_MAX_VIF_GROUP_NUM (4)
    #define ST_MAX_VIF_OUTPORT_NUM (1)
    #define ST_VIF_OUTPORT1_SCALING_LIMIT_W (0)

    #define ST_MAX_ISP_DEV_NUM (1)
    #define ST_MAX_ISP_CHN_NUM (32)
    #define ST_MAX_ISP_INPORT_NUM (1)
    #define ST_MAX_ISP_OUTPORT_NUM (2)

    #define ST_MAX_SCL_DEV_NUM (4)
    #define ST_MAX_SCL_CHN_NUM (32)
    #define ST_MAX_SCL_INPORT_NUM (1)
    #define ST_MAX_SCL_OUTPORT_NUM (7)
    #define ST_MAX_SCL_HWSCLID_NUM (7)
    #define ST_SCL_USER_STRETCHBUFF_DEVID (1)
    #define ST_SCL_USER_STRETCHBUFF_PORTID E_MI_SCL_HWSCL6

    #define ST_MAX_JPD_CHN_NUM (16)
    #define ST_MAX_VENC_CHN_NUM_PERDEV (32)
    #define ST_MAX_VENC_DEV_NUM (1)
    #define ST_MAX_VENC_NUM   (ST_MAX_VENC_CHN_NUM_PERDEV*ST_MAX_VENC_DEV_NUM)

    #define ST_MAX_DISP_DEV_NUM (2)
    #define ST_MAX_DISP_LAYER_NUM (2)
    #define ST_MAX_DISP_LAYER0_PORT_NUM (16)
    #define ST_MAX_DISP_LAYER1_PORT_NUM (1)
#else
    #define ST_MAX_SENSOR_NUM (3)

    #define ST_MAX_VIF_DEV_PERGROUP (4)
    #define ST_MAX_VIF_GROUP_NUM (2)
    #define ST_MAX_VIF_OUTPORT_NUM (2)
    #define ST_VIF_OUTPORT1_SCALING_LIMIT_W (0)

    #define ST_MAX_ISP_DEV_NUM (1)
    #define ST_MAX_ISP_CHN_NUM (16)
    #define ST_MAX_ISP_INPORT_NUM (1)
    #define ST_MAX_ISP_OUTPORT_NUM (3)

    #define ST_MAX_SCL_DEV_NUM (4)
    #define ST_MAX_SCL_CHN_NUM (16)
    #define ST_MAX_SCL_INPORT_NUM (1)
    #define ST_MAX_SCL_OUTPORT_NUM (6)
    #define ST_MAX_SCL_HWSCLID_NUM (6)
    #define ST_SCL_USER_STRETCHBUFF_DEVID (255)
    #define ST_SCL_USER_STRETCHBUFF_PORTID E_MI_SCL_HWSCL_MAX

    #define ST_MAX_JPD_CHN_NUM (0)
    #define ST_MAX_VENC_CHN_NUM_PERDEV (16)
    #define ST_MAX_VENC_DEV_NUM (1)
    #define ST_MAX_VENC_NUM   (ST_MAX_VENC_CHN_NUM_PERDEV*ST_MAX_VENC_DEV_NUM)

    #define ST_MAX_DISP_DEV_NUM (1)
    #define ST_MAX_DISP_LAYER_NUM (2)
    #define ST_MAX_DISP_LAYER0_PORT_NUM (16)
    #define ST_MAX_DISP_LAYER1_PORT_NUM (1)
#endif

#ifndef STCHECKRESULTEXIT
#define STCHECKRESULTEXIT(_func_)\
    do{ \
        MI_S32 s32Ret = MI_SUCCESS; \
        s32Ret = _func_; \
        if (s32Ret != MI_SUCCESS)\
        { \
            printf("[%s %d]exec function failed, error:%x\n", __func__, __LINE__, s32Ret); \
            goto EXIT; \
        } \
        else \
        { \
            printf("(%s %d)exec function pass\n", __FUNCTION__,__LINE__); \
        } \
    } while(0)
#endif

#define ASCII_COLOR_GREEN                        "\033[1;32m"
#define ASCII_COLOR_END                          "\033[0m"
#define ASCII_COLOR_RED                          "\033[1;31m"
#define ASCII_COLOR_YELLOW                       "\033[1;33m"

#define DBG_INFO(fmt, args...) printf(ASCII_COLOR_GREEN"%s[%d]: " fmt ASCII_COLOR_END, __FUNCTION__,__LINE__, ##args);
#define DBG_WRN(fmt, args...) printf(ASCII_COLOR_YELLOW"%s[%d]: " fmt ASCII_COLOR_END, __FUNCTION__,__LINE__, ##args);
#define DBG_ERR(fmt, args...) printf(ASCII_COLOR_RED"%s[%d]: " fmt ASCII_COLOR_END, __FUNCTION__,__LINE__, ##args);

#define GET_DIFF_TIME_US(Start,End) (abs((End.tv_sec * 1000000 + End.tv_usec) - (Start.tv_sec * 1000000 + Start.tv_usec)))

#define BAYER_LIMIT_BITS(x, bits) ((x) > ((1 << (bits)) - 1) ? (((1 << (bits)) - 1)) : (x))

#define BUFFER_SIZE_OUT_16BITS(w) ((((w + 7) / 8) * 8) * 2) //unsigned char
#define BUFFER_SIZE_IN_12BITS(w) ((((w + 31) / 32) * 32) * 12 / 8) // (12bits/ pixel) -> (12 / 8) (bytes / pixel)
#define BUFFER_SIZE_IN_10BITS(w) ((((w + 63) / 64) * 64) * 10 / 8)  // (10bits/ pixel) -> (10 / 8) (bytes / pixel)
#define BUFFER_SIZE_IN_8BITS(w) ((((w + 15) / 16) * 16))   // (8bits/ pixel) -> (8 / 8) (bytes / pixel)


#define RTSP_LISTEN_PORT        (554)
#define BUFFER_DEST_W           (3840)
#define BUFFER_DEST_H           (2160)
#define MAX_ADDRESS_COUNT       (3)

#define ST_MAX_VIF_DEV_NUM (ST_MAX_VIF_GROUP_NUM * ST_MAX_VIF_DEV_PERGROUP)
#define ST_MAX_VIF_RES_NUM (ST_MAX_VIF_DEV_NUM * ST_MAX_VIF_OUTPORT_NUM)
#define ST_MAX_ISP_RES_NUM (ST_MAX_ISP_DEV_NUM * ST_MAX_ISP_CHN_NUM * ST_MAX_ISP_OUTPORT_NUM)
#define ST_MAX_SCL_RES_NUM (ST_MAX_SCL_DEV_NUM * ST_MAX_SCL_CHN_NUM * ST_MAX_SCL_OUTPORT_NUM)

typedef enum
{
    E_ST_SCL_SRC_FROM_INVALID,
    E_ST_SCL_SRC_FROM_ISP_REALTIME = 0x01,//dev0
    E_ST_SCL_SRC_FROM_RDMA0        = 0x02,//dev1
    E_ST_SCL_SRC_FROM_YUV_REALTIME = 0x04,//dev2
    E_ST_SCL_SRC_FROM_RDMA1        = 0x08,//dev3
    E_ST_SCL_SRC_FROM_RDMA_ROT     = 0x10,//dev3
    E_ST_SCL_SRC_FROM_RDMA_MAP     = 0x20,//dev3
    E_ST_SCL_SRC_FROM_ISP_REALTIME1= 0x40,//dev4
    E_ST_SCL_SRC_FROM_RDMA2        = 0x80,//dev5
    E_ST_SCL_SRC_FROM_YUV_REALTIME1= 0x100,//dev6
    E_ST_SCL_SRC_FROM_MAX = 0xffff
}ST_SCL_SourceSelect_e;

typedef enum
{
    E_MI_SCL_DEVID_0,
    E_MI_SCL_DEVID_1,
    E_MI_SCL_DEVID_2,
    E_MI_SCL_DEVID_3,
    E_MI_SCL_DEVID_4,
    E_MI_SCL_DEVID_5,
    E_MI_SCL_DEVID_6,
    E_MI_SCL_DEVID_MAX
} MI_SCL_DevId_e;

typedef enum
{
    E_ST_DUMP_STATUS_BEGIN      = 0x00,
    E_ST_DUMP_STATUS_RUNNING    = 0x01,
    E_ST_DUMP_STATUS_END        = 0x02,
    E_ST_DUMP_STATUS_MAX        = 0x03
}ST_DumpOutputStatus_e;

typedef enum
{
    E_ST_MD5_ACTION_NONE       = 0x00,
    E_ST_MD5_ACTION_CHECK,
    E_ST_MD5_ACTION_RESET,
    E_ST_MD5_ACTION_ADD,
    E_ST_MD5_ACTION_MAX
}ST_Md5Action_e;

typedef enum
{
    E_CHNPORT_POSITION_CHN,
    E_CHNPORT_POSITION_PORT,
    E_CHNPORT_POSITION_MAX,
}ST_DynamicChnPortPosition_e;


#define ST_MAX_MD5_VALUE_NUM  (64)
#define ST_NEED_STATICS_MD5_VALUE (0)
#define ST_MD5_CHAR_NUM (32)
#define STATICS_MD5_TOTAL_CHARNUM (ST_MAX_MD5_VALUE_NUM * (ST_MD5_CHAR_NUM+4) + 5)
typedef struct ST_MD5Info_s
{
    MI_U8 u8MD5ExpectValue[16];
    MI_BOOL bUsed;
}ST_MD5Info_t;

typedef struct ST_RandomParamAttr_s
{
    MI_BOOL   bParamRandomTest;
    pthread_t pRandomParamthread;
}ST_RandomParamAttr_t;

typedef struct ST_VifParamSave_s
{
    MI_SYS_ChnPort_t      stValidVifResId[ST_MAX_VIF_RES_NUM];
    MI_U32                u32ValidVifResNum;
}ST_VifParamSave_t;

typedef struct ST_IspParamSave_s
{
    MI_SYS_ChnPort_t      stValidIspResId[ST_MAX_ISP_RES_NUM];
    MI_U32                u32ValidIspResNum;
}ST_IspParamSave_t;

typedef struct ST_SclParamSave_s
{
    MI_SYS_ChnPort_t      stValidSclResId[ST_MAX_SCL_RES_NUM];
    MI_U32                u32ValidSclResNum;
}ST_SclParamSave_t;

typedef struct ST_BindParam_s
{
    MI_SYS_ChnPort_t stChnPort;
    MI_SYS_BindType_e       eBindType;
    MI_U32 u32BindParam;
    MI_U32 u32SrcFrmrate;
    MI_U32 u32DstFrmrate;
}ST_BindParam_t;

typedef struct ST_Output_BindParam_List_s
{
    ST_Sys_BindInfo_T stBindInfo;
    struct list_head pos;
}ST_Output_BindParam_List_t;

typedef struct ST_InputFile_Attr_s
{
    MI_U32 u32Width;
    MI_U32 u32Height;
    MI_U32 u32SleepMs;
    MI_SYS_CompressMode_e eCompress;
    MI_BOOL              bCrcCheck;
    MI_SYS_PixelFormat_e ePixelFormat;
    char InputFilePath[256];
    pthread_mutex_t mutex;
    pthread_t pPutDatathread;

    MI_SYS_ChnPort_t  stModuleInfo;
    MI_SYS_WindowRect_t stContentCropWindow;
}ST_InputFile_Attr_t;

typedef struct ST_Md5Attr_s
{
    ST_MD5Info_t stMD5ExpectValue[ST_MAX_MD5_VALUE_NUM];
    MI_U8 *pu8IniPath;
    char key[30];
    MI_U8 Md5ValueString[32];
    ST_Md5Action_e eMd5Action;
}ST_Md5Attr_t;


typedef struct ST_OutputFile_Attr_s
{
    MI_S32 s32DumpBuffNum;
    MI_U32 u32FinishCnt;
    char FilePath[100];
    pthread_mutex_t Portmutex;
    pthread_t pGetDatathread;
    MI_BOOL bThreadExit;

    MI_U16 u16Depth;
    MI_U16 u16UserDepth;

    MI_SYS_ChnPort_t  stModuleInfo;

    ST_Md5Attr_t stMd5Attr;

    MI_BOOL bNeedFbd;
    FILE *fp;
}ST_OutputFile_Attr_t;

typedef struct ST_Sensor_Attr_s
{
    MI_U32 u32BindVifDev;
    MI_BOOL bUsed;
    MI_BOOL bCreate;
    MI_BOOL bPlaneMode;
    MI_U8 u8ResIndex;
    MI_U8 ADIndex;
    CUS_SENIF_BUS eUserSensorIntf;
}ST_Sensor_Attr_t;

typedef struct ST_VifPortAttr_s
{
    MI_BOOL    bCreate;
    MI_BOOL    bUsed;
    MI_SYS_WindowRect_t      stCapRect;
    MI_SYS_WindowSize_t      stDestSize;
    MI_SYS_PixelFormat_e     ePixFormat;
    MI_U32 u32FrameModeLineCount;
    MI_VIF_FrameRate_e eFrameRate;
    MI_SYS_CompressMode_e eCompressMode;

    ST_OutputFile_Attr_t  stoutFileAttr;
    ST_Output_BindParam_List_t head;
}ST_VifPortAttr_t;

typedef struct ST_VifDevAttr_s
{
    MI_BOOL    bCreate;
    MI_BOOL    bUsed;
    MI_VIF_DevAttr_t stVifDevAttr;
    ST_VifPortAttr_t  stVifOutPortAttr[ST_MAX_VIF_OUTPORT_NUM];
    pthread_mutex_t   Devmutex;
}ST_VifDevAttr_t;

typedef struct ST_VifGroupAttr_s
{
    MI_BOOL    bCreate;
    MI_BOOL    bUsed;
    MI_VIF_GroupExtAttr_t stBindSensor;
    MI_BOOL    bNeedSetVifGroup2SnrPad;

    MI_VIF_WorkMode_e       eWorkMode;
    MI_VIF_HDRType_e        eHDRType;
    MI_U32                  u32DevStitchMask;
    ST_VifDevAttr_t stVifDevAttr[ST_MAX_VIF_DEV_PERGROUP];
}ST_VifGroupAttr_t;

typedef struct ST_VifModeAttr_s
{
    ST_VifParamSave_t    stVifParamSave;
    ST_RandomParamAttr_t stRandomParamAttr;

    ST_VifGroupAttr_t stVifGroupAttr[ST_MAX_VIF_GROUP_NUM];
}ST_VifModAttr_t;

typedef struct ST_PortAttr_s
{
    MI_BOOL bUsed;
    MI_BOOL bEnable;
    MI_BOOL bMirror;
    MI_BOOL bFlip;
    MI_SYS_PixelFormat_e ePixelFormat;
    MI_SYS_CompressMode_e eCompressMode;
    MI_SYS_WindowSize_t  stOrigPortSize;
    MI_SYS_WindowRect_t  stOrigPortCrop;

    MI_SYS_WindowRect_t  stPortCrop;
    MI_SYS_WindowSize_t  stPortSize;

    ST_OutputFile_Attr_t stoutFileAttr;
    ST_Output_BindParam_List_t head;
}ST_PortAttr_t;

typedef struct ST_InPortAttr_s
{
    MI_BOOL bUsed;
    MI_SYS_WindowRect_t stOrigInputCropWin;
    MI_SYS_WindowRect_t stInputCropWin;
    ST_BindParam_t stBindParam;

    ST_InputFile_Attr_t         stInputFileAttr ;
}ST_InPortAttr_t;

typedef struct ST_SkipFrameAttr_s
{
    MI_U32          u32SikpFrameCnt;
    struct timeval  stSatrtTime;
    pthread_mutex_t SkipMutex;
}ST_SkipFrameAttr_t;

typedef struct ST_AiIspThreadAttr_s
{
    MI_U8 u8DevId;
    MI_U8 u8ChnId;
    MI_BOOL  bUseIpu;
    MI_U32 u32IpuChn;
    pthread_t pThreadHandle;
    MI_BOOL bExit;
}ST_AiIspThreadAttr_t;


typedef struct ST_IspChannelAttr_s
{
    MI_U8 u8ChnId;
    MI_BOOL bUsed;
    MI_BOOL bCreate;
    ST_InPortAttr_t stIspInPortAttr[ST_MAX_ISP_INPORT_NUM];

    ST_PortAttr_t        stIspOutPortAttr[ST_MAX_ISP_OUTPORT_NUM];
    MI_ISP_ChannelAttr_t stIspChnAttr;
    MI_ISP_ChnParam_t    stIspChnParam;
    MI_SNR_PADID         eBindMiSnrPadId;
    ST_SkipFrameAttr_t   stIspSkipFarme;
    MI_ISP_CustSegAttr_t stIspCustSegAttr;
    MI_BOOL  bUseIpu;
    MI_U32 u32IpuChn;
    char aIpuImagePath[128];

    char IqCfgbin_Path[128];

    ST_AiIspThreadAttr_t stAiIspThreadAttr;
}ST_IspChannelAttr_t;

typedef struct ST_IspDevAttr_s
{
    MI_U8 u8DevId;
    MI_BOOL bUsed;
    MI_BOOL bCreate;
    ST_IspChannelAttr_t stIspChnlAttr[ST_MAX_ISP_CHN_NUM];
}ST_IspDevAttr_t;

typedef struct ST_IspModeAttr_s
{
    ST_IspParamSave_t    stIspParamSave;
    ST_RandomParamAttr_t stRandomParamAttr;

    ST_IspDevAttr_t  stIspDevAttr[ST_MAX_ISP_DEV_NUM];
}ST_IspModeAttr_t;

typedef struct ST_SclChannelAttr_s
{
    MI_U8 u8ChnId;
    MI_BOOL bUsed;
    MI_BOOL bCreate;
    MI_SYS_Rotate_e         eRotate;
    ST_SCL_SourceSelect_e   eSclSrcSelect;
    ST_InPortAttr_t stSclInPortAttr[ST_MAX_SCL_INPORT_NUM];
    ST_PortAttr_t        stSclOutPortAttr[ST_MAX_SCL_OUTPORT_NUM];
}ST_SclChannelAttr_t;

typedef struct ST_SclDevAttr_s
{
    MI_U8 u8DevId;
    MI_BOOL bUsed;
    MI_BOOL bCreate;
    MI_U32 u32UseHwSclMask;
    MI_U8  u8HwSclPortId[ST_MAX_SCL_HWSCLID_NUM];
    ST_SclChannelAttr_t stSclChnlAttr[ST_MAX_SCL_CHN_NUM];
}ST_SclDevAttr_t;

typedef struct ST_SclModeAttr_s
{
    ST_SclParamSave_t    stSclParamSave;
    ST_RandomParamAttr_t stRandomParamAttr;

    ST_SclDevAttr_t  stSclDevAttr[ST_MAX_SCL_DEV_NUM];
}ST_SclModeAttr_t;

typedef struct ST_VencAttr_s
{
    ST_BindParam_t stVencInBindParam;
	MI_BOOL 	   bEnable;

    MI_VENC_CHN vencChn;
    MI_VENC_ModType_e eType;
    MI_U32    u32Width;
    MI_U32     u32Height;
    MI_U32     u32Fps;
    //char szStreamName[128];
    const char *szStreamName;
    MI_BOOL bUsed;
    MI_BOOL bCreate;
    MI_VENC_DEV DevId;

    MI_S32 s32DumpBuffNum;
    char FilePath[128];

    ST_Md5Attr_t stMd5Attr;
}ST_VencAttr_t;

typedef struct ST_DispPortAttr_s
{
    MI_BOOL    bCreate;
    MI_BOOL    bUsed;

    MI_U16      u16Width;
    MI_U16      u16Height;

    MI_DISP_VidWinRect_t stDispPortWin;
    ST_BindParam_t  stBindInfo;
}ST_DispPortAttr_t;

typedef struct ST_DispLayerAttr_s
{
    MI_BOOL    bCreate;
    MI_BOOL    bUsed;

    MI_U16      u16PortUseCount;

    MI_U16      u16Width;
    MI_U16      u16Height;
    MI_DISP_RotateMode_e eRotMode;

    MI_DISP_VidWinRect_t stDispLayerWin;
    ST_DispPortAttr_t  stDispPortAttr[ST_MAX_DISP_LAYER0_PORT_NUM];
}ST_DispLayerAttr_t;


typedef struct ST_DispDevAttr_s
{
    MI_BOOL    bCreate;
    MI_BOOL    bUsed;

    MI_U16      u16Width;
    MI_U16      u16Height;

    MI_DISP_Interface_e eIntfType;
    MI_DISP_OutputTiming_e eIntfSync;
    MI_HDMI_TimingType_e eHdmiTiming;
    ST_DispLayerAttr_t stDispLayerAttr[ST_MAX_DISP_LAYER_NUM];
}ST_DispDevAttr_t;


typedef struct ST_DispModeAttr_s
{
    ST_DispDevAttr_t stDispDevAttr[ST_MAX_DISP_DEV_NUM];
}ST_DispModAttr_t;

typedef struct ST_JpdChnAttr_s
{
    MI_U32 u32ChnId;
    MI_BOOL bUsed;
    MI_BOOL bCreate;
    MI_U32 u32MaxW;
    MI_U32 u32MaxH;
    MI_U32 u32FrameBuffSize;
    MI_U32 u32FrameRate;
    char InputFilePath[256];
    pthread_t pPutDatathread;
    ST_Output_BindParam_List_t head;
}ST_JpdChnAttr_t;

typedef struct ST_JpdModeAttr_s
{
    ST_JpdChnAttr_t stJpdChnAttr[ST_MAX_JPD_CHN_NUM];
}ST_JpdModeAttr_t;

typedef enum
{
    E_DYNAMIC_CHANGE_HDRRES,
    E_DYNAMIC_CHANGE_VIFGROUPATTR,
    E_DYNAMIC_CHANGE_VIFDEVATTR,
    E_DYNAMIC_CHANGE_VIFOUTPORTATTR,
    E_DYNAMIC_CHANGE_VIFOUTFRC,
    E_DYNAMIC_CHANGE_VIFOUTPORTONOFF,
    E_DYNAMIC_CHANGE_VIFDEVONOFF,
    E_DYNAMIC_CHANGE_ISPROT,
    E_DYNAMIC_CHANGE_ISP3DNRLEVEL,
    E_DYNAMIC_CHANGE_ISPINPUTCROP,
    E_DYNAMIC_CHANGE_ISPOUTPARAM,
    E_DYNAMIC_CHANGE_ISPZOOM,
    E_DYNAMIC_CHANGE_ISPSKIPFRAME,
    E_DYNAMIC_CHANGE_SCLINPUTCROP,
    E_DYNAMIC_CHANGE_SCLOUTPUTPARAM,
    E_DYNAMIC_CHANGE_SCLSTRETCHBUFF,
    E_DYNAMIC_CHANGE_OUTPUTSTEPSIZE,
    E_DYNAMIC_CHANGE_CROPSTEPSIZE,
    E_DYNAMIC_CHANGE_HWRESET,
    E_DYNAMIC_CHANGE_SCLROTATE,
    E_DYNAMIC_DESTROYCREATE_CHANNEL,
    E_DYNAMIC_CHANGE_CRCONOFF,
    E_DYNAMIC_CHANGE_CHNPORTONOFF,
}ST_DynamicFuncType_e;

typedef struct ST_DynamicChangHdrRes_s
{
    MI_VIF_GROUP GroupId ;
    MI_BOOL bUseHdr;
    MI_U8  u8ResIndex;
}ST_DynamicChangeHdrRes_t;

typedef struct ST_DynamicChangeVifGroupAttr_s
{
    MI_VIF_GROUP GroupId;
    MI_SYS_PixelFormat_e eInputPixel;
}ST_DynamicChangeVifGroupAttr_t;

typedef struct ST_DynamicChangeVifDevAttr_s
{
    MI_VIF_DEV DevId;
    MI_VIF_DevAttr_t stDevAttr;
}ST_DynamicChangeVifDevAttr_t;

typedef struct ST_DynamicChangeVifOutPutAttr_s
{
    MI_VIF_DEV DevId;
    MI_VIF_PORT PortId;
    MI_VIF_OutputPortAttr_t stOutPutAttr;
}ST_DynamicChangeVifOutPutAttr_t;

typedef struct ST_DynamicChangeVifOutFRC_s
{
    MI_VIF_DEV DevId;
    MI_VIF_PORT PortId;
    MI_U32 u32FrameRate;
}ST_DynamicChangeVifOutFRC_t;

typedef struct ST_DynamicChangeVifOutPutOnOff_s
{
    MI_VIF_DEV DevId;
    MI_VIF_PORT PortId;
    MI_BOOL bOn;
}ST_DynamicChangeVifOutPutOnOff_t;

typedef struct ST_DynamicChangIspRot_s
{
    MI_ISP_DEV DevId;
    MI_ISP_CHANNEL ChnId;
    MI_SYS_Rotate_e  eRot;
    MI_BOOL  bMirror;
    MI_BOOL  bFlip;
}ST_DynamicChangeIspRot_t;

typedef struct ST_DynamicChangIsp3dnr_s
{
    MI_ISP_DEV DevId;
    MI_ISP_CHANNEL ChnId;
    MI_ISP_3DNR_Level_e e3dnrlevel;
}ST_DynamicChangeIsp3dnr_t;

typedef struct ST_DynamicChangIspInputCrop_s
{
    MI_ISP_DEV DevId;
    MI_ISP_CHANNEL ChnId;
    MI_SYS_WindowRect_t stCropInfo;
}ST_DynamicChangeIspInputCrop_t;

typedef struct ST_DynamicChangIspOutputParam_s
{
    MI_ISP_DEV DevId;
    MI_ISP_CHANNEL ChnId;
    MI_ISP_PORT PortId;
    MI_ISP_OutPortParam_t stOutPortParam;
}ST_DynamicChangeIspOutputParam_t;

typedef struct ST_DynamicChangIspZoom_s
{
    MI_ISP_DEV DevId;
    MI_ISP_CHANNEL ChnId;
    MI_BOOL bRev;
    MI_SYS_WindowRect_t stStart;
    MI_SYS_WindowRect_t stDest;
    MI_SYS_WindowRect_t stStepWH;
}ST_DynamicChangeIspZoom_t;

typedef struct ST_DynamicChangIspSkipFrame_s
{
    MI_ISP_DEV DevId;
    MI_ISP_CHANNEL ChnId;
    MI_U32 u32FrameNum;
}ST_DynamicChangeIspSkipFrame_t;

typedef struct ST_DynamicChangOutputSizeStep_s
{
    MI_SYS_ChnPort_t  stChnPort;
    char FilePath[128];
    MI_SYS_WindowSize_t stMaxWin;
}ST_DynamicChangOutputSizeStep_t;

typedef enum
{
    E_MI_VIF_DEV_CROP=1,
    E_MI_VIF_OUTPUTPORT_CROP,
    E_MI_ISP_INPUTPORT_CROP,
    E_MI_ISP_OUTPUTPORT_CROP,
    E_MI_SCL_INPUTPORT_CROP,
    E_MI_SCL_OUTPUTPORT_CROP,
}ST_CropPosition_e;

typedef struct ST_DynamicChangCropStep_s
{
    ST_CropPosition_e eCropPosition;
    MI_SYS_ChnPort_t  stChnPort;
    char FilePath[128];
    MI_SYS_WindowSize_t stMaxWin;
}ST_DynamicChangeCropStep_t;

typedef struct ST_DynamicChangeStretchBuff_s
{
    char  sFileInputPath[128];
    char  sFileOutputPath[128];
    MI_SYS_PixelFormat_e eInputPixel;
    MI_SYS_WindowSize_t stInputWinSize;
    MI_U32 u32InputStride;
    MI_SYS_WindowRect_t stCropWin;
    MI_SYS_WindowSize_t stOutputWinSize;
    MI_SYS_PixelFormat_e eOutputPixel;
}ST_DynamicChangeStretchBuff_t;

typedef struct ST_DynamicChangeSclOutputParam_s
{
    MI_SCL_DEV  DevId;
    MI_SCL_CHANNEL  ChnId;
    MI_SCL_PORT  PortId;
    MI_SCL_OutPortParam_t  stOutputParam;
}ST_DynamicChangeSclOutputParam_t;

typedef struct ST_DynamicChangeHwReset_s
{
    MI_ModuleId_e eModId;
    MI_U32  DevId;
    MI_U32  ChnId;
    MI_U8   u8Pos;
}ST_DynamicChangeHwReset_t;

typedef struct ST_DynamicChangeVifDevOnOff_s
{
    MI_U32  DevId;
    MI_BOOL bOn;
}ST_DynamicChangeVifDevOnOff_t;

typedef struct ST_DynamicChangeSclRotate_s
{
    MI_SCL_DEV DevId;
    MI_SCL_CHANNEL ChnId;
    MI_SYS_Rotate_e eRot;
}ST_DynamicChangeSclRotate_t;

typedef struct ST_DynamicChangeSclInputCrop_s
{
    MI_SCL_DEV DevId;
    MI_SCL_CHANNEL ChnId;
    MI_SYS_WindowRect_t stCropInfo;
}ST_DynamicChangeSclInputCrop_t;

typedef struct ST_DynamicDestroyCreateChannel_s
{
    MI_ModuleId_e eModId;
    MI_U32  DevId;
}ST_DynamicDestroyCreateChannel_t;

typedef struct ST_DynamicChangeCRCOnoff_s
{
    MI_ModuleId_e eModuleId;
    MI_U32 u32DevId;
    MI_U32 u32ChnId;
    MI_U32 u32PortId;
    MI_BOOL bOn;
}ST_DynamicChangeCRCOnoff_t;

typedef struct ST_DynamicChangeChnPortOnoff_s
{
    MI_BOOL bStatus;
    MI_SYS_ChnPort_t  stChnPort;
    ST_DynamicChnPortPosition_e ePosition;
}ST_DynamicChangeChnPortOnoff_t;

typedef struct ST_DynamicFuncParam_s
{
    ST_DynamicFuncType_e eFuncType;
    MI_U32 u32Cnt;
    MI_U16 u16Id;
    MI_U32 u32Sleep;
    struct list_head  stDynamicFuncListNode;
    union{
        ST_DynamicChangeHdrRes_t stChangeHdrResParam;
        ST_DynamicChangeVifGroupAttr_t stChangeVifGroupAttr;
        ST_DynamicChangeVifDevAttr_t stChangeVifDevAttr;
        ST_DynamicChangeVifOutPutAttr_t stChangeVifOutPutAttr;
        ST_DynamicChangeVifOutFRC_t stChangeVifOutOutFRC;
        ST_DynamicChangeVifOutPutOnOff_t stChangeVifOutPutOnOff;
        ST_DynamicChangeVifDevOnOff_t stChangeVifDevOnOff;
        ST_DynamicChangeIspRot_t stChangeIspRot;
        ST_DynamicChangeIsp3dnr_t stChangeIsp3dnr;
        ST_DynamicChangeIspInputCrop_t stChangeIspInputCrop;
        ST_DynamicChangeIspOutputParam_t stChangeIspOutputParam;
        ST_DynamicChangeIspZoom_t stChangeIspZoom;
        ST_DynamicChangeIspSkipFrame_t stChangeIspSkipFrame;
        ST_DynamicChangeSclOutputParam_t stChangeSclOutputParam;
        ST_DynamicChangOutputSizeStep_t stChangeOutputSizeStep;
        ST_DynamicChangeCropStep_t stChangeCropStep;
        ST_DynamicChangeStretchBuff_t stChangeStretchBuff;
        ST_DynamicChangeHwReset_t stChangeHwReset;
        ST_DynamicChangeSclRotate_t stChangeSclRotate;
        ST_DynamicChangeSclInputCrop_t stChangeSclInputCrop;
        ST_DynamicDestroyCreateChannel_t stDestroyCreateChannel;
        ST_DynamicChangeCRCOnoff_t stChangeCRCOnoff;
        ST_DynamicChangeChnPortOnoff_t stChangeChnPortOnOff;
    };
}ST_DynamicFuncParam_t;

typedef struct ST_DumpFileInfo_s
{
    MI_SYS_ChnPort_t stChnPort;
    MI_U16 u16FileCnt;
    char  FilePath[128];
    struct list_head  stDumpFileListNode;
    void *poutputFileAttr;
}ST_DumpFileInfo_t;

typedef struct ST_DynamicTestInfo_s
{
    struct list_head  stDynamicFuncListHead;
    struct list_head  stDumpFileListHead;
    MI_U32  u32ListTotalNodeCnt;
    MI_U32  u32ListRepeatCnt;
    MI_S32  s32DynamicDumpCnt;
}ST_DynamicTestInfo_t;

typedef struct ST_StepTestInfo_s
{
    MI_SYS_WindowSize_t   stStepSize;
    MI_SYS_WindowSize_t   stLimitSize;
    MI_SYS_WindowSize_t   stOutputSize;
    MI_SYS_WindowRect_t   stOrgRect;
    MI_SYS_PixelFormat_e  ePixelFormat;
    ST_OutputFile_Attr_t *pstoutFileAttr;
    MI_SYS_WindowSize_t   stMaxWin;
}ST_StepTestInfo_t;

typedef enum
{
    UT_CASE_FAIL = -1,
    UT_CASE_SUCCESS = 0,
}UT_CaseStatus_e;

MI_S32 ST_MD5Action2(MI_U8 *pu8MD5ExpectValue, ST_Md5Attr_t *pstMd5Attr, ST_Md5Action_e eMd5Action);
MI_S32 ST_GetModuleOutputData(MI_SYS_ChnPort_t *pstChnPort, char *sFilePath, MI_S32 s32DumpBuffNum);
MI_S32 ST_CheckMkdirOutFile(char *pFilePath);


#ifdef __cplusplus
}
#endif


#endif



