#ifndef __ST_VPE_STRESS_TEST_H_
#define __ST_VPE_STRESS_TEST_H_

#include <signal.h>
#include "st_vpe_datatype.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ST_VIF_DEV_CROP_MAX_W 1920
#define ST_VIF_DEV_CROP_MAX_H 1080

#define ST_VIF_OUTPORT1_CROP_MAX_W 960
#define ST_VIF_OUTPORT1_CROP_MAX_H 1080
#define ST_VIF_OUTPORT1_SIZE_MAX_W 960
#define ST_VIF_OUTPORT1_SIZE_MAX_H 720

#define ST_ISP_AE_LIMIT_W 320
#define ST_ISP_AE_LIMIT_H 256

#define ST_ISP_INPUTCROP_MAX_W    1920
#define ST_ISP_INPUTCROP_MAX_H    1080
#define ST_ISP_OUTPUTCROP_LIMIT_W 80
#define ST_ISP_OUTPUTCROP_LIMIT_H 46
#define ST_ISP_OUTPUTCROP_MAX_W   1920
#define ST_ISP_OUTPUTCROP_MAX_H   1080

#define ST_SCL_INPUTCROP_LIMIT_W  80
#define ST_SCL_INPUTCROP_LIMIT_H  46
#define ST_SCL_INPUTCROP_MAX_W    3840
#define ST_SCL_INPUTCROP_MAX_H    2160

#define ST_SCL_OUTPUTCROP_LIMIT_W 80
#define ST_SCL_OUTPUTCROP_LIMIT_H 46
#define ST_SCL_OUTPUTCROP_MAX_W   3840
#define ST_SCL_OUTPUTCROP_MAX_H   2160

#define ST_SCL_OUTPUTSIZE_LIMIT_W 80
#define ST_SCL_OUTPUTSIZE_LIMIT_H 46
#define ST_SCL_OUTPUTSIZE_MAX_W   3840
#define ST_SCL_OUTPUTSIZE_MAX_H   2160


typedef enum
{
    E_VIF_RUN_FUNC_INVALIDPARAM,
    E_VIF_RUN_FUNC_DEVATTR,
    E_VIF_RUN_FUNC_OUTPUTATTR,
    E_VIF_RUN_FUNC_MAX,
}ST_VifRunFuncCase_e;

typedef enum
{
    E_ISP_RUN_FUNC_INVALIDPARAM,
    E_ISP_RUN_FUNC_INPUTCROP,
    E_ISP_RUN_FUNC_CHNPARAM,
    E_ISP_RUN_FUNC_OUTPUTPARAM,
    E_ISP_RUN_FUNC_MAX,
}ST_IspRunFuncCase_e;

typedef enum
{
    E_SCL_RUN_FUNC_INVALIDPARAM,
    E_SCL_RUN_FUNC_INPUTCROP,
    E_SCL_RUN_FUNC_CHNPARAM,
    E_SCL_RUN_FUNC_OUTPUTPARAM,
    E_SCL_RUN_FUNC_MAX,
}ST_SclRunFuncCase_e;

typedef enum
{
    E_RANDOM_STATE_INVALID_TEST,
    E_RANDOM_STATE_VALID_TEST,
    E_RANDOM_STATE_MIX_TEST,
}ST_RandomState_e;

typedef enum
{
    E_RANDOM_PARAM_STATE_INVALID,
    E_RANDOM_PARAM_STATE_VALID,
}ST_ParamState_e;

typedef struct ST_VifRunRandomFun_s
{
    ST_RandomState_e eRandomState;
    MI_U32  u32RunNum;
    MI_BOOL bRandomAllFunc;
    MI_U32  u32DumpBuffNum;
    char    FilePath[100];

    MI_BOOL bRandomDevCrop;
    MI_U32  u32DevCropMaxW;
    MI_U32  u32DevCropMaxH;
    MI_U32  u32DevCropLimitW;
    MI_U32  u32DevCropLimitH;
    MI_BOOL bRandomDevPixel;
    MI_BOOL bRandomHdn;
    MI_BOOL bRandomField;

    MI_BOOL bRandomOutputCrop;
    MI_U32  u32OutputCropMaxW;
    MI_U32  u32OutputCropMaxH;
    MI_U32  u32OutputCropLimitW;
    MI_U32  u32OutputCropLimitH;

    MI_BOOL bRandomOutputSize;
    MI_U32  u32OutputSizeMaxW;
    MI_U32  u32OutputSizeMaxH;
    MI_U32  u32OutputSizeLimitW;
    MI_U32  u32OutputSizeLimitH;
    MI_BOOL bRandomOutputPixel;
    MI_BOOL bRandomFramerate;

}ST_VifRunRandomFunc_t;

typedef struct ST_IspRunRandomFun_s
{
    ST_RandomState_e eRandomState;
    MI_U32  u32RunNum;
    MI_BOOL bRandomAllFunc;
    MI_U32  u32DumpBuffNum;
    char    FilePath[100];

    MI_BOOL bRandomInputCrop;
    MI_U32  u32InputCropMaxW;
    MI_U32  u32InputCropMaxH;
    MI_U32  u32InputCropLimitW;
    MI_U32  u32InputCropLimitH;

    MI_BOOL bRandom3dnr;
    MI_BOOL bRandomRot;
    MI_BOOL bRandomMirrorFlip;

    MI_BOOL bRandomOutputCrop;
    MI_U32  u32OutputCropMaxW;
    MI_U32  u32OutputCropMaxH;
    MI_U32  u32OutputCropLimitW;
    MI_U32  u32OutputCropLimitH;
    MI_BOOL bRandomPixel;

}ST_IspRunRandomFunc_t;

typedef struct ST_SclRunRandomFun_s
{
    ST_RandomState_e eRandomState;
    MI_U32  u32RunNum;
    MI_BOOL bRandomAllFunc;
    MI_U32  u32DumpBuffNum;
    char    FilePath[100];

    MI_BOOL bRandomInputCrop;
    MI_U32  u32InputCropMaxW;
    MI_U32  u32InputCropMaxH;
    MI_U32  u32InputCropLimitW;
    MI_U32  u32InputCropLimitH;

    MI_BOOL bRandomChnParam;

    MI_BOOL bRandomOutputCrop;
    MI_U32  u32OutputCropMaxW;
    MI_U32  u32OutputCropMaxH;
    MI_U32  u32OutputCropLimitW;
    MI_U32  u32OutputCropLimitH;

    MI_BOOL bRandomOutputSize;
    MI_U32  u32OutputSizeMaxW;
    MI_U32  u32OutputSizeMaxH;
    MI_U32  u32OutputSizeLimitW;
    MI_U32  u32OutputSizeLimitH;

    MI_BOOL bRandomPixel;
    MI_BOOL bRandomMirrorFlip;

}ST_SclRunRandomFunc_t;


MI_S32 ST_SaveValidParam();

MI_S32 ST_ParseRandomTestIni(char *pIniPath);

void * ST_VifRandomParamTestThread(void * args);

void * ST_IspRandomParamTestThread(void * args);

void * ST_SclRandomParamTestThread(void * args);


#ifdef __cplusplus
}
#endif


#endif
