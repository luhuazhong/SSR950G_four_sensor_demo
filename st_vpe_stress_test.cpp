#include "st_vpe_stress_test.h"

static ST_VifRunRandomFunc_t gstVifRunRandomFunc;
static ST_IspRunRandomFunc_t gstIspRunRandomFunc;
static ST_SclRunRandomFunc_t gstSclRunRandomFunc;

extern ST_VifModAttr_t  gstVifModule;
extern ST_IspModeAttr_t gstIspModule;
extern ST_SclModeAttr_t gstSclModule;

MI_S32 ST_SaveValidParam()
{
    ST_VifModAttr_t  * pstVifModAttr = &gstVifModule;
    MI_U32  u32VifGroupId;
    MI_U32  u32VifDevIdPerGroup;
    MI_U32  u32VifDevId;
    MI_U32  u32VifChnId = 0;
    MI_U32  u32VifOutPortId;

    ST_IspModeAttr_t * pstIspModeAttr = &gstIspModule;
    MI_U32  u32IspDevId;
    MI_U32  u32IspChnId;
    MI_U32  u32IspOutPortId;
    MI_U32  u32ResNum;

    ST_SclModeAttr_t * pstSclModeAttr = &gstSclModule;
    MI_U32  u32SclDevId;
    MI_U32  u32SclChnId;
    MI_U32  u32SclOutPortId;
    MI_U32  u32SclResNum;

    srand((unsigned)time(NULL)); //init random number seed

    if(pstVifModAttr->stRandomParamAttr.bParamRandomTest == TRUE)
    {
        for(u32VifGroupId=0; u32VifGroupId<ST_MAX_VIF_GROUP_NUM; u32VifGroupId++)
        {
            if(pstVifModAttr->stVifGroupAttr[u32VifGroupId].bUsed == TRUE)
            {
                for(u32VifDevIdPerGroup=0; u32VifDevIdPerGroup<ST_MAX_VIF_DEV_PERGROUP; u32VifDevIdPerGroup++)
                {
                    if(pstVifModAttr->stVifGroupAttr[u32VifGroupId].stVifDevAttr[u32VifDevIdPerGroup].bUsed == TRUE)
                    {
                        for(u32VifOutPortId=0; u32VifOutPortId<ST_MAX_VIF_OUTPORT_NUM; u32VifOutPortId++)
                        {
                            if(pstVifModAttr->stVifGroupAttr[u32VifGroupId].stVifDevAttr[u32VifDevIdPerGroup].stVifOutPortAttr[u32VifOutPortId].bUsed == TRUE)
                            {
                                u32ResNum = pstVifModAttr->stVifParamSave.u32ValidVifResNum;
                                u32VifDevId = u32VifGroupId*ST_MAX_VIF_DEV_PERGROUP+u32VifDevIdPerGroup;
                                pstVifModAttr->stVifParamSave.stValidVifResId[u32ResNum].eModId    = E_MI_MODULE_ID_VIF;
                                pstVifModAttr->stVifParamSave.stValidVifResId[u32ResNum].u32DevId  = u32VifDevId;
                                pstVifModAttr->stVifParamSave.stValidVifResId[u32ResNum].u32ChnId  = u32VifChnId;
                                pstVifModAttr->stVifParamSave.stValidVifResId[u32ResNum].u32PortId = u32VifOutPortId;
                                pstVifModAttr->stVifParamSave.u32ValidVifResNum++;
                            }
                        }
                    }
                }
            }
        }
    }

    if(pstIspModeAttr->stRandomParamAttr.bParamRandomTest == TRUE)
    {
        for(u32IspDevId=0; u32IspDevId<ST_MAX_ISP_DEV_NUM; u32IspDevId++)
        {
            if(pstIspModeAttr->stIspDevAttr[u32IspDevId].bUsed == TRUE)
            {
                for(u32IspChnId=0; u32IspChnId<ST_MAX_ISP_CHN_NUM; u32IspChnId++)
                {
                    if(pstIspModeAttr->stIspDevAttr[u32IspDevId].stIspChnlAttr[u32IspChnId].bUsed == TRUE)
                    {
                        for(u32IspOutPortId=0; u32IspOutPortId<ST_MAX_ISP_OUTPORT_NUM; u32IspOutPortId++)
                        {
                            if(pstIspModeAttr->stIspDevAttr[u32IspDevId].stIspChnlAttr[u32IspChnId].stIspOutPortAttr[u32IspOutPortId].bUsed == TRUE)
                            {
                                u32ResNum = pstIspModeAttr->stIspParamSave.u32ValidIspResNum;
                                pstIspModeAttr->stIspParamSave.stValidIspResId[u32ResNum].eModId    = E_MI_MODULE_ID_ISP;
                                pstIspModeAttr->stIspParamSave.stValidIspResId[u32ResNum].u32DevId  = u32IspDevId;
                                pstIspModeAttr->stIspParamSave.stValidIspResId[u32ResNum].u32ChnId  = u32IspChnId;
                                pstIspModeAttr->stIspParamSave.stValidIspResId[u32ResNum].u32PortId = u32IspOutPortId;
                                pstIspModeAttr->stIspParamSave.u32ValidIspResNum++;
                            }
                        }
                    }
                }
            }
        }
    }

    if(pstSclModeAttr->stRandomParamAttr.bParamRandomTest == TRUE)
    {
        for(u32SclDevId=0; u32SclDevId<ST_MAX_SCL_DEV_NUM; u32SclDevId++)
        {
            if(pstSclModeAttr->stSclDevAttr[u32SclDevId].bUsed == TRUE)
            {
                for(u32SclChnId=0; u32SclChnId<ST_MAX_SCL_CHN_NUM; u32SclChnId++)
                {
                    if(pstSclModeAttr->stSclDevAttr[u32SclDevId].stSclChnlAttr[u32SclChnId].bUsed == TRUE)
                    {
                        for(u32SclOutPortId=0; u32SclOutPortId<ST_MAX_SCL_OUTPORT_NUM; u32SclOutPortId++)
                        {
                            if(pstSclModeAttr->stSclDevAttr[u32SclDevId].stSclChnlAttr[u32SclChnId].stSclOutPortAttr[u32SclOutPortId].bUsed == TRUE)
                            {
                                u32SclResNum = pstSclModeAttr->stSclParamSave.u32ValidSclResNum;
                                pstSclModeAttr->stSclParamSave.stValidSclResId[u32SclResNum].eModId    = E_MI_MODULE_ID_SCL;
                                pstSclModeAttr->stSclParamSave.stValidSclResId[u32SclResNum].u32DevId  = u32SclDevId;
                                pstSclModeAttr->stSclParamSave.stValidSclResId[u32SclResNum].u32ChnId  = u32SclChnId;
                                pstSclModeAttr->stSclParamSave.stValidSclResId[u32SclResNum].u32PortId = u32SclOutPortId;
                                pstSclModeAttr->stSclParamSave.u32ValidSclResNum++;
                            }
                        }
                    }
                }
            }
        }
    }

    return MI_SUCCESS;
}

MI_S32 ST_ParseRandomTestIni(char *pIniPath)
{
    ST_VifRunRandomFunc_t *pstVifRunRandomFunc = &gstVifRunRandomFunc;
    ST_IspRunRandomFunc_t *pstIspRunRandomFunc = &gstIspRunRandomFunc;
    ST_SclRunRandomFunc_t *pstSclRunRandomFunc = &gstSclRunRandomFunc;

    ST_VifModAttr_t  *pstVifModAttr  = &gstVifModule;
    ST_IspModeAttr_t *pstIspModeAttr = &gstIspModule;
    ST_SclModeAttr_t *pstSclModeAttr = &gstSclModule;
    char  *string = NULL;
    MI_S32 s32Ret = MI_SUCCESS;

    dictionary *pstDict = iniparser_load(pIniPath);
    if(pstDict == NULL)
    {
        return -1;
    }

    printf("ParseRandomTestIni pstDict %p \n", pstDict);


    pstVifModAttr->stRandomParamAttr.bParamRandomTest  = (MI_BOOL)iniparser_getint(pstDict,":VifRandomParam",0);
    pstIspModeAttr->stRandomParamAttr.bParamRandomTest = (MI_BOOL)iniparser_getint(pstDict,":IspRandomParam",0);
    pstSclModeAttr->stRandomParamAttr.bParamRandomTest = (MI_BOOL)iniparser_getint(pstDict,":SclRandomParam",0);

    if(pstVifModAttr->stRandomParamAttr.bParamRandomTest == TRUE)
    {
        pstVifRunRandomFunc->u32RunNum      = iniparser_getint(pstDict,":VifRandomParamNum",0);
        pstVifRunRandomFunc->eRandomState   = (ST_RandomState_e)iniparser_getint(pstDict,":VifRandomState",E_RANDOM_STATE_MIX_TEST);
        pstVifRunRandomFunc->bRandomAllFunc = (MI_BOOL)iniparser_getint(pstDict,":VifRandomAllFuncEnable",0);
        pstVifRunRandomFunc->u32DumpBuffNum = iniparser_getint(pstDict,":VifDumpBuffNum",0);
        string = iniparser_getstring(pstDict, ":VifOutPutPath", (char *)"NULL");
        if(!strcmp((const char *)string, (const char *)"NULL"))
        {
            printf("VifRandom End DumpOutPutFile_Path NULL \n");
        }
        else
        {
            sprintf(pstVifRunRandomFunc->FilePath, "%s/", string);
            printf("VifRandom End DumpOutPutFile_Path:%s \n",pstVifRunRandomFunc->FilePath);

            /***********************************************
            Mkdir path here to avoid running at the same time
            when Multi-threading and Multi-ports dumpfile
            ************************************************/
            s32Ret = ST_CheckMkdirOutFile(pstVifRunRandomFunc->FilePath);
            if(s32Ret != MI_SUCCESS)
            {
                goto EXIT;
            }
        }

        pstVifRunRandomFunc->bRandomDevCrop   = (MI_BOOL)iniparser_getint(pstDict,":VifRandomDevCropEnable",0);
        pstVifRunRandomFunc->u32DevCropMaxW   = iniparser_getint(pstDict,":VifDevCropMaxW",ST_VIF_DEV_CROP_MAX_W);
        pstVifRunRandomFunc->u32DevCropMaxH   = iniparser_getint(pstDict,":VifDevCropMaxH",ST_VIF_DEV_CROP_MAX_H);
        pstVifRunRandomFunc->u32DevCropLimitW = iniparser_getint(pstDict,":VifDevCropLimitW",ST_ISP_AE_LIMIT_W);
        pstVifRunRandomFunc->u32DevCropLimitH = iniparser_getint(pstDict,":VifDevCropLimitH",ST_ISP_AE_LIMIT_H);

        pstVifRunRandomFunc->bRandomDevPixel = (MI_BOOL)iniparser_getint(pstDict,":VifRandomDevPixelEnable",0);
        pstVifRunRandomFunc->bRandomHdn      = (MI_BOOL)iniparser_getint(pstDict,":VifRandomHdnEnable",0);
        pstVifRunRandomFunc->bRandomField    = (MI_BOOL)iniparser_getint(pstDict,":VifRandomFieldEnable",0);

        pstVifRunRandomFunc->bRandomOutputCrop   = (MI_BOOL)iniparser_getint(pstDict,":VifRandomOutputCropEnable",0);
        pstVifRunRandomFunc->u32OutputCropMaxW   = iniparser_getint(pstDict,":VifOutputCropMaxW",ST_VIF_OUTPORT1_CROP_MAX_W);
        pstVifRunRandomFunc->u32OutputCropMaxH   = iniparser_getint(pstDict,":VifOutputCropMaxH",ST_VIF_OUTPORT1_CROP_MAX_H);
        pstVifRunRandomFunc->u32OutputCropLimitW = iniparser_getint(pstDict,":VifOutputCropLimitW",ST_ISP_AE_LIMIT_W);
        pstVifRunRandomFunc->u32OutputCropLimitH = iniparser_getint(pstDict,":VifOutputCropLimitH",ST_ISP_AE_LIMIT_H);

        pstVifRunRandomFunc->bRandomOutputSize   = (MI_BOOL)iniparser_getint(pstDict,":VifRandomOutputSizeEnable",0);
        pstVifRunRandomFunc->u32OutputSizeMaxW   = iniparser_getint(pstDict,":VifOutputSizeMaxW",ST_VIF_OUTPORT1_SIZE_MAX_W);
        pstVifRunRandomFunc->u32OutputSizeMaxH   = iniparser_getint(pstDict,":VifOutputSizeMaxH",ST_VIF_OUTPORT1_SIZE_MAX_H);
        pstVifRunRandomFunc->u32OutputSizeLimitW = iniparser_getint(pstDict,":VifOutputSizeLimitW",ST_ISP_AE_LIMIT_W);
        pstVifRunRandomFunc->u32OutputSizeLimitH = iniparser_getint(pstDict,":VifOutputSizeLimitH",ST_ISP_AE_LIMIT_H);

        pstVifRunRandomFunc->bRandomFramerate   = (MI_BOOL)iniparser_getint(pstDict,":VifRandomFramerateEnable",0);
        pstVifRunRandomFunc->bRandomOutputPixel = (MI_BOOL)iniparser_getint(pstDict,":VifRandomOutputPixelEnable",0);
    }

    if(pstIspModeAttr->stRandomParamAttr.bParamRandomTest == TRUE)
    {
        pstIspRunRandomFunc->u32RunNum      = iniparser_getint(pstDict,":IspRandomParamNum",0);
        pstIspRunRandomFunc->eRandomState   = (ST_RandomState_e)iniparser_getint(pstDict,":IspRandomState",E_RANDOM_STATE_MIX_TEST);
        pstIspRunRandomFunc->bRandomAllFunc = (MI_BOOL)iniparser_getint(pstDict,":IspRandomAllFuncEnable",0);
        pstIspRunRandomFunc->u32DumpBuffNum = iniparser_getint(pstDict,":IspDumpBuffNum",0);
        string = iniparser_getstring(pstDict, ":IspOutPutPath", (char *)"NULL");
        if(!strcmp((const char *)string, (const char *)"NULL"))
        {
            printf("IspRandom End DumpOutPutFile_Path NULL \n");
        }
        else
        {
            sprintf(pstIspRunRandomFunc->FilePath, "%s/", string);
            printf("IspRandom End DumpOutPutFile_Path:%s \n",pstIspRunRandomFunc->FilePath);

            /***********************************************
            Mkdir path here to avoid running at the same time
            when Multi-threading and Multi-ports dumpfile
            ************************************************/
            s32Ret = ST_CheckMkdirOutFile(pstIspRunRandomFunc->FilePath);
            if(s32Ret != MI_SUCCESS)
            {
                goto EXIT;
            }
        }

        pstIspRunRandomFunc->bRandomInputCrop   = (MI_BOOL)iniparser_getint(pstDict,":IspRandomInputCropEnable",0);
        pstIspRunRandomFunc->u32InputCropMaxW   = iniparser_getint(pstDict,":IspInputCropMaxW",ST_ISP_INPUTCROP_MAX_W);
        pstIspRunRandomFunc->u32InputCropMaxH   = iniparser_getint(pstDict,":IspInputCropMaxH",ST_ISP_INPUTCROP_MAX_H);
        pstIspRunRandomFunc->u32InputCropLimitW = iniparser_getint(pstDict,":IspInputCropLimitW",ST_ISP_AE_LIMIT_W);
        pstIspRunRandomFunc->u32InputCropLimitH = iniparser_getint(pstDict,":IspInputCropLimitH",ST_ISP_AE_LIMIT_H);

        pstIspRunRandomFunc->bRandom3dnr       = (MI_BOOL)iniparser_getint(pstDict,":IspRandom3dnrEnable",0);
        pstIspRunRandomFunc->bRandomRot        = (MI_BOOL)iniparser_getint(pstDict,":IspRandomRotEnable",0);
        pstIspRunRandomFunc->bRandomMirrorFlip = (MI_BOOL)iniparser_getint(pstDict,":IspRandomMirrorFlipEnable",0);

        pstIspRunRandomFunc->bRandomPixel = (MI_BOOL)iniparser_getint(pstDict,":IspRandomPixelEnable",0);
        pstIspRunRandomFunc->bRandomOutputCrop   = (MI_BOOL)iniparser_getint(pstDict,":IspRandomOutputCropEnable",0);
        pstIspRunRandomFunc->u32OutputCropMaxW   = iniparser_getint(pstDict,":IspOutputCropMaxW",ST_ISP_OUTPUTCROP_MAX_W);
        pstIspRunRandomFunc->u32OutputCropMaxH   = iniparser_getint(pstDict,":IspOutputCropMaxH",ST_ISP_OUTPUTCROP_MAX_H);
        pstIspRunRandomFunc->u32OutputCropLimitW = iniparser_getint(pstDict,":IspOutputCropLimitW",ST_ISP_OUTPUTCROP_LIMIT_W);
        pstIspRunRandomFunc->u32OutputCropLimitH = iniparser_getint(pstDict,":IspOutputCropLimitH",ST_ISP_OUTPUTCROP_LIMIT_H);

    }

    if(pstSclModeAttr->stRandomParamAttr.bParamRandomTest == TRUE)
    {
        pstSclRunRandomFunc->u32RunNum      = iniparser_getint(pstDict,":SclRandomParamNum",0);
        pstSclRunRandomFunc->eRandomState   = (ST_RandomState_e)iniparser_getint(pstDict,":SclRandomState",E_RANDOM_STATE_MIX_TEST);
        pstSclRunRandomFunc->bRandomAllFunc = (MI_BOOL)iniparser_getint(pstDict,":SclRandomAllFuncEnable",0);
        pstSclRunRandomFunc->u32DumpBuffNum = iniparser_getint(pstDict,":SclDumpBuffNum",0);
        string = iniparser_getstring(pstDict, ":SclOutPutPath", (char *)"NULL");
        if(!strcmp((const char *)string, (const char *)"NULL"))
        {
            printf("SclRandom End DumpOutPutFile_Path NULL \n");
        }
        else
        {
            sprintf(pstSclRunRandomFunc->FilePath, "%s/", string);
            printf("SclRandom End DumpOutPutFile_Path:%s \n",pstSclRunRandomFunc->FilePath);

            /***********************************************
            Mkdir path here to avoid running at the same time
            when Multi-threading and Multi-ports dumpfile
            ************************************************/
            s32Ret = ST_CheckMkdirOutFile(pstSclRunRandomFunc->FilePath);
            if(s32Ret != MI_SUCCESS)
            {
                goto EXIT;
            }
        }

        pstSclRunRandomFunc->bRandomInputCrop   = (MI_BOOL)iniparser_getint(pstDict,":SclRandomInputCropEnable",0);
        pstSclRunRandomFunc->u32InputCropMaxW   = iniparser_getint(pstDict,":SclInputCropMaxW",ST_SCL_INPUTCROP_MAX_W);
        pstSclRunRandomFunc->u32InputCropMaxH   = iniparser_getint(pstDict,":SclInputCropMaxH",ST_SCL_INPUTCROP_MAX_H);
        pstSclRunRandomFunc->u32InputCropLimitW = iniparser_getint(pstDict,":SclInputCropLimitW",ST_SCL_INPUTCROP_LIMIT_W);
        pstSclRunRandomFunc->u32InputCropLimitH = iniparser_getint(pstDict,":SclInputCropLimitH",ST_SCL_INPUTCROP_LIMIT_H);

        pstSclRunRandomFunc->bRandomChnParam   = (MI_BOOL)iniparser_getint(pstDict,":SclRandomChnParamEnable",0);
        pstSclRunRandomFunc->bRandomOutputCrop = (MI_BOOL)iniparser_getint(pstDict,":SclRandomOutputCropEnable",0);
        pstSclRunRandomFunc->u32OutputCropMaxW   = iniparser_getint(pstDict,":SclOutputCropMaxW",ST_SCL_OUTPUTCROP_MAX_W);
        pstSclRunRandomFunc->u32OutputCropMaxH   = iniparser_getint(pstDict,":SclOutputCropMaxH",ST_SCL_OUTPUTCROP_MAX_H);
        pstSclRunRandomFunc->u32OutputCropLimitW = iniparser_getint(pstDict,":SclOutputCropLimitW",ST_SCL_OUTPUTCROP_LIMIT_W);
        pstSclRunRandomFunc->u32OutputCropLimitH = iniparser_getint(pstDict,":SclOutputCropLimitH",ST_SCL_OUTPUTCROP_LIMIT_H);

        pstSclRunRandomFunc->bRandomOutputSize   = (MI_BOOL)iniparser_getint(pstDict,":SclRandomOutputSizeEnable",0);
        pstSclRunRandomFunc->u32OutputSizeMaxW   = iniparser_getint(pstDict,":SclOutputSizeMaxW",ST_SCL_OUTPUTCROP_MAX_W);
        pstSclRunRandomFunc->u32OutputSizeMaxH   = iniparser_getint(pstDict,":SclOutputSizeMaxH",ST_SCL_OUTPUTCROP_MAX_H);
        pstSclRunRandomFunc->u32OutputSizeLimitW = iniparser_getint(pstDict,":SclOutputSizeLimitW",ST_SCL_OUTPUTCROP_LIMIT_W);
        pstSclRunRandomFunc->u32OutputSizeLimitH = iniparser_getint(pstDict,":SclOutputSizeLimitH",ST_SCL_OUTPUTCROP_LIMIT_H);

        pstSclRunRandomFunc->bRandomPixel      = (MI_BOOL)iniparser_getint(pstDict,":SclRandomPixelEnable",0);
        pstSclRunRandomFunc->bRandomMirrorFlip = (MI_BOOL)iniparser_getint(pstDict,":SclRandomMirrorFlipEnable",0);
    }

EXIT:
    iniparser_freedict(pstDict);
    return MI_SUCCESS;
}

MI_S32 ST_GetRandomParam(MI_U32 u32Min, MI_U32 u32Max, ST_ParamState_e eParamState)
{
    MI_S32 s32Random;
    if(eParamState == E_RANDOM_PARAM_STATE_VALID)
    {
        s32Random = rand() % (u32Max - u32Min + 1) + u32Min;
    }
    else if(eParamState == E_RANDOM_PARAM_STATE_INVALID)
    {
        if(u32Max)
        {
            //state=0 will rand err param in the range (u32Max+1, u32Max*5)
            s32Random = rand() % (u32Max*5 - u32Max+1 + 1) + u32Max+1;
        }
        else
        {
            //u32Max=0 will rand in the range (1,5)
            s32Random = rand() % 5 + 1;
        }
    }
    return s32Random;
}

MI_S32 ST_VifRandomDevAttr(MI_SYS_ChnPort_t *pstVifChnPort, ST_ParamState_e eParamState, ST_VifRunRandomFunc_t *pstVifRunRandomFunc)
{
    MI_VIF_DEV          VifDevId    = pstVifChnPort->u32DevId;
    MI_VIF_GROUP        VifGroupId  = VifDevId/ST_MAX_VIF_GROUP_NUM;
    MI_VIF_PORT         VifPortId   = pstVifChnPort->u32PortId;

    MI_BOOL bRandomField = pstVifRunRandomFunc->bRandomField;
    MI_BOOL bRandomHdn   = pstVifRunRandomFunc->bRandomHdn;
    MI_BOOL bRandomPixel = pstVifRunRandomFunc->bRandomDevPixel;
    MI_BOOL bRandomCrop  = pstVifRunRandomFunc->bRandomDevCrop;
    MI_U32  u32CropLimitWidth  = pstVifRunRandomFunc->u32DevCropLimitW;
    MI_U32  u32CropLimitHeight = pstVifRunRandomFunc->u32DevCropLimitH;
    MI_U32  u32CropMaxWidth    = pstVifRunRandomFunc->u32DevCropMaxW;
    MI_U32  u32CropMaxHeight   = pstVifRunRandomFunc->u32DevCropMaxH;
    MI_U32  u32ChoicePixel;

    MI_VIF_DevAttr_t    stOrgVifDevAttr;
    MI_VIF_DevAttr_t    stDestVifDevAttr;
    MI_VIF_GroupAttr_t  stVifGroupAttr;
    //MI_VIF_DevStatus_t  stVifDevStatus;
    //MI_SNR_PlaneInfo_t  stSnrPlane0Info;

    memset(&stOrgVifDevAttr,   0x0, sizeof(MI_VIF_DevAttr_t));
    memset(&stDestVifDevAttr,  0x0, sizeof(MI_VIF_DevAttr_t));
    memset(&stVifGroupAttr,    0x0, sizeof(MI_VIF_GroupAttr_t));
    //memset(&stVifDevStatus,    0x0, sizeof(MI_VIF_DevStatus_t));
    //memset(&stSnrPlane0Info,   0x0, sizeof(MI_SNR_PlaneInfo_t));

    MI_VIF_DisableOutputPort(VifDevId, VifPortId);

    MI_VIF_DisableDev(VifDevId);

    MI_VIF_GetDevGroupAttr(VifGroupId,&stVifGroupAttr);

    //MI_VIF_GetDevStatus(VifDevId, &stVifDevStatus);

    //MI_SNR_GetPlaneInfo(stVifDevStatus.eSensorPadID, stVifDevStatus.u32PlaneID, &stSnrPlane0Info);

    MI_VIF_GetDevAttr(VifDevId,&stOrgVifDevAttr);

    DBG_INFO("VifRandomDevAttr devId %d, orgDevAttr [pixel %d, H2T1PMode %d, Field %d, Crop(%d,%d,%d,%d)]\n",VifDevId, stOrgVifDevAttr.eInputPixel,
        stOrgVifDevAttr.bEnH2T1PMode, stOrgVifDevAttr.eField,
        stOrgVifDevAttr.stInputRect.u16X, stOrgVifDevAttr.stInputRect.u16Y,
        stOrgVifDevAttr.stInputRect.u16Width, stOrgVifDevAttr.stInputRect.u16Height);

    memcpy(&stDestVifDevAttr, &stOrgVifDevAttr,sizeof(MI_VIF_DevAttr_t));

    if(bRandomHdn == TRUE)
    {
        if(eParamState == E_RANDOM_PARAM_STATE_INVALID || stVifGroupAttr.eIntfMode == E_MI_VIF_MODE_BT656)
        {
            stDestVifDevAttr.bEnH2T1PMode = ST_GetRandomParam(0, 1, eParamState);
        }
    }

    if(bRandomField == TRUE)
    {
        if(eParamState == E_RANDOM_PARAM_STATE_INVALID || stVifGroupAttr.eIntfMode == E_MI_VIF_MODE_BT656)
        {
            if(VifPortId == 0)
            {
                stDestVifDevAttr.eField = (MI_SYS_FieldType_e)ST_GetRandomParam(E_MI_SYS_FIELDTYPE_NONE, E_MI_SYS_FIELDTYPE_BOTH, eParamState);
            }
            else if(VifPortId == 1)
            {
                stDestVifDevAttr.eField = (MI_SYS_FieldType_e)ST_GetRandomParam(E_MI_SYS_FIELDTYPE_NONE, E_MI_SYS_FIELDTYPE_BOTTOM, eParamState);
            }
        }
    }

    if(bRandomPixel == TRUE)
    {
        if(eParamState == E_RANDOM_PARAM_STATE_INVALID || stVifGroupAttr.eIntfMode == E_MI_VIF_MODE_BT656)
        {
            u32ChoicePixel = ST_GetRandomParam(0, 3, eParamState);
            switch(u32ChoicePixel)
            {
                case 0:
                    stDestVifDevAttr.eInputPixel = E_MI_SYS_PIXEL_FRAME_YUV422_YUYV;
                    break;
                case 1:
                    stDestVifDevAttr.eInputPixel = E_MI_SYS_PIXEL_FRAME_YUV422_UYVY;
                    break;
                case 2:
                    stDestVifDevAttr.eInputPixel = E_MI_SYS_PIXEL_FRAME_YUV422_YVYU;
                    break;
                case 3:
                    stDestVifDevAttr.eInputPixel = E_MI_SYS_PIXEL_FRAME_YUV422_VYUY;
                    break;
                default:
                    stDestVifDevAttr.eInputPixel = (MI_SYS_PixelFormat_e)ST_GetRandomParam(0, E_MI_SYS_PIXEL_FRAME_FORMAT_MAX, E_RANDOM_PARAM_STATE_INVALID);
                    break;
            }
        }
    }

    if(bRandomCrop == TRUE)
    {
        if(eParamState == E_RANDOM_PARAM_STATE_INVALID || stVifGroupAttr.eIntfMode == E_MI_VIF_MODE_MIPI)
        {
            //AE LimitWH(320,256)
            stDestVifDevAttr.stInputRect.u16Width  = ST_GetRandomParam(u32CropLimitWidth,  u32CropMaxWidth, eParamState);
            stDestVifDevAttr.stInputRect.u16Height = ST_GetRandomParam(u32CropLimitHeight, u32CropMaxHeight, eParamState);
            stDestVifDevAttr.stInputRect.u16X      = ST_GetRandomParam(0, u32CropMaxWidth  - stDestVifDevAttr.stInputRect.u16Width, eParamState);
            stDestVifDevAttr.stInputRect.u16Y      = ST_GetRandomParam(0, u32CropMaxHeight - stDestVifDevAttr.stInputRect.u16Height, eParamState);
        }
    }

    DBG_INFO("VifRandomDevAttr devId %d, dsetDevAttr [pixel %d, H2T1PMode %d, Field %d, Crop(%d,%d,%d,%d)]\n",VifDevId, stDestVifDevAttr.eInputPixel,
        stDestVifDevAttr.bEnH2T1PMode, stDestVifDevAttr.eField,
        stDestVifDevAttr.stInputRect.u16X, stDestVifDevAttr.stInputRect.u16Y,
        stDestVifDevAttr.stInputRect.u16Width, stDestVifDevAttr.stInputRect.u16Height);

    MI_VIF_SetDevAttr(VifDevId,&stDestVifDevAttr);

    MI_VIF_EnableDev(VifDevId);

    MI_VIF_EnableOutputPort(VifDevId, VifPortId);

    return MI_SUCCESS;
}

MI_S32 ST_VifRandomOutputAttr(MI_SYS_ChnPort_t *pstVifChnPort, ST_ParamState_e eParamState, ST_VifRunRandomFunc_t *pstVifRunRandomFunc)
{
    MI_VIF_DEV           VifDevId    = pstVifChnPort->u32DevId;
    MI_VIF_PORT          VifPortId   = pstVifChnPort->u32PortId;
    MI_VIF_GROUP         VifGroupId  = VifDevId/ST_MAX_VIF_GROUP_NUM;

    MI_BOOL bRandomCrop  = pstVifRunRandomFunc->bRandomOutputCrop;
    MI_BOOL bRandomPixel = pstVifRunRandomFunc->bRandomOutputPixel;
    MI_BOOL bRandomFramerate   = pstVifRunRandomFunc->bRandomFramerate;
    MI_BOOL bRandomOutputSize  = pstVifRunRandomFunc->bRandomOutputSize;
    MI_U32  u32CropLimitWidth  = pstVifRunRandomFunc->u32OutputCropLimitW;
    MI_U32  u32CropLimitHeight = pstVifRunRandomFunc->u32OutputCropLimitH;
    MI_U32  u32CropMaxWidth    = pstVifRunRandomFunc->u32OutputCropMaxW;
    MI_U32  u32CropMaxHeight   = pstVifRunRandomFunc->u32OutputCropMaxH;
    MI_U32  u32OutputSizeLimitWidth  = pstVifRunRandomFunc->u32OutputSizeLimitW;
    MI_U32  u32OutputSizeLimitHeight = pstVifRunRandomFunc->u32OutputSizeLimitH;
    MI_U32  u32OutputSizeMaxWidth    = pstVifRunRandomFunc->u32OutputSizeMaxW;
    MI_U32  u32OutputSizeMaxHeight   = pstVifRunRandomFunc->u32OutputSizeMaxH;

    MI_SYS_PixelFormat_e ePixeFormatRG12B;
    MI_SYS_PixelFormat_e ePixeFormatGB12B;
    MI_VIF_DevAttr_t        stVifDevAttr;
    MI_VIF_GroupAttr_t      stVifGroupAttr;
    MI_VIF_OutputPortAttr_t stOrgVifPortAttr;
    MI_VIF_OutputPortAttr_t stDestVifPortAttr;

    memset(&stVifDevAttr,      0x0, sizeof(MI_VIF_DevAttr_t));
    memset(&stVifGroupAttr,    0x0, sizeof(MI_VIF_GroupAttr_t));
    memset(&stOrgVifPortAttr,  0x0, sizeof(MI_VIF_OutputPortAttr_t));
    memset(&stDestVifPortAttr, 0x0, sizeof(MI_VIF_OutputPortAttr_t));

     //RG12B---44 ;GB12B--47
    ePixeFormatRG12B = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_12BPP, E_MI_SYS_PIXEL_BAYERID_RG);
    ePixeFormatGB12B = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(E_MI_SYS_DATA_PRECISION_12BPP, E_MI_SYS_PIXEL_BAYERID_GB);

    MI_VIF_DisableOutputPort(VifDevId, VifPortId);

    MI_VIF_GetDevGroupAttr(VifGroupId,&stVifGroupAttr);

    MI_VIF_GetDevAttr(VifDevId,&stVifDevAttr);

    MI_VIF_GetOutputPortAttr(VifDevId, VifPortId, &stOrgVifPortAttr);

    DBG_INFO("VifRandomOutputAttr devId %d protId %d, orgVifPortAttr  [pixel %d, FrameRate %d, Crop(%d,%d,%d,%d), DestSize(%d,%d)]\n",VifDevId, VifPortId,
        stOrgVifPortAttr.ePixFormat, stOrgVifPortAttr.eFrameRate,
        stOrgVifPortAttr.stCapRect.u16X, stOrgVifPortAttr.stCapRect.u16Y,
        stOrgVifPortAttr.stCapRect.u16Width, stOrgVifPortAttr.stCapRect.u16Height,
        stOrgVifPortAttr.stDestSize.u16Width, stOrgVifPortAttr.stDestSize.u16Height);

    memcpy(&stDestVifPortAttr, &stOrgVifPortAttr, sizeof(MI_VIF_OutputPortAttr_t));

    if(bRandomFramerate == TRUE)
    {
        if(eParamState == E_RANDOM_PARAM_STATE_INVALID || stVifGroupAttr.eIntfMode == E_MI_VIF_MODE_BT656)
        {
            stDestVifPortAttr.eFrameRate = (MI_VIF_FrameRate_e)ST_GetRandomParam(E_MI_VIF_FRAMERATE_FULL, E_MI_VIF_FRAMERATE_THREE_QUARTERS, eParamState);
        }
    }

    if(bRandomCrop == TRUE)
    {
        if(eParamState == E_RANDOM_PARAM_STATE_INVALID || (stVifGroupAttr.eIntfMode == E_MI_VIF_MODE_BT656 && VifPortId == 1))
        {
            stDestVifPortAttr.stCapRect.u16Width   = ST_GetRandomParam(u32CropLimitWidth,  u32CropMaxWidth, eParamState);
            stDestVifPortAttr.stCapRect.u16Height  = ST_GetRandomParam(u32CropLimitHeight, u32CropMaxHeight, eParamState);
            stDestVifPortAttr.stCapRect.u16X       = ST_GetRandomParam(0, u32CropMaxWidth  - stDestVifPortAttr.stCapRect.u16Width, eParamState);
            stDestVifPortAttr.stCapRect.u16Y       = ST_GetRandomParam(0, u32CropMaxHeight - stDestVifPortAttr.stCapRect.u16Height, eParamState);
            stDestVifPortAttr.stDestSize.u16Width  = stDestVifPortAttr.stCapRect.u16Width;
            stDestVifPortAttr.stDestSize.u16Height = stDestVifPortAttr.stCapRect.u16Height;
        }
    }

    if(bRandomOutputSize == TRUE)
    {
        if(eParamState == E_RANDOM_PARAM_STATE_INVALID || (stVifGroupAttr.eIntfMode == E_MI_VIF_MODE_BT656 && VifPortId == 1))
        {
            u32OutputSizeMaxWidth  = (stDestVifPortAttr.stCapRect.u16Width > u32OutputSizeMaxWidth) ? u32OutputSizeMaxWidth : stDestVifPortAttr.stCapRect.u16Width;
            u32OutputSizeMaxHeight = (stDestVifPortAttr.stCapRect.u16Height > u32OutputSizeMaxHeight) ? u32OutputSizeMaxHeight : stDestVifPortAttr.stCapRect.u16Height;

            stDestVifPortAttr.stDestSize.u16Width  = ST_GetRandomParam(u32OutputSizeLimitWidth, u32OutputSizeMaxWidth, eParamState);
            stDestVifPortAttr.stDestSize.u16Height = ST_GetRandomParam(u32OutputSizeLimitHeight, u32OutputSizeMaxHeight, eParamState);
        }
    }

    if(bRandomPixel == TRUE)
    {
        stDestVifPortAttr.ePixFormat = (MI_SYS_PixelFormat_e)ST_GetRandomParam(ePixeFormatRG12B, ePixeFormatGB12B, eParamState);
    }

    DBG_INFO("VifRandomOutputAttr devId %d protId %d, destVifPortAttr  [pixel %d, FrameRate %d, Crop(%d,%d,%d,%d), DestSize(%d,%d)]\n",VifDevId, VifPortId,
        stDestVifPortAttr.ePixFormat, stDestVifPortAttr.eFrameRate,
        stDestVifPortAttr.stCapRect.u16X, stDestVifPortAttr.stCapRect.u16Y,
        stDestVifPortAttr.stCapRect.u16Width, stDestVifPortAttr.stCapRect.u16Height,
        stDestVifPortAttr.stDestSize.u16Width, stDestVifPortAttr.stDestSize.u16Height);

    MI_VIF_SetOutputPortAttr(VifDevId, VifPortId, &stDestVifPortAttr);

    MI_VIF_EnableOutputPort(VifDevId, VifPortId);

    return MI_SUCCESS;
}

MI_S32 ST_VifRandomAllFunc(MI_SYS_ChnPort_t *pstVifChnPort, ST_ParamState_e eParamState, ST_VifRunRandomFunc_t *pstVifRunRandomFunc)
{
    if(pstVifRunRandomFunc->bRandomDevCrop == TRUE
        || pstVifRunRandomFunc->bRandomField == TRUE
        || pstVifRunRandomFunc->bRandomHdn   == TRUE
        || pstVifRunRandomFunc->bRandomDevPixel == TRUE)
    {
        ST_VifRandomDevAttr(pstVifChnPort, eParamState, pstVifRunRandomFunc);
    }

    if(pstVifRunRandomFunc->bRandomOutputSize   == TRUE
        || pstVifRunRandomFunc->bRandomFramerate == TRUE
        || pstVifRunRandomFunc->bRandomOutputCrop == TRUE
        || pstVifRunRandomFunc->bRandomOutputPixel == TRUE)
    {
        ST_VifRandomOutputAttr(pstVifChnPort, eParamState, pstVifRunRandomFunc);
    }

    return MI_SUCCESS;
}

MI_S32 ST_VifGetRunCase(ST_VifRunRandomFunc_t *pstVifRunRandomFunc, ST_VifRunFuncCase_e *eSaveRandomCase)
{
    MI_U32 u32CaseNum = 0;
    if(pstVifRunRandomFunc->bRandomAllFunc == TRUE)
    {
        eSaveRandomCase[0] = E_VIF_RUN_FUNC_DEVATTR;
        eSaveRandomCase[1] = E_VIF_RUN_FUNC_OUTPUTATTR;
        if(pstVifRunRandomFunc->eRandomState != E_RANDOM_STATE_VALID_TEST)
        {
            eSaveRandomCase[2] = E_VIF_RUN_FUNC_INVALIDPARAM;
            u32CaseNum = 3;
        }
        else
        {
            u32CaseNum = 2;
        }
        pstVifRunRandomFunc->bRandomField = TRUE;
        pstVifRunRandomFunc->bRandomHdn   = TRUE;
        pstVifRunRandomFunc->bRandomDevCrop  = TRUE;
        pstVifRunRandomFunc->bRandomDevPixel = TRUE;
        pstVifRunRandomFunc->bRandomFramerate   = TRUE;
        pstVifRunRandomFunc->bRandomOutputSize  = TRUE;
        pstVifRunRandomFunc->bRandomOutputCrop  = TRUE;
        pstVifRunRandomFunc->bRandomOutputPixel = TRUE;
        return u32CaseNum;
    }

    if(pstVifRunRandomFunc->bRandomDevCrop == TRUE
        || pstVifRunRandomFunc->bRandomField == TRUE
        || pstVifRunRandomFunc->bRandomHdn   == TRUE
        || pstVifRunRandomFunc->bRandomDevPixel == TRUE)
    {
        eSaveRandomCase[u32CaseNum] = E_VIF_RUN_FUNC_DEVATTR;
        u32CaseNum++;
    }

    if(pstVifRunRandomFunc->bRandomOutputSize   == TRUE
        || pstVifRunRandomFunc->bRandomFramerate == TRUE
        || pstVifRunRandomFunc->bRandomOutputCrop == TRUE
        || pstVifRunRandomFunc->bRandomOutputPixel == TRUE)
    {
        eSaveRandomCase[u32CaseNum] = E_VIF_RUN_FUNC_OUTPUTATTR;
        u32CaseNum++;
    }

    return u32CaseNum;
}

void * ST_VifRandomParamTestThread(void * args)
{
    ST_VifParamSave_t *pstVifParamSave = (ST_VifParamSave_t *)args;
    ST_VifRunRandomFunc_t *pstVifRunRandomFunc = &gstVifRunRandomFunc;
    MI_SYS_ChnPort_t stChnPort;
    ST_ParamState_e eParamState;
    ST_VifRunFuncCase_e eSaveRandomCase[8];
    ST_VifRunFuncCase_e eRandomCase;
    MI_VIF_GROUP        VifGroupId;
    MI_VIF_DEV          DevIdPerGroup;
    MI_U32 u32CaseNum;
    MI_U32 u32CaseTemp;
    MI_U32 u32RunNum = pstVifRunRandomFunc->u32RunNum;
    MI_S32 s32Resnum;
    MI_U32 u32ChnId = 0;
    MI_U32 i;

    for(i=0; i < pstVifParamSave->u32ValidVifResNum; i++)
    {
        DBG_INFO("ValidVifDevId %d, ValidVifChnId %d, ValidVifPortId %d, Validnum %d\n",pstVifParamSave->stValidVifResId[i].u32DevId,
            pstVifParamSave->stValidVifResId[i].u32ChnId,pstVifParamSave->stValidVifResId[i].u32PortId, i);
    }

    u32CaseNum = ST_VifGetRunCase(pstVifRunRandomFunc, eSaveRandomCase);
    if(u32CaseNum <= 0)
    {
        DBG_ERR("CaseNum %d, no select case to run \n", u32CaseNum);
        return NULL;
    }

    while(u32RunNum > 0)
    {
        //step 1:random run which case
        u32CaseTemp = ST_GetRandomParam(0, u32CaseNum-1, E_RANDOM_PARAM_STATE_VALID);
        eRandomCase = eSaveRandomCase[u32CaseTemp];

        //step 2:random run case state true or false
        if(pstVifRunRandomFunc->eRandomState == E_RANDOM_STATE_MIX_TEST)
        {
            eParamState = (ST_ParamState_e)ST_GetRandomParam(E_RANDOM_PARAM_STATE_INVALID, E_RANDOM_PARAM_STATE_VALID, E_RANDOM_PARAM_STATE_VALID);
        }
        else if(pstVifRunRandomFunc->eRandomState == E_RANDOM_STATE_VALID_TEST)
        {
            eParamState = E_RANDOM_PARAM_STATE_VALID;
        }
        else if(pstVifRunRandomFunc->eRandomState == E_RANDOM_STATE_INVALID_TEST)
        {
            eParamState = E_RANDOM_PARAM_STATE_INVALID;
        }

        memset(&stChnPort, 0x0, sizeof(MI_SYS_ChnPort_t));
        //select a set of valid Dev、Chn、Port
        s32Resnum = ST_GetRandomParam(0, pstVifParamSave->u32ValidVifResNum-1, E_RANDOM_PARAM_STATE_VALID);
        stChnPort.eModId    = E_MI_MODULE_ID_VIF;
        stChnPort.u32DevId  = pstVifParamSave->stValidVifResId[s32Resnum].u32DevId;
        stChnPort.u32ChnId  = pstVifParamSave->stValidVifResId[s32Resnum].u32ChnId;
        stChnPort.u32PortId = pstVifParamSave->stValidVifResId[s32Resnum].u32PortId;

        VifGroupId    = stChnPort.u32DevId/ST_MAX_VIF_GROUP_NUM;
        DevIdPerGroup = stChnPort.u32DevId%ST_MAX_VIF_DEV_PERGROUP;
        pthread_mutex_lock(&gstVifModule.stVifGroupAttr[VifGroupId].stVifDevAttr[DevIdPerGroup].Devmutex);
        switch(eRandomCase)
        {
            case E_VIF_RUN_FUNC_INVALIDPARAM:
                //get out of range resource Id
                stChnPort.u32DevId = ST_GetRandomParam(ST_MAX_VIF_DEV_NUM, ST_MAX_VIF_DEV_NUM, E_RANDOM_PARAM_STATE_INVALID);
                stChnPort.u32ChnId = ST_GetRandomParam(u32ChnId, u32ChnId, E_RANDOM_PARAM_STATE_INVALID);
                stChnPort.u32PortId = ST_GetRandomParam(ST_MAX_VIF_OUTPORT_NUM, ST_MAX_VIF_OUTPORT_NUM, E_RANDOM_PARAM_STATE_INVALID);
                printf("run VifRandomInValidParam\n");
                ST_VifRandomAllFunc(&stChnPort, E_RANDOM_PARAM_STATE_INVALID, pstVifRunRandomFunc);
                break;
            case E_VIF_RUN_FUNC_DEVATTR:
                printf("run VifRandomDevAttr, paramstate %d\n",eParamState);
                ST_VifRandomDevAttr(&stChnPort, eParamState, pstVifRunRandomFunc);
                break;
            case E_VIF_RUN_FUNC_OUTPUTATTR:
                printf("run VifRandomOutputAttr, paramstate %d\n",eParamState);
                ST_VifRandomOutputAttr(&stChnPort, eParamState, pstVifRunRandomFunc);
                break;
            default:
                printf("case %d get fail \n", eRandomCase);
                break;
        }
        pthread_mutex_unlock(&gstVifModule.stVifGroupAttr[VifGroupId].stVifDevAttr[DevIdPerGroup].Devmutex);

        printf("VifRandomParamTestThread run num %d\n", u32RunNum--);
        usleep(THREAD_SLEEP_TIME_US*100);
    }

    if(pstVifRunRandomFunc->u32DumpBuffNum > 0)
    {
        for(i=0; i < pstVifParamSave->u32ValidVifResNum; i++)
        {
            printf("VifRandomParamTestThread run end, set valid state and dumpfile to check\n");
            ST_VifRandomAllFunc(&pstVifParamSave->stValidVifResId[i], E_RANDOM_PARAM_STATE_VALID, pstVifRunRandomFunc);
            ST_GetModuleOutputData(&pstVifParamSave->stValidVifResId[i], pstVifRunRandomFunc->FilePath, pstVifRunRandomFunc->u32DumpBuffNum);
        }
    }

    return NULL;
}

MI_S32 ST_IspRandomOutputPortParam(MI_SYS_ChnPort_t *pstIspChnPort, ST_ParamState_e eParamState, ST_IspRunRandomFunc_t *pstIspRunRandomFunc)
{
    MI_ISP_DEV     IspDevId       = pstIspChnPort->u32DevId;
    MI_ISP_CHANNEL IspChnId       = pstIspChnPort->u32ChnId;
    MI_ISP_PORT    IspOutPortId   = pstIspChnPort->u32PortId;

    MI_BOOL bRandomCrop  = pstIspRunRandomFunc->bRandomOutputCrop;
    MI_BOOL bRandomPixel = pstIspRunRandomFunc->bRandomPixel;
    MI_U32  u32CropLimitWidth  = pstIspRunRandomFunc->u32OutputCropLimitW;
    MI_U32  u32CropLimitHeight = pstIspRunRandomFunc->u32OutputCropLimitH;
    MI_U32  u32CropMaxWidth    = pstIspRunRandomFunc->u32OutputCropMaxW;
    MI_U32  u32CropMaxHeight   = pstIspRunRandomFunc->u32OutputCropMaxH;
    MI_U32  u32ChoicePixel;
    MI_ISP_OutPortParam_t stOrgIspOutputParam;
    MI_ISP_OutPortParam_t stDestIspOutputParam;
    memset(&stOrgIspOutputParam, 0x0, sizeof(MI_ISP_OutPortParam_t));
    memset(&stDestIspOutputParam, 0x0, sizeof(MI_ISP_OutPortParam_t));

    MI_ISP_DisableOutputPort(IspDevId, IspChnId, IspOutPortId);

    MI_ISP_GetOutputPortParam(IspDevId,IspChnId,IspOutPortId,&stOrgIspOutputParam);

    DBG_INFO("ISP[%d-%d-%d], orgOutputParam[pixel %d,Crop(%d,%d,%d,%d)]\n",IspDevId,IspChnId,IspOutPortId,
        stOrgIspOutputParam.ePixelFormat,
        stOrgIspOutputParam.stCropRect.u16X,stOrgIspOutputParam.stCropRect.u16Y,
        stOrgIspOutputParam.stCropRect.u16Width,stOrgIspOutputParam.stCropRect.u16Height);

    memcpy(&stDestIspOutputParam, &stOrgIspOutputParam, sizeof(MI_ISP_OutPortParam_t));

    if(bRandomCrop == TRUE)
    {
        if(eParamState == E_RANDOM_PARAM_STATE_INVALID || IspOutPortId != 0)
        {
            stDestIspOutputParam.stCropRect.u16Width  = ST_GetRandomParam(u32CropLimitWidth,  u32CropMaxWidth, eParamState);
            stDestIspOutputParam.stCropRect.u16Height = ST_GetRandomParam(u32CropLimitHeight, u32CropMaxHeight, eParamState);
            stDestIspOutputParam.stCropRect.u16X      = ST_GetRandomParam(0, u32CropMaxWidth  - stDestIspOutputParam.stCropRect.u16Width, eParamState);
            stDestIspOutputParam.stCropRect.u16Y      = ST_GetRandomParam(0, u32CropMaxHeight - stDestIspOutputParam.stCropRect.u16Height, eParamState);
        }
    }

    if(bRandomPixel == TRUE)
    {
        u32ChoicePixel = ST_GetRandomParam(0, 1, eParamState);
        switch(u32ChoicePixel)
        {
            case 0:
                stDestIspOutputParam.ePixelFormat = E_MI_SYS_PIXEL_FRAME_YUV422_YUYV;
                break;
            case 1:
                stDestIspOutputParam.ePixelFormat = E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420;
                break;
            default:
                stDestIspOutputParam.ePixelFormat = (MI_SYS_PixelFormat_e)ST_GetRandomParam(0, E_MI_SYS_PIXEL_FRAME_FORMAT_MAX, E_RANDOM_PARAM_STATE_INVALID);
                break;
        }
    }

    DBG_INFO("ISP[%d-%d-%d], DestOutputParam[pixel %d,Crop(%d,%d,%d,%d)]\n",IspDevId,IspChnId,IspOutPortId,
        stDestIspOutputParam.ePixelFormat,
        stDestIspOutputParam.stCropRect.u16X,stDestIspOutputParam.stCropRect.u16Y,
        stDestIspOutputParam.stCropRect.u16Width,stDestIspOutputParam.stCropRect.u16Height);

    MI_ISP_SetOutputPortParam(IspDevId,IspChnId,IspOutPortId,&stDestIspOutputParam);

    MI_ISP_EnableOutputPort(IspDevId, IspChnId, IspOutPortId);

    return MI_SUCCESS;
}

MI_S32 ST_IspRandomInputCrop(MI_SYS_ChnPort_t *pstIspChnPort, ST_ParamState_e eParamState, ST_IspRunRandomFunc_t *pstIspRunRandomFunc)
{
    MI_ISP_DEV     IspDevId     = pstIspChnPort->u32DevId;
    MI_ISP_CHANNEL IspChnId     = pstIspChnPort->u32ChnId;
    MI_ISP_CHANNEL IspOutPortId = pstIspChnPort->u32PortId;
    MI_U32 u32InputCropLimitWidth  = pstIspRunRandomFunc->u32InputCropLimitW;
    MI_U32 u32InputCropLimitHeight = pstIspRunRandomFunc->u32InputCropLimitH;
    MI_U32 u32InputCropMaxWidth    = pstIspRunRandomFunc->u32InputCropMaxW;
    MI_U32 u32InputCropMaxHeight   = pstIspRunRandomFunc->u32InputCropMaxH;
    MI_SYS_WindowRect_t stOrgIspInputRect;
    MI_SYS_WindowRect_t stDestIspInputRect;
    memset(&stOrgIspInputRect, 0x0, sizeof(MI_SYS_WindowRect_t));
    memset(&stDestIspInputRect, 0x0, sizeof(MI_SYS_WindowRect_t));

    MI_ISP_GetInputPortCrop(IspDevId,IspChnId,&stOrgIspInputRect);
    //step 3:random parameter value
    //AE LimitWH(320,256), if rot will (256,320),so set(320,320)
    stDestIspInputRect.u16Width    = ST_GetRandomParam(u32InputCropLimitWidth,  u32InputCropMaxWidth, eParamState);
    stDestIspInputRect.u16Height   = ST_GetRandomParam(u32InputCropLimitHeight, u32InputCropMaxHeight, eParamState);
    stDestIspInputRect.u16X        = ST_GetRandomParam(0, u32InputCropMaxWidth  - stDestIspInputRect.u16Width, eParamState);
    stDestIspInputRect.u16Y        = ST_GetRandomParam(0, u32InputCropMaxHeight - stDestIspInputRect.u16Height, eParamState);

    DBG_INFO("ISP[dev %d, chn %d, outport %d] orginputcrop(%d,%d,%d,%d),Destinputcrop(%d,%d,%d,%d)\n",IspDevId,IspChnId,IspOutPortId,
        stOrgIspInputRect.u16X,stOrgIspInputRect.u16Y,stOrgIspInputRect.u16Width,stOrgIspInputRect.u16Height,
        stDestIspInputRect.u16X,stDestIspInputRect.u16Y,stDestIspInputRect.u16Width,stDestIspInputRect.u16Height);

    MI_ISP_SetInputPortCrop(IspDevId, IspChnId, &stDestIspInputRect);

    return MI_SUCCESS;
}

MI_S32 ST_IspRandomChnParam(MI_SYS_ChnPort_t *pstIspChnPort, ST_ParamState_e eParamState, ST_IspRunRandomFunc_t *pstIspRunRandomFunc)
{
    MI_ISP_DEV     IspDevId = pstIspChnPort->u32DevId;
    MI_ISP_CHANNEL IspChnId = pstIspChnPort->u32ChnId;

    MI_BOOL bRandom3dnr       = pstIspRunRandomFunc->bRandom3dnr;
    MI_BOOL bRandomRot        = pstIspRunRandomFunc->bRandomRot;
    MI_BOOL bRandomMirrorFlip = pstIspRunRandomFunc->bRandomMirrorFlip;
    MI_ISP_ChnParam_t stOrgIspChnParam;
    MI_ISP_ChnParam_t stDestIspChnParam;
    memset(&stOrgIspChnParam,  0x0, sizeof(MI_ISP_ChnParam_t));
    memset(&stDestIspChnParam, 0x0, sizeof(MI_ISP_ChnParam_t));

    MI_ISP_StopChannel(IspDevId, IspChnId);

    MI_ISP_GetChnParam(IspDevId, IspChnId, &stOrgIspChnParam);

    DBG_INFO("OrgIspRandomChnParam IspDev %d, IspChn %d, 3DNRLevel %d, HDRType %d, Rot %d, Flip %d, Mirror %d\n", IspDevId, IspChnId,
        stOrgIspChnParam.e3DNRLevel, stOrgIspChnParam.eHDRType, stOrgIspChnParam.eRot, stOrgIspChnParam.bFlip, stOrgIspChnParam.bMirror);

    memcpy(&stDestIspChnParam, &stOrgIspChnParam, sizeof(MI_ISP_ChnParam_t));

    if(bRandom3dnr == TRUE)
    {
        //input pixel is bayer will need 3dnr, so eParamState valid, default open 3dnr
        stDestIspChnParam.e3DNRLevel = (MI_ISP_3DNR_Level_e)ST_GetRandomParam(E_MI_ISP_3DNR_LEVEL1, E_MI_ISP_3DNR_LEVEL2, eParamState);
    }

    if(bRandomRot == TRUE)
    {
        stDestIspChnParam.eRot = (MI_SYS_Rotate_e)ST_GetRandomParam(E_MI_SYS_ROTATE_NONE, E_MI_SYS_ROTATE_270, eParamState);
    }

    if(bRandomMirrorFlip == TRUE)
    {
        stDestIspChnParam.bFlip   = (MI_BOOL)ST_GetRandomParam(0, 1, eParamState);
        stDestIspChnParam.bMirror = (MI_BOOL)ST_GetRandomParam(0, 1, eParamState);
    }
    //stDestIspChnParam.eHDRType   = (MI_ISP_HDRType_e)ST_GetRandomParam(E_MI_ISP_HDR_TYPE_OFF, E_MI_ISP_HDR_TYPE_LI, eParamState);

    DBG_INFO("DestIspRandomChnParam IspDev %d, IspChn %d, 3DNRLevel %d, HDRType %d, Rot %d, Flip %d, Mirror %d\n", IspDevId, IspChnId,
        stDestIspChnParam.e3DNRLevel, stDestIspChnParam.eHDRType, stDestIspChnParam.eRot, stDestIspChnParam.bFlip, stDestIspChnParam.bMirror);

    MI_ISP_SetChnParam(IspDevId, IspChnId, &stDestIspChnParam);

    MI_ISP_StartChannel(IspDevId, IspChnId);

    return MI_SUCCESS;
}

MI_S32 ST_IspRandomAllFunc(MI_SYS_ChnPort_t *pstIspChnPort, ST_ParamState_e eParamState, ST_IspRunRandomFunc_t *pstIspRunRandomFunc)
{

    if(pstIspRunRandomFunc->bRandomRot == TRUE
        || pstIspRunRandomFunc->bRandom3dnr == TRUE
        || pstIspRunRandomFunc->bRandomMirrorFlip == TRUE)
    {
        ST_IspRandomChnParam(pstIspChnPort, eParamState, pstIspRunRandomFunc);
    }

    if(pstIspRunRandomFunc->bRandomInputCrop == TRUE)
    {
        ST_IspRandomInputCrop(pstIspChnPort, eParamState, pstIspRunRandomFunc);
    }

    if(pstIspRunRandomFunc->bRandomPixel == TRUE
        || pstIspRunRandomFunc->bRandomOutputCrop == TRUE)
    {
        ST_IspRandomOutputPortParam(pstIspChnPort, eParamState, pstIspRunRandomFunc);
    }

    return MI_SUCCESS;
}

MI_S32 ST_IspGetRunCase(ST_IspRunRandomFunc_t *pstIspRunRandomFunc, ST_IspRunFuncCase_e *eSaveRandomCase)
{
    MI_U32 u32CaseNum = 0;
    if(pstIspRunRandomFunc->bRandomAllFunc == TRUE)
    {
        eSaveRandomCase[0] = E_ISP_RUN_FUNC_INPUTCROP;
        eSaveRandomCase[1] = E_ISP_RUN_FUNC_CHNPARAM;
        eSaveRandomCase[2] = E_ISP_RUN_FUNC_OUTPUTPARAM;
        if(pstIspRunRandomFunc->eRandomState != E_RANDOM_STATE_VALID_TEST)
        {
            eSaveRandomCase[3] = E_ISP_RUN_FUNC_INVALIDPARAM;
            u32CaseNum = 4;
        }
        else
        {
            u32CaseNum = 3;
        }
        pstIspRunRandomFunc->bRandomInputCrop   = TRUE;
        pstIspRunRandomFunc->bRandom3dnr        = TRUE;
        pstIspRunRandomFunc->bRandomRot         = TRUE;
        pstIspRunRandomFunc->bRandomMirrorFlip  = TRUE;
        pstIspRunRandomFunc->bRandomOutputCrop  = TRUE;
        pstIspRunRandomFunc->bRandomPixel = TRUE;
        return u32CaseNum;
    }

    if(pstIspRunRandomFunc->bRandomInputCrop == TRUE)
    {
        eSaveRandomCase[u32CaseNum] = E_ISP_RUN_FUNC_INPUTCROP;
        u32CaseNum++;
    }

    if(pstIspRunRandomFunc->bRandom3dnr == TRUE
        || pstIspRunRandomFunc->bRandomRot == TRUE
        || pstIspRunRandomFunc->bRandomMirrorFlip == TRUE)
    {
        eSaveRandomCase[u32CaseNum] = E_ISP_RUN_FUNC_CHNPARAM;
        u32CaseNum++;
    }

    if(pstIspRunRandomFunc->bRandomPixel == TRUE
        || pstIspRunRandomFunc->bRandomOutputCrop == TRUE)
    {
        eSaveRandomCase[u32CaseNum] = E_ISP_RUN_FUNC_OUTPUTPARAM;
        u32CaseNum++;
    }

    return u32CaseNum;
}

void * ST_IspRandomParamTestThread(void * args)
{
    ST_IspParamSave_t *pstIspParamSave = (ST_IspParamSave_t *)args;
    ST_IspRunRandomFunc_t *pstIspRunRandomFunc = &gstIspRunRandomFunc;
    MI_SYS_ChnPort_t stChnPort;
    ST_ParamState_e eParamState;
    ST_IspRunFuncCase_e eSaveRandomCase[8];
    ST_IspRunFuncCase_e eRandomCase;
    MI_U32 u32CaseNum;
    MI_U32 u32CaseTemp;
    MI_U32 u32RunNum = pstIspRunRandomFunc->u32RunNum;
    MI_S32 s32Resnum;
    MI_U32 i;

    for(i=0; i < pstIspParamSave->u32ValidIspResNum; i++)
    {
        DBG_INFO("ValidIspDevId %d, ValidIspChnId %d, ValidIspPortId %d, Validnum %d\n",pstIspParamSave->stValidIspResId[i].u32DevId,
            pstIspParamSave->stValidIspResId[i].u32ChnId,pstIspParamSave->stValidIspResId[i].u32PortId, i);
    }

    u32CaseNum = ST_IspGetRunCase(pstIspRunRandomFunc, eSaveRandomCase);
    if(u32CaseNum <= 0)
    {
        DBG_ERR("CaseNum %d, no select case to run \n", u32CaseNum);
        return NULL;
    }

    while(u32RunNum > 0)
    {
        //step 1:random run which case
        u32CaseTemp = ST_GetRandomParam(0, u32CaseNum-1, E_RANDOM_PARAM_STATE_VALID);
        eRandomCase = eSaveRandomCase[u32CaseTemp];

        //step 2:random run case state true or false
        if(pstIspRunRandomFunc->eRandomState == E_RANDOM_STATE_MIX_TEST)
        {
            eParamState = (ST_ParamState_e)ST_GetRandomParam(E_RANDOM_PARAM_STATE_INVALID, E_RANDOM_PARAM_STATE_VALID, E_RANDOM_PARAM_STATE_VALID);
        }
        else if(pstIspRunRandomFunc->eRandomState == E_RANDOM_STATE_VALID_TEST)
        {
            eParamState = E_RANDOM_PARAM_STATE_VALID;
        }
        else if(pstIspRunRandomFunc->eRandomState == E_RANDOM_STATE_INVALID_TEST)
        {
            eParamState = E_RANDOM_PARAM_STATE_INVALID;
        }

        memset(&stChnPort, 0x0, sizeof(MI_SYS_ChnPort_t));
        s32Resnum = ST_GetRandomParam(0, pstIspParamSave->u32ValidIspResNum-1, E_RANDOM_PARAM_STATE_VALID);
        stChnPort.eModId    = E_MI_MODULE_ID_ISP;
        stChnPort.u32DevId  = pstIspParamSave->stValidIspResId[s32Resnum].u32DevId;
        stChnPort.u32ChnId  = pstIspParamSave->stValidIspResId[s32Resnum].u32ChnId;
        stChnPort.u32PortId = pstIspParamSave->stValidIspResId[s32Resnum].u32PortId;
        switch(eRandomCase)
        {
            case E_ISP_RUN_FUNC_INVALIDPARAM:
                //get out of range resource Id
                stChnPort.u32DevId = ST_GetRandomParam(ST_MAX_ISP_DEV_NUM, ST_MAX_ISP_DEV_NUM, E_RANDOM_PARAM_STATE_INVALID);
                stChnPort.u32ChnId = ST_GetRandomParam(ST_MAX_ISP_CHN_NUM, ST_MAX_ISP_CHN_NUM, E_RANDOM_PARAM_STATE_INVALID);
                stChnPort.u32PortId = ST_GetRandomParam(ST_MAX_ISP_OUTPORT_NUM, ST_MAX_ISP_OUTPORT_NUM, E_RANDOM_PARAM_STATE_INVALID);
                printf("run IspRandomInValidParam\n");
                ST_IspRandomAllFunc(&stChnPort, E_RANDOM_PARAM_STATE_INVALID, pstIspRunRandomFunc);
                break;
            case E_ISP_RUN_FUNC_INPUTCROP:
                printf("run IspRandomInputCrop, paramstate %d\n",eParamState);
                ST_IspRandomInputCrop(&stChnPort, eParamState, pstIspRunRandomFunc);
                break;
            case E_ISP_RUN_FUNC_CHNPARAM:
                printf("run IspRandomChnParam, paramstate %d\n",eParamState);
                ST_IspRandomChnParam(&stChnPort, eParamState, pstIspRunRandomFunc);
                break;
            case E_ISP_RUN_FUNC_OUTPUTPARAM:
                printf("run IspRandomOutputPortParam, paramstate %d\n",eParamState);
                ST_IspRandomOutputPortParam(&stChnPort, eParamState, pstIspRunRandomFunc);
                break;
            default:
                printf("case %d get fail \n", eRandomCase);
                break;
        }
        printf("IspRandomParamTestThread run num %d\n", u32RunNum--);
        usleep(THREAD_SLEEP_TIME_US*100);
    }

    if(pstIspRunRandomFunc->u32DumpBuffNum > 0)
    {
        for(i=0; i < pstIspParamSave->u32ValidIspResNum; i++)
        {
            printf("IspRandomParamTestThread run end, set valid state and dumpfile to check\n");
            ST_IspRandomAllFunc(&pstIspParamSave->stValidIspResId[i], E_RANDOM_PARAM_STATE_VALID, pstIspRunRandomFunc);
            ST_GetModuleOutputData(&pstIspParamSave->stValidIspResId[i], pstIspRunRandomFunc->FilePath, pstIspRunRandomFunc->u32DumpBuffNum);
        }
    }
    return NULL;
}

MI_BOOL ST_SclDevSupportRot(MI_SCL_DEV SclDevId)
{
    MI_BOOL bSupport = FALSE;

    if (SclDevId == E_MI_SCL_DEVID_3)
    {
        bSupport = TRUE;
    }

    return bSupport;
}

MI_S32 ST_SclRandomChnParam(MI_SYS_ChnPort_t *pstSclChnPort, ST_ParamState_e eParamState)
{
    MI_SCL_DEV     SclDevId = pstSclChnPort->u32DevId;
    MI_SCL_CHANNEL SclChnId = pstSclChnPort->u32ChnId;
    MI_SCL_CHANNEL SclPortId = pstSclChnPort->u32PortId;
    MI_SCL_ChnParam_t stSclChnParam;

    memset(&stSclChnParam, 0x0, sizeof(MI_SCL_ChnParam_t));

    MI_SCL_StopChannel(SclDevId, SclChnId);

    MI_SCL_GetChnParam(SclDevId, SclChnId, &stSclChnParam);

    DBG_INFO("OrgSclChnParam paramstate %d, SclDev %d, SclChn %d, Rot %d\n", eParamState, SclDevId, SclChnId, stSclChnParam.eRot);

    //step 3:random parameter value
    if(eParamState == E_RANDOM_PARAM_STATE_INVALID || ST_SclDevSupportRot(SclDevId) == TRUE)
    {
        stSclChnParam.eRot = (MI_SYS_Rotate_e)ST_GetRandomParam(E_MI_SYS_ROTATE_NONE, E_MI_SYS_ROTATE_270, eParamState);
    }
    else
    {
        stSclChnParam.eRot = E_MI_SYS_ROTATE_NONE;
    }

    DBG_INFO("DestSclRandomChnParam paramstate %d, SclDev %d, SclChn %d, Rot %d\n", eParamState, SclDevId, SclChnId, stSclChnParam.eRot);

    MI_SCL_SetChnParam(SclDevId, SclChnId, &stSclChnParam);

    MI_SCL_StartChannel(SclDevId, SclChnId);

    if(eParamState == E_RANDOM_PARAM_STATE_VALID && ST_SclDevSupportRot(SclDevId) == TRUE && stSclChnParam.eRot > E_MI_SYS_ROTATE_NONE)
    {
        MI_SYS_WindowRect_t stOrgSclInputRect;
        MI_SCL_OutPortParam_t stDestSclOutputParam;
        memset(&stOrgSclInputRect, 0x0, sizeof(MI_SYS_WindowRect_t));
        memset(&stDestSclOutputParam, 0x0, sizeof(MI_SCL_OutPortParam_t));

        MI_SCL_GetInputPortCrop(SclDevId, SclChnId, &stOrgSclInputRect);
        MI_SCL_GetOutputPortParam(SclDevId, SclChnId, SclPortId, &stDestSclOutputParam);

        if( stSclChnParam.eRot == E_MI_SYS_ROTATE_90 || stSclChnParam.eRot == E_MI_SYS_ROTATE_270 )
        {
            stDestSclOutputParam.stSCLOutputSize.u16Width  = stOrgSclInputRect.u16Height;
            stDestSclOutputParam.stSCLOutputSize.u16Height = stOrgSclInputRect.u16Width;
        }
        else
        {
            stDestSclOutputParam.stSCLOutputSize.u16Width  = stOrgSclInputRect.u16Width;
            stDestSclOutputParam.stSCLOutputSize.u16Height = stOrgSclInputRect.u16Height;
        }

        stDestSclOutputParam.bFlip   = FALSE;
        stDestSclOutputParam.bMirror = FALSE;
        memset(&stDestSclOutputParam.stSCLOutCropRect, 0x0, sizeof(MI_SYS_WindowRect_t));

        DBG_INFO("DestSclRandomChnParam SclDev %d, SclChn %d, outsize w %d h %d, pixel %d\n", SclDevId, SclChnId,
        stDestSclOutputParam.stSCLOutputSize.u16Width, stDestSclOutputParam.stSCLOutputSize.u16Height, stDestSclOutputParam.ePixelFormat);

        MI_SCL_DisableOutputPort(SclDevId, SclChnId, SclPortId);

        MI_SCL_SetOutputPortParam(SclDevId, SclChnId, SclPortId, &stDestSclOutputParam);

        MI_SCL_EnableOutputPort(SclDevId, SclChnId, SclPortId);
    }

    return MI_SUCCESS;
}


MI_S32 ST_SclRandomInputCropCheck( MI_SYS_WindowRect_t *stRandomSclInputRect, MI_SCL_OutPortParam_t *stSclOutputParam )
{
    //this function is to stop scaling up/down more than 16 times
    MI_U32 u32MinWidth;
    MI_U32 u32MinHeight;
    MI_U32 u32MaxWidth;
    MI_U32 u32MaxHeight;
    MI_U32 u32Width;
    MI_U32 u32Height;

    MI_U32 u32ret = MI_SUCCESS;

    u32MinWidth    = stSclOutputParam->stSCLOutputSize.u16Width / 16;
    u32MinHeight   = stSclOutputParam->stSCLOutputSize.u16Height / 16;
    u32MaxWidth    = stSclOutputParam->stSCLOutputSize.u16Width * 16;
    u32MaxHeight   = stSclOutputParam->stSCLOutputSize.u16Height * 16;

    u32Width  = stRandomSclInputRect->u16Width;
    u32Height = stRandomSclInputRect->u16Height;

    if( u32Width<u32MinWidth || u32Width>u32MaxWidth || u32Height<u32MinHeight || u32Height>u32MaxHeight )
    {
        u32ret = -1;
    }

    return u32ret;
}


MI_S32 ST_SclRandomInputCrop(MI_SYS_ChnPort_t *pstSclChnPort, ST_ParamState_e eParamState, ST_SclRunRandomFunc_t *pstSclRunRandomFunc)
{
    MI_SCL_DEV     SclDevId     = pstSclChnPort->u32DevId;
    MI_SCL_CHANNEL SclChnId     = pstSclChnPort->u32ChnId;
    MI_SCL_CHANNEL SclOutPortId = pstSclChnPort->u32PortId;
    MI_U32 u32CropLimitWidth  = pstSclRunRandomFunc->u32InputCropLimitW;
    MI_U32 u32CropLimitHeight = pstSclRunRandomFunc->u32InputCropLimitH;
    MI_U32 u32CropMaxWidth    = pstSclRunRandomFunc->u32InputCropMaxW;
    MI_U32 u32CropMaxHeight   = pstSclRunRandomFunc->u32InputCropMaxH;
    MI_SYS_WindowRect_t stOrgSclInputRect;
    MI_SYS_WindowRect_t stDestSclInputRect;
    MI_SYS_WindowRect_t stRandomSclInputRect;
    MI_SCL_OutPortParam_t stSclOutputParam;
    MI_SCL_ChnParam_t     stSclChnParam;

    memset(&stOrgSclInputRect,  0x0, sizeof(MI_SYS_WindowRect_t));
    memset(&stDestSclInputRect, 0x0, sizeof(MI_SYS_WindowRect_t));
    memset(&stSclOutputParam,   0x0, sizeof(MI_SCL_OutPortParam_t));
    memset(&stSclChnParam,      0x0, sizeof(MI_SCL_ChnParam_t));

    MI_SCL_GetInputPortCrop(SclDevId,SclChnId,&stOrgSclInputRect);
    MI_SCL_GetOutputPortParam(SclDevId, SclChnId, SclOutPortId, &stSclOutputParam);

    //step 3:random parameter value
    //LimitWH within 16 times the output size
    stRandomSclInputRect.u16Width    = ST_GetRandomParam(u32CropLimitWidth,  u32CropMaxWidth,  eParamState);
    stRandomSclInputRect.u16Height   = ST_GetRandomParam(u32CropLimitHeight, u32CropMaxHeight, eParamState);
    stRandomSclInputRect.u16X        = ST_GetRandomParam(0, u32CropMaxWidth  - stRandomSclInputRect.u16Width, eParamState);
    stRandomSclInputRect.u16Y        = ST_GetRandomParam(0, u32CropMaxHeight - stRandomSclInputRect.u16Height, eParamState);

    DBG_INFO("random inputcrop(%d, %d, %d, %d), outputsize(%d, %d)\n",
        stRandomSclInputRect.u16X,stRandomSclInputRect.u16Y,stRandomSclInputRect.u16Width,stRandomSclInputRect.u16Height,
        stSclOutputParam.stSCLOutputSize.u16Width, stSclOutputParam.stSCLOutputSize.u16Height);

    if( MI_SUCCESS == ST_SclRandomInputCropCheck(&stRandomSclInputRect,&stSclOutputParam) )
    {
        DBG_INFO("random inputcrop meet scaling up/down requirement \n");
        stDestSclInputRect.u16X = stRandomSclInputRect.u16X;
        stDestSclInputRect.u16Y = stRandomSclInputRect.u16Y;
        stDestSclInputRect.u16Width = stRandomSclInputRect.u16Width;
        stDestSclInputRect.u16Height = stRandomSclInputRect.u16Height;
    }
    else
    {
        DBG_INFO("random inputcrop no meet scaling up/down less 16 times requirement\n");
        memcpy(&stDestSclInputRect, &stOrgSclInputRect, sizeof(MI_SYS_WindowRect_t));
    }

    stDestSclInputRect.u16Width  = ALIGN_UP(stDestSclInputRect.u16Width, 2);
    stDestSclInputRect.u16Height = ALIGN_UP(stDestSclInputRect.u16Height, 2);
    MI_SCL_SetInputPortCrop(SclDevId, SclChnId, &stDestSclInputRect);

    MI_SCL_GetChnParam(SclDevId, SclChnId, &stSclChnParam);

    if(eParamState == E_RANDOM_PARAM_STATE_VALID && ST_SclDevSupportRot(SclDevId) == TRUE && stSclChnParam.eRot > E_MI_SYS_ROTATE_NONE)
    {
        MI_SCL_ChnParam_t stSclChnParam;

        memset(&stSclChnParam, 0x0, sizeof(MI_SCL_ChnParam_t));
        MI_SCL_GetChnParam(SclDevId, SclChnId, &stSclChnParam);

        if( stSclChnParam.eRot == E_MI_SYS_ROTATE_90 || stSclChnParam.eRot == E_MI_SYS_ROTATE_270 )
        {
            stSclOutputParam.stSCLOutputSize.u16Width  = stDestSclInputRect.u16Height;
            stSclOutputParam.stSCLOutputSize.u16Height = stDestSclInputRect.u16Width;
        }
        else
        {
            stSclOutputParam.stSCLOutputSize.u16Width  = stDestSclInputRect.u16Width;
            stSclOutputParam.stSCLOutputSize.u16Height = stDestSclInputRect.u16Height;
        }

        memset(&stSclOutputParam.stSCLOutCropRect, 0x0, sizeof(MI_SYS_WindowRect_t));

        MI_SCL_DisableOutputPort(SclDevId, SclChnId, SclOutPortId);

        MI_SCL_SetOutputPortParam(SclDevId, SclChnId, SclOutPortId, &stSclOutputParam);

        MI_SCL_EnableOutputPort(SclDevId, SclChnId, SclOutPortId);
    }

    DBG_INFO("SCL[dev %d, chn %d, outport %d] destinputcrop(%d, %d, %d, %d), outputsize(%d, %d)\n",SclDevId, SclChnId, SclOutPortId,
        stDestSclInputRect.u16X,stDestSclInputRect.u16Y,stDestSclInputRect.u16Width,stDestSclInputRect.u16Height,
        stSclOutputParam.stSCLOutputSize.u16Width, stSclOutputParam.stSCLOutputSize.u16Height);

    return MI_SUCCESS;
}

MI_S32 ST_SclRandomOutputSizeCheck( MI_SCL_OutPortParam_t *stRandomSclOutputParam)
{
    //this function is to stop scaling up/down more than 16 times
    MI_U32 u32MinWidth;
    MI_U32 u32MinHeight;
    MI_U32 u32MaxWidth;
    MI_U32 u32MaxHeight;
    MI_U32 u32Width;
    MI_U32 u32Height;

    MI_U32 u32ret = MI_SUCCESS;

    u32Width  = stRandomSclOutputParam->stSCLOutputSize.u16Width;
    u32Height = stRandomSclOutputParam->stSCLOutputSize.u16Height;

    u32MinWidth    = stRandomSclOutputParam->stSCLOutCropRect.u16Width / 16;
    u32MinHeight   = stRandomSclOutputParam->stSCLOutCropRect.u16Height / 16;
    u32MaxWidth    = stRandomSclOutputParam->stSCLOutCropRect.u16Width * 16;
    u32MaxHeight   = stRandomSclOutputParam->stSCLOutCropRect.u16Height * 16;

    if( u32Width<u32MinWidth || u32Width>u32MaxWidth || u32Height<u32MinHeight || u32Height>u32MaxHeight )
    {
        u32ret = -1;
    }

    return u32ret;
}


MI_S32 ST_SclRandomOutputPortParam(MI_SYS_ChnPort_t *pstSclChnPort, ST_ParamState_e eParamState, ST_SclRunRandomFunc_t *pstSclRunRandomFunc)
{
    MI_SCL_DEV     SclDevId       = pstSclChnPort->u32DevId;
    MI_SCL_CHANNEL SclChnId       = pstSclChnPort->u32ChnId;
    MI_SCL_PORT    SclOutPortId   = pstSclChnPort->u32PortId;
    MI_SCL_OutPortParam_t stOrgSclOutputParam;
    MI_SCL_OutPortParam_t stDestSclOutputParam;
    MI_SCL_OutPortParam_t stRandomSclOutputParam;
    MI_SYS_WindowRect_t   stSclInputRect;
    MI_SCL_ChnParam_t     stSclChnParam;
    MI_U32 u32ChoicePixel;

    MI_BOOL bRandomPixel      = pstSclRunRandomFunc->bRandomPixel;
    MI_BOOL bRandomOutputCrop = pstSclRunRandomFunc->bRandomOutputCrop;
    MI_BOOL bRandomOutputSize = pstSclRunRandomFunc->bRandomOutputSize;
    MI_BOOL bRandomMirrorFlip = pstSclRunRandomFunc->bRandomMirrorFlip;
    MI_U32  u32CropLimitWidth  = pstSclRunRandomFunc->u32OutputCropLimitW;
    MI_U32  u32CropLimitHeight = pstSclRunRandomFunc->u32OutputCropLimitH;
    MI_U32  u32CropMaxWidth    = pstSclRunRandomFunc->u32OutputCropMaxW;
    MI_U32  u32CropMaxHeight   = pstSclRunRandomFunc->u32OutputCropMaxH;
    MI_U32  u32OutputSizeLimitWidth  = pstSclRunRandomFunc->u32OutputSizeLimitW;
    MI_U32  u32OutputSizeLimitHeight = pstSclRunRandomFunc->u32OutputSizeLimitH;
    MI_U32  u32OutputSizeMaxWidth    = pstSclRunRandomFunc->u32OutputSizeMaxW;
    MI_U32  u32OutputSizeMaxHeight   = pstSclRunRandomFunc->u32OutputSizeMaxH;

    memset(&stOrgSclOutputParam,    0x0, sizeof(MI_SCL_OutPortParam_t));
    memset(&stDestSclOutputParam,   0x0, sizeof(MI_SCL_OutPortParam_t));
    memset(&stRandomSclOutputParam, 0x0, sizeof(MI_SCL_OutPortParam_t));
    memset(&stSclInputRect, 0x0, sizeof(MI_SYS_WindowRect_t));
    memset(&stSclChnParam,  0x0, sizeof(MI_SCL_ChnParam_t));

    MI_SCL_GetOutputPortParam(SclDevId,SclChnId,SclOutPortId,&stOrgSclOutputParam);
    MI_SCL_GetInputPortCrop(SclDevId, SclChnId, &stSclInputRect);

    DBG_INFO("SCL[%d-%d-%d], inputCropWH(%d,%d) orgOutputParam[pixel %d, mirror/flip(%d,%d) Crop(%d,%d,%d,%d), Dest(%d,%d)]\n",SclDevId,SclChnId,SclOutPortId,
        stSclInputRect.u16Width,stSclInputRect.u16Height,stOrgSclOutputParam.ePixelFormat,
        stOrgSclOutputParam.bMirror, stOrgSclOutputParam.bFlip,
        stOrgSclOutputParam.stSCLOutCropRect.u16X,stOrgSclOutputParam.stSCLOutCropRect.u16Y,
        stOrgSclOutputParam.stSCLOutCropRect.u16Width,stOrgSclOutputParam.stSCLOutCropRect.u16Height,
        stOrgSclOutputParam.stSCLOutputSize.u16Width, stOrgSclOutputParam.stSCLOutputSize.u16Height);

    MI_SCL_DisableOutputPort(SclDevId, SclChnId, SclOutPortId);

    memcpy(&stDestSclOutputParam, &stOrgSclOutputParam, sizeof(MI_SCL_OutPortParam_t));

    //step 3:random parameter value
    if(bRandomMirrorFlip == TRUE)
    {
        stDestSclOutputParam.bFlip            = (MI_BOOL)ST_GetRandomParam(0, 1, eParamState);
        stDestSclOutputParam.bMirror          = (MI_BOOL)ST_GetRandomParam(0, 1, eParamState);
    }

    if(bRandomPixel == TRUE)
    {
        u32ChoicePixel = ST_GetRandomParam(1, 12, eParamState);
        //u32ChoicePixel = 1;
        switch(u32ChoicePixel)
        {
            case 1:
                stDestSclOutputParam.ePixelFormat = E_MI_SYS_PIXEL_FRAME_YUV422_YUYV;
                break;
            case 2:
                stDestSclOutputParam.ePixelFormat = E_MI_SYS_PIXEL_FRAME_YUV422_UYVY;
                break;
            case 3:
                stDestSclOutputParam.ePixelFormat = E_MI_SYS_PIXEL_FRAME_YUV422_YVYU;
                break;
            case 4:
                stDestSclOutputParam.ePixelFormat = E_MI_SYS_PIXEL_FRAME_YUV422_VYUY;
                break;
            case 5:
                stDestSclOutputParam.ePixelFormat = E_MI_SYS_PIXEL_FRAME_YUV422_PLANAR;
                break;
            case 6:
                stDestSclOutputParam.ePixelFormat = E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_422;
                break;
            case 7:
                stDestSclOutputParam.ePixelFormat = E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420;
                break;
            case 8:
                stDestSclOutputParam.ePixelFormat = E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420_NV21;
                break;
            case 9:
                stDestSclOutputParam.ePixelFormat = E_MI_SYS_PIXEL_FRAME_YUV420_PLANAR;
                break;
            case 10:
                stDestSclOutputParam.ePixelFormat = E_MI_SYS_PIXEL_FRAME_ARGB8888;
                break;
            case 11:
                stDestSclOutputParam.ePixelFormat = E_MI_SYS_PIXEL_FRAME_ABGR8888;
                break;
            case 12:
                stDestSclOutputParam.ePixelFormat = E_MI_SYS_PIXEL_FRAME_BGRA8888;
                break;
            default:
                stDestSclOutputParam.ePixelFormat = (MI_SYS_PixelFormat_e)ST_GetRandomParam(0, E_MI_SYS_PIXEL_FRAME_FORMAT_MAX, E_RANDOM_PARAM_STATE_INVALID);
                break;
        }
    }

    u32CropLimitWidth  = stSclInputRect.u16Width;
    u32CropLimitHeight = stSclInputRect.u16Height;

    if(u32CropMaxWidth==0 || u32CropMaxHeight==0)
    {
        u32CropMaxWidth  = pstSclRunRandomFunc->u32OutputCropMaxW;
        u32CropMaxHeight = pstSclRunRandomFunc->u32OutputCropMaxH;
    }

    if(bRandomOutputCrop == TRUE)
    {
        stRandomSclOutputParam.stSCLOutCropRect.u16Width  = ST_GetRandomParam(u32CropLimitWidth,  u32CropMaxWidth,  eParamState);
        stRandomSclOutputParam.stSCLOutCropRect.u16Height = ST_GetRandomParam(u32CropLimitHeight, u32CropMaxHeight, eParamState);
        stRandomSclOutputParam.stSCLOutCropRect.u16X      = ST_GetRandomParam(0, u32CropMaxWidth  - stRandomSclOutputParam.stSCLOutCropRect.u16Width, eParamState);
        stRandomSclOutputParam.stSCLOutCropRect.u16Y      = ST_GetRandomParam(0, u32CropMaxHeight - stRandomSclOutputParam.stSCLOutCropRect.u16Height, eParamState);

        memcpy(&stDestSclOutputParam.stSCLOutCropRect, &stRandomSclOutputParam.stSCLOutCropRect, sizeof(MI_SYS_WindowRect_t));

        stDestSclOutputParam.stSCLOutputSize.u16Width = stRandomSclOutputParam.stSCLOutCropRect.u16Width;
        stDestSclOutputParam.stSCLOutputSize.u16Height = stRandomSclOutputParam.stSCLOutCropRect.u16Height;

        DBG_INFO("enable random outputcrop(%d, %d, %d, %d), outputsize(%d, %d)\n",stDestSclOutputParam.stSCLOutCropRect.u16X,stDestSclOutputParam.stSCLOutCropRect.u16Y,
            stDestSclOutputParam.stSCLOutCropRect.u16Width,stDestSclOutputParam.stSCLOutCropRect.u16Height,
            stDestSclOutputParam.stSCLOutputSize.u16Width, stDestSclOutputParam.stSCLOutputSize.u16Height);
    }

    if(bRandomOutputSize == TRUE)
    {
        stRandomSclOutputParam.stSCLOutputSize.u16Width = ST_GetRandomParam(u32OutputSizeLimitWidth,   u32OutputSizeMaxWidth, eParamState);
        stRandomSclOutputParam.stSCLOutputSize.u16Height = ST_GetRandomParam(u32OutputSizeLimitHeight, u32OutputSizeMaxHeight, eParamState);

        DBG_INFO("enable random outputsize(%d, %d)\n", stRandomSclOutputParam.stSCLOutputSize.u16Width, stRandomSclOutputParam.stSCLOutputSize.u16Height);
        if(bRandomOutputCrop)
        {
            if( MI_SUCCESS == ST_SclRandomOutputSizeCheck(&stRandomSclOutputParam) )
            {
                DBG_INFO("random outputsize meet scaling up/down requirement \n");

                stDestSclOutputParam.stSCLOutputSize.u16Width = stRandomSclOutputParam.stSCLOutputSize.u16Width;
                stDestSclOutputParam.stSCLOutputSize.u16Height = stRandomSclOutputParam.stSCLOutputSize.u16Height;
            }
            else
            {
                DBG_INFO("random outputsize no meet scaling up/down less 16 times requirement\n");
            }
        }
        else
        {
            memcpy(&stRandomSclOutputParam.stSCLOutCropRect, &stSclInputRect, sizeof(MI_SYS_WindowRect_t));
            if( MI_SUCCESS == ST_SclRandomOutputSizeCheck(&stRandomSclOutputParam) )
            {
                DBG_INFO("random outputsize meet scaling up/down requirement \n");

                stDestSclOutputParam.stSCLOutputSize.u16Width = stRandomSclOutputParam.stSCLOutputSize.u16Width;
                stDestSclOutputParam.stSCLOutputSize.u16Height = stRandomSclOutputParam.stSCLOutputSize.u16Height;
            }
            else
            {
                DBG_INFO("random outputsize no meet scaling up/down less 16 times requirement\n");
            }
        }
    }

    MI_SCL_GetChnParam(SclDevId, SclChnId, &stSclChnParam);
    if(eParamState == E_RANDOM_PARAM_STATE_VALID && ST_SclDevSupportRot(SclDevId) == TRUE && stSclChnParam.eRot > E_MI_SYS_ROTATE_NONE)
    {
        //dev3 not support sclaing/crop/pixel convert/mirror/flip
        memcpy(&stDestSclOutputParam, &stOrgSclOutputParam, sizeof(MI_SCL_OutPortParam_t));
    }

    DBG_INFO("SCL[%d-%d-%d], DestOutputParam[pixel %d, flip %d, mirror %d, OutputCrop(%d,%d,%d,%d), DestOutputSize(%d,%d)]\n",SclDevId,SclChnId,SclOutPortId,
    stDestSclOutputParam.ePixelFormat,stDestSclOutputParam.bFlip,stDestSclOutputParam.bMirror,stDestSclOutputParam.stSCLOutCropRect.u16X,stDestSclOutputParam.stSCLOutCropRect.u16Y,
    stDestSclOutputParam.stSCLOutCropRect.u16Width,stDestSclOutputParam.stSCLOutCropRect.u16Height,
    stDestSclOutputParam.stSCLOutputSize.u16Width, stDestSclOutputParam.stSCLOutputSize.u16Height);

    MI_SCL_SetOutputPortParam(SclDevId,SclChnId,SclOutPortId,&stDestSclOutputParam);

    MI_SCL_EnableOutputPort(SclDevId, SclChnId, SclOutPortId);

    return MI_SUCCESS;
}

MI_S32 ST_SclRandomAllFunc(MI_SYS_ChnPort_t *pstSclChnPort, ST_ParamState_e eParamState, ST_SclRunRandomFunc_t *pstSclRunRandomFunc)
{

    if(pstSclRunRandomFunc->bRandomChnParam == TRUE)
    {
        ST_SclRandomChnParam(pstSclChnPort, eParamState);
    }

    if(pstSclRunRandomFunc->bRandomInputCrop == TRUE)
    {
        ST_SclRandomInputCrop(pstSclChnPort, eParamState, pstSclRunRandomFunc);
    }

    if(pstSclRunRandomFunc->bRandomPixel == TRUE
        || pstSclRunRandomFunc->bRandomMirrorFlip == TRUE
        || pstSclRunRandomFunc->bRandomOutputCrop == TRUE
        || pstSclRunRandomFunc->bRandomOutputSize == TRUE)
    {
        ST_SclRandomOutputPortParam(pstSclChnPort, eParamState, pstSclRunRandomFunc);
    }

    return MI_SUCCESS;
}


MI_S32 ST_SclGetRunCase(ST_SclRunRandomFunc_t *pstSclRunRandomFunc, ST_SclRunFuncCase_e *eSaveRandomCase)
{
    MI_U32 u32CaseNum = 0;
    if(pstSclRunRandomFunc->bRandomAllFunc == TRUE)
    {
        eSaveRandomCase[0] = E_SCL_RUN_FUNC_INPUTCROP;
        eSaveRandomCase[1] = E_SCL_RUN_FUNC_CHNPARAM;
        eSaveRandomCase[2] = E_SCL_RUN_FUNC_OUTPUTPARAM;
        if(pstSclRunRandomFunc->eRandomState != E_RANDOM_STATE_VALID_TEST)
        {
            eSaveRandomCase[3] = E_SCL_RUN_FUNC_INVALIDPARAM;
            u32CaseNum = 4;
        }
        else
        {
            u32CaseNum = 3;
        }
        pstSclRunRandomFunc->bRandomInputCrop  = TRUE;
        pstSclRunRandomFunc->bRandomChnParam   = TRUE;
        pstSclRunRandomFunc->bRandomPixel      = TRUE;
        pstSclRunRandomFunc->bRandomMirrorFlip = TRUE;
        pstSclRunRandomFunc->bRandomOutputCrop = TRUE;
        pstSclRunRandomFunc->bRandomOutputSize = TRUE;
        return u32CaseNum;
    }

    if(pstSclRunRandomFunc->bRandomInputCrop == TRUE)
    {
        eSaveRandomCase[u32CaseNum] = E_SCL_RUN_FUNC_INPUTCROP;
        u32CaseNum++;
    }

    if(pstSclRunRandomFunc->bRandomChnParam == TRUE)
    {
        eSaveRandomCase[u32CaseNum] = E_SCL_RUN_FUNC_CHNPARAM;
        u32CaseNum++;
    }

    if(pstSclRunRandomFunc->bRandomPixel == TRUE
        || pstSclRunRandomFunc->bRandomOutputCrop == TRUE
        || pstSclRunRandomFunc->bRandomOutputSize == TRUE
        || pstSclRunRandomFunc->bRandomMirrorFlip ==TRUE)
    {
        eSaveRandomCase[u32CaseNum] = E_SCL_RUN_FUNC_OUTPUTPARAM;
        u32CaseNum++;
    }

    return u32CaseNum;
}

void * ST_SclRandomParamTestThread(void * args)
{
    ST_SclParamSave_t *pstSclParamSave = (ST_SclParamSave_t *)args;
    ST_SclRunRandomFunc_t *pstSclRunRandomFunc = &gstSclRunRandomFunc;
    MI_SYS_ChnPort_t stChnPort;
    ST_ParamState_e  eParamState;
    ST_SclRunFuncCase_e eSaveRandomCase[8];
    ST_SclRunFuncCase_e eRandomCase;
    MI_U32 u32CaseNum;
    MI_U32 u32CaseTemp;
    MI_U32 u32RunNum = pstSclRunRandomFunc->u32RunNum;
    MI_S32 s32Resnum;
    MI_U32 i;

    for(i=0; i < pstSclParamSave->u32ValidSclResNum; i++)
    {
        DBG_INFO("ValidSclDevId %d, ValidSclChnId %d, ValidSclPortId %d, Validnum %d\n",pstSclParamSave->stValidSclResId[i].u32DevId,
            pstSclParamSave->stValidSclResId[i].u32ChnId,pstSclParamSave->stValidSclResId[i].u32PortId, i);
    }

    u32CaseNum = ST_SclGetRunCase(pstSclRunRandomFunc, eSaveRandomCase);
    if(u32CaseNum <= 0)
    {
        DBG_ERR("CaseNum %d, no select case to run \n", u32CaseNum);
        return NULL;
    }

    while(u32RunNum > 0)
    {
        //step 1:random run which case
        u32CaseTemp = ST_GetRandomParam(0, u32CaseNum-1, E_RANDOM_PARAM_STATE_VALID);
        eRandomCase = eSaveRandomCase[u32CaseTemp];

        //step 2:random run case state true or false
        if(pstSclRunRandomFunc->eRandomState == E_RANDOM_STATE_MIX_TEST)
        {
            eParamState = (ST_ParamState_e)ST_GetRandomParam(E_RANDOM_PARAM_STATE_INVALID, E_RANDOM_PARAM_STATE_VALID, E_RANDOM_PARAM_STATE_VALID);
        }
        else if(pstSclRunRandomFunc->eRandomState == E_RANDOM_STATE_VALID_TEST)
        {
            eParamState = E_RANDOM_PARAM_STATE_VALID;
        }
        else if(pstSclRunRandomFunc->eRandomState == E_RANDOM_STATE_INVALID_TEST)
        {
            eParamState = E_RANDOM_PARAM_STATE_INVALID;
        }

        memset(&stChnPort, 0x0, sizeof(MI_SYS_ChnPort_t));
        s32Resnum = ST_GetRandomParam(0, pstSclParamSave->u32ValidSclResNum-1, E_RANDOM_PARAM_STATE_VALID);
        stChnPort.eModId    = E_MI_MODULE_ID_SCL;
        stChnPort.u32DevId  = pstSclParamSave->stValidSclResId[s32Resnum].u32DevId;
        stChnPort.u32ChnId  = pstSclParamSave->stValidSclResId[s32Resnum].u32ChnId;
        stChnPort.u32PortId = pstSclParamSave->stValidSclResId[s32Resnum].u32PortId;

        switch(eRandomCase)
        {
            case E_SCL_RUN_FUNC_INVALIDPARAM:
                //get out of range resource Id
                stChnPort.u32DevId = ST_GetRandomParam(ST_MAX_SCL_DEV_NUM, ST_MAX_SCL_DEV_NUM, E_RANDOM_PARAM_STATE_INVALID);
                stChnPort.u32ChnId = ST_GetRandomParam(ST_MAX_SCL_CHN_NUM, ST_MAX_SCL_CHN_NUM, E_RANDOM_PARAM_STATE_INVALID);
                stChnPort.u32PortId = ST_GetRandomParam(ST_MAX_SCL_OUTPORT_NUM, ST_MAX_SCL_OUTPORT_NUM, E_RANDOM_PARAM_STATE_INVALID);
                printf("run SclRandomInValidParam\n");
                ST_SclRandomAllFunc(&stChnPort, E_RANDOM_PARAM_STATE_INVALID, pstSclRunRandomFunc);
                break;
            case E_SCL_RUN_FUNC_INPUTCROP:
                printf("run SclRandomInputCrop, paramstate %d\n",eParamState);
                ST_SclRandomInputCrop(&stChnPort, eParamState, pstSclRunRandomFunc);
                break;
            case E_SCL_RUN_FUNC_CHNPARAM:
                printf("run SclRandomChnParam, paramstate %d\n",eParamState);
                ST_SclRandomChnParam(&stChnPort, eParamState);
                break;
            case E_SCL_RUN_FUNC_OUTPUTPARAM:
                printf("run SclRandomOutputPortParam, paramstate %d\n",eParamState);
                ST_SclRandomOutputPortParam(&stChnPort, eParamState, pstSclRunRandomFunc);
                break;
            default:
                printf("case %d get fail \n", eRandomCase);
                break;
        }
        printf("SclRandomParamTestThread run num %d\n", u32RunNum--);
        usleep(THREAD_SLEEP_TIME_US*100);
    }

    if(pstSclRunRandomFunc->u32DumpBuffNum > 0)
    {
        for(i=0; i < pstSclParamSave->u32ValidSclResNum; i++)
        {
            printf("SclRandomParamTestThread run end, set valid state and dumpfile to check\n");
            ST_SclRandomAllFunc(&pstSclParamSave->stValidSclResId[i], E_RANDOM_PARAM_STATE_VALID, pstSclRunRandomFunc);
            ST_GetModuleOutputData(&pstSclParamSave->stValidSclResId[i], pstSclRunRandomFunc->FilePath, pstSclRunRandomFunc->u32DumpBuffNum);
        }
    }

    return NULL;
}

