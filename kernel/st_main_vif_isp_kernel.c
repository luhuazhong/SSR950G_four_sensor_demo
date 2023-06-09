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
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include "mi_vif.h"
#include "mi_isp.h"

static struct task_struct *phtask;

#define ST_VIF_MAX_CALLBACK_NUM  (4)
#define ST_ISP_MAX_CALLBACK_NUM  (4)

#if (defined CONFIG_SIGMASTAR_CHIP_I7) && (CONFIG_SIGMASTAR_CHIP_I7 == 1)
    #define ST_MAX_VIF_DEV_PERGROUP         (4)
    #define ST_MAX_VIF_GROUP_NUM            (3)
    #define ST_MAX_VIF_DEV_NUM              (ST_MAX_VIF_GROUP_NUM * ST_MAX_VIF_DEV_PERGROUP)
    #define ST_MAX_ISP_DEV_NUM              (1)
    #define ST_SCL_STRETCHBUFF_DEVID        (3)
    #define ST_SCL_VDEC_STRETCHBUFF_CHNID   (32 + 1)
    #define ST_SCL_STRETCHBUFF_PORTID       (8)
#elif ((defined CONFIG_SIGMASTAR_CHIP_M6P) && (CONFIG_SIGMASTAR_CHIP_M6P == 1))
    #define ST_MAX_VIF_DEV_PERGROUP         (4)
    #define ST_MAX_VIF_GROUP_NUM            (4)
    #define ST_MAX_VIF_DEV_NUM              (ST_MAX_VIF_GROUP_NUM * ST_MAX_VIF_DEV_PERGROUP)
    #define ST_MAX_ISP_DEV_NUM              (1)
    #define ST_SCL_STRETCHBUFF_DEVID        (3)
    #define ST_SCL_VDEC_STRETCHBUFF_CHNID   (32 + 1)
    #define ST_SCL_STRETCHBUFF_PORTID       (6)
#else
    #define ST_MAX_VIF_DEV_PERGROUP         (0)
    #define ST_MAX_VIF_GROUP_NUM            (0)
    #define ST_MAX_VIF_DEV_NUM              (ST_MAX_VIF_GROUP_NUM * ST_MAX_VIF_DEV_PERGROUP)
    #define ST_MAX_ISP_DEV_NUM              (0)
    #define ST_SCL_STRETCHBUFF_DEVID        (0)
    #define ST_SCL_VDEC_STRETCHBUFF_CHNID   (0)
    #define ST_SCL_STRETCHBUFF_PORTID       (0)
#endif

#define SCL_PERF_TIME(pu64Time)                                                         \
do                                                                                      \
{                                                                                       \
    CamOsTimespec_t stTime1;                                                            \
    CamOsGetMonotonicTime(&stTime1);                                                    \
    pu64Time = (MI_U64)stTime1.nSec * 1000000ULL + (MI_U64)(stTime1.nNanoSec / 1000LL); \
                                                                                        \
} while (0)
#define ST_MAX_MAPCASE_NUM (4)

static int MapDumpBufCnt = 0;
#if (defined(__linux__) && ((defined MAP_CONVER) && (MAP_CONVER == 1)))
    #include <asm/uaccess.h>
    #include "mi_scl.h"
    #include "mi_sys.h"
    #include "mi_scl_internal.h"
    #define SUPPORT_MAP_CONVER (1)
    static int MapExeCaseNum = 0;
    #ifdef __KERNEL__
    module_param(MapDumpBufCnt, int, S_IRUGO | S_IWUSR);
    MODULE_PARM_DESC(MapDumpBufCnt, "Set SCL MAP Feature dump buff cnt");
    module_param(MapExeCaseNum, int, S_IRUGO | S_IWUSR);
    MODULE_PARM_DESC(MapExeCaseNum, "Set SCL MAP Feature execute case num");
    #endif
#else
    #define SUPPORT_MAP_CONVER (0)
#endif


static __attribute__((unused)) MI_U32 u32param_num =3;
static MI_U32 VifRegister[3]={0xff,0xff};//devid,funcid,param
static MI_U32 VifUnRegister[3] = {0xff,0xff};//DevId,funcid
#if ((defined VIF_REGISTER) && (VIF_REGISTER == 1))
    #define SUPPORT_VIF_REGISTER (1)
    #ifdef __KERNEL__
    module_param_array(VifRegister, int, &u32param_num, S_IRUGO|S_IWUSR);
    MODULE_PARM_DESC(VifRegister, "set vif register param");
    #endif
    #ifdef __KERNEL__
    module_param_array(VifUnRegister, int, &u32param_num, S_IRUGO|S_IWUSR);
    MODULE_PARM_DESC(VifUnRegister, "set vif unregister param");
    #endif
#else
    #define SUPPORT_VIF_REGISTER (0)
#endif


static MI_U32 ISPRegister[3]={0xff,0xff};//devid,funcid,param
static MI_U32 ISPUnRegister[3] = {0xff,0xff};//DevId,funcid
#if ((defined ISP_REGISTER) && (ISP_REGISTER == 1))
    #define SUPPORT_ISP_REGISTER (1)
    #ifdef __KERNEL__
    module_param_array(ISPRegister, int, &u32param_num, S_IRUGO|S_IWUSR);
    MODULE_PARM_DESC(ISPRegister, "set ISP register param");
    #endif
    #ifdef __KERNEL__
    module_param_array(ISPUnRegister, int, &u32param_num, S_IRUGO|S_IWUSR);
    MODULE_PARM_DESC(ISPUnRegister, "set ISP unregister param");
    #endif
#else
    #define SUPPORT_ISP_REGISTER (0)
#endif

typedef struct ST_VifCBFunc_s
{
    MI_VIF_CALLBK_FUNC  pfCBFunc;
    struct list_head pos;
} ST_VifCBFunc_t;

typedef struct ST_IspCBFunc_s
{
    MI_ISP_CALLBK_FUNC  pfCBFunc;
    struct list_head pos;
} ST_IspCBFunc_t;

ST_VifCBFunc_t gstVifDevCBListHead[ST_MAX_VIF_DEV_NUM];
ST_IspCBFunc_t gstIspDevCBListHead[ST_MAX_ISP_DEV_NUM];

MI_S32 ST_VifCallback0(MI_U64 u64Data)
{
    printk("[%s]:%d data %lld \n", __FUNCTION__,__LINE__, u64Data);
    return 0;
}

MI_S32 ST_VifCallback1(MI_U64 u64Data)
{
    printk("[%s]:%d data %lld\n", __FUNCTION__,__LINE__, u64Data);
    return 0;
}

MI_S32 ST_VifCallback2(MI_U64 u64Data)
{
    printk("[%s]:%d data %lld \n", __FUNCTION__,__LINE__, u64Data);
    return 0;
}

MI_S32 ST_VifCallback3(MI_U64 u64Data)
{
    printk("[%s]:%d data %lld \n", __FUNCTION__,__LINE__, u64Data);
    return 0;
}

MI_S32 ST_ISPCallback0(MI_U64 u64Data)
{
    printk("[%s]:%d data %lld \n", __FUNCTION__,__LINE__, u64Data);
    return 0;
}

MI_S32 ST_ISPCallback1(MI_U64 u64Data)
{
    printk("[%s]:%d data %lld\n", __FUNCTION__,__LINE__, u64Data);
    return 0;
}

MI_S32 ST_ISPCallback2(MI_U64 u64Data)
{
    printk("[%s]:%d data %lld \n", __FUNCTION__,__LINE__, u64Data);
    return 0;
}

MI_S32 ST_ISPCallback3(MI_U64 u64Data)
{
    printk("[%s]:%d data %lld \n", __FUNCTION__,__LINE__, u64Data);
    return 0;
}

MI_S32 ST_VifRegisterCallBack(MI_VIF_DEV DevId, MI_VIF_CALLBK_FUNC pfnCallBack, MI_U64 u64Data)
{
#if SUPPORT_VIF_REGISTER
    MI_VIF_CallBackParam_t stCallBackParam;
    ST_VifCBFunc_t *pstCbFuncListNode=(ST_VifCBFunc_t *)kmalloc(sizeof(ST_VifCBFunc_t), GFP_KERNEL);

    memset(&stCallBackParam, 0x0, sizeof(MI_VIF_CallBackParam_t));

    stCallBackParam.eIrqType = E_MI_VIF_IRQ_FRAMESTART;
    stCallBackParam.pfnCallBackFunc = pfnCallBack;
    stCallBackParam.eCallBackMode = E_MI_VIF_CALLBACK_ISR;
    stCallBackParam.u64Data = u64Data;
    MI_VIF_CallBackTask_Register(DevId, &stCallBackParam);

    pstCbFuncListNode->pfCBFunc = pfnCallBack;
    list_add_tail(&pstCbFuncListNode->pos, &gstVifDevCBListHead[DevId].pos);
#endif
    return 0;
}

MI_S32 ST_VifUnRegisterCallBack(MI_VIF_DEV DevId, MI_VIF_CALLBK_FUNC pfnCallBack)
{
#if SUPPORT_VIF_REGISTER
    MI_VIF_CallBackParam_t stCallBackParam;
    ST_VifCBFunc_t *head = &gstVifDevCBListHead[DevId], *node = NULL, *node_safe = NULL;

    memset(&stCallBackParam, 0x0, sizeof(MI_VIF_CallBackParam_t));
    stCallBackParam.eIrqType = E_MI_VIF_IRQ_FRAMESTART;
    stCallBackParam.pfnCallBackFunc = pfnCallBack;
    stCallBackParam.eCallBackMode = E_MI_VIF_CALLBACK_ISR;
    MI_VIF_CallBackTask_UnRegister(DevId, &stCallBackParam);

    list_for_each_entry_safe(node, node_safe, &head->pos, pos)
    {
        if(node->pfCBFunc == pfnCallBack)
        {
            list_del(&node->pos);
            kfree(node);
            node = NULL;
            break;
        }
    }
#endif

    return 0;
}

MI_S32 ST_ISPRegisterCallBack(MI_ISP_DEV DevId, MI_ISP_CALLBK_FUNC pfnCallBack, MI_U64 u64Data)
{
#if SUPPORT_ISP_REGISTER
    MI_ISP_CallBackParam_t stCallBackParam;
    ST_IspCBFunc_t *pstCbFuncListNode=(ST_IspCBFunc_t *)kmalloc(sizeof(ST_IspCBFunc_t), GFP_KERNEL);

    memset(&stCallBackParam, 0x0, sizeof(MI_ISP_CallBackParam_t));

    stCallBackParam.eIrqType = E_MI_ISP_IRQ_ISPFRAMEDONE;
    stCallBackParam.pfnCallBackFunc = pfnCallBack;
    stCallBackParam.eCallBackMode = E_MI_ISP_CALLBACK_ISR;
    stCallBackParam.u64Data = u64Data;
    MI_ISP_CallBackTask_Register(DevId, &stCallBackParam);

    pstCbFuncListNode->pfCBFunc = pfnCallBack;
    list_add_tail(&pstCbFuncListNode->pos, &gstIspDevCBListHead[DevId].pos);
#endif
    return 0;
}

MI_S32 ST_ISPUnRegisterCallBack(MI_ISP_DEV DevId, MI_ISP_CALLBK_FUNC pfnCallBack)
{
#if SUPPORT_ISP_REGISTER
    MI_ISP_CallBackParam_t stCallBackParam;
    ST_IspCBFunc_t *head = &gstIspDevCBListHead[DevId], *node = NULL, *node_safe = NULL;

    memset(&stCallBackParam, 0x0, sizeof(MI_ISP_CallBackParam_t));
    stCallBackParam.eIrqType = E_MI_ISP_IRQ_ISPFRAMEDONE;
    stCallBackParam.pfnCallBackFunc = pfnCallBack;
    stCallBackParam.eCallBackMode = E_MI_ISP_CALLBACK_ISR;

    MI_ISP_CallBackTask_Unregister(DevId, &stCallBackParam);

    list_for_each_entry_safe(node, node_safe, &head->pos, pos)
    {
        if(node->pfCBFunc == pfnCallBack)
        {
            list_del(&node->pos);
            kfree(node);
            node = NULL;
            break;
        }
    }
#endif
    return 0;
}


#if SUPPORT_MAP_CONVER

typedef struct ST_MapConverBufPHY_s
{
    MI_PHY phySrcDataAddr[3];
    MI_PHY phySrcTableAddr[3];
    MI_PHY phyDestBufAddr;
}ST_MapConverBufPHY_t;

typedef struct ST_MapConverBufPath_s
{
    MI_U8 u8Datapath[3][128];
    MI_U8 u8Tablepath[3][128];
}ST_MapConverBufPath_t;

typedef struct ST_MapConverBufInfo_s
{
    MI_U16 u16StartLine;
    MI_U16 u16TotalLines;
    MI_SYS_WindowSize_t stSrcSize;
    MI_SYS_WindowSize_t stDestSize;
    MI_SYS_WindowRect_t stCropCfg;
    MI_SYS_FrameData_PhySignalType ePhylayoutType;
    ST_MapConverBufPHY_t stMapBufPHY;
    ST_MapConverBufPath_t stMapBufPath;
}ST_MapConverBufInfo_t;

static __attribute__((unused)) ST_MapConverBufInfo_t stMapConverBufInfo[] = {
    [0] = {
        .ePhylayoutType = NORMAL_FRAME_DATA,
        .u16StartLine = 0,
        .u16TotalLines = 144,
        .stSrcSize = {
            .u16Width = 176,
            .u16Height = 144,
        },
        .stDestSize = {
            .u16Width = 176,
            .u16Height = 144,
        },
        .stCropCfg = {
            .u16X = 0,
            .u16Y = 0,
            .u16Width = 80,
            .u16Height = 72,
        },
        .stMapBufPath = {
            .u8Datapath[0] = "inplace_off_176x144/map_comp_data_y_frm.bin",
            .u8Datapath[1] = "inplace_off_176x144/map_comp_data_c_frm.bin",
            .u8Tablepath[0] = "inplace_off_176x144/map_offset_table_y_frm.bin",
            .u8Tablepath[1] = "inplace_off_176x144/map_offset_table_c_frm.bin",
        },
    },
    [1] = {
        .ePhylayoutType = RINGBUF_FRAME_DATA,
        .u16StartLine = 288,
        .u16TotalLines = 416,
        .stSrcSize = {
            .u16Width = 176,
            .u16Height = 144,
        },
        .stDestSize = {
            .u16Width = 176,
            .u16Height = 144,
        },
        .stCropCfg = {
            .u16X = 0,
            .u16Y = 0,
            .u16Width = 80,
            .u16Height = 72,
        },
        .stMapBufPath = {
            .u8Datapath[0] = "inplace_on_176x144/map_comp_data_y_frm.bin",
            .u8Datapath[1] = "inplace_on_176x144/map_comp_data_c_frm.bin",
            .u8Tablepath[0] = "inplace_on_176x144/map_offset_table_y_frm.bin",
            .u8Tablepath[1] = "inplace_on_176x144/map_offset_table_c_frm.bin",
        },
    },
    [2] = {
        .ePhylayoutType = NORMAL_FRAME_DATA,
        .u16StartLine = 0,
        .u16TotalLines = 1080,
        .stSrcSize = {
            .u16Width = 1920,
            .u16Height = 1080,
        },
        .stDestSize = {
            .u16Width = 1920,
            .u16Height = 1080,
        },
        .stCropCfg = {
            .u16X = 0,
            .u16Y = 0,
            .u16Width = 1280,
            .u16Height = 720,
        },
        .stMapBufPath = {
            .u8Datapath[0] = "inplace_off_1080p/map_comp_data_y_frm.bin",
            .u8Datapath[1] = "inplace_off_1080p/map_comp_data_c_frm.bin",
            .u8Tablepath[0] = "inplace_off_1080p/map_offset_table_y_frm.bin",
            .u8Tablepath[1] = "inplace_off_1080p/map_offset_table_c_frm.bin",
        },
    },
    [3] = {
        .ePhylayoutType = RINGBUF_FRAME_DATA,
        .u16StartLine = 1080,
        .u16TotalLines = 1344,
        .stSrcSize = {
            .u16Width = 1920,
            .u16Height = 1080,
        },
        .stDestSize = {
            .u16Width = 1920,
            .u16Height = 1080,
        },
        .stCropCfg = {
            .u16X = 0,
            .u16Y = 0,
            .u16Width = 1280,
            .u16Height = 720,
        },
        .stMapBufPath = {
            .u8Datapath[0] = "inplace_on_1080p/map_comp_data_y_frm.bin",
            .u8Datapath[1] = "inplace_on_1080p/map_comp_data_c_frm.bin",
            .u8Tablepath[0] = "inplace_on_1080p/map_offset_table_y_frm.bin",
            .u8Tablepath[1] = "inplace_on_1080p/map_offset_table_c_frm.bin",
        },
    },
};

static struct file * ST_FileOpen(MI_U8 * pu8Name,MI_U32 option)
{
    struct file *fp;

    fp =filp_open(pu8Name, option, 0777);
    if (IS_ERR(fp))
    {
        printk("Open %s Faild  PTR_ERR_fp=%ld\n",pu8Name, PTR_ERR(fp));
        return NULL;
    }

    return fp;
}
static MI_U32 ST_ReadFileFp(struct file* fp, MI_U8 *pu8Buf, MI_U32 u32Len)
{
    MI_U32 u32DataSize = 0;
    mm_segment_t oldfs = get_fs();

    set_fs(KERNEL_DS);
    u32DataSize = vfs_read(fp, pu8Buf, u32Len, &(fp->f_pos));
    set_fs(oldfs);
    return u32DataSize;
}
static MI_U32 ST_WriteFileFp(struct file* fp, MI_U8 *pu8Buf, MI_U32 u32Len)
{
    MI_U32 u32DataSize = 0;

    mm_segment_t oldfs = get_fs();
    set_fs(KERNEL_DS);
    u32DataSize = vfs_write(fp, pu8Buf, u32Len, &(fp->f_pos));
    set_fs(oldfs);
    return u32DataSize;
}

static MI_S32 ST_GetFileSize(struct file* fp)
{
    MI_S32 s32Len = 0;
    mm_segment_t oldfs = get_fs();

    set_fs(KERNEL_DS);
    fp->f_op->llseek(fp, 0L, SEEK_SET);
    s32Len = (MI_S32)fp->f_op->llseek(fp, 0L, SEEK_END);
    fp->f_op->llseek(fp, 0L, SEEK_SET);
    set_fs(oldfs);

    return s32Len;
}

static MI_S32 ST_CloseFile(struct file* fp)
{
    return filp_close(fp,NULL);
}


static MI_S32 ST_SCLStretchBuffCallBack(MI_SCL_DirectBuf_t *pstDstBuf, mi_sys_TaskStatus_e eTaskStatus)
{
    MI_S32 s32Ret = MI_SUCCESS;

    if(E_MI_SYS_TASK_DONE == eTaskStatus)
    {
        static MI_BOOL bFileOpen = FALSE;
        static struct file *fp;
        static char strDstFilePath[256] = {0};
        char strBasePath[128] = "/mnt";
        void *pBuffer = NULL;
        MI_U32 u32DataSize = 0;

        if(bFileOpen == FALSE && MapDumpBufCnt > 0)
        {
            MI_U64 u64Time;
            SCL_PERF_TIME(u64Time);
            sprintf(strDstFilePath, "%s/SCLDev%dChn%dPort%d_%dx%d_Pixel%d_%lld.yuv", strBasePath,
            ST_SCL_STRETCHBUFF_DEVID, ST_SCL_VDEC_STRETCHBUFF_CHNID,ST_SCL_STRETCHBUFF_PORTID,
            pstDstBuf->u32Width, pstDstBuf->u32Height,pstDstBuf->ePixelFormat, u64Time);

            fp = ST_FileOpen(strDstFilePath,O_CREAT|O_RDWR);
            if(fp == NULL)
            {
                printk("file %s open fail\n", strDstFilePath);
                s32Ret = MI_ERR_SCL_ILLEGAL_PARAM;
                goto EXIT;
            }
            bFileOpen = TRUE;
        }

        printk(
"=======begin writ port %d, file path %s, bufsize %d, stride [%d,%d], height %d\n", ST_SCL_STRETCHBUFF_PORTID,strDstFilePath,
                                              pstDstBuf->u32BuffSize,pstDstBuf->u32Stride[0],pstDstBuf->u32Stride[1],pstDstBuf->u32Height);

        pBuffer = mi_sys_Vmap(pstDstBuf->phyAddr[0], pstDstBuf->u32BuffSize, FALSE);
        u32DataSize = ST_WriteFileFp(fp, pBuffer, pstDstBuf->u32BuffSize);
        mi_sys_UnVmap(pBuffer);
        if(u32DataSize == pstDstBuf->u32BuffSize)
        {
            printk(
"=======end   writ port %d, file path %s size[%d] done,MapBuffCnt %d\n", ST_SCL_STRETCHBUFF_PORTID, strDstFilePath,pstDstBuf->u32BuffSize,MapDumpBufCnt);
        }
        else
        {
            printk("write file[%s] size[%d] WriteSize[%d] MapBuffCnt %d err \n ",strDstFilePath,pstDstBuf->u32BuffSize,u32DataSize,MapDumpBufCnt);
            s32Ret = MI_ERR_SCL_ILLEGAL_PARAM;
        }

        MapDumpBufCnt--;
        if(bFileOpen == TRUE && MapDumpBufCnt == 0)
        {
            ST_CloseFile(fp);
            fp = NULL;
            bFileOpen = FALSE;
            printk("close file %s\n", strDstFilePath);
        }
    }

EXIT:

    return s32Ret;
}

MI_S32 ST_SclReadFileToPHY(MI_U8 * pu8filePath,MI_U8 *pu8VirName,MI_PHY *pAddrPhy,MI_U32 *pu32FileSize)
{
    MI_S32 s32Ret = MI_SUCCESS;
    struct file *fp = NULL;

    if(pu8filePath == NULL || pu8VirName == NULL)
    {
        printk("err:pu8filePath / pu8VirName is NULL\n");
        goto EXIT;
    }

    fp = ST_FileOpen(pu8filePath,O_RDONLY);
    if(fp != NULL)
    {
        void *pBuffer = NULL;
        MI_U32 u32DataSize = 0;
        *pu32FileSize = ST_GetFileSize(fp);
        if(*pu32FileSize == 0)
        {
            printk("get file size fail \n");
            goto CLOSE_FILE;
        }

        if((*pAddrPhy) == 0)
        {
           s32Ret = mi_sys_MMA_Alloc("mma_heap_name0",pu8VirName, *pu32FileSize, pAddrPhy);
           if(s32Ret != MI_SUCCESS)
            {
                printk("alloc %s  mempory size %d fail \n",pu8VirName,*pu32FileSize);
                goto CLOSE_FILE;
            }
        }

        pBuffer = mi_sys_Vmap(*pAddrPhy, *pu32FileSize, FALSE);

        u32DataSize = ST_ReadFileFp(fp,pBuffer,*pu32FileSize);
        if(u32DataSize != *pu32FileSize)
        {
            printk("read file[%s] size[%d] ReadSize[%d]  err \n ",pu8filePath,*pu32FileSize,u32DataSize);
            s32Ret = MI_ERR_SCL_ILLEGAL_PARAM;
            goto CLOSE_FILE;
        }

        mi_sys_UnVmap(pBuffer);
    }
    else
    {
        printk("open y data failed\n");
        s32Ret = MI_ERR_SCL_ILLEGAL_PARAM;
        goto EXIT;
    }
CLOSE_FILE:
    ST_CloseFile(fp);
    fp = NULL;
EXIT:

    return s32Ret;
}
#endif

MI_S32 ST_SCLStretchMapBuf(void)
{
    MI_S32 s32Ret=MI_SUCCESS;
#if SUPPORT_MAP_CONVER

    MI_S8 s8Plane = 0;
    MI_U8 u8VaildPlane = 0;
    MI_U8 u8BufferName[256];
    MI_U8 u8BasePath[64] = "/mnt/ini/mochi/scl/mapconver/bin/";

    MI_U8 u8SrcFilePath[256];
    MI_U32 u32DataPlaneFileSize[3];
    MI_U32 u32TablePlaneFileSize[3];
    MI_U8 u8SrcDataBufName[3][64] = {"mapydatavdec","mapcdatavdec"};
    MI_U8 u8SrcTableBufName[3][64] = {"mapytablevdec","mapctablevdec"};

    MI_U32 u32SrcBuffSize = 0;
    MI_U32 u32SrcPlaneStride[3];
    MI_U32 u32SrcPlaneSize[3];
    mi_scl_DirectBuff_t stSrcVdecBuf;

    MI_U32 u32DestBuffSize = 0;
    MI_U32 u32DestPlaneSize[3];
    MI_U32 u32DestPlaneStride[3];
    mi_scl_DirectBuff_t stDstVdecBuf;

    ST_MapConverBufInfo_t *pMapConverBufInfo = NULL;
    MI_SYS_WindowSize_t *pMapSrcSize = NULL;
    MI_SYS_WindowSize_t *pMapDestSize = NULL;
    ST_MapConverBufPHY_t *pMapBufPhy = NULL;
    ST_MapConverBufPath_t *pMapBufPath = NULL;

    memset(u32SrcPlaneStride, 0x0, sizeof(MI_U32)*3);
    memset(u32DestPlaneStride, 0x0, sizeof(MI_U32)*3);
    memset(u32SrcPlaneSize, 0x0, sizeof(MI_U32)*3);
    memset(u32DestPlaneSize, 0x0, sizeof(MI_U32)*3);
    memset(u32DataPlaneFileSize, 0x0, sizeof(MI_U32)*3);
    memset(u32TablePlaneFileSize, 0x0, sizeof(MI_U32)*3);
    memset(&stSrcVdecBuf, 0x0, sizeof(mi_scl_DirectBuff_t));
    memset(&stDstVdecBuf, 0x0, sizeof(mi_scl_DirectBuff_t));

    if(MapExeCaseNum < 0 || MapExeCaseNum >= ST_MAX_MAPCASE_NUM)
    {
        printk("current case %d is bigger than Max case num %d\n",MapExeCaseNum,ST_MAX_MAPCASE_NUM);
        s32Ret = MI_ERR_SCL_ILLEGAL_PARAM;
        goto EXIT;
    }

    pMapConverBufInfo = &stMapConverBufInfo[MapExeCaseNum];
    pMapSrcSize = &pMapConverBufInfo->stSrcSize;
    pMapDestSize = &pMapConverBufInfo->stDestSize;
    pMapBufPhy = &pMapConverBufInfo->stMapBufPHY;
    pMapBufPath = &pMapConverBufInfo->stMapBufPath;

    //MAP Conver only support NV12
    u8VaildPlane = 2;//Y plane & C plane

    u32SrcPlaneStride[0] = ALIGN_UP(pMapSrcSize->u16Width,16);
    u32SrcPlaneStride[1] = ALIGN_UP(u32SrcPlaneStride[0]/2,16);
    u32SrcPlaneSize[0] = u32SrcPlaneStride[0]*pMapConverBufInfo->u16TotalLines;
    u32SrcPlaneSize[1] = u32SrcPlaneStride[1]*pMapConverBufInfo->u16TotalLines;
    u32SrcBuffSize = u32SrcPlaneSize[0] + u32SrcPlaneSize[1];

    u32DestPlaneStride[0] = pMapDestSize->u16Width;
    u32DestPlaneStride[1] = u32DestPlaneStride[0];
    u32DestPlaneSize[0] = pMapDestSize->u16Height * pMapDestSize->u16Width;
    u32DestPlaneSize[1] = u32DestPlaneSize[0]/2;
    u32DestBuffSize = u32DestPlaneSize[0] + u32DestPlaneSize[1];

    //read src data bin file by vdec
    for(s8Plane = 0; s8Plane < u8VaildPlane;s8Plane++)
    {
        memset(u8SrcFilePath, 0x0, sizeof(u8SrcFilePath));
        memset(u8BufferName, 0x0, sizeof(u8BufferName));
        sprintf(u8SrcFilePath, "%s%s", u8BasePath,pMapBufPath->u8Datapath[s8Plane]);
        sprintf(u8BufferName, "%s_%d", u8SrcDataBufName[s8Plane],MapExeCaseNum);
        s32Ret = ST_SclReadFileToPHY(u8SrcFilePath,u8BufferName,&pMapBufPhy->phySrcDataAddr[s8Plane],&u32DataPlaneFileSize[s8Plane]);
        if(s32Ret != MI_SUCCESS)
        {
            s32Ret = MI_ERR_SCL_ILLEGAL_PARAM;
            goto EXIT;
        }
    }

    //read src table bin file by vdec
    for(s8Plane = 0; s8Plane < u8VaildPlane;s8Plane++)
    {
        memset(u8SrcFilePath, 0x0, sizeof(u8SrcFilePath));
        memset(u8BufferName, 0x0, sizeof(u8BufferName));
        sprintf(u8SrcFilePath, "%s%s", u8BasePath,pMapBufPath->u8Tablepath[s8Plane]);
        sprintf(u8BufferName, "%s_%d", u8SrcTableBufName[s8Plane],MapExeCaseNum);
        s32Ret = ST_SclReadFileToPHY(u8SrcFilePath,u8BufferName,&pMapBufPhy->phySrcTableAddr[s8Plane],&u32TablePlaneFileSize[s8Plane]);
        if(s32Ret != MI_SUCCESS)
        {
            s32Ret = MI_ERR_SCL_ILLEGAL_PARAM;
            goto EXIT;
        }
    }

    //alloc dest buffer
    if(0 == pMapBufPhy->phyDestBufAddr)
    {
        memset(u8BufferName, 0x0, sizeof(u8BufferName));
        sprintf(u8BufferName, "%s_%d", "mapdestdata",MapExeCaseNum);

        s32Ret = mi_sys_MMA_Alloc("mma_heap_name0", u8BufferName, u32DestBuffSize, &pMapBufPhy->phyDestBufAddr);
        if(s32Ret != MI_SUCCESS)
        {
            printk("alloc %s mempory size %d fail \n",u8BufferName,u32DestBuffSize);
            goto EXIT;
        }
    }

    //set map cfg
    stSrcVdecBuf.eBufType = E_MI_SYS_BUFDATA_FBC;
    stSrcVdecBuf.stFbcData.ePhylayoutType = pMapConverBufInfo->ePhylayoutType;
    stSrcVdecBuf.stFbcData.ePixelFormat = E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420;
    stSrcVdecBuf.stFbcData.eCompressMode = E_MI_SYS_COMPRESS_MODE_SFBC2;
    stSrcVdecBuf.stFbcData.eTileMode = E_MI_SYS_FRAME_TILE_MODE_NONE;
    stSrcVdecBuf.stFbcData.u16Height = pMapSrcSize->u16Height;
    stSrcVdecBuf.stFbcData.u16Width = pMapSrcSize->u16Width;
    stSrcVdecBuf.stFbcData.u16RingBufStartLine = pMapConverBufInfo->u16StartLine;
    stSrcVdecBuf.stFbcData.u16RingHeapTotalLines = pMapConverBufInfo->u16TotalLines;
    stSrcVdecBuf.stFbcData.u32Stride[0] = u32SrcPlaneStride[0];
    stSrcVdecBuf.stFbcData.u32Stride[1] = u32SrcPlaneStride[1];
    stSrcVdecBuf.stFbcData.u32FbcTableSize[0] = u32TablePlaneFileSize[0];
    stSrcVdecBuf.stFbcData.u32FbcTableSize[1] = u32TablePlaneFileSize[1];
    stSrcVdecBuf.stFbcData.u32BufSize = u32SrcBuffSize;
    stSrcVdecBuf.stFbcData.phyAddr[0] = pMapBufPhy->phySrcDataAddr[0];
    stSrcVdecBuf.stFbcData.phyAddr[1] = pMapBufPhy->phySrcDataAddr[1];
    stSrcVdecBuf.stFbcData.phyFbcTableAddr[0] = pMapBufPhy->phySrcTableAddr[0];
    stSrcVdecBuf.stFbcData.phyFbcTableAddr[1] = pMapBufPhy->phySrcTableAddr[1];

    stDstVdecBuf.eBufType = E_MI_SYS_BUFDATA_FRAME;
    stDstVdecBuf.stFrameData.ePixelFormat = E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420;
    stDstVdecBuf.stFrameData.phyAddr[0] = pMapBufPhy->phyDestBufAddr;
    stDstVdecBuf.stFrameData.phyAddr[1] = (MI_PHY)(pMapBufPhy->phyDestBufAddr + u32DestPlaneSize[0]);
    stDstVdecBuf.stFrameData.u32BuffSize = u32DestBuffSize;
    stDstVdecBuf.stFrameData.u32Width = pMapDestSize->u16Width;
    stDstVdecBuf.stFrameData.u32Height = pMapDestSize->u16Height;
    stDstVdecBuf.stFrameData.u32Stride[0] = u32DestPlaneStride[0];
    stDstVdecBuf.stFrameData.u32Stride[1] = u32DestPlaneStride[1];

/*
    printk("stSrcVdecBuftype %d,pixel %d, compress %d, layouttype %d, tilemode %d, buffsize %d,wh(%d,%d),ring(%d,%d)",
    stSrcVdecBuf.eBufType,stSrcVdecBuf.stFbcData.ePixelFormat,stSrcVdecBuf.stFbcData.eCompressMode,
    stSrcVdecBuf.stFbcData.ePhylayoutType,stSrcVdecBuf.stFbcData.eTileMode,
    stSrcVdecBuf.stFbcData.u32BufSize,stSrcVdecBuf.stFbcData.u16Width,stSrcVdecBuf.stFbcData.u16Height,
    stSrcVdecBuf.stFbcData.u16RingBufStartLine,stSrcVdecBuf.stFbcData.u16RingHeapTotalLines);

    printk("phy(0x%llx,0x%llx),stride(%d,%d),phyFbcTableAddr(0x%llx,0x%llx),u32FbcTableSize(%d,%d)\n",
    stSrcVdecBuf.stFbcData.phyAddr[0],stSrcVdecBuf.stFbcData.phyAddr[1],
    stSrcVdecBuf.stFbcData.u32Stride[0],stSrcVdecBuf.stFbcData.u32Stride[1],
    stSrcVdecBuf.stFbcData.phyFbcTableAddr[0],stSrcVdecBuf.stFbcData.phyFbcTableAddr[1],
    stSrcVdecBuf.stFbcData.u32FbcTableSize[0],stSrcVdecBuf.stFbcData.u32FbcTableSize[1]);


    printk("Crop (%d,%d,%d,%d)\n",pMapConverBufInfo->stCropCfg.u16X,pMapConverBufInfo->stCropCfg.u16Y,
            pMapConverBufInfo->stCropCfg.u16Width,pMapConverBufInfo->stCropCfg.u16Height);

    printk("stDstVdecBuftype %d,pixel %d,buffsize %d,wh(%d,%d),phy(0x%llx,0x%llx),stride(%d,%d)\n",
    stDstVdecBuf.eBufType,stDstVdecBuf.stFrameData.ePixelFormat,stDstVdecBuf.stFrameData.u32BuffSize,
    stDstVdecBuf.stFrameData.u32Width,stDstVdecBuf.stFrameData.u32Height,
    stDstVdecBuf.stFrameData.phyAddr[0],stDstVdecBuf.stFrameData.phyAddr[1],
    stDstVdecBuf.stFrameData.u32Stride[0],stDstVdecBuf.stFrameData.u32Stride[1]);

*/
    if(stSrcVdecBuf.stFbcData.u32BufSize == 0 || stDstVdecBuf.stFrameData.u32BuffSize == 0)
    {
        printk("stSrcVdecBufSize %d , stDstVdecBufSize %d is err \n",
                stSrcVdecBuf.stFbcData.u32BufSize,stDstVdecBuf.stFrameData.u32BuffSize);
        goto EXIT;
    }

    mi_scl_StretchBufByVdec(&stSrcVdecBuf, &pMapConverBufInfo->stCropCfg, &stDstVdecBuf, ST_SCLStretchBuffCallBack);

EXIT:
#endif

    return s32Ret;
}

MI_VIF_CALLBK_FUNC fnVifCallBack[ST_VIF_MAX_CALLBACK_NUM]={ST_VifCallback0, ST_VifCallback1, ST_VifCallback2, ST_VifCallback3};
MI_ISP_CALLBK_FUNC fnISPCallBack[ST_ISP_MAX_CALLBACK_NUM]={ST_ISPCallback0, ST_ISPCallback1, ST_ISPCallback2, ST_ISPCallback3};

static int main_work_thread(void *data)
{
    while(0==kthread_should_stop())
    {
        if(VifRegister[0] < ST_MAX_VIF_DEV_NUM && VifRegister[1] < ST_VIF_MAX_CALLBACK_NUM)
        {
            ST_VifRegisterCallBack(VifRegister[0],fnVifCallBack[VifRegister[1]],VifRegister[2]);
            VifRegister[0]=0XFF;
            VifRegister[1]=0xFF;
        }

        if(VifUnRegister[0] < ST_MAX_VIF_DEV_NUM && VifUnRegister[1] < ST_VIF_MAX_CALLBACK_NUM)
        {
            ST_VifUnRegisterCallBack(VifUnRegister[0],fnVifCallBack[VifUnRegister[1]]);
            VifUnRegister[0]=0XFF;
            VifUnRegister[1]=0xFF;
        }

        if(ISPRegister[0] < ST_MAX_ISP_DEV_NUM && ISPRegister[1] < ST_ISP_MAX_CALLBACK_NUM)
        {
            ST_ISPRegisterCallBack(ISPRegister[0],fnISPCallBack[ISPRegister[1]],ISPRegister[2]);
            ISPRegister[0]=0XFF;
            ISPRegister[1]=0xFF;
        }

        if(ISPUnRegister[0] < ST_MAX_ISP_DEV_NUM && ISPUnRegister[1] < ST_ISP_MAX_CALLBACK_NUM)
        {
            ST_ISPUnRegisterCallBack(ISPUnRegister[0],fnISPCallBack[ISPUnRegister[1]]);
            ISPUnRegister[0]=0XFF;
            ISPUnRegister[1]=0xFF;
        }
        if(MapDumpBufCnt)
        {
            ST_SCLStretchMapBuf();
        }
        msleep(300);
    }
    printk("thread exit \n");
    return 0;
}

static int vif_isp_module_init(void)
{
    MI_U8 VifDevId=0, IspDevId=0;
    printk("vif/isp demo module init\n");

    for(VifDevId=0; VifDevId<ST_MAX_VIF_DEV_NUM; VifDevId++)
    {
        INIT_LIST_HEAD(&gstVifDevCBListHead[VifDevId].pos);
    }

    for(IspDevId=0; IspDevId<ST_MAX_ISP_DEV_NUM; IspDevId++)
    {
        INIT_LIST_HEAD(&gstIspDevCBListHead[IspDevId].pos);
    }

    phtask=kthread_run(main_work_thread, NULL, "vif/isp_demo_thread");
    printk("thread run %p\n", phtask);

    return 0;
}

static void vif_isp_module_exit(void)
{
    MI_U8 VifDevId=0, IspDevId=0,u8MapCaseNum = 0;
    printk("vif/isp demo module exit\n");

    if(phtask != NULL)
    {
        printk("thread stop %p\n", phtask);
        kthread_stop(phtask);
    }

    for(u8MapCaseNum = 0; u8MapCaseNum < ST_MAX_MAPCASE_NUM; u8MapCaseNum++)
    {
#if SUPPORT_MAP_CONVER
        MI_U8 u8PlaneNum = 0;
        ST_MapConverBufInfo_t *pMapConverBufInfo = NULL;
        pMapConverBufInfo = &stMapConverBufInfo[u8MapCaseNum];

        for(u8PlaneNum = 0; u8PlaneNum < 3; u8PlaneNum++)
        {
            if(pMapConverBufInfo->stMapBufPHY.phySrcDataAddr[u8PlaneNum] != 0)
            {
                mi_sys_MMA_Free(pMapConverBufInfo->stMapBufPHY.phySrcDataAddr[u8PlaneNum]);
            }
            if(pMapConverBufInfo->stMapBufPHY.phySrcTableAddr[u8PlaneNum] != 0)
            {
                mi_sys_MMA_Free(pMapConverBufInfo->stMapBufPHY.phySrcTableAddr[u8PlaneNum]);
            }
        }
        if(pMapConverBufInfo->stMapBufPHY.phyDestBufAddr != 0)
        {
            mi_sys_MMA_Free(pMapConverBufInfo->stMapBufPHY.phyDestBufAddr);
        }
#endif
    }

    for(IspDevId=0; IspDevId<ST_MAX_ISP_DEV_NUM; IspDevId++)
    {
#if SUPPORT_ISP_REGISTER
        ST_IspCBFunc_t *head = &gstIspDevCBListHead[IspDevId], *node = NULL, *node_safe = NULL;
        list_for_each_entry_safe(node, node_safe, &head->pos, pos)
        {
            if(node->pfCBFunc != NULL)
            {
                MI_ISP_CallBackParam_t stCallBackParam;

                memset(&stCallBackParam, 0x0, sizeof(MI_ISP_CallBackParam_t));
                stCallBackParam.eIrqType = E_MI_ISP_IRQ_ISPFRAMEDONE;
                stCallBackParam.pfnCallBackFunc = node->pfCBFunc;
                stCallBackParam.eCallBackMode = E_MI_ISP_CALLBACK_ISR;
                MI_ISP_CallBackTask_Unregister(IspDevId, &stCallBackParam);

                list_del(&node->pos);
                kfree(node);
                node = NULL;
            }
        }
#endif
    }

    for(VifDevId=0; VifDevId<ST_MAX_VIF_DEV_NUM; VifDevId++)
    {
#if SUPPORT_VIF_REGISTER
        ST_VifCBFunc_t *head = &gstVifDevCBListHead[VifDevId], *node = NULL, *node_safe = NULL;
        list_for_each_entry_safe(node, node_safe, &head->pos, pos)
        {
            if(node->pfCBFunc != NULL)
            {
                MI_VIF_CallBackParam_t stCallBackParam;

                memset(&stCallBackParam, 0x0, sizeof(MI_VIF_CallBackParam_t));
                stCallBackParam.eIrqType = E_MI_VIF_IRQ_FRAMESTART;
                stCallBackParam.pfnCallBackFunc = node->pfCBFunc;
                stCallBackParam.eCallBackMode = E_MI_VIF_CALLBACK_ISR;
                MI_VIF_CallBackTask_UnRegister(VifDevId, &stCallBackParam);

                list_del(&node->pos);
                kfree(node);
                node = NULL;
            }
        }
#endif
    }

    return;
}

module_init(vif_isp_module_init);
module_exit(vif_isp_module_exit);
MODULE_LICENSE("Dual BSD/GPL");
