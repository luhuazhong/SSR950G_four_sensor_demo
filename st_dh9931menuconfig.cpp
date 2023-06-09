/*
* XXX.c - Sigmastar
*
* Copyright (c) [2019~2020] SigmaStar Technology.
*
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License version 2 for more details.
*
*/

#ifdef __cplusplus
extern "C"
{
#endif

#include <signal.h>
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
#include <sys/time.h>
#include <unistd.h>
#include <string.h>

#include "st_dh9931.h"
#include "st_dh9931menuconfig.h"

#ifdef __cplusplus
}
#endif

#define ASCII_COLOR_GREEN                        "\033[1;32m"
#define ASCII_COLOR_END                          "\033[0m"
#define ASCII_COLOR_RED                          "\033[1;31m"

#define DBG_INFO(fmt, args...) printf(ASCII_COLOR_GREEN"%s[%d]: " fmt ASCII_COLOR_END, __FUNCTION__,__LINE__, ##args);
#define DBG_ERR(fmt, args...) printf(ASCII_COLOR_RED"%s[%d]: " fmt ASCII_COLOR_END, __FUNCTION__,__LINE__, ##args);

#if ((defined PROG_USER_SENSOR) && (PROG_USER_SENSOR == 1))
    #define USER_SENSOR_SUPPORT (PROG_USER_SENSOR)
#else
    #define USER_SENSOR_SUPPORT (0)
#endif

#if USER_SENSOR_SUPPORT
MI_S32 ST_DH9931ControlSensorMenu(MI_U8 u8ChipId, MI_U8 u8ChnId, ST_DH9931OsdMenuCmd_e eCmd)
{
    struct timeval stTimeVal;
    DHC_DH9931_ERR_ID_E eRet=DHC_ERRID_SUCCESS;

/*
    MI_U8 u8CmdLen = 8;
    DHC_U8 cmd_down[8] =  {0, 0, 0, 0, 0, 8, 8, 32};
    DHC_U8 cmd_up[8] =    {0, 0, 0, 0, 0, 16, 16, 32};
    DHC_U8 cmd_right[8] = {0, 0, 0, 0, 0, 0x40, 0x40, 0};
    DHC_U8 cmd_left[8] =  {0, 0, 0, 0, 0, 0x20, 0x20, 0};
    DHC_U8 cmd_enter[8] = {0, 0, 0, 0, 0x40, 0, 0, 0};
    DHC_U8 cmd_add[8] =   {0, 0, 0, 0, 0x40, 0, 0, 0};
    */
/*
    MI_U8 u8CmdLen = 7;
    DHC_U8 cmd_up[7]    = {0xA5, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00};
    DHC_U8 cmd_down[7] = {0xA5, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00};
    DHC_U8 cmd_left[7] = {0xA5, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00};
    DHC_U8 cmd_right[7] = {0xA5, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};

    DHC_U8 TileUpstop[7]    = {0xA5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    DHC_U8 TileDownsstop[7] = {0xA5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    DHC_U8 PanLeftstop[7] = {0xA5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    DHC_U8 PanRightstop[7] = {0xA5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    */

/*CVI*/
    MI_U8 u8CmdLen = 21;
    DHC_U8 cmd_up[21]    = {165  ,1  ,8   ,0  ,1  ,0  ,175  ,165  ,1  ,0   ,0  ,1  ,0  ,167  ,165  ,1  ,0   ,0  ,1  ,0  ,167};
    DHC_U8 cmd_down[21]  = {165  ,1  ,4   ,0  ,1  ,0  ,171  ,165  ,1  ,0   ,0  ,1  ,0  ,167  ,165  ,1  ,0   ,0  ,1  ,0  ,167};
    DHC_U8 cmd_left[21] = {165  ,1  ,2   ,1  ,0  ,0  ,169  ,165  ,1  ,0   ,1  ,0  ,0  ,167  ,165  ,1  ,0   ,1  ,0  ,0  ,167};
    DHC_U8 cmd_right[21]  = {165  ,1  ,1   ,1  ,0  ,0  ,168  ,165  ,1  ,0   ,1  ,0  ,0  ,167  ,165  ,1  ,0   ,1  ,0  ,0  ,167};
    DHC_U8 cmd_enter[21] = {165  ,1  ,80  ,0  ,0  ,0  ,246  ,165  ,1  ,64  ,0  ,0  ,0  ,230  ,165  ,1  ,64  ,0  ,0  ,0  ,230};

/*AHD 720+1080p*/
    MI_U8 u8ahdCmdLen = 8;

    DHC_U8  ahd720pcmd_down[8]    = {0,0,0,0,0,8,8,32};
    DHC_U8  ahd720pcmd_up[8]  = {0,0,0,0,0,16,16,32};
    DHC_U8  ahd720pcmd_right[8] = {0,0,0,0,0,64,64,0};
    DHC_U8  ahd720pcmd_left[8]  = {0,0,0,0,0,32,32,0};
    DHC_U8  ahd720pcmd_enter[8] = {0,0,0,0,64,0,0,0};
    DHC_U8  ahd720pcmd_add[8] =   {0,0,0,0,64,0,0,0};

//AHD 1080p

    DHC_U8 ahd1080pcmd_down[8]    = {0,0,0,0,0,16,0,32};
    DHC_U8 ahd1080pcmd_up[8]  =     {0,0,0,0,0,8,0,32};
    DHC_U8 ahd1080pcmd_right[8] = {0,0,0,0,0,2,2,0};
    DHC_U8 ahd1080pcmd_left[8]  = {0,0,0,0,0,4,4,0};
    DHC_U8 ahd1080pcmd_enter[8] = {0,0,0,0,2,0,0,0};
    DHC_U8 ahd1080pcmd_add[8] =   {0,0,0,0,2,0,0,0};


    gettimeofday(&stTimeVal, NULL);
    //printf("tv_usec = %ld tv_sec = %ld\n", stTimeVal.tv_usec, stTimeVal.tv_sec);
    if(eCmd == E_SEND_CMD_ENABLE)
    {
        eRet = DHC_DH9931_SDK_SendCo485Enable(u8ChipId, u8ChnId, DHC_TRUE);
    }
    else if (eCmd == E_SEND_CMD_CVI_UP)
    {
        eRet = DHC_DH9931_SDK_SendCo485Buf(u8ChipId, u8ChnId, cmd_up, u8CmdLen);
    }
    else if (eCmd == E_SEND_CMD_CVI_DOWN)
    {
        eRet = DHC_DH9931_SDK_SendCo485Buf(u8ChipId, u8ChnId, cmd_down, u8CmdLen);
    }
    else if (eCmd == E_SEND_CMD_CVI_LEFT)
    {
        eRet = DHC_DH9931_SDK_SendCo485Buf(u8ChipId, u8ChnId, cmd_left, u8CmdLen);
    }
    else if (eCmd == E_SEND_CMD_CVI_RIGHT)
    {
        eRet = DHC_DH9931_SDK_SendCo485Buf(u8ChipId, u8ChnId, cmd_right, u8CmdLen);
    }
    else if (eCmd == E_SEND_CMD_CVI_ENTER)
    {
        eRet = DHC_DH9931_SDK_SendCo485Buf(u8ChipId, u8ChnId, cmd_enter, u8CmdLen);
    }
    else if (eCmd ==8)
    {
        eRet = DHC_DH9931_SDK_SendCo485Buf(u8ChipId, u8ChnId, ahd1080pcmd_up, u8ahdCmdLen);
    }
    else if (eCmd ==9)
    {
        eRet = DHC_DH9931_SDK_SendCo485Buf(u8ChipId, u8ChnId, ahd1080pcmd_down, u8ahdCmdLen);
    }
    else if (eCmd ==10)
    {
        eRet = DHC_DH9931_SDK_SendCo485Buf(u8ChipId, u8ChnId, ahd1080pcmd_left, u8ahdCmdLen);
    }
    else if (eCmd ==11)
    {
        eRet = DHC_DH9931_SDK_SendCo485Buf(u8ChipId, u8ChnId, ahd1080pcmd_right, u8ahdCmdLen);
    }
    else if (eCmd ==12)
    {
        eRet = DHC_DH9931_SDK_SendCo485Buf(u8ChipId, u8ChnId, ahd1080pcmd_enter, u8ahdCmdLen);
    }
    else if (eCmd ==13)
    {
        eRet = DHC_DH9931_SDK_SendCo485Buf(u8ChipId, u8ChnId, ahd1080pcmd_add, u8ahdCmdLen);
    }
    else if (eCmd ==E_SEND_CMD_AHD_UP)
    {
        eRet = DHC_DH9931_SDK_SendCo485Buf(u8ChipId, u8ChnId, ahd720pcmd_up, u8ahdCmdLen);
    }
    else if (eCmd ==E_SEND_CMD_AHD_DOWN)
    {
        eRet = DHC_DH9931_SDK_SendCo485Buf(u8ChipId, u8ChnId, ahd720pcmd_down, u8ahdCmdLen);
    }
    else if (eCmd ==E_SEND_CMD_AHD_LEFT)
    {
        eRet = DHC_DH9931_SDK_SendCo485Buf(u8ChipId, u8ChnId, ahd720pcmd_left, u8ahdCmdLen);
    }
    else if (eCmd ==E_SEND_CMD_AHD_RIGHT)
    {
        eRet = DHC_DH9931_SDK_SendCo485Buf(u8ChipId, u8ChnId, ahd720pcmd_right, u8ahdCmdLen);
    }
    else if (eCmd ==E_SEND_CMD_AHD_ENTER)
    {
        eRet = DHC_DH9931_SDK_SendCo485Buf(u8ChipId, u8ChnId, ahd720pcmd_enter, u8ahdCmdLen);
    }
    else if (eCmd ==19)
    {
        eRet = DHC_DH9931_SDK_SendCo485Buf(u8ChipId, u8ChnId, ahd720pcmd_add, u8ahdCmdLen);
    }

    printf("chn %d send data, eCmd %d  ret %d\n", u8ChnId, eCmd, eRet);
    gettimeofday(&stTimeVal, NULL);
    //printf("tv_usec = %ld tv_sec = %ld\n", stTimeVal.tv_usec, stTimeVal.tv_sec);

    if(eCmd == E_SEND_CMD_ENABLE
        || eCmd ==E_SEND_CMD_AHD_ENTER
        || eCmd ==E_SEND_CMD_CVI_ENTER)
    {
        sleep(1);
    }
    else
    {
        usleep(800*1000);
    }
    return MI_SUCCESS;
}

MI_S32 ST_GetSendOsdCmd(ST_SensorFormat_e eSnrFormat, ST_OsdCmd_e eOsdCmd, ST_DH9931OsdMenuCmd_e *peSendCmd)
{
    MI_S32 s32Ret = MI_SUCCESS;
    if(eOsdCmd == E_OSD_CMD_ENABLE)
    {
        *peSendCmd = E_SEND_CMD_ENABLE;
    }
    else if(eSnrFormat == E_SENSOR_FORMAT_AHD)
    {
        switch(eOsdCmd)
        {
            case E_OSD_CMD_ENTER:
                *peSendCmd = E_SEND_CMD_AHD_ENTER;
                break;
            case E_OSD_CMD_UP:
                *peSendCmd = E_SEND_CMD_AHD_UP;
                break;
            case E_OSD_CMD_DOWN:
                *peSendCmd = E_SEND_CMD_AHD_DOWN;
                break;
            case E_OSD_CMD_RIGHT:
                *peSendCmd = E_SEND_CMD_AHD_RIGHT;
                break;
            case E_OSD_CMD_LEFT:
                *peSendCmd = E_SEND_CMD_AHD_LEFT;
                break;
            default:
                DBG_ERR("osdcmd %d not support \n", eOsdCmd);
                s32Ret=-1;
                break;
        }
    }
    else if(eSnrFormat == E_SENSOR_FORMAT_CVI)
    {
        switch(eOsdCmd)
        {
            case E_OSD_CMD_ENTER:
                *peSendCmd = E_SEND_CMD_CVI_ENTER;
                break;
            case E_OSD_CMD_UP:
                *peSendCmd = E_SEND_CMD_CVI_UP;
                break;
            case E_OSD_CMD_DOWN:
                *peSendCmd = E_SEND_CMD_CVI_DOWN;
                break;
            case E_OSD_CMD_RIGHT:
                *peSendCmd = E_SEND_CMD_CVI_RIGHT;
                break;
            case E_OSD_CMD_LEFT:
                *peSendCmd = E_SEND_CMD_CVI_LEFT;
                break;
            default:
                DBG_ERR("osdcmd %d not support \n", eOsdCmd);
                s32Ret=-1;
                break;
        }
    }

    return s32Ret;
}

MI_S32 ST_Get9931Fps(ST_SensorRes_e eSensorRes, ST_SensorFps_e *peSensorFps)
{
    MI_S32 s32Ret = MI_SUCCESS;
    switch(eSensorRes)
    {
        case E_SENSOR_RES_720P_25:
        case E_SENSOR_RES_1080P_25:
            *peSensorFps = E_SENSOR_FPS_PAL_25;
            break;
        case E_SENSOR_RES_720P_30:
        case E_SENSOR_RES_1080P_30:
            *peSensorFps = E_SENSOR_FPS_NTSC_30;
            break;
        default:
            DBG_ERR("unknow sensorres %d \n", eSensorRes);
            s32Ret=-1;
            break;
    }

    return s32Ret;
}

MI_S32 ST_Separate9931Format(DHC_DH9931_VIDEO_FMT_E enVideoReportFormat, ST_SensorFormat_e *peSensorFormat, ST_SensorRes_e *peSensorRes)
{
    MI_S32 s32Ret = MI_SUCCESS;
    switch(enVideoReportFormat)
    {
        case DHC_CVI_1280x720_25HZ:
            *peSensorFormat = E_SENSOR_FORMAT_CVI;
            *peSensorRes = E_SENSOR_RES_720P_25;
            break;
        case DHC_CVI_1280x720_30HZ:
            *peSensorFormat = E_SENSOR_FORMAT_CVI;
            *peSensorRes = E_SENSOR_RES_720P_30;
            break;
        case DHC_CVI_1920x1080_25HZ:
            *peSensorFormat = E_SENSOR_FORMAT_CVI;
            *peSensorRes = E_SENSOR_RES_1080P_25;
            break;
        case DHC_CVI_1920x1080_30HZ:
            *peSensorFormat = E_SENSOR_FORMAT_CVI;
            *peSensorRes = E_SENSOR_RES_1080P_30;
            break;
        case DHC_AHD_1280x720_25HZ:
            *peSensorFormat = E_SENSOR_FORMAT_AHD;
            *peSensorRes = E_SENSOR_RES_720P_25;
            break;
        case DHC_AHD_1280x720_30HZ:
            *peSensorFormat = E_SENSOR_FORMAT_AHD;
            *peSensorRes = E_SENSOR_RES_720P_30;
            break;
        case DHC_AHD_1920x1080_25HZ:
            *peSensorFormat = E_SENSOR_FORMAT_AHD;
            *peSensorRes = E_SENSOR_RES_1080P_25;
            break;
        case DHC_AHD_1920x1080_30HZ:
            *peSensorFormat = E_SENSOR_FORMAT_AHD;
            *peSensorRes = E_SENSOR_RES_1080P_30;
            break;
        default:
            DBG_ERR("unknow format %d \n", enVideoReportFormat);
            s32Ret=-1;
            break;
    }

    return s32Ret;
}

MI_S32 ST_YOFSensorChangeRes(MI_U8 u8ChipId, MI_U8 u8ChnId,
    ST_SensorFormat_e eCurSnrFormat, ST_SensorRes_e eCurSnrRes, ST_SensorRes_e eChangeSnrRes)
{
    ST_DH9931OsdMenuCmd_e eOsdMenuCmd;
    ST_OsdCmd_e eOsdCmd = E_OSD_CMD_NULL;
    MI_U8 i=0, u8CmdCnt=0;
    DHC_DH9931_VIDEO_STATUS_S  stVideoStatus;
    memset(&stVideoStatus, 0x0, sizeof(DHC_DH9931_VIDEO_STATUS_S));
    stVideoStatus.enVideoLost = DHC_VIDEO_LOST;

    ST_GetSendOsdCmd(eCurSnrFormat, E_OSD_CMD_ENABLE, &eOsdMenuCmd);
    ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);

    ST_GetSendOsdCmd(eCurSnrFormat, E_OSD_CMD_ENTER, &eOsdMenuCmd);
    ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);

    ST_GetSendOsdCmd(eCurSnrFormat, E_OSD_CMD_DOWN, &eOsdMenuCmd);
    ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);
    ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);
    ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);
    ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);

    ST_GetSendOsdCmd(eCurSnrFormat, E_OSD_CMD_ENTER, &eOsdMenuCmd);
    ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);

    ST_GetSendOsdCmd(eCurSnrFormat, E_OSD_CMD_DOWN, &eOsdMenuCmd);
    ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);

    if(eCurSnrRes < eChangeSnrRes)
    {
        eOsdCmd = E_OSD_CMD_RIGHT;
        u8CmdCnt = eChangeSnrRes - eCurSnrRes;
    }
    else if(eCurSnrRes > eChangeSnrRes)
    {
        eOsdCmd = E_OSD_CMD_LEFT;
        u8CmdCnt = eCurSnrRes - eChangeSnrRes;
    }

    for(i=0; i < u8CmdCnt; i++)
    {
        ST_GetSendOsdCmd(eCurSnrFormat, eOsdCmd, &eOsdMenuCmd);
        ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);
    }

    ST_GetSendOsdCmd(eCurSnrFormat, E_OSD_CMD_DOWN, &eOsdMenuCmd);
    ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);
    ST_GetSendOsdCmd(eCurSnrFormat, E_OSD_CMD_ENTER, &eOsdMenuCmd);
    ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);

    printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>Test \n");
    while(stVideoStatus.enVideoLost == DHC_VIDEO_LOST
        || eChangeSnrRes != eCurSnrRes
        )
    {
        DHC_DH9931_SDK_GetVideoStatus(u8ChipId, u8ChnId, &stVideoStatus);
        printf("ad[%d],chn[%d]: isLost[%d], VideoFormat[%d], ReportFormat[%d] \n\n",
            u8ChipId,u8ChnId,stVideoStatus.enVideoLost,stVideoStatus.enVideoFormat,stVideoStatus.enVideoReportFormat);

        if(stVideoStatus.enVideoLost == DHC_VIDEO_CONNECT)
        {
            ST_Separate9931Format(stVideoStatus.enVideoReportFormat, &eCurSnrFormat, &eCurSnrRes);
            printf("cur sensor format %d, res %d \n", eCurSnrFormat, eCurSnrRes);
        }

        usleep(500*1000);
    }

        sleep(2);

    return MI_SUCCESS;
}


MI_S32 ST_YOFSensorChangeFormat(MI_U8 u8ChipId, MI_U8 u8ChnId,
    ST_SensorFormat_e eCurSnrFormat, ST_SensorFormat_e eChangeSnrFormat)
{
    ST_DH9931OsdMenuCmd_e eOsdMenuCmd;
    DHC_DH9931_VIDEO_STATUS_S  stVideoStatus;
    ST_SensorRes_e eCurSnrRes = E_SENSOR_RES_NULL;
    memset(&stVideoStatus, 0x0, sizeof(DHC_DH9931_VIDEO_STATUS_S));
    stVideoStatus.enVideoLost = DHC_VIDEO_LOST;

    ST_GetSendOsdCmd(eCurSnrFormat, E_OSD_CMD_ENABLE, &eOsdMenuCmd);
    ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);

    ST_GetSendOsdCmd(eCurSnrFormat, E_OSD_CMD_ENTER, &eOsdMenuCmd);
    ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);

    ST_GetSendOsdCmd(eCurSnrFormat, E_OSD_CMD_DOWN, &eOsdMenuCmd);
    ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);
    ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);
    ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);
    ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);

    ST_GetSendOsdCmd(eCurSnrFormat, E_OSD_CMD_ENTER, &eOsdMenuCmd);
    ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);

    if(eCurSnrFormat == E_SENSOR_FORMAT_CVI && eChangeSnrFormat == E_SENSOR_FORMAT_AHD)
    {
        ST_GetSendOsdCmd(eCurSnrFormat, E_OSD_CMD_LEFT, &eOsdMenuCmd);
        ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);
    }
    else if(eCurSnrFormat == E_SENSOR_FORMAT_AHD && eChangeSnrFormat == E_SENSOR_FORMAT_CVI)
    {
        ST_GetSendOsdCmd(eCurSnrFormat, E_OSD_CMD_RIGHT, &eOsdMenuCmd);
        ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);
    }

    ST_GetSendOsdCmd(eCurSnrFormat, E_OSD_CMD_DOWN, &eOsdMenuCmd);
    ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);
    ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);
    ST_GetSendOsdCmd(eCurSnrFormat, E_OSD_CMD_ENTER, &eOsdMenuCmd);
    ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);

    //wait sensor param active
    while(stVideoStatus.enVideoLost == DHC_VIDEO_LOST
            || eChangeSnrFormat != eCurSnrFormat
        )
    {
        DHC_DH9931_SDK_GetVideoStatus(u8ChipId, u8ChnId, &stVideoStatus);
        printf("ad[%d],chn[%d]: isLost[%d], VideoFormat[%d], ReportFormat[%d] \n\n",
            u8ChipId,u8ChnId,stVideoStatus.enVideoLost,stVideoStatus.enVideoFormat,stVideoStatus.enVideoReportFormat);

        if(stVideoStatus.enVideoLost == DHC_VIDEO_CONNECT)
        {
            ST_Separate9931Format(stVideoStatus.enVideoReportFormat, &eCurSnrFormat, &eCurSnrRes);
            printf("cur sensor format %d, res %d \n", eCurSnrFormat, eCurSnrRes);
        }

        usleep(500*1000);
    }

    sleep(2);
    return MI_SUCCESS;
}

MI_S32 ST_YOFSensorShowFormatRes(MI_U8 u8ChipId, MI_U8 u8ChnId,ST_SensorFormat_e eCurSnrFormat)
{
    ST_DH9931OsdMenuCmd_e eOsdMenuCmd;
    ST_GetSendOsdCmd(eCurSnrFormat, E_OSD_CMD_ENABLE, &eOsdMenuCmd);
    ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);

    ST_GetSendOsdCmd(eCurSnrFormat, E_OSD_CMD_ENTER, &eOsdMenuCmd);
    ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);

    ST_GetSendOsdCmd(eCurSnrFormat, E_OSD_CMD_DOWN, &eOsdMenuCmd);
    ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);
    ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);
    ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);
    ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);

    ST_GetSendOsdCmd(eCurSnrFormat, E_OSD_CMD_ENTER, &eOsdMenuCmd);
    ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);

    sleep(3);
    /*EXIT*/

    ST_GetSendOsdCmd(eCurSnrFormat, E_OSD_CMD_DOWN, &eOsdMenuCmd);
    ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);
    ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);
    ST_GetSendOsdCmd(eCurSnrFormat, E_OSD_CMD_ENTER, &eOsdMenuCmd);
    ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);
    ST_GetSendOsdCmd(eCurSnrFormat, E_OSD_CMD_DOWN, &eOsdMenuCmd);
    ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);
    ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);
    ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);
    ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);
    ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);
    ST_GetSendOsdCmd(eCurSnrFormat, E_OSD_CMD_ENTER, &eOsdMenuCmd);
    ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);

    return MI_SUCCESS;
}

MI_S32 ST_DHSensorChangeRes(MI_U8 u8ChipId, MI_U8 u8ChnId,
    ST_SensorFormat_e eCurSnrFormat, ST_SensorRes_e eCurSnrRes, ST_SensorRes_e eChangeSnrRes)
{
    MI_S32 s32Ret=MI_SUCCESS;
    MI_U8 i=0, u8TimeOutCnt=240;//120s
    ST_OsdCmd_e eOsdCmd = E_OSD_CMD_NULL;
    MI_U8 u8CmdCnt=0;
    ST_DH9931OsdMenuCmd_e eOsdMenuCmd;
    ST_SensorFps_e eCurFps, eChangeFps;
    DHC_DH9931_VIDEO_STATUS_S  stVideoStatus;
    memset(&stVideoStatus, 0x0, sizeof(DHC_DH9931_VIDEO_STATUS_S));
    stVideoStatus.enVideoLost = DHC_VIDEO_LOST;

    ST_Get9931Fps(eCurSnrRes, &eCurFps);
    ST_Get9931Fps(eChangeSnrRes, &eChangeFps);

    ST_GetSendOsdCmd(eCurSnrFormat, E_OSD_CMD_ENABLE, &eOsdMenuCmd);
    ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);

    ST_GetSendOsdCmd(eCurSnrFormat, E_OSD_CMD_ENTER, &eOsdMenuCmd);
    ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);

    if(eCurFps!=eChangeFps)
    {
        ST_GetSendOsdCmd(eCurSnrFormat, E_OSD_CMD_RIGHT, &eOsdMenuCmd);
        ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);

        ST_GetSendOsdCmd(eCurSnrFormat, E_OSD_CMD_ENTER, &eOsdMenuCmd);
        ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);
        ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);

        while(s32Ret == MI_SUCCESS && i < u8TimeOutCnt
            && (stVideoStatus.enVideoLost == DHC_VIDEO_LOST
                || eChangeFps != eCurFps
                )
            )
        {
            i++;
            DHC_DH9931_SDK_GetVideoStatus(u8ChipId, u8ChnId, &stVideoStatus);
            printf("ad[%d],chn[%d]: isLost[%d], VideoFormat[%d], ReportFormat[%d] \n\n",
                u8ChipId,u8ChnId,stVideoStatus.enVideoLost,stVideoStatus.enVideoFormat,stVideoStatus.enVideoReportFormat);

            if(stVideoStatus.enVideoLost == DHC_VIDEO_CONNECT)
            {
                ST_Separate9931Format(stVideoStatus.enVideoReportFormat, &eCurSnrFormat, &eCurSnrRes);
                ST_Get9931Fps(eCurSnrRes, &eCurFps);
                printf("cur sensor format %d, res %d fps %d\n", eCurSnrFormat, eCurSnrRes, eCurFps);
            }

            usleep(500*1000);
        }
        i=0;

        ST_GetSendOsdCmd(eCurSnrFormat, E_OSD_CMD_ENABLE, &eOsdMenuCmd);
        ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);
    }

    ST_GetSendOsdCmd(eCurSnrFormat, E_OSD_CMD_DOWN, &eOsdMenuCmd);
    ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);

    if(eCurSnrRes != eChangeSnrRes)
    {
        if(eChangeSnrRes>eCurSnrRes)
        {
            eOsdCmd = E_OSD_CMD_RIGHT;
            u8CmdCnt = 2;
        }
        else if(eChangeSnrRes<eCurSnrRes)
        {
            eOsdCmd = E_OSD_CMD_LEFT;
            u8CmdCnt = 2;
        }

        for(i=0; i < u8CmdCnt; i++)
        {
            ST_GetSendOsdCmd(eCurSnrFormat, eOsdCmd, &eOsdMenuCmd);
            ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);
        }
        i=0;

        ST_GetSendOsdCmd(eCurSnrFormat, E_OSD_CMD_ENTER, &eOsdMenuCmd);
        ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);
        ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);

        while(s32Ret == MI_SUCCESS && i < u8TimeOutCnt
            && (stVideoStatus.enVideoLost == DHC_VIDEO_LOST
                || eCurSnrRes != eChangeSnrRes
                )
            )
        {
            i++;
            DHC_DH9931_SDK_GetVideoStatus(u8ChipId, u8ChnId, &stVideoStatus);
            printf("ad[%d],chn[%d]: isLost[%d], VideoFormat[%d], ReportFormat[%d] \n\n",
                u8ChipId,u8ChnId,stVideoStatus.enVideoLost,stVideoStatus.enVideoFormat,stVideoStatus.enVideoReportFormat);

            if(stVideoStatus.enVideoLost == DHC_VIDEO_CONNECT)
            {
                ST_Separate9931Format(stVideoStatus.enVideoReportFormat, &eCurSnrFormat, &eCurSnrRes);
                ST_Get9931Fps(eCurSnrRes, &eCurFps);
                printf("cur sensor format %d, res %d fps %d\n", eCurSnrFormat, eCurSnrRes, eCurFps);
            }

            usleep(500*1000);
        }
        ST_GetSendOsdCmd(eCurSnrFormat, E_OSD_CMD_ENABLE, &eOsdMenuCmd);
        ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);
    }

    //EXIT
    for(i=0; i < 9; i++)
    {
        ST_GetSendOsdCmd(eCurSnrFormat, E_OSD_CMD_DOWN, &eOsdMenuCmd);
        ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);
    }

    ST_GetSendOsdCmd(eCurSnrFormat, E_OSD_CMD_ENTER, &eOsdMenuCmd);
    ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);

    return s32Ret;

}

MI_S32 ST_DHSensorShowFormatRes(MI_U8 u8ChipId, MI_U8 u8ChnId,ST_SensorFormat_e eCurSnrFormat)
{
    MI_U8 i=0, u8CmdCnt=0;
    ST_DH9931OsdMenuCmd_e eOsdMenuCmd;
    ST_GetSendOsdCmd(eCurSnrFormat, E_OSD_CMD_ENABLE, &eOsdMenuCmd);
    ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);

    ST_GetSendOsdCmd(eCurSnrFormat, E_OSD_CMD_ENTER, &eOsdMenuCmd);
    ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);

    sleep(3);
    /*EXIT*/

    ST_GetSendOsdCmd(eCurSnrFormat, E_OSD_CMD_DOWN, &eOsdMenuCmd);
    u8CmdCnt = 10;
    for(i=0; i< u8CmdCnt; i++)
    {
        ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);
    }

    ST_GetSendOsdCmd(eCurSnrFormat, E_OSD_CMD_ENTER, &eOsdMenuCmd);
    ST_DH9931ControlSensorMenu(u8ChipId, u8ChnId, eOsdMenuCmd);

    return MI_SUCCESS;
}

MI_S32 ST_SensorShowFormatRes(ST_SensorType_e eSnrType, MI_U8 u8ChipId, MI_U8 u8ChnId,ST_SensorFormat_e eCurSnrFormat)
{
    MI_S32 s32Ret = MI_SUCCESS;
    switch(eSnrType)
    {
        case E_SENSOR_TYPE_YOF:
            ST_YOFSensorShowFormatRes(u8ChipId, u8ChnId, eCurSnrFormat);
            break;
        case E_SENSOR_TYPE_DH:
            ST_DHSensorShowFormatRes(u8ChipId, u8ChnId, eCurSnrFormat);
            break;
        default:
            DBG_ERR("unknow sensor type %d \n", eSnrType);
            break;
    }

    return s32Ret;
}

MI_S32 ST_SensorChangeFormat(ST_SensorType_e eSnrType, MI_U8 u8ChipId, MI_U8 u8ChnId,
    ST_SensorFormat_e eCurSnrFormat, ST_SensorFormat_e eChangeSnrFormat)
{
    MI_S32 s32Ret = MI_SUCCESS;
    switch(eSnrType)
    {
        case E_SENSOR_TYPE_YOF:
            ST_YOFSensorChangeFormat(u8ChipId, u8ChnId, eCurSnrFormat, eChangeSnrFormat);
            break;
        case E_SENSOR_TYPE_DH:
            DBG_ERR("DH sensor no support CVI/AHD SW change \n");
            break;
        default:
            DBG_ERR("unknow sensor type %d \n", eSnrType);
            break;
    }

    return s32Ret;
}

MI_S32 ST_SensorChangeRes(ST_SensorType_e eSnrType,MI_U8 u8ChipId, MI_U8 u8ChnId,
    ST_SensorFormat_e eCurSnrFormat, ST_SensorRes_e eCurSnrRes, ST_SensorRes_e eChangeSnrRes)
{
    MI_S32 s32Ret = MI_SUCCESS;
    switch(eSnrType)
    {
        case E_SENSOR_TYPE_YOF:
            ST_YOFSensorChangeRes(u8ChipId, u8ChnId, eCurSnrFormat, eCurSnrRes, eChangeSnrRes);
            break;
        case E_SENSOR_TYPE_DH:
            ST_DHSensorChangeRes(u8ChipId, u8ChnId, eCurSnrFormat, eCurSnrRes, eChangeSnrRes);
            break;
        default:
            DBG_ERR("unknow sensor type %d \n", eSnrType);
            break;
    }

    return s32Ret;

}

MI_S32 ST_DH9931ChoiceSnrFormatRes(MI_U8  u8ChipIndex, MI_U8  u8ChannelIndex, DHC_DH9931_VIDEO_FMT_E  eChangeSnrFormatRes, ST_SensorType_e eSnrType)
{
    MI_S32 s32Ret=MI_SUCCESS;
    DHC_DH9931_VIDEO_STATUS_S  stVideoStatus;
    ST_SensorFormat_e  eChangeSnrFormat=E_SENSOR_FORMAT_NULL, eCurSnrFormat=E_SENSOR_FORMAT_NULL;
    ST_SensorRes_e eChangeSnrRes = E_SENSOR_RES_NULL, eCurSnrRes = E_SENSOR_RES_NULL;
    ST_SensorFps_e eChangeSnrfps = E_SENSOR_FPS_NULL, eCurSnrfps = E_SENSOR_FPS_NULL;
    memset(&stVideoStatus, 0x0, sizeof(DHC_DH9931_VIDEO_STATUS_S));
    stVideoStatus.enVideoLost = DHC_VIDEO_LOST;

    DHC_DH9931_SDK_GetVideoStatus(u8ChipIndex, u8ChannelIndex, &stVideoStatus);
    printf("ad[%d],chn[%d]: isLost[%d], VideoFormat[%d], ReportFormat[%d] \n\n",
        u8ChipIndex,u8ChannelIndex,stVideoStatus.enVideoLost,stVideoStatus.enVideoFormat,stVideoStatus.enVideoReportFormat);

    ST_Separate9931Format(stVideoStatus.enVideoReportFormat, &eCurSnrFormat, &eCurSnrRes);
    ST_Get9931Fps(eCurSnrRes, &eCurSnrfps);
    printf("cur sensor format %d, res %d fps %d\n", eCurSnrFormat, eCurSnrRes, eCurSnrfps);

    if(eSnrType == E_SENSOR_TYPE_YOF
        && stVideoStatus.enVideoReportFormat == DHC_CVI_1280x720_30HZ)
    {
        DBG_ERR("YOF Sensor no support cvi 720p30 osd menu \n");
        s32Ret=-1 ;
        goto EXIT;
    }

    if(stVideoStatus.enVideoLost == DHC_VIDEO_LOST
        || (eCurSnrFormat != E_SENSOR_FORMAT_AHD
        && eCurSnrFormat != E_SENSOR_FORMAT_CVI)
        )
    {
        DBG_ERR("only sensor connect and sensor format is AHD or CVI can control menu \n");
        s32Ret=-1 ;
        goto EXIT;
    }

    if(eChangeSnrFormatRes == 222)
    {
        ST_SensorShowFormatRes(eSnrType, u8ChipIndex, u8ChannelIndex, eCurSnrFormat);
        goto EXIT;
    }

    ST_Separate9931Format(eChangeSnrFormatRes, &eChangeSnrFormat, &eChangeSnrRes);
    ST_Get9931Fps(eChangeSnrRes, &eChangeSnrfps);
    printf("cur sensor format %d, res %d, change format %d, res %d fps %d\n", eCurSnrFormat, eCurSnrRes, eChangeSnrFormat, eChangeSnrRes, eChangeSnrfps);

    if(eCurSnrFormat != eChangeSnrFormat)
    {
        s32Ret = ST_SensorChangeFormat(eSnrType, u8ChipIndex, u8ChannelIndex, eCurSnrFormat, eChangeSnrFormat);
    }

    if(eCurSnrRes != eChangeSnrRes)
    {
        ST_SensorChangeRes(eSnrType, u8ChipIndex, u8ChannelIndex, eCurSnrFormat, eCurSnrRes, eChangeSnrRes);
    }

EXIT:

    return s32Ret;
}

#endif

