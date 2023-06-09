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

#ifndef _ST_DH9931MENUCONFIG_H_
#define _ST_DH9931MENUCONFIG_H_

#ifdef __cplusplus
extern "C"{
#endif  // __cplusplus

#include "st_dh9931.h"
#include "mi_common_datatype.h"

typedef enum
{
    E_SEND_CMD_NULL =0,
    E_SEND_CMD_ENABLE =1,
    E_SEND_CMD_CVI_UP =2,
    E_SEND_CMD_CVI_DOWN =3,
    E_SEND_CMD_CVI_LEFT =4,
    E_SEND_CMD_CVI_RIGHT =5,
    E_SEND_CMD_CVI_ENTER =6,
    E_SEND_CMD_AHD_UP =14,
    E_SEND_CMD_AHD_DOWN =15,
    E_SEND_CMD_AHD_LEFT =16,
    E_SEND_CMD_AHD_RIGHT =17,
    E_SEND_CMD_AHD_ENTER =18,
    E_SEND_CMD_NUM,
}ST_DH9931OsdMenuCmd_e;

typedef enum
{
    E_OSD_CMD_NULL,
    E_OSD_CMD_ENABLE,
    E_OSD_CMD_ENTER,
    E_OSD_CMD_UP,
    E_OSD_CMD_DOWN,
    E_OSD_CMD_LEFT,
    E_OSD_CMD_RIGHT,
    E_OSD_CMD_NUM,
}ST_OsdCmd_e;


typedef enum
{
    E_SENSOR_FORMAT_NULL,
    E_SENSOR_FORMAT_AHD,
    E_SENSOR_FORMAT_CVI,
    E_SENSOR_FORMAT_NUM,
}ST_SensorFormat_e;

typedef enum
{
    E_SENSOR_FPS_NULL,
    E_SENSOR_FPS_PAL_25,
    E_SENSOR_FPS_NTSC_30,
    E_SENSOR_FPS_NUM,
}ST_SensorFps_e;

typedef enum
{
    E_SENSOR_RES_NULL,
    E_SENSOR_RES_720P_25,
    E_SENSOR_RES_720P_30,
    E_SENSOR_RES_1080P_25,
    E_SENSOR_RES_1080P_30,
    E_SENSOR_RES_NUM,
}ST_SensorRes_e;

typedef enum
{
    E_SENSOR_TYPE_NULL,
    E_SENSOR_TYPE_YOF,
    E_SENSOR_TYPE_DH,
    E_SENSOR_TYPE_NUM,
}ST_SensorType_e;

MI_S32 ST_DH9931ControlSensorMenu(MI_U8 u8ChipId, MI_U8 u8ChnId, ST_DH9931OsdMenuCmd_e eCmd);
MI_S32 ST_DH9931ChoiceSnrFormatRes(MI_U8  u8ChipIndex, MI_U8  u8ChannelIndex, DHC_DH9931_VIDEO_FMT_E  eChangeSnrFormatRes, ST_SensorType_e eSnrType);

#ifdef __cplusplus
}
#endif  // __cplusplus

#endif

