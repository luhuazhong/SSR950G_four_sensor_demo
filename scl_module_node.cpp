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
/*
 * scl_module_node.cpp
 */

#include "json.hpp"
#include "log.h"

#include "mi_scl.h"
#include "mi_sys.h"

#include "scl_module_node.h"

std::map<uint32_t, std::map<uint32_t, std::set<uint32_t> > > SclModuleNode::initStatusMap;


bool SclModuleNode::StInit()
{
    MI_SCL_DevAttr_t stDevAttr;
    MI_SCL_ChannelAttr_t stChnAttr;
    MI_SCL_ChnParam_t stChnParam;
    MI_SCL_OutPortParam_t stOutputParam;

    stDevAttr.u32NeedUseHWOutPortMask = this->u32HwPortMask;
    Log::Info() << "u32NeedUseHWOutPortMask:" << this->u32HwPortMask << Log::End;

    stChnAttr.u32Reserved = 0;

    stChnParam.eRot = E_MI_SYS_ROTATE_NONE;

    stOutputParam.stSCLOutputSize.u16Width = this->u16Width;
    stOutputParam.stSCLOutputSize.u16Height = this->u16Height;
    stOutputParam.bFlip = FALSE;
    stOutputParam.bMirror = FALSE;
    stOutputParam.eCompressMode = E_MI_SYS_COMPRESS_MODE_NONE;
    stOutputParam.ePixelFormat = this->ePixelFormat;
    stOutputParam.stSCLOutCropRect.u16X = 0;
    stOutputParam.stSCLOutCropRect.u16Y = 0;
    stOutputParam.stSCLOutCropRect.u16Width = 0;
    stOutputParam.stSCLOutCropRect.u16Height = 0;

    if (this->bNeedCreatDev)
    {
        if (this->initStatusMap.end() == this->initStatusMap.find(this->u32DevId)) {
            CHECK_RESULT(MI_SUCCESS, return false, MI_SCL_CreateDevice, this->u32DevId, &stDevAttr);
            this->initStatusMap.insert(std::pair<uint32_t, std::map<uint32_t, std::set<uint32_t>>>(this->u32DevId,
                std::map<uint32_t, std::set<uint32_t>>()));
        }
    }
    else 
    {
        if (this->initStatusMap.end() == this->initStatusMap.find(this->u32DevId)) {
            this->initStatusMap.insert(std::pair<uint32_t, std::map<uint32_t, std::set<uint32_t>>>(this->u32DevId,
                std::map<uint32_t, std::set<uint32_t>>()));
        }
    }

    //if (this->u32DevId != 3 || (this->u32DevId == 3 && (this->u32ChnId != 32 && this->u32ChnId != 33)))
    {
        auto iterChnInitStatus = this->initStatusMap.find(this->u32DevId);
        if (iterChnInitStatus->second.end() == iterChnInitStatus->second.find(this->u32ChnId)) {
            Log::Info() << "u32DevId:" << this->u32DevId << " u32ChnId:" << this->u32ChnId << Log::End;
            CHECK_RESULT(MI_SUCCESS, return false, MI_SCL_CreateChannel, this->u32DevId, this->u32ChnId, &stChnAttr);
            CHECK_RESULT(MI_SUCCESS, return false, MI_SCL_SetChnParam, this->u32DevId, this->u32ChnId, &stChnParam);
            CHECK_RESULT(MI_SUCCESS, return false, MI_SCL_StartChannel, this->u32DevId, this->u32ChnId);
            iterChnInitStatus->second.insert(std::pair<uint32_t, std::set<uint32_t>>(this->u32ChnId, std::set<uint32_t>()));
        }

        auto iterPortInitStatus = iterChnInitStatus->second.find(this->u32ChnId);
        if (iterPortInitStatus->second.end() == iterPortInitStatus->second.find(this->u32OutPort)) {
            CHECK_RESULT(MI_SUCCESS, return false, MI_SCL_SetOutputPortParam,
                    this->u32DevId, this->u32ChnId, this->u32OutPort, &stOutputParam);
            CHECK_RESULT(MI_SUCCESS, return false, MI_SCL_EnableOutputPort,
                    this->u32DevId, this->u32ChnId, this->u32OutPort);
            iterPortInitStatus->second.insert(this->u32OutPort);
        }
    }

    return true;
}

bool SclModuleNode::StDeinit()
{
    auto iterChnInitStatus = this->initStatusMap.find(this->u32DevId);
    auto iterPortInitStatus = iterChnInitStatus->second.find(this->u32ChnId);
    if (iterPortInitStatus->second.end() != iterPortInitStatus->second.find(this->u32OutPort)) {
        CHECK_RESULT(MI_SUCCESS, return false, MI_SCL_DisableOutputPort,
                this->u32DevId, this->u32ChnId, this->u32OutPort);
        iterPortInitStatus->second.erase(this->u32OutPort);
    }
    if (iterPortInitStatus->second.empty()) {
        CHECK_RESULT(MI_SUCCESS, return false, MI_SCL_StopChannel, this->u32DevId, this->u32ChnId);
        CHECK_RESULT(MI_SUCCESS, return false, MI_SCL_DestroyChannel, this->u32DevId, this->u32ChnId);
        iterChnInitStatus->second.erase(this->u32ChnId);
    }

    if (this->bNeedCreatDev)
    {
        if (iterChnInitStatus->second.empty()) {
            CHECK_RESULT(MI_SUCCESS, return false, MI_SCL_DestroyDevice, this->u32DevId);
            this->initStatusMap.erase(this->u32DevId);
        }
    }

    return true;
}

bool SclModuleNode::setOutputPortDepth(MI_U32 u32UserFrameDepth, MI_U32 u32BufQueueDepth)
{
    MI_SYS_ChnPort_t stChnPort;
    stChnPort.eModId    = this->eModId;
    stChnPort.u32DevId  = this->u32DevId;
    stChnPort.u32ChnId  = this->u32ChnId;
    stChnPort.u32PortId = this->u32OutPort;

    CHECK_RESULT(MI_SUCCESS, return false, MI_SYS_SetChnOutputPortDepth, 0, &stChnPort, u32UserFrameDepth, u32BufQueueDepth);

    return true;
}

std::string SclModuleNode::output_file_postfix()const
{
    if (this->ePixelFormat == E_MI_SYS_PIXEL_FRAME_YUV422_YUYV) {
        return "yuv422_yuyv.yuv";
    } else if (this->ePixelFormat == E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420) {
        return "nv12.yuv";
    } else {
        return "yuv";
    }
}

void from_json(const nlohmann::json& j, SclModuleNode & node)
{
    std::vector<uint8_t> au8HwScl;
    j.at("hwscl").get_to(au8HwScl);
    node.u32HwPortMask = 0;
    for (auto hwid : au8HwScl) {
        node.u32HwPortMask |= 0x0001 << hwid;
    }
    j.at("dev").get_to(node.u32DevId);
    j.at("channel").get_to(node.u32ChnId);
    j.at("outport").get_to(node.u32OutPort);
    j.at("width").get_to(node.u16Width);
    j.at("height").get_to(node.u16Height);
    j.at("outfmt").get_to(node.ePixelFormat);
}

void SclModuleNode::initSclNode(SclModuleNode& node, SclModuleInitInfo info)
{
    node.u32HwPortMask = info.u32HwPortMask;
    node.u32DevId = info.u32DevId;
    node.u32ChnId = info.u32ChnId;
    node.u32OutPort = info.u32OutPort;
    node.u16Width = info.u16Width;
    node.u16Height = info.u16Height;
    node.ePixelFormat = info.ePixelFormat;
    node.bNeedCreatDev = info.bNeedCreatDev;
}