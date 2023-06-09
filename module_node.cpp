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
 * module_node.cpp
 */

#include "mi_sys.h"

#include "json.hpp"
#include "log.h"

#include "module_node.h"
//#include "buffer_node.h"
//#include "buffer_node_factory.h"

bool ModuleNode::GetInputChnPort(MI_SYS_ChnPort_t &stChnPort)const
{
    stChnPort.eModId    = this->eModId;
    stChnPort.u32DevId  = this->u32DevId;
    stChnPort.u32ChnId  = this->u32ChnId;
    stChnPort.u32PortId = this->u32InPort;
    return true;
}
bool ModuleNode::GetOutputChnPort(MI_SYS_ChnPort_t &stChnPort)const
{
    stChnPort.eModId    = this->eModId;
    stChnPort.u32DevId  = this->u32DevId;
    stChnPort.u32ChnId  = this->u32ChnId;
    stChnPort.u32PortId = this->u32OutPort;
    return true;
}

#if 1
bool ModuleNode::RunCommand(const nlohmann::json &j)
{
    std::string func_str;
    try {
        func_str = j.at("func");
        nlohmann::json attr  = j.at("param");
        auto func = cmd_map.at(func_str);
        Log::Debug() << "RunCommand : " << func_str << attr << Log::End;
        return (this->*func)(attr);
    }
    catch (nlohmann::json::out_of_range &e) {
        Log::Err() << e.what() << Log::End;
        return false;
    }
    catch (std::out_of_range &e) {
        Log::Err() << "func [ " << func_str << " ] is not support" << Log::End;
        return false;
    }
    return false;
}
#endif

bool ModuleNode::Bind(const ModuleNode & node)const
{
    MI_SYS_BindType_e eBindType = E_MI_SYS_BIND_TYPE_FRAME_BASE;
    MI_SYS_ChnPort_t stSrcChnPort;
    MI_SYS_ChnPort_t stDstChnPort;
    MI_U32 u32SrcFrmrate = this->u32FrmRate;
    MI_U32 u32DstFrmrate = node.u32FrmRate;
    CHECK_RESULT_NO_LOG(true, return false, this->GetOutputChnPort, stSrcChnPort);
    CHECK_RESULT_NO_LOG(true, return false, node.GetInputChnPort, stDstChnPort);
    CHECK_RESULT(MI_SUCCESS, return false, MI_SYS_BindChnPort2, 0, &stSrcChnPort, &stDstChnPort,
            u32SrcFrmrate, u32DstFrmrate, eBindType, 0);
    return true;
}

bool ModuleNode::UnBind(const ModuleNode & node)const
{
    MI_SYS_ChnPort_t stSrcChnPort;
    MI_SYS_ChnPort_t stDstChnPort;
    CHECK_RESULT_NO_LOG(true, return false, this->GetOutputChnPort, stSrcChnPort);
    CHECK_RESULT_NO_LOG(true, return false, node.GetInputChnPort, stDstChnPort);
    CHECK_RESULT(MI_SUCCESS, return false, MI_SYS_UnBindChnPort, 0, &stSrcChnPort, &stDstChnPort);
    return true;
}

//void ModuleNode::_InjectThreadFunc(const BufferNode &node, int fps)
//{
//    std::unique_lock<std::mutex> lck(this->InjectWaitMutex);
//    MI_SYS_ChnPort_t stChnPort;
//    MI_SYS_BufConf_t stBufConf;
//    MI_SYS_BUF_HANDLE hBufHandle;
//    MI_SYS_BufInfo_t stBufInfo;
//
//    memset(&stChnPort, 0, sizeof(MI_SYS_ChnPort_t));
//    memset(&stBufConf, 0, sizeof(MI_SYS_BufConf_t));
//    memset(&hBufHandle, 0, sizeof(MI_SYS_BUF_HANDLE));
//    memset(&stBufInfo, 0, sizeof(MI_SYS_BufInfo_t));
//
//    CHECK_RESULT_NO_LOG(true, return, this->GetInputChnPort, stChnPort);
//
//    stBufConf.eBufType = E_MI_SYS_BUFDATA_FRAME;
//    stBufConf.u32Flags = 0;
//    stBufConf.stFrameCfg.eFrameScanMode = E_MI_SYS_FRAME_SCAN_MODE_PROGRESSIVE;
//    stBufConf.stFrameCfg.eCompressMode  = E_MI_SYS_COMPRESS_MODE_NONE;
//    stBufConf.stFrameCfg.eFormat = node.get_fmt();
//    stBufConf.stFrameCfg.u16Width = node.get_width();
//    stBufConf.stFrameCfg.u16Height = node.get_height();
//    CHECK_RESULT_NO_LOG(MI_SUCCESS, return, MI_SYS_GetCurPts, 0, &stBufConf.u64TargetPts);
//
//    memset(&hBufHandle, 0, sizeof(MI_SYS_BUF_HANDLE));
//    memset(&stBufInfo, 0, sizeof(MI_SYS_BufInfo_t));
//
//    CHECK_RESULT_NO_LOG(MI_SUCCESS, return, MI_SYS_ChnInputPortGetBuf, &stChnPort, &stBufConf, &stBufInfo, &hBufHandle, 3000);
//    node.copy_to(stBufInfo.stFrameData);
//
//    MI_SYS_BUF_HANDLE hDupBufHandle;
//    unsigned int timeout_ms = fps == 0 ? 0 : 1000 / fps;
//
//    while (InjectThreadRunning) {
//        if (timeout_ms) {
//            this->InjectContinueCond.wait_for(lck, std::chrono::milliseconds(timeout_ms));
//        } else {
//            this->InjectContinueCond.wait(lck);
//        }
//        if (MI_SUCCESS == MI_SYS_DupBuf(hBufHandle, &hDupBufHandle)) {
//            CHECK_RESULT_NO_LOG(MI_SUCCESS, continue, MI_SYS_ChnInputPortPutBuf, hDupBufHandle, &stBufInfo,
//                    InjectThreadRunning ? FALSE : TRUE);
//        }
//    }
//
//    CHECK_RESULT_NO_LOG(MI_SUCCESS, return, MI_SYS_ChnInputPortPutBuf, hBufHandle, &stBufInfo, TRUE);
//}

//bool ModuleNode::StartInject(const BufferNode &node, int fps)
//{
//    this->InjectThreadRunning.store(true);
//    this->InjectThread = std::thread(&ModuleNode::_InjectThreadFunc, this, std::ref(node), fps);
//    Log::Info() << "inject start" << Log::End;
//    return true;
//}

bool ModuleNode::StopInject()
{
    std::unique_lock<std::mutex> lck(this->InjectWaitMutex);
    this->InjectThreadRunning.store(false);
    this->InjectContinueCond.notify_all();
    lck.unlock();
    this->InjectThread.join();
    Log::Info() << "inject stop" << Log::End;
    return true;
}

//bool ModuleNode::DumpBuffer(BufferNode*& node)
//{
//    int fd;
//    unsigned int timeoutCnt = 0;
//    fd_set read_fds;
//    int bypass = this->outDelayCnt;
//
//    CHECK_RESULT_NO_LOG(true, return false, this->_DumpBufferStepInit, fd);
//
//    while (1)
//    {
//        if (bypass == 0) {
//            if (!this->_DumpBufferStepDumpPre()) {
//                break;
//            }
//        }
//        std::unique_lock<std::mutex> lck(this->InjectWaitMutex);
//        this->InjectContinueCond.notify_all();
//        lck.unlock();
//        this->_DumpBufferStepWait();
//
//        FD_ZERO(&read_fds);
//        FD_SET(fd, &read_fds);
//
//        struct timeval tv;
//        tv.tv_sec = 1;
//        tv.tv_usec = 0;
//
//        int ret = select(fd + 1, &read_fds, NULL, NULL, &tv);
//
//        if (ret < 0) {
//            Log::Err() << "select output port fd Failed" << Log::End;
//            break;
//        } else if (ret == 0) {
//            Log::Warn() << "select output port timeout" << Log::End;
//            if (++timeoutCnt > 10) {
//                Log::Err() << "select output port timeoutCnt " << timeoutCnt << Log::End;
//                break;
//            }
//            continue;
//        } else {
//            if (!FD_ISSET(fd, &read_fds)) {
//                continue;
//            }
//            if (this->_DumpBufferStepDumpOne(node, bypass != 0)) {
//                if (bypass == 0) {
//                    Log::Info() <<  "Success DumpBuffer" << Log::End;
//                } else {
//                    Log::Info() <<  "Success Bypass" << Log::End;
//                }
//                --bypass;
//            } else {
//                break;
//            }
//            if (bypass < 0) {
//                break;
//            }
//        }
//    }
//
//    CHECK_RESULT_NO_LOG(true, return false, this->_DumpBufferStepDeinit);
//    return true;
//}

bool ModuleNode::_DumpBufferStepInit(int &fd)
{
    MI_SYS_ChnPort_t stChnPort;
    CHECK_RESULT_NO_LOG(true, return false, this->GetOutputChnPort, stChnPort);
    CHECK_RESULT_NO_LOG(MI_SUCCESS, return false, MI_SYS_SetChnOutputPortDepth, 0, &stChnPort, 1, 3);
    CHECK_RESULT_NO_LOG(MI_SUCCESS, return false, MI_SYS_GetFd, &stChnPort, &fd);
    return true;
}
bool ModuleNode::_DumpBufferStepDumpPre()
{
    return true;
}
bool ModuleNode::_DumpBufferStepWait()
{
    return true;
}
//bool ModuleNode::_DumpBufferStepDumpOne(BufferNode*& node, bool bypass)
//{
//    MI_SYS_ChnPort_t stChnPort;
//    MI_SYS_BufInfo_t stBufInfo;
//    MI_SYS_BUF_HANDLE hBufHandle;
//    unsigned int failedCnt = 0;
//
//    CHECK_RESULT_NO_LOG(true, return false, this->GetOutputChnPort, stChnPort);
//    while (1) {
//        if (MI_SUCCESS == MI_SYS_ChnOutputPortGetBuf(&stChnPort, &stBufInfo, &hBufHandle)) {
//            if (!bypass) {
//                if (node == nullptr) {
//                    node = BufferNodeFactory::Create(stBufInfo.stFrameData.ePixelFormat,
//                            stBufInfo.stFrameData.u16Width, stBufInfo.stFrameData.u16Height);
//                }
//                if (node != nullptr) {
//                    node->copy_from(stBufInfo.stFrameData);
//                }
//            }
//            if (MI_SUCCESS == MI_SYS_ChnOutputPortPutBuf(hBufHandle)) {
//                break;
//            }
//        } else {
//            if (++failedCnt > 10) {
//                Log::Err() << "MI_SYS_ChnOutputPortGetBuf FailedCnt " << failedCnt << Log::End;
//                return false;
//            }
//        }
//    }
//    return true;
//}
bool ModuleNode::_DumpBufferStepDeinit()
{
    MI_SYS_ChnPort_t stChnPort;
    CHECK_RESULT_NO_LOG(true, return false, this->GetOutputChnPort, stChnPort);
    CHECK_RESULT_NO_LOG(MI_SUCCESS, return false, MI_SYS_SetChnOutputPortDepth, 0, &stChnPort, 0, 3);
    return true;
}

std::string ModuleNode::output_file_postfix()const
{
    return "yuv";
}

