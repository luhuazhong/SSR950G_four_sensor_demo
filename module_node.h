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
 * module_node.h
 */

#ifndef _MODULE_NODE_H_
#define _MODULE_NODE_H_

#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <stdexcept>
#include "mi_sys_datatype.h"

//class BufferNode;

class ModuleNode
{
public:
    ModuleNode(MI_ModuleId_e mod, MI_U32 dev, MI_U32 chn, MI_U32 inport, MI_U32 outport, unsigned int outDelayCnt = 0) :
        eModId(mod), u32DevId(dev), u32ChnId(chn), u32InPort(inport), u32OutPort(outport), outDelayCnt(outDelayCnt)
    {
        InjectThreadRunning = false;
    }
    virtual ~ModuleNode()
    {

    }

    bool Bind(const ModuleNode & node)const;
    bool UnBind(const ModuleNode & node)const;
    //bool StartInject(const BufferNode & node, int fps);
    bool StopInject();

    virtual std::string output_file_postfix()const;
    //bool DumpBuffer(BufferNode*& node);
    virtual bool StInit()= 0;
    virtual bool StDeinit()= 0;
    bool RunCommand(const nlohmann::json &j);
protected:
    //void _InjectThreadFunc(const BufferNode &node, int fps);
    virtual bool _DumpBufferStepInit(int &fd);
    virtual bool _DumpBufferStepDumpPre();
    virtual bool _DumpBufferStepWait();
    //virtual bool _DumpBufferStepDumpOne(BufferNode *&node, bool bypass);
    virtual bool _DumpBufferStepDeinit();
    virtual bool GetInputChnPort(MI_SYS_ChnPort_t &stChnPort)const;
    virtual bool GetOutputChnPort(MI_SYS_ChnPort_t &stChnPort)const;

public:
    typedef bool (ModuleNode::*CmdFunc_t)(const nlohmann::json&);
    std::map<std::string, CmdFunc_t> cmd_map;

    std::thread InjectThread;
    std::mutex InjectWaitMutex;
    std::condition_variable InjectContinueCond;
    std::atomic_bool InjectThreadRunning;

    MI_ModuleId_e eModId;
    MI_U32 u32DevId;
    MI_U32 u32ChnId;
    MI_U32 u32InPort;
    MI_U32 u32OutPort;
    MI_U32 u32FrmRate;

    unsigned int outDelayCnt;

    bool bNeedCreatDev = true;
};

#endif /* _MODULE_NODE_H_ */

