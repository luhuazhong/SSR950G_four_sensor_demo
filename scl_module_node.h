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
 * scl_module_node.h
 */

#ifndef _SCL_MODULE_NODE_H
#define _SCL_MODULE_NODE_H

#include <set>
#include "mi_scl_datatype.h"
#include "module_node.h"

typedef struct
{
    MI_U32 u32HwPortMask;
    MI_U32 u32DevId;
    MI_U32 u32ChnId;
    MI_U32 u32OutPort;
    MI_U16 u16Width;
    MI_U16 u16Height;
    MI_SYS_PixelFormat_e ePixelFormat;
    bool bNeedCreatDev;
}SclModuleInitInfo;

class SclModuleNode : public ModuleNode
{
    friend void from_json(const nlohmann::json& j, SclModuleNode & node);

public:
    static void initSclNode(SclModuleNode& node, SclModuleInitInfo info);

public:
    SclModuleNode() : ModuleNode(E_MI_MODULE_ID_SCL, 0, 0, 0, 0, 0) {
        this->u32FrmRate = 60;
    }
    ~SclModuleNode() {

    }
    bool StInit();
    bool StDeinit();
    bool setOutputPortDepth(MI_U32 u32UserFrameDepth, MI_U32 u32BufQueueDepth);
    std::string output_file_postfix()const;
private:
    static std::map<uint32_t, std::map<uint32_t, std::set<uint32_t> > > initStatusMap;
    MI_SYS_PixelFormat_e ePixelFormat;
    MI_U32 u32HwPortMask;
    MI_U16 u16Width;
    MI_U16 u16Height;
};
#endif
