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
 * src/log.cpp
 */

#include "log.h"

std::shared_ptr<Log> Log::log(Log::GetInstance());

Log::Log() : NullStream(new NullStreamBuffer())
{
}
Log* Log::GetInstance()
{
    if (Log::log == nullptr) {
        Log::log.reset(new Log());
    }
    return Log::log.get();
}
void Log::Init(Log::LogLevel_e level, Log::LogTarget_e target, const char *filepath)
{
    this->eLevel  = level;
    if (target == E_LOG_TARGET_FILE && filepath != nullptr) {
        this->FileStream.open(filepath);
        if (!this->FileStream.is_open()) {
            this->eTarget = E_LOG_TARGET_CONSOLE;
        }
    }
    this->eTarget = target;
    switch (eTarget) {
    case E_LOG_TARGET_CONSOLE:
        {
            strError   = "\033[1;31m";
            strWarning = "\033[1;35m";
            strInfo    = "\033[1;36m";
            strDebug   = "\033[1;37m";
            strEnd     = "\033[0m";
        }
        break;
    case E_LOG_TARGET_FILE:
        {
            strError   = "[ ERR] ";
            strWarning = "[WARN] ";
            strInfo    = "[INFO] ";
            strDebug   = "[ DBG] ";
            strEnd     = "";
        }
        break;
    }
    std::cout << "Log System Init"
        << "\nlevel:  "
        << (level == E_LOG_LEVEL_ERROR ? "ERROR"
            : level == E_LOG_LEVEL_WARNING ? "WARNING"
            : level == E_LOG_LEVEL_INFO ? "INFO"
            : "DEBUG")
        << "\ntarget: "
        << (E_LOG_TARGET_CONSOLE == eTarget ? "console"
            : std::string("file>")+filepath)
        << std::endl;
}
std::ostream& Log::Ask()
{
    return std::cout << "\033[1;33m";
}
std::ostream& Log::Out()
{
    return Log::GetInstance()->Out(E_LOG_LEVEL_ERROR);
}
std::ostream& Log::Err()
{
    return Log::GetInstance()->Out(E_LOG_LEVEL_ERROR) << Log::GetInstance()->strError;
}
std::ostream& Log::Warn()
{
    return Log::GetInstance()->Out(E_LOG_LEVEL_WARNING) << Log::GetInstance()->strWarning;
}
std::ostream& Log::Info()
{
    return Log::GetInstance()->Out(E_LOG_LEVEL_INFO) << Log::GetInstance()->strInfo;
}
std::ostream& Log::Debug()
{
    return Log::GetInstance()->Out(E_LOG_LEVEL_DEBUG) << Log::GetInstance()->strDebug;
}
std::ostream& Log::Cache()
{
    return Log::GetInstance()->CacheStream;
}
std::ostream& Log::Out(Log::LogLevel_e level)
{
    if (level > this->eLevel) {
        return this->NullStream;
    }
    std::ostream &os = E_LOG_TARGET_CONSOLE == this->eTarget ? std::cout : this->FileStream;
    return os;
}
std::ostream& Log::End(std::ostream &os)
{
    os << Log::GetInstance()->strEnd << std::endl;
    return os;
}
std::ostream& Log::EndCache(std::ostream &os)
{
    os << Log::GetInstance()->CacheStream.str();
    Log::GetInstance()->CacheStream.clear();
    Log::GetInstance()->CacheStream.str("");
    return os;
}

