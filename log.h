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
 * log.h
 */

#ifndef _LOG_H
#define _LOG_H

#include <memory>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>

class Log
{
private:
    class NullStreamBuffer : public std::streambuf
    {
    public:
        NullStreamBuffer(){}
        ~NullStreamBuffer(){}
        int_type overflow(int_type __c = traits_type::eof()) {
            return 0;
        }
    };
public:
    enum LogLevel_e {
        E_LOG_LEVEL_ERROR    = 0,
        E_LOG_LEVEL_WARNING  = 1,
        E_LOG_LEVEL_INFO     = 2,
        E_LOG_LEVEL_DEBUG    = 3,
    };
    enum LogTarget_e {
        E_LOG_TARGET_CONSOLE = 0,
        E_LOG_TARGET_FILE    = 1,
    };
public:
    static Log* GetInstance();
    static std::ostream& End(std::ostream&);
    static std::ostream& EndCache(std::ostream&);
    static std::ostream& Ask();
    static std::ostream& Out();
    static std::ostream& Err();
    static std::ostream& Warn();
    static std::ostream& Info();
    static std::ostream& Debug();
    static std::ostream& Cache();
    void Init(Log::LogLevel_e, Log::LogTarget_e, const char *filepath = nullptr);
private:
    Log();
    Log(const Log &) = delete;
    Log& operator=(const Log &) = delete;
    std::ostream& Out(LogLevel_e);
private:
    static std::shared_ptr<Log> log;

    LogLevel_e eLevel;
    LogTarget_e eTarget;

    std::ostringstream CacheStream;

    std::ofstream FileStream;

    std::ostream NullStream;

    std::string strError;
    std::string strWarning;
    std::string strInfo;
    std::string strDebug;
    std::string strEnd;
};

#define CHECK_RESULT(expectation, erraction, function, ...)         \
    ({                                                              \
        auto ret = (function(__VA_ARGS__));                         \
        if ((expectation) == ret) {                                 \
            Log::Info() << "Success " << #function                  \
                << " <- " << __FUNCTION__ << "[" << __LINE__ << "]" \
                << Log::End;                                        \
        } else {                                                    \
            Log::Err() << "Failed " << #function                    \
                << " <- " << __FUNCTION__ << "[" << __LINE__ << "]" \
                << Log::End;                                        \
            printf("ret:0X%X\n", ret);                              \
            erraction;                                              \
        }                                                           \
    })

#define CHECK_RESULT_NO_LOG(expectation, erraction, function, ...)  \
    ({                                                               \
        if ((expectation) != (function(__VA_ARGS__))) {             \
            erraction;                                              \
        }                                                           \
    })

#endif
