//
// Created by Raghavasimhan Sankaranarayanan on 5/6/22.
//

#ifndef SHIMONCONTROLLER_LOGGER_H
#define SHIMONCONTROLLER_LOGGER_H
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"

class Logger {
public:
    enum Level
    {
        trace = SPDLOG_LEVEL_TRACE,
        debug = SPDLOG_LEVEL_DEBUG,
        info = SPDLOG_LEVEL_INFO,
        warn = SPDLOG_LEVEL_WARN,
        err = SPDLOG_LEVEL_ERROR,
        critical = SPDLOG_LEVEL_CRITICAL,
        off = SPDLOG_LEVEL_OFF,

        n_levels
    };

//    static void init(Level level = trace) {
////        spdlog::set_pattern("%^[%r]\t[%s]\t[line %#]\t[---%l---]\t%v%$");
//        spdlog::set_level((spdlog::level::level_enum)level);
//        SPDLOG_INFO("Logger initialized");
//    }
};

#define LOG_TRACE(...)      SPDLOG_TRACE    (__VA_ARGS__)
#define LOG_DEBUG(...)      SPDLOG_DEBUG    (__VA_ARGS__)
#define LOG_INFO(...)       SPDLOG_INFO     (__VA_ARGS__)
#define LOG_WARN(...)       SPDLOG_WARN     (__VA_ARGS__)
#define LOG_ERROR(...)      SPDLOG_ERROR    (__VA_ARGS__)
#define LOG_CRITICAL(...)   SPDLOG_CRITICAL (__VA_ARGS__)

#endif //SHIMONCONTROLLER_LOGGER_H
