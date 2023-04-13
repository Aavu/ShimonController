//
// Created by Raghavasimhan Sankaranarayanan on 5/4/22.
//

#ifndef SHIMONCONTROLLER_ERRORDEF_H
#define SHIMONCONTROLLER_ERRORDEF_H
#include "Logger.h"

#define ERROR_CHECK(e, ret) {if (e != kNoError) {LOG_ERROR("Error Code {}", e); return ret;}}

enum Error_t {
    kNoError = 0,

    kNotInitializedError,
    kReInitializationError,
    kNotHomedError,

    kNotImplementedError,
    kInvalidArgsError,
    kArgLimitError,

    kCorruptedDataError,
    kSegvError,
    kInsufficientMemError,

    kFileParseError,

    kFileOpenError,
    kFileCloseError,

    kNotOpenedError,
    kNotClosedError,

    kSetValueError,
    kGetValueError,

    kWriteError,
    kReadError,
    kFlushError,

    kImpossibleError,
    kAlreadyThereError,
    kInterferenceError,

    kTimeoutError,

    kNamingError,

    kBufferReadError,
    kBufferWriteError,

    kUnknownCaseError,
    kUnknownError
};

#endif //SHIMONCONTROLLER_ERRORDEF_H
