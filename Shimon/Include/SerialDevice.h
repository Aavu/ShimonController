//
// Created by Raghavasimhan Sankaranarayanan on 4/9/22.
//

#ifndef SHIMONCONTROLLER_SERIALDEVICE_H
#define SHIMONCONTROLLER_SERIALDEVICE_H

#include <iostream>
#include "Def.h"
#include "ErrorDef.h"
#include <thread>
#include <mutex>

#ifndef SIMULATE
#include "serialib.h"
#endif

class SerialDevice {
public:

    ~SerialDevice() {
        reset();
    }

    Error_t openPort(const std::string& device) {
        std::lock_guard<std::mutex> lk(m_mtx);
#ifndef SIMULATE
        if (!m_serial.isDeviceOpen())
            if (m_serial.openDevice(device.c_str()) != 0) return kFileOpenError;
#endif
        return kNoError;
    }

    [[nodiscard]] bool isPortOpen() const {
#ifndef SIMULATE
        return m_serial.isDeviceOpen();
#endif
        return true;
    }

    Error_t init(int iBaudrate) {
        std::lock_guard<std::mutex> lk(m_mtx);
#ifndef SIMULATE
        if (!m_serial.isDeviceOpen()) return kNotOpenedError;
        if (m_serial.init(iBaudrate) != 1) return kNotInitializedError;
        if (m_serial.flushReceiver() == 0) return kFlushError;
#endif
        m_bInitialized = true;
        return kNoError;
    }

    Error_t reset() {
        std::lock_guard<std::mutex> lk(m_mtx);
#ifndef SIMULATE
        if (m_serial.isDeviceOpen()) m_serial.closeDevice();
#endif
        m_bInitialized = false;
        return kNoError;
    }

    Error_t write(const std::string& msg) {
        if (!m_bInitialized) return kNotInitializedError;
        std::lock_guard<std::mutex> lk(m_mtx);
#ifndef SIMULATE
        if (m_serial.writeString(msg.c_str()) != 1) return kWriteError;
#endif
        return kNoError;
    }

    Error_t write(const char* msg) {
        if (!m_bInitialized) return kNotInitializedError;
        std::lock_guard<std::mutex> lk(m_mtx);
#ifndef SIMULATE
        if (m_serial.writeString(msg) != 1) return kWriteError;
#endif
        return kNoError;
    }

    Error_t write(uint8_t* buf, size_t len) {
        if (!m_bInitialized) return kNotInitializedError;
        std::lock_guard<std::mutex> lk(m_mtx);
#ifndef SIMULATE
        if (m_serial.writeBytes(buf, len) != 1) return kWriteError;
#endif
        return kNoError;
    }

    Error_t readline(char* line, char finalChar='\n', int maxNbBytes=64, unsigned int timeout_ms=0) {
        if (!m_bInitialized) return kNotInitializedError;
        std::lock_guard<std::mutex> lk(m_mtx);
#ifndef SIMULATE
        int _n = m_serial.readString(line, finalChar, maxNbBytes, timeout_ms);
        if (_n != maxNbBytes) return kReadError;
#endif
        return kNoError;
    }

    Error_t readBytes(uint8_t* buf, size_t length=64, unsigned int timeout_ms=0) {
        if (!m_bInitialized) return kNotInitializedError;
        std::lock_guard<std::mutex> lk(m_mtx);
#ifndef SIMULATE
        int _n = m_serial.readBytes(buf, length, timeout_ms);
        if (_n != length) return kReadError;
#endif
        return kNoError;
    }

    [[nodiscard]] Error_t sendBreak(int time=0) {
        std::lock_guard<std::mutex> lk(m_mtx);
#ifndef SIMULATE
        if (m_serial.sendBreak(time) != 0) return kSetValueError;
#endif
        return kNoError;
    }

    [[nodiscard]] bool isInitialized() const { return m_bInitialized; }

private:
    std::atomic<bool> m_bInitialized = false;
    std::mutex m_mtx;

#ifndef SIMULATE
    serialib m_serial;
#endif
};

#endif //SHIMONCONTROLLER_SERIALDEVICE_H
