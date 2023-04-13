//
// Created by Raghavasimhan Sankaranarayanan on 4/9/22.
//

#ifndef SHIMONCONTROLLER_STRIKERHANDLER_H
#define SHIMONCONTROLLER_STRIKERHANDLER_H

#include <iostream>
#include <sstream>
#include <array>
#include <string>
#include <cmath>
#include <mutex>
#include "SerialDevice.h"
#include "UdpSender.h"
#include "Def.h"
#include "ErrorDef.h"

#define STRIKER_BUFFER_SIZE 4
#define STRIKER_SEND_RATE 10 // ms
#define STRIKER_UDP_ROUTE_INTERNAL "/striker"

class StrikerHandler {
public:
    typedef std::chrono::steady_clock::time_point _tp;

//    Error_t init(const std::string& host, int iPort) {
//        LOG_TRACE("Initializing Striker Transmitter");
//        return m_sender.init(host, iPort);
//    }
    ~StrikerHandler() {
        reset();
    }

    Error_t init(const std::string& device, int baudrate) {
        LOG_TRACE("Initializing Striker Handler");
        Error_t e = m_serial.openPort(device);
        ERROR_CHECK(e, e);

        e = m_serial.init(baudrate);
        ERROR_CHECK(e, e);

        m_bRunning = true;

        m_pSerialRecvThread = std::make_unique<std::thread>([this]() {
            recvHandler();
        });

        return kNoError;
    }

    void reset() {
        m_bRunning = false;
        if (m_pSerialRecvThread && m_pSerialRecvThread->joinable()) m_pSerialRecvThread->join();
        m_serial.reset();
    }

//    Error_t send(uint8_t strikerId, float fPos, float fAcc, const char mode) {
//        std::lock_guard<std::mutex> lk(m_mtx);
//        _tp now = std::chrono::steady_clock::now();
//
//        auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_lastTime).count();
//
//        static uint8_t buf[STRIKER_BUFFER_SIZE];
//        int pos = (int)std::round(fPos);
//        int acc = (int)std::round(fAcc);
//        sprintf((char*)buf, STRIKER_UDP_ROUTE_INTERNAL);
//        buf[8] = mode;
//        buf[9] = strikerId;
//        buf[10] = 1;    // Style - 1: regular single Stroke, 2: fast single stroke, 3-6: levels of damping from 5ms to 2000ms
//        buf[11] = pos;  // Deg
//        buf[12] = acc & 0xFF;
//        buf[13] = (acc >> 8) & 0xFF;
//        buf[14] = (acc >> 16) & 0xFF;
//        buf[15] = (acc >> 24) & 0xFF;
//
//        std::stringstream ss;
//        for (int i=0; i<STRIKER_BUFFER_SIZE; ++i) {
//            ss << (int)buf[i] << " ";
//        }
////        LOG_DEBUG("Striker msg: {}", ss.str());
//
//        Error_t e = m_sender.send(buf, STRIKER_BUFFER_SIZE);
//        ERROR_CHECK(e, e);
//
//        m_lastTime = now;
//        return kNoError;
//    }

    Error_t send(uint8_t strikerId, uint8_t midiVelocity, const char mode) {
        std::lock_guard<std::mutex> lk(m_mtx);
        _tp now = std::chrono::steady_clock::now();

        auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_lastTime).count();

        uint8_t buf[STRIKER_BUFFER_SIZE];
        buf[0] = mode;
        buf[1] = strikerId;
        buf[2] = midiVelocity;
        buf[3] = '\n';
//        LOG_INFO("Striker msg: {} {} {}", buf[0], buf[1], buf[2]);
        Error_t e = m_serial.write(buf, STRIKER_BUFFER_SIZE);
        ERROR_CHECK(e, e);

        m_lastTime = now;
        return kNoError;
    }

private:
    SerialDevice m_serial;
    std::mutex m_mtx;
    std::atomic<bool> m_bRunning = false;
    std::unique_ptr<std::thread> m_pSerialRecvThread = nullptr;

    void recvHandler() {
        m_bRunning = true;
        while (m_bRunning) {
            char msg[64];
            int n = m_serial.readline(msg, '\n', 64, 10);
            if (n > 0) {
                handleLog(msg);
            }
            if (!m_bRunning) return;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    static void handleLog(const std::string& msg) {
        if (msg.substr(0, 4) == "INFO") {
            LOG_INFO("{}", msg.substr(6));
        } else if (msg.substr(0, 5) == "ERROR") {
            LOG_ERROR("{}", msg.substr(7));
        } else if (msg.substr(0, 5) == "TRACE") {
            LOG_TRACE("{}", msg.substr(7));
        } else if (msg.substr(0, 4) == "WARN") {
            LOG_WARN("{}", msg.substr(6));
        } else {
            LOG_WARN("Unknown striker message... - {}", msg);
        }
    }

    _tp m_lastTime;
};

#endif // SHIMONCONTROLLER_STRIKERHANDLER_H
