//
// Created by Raghavasimhan Sankaranarayanan on 4/9/22.
//

#ifndef SHIMONCONTROLLER_STRIKERTRANSMITTER_H
#define SHIMONCONTROLLER_STRIKERTRANSMITTER_H

#include <iostream>
#include <sstream>
#include <array>
#include <string>
#include <cmath>
#include <mutex>
#include "UdpSender.h"
#include "Def.h"
#include "ErrorDef.h"

#define STRIKER_BUFFER_SIZE 16
#define STRIKER_SEND_RATE 10 // ms
#define STRIKER_UDP_ROUTE_INTERNAL "/striker"

class StrikerTransmitter {
public:
    typedef std::chrono::steady_clock::time_point _tp;

    Error_t init(const std::string& host, int iPort) {
        LOG_TRACE("Initializing Striker Transmitter");
        return m_sender.init(host, iPort);
    }

    Error_t send(uint8_t strikerId, float fPos, float fAcc, const char mode) {
        std::lock_guard<std::mutex> lk(m_mtx);
        _tp now = std::chrono::steady_clock::now();

        auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_lastTime).count();
//        if (diff < STRIKER_SEND_RATE) {
//            LOG_WARN("Ignoring message. Sending Too Fast. Slow Down...");
//            return kNoError;
//        }

        static uint8_t buf[STRIKER_BUFFER_SIZE];
        int pos = (int)std::round(fPos);
        int acc = (int)std::round(fAcc);
        sprintf((char*)buf, STRIKER_UDP_ROUTE_INTERNAL);
        buf[8] = mode;
        buf[9] = strikerId;
        buf[10] = 1;
        buf[11] = pos;
        buf[12] = acc & 0xFF;
        buf[13] = (acc >> 8) & 0xFF;
        buf[14] = (acc >> 16) & 0xFF;
        buf[15] = (acc >> 24) & 0xFF;

        std::stringstream ss;
        for (int i=0; i<STRIKER_BUFFER_SIZE; ++i) {
            ss << (int)buf[i] << " ";
        }
//        LOG_DEBUG("Striker msg: {}", ss.str());

        Error_t e = m_sender.send(buf, STRIKER_BUFFER_SIZE);
        ERROR_CHECK(e, e);

        m_lastTime = now;
        return kNoError;
    }

private:
    UdpSender m_sender;
    std::mutex m_mtx;

    _tp m_lastTime;
};

#endif // SHIMONCONTROLLER_STRIKERTRANSMITTER_H
