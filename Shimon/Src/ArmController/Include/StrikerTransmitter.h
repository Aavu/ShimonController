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
#include "osc/OscOutboundPacketStream.h"
#include "ip/UdpSocket.h"
#include "Def.h"
#include "ErrorDef.h"

#define STRIKER_BUFFER_SIZE 64

class StrikerTransmitter {
public:
    StrikerTransmitter() : m_stream(m_buffer, STRIKER_BUFFER_SIZE) {}

    Error_t init(const std::string& host, int iPort) {
        if (m_pSocket) return kReInitializationError;
        m_pSocket = std::make_unique<UdpTransmitSocket>(IpEndpointName(host.c_str(), iPort));
        return kNoError;
    }

    Error_t send(uint8_t strikerId, float fPos, float fAcc, const char mode) {
        std::lock_guard<std::mutex> lk(m_mtx);
        if (!m_pSocket) return kNotInitializedError;

        static char buf[32];
        sprintf(buf, "%c ", mode);
//        std::stringstream ss;
//
//        ss << mode << " ";
//        for (int i = NUM_STRIKERS-1; i>-1; --i) {
//            if (strikerId & (1 << i)) ss << "1 ";
//            else ss << "0 ";
//        }

        for (int i = 0; i < NUM_STRIKERS; ++i) {
            sprintf(&buf[2 + (i*2)], "0 ");
        }

        for (int i = NUM_STRIKERS-1; i>-1; --i) {
            buf[2 + (i*2)] = (strikerId & (1 << i)) ? '1' : '0';
        }

        sprintf(&buf[18], "1 ");
//        ss << "1 ";     // Copying max patch. Not exactly sure why this is needed.

//        sprintf(m_cPos, "%03d ", (int)std::round(fPos));
//        sprintf(m_cAcc, "%06d", (int)std::round(fAcc));

        sprintf(&buf[20], "%03d %06d", (int)std::round(fPos), (int)std::round(fAcc));
//        ss << m_cPos << m_cAcc;

        buf[31] = 0;

        m_stream << osc::BeginMessage("/s");
        m_stream << buf;
        m_stream << osc::EndMessage;

        LOG_INFO(buf);
        m_pSocket->Send(m_stream.Data(), m_stream.Size());
        m_stream.Clear();

        return kNoError;
    }

private:
    std::unique_ptr<UdpTransmitSocket> m_pSocket;
    osc::OutboundPacketStream m_stream;
    std::mutex m_mtx;
    char m_buffer[STRIKER_BUFFER_SIZE]{};
};

#endif // SHIMONCONTROLLER_STRIKERTRANSMITTER_H
