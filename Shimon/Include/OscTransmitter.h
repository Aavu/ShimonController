//
// Created by Raghavasimhan Sankaranarayanan on 4/11/22.
//

#ifndef SHIMONCONTROLLER_OSCTRANSMITTER_H
#define SHIMONCONTROLLER_OSCTRANSMITTER_H

#include <iostream>
#include "osc/OscOutboundPacketStream.h"
#include "ip/UdpSocket.h"

#include "Def.h"
#include "ErrorDef.h"
#include "StatusDef.h"

#define OUT_BUFFER_SIZE 64

class OscTransmitter {
public:
    OscTransmitter() : m_stream(m_buffer, OUT_BUFFER_SIZE) {

    }

    Error_t init(const std::string& host, int iPort) {
        m_pRemoteEndPoint = std::make_unique<IpEndpointName>(host.c_str(), iPort);
        m_pSocket = std::make_unique<UdpTransmitSocket>(*m_pRemoteEndPoint);
        return kNoError;
    }

    void send(const char* addrPattern, Status_t status) {
        if (!m_pSocket) return;
        m_stream << osc::BeginMessage(addrPattern) << (int) status << osc::EndMessage;
        m_pSocket->Send(m_stream.Data(), m_stream.Size());
        m_stream.Clear();
    }

    void send(const char* addrPattern, const std::array<int, NUM_ARMS>& positions) {
        if (!m_pSocket) return;
        m_stream << osc::BeginMessage(addrPattern);
        for (int i=0; i<NUM_ARMS; ++i) {
            m_stream << positions[i];
        }
        m_stream << osc::EndMessage;
        m_pSocket->Send(m_stream.Data(), m_stream.Size());
        m_stream.Clear();
    }

    void setHost(unsigned long host, int port) {
        m_pRemoteEndPoint = std::make_unique<IpEndpointName>(host, port);
        m_pSocket = std::make_unique<UdpTransmitSocket>(*m_pRemoteEndPoint);
    }

    void setHost(const std::string& host, int port) {
        m_pRemoteEndPoint = std::make_unique<IpEndpointName>(host.c_str(), port);
        m_pSocket = std::make_unique<UdpTransmitSocket>(*m_pRemoteEndPoint);
    }

    [[nodiscard]] std::string getHost() const {
        auto addr = m_pRemoteEndPoint->address;
        std::string host = std::to_string(addr >> 24 & 0xFF) + "."
                + std::to_string(addr >> 16 & 0xFF) + "."
                + std::to_string(addr >> 8 & 0xFF) + "."
                + std::to_string(addr & 0xFF);
        return std::move(host);
    }

private:
    std::unique_ptr<UdpTransmitSocket> m_pSocket = nullptr;
    std::unique_ptr<IpEndpointName> m_pRemoteEndPoint = nullptr;
    char m_buffer[OUT_BUFFER_SIZE]{};
    osc::OutboundPacketStream m_stream;

};

#endif //SHIMONCONTROLLER_OSCTRANSMITTER_H
