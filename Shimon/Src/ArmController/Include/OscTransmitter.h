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

#define OUT_BUFFER_SIZE 64

class OscTransmitter {
public:
    OscTransmitter() : m_stream(m_buffer, OUT_BUFFER_SIZE) {

    }

    Error_t init(const std::string& host, int iPort) {
        m_pSocket = std::make_unique<UdpTransmitSocket>(IpEndpointName(host.c_str(), iPort));
        return kNoError;
    }

    void send(float fVal) {
        if (!m_pSocket) return;
        m_stream << osc::BeginMessage("/arm") << fVal << osc::EndMessage;
        m_pSocket->Send(m_stream.Data(), m_stream.Size());
        m_stream.Clear();
    }

private:
    std::unique_ptr<UdpTransmitSocket> m_pSocket = nullptr;
    char m_buffer[OUT_BUFFER_SIZE]{};
    osc::OutboundPacketStream m_stream;

};

#endif //SHIMONCONTROLLER_OSCTRANSMITTER_H
