//
// Created by Raghavasimhan Sankaranarayanan on 5/16/22.
//

#ifndef SHIMONCONTROLLER_UDPSENDER_H
#define SHIMONCONTROLLER_UDPSENDER_H

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <cstring>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#include "ErrorDef.h"

class UdpSender {
public:
    UdpSender() = default;

    Error_t init(const std::string& host, int port) {
#ifndef SIMULATE
        if ( (m_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
            perror("socket creation failed");
            return kFileOpenError;
        }

        m_serverAddr.sin_family = AF_INET;
        m_serverAddr.sin_port = htons(port);
        m_serverAddr.sin_addr.s_addr = inet_addr(host.c_str());
#endif
        return kNoError;
    }

    Error_t send(const uint8_t* buf, size_t len) {
#ifndef SIMULATE
        if (m_fd < 0) return kNotInitializedError;
        ssize_t n = sendto(m_fd, buf, len, 0, (const sockaddr *)(&m_serverAddr), sizeof m_serverAddr);
        if (n != len) {
            LOG_ERROR("Sent {} bytes. Expected to send {} bytes", n, len);
            return kWriteError;
        }
#endif
        return kNoError;
    }

private:
    int m_fd = -1;
    sockaddr_in m_serverAddr{};
};

#endif //SHIMONCONTROLLER_UDPSENDER_H
