//
// Created by Raghavasimhan Sankaranarayanan on 4/12/22.
//

#include "Shimon.h"

Shimon::Shimon(int port, const std::string& hmConfigFile, size_t cmdBufferSize) :
                                                    m_oscListener(port)
                                                    , m_armController(m_oscListener, cmdBufferSize)
                                                    , m_headController(m_oscListener, hmConfigFile, cmdBufferSize)
{
    m_oscListener.setSystemMsgCallback([this](auto &&PH) {
        sysMsgCallback(std::forward<decltype(PH)>(PH));
    });
}

void Shimon::start() {
    // Start Arm thread
    m_armController.start();

    // Start head thread
    m_headController.start();

    // Start listening to OSC Messages
    m_oscListener.start();
}

void Shimon::stop() {
    m_oscListener.stop();
    m_headController.stop();
    m_armController.stop();
    LOG_TRACE("Shimon Stopped...");
}

Error_t Shimon::initArms() {
    LOG_INFO("Initializing Arms");
    return m_armController.init();
}

Error_t Shimon::initHead() {
    LOG_INFO("Initializing Head...");
    return m_headController.init();
}

void Shimon::sysMsgCallback(const char *msg) {
    if (strcmp(msg, "quit") == 0 || strcmp(msg, "close") == 0 || strcmp(msg, "exit") == 0) {
        stop();
    }
}
