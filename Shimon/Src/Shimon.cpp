//
// Created by Raghavasimhan Sankaranarayanan on 4/12/22.
//

#include "Shimon.h"

Shimon::Shimon(int port, const std::string& hmConfigFile, tp programStartTime) :
                                                    m_oscListener(port)
                                                    , m_positionTransmitQueue(POSITION_BUFFER_SIZE)
                                                    , m_kProgramStartTime(programStartTime)
                                                    , m_armController(m_oscListener, programStartTime)
                                                    , m_headController(m_oscListener, hmConfigFile, programStartTime)
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

    m_bRunning = true;
    m_pMasterTransmitThread = std::make_unique<std::thread>([this]() {
        masterTransmitHandler();
    });

    // Start listening to OSC Messages. Call this last!. This is a blocking call.
    m_oscListener.start();
}

void Shimon::stop() {
    m_oscListener.stop();

    m_headController.stop();
    m_armController.stop();
    m_bRunning = false;
    m_cv.notify_all();
    if (m_pMasterTransmitThread) m_pMasterTransmitThread->join();
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

Error_t Shimon::initMasterTransmitter(const std::string& host, int port) {
    return m_oscTransmitter.init(host, port);
}

void Shimon::masterTransmitHandler() {
    while (m_bRunning) {
        bool success = false;
        std::array<int, NUM_ARMS> position{};
        {
            std::unique_lock<std::mutex> lk(m_mtx);
            m_cv.wait(lk, [this] { return !m_positionTransmitQueue.isEmpty() || !m_bRunning; });
            if (!m_bRunning) break;
            success = m_positionTransmitQueue.pop(position);
        }

        if (success) {
            m_oscTransmitter.send("/arm", position);
        }
    }
}

void Shimon::sysMsgCallback(const char *msg) {
    if (strcmp(msg, "quit") == 0 || strcmp(msg, "close") == 0 || strcmp(msg, "exit") == 0) {
        stop();
    } else if (strcmp(msg, "setHost") == 0 || strcmp(msg, "SetHost") == 0 || strcmp(msg, "sethost") == 0) {
        m_oscTransmitter.setHost(m_oscListener.getHost(), MASTER_PORT);
        LOG_INFO("Host and post set to {}, {}", m_oscTransmitter.getHost(), MASTER_PORT);
    }
}

Error_t Shimon::init(const std::string& masterHostName, int masterPort) {
    Error_t e;

#ifdef USE_ARMS
    e = initArms();
    if (e != kNoError) {
        LOG_ERROR("Arm Init failed. Error Code {}", e);
        return e;
    }

    m_armController.setStatusCallback([this](auto &&PH1) {
        armStatusCallback(std::forward<decltype(PH1)>(PH1));
    });

    m_armController.setPositionCallback([this](auto && PH1) {
        armPositionCallback(std::forward<decltype(PH1)>(PH1));
    });
#else
    LOG_INFO("Not using Arms");
#endif


#ifdef USE_HEAD
    e = initHead();
    if (e != kNoError) {
        LOG_ERROR("Head Init failed. Error Code {}", e);
        return e;
    }
#else
    LOG_INFO("Not using Head");
#endif

    e = initMasterTransmitter(masterHostName, masterPort);
    if (e != kNoError) {
        LOG_ERROR("master transmitter Init failed. Error Code {}", e);
        return e;
    }

    return kNoError;
}

void Shimon::armStatusCallback(Status_t status) {
    m_oscTransmitter.send("/arm-status", status);
}

void Shimon::headStatusCallback(Status_t status) {
    m_oscTransmitter.send("/head-status", status);
}

void Shimon::armPositionCallback(std::array<int, 4> position) {
    m_positionTransmitQueue.push(position);
    m_cv.notify_all();
}
