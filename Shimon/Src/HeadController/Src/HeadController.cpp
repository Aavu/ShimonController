//
// Created by Raghavasimhan Sankaranarayanan on 4/12/22.
//

#include "HeadController.h"

HeadController::HeadController(OscListener &oscListener, const std::string& configFile, size_t cmdBufferSize) :
m_oscListener(oscListener),
m_motorController(configFile),
m_cmdManager(cmdBufferSize, m_cv),
m_breatheGesture(m_cmdManager),
m_headBangGesture(m_cmdManager)
{
    HeadCommandUtil::populateStrCmdMap();

}

HeadController::~HeadController() {
    reset();
}

Error_t HeadController::init(bool shouldHome) {
    m_oscListener.setHeadCallback([this](auto &&PH1, auto &&PH2, auto &&PH3) {
        headCallback(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2), std::forward<decltype(PH3)>(PH3));
    });

    Error_t e;

    e = m_motorController.init(shouldHome);
    ERROR_CHECK(e, e);

    m_bInitialized = true;
    LOG_INFO("Head Initialized");
    return kNoError;
}

Error_t HeadController::reset() {
    m_breatheGesture.stop();
    m_motorController.reset();

    m_bInitialized = false;
    return kNoError;
}

void HeadController::headCallback(const char *sCmd, std::optional<float> val1, std::optional<float> val2) {
    static auto startTime = std::chrono::steady_clock::now();
    static bool bFirstTime = true;

    HeadCommand cmd = HeadCommandUtil::getCommand(sCmd);

    if (HeadCommandUtil::isHighLevelCommand(cmd)) {
        handleHighLevelCommands(cmd, val1);
        return;
    }

    auto now = std::chrono::steady_clock::now();
//    auto diffTime = (double)std::chrono::duration_cast<std::chrono::microseconds>(now - startTime).count() / 1000.0;

//    if (diffTime < THREAD_TIME_PERIOD + 5) {
//        if (bFirstTime) bFirstTime = false;
//        else return;
//    }

    Port::Head port = HeadCommandUtil::getPortFor(cmd);
    if (port == Port::Head::kNumPorts)    // Invalid port or high level command
        return;

    HeadCmdPacket_t cmdPkt(cmd, val1, val2);
    {
        std::lock_guard<std::mutex> lk(m_mtx);
        if (!m_cmdManager.push(port, std::move(cmdPkt))) LOG_WARN("Warning: Cannot push: {} ", sCmd);
    }
    startTime = std::chrono::steady_clock::now();
}

Error_t HeadController::home() {
    if (!m_bInitialized) return kNotInitializedError;
    return m_motorController.home();
}

Error_t HeadController::start() {
    if (!m_bInitialized) return kNotInitializedError;
    Error_t e;
    e = m_motorController.zero();
    ERROR_CHECK(e, e);

    m_bRunning = true;
    m_pThread = std::make_unique<std::thread>([this] {threadHandler();});
    return kNoError;
}

Error_t HeadController::stop() {
    if (!m_bInitialized) return kNotInitializedError;
    m_bRunning = false;
    m_cv.notify_all();
    if (m_pThread->joinable()) m_pThread->join();
    m_motorController.reset();
    return kNoError;
}

void HeadController::threadHandler() {
    while (true) {
        auto loopTime = std::chrono::steady_clock::now();
        std::unique_lock<std::mutex> lk(m_mtx);
        LOG_TRACE("Waiting for Head Command...");
        m_cv.wait(lk, [this] { return !m_cmdManager.isEmpty() || !m_bRunning;});

        if (!m_bRunning) break;

        HeadCmdPacket_t cmdPkt;

        // Head Commands
        if (m_cmdManager.pop(Port::Head::Copley, cmdPkt)) {
            // Ignore any error
            m_motorController.send(cmdPkt);
        }

        // Mouth/Eyebrow Commands
        if (m_cmdManager.pop(Port::Head::Dxl, cmdPkt)) {
            // Ignore any error
            m_motorController.send(cmdPkt);
        }

        std::this_thread::sleep_until(loopTime + std::chrono::milliseconds (THREAD_TIME_PERIOD));
    }
}

void HeadController::handleHighLevelCommands(HeadCommand cmd, std::optional<float> val) {
    int iVal;
    switch (cmd) {
        case HeadCommand::Breathe:
            m_breatheGesture.start();
            break;
        case HeadCommand::StopBreath:
            m_breatheGesture.stop();
            break;
        case HeadCommand::HeadBang:
            iVal = (int)val.value_or(960);
            m_headBangGesture.beat(iVal);
            break;
        default:
            LOG_ERROR("Unknown HighLevel Command: {}", HeadCommandUtil::getCommand(cmd));
    }
}
