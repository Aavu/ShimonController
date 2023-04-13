//
// Created by Raghavasimhan Sankaranarayanan on 4/9/22.
//

#ifndef SHIMONCONTROLLER_STRIKERCONTROLLER_H
#define SHIMONCONTROLLER_STRIKERCONTROLLER_H

#include <iostream>
#include <thread>
#include <mutex>
#include "Def.h"
#include "Util.h"
#include "CommandManager.h"
#include "StrikerHandler.h"
#include "ErrorDef.h"
#include "Logger.h"

#define BINNING_TIME_THRESHOLD 20 // ms
#define BULK_MSG_MIDI_VELOCITY 80

using std::chrono::milliseconds, std::chrono::duration_cast, std::chrono::steady_clock;

class StrikerController {
public:
    enum class Mode {
        None,
        Slow,
        Fast,
        Choreo,
        Tremolo,
        TremoloStop,
        Restart
    };

    struct StrikerMessage_t {
        uint8_t strikerId;
        uint8_t midiVelocity;
        float position;
        float acceleration;
        Mode mode;
        tp strikeTime;
    };

    explicit StrikerController(size_t cmdBufferSize) : m_cmdManager(cmdBufferSize, m_cv) {}

    ~StrikerController() {
        reset();
    }

    Error_t init(const std::string& host, int iPort) {
        LOG_TRACE("Initializing Striker Controller with Host: {} \t port: {}", host, iPort);
        Error_t e = m_transmitter.init(host, iPort);
        ERROR_CHECK(e, e);
        setMode(Mode::Slow);

        m_bInitialized = true;
        return kNoError;
    }

    Error_t start() {
        if (!m_bInitialized) return kNotInitializedError;

        m_bRunning = true;
        m_pMultiStrikerMsgThread = std::make_unique<std::thread>([this]{ msgTimeoutHandler(); });
        m_pStrikerThread = std::make_unique<std::thread>([this] {strikerHandler();});

        return kNoError;
    }

    void stop() {
        m_bRunning = false;
        m_cv.notify_all();
        m_bulkCv.notify_all();

        if (m_pStrikerThread) if (m_pStrikerThread->joinable()) m_pStrikerThread->join();
        if (m_pMultiStrikerMsgThread) if (m_pMultiStrikerMsgThread->joinable()) m_pMultiStrikerMsgThread->join();
    }

    Error_t reset() {
        if (!m_bInitialized) return kNoError;
        LOG_TRACE("Resetting Strikers");
        stop();
        m_transmitter.reset();
        m_bInitialized = false;

        return kNoError;
    }

    Error_t restart() {
        LOG_DEBUG("Restarting Strikers");
        Error_t e = strike(0, 0, Mode::Restart);
        ERROR_CHECK(e, e);

        setMode(Mode::Slow);
        return kNoError;
    }

    void setMode(Mode mode) {
        if (mode != Mode::None) m_currentMode = mode;
//        LOG_DEBUG("Setting Strike mode to: {}", getMode(m_currentMode));
    }
    Mode getMode() { return m_currentMode; }

    Error_t scheduleStrike(int note, int armId, int midiVelocity, tp strikeTime, Mode mode = Mode::None) {
        if (!m_bInitialized) return kNotInitializedError;

        setMode(mode);
        uint8_t uiVelocity = std::max(0, std::min(127, midiVelocity));

        uint8_t strikerId = 1 << ((armId * 2) + Util::isWhiteKey(note));

        StrikerMessage_t msg = {.strikerId = strikerId,
                                .midiVelocity = uiVelocity,
                                .position = getPosition(uiVelocity),
                                .acceleration = getAcceleration(uiVelocity),
                                .mode = m_currentMode,
                                .strikeTime = strikeTime};

        bool success = m_cmdManager.push(Port::Arm::Epos, msg);
        if (!success) return kBufferWriteError;
        return kNoError;
    }

    Error_t strike(int note, int armId, int midiVelocity, Mode mode = Mode::None) {
        if (!m_bInitialized) return kNotInitializedError;
//        auto now = steady_clock::now();

        midiVelocity = std::max(0, std::min(127, midiVelocity));
        int strikerId = (armId * 2) + Util::isWhiteKey(note);
        return strike((1 << strikerId), midiVelocity, mode);
//        {
//            std::lock_guard<std::mutex> lk(m_bulkMtx);
//            m_bulkMsgStrikerIds.push_back(strikerId);
//            m_lastTime = now;
//        }

//        m_bulkCv.notify_all();
//        return kNoError;
    }

    Error_t strike(uint8_t strikerId, int midiVelocity, Mode mode = Mode::None) {
        if (mode != Mode::None) setMode(mode);
        return m_transmitter.send(strikerId, midiVelocity, getMode(m_currentMode));
//        return strike(strikerId, getPosition(midiVelocity), getAcceleration(midiVelocity), mode);
    }

//    Error_t strike(uint8_t strikerId, float position, float acceleration, Mode mode = Mode::None) {
//
//        return m_transmitter.send(strikerId, position, acceleration, getMode(m_currentMode));
//    }

    static char getMode(Mode mode) {
        switch (mode) {
            case Mode::Slow:
            case Mode::Fast:
                return 's';
            case Mode::Tremolo:
                return 't';
            case Mode::Choreo:
                return 'c';
            case Mode::TremoloStop:
                return 'p';
            case Mode::Restart:
                return 'r';
            default:
                return 'u';   // Unknown
        }
    }

    static Mode getMode(char mode) {
        switch (mode) {
            case 's':
                return Mode::Slow;
            case 't':
                return Mode::Tremolo;
            case 'c':
                return Mode::Choreo;
            case 'p':
                return Mode::TremoloStop;
            default:
                return Mode::None;   // Unknown
        }
    }

private:
    StrikerHandler m_transmitter;
    Mode m_currentMode = Mode::Slow;
    bool m_bInitialized = false;

    std::mutex m_mtx, m_bulkMtx;

    std::condition_variable m_cv, m_bulkCv;
    volatile std::atomic<bool> m_bRunning = false;

    std::unique_ptr<std::thread> m_pStrikerThread = nullptr;
    std::unique_ptr<std::thread> m_pMultiStrikerMsgThread;

    CommandManager<StrikerMessage_t, Port::Arm> m_cmdManager;
    std::list<int> m_bulkMsgStrikerIds;
    tp m_lastTime = std::chrono::steady_clock::now();

    float getPosition(int midiVelocity) {
        midiVelocity = std::max(0, std::min(midiVelocity, 127));
        if (m_currentMode == Mode::Slow) return (float)midiVelocity * 0.317f + 9.683f;
        else if (m_currentMode == Mode::Fast) return (float)midiVelocity * 0.095f + 7.905f;
        return 0;
    }

    float getAcceleration(int midiVelocity) {
        midiVelocity = std::max(0, std::min(midiVelocity, 127));
        if (m_currentMode == Mode::Slow) return (float)midiVelocity * 476.19f + 29524.81f;
        else if (m_currentMode == Mode::Fast) return (float)midiVelocity * 158.73f + 99841.27f;
        return 0;
    }

    void msgTimeoutHandler() {
        while (m_bRunning) {
            std::unique_lock<std::mutex> lk(m_bulkMtx);
            m_bulkCv.wait(lk, [this] { return !m_bRunning || !m_bulkMsgStrikerIds.empty(); });
            if (!m_bRunning) break;
            auto now = steady_clock::now();

            auto diffTime = duration_cast<milliseconds>(now - m_lastTime).count();
            if (diffTime >= BINNING_TIME_THRESHOLD) {
                uint8_t strikerId = Util::computeStrikerId(m_bulkMsgStrikerIds);
                Error_t e = strike(strikerId, BULK_MSG_MIDI_VELOCITY);
                if (e != kNoError) {
                    LOG_ERROR("Error Code: {}", e);
                }
                m_bulkMsgStrikerIds.clear();
            }
        }
    }

    void strikerHandler() {
        bool success = false;
        LOG_DEBUG("Striker handler started...");
        while (m_bRunning) {
            StrikerMessage_t msg{};
            {
                std::unique_lock<std::mutex> lk(m_mtx);
                m_cv.wait(lk, [this] { return !m_bRunning || !m_cmdManager.isEmpty(); });
                if (!m_bRunning) break;
                success = m_cmdManager.pop(Port::Arm::Epos, msg);
            }

            if (success) {
                std::this_thread::sleep_until(msg.strikeTime - std::chrono::milliseconds(STRIKE_TIME));
                Error_t e = strike(msg.strikerId, msg.midiVelocity, msg.mode);
                if (e != kNoError) LOG_ERROR("Error striking. Error Code: {}", e);
            } else {
                LOG_ERROR("Cannot get Striker msg. Error Code: {}", kBufferReadError);
            }

        }
    }
};
#endif //SHIMONCONTROLLER_STRIKERCONTROLLER_H
