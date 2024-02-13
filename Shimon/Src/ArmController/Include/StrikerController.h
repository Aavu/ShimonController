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

using namespace std::chrono;
using std::chrono::milliseconds, std::chrono::duration_cast, std::chrono::steady_clock;

class StrikerController {
public:
    enum class Mode {
        None,
        Strike,
        Choreo,
        Tremolo,
        TremoloStop,
        Restart
    };

    struct StrikerMessage_t {
        uint8_t strikerId = 0;
        uint8_t midiVelocity = 0;
        int position = 0;
        int time_ms = 0;
        Mode mode = Mode::None;
        tp strikeTime{};
    };

    explicit StrikerController(tp programStartTime) : m_cmdManager(m_cv),
                               kProgramStartTime(programStartTime) {}

    ~StrikerController() {
        reset();
    }

    Error_t init(const std::string& host, int iPort) {
        LOG_TRACE("Initializing Striker Controller with Host: {} \t port: {}", host, iPort);
        Error_t e = m_transmitter.init(host, iPort);
        ERROR_CHECK(e, e);
        restart();

        m_bInitialized = true;
        return kNoError;
    }

    Error_t start() {
        if (!m_bInitialized) return kNotInitializedError;

        m_bRunning = true;
        m_pStrikerThread = std::make_unique<std::thread>([this] {strikerHandler();});

        return kNoError;
    }

    void stop() {
        m_bRunning = false;
        m_cv.notify_all();

        if (m_pStrikerThread) if (m_pStrikerThread->joinable()) m_pStrikerThread->join();
    }

    Error_t reset(bool stopThread = true) {
        if (!m_bInitialized) return kNoError;
        LOG_TRACE("Resetting Strikers");
        if (stopThread) { stop(); }
        m_transmitter.reset();
        m_bInitialized = false;

        return kNoError;
    }

    Error_t restart() {
        LOG_DEBUG("Restarting Strikers");
        Error_t e = strike(0, 0, Mode::Restart);
        ERROR_CHECK(e, e);

        setMode(Mode::Strike);

        resetBuffer();
        return kNoError;
    }

    void setMode(Mode mode) {
        if (mode != Mode::None) m_currentMode = mode;
    }

    Error_t scheduleStrike(uint8_t idCode, int target, int time_ms) {
        if (!m_bInitialized) return kNotInitializedError;

        std::lock_guard<std::mutex> lk(m_mtx);

        setMode(StrikerController::Mode::Choreo);
        StrikerMessage_t msg = {.strikerId = idCode,
                                .position = target,
                                .time_ms = time_ms,
                                .mode = m_currentMode,
                                .strikeTime = steady_clock::now() + milliseconds(DLY)};

        bool success = m_cmdManager.push(Port::Arm::Epos, msg);
        if (!success) return kBufferWriteError;
        return kNoError;
    }

    Error_t scheduleStrike(int note, int armId, int midiVelocity, tp strikeTime, Mode mode = Mode::None) {
        if (!m_bInitialized) return kNotInitializedError;

        std::lock_guard<std::mutex> lk(m_mtx);
        setMode(mode);
        uint8_t uiVelocity = std::max(0, std::min(127, midiVelocity));

        uint8_t strikerId = 1 << ((armId * 2) + Util::isWhiteKey(note));

        StrikerMessage_t msg = {.strikerId = strikerId,
                                .midiVelocity = uiVelocity,
                                .mode = m_currentMode,
                                .strikeTime = strikeTime};

        bool success = m_cmdManager.push(Port::Arm::Epos, msg);
        if (!success) return kBufferWriteError;
        return kNoError;
    }

    Error_t strike(int note, int armId, int midiVelocity, Mode mode = Mode::None) {
        if (!m_bInitialized) return kNotInitializedError;

        midiVelocity = std::max(0, std::min(127, midiVelocity));
        int strikerId = (armId * 2) + Util::isWhiteKey(note);
        return strike((1 << strikerId), midiVelocity, mode);
    }

    Error_t strike(uint8_t strikerId, int midiVelocity, Mode mode = Mode::None) {
        if (mode != Mode::None) setMode(mode);
        auto m = getMode(m_currentMode);
        return m_transmitter.send(strikerId, midiVelocity, m);
    }

    Error_t choreo(uint8_t strikerId, int position, int time_ms) {
        auto m = getMode(Mode::Choreo);
        return m_transmitter.send(strikerId, position, time_ms, m);
    }

    static char getMode(Mode mode) {
        switch (mode) {
            case Mode::Strike:
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
                return Mode::Strike;
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

    Error_t resetBuffer() {
        if (m_cmdManager.reset(Port::Arm::IAI)) return kNoError;
        return kBufferWriteError;
    }

private:
    StrikerHandler m_transmitter;
    Mode m_currentMode = Mode::Strike;
    bool m_bInitialized = false;

    std::mutex m_mtx;
    std::condition_variable m_cv;
    volatile std::atomic<bool> m_bRunning = false;

    std::unique_ptr<std::thread> m_pStrikerThread = nullptr;

    CommandManager<StrikerMessage_t, Port::Arm> m_cmdManager;
    std::list<int> m_bulkMsgStrikerIds;
    tp m_lastTime = std::chrono::steady_clock::now();
    tp kProgramStartTime;

    float getPosition(int midiVelocity) {
        midiVelocity = std::max(0, std::min(midiVelocity, 127));
        if (m_currentMode == Mode::Strike) return (float)midiVelocity * 0.317f + 9.683f;
//        else if (m_currentMode == Mode::Fast) return (float)midiVelocity * 0.095f + 7.905f;
        return 0;
    }

    float getAcceleration(int midiVelocity) {
        midiVelocity = std::max(0, std::min(midiVelocity, 127));
        if (m_currentMode == Mode::Strike) return (float)midiVelocity * 476.19f + 29524.81f;
//        else if (m_currentMode == Mode::Fast) return (float)midiVelocity * 158.73f + 99841.27f;
        return 0;
    }

    void strikerHandler() {
        bool success;
        while (m_bRunning) {
            StrikerMessage_t msg{};
            {
                std::unique_lock<std::mutex> lk(m_mtx);
                m_cv.wait(lk, [this] { return !m_bRunning || !m_cmdManager.isEmpty(); });
                if (!m_bRunning) break;
                success = m_cmdManager.pop(Port::Arm::Epos, msg);
            }

            if (success) {
                Error_t e;
                if (msg.mode == Mode::Choreo) {
                    e = choreo(msg.strikerId, msg.position, msg.time_ms);
                } else {
                    std::this_thread::sleep_until(msg.strikeTime - std::chrono::milliseconds(STRIKE_TIME));
                    e = strike(msg.strikerId, msg.midiVelocity, msg.mode);
                }
                if (e != kNoError) LOG_ERROR("Error striking. Error Code: {}", e);
            } else {
                LOG_ERROR("Cannot get Striker msg. Error Code: {}", kBufferReadError);
            }
        }
    }
};
#endif //SHIMONCONTROLLER_STRIKERCONTROLLER_H
