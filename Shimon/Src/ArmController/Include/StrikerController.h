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
#include "StrikerTransmitter.h"
#include "ErrorDef.h"
#include "Logger.h"

#define NUM_STRIKER_THREADS 32

class StrikerController {
public:
    enum class Mode {
        None,
        Slow,
        Fast,
        Choreo,
        Tremolo,
        TremoloStop
    };

    struct StrikerMessage_t {
        uint8_t strikerId;
        float position;
        float acceleration;
        Mode mode;
    };

    explicit StrikerController(size_t cmdBufferSize) : m_cmdManager(cmdBufferSize, m_cv) {}

    ~StrikerController() {
        reset();
    }

    Error_t init(const std::string& host, int iPort) {
        Error_t e = m_transmitter.init(host, iPort);
        ERROR_CHECK(e, e);
        setMode(Mode::Slow);

        m_bRunning = true;
        for (auto & thread : m_pThreadPool) {
            thread = std::make_unique<std::thread>([this] { threadPoolHandler(); });
        }

        m_bInitialized = true;
        return kNoError;
    }

    Error_t reset() {
        m_bRunning = false;
        m_cv.notify_all();
        for (auto & thread : m_pThreadPool)
            if (thread)
                if (thread->joinable())
                    thread->join();

        m_bInitialized = false;

        return kNoError;
    }

    void setMode(Mode mode) {
        if (mode != Mode::None) m_currentMode = mode;
        LOG_INFO("Setting Strike mode to: {}", getMode(m_currentMode));
    }
    Mode getMode() { return m_currentMode; }

    Error_t strike(int note, int armId, int midiVelocity, Mode mode = Mode::None) {
        if (!m_bInitialized) return kNotInitializedError;
        int strikerId = (armId * 2) + Util::isWhiteKey(note);
        return strike((1 << strikerId), midiVelocity, mode);
    }

    Error_t strike(uint8_t strikerId, int midiVelocity, Mode mode = Mode::None) {
        std::lock_guard<std::mutex> lk(m_mtx);
        StrikerMessage_t msg {
                .strikerId = strikerId,
                .position = getPosition(midiVelocity),
                .acceleration = getAcceleration(midiVelocity)
        };

        if (m_cmdManager.push(Port::Arm::Epos, msg)) {
            return kNoError;
        }
        return kSetValueError;
    }

    Error_t strike(uint8_t strikerId, float position, float acceleration, Mode mode = Mode::None) {
        if (mode != Mode::None) setMode(mode);
        return m_transmitter.send(strikerId, position, acceleration, getMode(m_currentMode));
    }

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
    StrikerTransmitter m_transmitter;
    Mode m_currentMode = Mode::Slow;
    bool m_bInitialized = false;

    std::mutex m_mtx;
    std::condition_variable m_cv;
    volatile std::atomic<bool> m_bRunning = false;

    std::unique_ptr<std::thread> m_pThreadPool[NUM_STRIKER_THREADS];

    CommandManager<StrikerMessage_t, Port::Arm> m_cmdManager;

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

    void threadPoolHandler() {
        while (true) {
            {
                std::unique_lock<std::mutex> ulk(m_mtx);
                m_cv.wait(ulk, [this] { return !m_cmdManager.isEmpty() || !m_bRunning; });
            }
            if (!m_bRunning) break;

            bool success = false;
            StrikerMessage_t msg{};
            {
//                LOG_INFO("Num cmds: {}", m_cmdManager.getNumCommandsInQueue(Port::Arm::Epos));
                success = m_cmdManager.pop(Port::Arm::Epos, msg);
            }

            if (success) {
//            LOG_INFO("{} {} {}", msg.strikerId, msg.position, msg.acceleration);
                std::this_thread::sleep_for(std::chrono::milliseconds(DLY));
                Error_t e = strike(msg.strikerId, msg.position, msg.acceleration);
                if (e != kNoError) LOG_ERROR("Error Code: {}", e);
            }
        }
    }
};
#endif //SHIMONCONTROLLER_STRIKERCONTROLLER_H
