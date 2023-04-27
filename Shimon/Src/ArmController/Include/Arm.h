//
// Created by Raghavasimhan Sankaranarayanan on 4/9/22.
//

#ifndef SHIMONCONTROLLER_ARM_H
#define SHIMONCONTROLLER_ARM_H

#include <iostream>
#include "CommandManager.h"
#include "IAIController.h"
#include "SliderModel.h"
#include "Def.h"
#include "Logger.h"

using namespace std::chrono;
class Arm {
public:
    typedef steady_clock::time_point _tp;
    struct Message_t {
        int arm_id{};
        int target{};
        float acceleration{};
        float v_max{};
        int midiNote{};
        int midiVelocity{};
        int time_ms{};
        _tp arrivalTime;
        _tp msgTime;

        void pprint() const {
            std::cout << "Id: " << arm_id
                      << "\t target: " << target
                      << "\t acc: " << acceleration
                      << "\t vmax: " << v_max
                      << "\t midi vel: " << midiVelocity
                      << "\t time (ms): " << time_ms
                      << std::endl;
        }
    };

    struct Boundary {
        int left=0;
        int right=0;
    };

    Arm(int id,
        int iHomePosition,
        int w, int b) : m_id(id),
                        kHomePosition(iHomePosition),
                        m_controller(IAIController::getInstance()),
                        m_iW(w), m_iB(b),
                        m_cmdManager(m_cmdCv)
    {
        reset();
    }

    Error_t reset() {
        Error_t e;
        m_bMoving = false;
        e = _setPosition(kHomePosition);
        if (e != kNoError) {
            LOG_ERROR("Error Code: {}", e);
            return e;
        }

        Message_t msg{};
        msg.arm_id = m_id;
        msg.target = kHomePosition;
        msg.msgTime = steady_clock::now()- milliseconds(DLY);   // Basically infinite time before now
        msg.arrivalTime = steady_clock::now() + milliseconds(DLY);
        msg.time_ms = DLY;
        e = setMessage(msg);
        if (e != kNoError) {
            LOG_ERROR("Error Code: {}", e);
            return e;
        }

        m_mostRecentMsg = {m_id,
                           kHomePosition,
                           0, 0, MIN_NOTE, 0, DLY,
                           steady_clock::now(),
                           steady_clock::now()- milliseconds(DLY)};
//        LOG_DEBUG("Id: {} \t position: {} \t lb: {} \t rb: {}", m_id, m_iPosition, m_boundary.left, m_boundary.right);
        return kNoError;
    }

    Error_t start() {
        m_bRunning = true;
        m_pThreadPool = std::make_unique<std::thread>([this] { threadPoolHandler(); });
        kProgramStartTime = steady_clock::now();
        return kNoError;
    }

    void stop() {
        m_bRunning = false;
        m_bMoving = false;
        m_cmdCv.notify_all();

        if (m_pThreadPool)
            if (m_pThreadPool->joinable()) m_pThreadPool->join();
    }

    Error_t addMessageToQueue(const Message_t& msg) {
        std::lock_guard<std::mutex> lk(m_mtx);
        bool success = m_cmdManager.push(Port::Arm::IAI, msg);
        if (!success) {
            LOG_ERROR("Unable to push msg");
            return kSetValueError;
        }
        m_mostRecentMsg = msg;
        return kNoError;
    }

    Error_t setMessage(const Message_t& msg) {
        std::lock_guard<std::mutex> lk(m_mtx);
        m_lastMsg = m_msg;
        m_msg = msg;
        _updateTargetBoundaries();
        _updateTrajectory();
        return kNoError;
    }

    [[nodiscard]] int getPosition() const { return m_iPosition; }

    void updatePosition(int iPos) {
        _setPosition(iPos);
    }

    bool updatePositionFromTrajectory(int interval_ms=1) {
        static int lastPos[NUM_ARMS] = {-1, -1, -1, -1};
        std::lock_guard<std::mutex> lk(m_mtx);

        m_iTrajIdx = std::max(m_iTrajIdx + interval_ms, 0);
        if (m_iTrajIdx > m_iEndTimeIdx) {
            m_iTrajIdx = m_iEndTimeIdx;
            m_bMoving = false;
        }

        if (!m_bMoving)
            return false;

        int pos = m_aiTraj[m_iTrajIdx];

        if (pos != lastPos[m_id]) {
            updatePosition(pos);
            lastPos[m_id] = m_iPosition;
            return true;
        }

        return false;
    }

    [[nodiscard]] int getID() const { return m_id; }

    [[nodiscard]] int getLeftBoundary() const {
        return std::min(m_boundary.left, std::min(m_tBoundary.left, getInstantaneousLeftBoundary()));
    }

    [[nodiscard]] int getRightBoundary() const {
        return std::max(m_boundary.right, std::max(m_tBoundary.right, getInstantaneousRightBoundary()));
    }

    [[nodiscard]] int getInstantaneousLeftBoundary() const {
        return m_iPosition - kBoundaries[m_id][0];
    }

    [[nodiscard]] int getInstantaneousRightBoundary() const {
        return m_iPosition + kBoundaries[m_id][1];
    }

    [[nodiscard]] int getPositionInSliderCoordinate(int pos) const { return pos * m_iW + m_iB; }

    // These are the info if all cmds in the queue is executed. This is not the last info at this point in time!
    [[nodiscard]] const _tp& getMostRecentMsgTime() const { return m_mostRecentMsg.msgTime; }
    [[nodiscard]] const _tp& getMostRecentArrivalTime() const { return m_mostRecentMsg.arrivalTime; }
    [[nodiscard]] int getMostRecentTarget() const { return m_mostRecentMsg.target; }

    /*
     * Computes the v max and acceleration for the arm. Returns kNoError if the motion is possible.
     */
    Error_t computeMotionParam(Message_t& msg) const {
        // Displacement is the difference between the target and the most recent target
        auto dPos = std::abs(msg.target - getMostRecentTarget());
        if (dPos == 0) {
            msg.acceleration = 0;
            msg.v_max = 0;
            return kNoError;    // No need to send kAlreadyThereError as IAI controller will take care of it.
        }

        if (msg.time_ms < 1)
            return kArgLimitError;

        auto fPos = (float)dPos / 1000.f;

        auto fTime = (float)msg.time_ms / 1000.f;

        auto temp = std::min(VELOCITY_LIMIT, 2.f*fPos/fTime);
        msg.v_max = 1000.f * (float)temp;

        msg.acceleration = std::pow(temp, 2.f) / (fTime * temp - fPos) / 9.8f;
        if (msg.acceleration > ACC_LIMIT || msg.acceleration < 0) return kArgLimitError;

        return kNoError;
    }

    Error_t move(const Message_t& msg) {
        setMessage(msg);
        return move();
    }

    Error_t move() {
        int scPosition = getPositionInSliderCoordinate(m_msg.target);
        Modbus::Message_t mbMsg {
                .armID = m_id,
                .position = scPosition,
                .precision = 0.1,
                .v_max = (int)std::round(m_msg.v_max),
                .acceleration = m_msg.acceleration,
                .push = 0
        };

        {
            Error_t e = m_controller.setPosition(mbMsg);
            if (e != kNoError && e != kAlreadyThereError) {
                LOG_ERROR("Set Position Error: {}", e);
                return e;
            }
        }

//        LOG_DEBUG("Arm: {} \t target: {} \t acc: {} \t v_max: {}", m_id, getTarget(), m_msg.acceleration, (int)std::round(m_msg.v_max));
//        LOG_DEBUG("Arm: {} \t lm: {} \t rm: {}", m_id, getLeftBoundary(), getRightBoundary());

        LOG_INFO("Note: {} \t Arm: {} \t time: {} \t pos: {} \t target: {} \t acc: {} \t v_max: {} \t msgTime: {} \t arrTime: {}",
                 m_msg.midiNote,
                 m_msg.arm_id,
                 m_msg.time_ms,
                 m_lastMsg.target,
                 m_msg.target,
                 m_msg.acceleration,
                 (int)std::round(m_msg.v_max),
                 duration_cast<milliseconds>(m_msg.msgTime - kProgramStartTime).count(),
                 duration_cast<milliseconds>(m_msg.arrivalTime - kProgramStartTime).count());
        m_iTrajIdx = 0;
        m_bMoving = true;
        return kNoError;
    }

    Error_t servoOn(bool bTurnOn = true) {
        if (m_bServoOn == bTurnOn) return kNoError;

        Error_t e;
        e = m_controller.setServo(m_id, true);
        ERROR_CHECK(e, e);
        std::this_thread::sleep_for(milliseconds(100));
        m_bServoOn = bTurnOn;
        return kNoError;
    }

    Error_t resetBuffer() {
        if (m_cmdManager.reset(Port::Arm::IAI)) return kNoError;
        return kBufferWriteError;
    }

private:
    int m_id = -1;
    int m_iPosition = -1;
    const int kHomePosition;
    std::atomic<bool> m_bRunning = false;
    std::atomic<bool> m_bMoving = false;
    IAIController& m_controller;
    Boundary m_boundary{};
    Boundary m_tBoundary{};
    // Multiplier for slider geometry conversion
    const int m_iW = 1;
    // Bias for slider geometry conversion
    const int m_iB = 0;

    tp kProgramStartTime = steady_clock::now();

    Message_t m_msg, m_lastMsg;
    Message_t m_mostRecentMsg;

    CommandManager<Message_t, Port::Arm> m_cmdManager;

    std::unique_ptr<std::thread> m_pThreadPool = nullptr;

    std::mutex m_mtx;
    std::condition_variable m_cmdCv;

    int m_aiTraj[DLY]{};
    int m_iEndTimeIdx = DLY-1;
    volatile std::atomic<int> m_iTrajIdx = 0;
    bool m_bServoOn = false;

    Error_t _updateBoundaries() {
        m_boundary.left = m_iPosition - kBoundaries[m_id][0];
        m_boundary.right = m_iPosition + kBoundaries[m_id][1];
        return kNoError;
    }

    Error_t _updateTargetBoundaries() {
        m_tBoundary.left = m_msg.target - kBoundaries[m_id][0];
        m_tBoundary.right = m_msg.target + kBoundaries[m_id][1];
        return kNoError;
    }

    void _updateTrajectory() {
        SliderModel::compute(m_msg.time_ms, m_lastMsg.target, m_msg.target, m_msg.acceleration, m_msg.v_max, m_aiTraj);
        m_iEndTimeIdx = m_msg.time_ms - 1;
    }

    Error_t _setPosition(int position) {
        m_iPosition = position;
        return _updateBoundaries();
    }

    void threadPoolHandler() {
        while (m_bRunning) {
            Message_t msg{};
            tp now, msgTime;
            bool success;
            {
                std::unique_lock<std::mutex> lk(m_mtx);
                m_cmdCv.wait(lk, [this] { return !m_cmdManager.isEmpty() || !m_bRunning; });
                if (!m_bRunning) break;
                success = m_cmdManager.pop(Port::Arm::IAI, msg);
            }

            if (success) {
                std::this_thread::sleep_until(msg.msgTime);
                Error_t e = move(msg);
                if (e != kNoError) {
                    if (e != kAlreadyThereError)
                        LOG_ERROR("Move Error Code: {}", e);
                }
            }
        }
    }
};


#endif //SHIMONCONTROLLER_ARM_H
