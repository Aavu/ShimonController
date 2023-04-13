//
// Created by Raghavasimhan Sankaranarayanan on 4/9/22.
//

#ifndef SHIMONCONTROLLER_ARM_H
#define SHIMONCONTROLLER_ARM_H

#include <iostream>
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
        uint8_t strikerId{};
        int time_ms{};
        _tp arrivalTime;
        _tp msgTime;

        void pprint() const {
            std::bitset<8> sId(strikerId);
            std::cout << "Id: " << arm_id
                      << "\t target: " << target
                      << "\t acc: " << acceleration
                      << "\t vmax: " << v_max
                      << "\t midi vel: " << midiVelocity
                      << "\t strikerId: " << sId
                      << "\t time (ms): " << time_ms
                      << std::endl;
        }
    };

    struct Boundary {
        int left=0;
        int right=0;
    };

    Arm(int id, int iHomePosition, int w, int b) : m_id(id),
                                                kHomePosition(iHomePosition),
                                                m_controller(IAIController::getInstance()),
                                                m_iW(w), m_iB(b)
    {
        reset();
    }

    Error_t reset() {
        Error_t e;

        Message_t msg{};
        msg.arm_id = m_id;
        msg.target = kHomePosition;
        msg.msgTime = steady_clock::now();
        msg.arrivalTime = steady_clock::now() + milliseconds(DLY);
        e = setMessage(msg);
        if (e != kNoError) {
            LOG_ERROR("Error Code: {}", e);
            return e;
        }

        e = _setPosition(kHomePosition);
        if (e != kNoError) {
            LOG_ERROR("Error Code: {}", e);
            return e;
        }

        m_lastMsg = {m_id, kHomePosition, 0, 0, 0, 0, 0, 0, steady_clock::now(), steady_clock::now()};

        LOG_DEBUG("Id: {} \t position: {} \t lb: {} \t rb: {}", m_id, m_iPosition, m_boundary.left, m_boundary.right);
        return kNoError;
    }

//    Error_t setTarget(int position) {
//        m_iTarget = position;
//        _updateTargetBoundaries();
//        return kNoError;
//    }

    Error_t setMessage(const Message_t& msg) {
        setLastMsg(m_msg);
        m_msg = msg;
        _updateTargetBoundaries();
        _updateTrajectory();
        return kNoError;
    }

    void setLastMsg(const Message_t& msg) {
        m_lastMsg = msg;
    }

    [[nodiscard]] int getPosition() const { return m_iPosition; }
    [[nodiscard]] int getTarget() const { return m_msg.target; }
    [[nodiscard]] int getLastTarget() const { return m_lastMsg.target; }
    /*
     * This function should be called periodically from the Arm Controller
     */
    Error_t updateStatusFromDevice() {
        Error_t e;
        int pos = m_msg.target;
#ifndef SIMULATE
        int timeout_ms = 100;
        e = m_controller.updateStats(m_id, timeout_ms);
        if (e == kNotInitializedError || e == kTimeoutError) return _setPosition(pos); // Ignore timeout
        else { ERROR_CHECK(e, e); }
        pos = m_controller.getPosition(m_id);
        pos = (int) round(((pos * 0.01) - m_iB) / m_iW);

        if (std::abs(pos - m_msg.target) > FOLLOW_ERROR_THRESHOLD) {
            LOG_WARN("Follow Error. Arm: {} \t target: {} \t pos: {}", m_id, m_msg.target, pos);
        }
#endif
        return _setPosition(pos);
    }

    [[nodiscard]] bool isMoving() const {
        return m_bIsMoving;
    }

    void setMoving(bool bMoving) {
        std::lock_guard<std::mutex> lk(m_mtx);
        m_bIsMoving = bMoving;
        m_iTrajIdx = 0;
        m_cv.notify_all();
    }

    void updatePosition(int iPos) {
        if (!m_bIsMoving) return;
//        LOG_INFO("Arm {} traj pos: {}", m_id, iPos);
        _setPosition(iPos);
    }

    void updatePositionFromTrajectory(int interval_ms=1, int endTime_ms=DLY) {
        updatePosition(m_aiTraj[m_iTrajIdx]);
//        updatePosition(m_iTarget);
        m_iTrajIdx = std::min(endTime_ms, m_iTrajIdx + interval_ms);
        if (m_iTrajIdx == endTime_ms - 1 && m_bIsMoving) {
//            if (m_id == 1) LOG_INFO("End Time: {}", endTime);
            setMoving(false);
            _setPosition(m_msg.target);
        }
    }

    [[nodiscard]] int getID() const { return m_id; }

    [[nodiscard]] int getLeftBoundary() const {
        return std::min(m_boundary.left, std::min(m_tBoundary.left, getInstantaneousLeftBoundary()));
    }

    [[nodiscard]] int getRightBoundary() const {
        return std::max(m_boundary.right, std::max(m_tBoundary.right, getInstantaneousRightBoundary()));
    }

    [[nodiscard]] int getInstantaneousLeftBoundary() const {
        int pos = m_bIsMoving ? m_aiTraj[std::min(m_iTrajIdx, DLY - 1)] : m_iPosition;
        return pos - kBoundaries[m_id][0];
    }

    [[nodiscard]] int getInstantaneousRightBoundary() const {
        int pos = m_bIsMoving ? m_aiTraj[std::min(m_iTrajIdx, DLY - 1)] : m_iPosition;
        return pos + kBoundaries[m_id][1];
    }

    [[nodiscard]] int getLeftBoundaryFor(int iTarget) const { return iTarget - kBoundaries[m_id][0]; }
    [[nodiscard]] int getRightBoundaryFor(int iTarget) const { return iTarget + kBoundaries[m_id][1]; }

    [[nodiscard]] int getTargetInSliderCoordinate(int pos) const { return pos * m_iW + m_iB; }

    void setMsgTime(_tp time) {
        m_msg.msgTime = std::max(m_lastMsg.arrivalTime, time);
        m_msg.arrivalTime = time + milliseconds (DLY);
    }

    [[nodiscard]] const _tp& getMsgTime() const { return m_msg.msgTime; }
    [[nodiscard]] const _tp& getArrivalTime() const { return m_msg.arrivalTime; }

    [[nodiscard]] const _tp& getLastArrivalTime() const { return m_lastMsg.arrivalTime; }

    [[nodiscard]] int getTime_ms() const { return m_msg.time_ms; }
    /*
 * Computes the v max and acceleration for the arm. Returns kNoError if the motion is possible.
 */
    Error_t computeMotionParam(Message_t& msg) const {
        // When computing, the target is not set. So m_msg is the last msg
        auto dPos = std::abs(msg.target - m_msg.target);
        if (dPos == 0) {
            msg.acceleration = 0;
            msg.v_max = 0;
            return kNoError;    // No need to send kAlreadyThereError as IAI controller will take care of it.
        }

        auto fPos = (float)dPos / 1000.f;

        auto fTime = (float)msg.time_ms / 1000.f;

        auto temp = std::min(VELOCITY_LIMIT, 2.f*fPos/fTime);
        msg.v_max = 1000.f * (float)temp;

        msg.acceleration = std::pow(temp, 2.f) / (fTime * temp - fPos) / 9.8f;
        if (msg.acceleration > ACC_LIMIT || msg.acceleration < 0) return kArgLimitError;

        return kNoError;
    }

    void move(Message_t& msg, float fallbackAcc ,float fallbackVel) {
        Error_t e;

        e = computeMotionParam(msg);
        if (e != kNoError) {
            LOG_ERROR("Cannot use delay of {}. Error code: {}. Fallback to {}", msg.time_ms, (int)e, DLY);
            msg.v_max = fallbackVel;
            msg.acceleration = fallbackAcc;
        }
        e = setTargetAndMove(msg);
        if (e != kNoError) {
            LOG_ERROR("Error moving arm {} with vel {} and acc {}", m_id, msg.acceleration, msg.v_max);
        }
    }

    Error_t move(int position, float time_ms) {
        Error_t e;

        Message_t msg{.target = position, .time_ms = static_cast<int>(time_ms)};
        e = computeMotionParam(msg);
        if (e != kNoError) {
            LOG_DEBUG("Cannot use delay of {} as it uses velocity = {} and Acc = {}. Error code: {}", time_ms, msg.v_max, msg.acceleration, (int)e);
            return e;
        }
        setTargetAndMove(msg);
        return kNoError;
    }

    Error_t setTargetAndMove(const Message_t& msg) {
        setMessage(msg);
        return move();
    }

    Error_t move() {
        int scPosition = getTargetInSliderCoordinate(m_msg.target);
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
                setMessage(m_lastMsg);
                LOG_ERROR("Set Position Error: {}", e);
                return e;
            }
        }

//        LOG_DEBUG("Arm: {} \t target: {} \t acc: {} \t v_max: {}", m_id, getTarget(), m_msg.acceleration, (int)std::round(m_msg.v_max));
//        LOG_DEBUG("Arm: {} \t lm: {} \t rm: {}", m_id, getLeftBoundary(), getRightBoundary());
        setMoving(true);

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

private:
    int m_id = -1;
    int m_iPosition = -1;
    const int kHomePosition;
    std::atomic<bool> m_bIsMoving = false;
    IAIController& m_controller;
    Boundary m_boundary{};
    Boundary m_tBoundary{};
    // Multiplier for slider geometry conversion
    const int m_iW = 1;
    // Bias for slider geometry conversion
    const int m_iB = 0;

    Message_t m_msg;
    Message_t m_lastMsg;

    std::mutex m_mtx;
    std::condition_variable m_cv;

    int m_aiTraj[DLY];
    int m_iTrajIdx = 0;
    bool m_bServoOn = false;

    Error_t _updateBoundaries() {
        m_boundary.left = getInstantaneousLeftBoundary();
        m_boundary.right = getInstantaneousRightBoundary();
//        LOG_DEBUG("Arm {} Boundary : {}, {}", m_id, m_boundary.left, m_boundary.right);
        return kNoError;
    }

    Error_t _updateTargetBoundaries() {
        auto temp0 = m_msg.target - kBoundaries[m_id][0];
        auto temp1 = m_msg.target + kBoundaries[m_id][1];
        m_tBoundary.left = temp0;
        m_tBoundary.right = temp1;
//        LOG_DEBUG("Arm {} Target Boundary updated : {}, {}", m_id, m_tBoundary.left, m_tBoundary.right);
        return kNoError;
    }

    void _updateTrajectory() {
        SliderModel::compute(m_msg.time_ms, m_lastMsg.target, m_msg.target, m_msg.acceleration, m_msg.v_max, m_aiTraj);
    }

    Error_t _setPosition(int position) {
        m_iPosition = position;
//        LOG_DEBUG("Arm {} Position: {}", m_id, m_iPosition);
        return _updateBoundaries();
    }
};


#endif //SHIMONCONTROLLER_ARM_H
