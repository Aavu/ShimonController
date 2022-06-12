//
// Created by Raghavasimhan Sankaranarayanan on 4/9/22.
//

#ifndef SHIMONCONTROLLER_ARM_H
#define SHIMONCONTROLLER_ARM_H

#include <iostream>
#include "IAIController.h"
#include "Trajectory.h"
#include "Def.h"
#include "Logger.h"

class Arm {
public:
    typedef std::chrono::steady_clock::time_point _tp;

    struct Boundary {
        int left=0;
        int right=0;
    };

    Arm() : m_controller(IAIController::getInstance()) {

    }

    Arm(int id, int iHomePosition, int w, int b) : m_id(id),
                                                m_iHomePosition(iHomePosition),
                                                m_controller(IAIController::getInstance()),
                                                m_iW(w), m_iB(b)
    {
        Error_t e;

        e = setTarget(iHomePosition);
        if (e != kNoError) {
            LOG_ERROR("Error Code: {}", e);
            return;
        }

        e = _setPosition(iHomePosition);
        if (e != kNoError) {
            LOG_ERROR("Error Code: {}", e);
            return;
        }

        LOG_DEBUG("Id: {} \t position: {} \t lb: {} \t rb: {}", m_id, m_iPosition, m_boundary.left, m_boundary.right);
    }

    Error_t setTarget(int position) {
        m_iTarget = position;
        LOG_DEBUG("Arm {},  set target: {}", m_id, m_iTarget);
//        m_msgTime = std::chrono::steady_clock::now();
//        m_arrivalTime = m_msgTime + std::chrono::milliseconds(DLY);
//        _setPosition(position);
        _updateTargetBoundaries();
        return kNoError;
    }

    void reset() {
        setTarget(m_iHomePosition);
    }

    [[nodiscard]] int getPosition() const { return m_iPosition; }

    /*
     * This function should be called periodically from the Arm Controller
     */
    Error_t updateStatusFromDevice() {
        Error_t e;
        int pos = m_iTarget;
//#ifndef SIMULATE
//        e = m_controller.updateStats(m_id);
//        if (e == kNotInitializedError || e == kTimeoutError) return _setPosition(pos); // Ignore timeout
//        else {ERROR_CHECK(e, e); }
//        pos = m_controller.getPosition(m_id);
//        pos = (int) round(pos * 0.01);
//        pos = (pos - m_iB) / m_iW;
//
//        if (pos != m_iTarget) {
//            LOG_INFO("Arm: {} \t target: {} \t pos: {}", m_id, m_iTarget, pos);
//        }
//#endif
        return _setPosition(pos);
    }

    bool isMoving() const {
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
        updatePosition((int)round(m_afTraj[m_iTrajIdx]));
//        updatePosition(m_iTarget);
        m_iTrajIdx += interval_ms;

        auto endTime = std::chrono::duration_cast<std::chrono::milliseconds>(m_arrivalTime - m_msgTime).count();
        if (m_iTrajIdx == endTime - 1 && m_bIsMoving) {
//            if (m_id == 1) LOG_INFO("End Time: {}", endTime);
            setMoving(false);
            updatePosition(m_iTarget);
        }
    }

    void updateTrajectory(int time_ms, int target, float acc, float v_max) {
        Trajectory::compute(time_ms, m_iPosition, target, acc, v_max, m_afTraj);
    }

    [[nodiscard]] int getID() const { return m_id; }

    [[nodiscard]] int getLeftBoundary() const { return m_boundary.left; }
    [[nodiscard]] int getRightBoundary() const { return m_boundary.right; }

    [[nodiscard]] int getTargetReachedLeftBoundary() const { return m_tBoundary.left; }
    [[nodiscard]] int getTargetReachedRightBoundary() const { return m_tBoundary.right; }

    [[nodiscard]] int getTargetInSliderCoordinate(int pos) const { return pos * m_iW + m_iB; }

    void setMsgTime(_tp time) {
        m_msgTime = std::max(m_arrivalTime, time);
        m_arrivalTime = time + std::chrono::milliseconds (DLY);
    }

    void setArrivalTime(_tp time) {
        m_arrivalTime = time;
    }

    [[nodiscard]] const _tp& getMsgTime() const { return m_msgTime; }
    const _tp& getArrivalTime() const {
        return m_arrivalTime;
    }

    /*
 * Computes the v max and acceleration for the arm. Returns kNoError is the motion is possible.
 */
    Error_t computeMotionParam(int position, float time_ms, float* v_max, float* acceleration) const {
        if (!v_max || !acceleration) return kInvalidArgsError;

        auto dPos = std::abs(position - m_iTarget);
//        LOG_INFO("Arm {} - {} {}", m_id, position, m_iTarget);
        auto fPos = (float)dPos / 1000.f;

        auto fTime = time_ms / 1000.0;

        auto temp = std::min(2.5, 2.*float(dPos)/time_ms);
        *v_max = 1000.f * (float)temp;

        auto acc = std::pow(temp, 2) / (fTime * temp - fPos) / 9.8f;

        *acceleration = (float)acc;
        if (acc > ACC_THRESHOLD || acc < 0) return kArgLimitError;

        return kNoError;
    }

    void move(int position, float time_ms, float fallbackAcc ,float fallbackVel) {
        float v_max, acc;
        Error_t e;
        e = computeMotionParam(position, time_ms, &v_max, &acc);
        if (e != kNoError) {
            LOG_ERROR("Cannot use delay of {}. Error code: {}. Fallback to {}", time_ms, (int)e, DLY);
            v_max = fallbackVel;
            acc = fallbackAcc;
        }
        e = move(position, acc, v_max);
        if (e != kNoError) {
            LOG_ERROR("Error moving arm {} with vel {} and acc {}", m_id, acc, v_max);
        }
    }

    Error_t move(int position, float time_ms) {
        float v_max, acc;
        Error_t e;
        e = computeMotionParam(position, time_ms, &v_max, &acc);
        if (e != kNoError) {
            LOG_DEBUG("Cannot use delay of {} as it uses velocity = {} and Acc = {}. Error code: {}", time_ms, v_max, acc, (int)e);
            return e;
        }
        move(position, acc, v_max);
        return kNoError;
    }

    Error_t move(int position, float acceleration, float v_max) {
        if (m_iTarget == position) return kAlreadyThereError;
        int oldPos = m_iTarget;
        setTarget(position);
        int scPosition = getTargetInSliderCoordinate(position);
        Modbus::Message_t mbMsg {
                .armID = m_id,
                .position = scPosition,
                .precision = 0.1,
                .v_max = (int)std::round(v_max),
                .acceleration = acceleration,
                .push = 0
        };

        {
//            std::unique_lock<std::mutex> lk(m_mtx);
//            m_cv.wait(lk, [this] { return !isMoving(); });
            Error_t e = m_controller.setPosition(mbMsg);
            if (e != kNoError) {
                setTarget(oldPos);
                LOG_ERROR("Set Position Error: {}", e);
                return e;
            }
        }

        LOG_INFO("ArmID: {} \t Position: {} \t acc: {} \t v_max: {}", m_id, position, acceleration, (int)std::round(v_max));
        setMoving(true);
        return _setPosition(position);
    }

    Error_t servoOn(bool bTurnOn = true) {
        if (m_bServoOn == bTurnOn) return kNoError;

        Error_t e;
        e = m_controller.setServo(m_id, true);
        ERROR_CHECK(e, e);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        m_bServoOn = bTurnOn;
        return kNoError;
    }

private:
    int m_id = -1;
    int m_iTarget = -1;
    int m_iPosition = -1;
    int m_iHomePosition = -1;
    std::atomic<bool> m_bIsMoving = false;
    IAIController& m_controller;
    Boundary m_boundary{};
    Boundary m_tBoundary{};
    // Multiplier for slider geometry conversion
    int m_iW = 1;
    // Bias for slider geometry conversion
    int m_iB = 0;

    std::mutex m_mtx;
    std::condition_variable m_cv;

    float m_afTraj[DLY];
    int m_iTrajIdx = 0;
    bool m_bServoOn = false;
    _tp m_msgTime = std::chrono::steady_clock::now();
    _tp m_arrivalTime = std::chrono::steady_clock::now();

    Error_t _updateBoundaries() {
        auto temp0 = m_iPosition - kBoundaries[m_id][0];
        auto temp1 = m_iPosition + kBoundaries[m_id][1];
//        if (temp0 != m_boundary.Left || temp1 != m_boundary.Right)
//            LOG_INFO("Arm {} Boundaries: {}, {}", m_id, m_boundary.Left, m_boundary.Right);
        m_boundary.left = temp0;
        m_boundary.right = temp1;
        LOG_TRACE("Arm {} Boundary updated : {}, {}", m_id, m_boundary.left, m_boundary.right);
        return kNoError;
    }

    Error_t _updateTargetBoundaries() {
        auto temp0 = m_iTarget - kBoundaries[m_id][0];
        auto temp1 = m_iTarget + kBoundaries[m_id][1];
        m_tBoundary.left = temp0;
        m_tBoundary.right = temp1;
        LOG_TRACE("Arm {} Target Boundary updated : {}, {}", m_id, m_tBoundary.left, m_tBoundary.right);
        return kNoError;
    }

    Error_t _setPosition(int position) {
        m_iPosition = position;
        LOG_TRACE("Arm {} Position: {}", m_id, m_iPosition);
        return _updateBoundaries();
    }
};


#endif //SHIMONCONTROLLER_ARM_H
