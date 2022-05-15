//
// Created by Raghavasimhan Sankaranarayanan on 4/9/22.
//

#ifndef SHIMONCONTROLLER_ARM_H
#define SHIMONCONTROLLER_ARM_H

#include <iostream>
#include "IAIController.h"
#include "Def.h"
#include "Logger.h"

class Arm {
public:
    typedef std::chrono::steady_clock::time_point _tp;

    struct Boundary {
        int Left=0;
        int Right=0;
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

        e = updateStatusFromDevice();
        if (e != kNoError) {
            LOG_ERROR("Error Code: {}", e);
            return;
        }
        LOG_INFO("Id: {} \t position: {} \t lb: {} \t rb: {}", m_id, m_iPosition, m_boundary.Left, m_boundary.Right);
    }

    Error_t setTarget(int position) {
        m_iTarget = position;
        m_lastTime = std::chrono::steady_clock::now();
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
#ifndef SIMULATE
        e = m_controller.updateStats(m_id);
        ERROR_CHECK(e, e);
        pos = m_controller.getPosition(m_id);
        if (pos < 0) return kGetValueError;

        pos = (int) round(pos / 0.01);
        pos = (pos - m_iB) * m_iW;
#endif

        return _setPosition(pos);
    }

    [[nodiscard]] int getID() const { return m_id; }

    [[nodiscard]] int getLeftBoundary() const { return m_boundary.Left; }
    [[nodiscard]] int getRightBoundary() const { return m_boundary.Right; }

    [[nodiscard]] int getTargetInSliderCoordinate(int pos) const { return pos * m_iW + m_iB; }

    void setLastTime(_tp lastTime) { m_lastTime = lastTime; }
    [[nodiscard]] const _tp& getLastTime() const { return m_lastTime; }

    /*
 * Computes the v max and acceleration for the arm. Returns kNoError is the motion is possible.
 */
    Error_t computeMotionParam(int position, float time_ms, float* v_max, float* acceleration) const {
        if (!v_max || !acceleration) return kInvalidArgsError;


        auto dPos = std::abs(position - m_iPosition);
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
        move(position, acc, v_max);
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
        int scPosition = getTargetInSliderCoordinate(position);
        Modbus::Message_t mbMsg {
                .armID = m_id,
                .position = scPosition,
                .precision = 0.1,
                .v_max = (int)std::round(v_max),
                .acceleration = acceleration,
                .push = 0
        };

//    LOG_INFO("ArmID: {} \t Position: {} \t acc: {} \t v_max: {}", armId, position, acceleration, (int)std::round(v_max));

        Error_t e = m_controller.write(mbMsg);
        ERROR_CHECK(e, e);

        return setTarget(position);
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
    IAIController& m_controller;
    Boundary m_boundary{};
    // Multiplier for slider geometry conversion
    int m_iW = 1;
    // Bias for slider geometry conversion
    int m_iB = 0;

    bool m_bServoOn = false;
    _tp m_lastTime;

    Error_t _updateBoundaries() {
        m_boundary.Left = m_iPosition - kBoundaries[m_id][0];
        m_boundary.Right = m_iPosition + kBoundaries[m_id][1];
        return kNoError;
    }

    Error_t _setPosition(int position) {
        m_iPosition = position;
        return _updateBoundaries();
    }
};


#endif //SHIMONCONTROLLER_ARM_H
