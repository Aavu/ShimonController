//
// Created by Raghavasimhan Sankaranarayanan on 4/13/22.
//

#ifndef SHIMONCONTROLLER_MOTOR_H
#define SHIMONCONTROLLER_MOTOR_H

#include <iostream>
#include <unordered_map>
#include "Util.h"
#include "Config.h"
#include "ErrorDef.h"

class Motor {
public:
    Motor() = default;
    Motor(const Motor&) = delete;

    virtual Error_t init(MotorConfig_t&& mc) {
        set(std::move(mc));
        return kNoError;
    }

    virtual Error_t reset() {
        m_bInitialized = false;
        return kNoError;
    }

    void set(MotorConfig_t&& info) {
        m_config = std::move(info);
    }

    void setName(const std::string& name) { m_config.name = HeadCommandUtil::getCommand(name); }
    void setName(HeadCommand name) { m_config.name = name; }
    [[nodiscard]] std::string getName() const { return HeadCommandUtil::getCommand(m_config.name); }

    void setAxis(int axis) { m_config.axis = axis; }
    [[maybe_unused]] [[nodiscard]] int getAxis() const { return m_config.axis; }

    void setHomingType(MotorConfig_t::HomingType type) { m_config.homing = type; }
    [[nodiscard]] MotorConfig_t::HomingType getHomingType() const { return m_config.homing; }

    void setVelocity(float fVelocity) { m_config.defaultVelocity = fVelocity; }
    void setAcceleration(float fAcc) { m_config.defaultAcceleration = fAcc; }

    Error_t set(std::optional<float> fVel, std::optional<float> fAcc) {
        if (fVel) setVelocity(*fVel);
        if (fAcc) setAcceleration(*fAcc);
        return kNoError;
    }

    virtual Error_t servoOn(bool turnOn) {
        return kNoError;
    }

    virtual Error_t home() {
        return kNoError;
    }

    virtual bool isHomed() {
        return true;
    }

    virtual Error_t zero() {
        return kNoError;
    }

    virtual Error_t goTo(float pos, std::optional<float> vel, std::optional<float> acc) {
        return kNoError;
    }

    virtual Error_t disable() {
        return kNoError;
    }
    virtual Error_t enable() {
        return kNoError;
    }

protected:
    MotorConfig_t m_config;
    bool m_bInitialized = false;
    bool m_bServoOn = false;
};

#endif //SHIMONCONTROLLER_MOTOR_H
