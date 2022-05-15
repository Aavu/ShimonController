//
// Created by Raghavasimhan Sankaranarayanan on 4/13/22.
//

#ifndef SHIMONCONTROLLER_HDMOTOR_H
#define SHIMONCONTROLLER_HDMOTOR_H

#include "CopleyASCIIController.h"
#include "Motor.h"
#include "ErrorDef.h"
#include <climits>

class HDMotor: public Motor {
public:
    HDMotor(): m_pController(&CopleyASCIIController::getInstance()) {
    }

    Error_t init(MotorConfig_t&& mc) final {
        Error_t e;
        e = Motor::init(std::move(mc));
        ERROR_CHECK(e, e);

        e = m_pController->init(COPLEY_PORT, COPLEY_BAUDRATE);
        ERROR_CHECK(e, e);

        m_bInitialized = true;
        return kNoError;
    }

    Error_t reset() final {
        if (!m_bInitialized) return kNoError;
        Error_t e;
        if (m_config.name == HeadCommand::Neck) {
            e = goTo(m_config.minPosition, std::optional<float>(), std::optional<float>());
            ERROR_CHECK(e, e);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        } else {
            e = zero();
            ERROR_CHECK(e, e);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        e = servoOn(false);
        ERROR_CHECK(e, e);
        m_bInitialized = false;
        return kNoError;
    }

    Error_t home() override {
        switch (m_config.homing) {
            case MotorConfig_t::HomingType::HardStop:
                return m_pController->homeHardStop(m_config.axis);
            case MotorConfig_t::HomingType::Manual:
                return m_pController->homeManual(m_config.axis);
        }
        return kUnknownCaseError;
    }

    Error_t servoOn(bool turnOn) override {
        if (m_bServoOn == turnOn) return kNoError;
        if (!m_bInitialized) return kNotInitializedError;
        Error_t e;
        if (turnOn) {
            e = m_pController->servoOn(m_config.axis);
            ERROR_CHECK(e, e);
            m_bServoOn = turnOn;
            return kNoError;
        }

        e = m_pController->servoOff(m_config.axis);
        ERROR_CHECK(e, e);
        m_bServoOn = turnOn;
        return kNoError;
    }

    Error_t zero() override {
        return goTo(0, std::optional<float>(),std::optional<float>());
    }

    Error_t goTo(float pos, std::optional<float> vel, std::optional<float> acc) override {
        auto fVel = vel.value_or(m_config.maxVelocity);
        auto fAcc = acc.value_or(m_config.maxAcceleration);
        pos = std::min(std::max(pos, m_config.minPosition), m_config.maxPosition);

//        auto scale = m_config.scale * (m_config.homing == MotorConfig_t::HomingType::NegHardStop ? -1. : 1.);
//        std::cout << "Axis: " << getAxis() << "\tScale: " << scale << std::endl;
        int encPos = (int)(pos * m_config.scale) + m_config.zeroEncoder;

//        std::cout << getName() << " " << " " << pos << " " << (int)(fVel * m_config.scale) << std::endl;
        return m_pController->setTarget(m_config.axis, encPos, (int)(fVel * m_config.scale), (int)(fAcc * m_config.scale), (int)(fAcc * m_config.scale));
    }

    Error_t disable() override {
        return m_pController->servoOff(m_config.axis);
    }

    Error_t enable() override {
        return m_pController->servoOn(m_config.axis);
    }

    bool isHomed() override {
        return m_pController->queryHomeEnd(m_config.axis) == 1;
    }

private:
    CopleyASCIIController* m_pController;
};

#endif //SHIMONCONTROLLER_HDMOTOR_H
