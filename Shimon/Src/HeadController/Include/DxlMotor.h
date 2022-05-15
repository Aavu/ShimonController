//
// Created by Raghavasimhan Sankaranarayanan on 4/27/22.
//

#ifndef SHIMONCONTROLLER_DXLMOTOR_H
#define SHIMONCONTROLLER_DXLMOTOR_H

#include <iostream>
#include "Def.h"
#include "DxlController.h"
#include "Motor.h"
#include "ErrorDef.h"

class DxlMotor : public Motor {
public:
    DxlMotor() : m_pController(&DxlController::getInstance()) {}

    Error_t init(MotorConfig_t&& mc) final {
        Motor::init(std::move(mc));
        return m_pController->init(DXL_PORT);
    }

    Error_t servoOn(bool turnOn) override {
        if (turnOn) return m_pController->servoOn(m_config.axis);
        return m_pController->servoOff(m_config.axis);
    }

    Error_t zero() override {
        return goTo(0, std::optional<float>(), std::optional<float>());
    }

    Error_t goTo(float pos, std::optional<float> vel, std::optional<float> acc) override {
        int iVel;
        if (vel) iVel = (*vel) * m_config.scale;

        auto fAcc = acc.value_or(m_config.defaultAcceleration);
        pos = std::min(std::max(pos, m_config.minPosition), m_config.maxPosition);
        int encPos = (int)(pos * m_config.scale) + m_config.zeroEncoder;

//        std::cout << getName() << " " << " " << encPos << " " << fVel << std::endl;

        if (acc) {
            return m_pController->setTarget(m_config.axis, encPos, (int)((*vel) * m_config.scale), (int)((*acc) * m_config.scale), (int)((*acc) * m_config.scale));
        } else if (vel) {
            return m_pController->setTarget(m_config.axis, encPos, (int)((*vel) * m_config.scale));
        }

        return m_pController->setTarget(m_config.axis, encPos);
    }

    Error_t disable() override {
        return m_pController->servoOff(m_config.axis);
    }

    Error_t enable() override {
        return m_pController->servoOn(m_config.axis);
    }

private:
    DxlController* m_pController;
};
#endif //SHIMONCONTROLLER_DXLMOTOR_H
