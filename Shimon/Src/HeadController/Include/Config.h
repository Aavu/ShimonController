//
// Created by Raghavasimhan Sankaranarayanan on 4/22/22.
//

#ifndef SHIMONCONTROLLER_CONFIG_H
#define SHIMONCONTROLLER_CONFIG_H

#include "HeadCommands.h"

struct MotorConfig_t {
    enum class HomingType {
        Manual,
        HardStop
    };

    enum class MotorType {
        HD,
        Dxl
    };

    HeadCommand name;
    int axis;
    int zeroEncoder;
    float minPosition;
    float maxPosition;
    float scale;
    float maxVelocity;
    float maxAcceleration;
    float defaultPosition;
    float defaultVelocity;
    float defaultAcceleration = 0;
    HomingType homing;
    MotorType motorType;

//    MotorConfig_t() = default;
//    MotorConfig_t(MotorConfig_t&&) = default;
//
//    MotorConfig_t& operator=(MotorConfig_t&& other) = default;
};

#endif //SHIMONCONTROLLER_CONFIG_H
