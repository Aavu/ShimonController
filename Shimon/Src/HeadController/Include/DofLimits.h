//
// Created by Raghavasimhan Sankaranarayanan on 4/12/22.
//

#ifndef SHIMONCONTROLLER_DOFLIMITS_H
#define SHIMONCONTROLLER_DOFLIMITS_H

#include <iostream>
#include <unordered_map>
#include "HeadCommands.h"

namespace Dof {
    struct Limit {
        float lower = 0;
        float higher = 0;
        float max_velocity = 0;
        Limit() = default;
        Limit(float low, float high) : lower(low), higher(high) {}
        Limit(float low, float high, float max_velocity) : lower(low), higher(high), max_velocity(max_velocity) {}
    };

    static inline std::unordered_map<Command, Limit> limits = {
            {Command::BasePan, Limit(-1.25, 1.25, 5)},
            {Command::Neck, Limit(-.5, 0.9, 15)},
            {Command::NeckPan, Limit(-1, 1, 25)},
            {Command::HeadTilt, Limit(-1.2, 0.4, 20)},
            {Command::Mouth, Limit(690, 1000)},
            {Command::Eyebrow, Limit(2600, 2150)}
    };
}

#endif //SHIMONCONTROLLER_DOFLIMITS_H
