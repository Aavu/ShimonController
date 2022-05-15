//
// Created by Raghavasimhan Sankaranarayanan on 4/11/22.
//

#ifndef SHIMONCONTROLLER_TRAJECTORY_H
#define SHIMONCONTROLLER_TRAJECTORY_H

#include <iostream>
#include <cmath>
#include <cstdint>
#include <cstdlib>

class Trajectory {
public:
    static void compute(int time_ms, float s0, float sf, float acc, float v_max, float* s) {
        if (!s) return;
        float dec = -acc;
        float dir = (s0 < sf) ? 1.f : -1.f;
        float t = 0.001;
        float prev_v = 0;
        int decIdx = time_ms;

        s[0] = s0;
        for (int n=1; n< time_ms; ++n) {
            if (n > decIdx) {
                acc = dec;
            }

            float v = abs(acc*t + prev_v);
            if (v > v_max) {
                v = v_max;
                acc = 0;
                decIdx = time_ms - n;
            }

            s[n] = s[n-1] + dir*prev_v*t + 0.5*dir*abs(acc)*t*t;
            prev_v = v;
        }
        s[time_ms - 1] = sf;
    }
};

#endif //SHIMONCONTROLLER_TRAJECTORY_H
