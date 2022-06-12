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
    static void compute(int time_ms, double s0, double sf, double acc, double v_max, float* s) {
        if (!s) return;
        acc = acc*9.8;
        v_max = v_max/1000.0;
        s0 = s0/1000.0;
        sf = sf/1000.0;
        double dec = -acc;
        double dir = (s0 < sf) ? 1.f : -1.f;
        double t = 0.001;
        double prev_v = 0;
        int decIdx = time_ms;

        s[0] = (float)s0;
        for (int n=1; n< time_ms; ++n) {
            if (n > decIdx) acc = dec;

            double v = abs(acc*t + prev_v);
            if (v > v_max) {
                v = v_max;
                acc = 0;
                decIdx = time_ms - n;
            }

            s[n] = s[n-1] + dir*prev_v*t + 0.5*dir*abs(acc)*t*t;
            prev_v = v;
            s[n-1] *= 1000.f;
        }
        s[time_ms - 1] = (float)sf * 1000.f;
    }

    static void compute(int time_ms, int s0, int sf, float acc, float v_max, float* s) {
        compute(time_ms, (double)s0, (double)sf, (double)acc, (double)v_max, s);
    }
};

#endif //SHIMONCONTROLLER_TRAJECTORY_H
