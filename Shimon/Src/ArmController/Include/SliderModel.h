//
// Created by Raghavasimhan Sankaranarayanan on 4/11/22.
//

#ifndef SHIMONCONTROLLER_SLIDERMODEL_H
#define SHIMONCONTROLLER_SLIDERMODEL_H

#include <iostream>
#include <cmath>
#include <cstdint>
#include <cstdlib>

class SliderModel {
public:
    static void compute(int time_ms, double s0, double sf, double acc, double v_max, float* s) {
        if (!s) return;
        acc = acc*9.8;
        v_max = v_max/1000.0;
        s0 = s0/1000.0;
        sf = sf/1000.0;
        double dec = -acc;
        double dir = (s0 < sf) ? 1. : -1.;
        double t = 0.001;
        double prev_v = 0;
        int decIdx = time_ms;

        s[0] = (float)s0;
        for (int n=1; n< time_ms; ++n) {
            if (n > decIdx) acc = dec;

            double v = std::abs(acc*t + prev_v);
            if (v > v_max) {
                v = v_max;
                acc = 0;
                decIdx = time_ms - n;
            }

            s[n] = s[n-1] + dir*prev_v*t + 0.5*dir*std::abs(acc)*t*t;
            prev_v = v;
            s[n-1] *= 1000.f;
        }
        s[time_ms - 1] = (float)sf * 1000.f;
    }

    static void compute(int time_ms, int s0, int sf, double acc, double v_max, int* s) {
        if (!s) return;
//        compute(time_ms, (double)s0, (double)sf, (double)acc, (double)v_max, s);
        for (int n=0; n< time_ms; ++n) {
            s[n] = compute(n, s0, sf, acc, v_max);
        }
    }

    // All arguments are assumed to be in SI units (sec, m, m, m/s^2, m/s)
    static double compute(double t, double Xo, double Xf, double acc, double v_max) {
        double eps = 5e-5;

        double deltaX = (Xf - Xo);
        double direction = deltaX >= 0 ? 1. : -1.;
//        expr min(2.5\, min($f3\, sqrt(abs($f1* $f2))))
        acc = direction * acc;
        v_max = direction * std::min(2.5, std::min(v_max, sqrt(abs(deltaX * acc))));

        double t1 = v_max / acc;
        double t2 = eps + deltaX / v_max;   // eps is added to make sure t2 > t1
// expr (t <= t1) * (Xo + 0.*t + 0.5 * a * pow(t, 2)) + ((t1 < t) && (t<= t2)) * (Xo + 0.5 * a * pow(t1, 2) + (t - t1) * v_max) + (t > t2 && (t < (t2+t1))) * (Xf +(.5 * - a*pow(t1+t2-t, 2))) + (t >= (t2+t1)) * Xf

        if (t <= t1) return Xo + (0. * t) + (0.5 * acc * pow(t, 2));                          // t <= t1
        else if ((t > t1) && (t<= t2)) return Xo + 0.5 * acc * pow(t1, 2) + (t - t1) * v_max; // t1 < t <= t2
        else if (t > t2 && (t < t2+t1)) return Xf +(0.5 * - (acc * pow(t1+t2-t, 2)));         // t2 < t < t1 + t2

        return Xf;                                                                                          // t >= t1 + t2
    }

    static int compute(int time_ms, int Xo_mm, int Xf_mm, double acc_g, double v_max_mm_s) {
        double y = compute(time_ms / 1000., Xo_mm / 1000., Xf_mm / 1000., acc_g * 9.8, v_max_mm_s / 1000.);
        return std::min((int)round(y * 1000. + 0.5), SLIDER_LIMIT);
    }
};

#endif //SHIMONCONTROLLER_SLIDERMODEL_H
