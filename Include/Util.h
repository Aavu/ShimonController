//
// Created by Raghavasimhan Sankaranarayanan on 4/9/22.
//

#ifndef SHIMONCONTROLLER_UTIL_H
#define SHIMONCONTROLLER_UTIL_H

#include <iostream>
#include <chrono>
#include <algorithm>
#include <cctype>
#include <string>
#include <cstdio>
#include <array>

#include "Def.h"

class Util {
public:
    static void sleep_us(int micro) {
        std::this_thread::sleep_for(std::chrono::microseconds (micro));
    }

    static void sleep_ms(int millis) {
        std::this_thread::sleep_for(std::chrono::milliseconds (millis));
    }

    static void sleep_s(int seconds) {
        std::this_thread::sleep_for(std::chrono::seconds(seconds));
    }

    static bool valueInArray(int value, const int* pArr, int len) {
        for (int i = 0; i < len; ++i) {
            if (value == pArr[i]) return true;
        }
        return false;
    }

    static bool isWhiteKey(int midiNote) {
        return !valueInArray(midiNote % 12, kBlackKeys, 5);
    }

    static int transpose(int midiNote, int semitones) {
        return (midiNote - MIN_NOTE + semitones) % MIN_NOTE + MIN_NOTE;
    }

    static std::string toLowerCase(const std::string& data) {
        auto out = data;
        std::transform(out.begin(), out.end(), out.begin(),
                       [](unsigned char c){ return std::tolower(c); });
        return std::move(out);
    }

    static std::string toUpperCase(const std::string& data) {
        auto out = data;
        std::transform(out.begin(), out.end(), out.begin(),
                       [](unsigned char c){ return std::toupper(c); });
        return std::move(out);
    }

    static int strtoi(const char* c, int n, int base=16) {
        int num = 0;
        int m = 0;
        for (int i=n-1; i>-1; --i) {
            int temp = hex2int(c[i]);
            if (temp < 0) break;
            num += std::pow(base, m) * temp;
            ++m;
        }
        return num;
    }

    static int hex2int(char c) {
        if (isdigit(c)) return c - '0';
        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        return -1;
    }

    // Refer: https://stackoverflow.com/questions/2602823/in-c-c-whats-the-simplest-way-to-reverse-the-order-of-bits-in-a-byte
    static uint8_t reverseBits(uint8_t num) {
        return (_bitRevLUT[num & 0xF] << 4) | _bitRevLUT[num >> 4];
    }

    // [1, 3, 2, 4]
    static uint8_t computeStrikerId(const std::list<int>& strikerIds) {
        if (strikerIds.empty()) return -1;

        uint8_t id = 0;
        for (const int& i: strikerIds) id += (1 << i);

        return id;
    }

    static bool arePositionsDifferent(std::array<int, NUM_ARMS> a, std::array<int, NUM_ARMS> b) {
        for (int i=0; i<NUM_ARMS; ++i) {
            if (a[i] != b[i]) return true;
        }
        return false;
    }

private:
    static constexpr uint8_t _bitRevLUT[] = {
        0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe,
        0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf
    };
};

#endif //SHIMONCONTROLLER_UTIL_H
