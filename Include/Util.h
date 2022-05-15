//
// Created by Raghavasimhan Sankaranarayanan on 4/9/22.
//

#ifndef SHIMONCONTROLLER_UTIL_H
#define SHIMONCONTROLLER_UTIL_H

#include <iostream>
#include <algorithm>
#include <cctype>
#include <string>
#include <cstdio>

#include "Def.h"

class Util {
public:
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
};

#endif //SHIMONCONTROLLER_UTIL_H
