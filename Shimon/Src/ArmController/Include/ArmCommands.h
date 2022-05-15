//
// Created by Raghavasimhan Sankaranarayanan on 5/2/22.
//

#ifndef SHIMONCONTROLLER_ARMCOMMANDS_H
#define SHIMONCONTROLLER_ARMCOMMANDS_H

#include <iostream>
#include <optional>
#include <unordered_map>
#include "Util.h"

enum class ArmCommand {
    Unknown = -1,
    Off = 0,
    On = 1,
    Home = 2
};

enum class ArmCommandType {
    Dummy = 0,
    LowLevel,
    Midi,
    Servo
};

class ArmCmdPacket_t {
public:
    explicit ArmCmdPacket_t(ArmCommand cmd) :cmd(cmd), cmdType(ArmCommandType::Servo) {}

    ArmCommand cmd = ArmCommand::Unknown;
    ArmCommandType cmdType = ArmCommandType::Dummy;
};

class ArmCommandUtil {
public:
    static void populateStrCmdMap() {
        for (auto& element: kCmdStrMap) {
            strCmdMap[element.second] = element.first;
        }
    }

    static std::string getCommand(ArmCommand c) {
        if (kCmdStrMap.empty() || kCmdStrMap.find(c) == kCmdStrMap.end()) return "UNKNOWN";
        return kCmdStrMap.at(c);
    }

    static ArmCommand getCommand(const std::string& c) {
        auto data = Util::toUpperCase(c);
        if (strCmdMap.empty() || strCmdMap.find(data) == strCmdMap.end()) return ArmCommand::Unknown;
        return strCmdMap[data];
    }

    static inline const std::unordered_map<ArmCommand, std::string> kCmdStrMap = {
            {ArmCommand::Unknown, "UNKNOWN"   },
            {ArmCommand::Off,     "OFF"       },
            {ArmCommand::On,      "ON"        },
            {ArmCommand::Home,    "HOME"      },
    };
    static inline std::unordered_map<std::string, ArmCommand> strCmdMap;
};

#endif //SHIMONCONTROLLER_ARMCOMMANDS_H
