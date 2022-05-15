//
// Created by Raghavasimhan Sankaranarayanan on 4/12/22.
//

#ifndef SHIMONCONTROLLER_HEADCOMMANDS_H
#define SHIMONCONTROLLER_HEADCOMMANDS_H

#include <iostream>
#include <optional>
#include <unordered_map>
#include "Util.h"

// Order matters!!! Dont change order
enum class HeadCommand {
    Unknown = -1,
    /****** Copley *******/
    BasePan = 0,
    Neck,
    NeckPan,
    HeadTilt,
    /****** Dxl *******/
    Mouth = 4,
    Eyebrow,
    /****** HighLevel Commands *******/
    Breathe = 6,
    StopBreath,
    HeadBang
};

enum class HeadCommandType {
    Dummy = 0,
    Control,
    LowLevel,
    Behavioral
};

class HeadCmdPacket_t {
public:
    HeadCmdPacket_t() = default;
    HeadCmdPacket_t(HeadCommand command, std::optional<float> fPosition) : cmd(command), fPos(fPosition), cmdType(HeadCommandType::LowLevel) {}
    HeadCmdPacket_t(HeadCommand command, std::optional<float> fPosition, std::optional<float> fVelocity) : cmd(command), fPos(fPosition), fVel(fVelocity), cmdType(HeadCommandType::LowLevel) {}
    HeadCmdPacket_t(HeadCommand command, std::optional<float> fPosition, std::optional<float> fVelocity, std::optional<float> fAcc) : cmd(command), fPos(fPosition), fVel(fVelocity), fAcc(fAcc), cmdType(HeadCommandType::LowLevel) {}
    explicit HeadCmdPacket_t(HeadCommand command) : cmd(command), cmdType(HeadCommandType::Behavioral) {}

    HeadCommand cmd = HeadCommand::Unknown;
    HeadCommandType cmdType = HeadCommandType::Dummy;

    std::optional<float> fPos;
    std::optional<float> fAcc;
    std::optional<float> fVel;

    HeadCmdPacket_t(const HeadCmdPacket_t&) = delete;
    HeadCmdPacket_t& operator=(const HeadCmdPacket_t&) = delete;
    HeadCmdPacket_t(HeadCmdPacket_t&& pkt)  noexcept : cmd(std::move(pkt.cmd)), cmdType(std::move(pkt.cmdType)), fPos(std::move(pkt.fPos)), fVel(std::move(pkt.fVel)), fAcc(std::move(pkt.fAcc)) {}

    HeadCmdPacket_t& operator=(HeadCmdPacket_t&& other)  noexcept {
        cmd     = std::move(other.cmd);
        cmdType = std::move(other.cmdType);
        fPos    = std::move(other.fPos);
        fVel    = std::move(other.fVel);
        fAcc    = std::move(other.fAcc);
        return *this;
    }
};

class HeadCommandUtil {
public:
    static void populateStrCmdMap() {
        for (auto& element: kCmdStrMap) {
            strCmdMap[element.second] = element.first;
        }
    }

    static std::string getCommand(HeadCommand c) {
        if (kCmdStrMap.empty() || kCmdStrMap.find(c) == kCmdStrMap.end()) return "UNKNOWN";
        return kCmdStrMap.at(c);
    }

    static HeadCommand getCommand(const std::string& c) {
        auto data = Util::toUpperCase(c);
        if (strCmdMap.empty() || strCmdMap.find(data) == strCmdMap.end()) return HeadCommand::Unknown;
        return strCmdMap[data];
    }

    static Port::Head getPortFor(const std::string& c) {
        return getPortFor(getCommand(c));
    }

    static Port::Head getPortFor(HeadCommand cmd) {
        if (cmd == HeadCommand::Unknown) return Port::Head::kNumPorts;
        else if (cmd < HeadCommand::Mouth) return Port::Head::Copley;
        else if (cmd < HeadCommand::Breathe) return Port::Head::Dxl;
        return Port::Head::kNumPorts;
    }

    static bool isHighLevelCommand(HeadCommand cmd) {
        return cmd >= HeadCommand::Breathe;
    }

    static inline const std::unordered_map<HeadCommand, std::string> kCmdStrMap = {
            {HeadCommand::Unknown,    "UNKNOWN"       },
            {HeadCommand::BasePan,    "BASEPAN"       },
            {HeadCommand::Neck,       "NECK"          },
            {HeadCommand::NeckPan,    "NECKPAN"       },
            {HeadCommand::HeadTilt,   "HEADTILT"      },
            {HeadCommand::Mouth,      "MOUTH"         },
            {HeadCommand::Eyebrow,    "EYEBROW"       },
            {HeadCommand::Breathe,    "BREATHING"     },
            {HeadCommand::StopBreath, "STOPBREATH"    },
            {HeadCommand::HeadBang,   "BEAT"          }
    };

    static inline std::unordered_map<std::string, HeadCommand> strCmdMap;

};

#endif //SHIMONCONTROLLER_HEADCOMMANDS_H
