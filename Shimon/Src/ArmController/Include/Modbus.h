//
// Created by Raghavasimhan Sankaranarayanan on 4/9/22.
//

#ifndef SHIMONCONTROLLER_MODBUS_H
#define SHIMONCONTROLLER_MODBUS_H
#include <iostream>
#include <cmath>
#include <sstream>
#include <iomanip>

#include "Util.h"
#include "ErrorDef.h"
#include "Def.h"

#define PEND 3
#define HEND 4
#define STP 5

#define PNOW 0x9000
#define ALMC 0x9002
#define DSS1 0x9005

#define FC03 0x03
#define FC05 0x05
#define FC10 0x10

#define MODBUS_MULTI_NUM_REG 10

namespace RTU {
    // Direct Writing of Control Info (Queries using Code 06)
    struct CtrlInfo_t {
        uint8_t slaveAddr;
        uint8_t functionCode;
        uint16_t startAddr;
        uint16_t data;
        uint16_t errCheck;
    };

    // Direct Writing of Positioning Data (Queries using Code 10)
    struct PositioningDataQuery_t {
        uint8_t slaveAddr;
        uint8_t functionCode;
        uint16_t startAddr;
        uint16_t numRegisters;
        uint8_t numBytes;
        uint16_t* pData;
        uint16_t errorCheck;
    };

    struct PositioningDataResponse_t {
        uint8_t slaveAddr;
        uint8_t functionCode;
        uint16_t startAddr;
        uint16_t numRegisters;
        uint16_t errorCheck;
    };

    struct DataReadQuery_t {
        uint8_t slaveAddr;
        uint8_t functionCode;
        uint16_t startAddr;
        uint16_t numRegisters;
        uint16_t errorCheck;
    };

    struct DataReadResponse_t {
        uint8_t slaveAddr;
        uint8_t functionCode;
        uint8_t numDataBytes;
        uint16_t data[MODBUS_MULTI_NUM_REG];
        uint16_t errorCheck;
    };

    struct MultiRegResponse_t {
        int32_t position;
        uint16_t alarm;
        uint16_t inputPort;
        uint16_t outputPort;
        uint16_t devStat1;
        uint16_t devStat2;
        uint16_t expDevStat;
        uint32_t sysStat;
    };
}

class Modbus {
public:
    struct Message_t {
        int armID;
        int position;
        float precision;
        int v_max;
        float acceleration;
        int push;
    };

    Modbus() = default;
    static Error_t computeAscii(const Message_t& msg, char* ret = nullptr) {
        static const uint8_t kNumRegisters = 9;
        uint16_t data[kNumRegisters]{};

        uint32_t vel = msg.v_max * 100;
        uint32_t pos = msg.position * 100;

        data[0] = (pos >> 16) & 0xFF;           // Position MSB
        data[1] = pos & 0xFFFF;                 // Position LSB
        data[2] = 0x0;                          // position band MSB
        data[3] = int(msg.precision * 100);     // position band LSB
        data[4] = (vel >> 16) & 0xFF;           // speed MSB
        data[5] = vel & 0xFFFF;                 // speed LSB
        data[6] = int(msg.acceleration * 100);  // Acceleration
        data[7] = (uint16_t)msg.push;
        data[8] = 0x0;

        static RTU::PositioningDataQuery_t query = {
                .functionCode = 0x10,
                .startAddr = 0x9900,
                .numRegisters = kNumRegisters,
                .numBytes = kNumRegisters * 2,
        };

        query.slaveAddr = msg.armID + 1;
        query.pData = data;

        return toAscii(query, ret);
    }

    [[nodiscard]] static Error_t servoOn(int armId, bool bOn = true, char* ret = nullptr) {
        RTU::CtrlInfo_t msg = {
            .slaveAddr = (uint8_t)(armId + 1),
            .functionCode = (uint8_t)((bOn) ? 0x06 : 0x05),
            .startAddr = (uint16_t)((bOn) ? 0x0D00 : 0x0403),
            .data = (uint16_t)((bOn) ? 0x1000 : 0x0000)
        };

        return toAscii(msg, ret);
    }

    [[nodiscard]] static Error_t home(int armId, char* ret = nullptr) {
        static RTU::CtrlInfo_t msg = {
                .functionCode = 0x06,
                .startAddr = 0x0D00,
                .data = 0x1010,
        };
        msg.slaveAddr = armId + 1;
        return toAscii(msg, ret);;
    }

    [[nodiscard]] static Error_t alarmReset(uint8_t armId, char* ret) {
        RTU::CtrlInfo_t msg = {
                .slaveAddr = uint8_t(armId + 1),
                .functionCode = 0x05,
                .startAddr = 0x0407,
                .data = 0xFF00,
        };

        return toAscii(msg, ret);
    }

    /*
     * Multiple FC03 register read
     */
    static Error_t queryAll(int armId, char* query= nullptr, uint8_t fc = FC03) {
        if (!query) return kInvalidArgsError;
        RTU::DataReadQuery_t temp = {
                .slaveAddr = (uint8_t)(armId + 1),
                .functionCode = fc,
                .startAddr = 0x9000,
                .numRegisters = MODBUS_MULTI_NUM_REG
        };

        return toAscii(temp, query);;
    }

    static Error_t query(int armId, char* query= nullptr, uint8_t fc = FC03, uint16_t reg=DSS1) {
        if (!query) return kInvalidArgsError;

        RTU::DataReadQuery_t temp = {
                .slaveAddr = (uint8_t)(armId + 1),
                .functionCode = fc,
                .startAddr = reg,
                .numRegisters = 1
        };

        return toAscii(temp, query);
    }

    static Error_t parseDataReadResponse(const char* response, RTU::DataReadResponse_t& msg) {
        if (!response) return kSegvError;

        msg.slaveAddr = Util::strtoi(&response[1], 2);
        msg.functionCode = Util::strtoi(&response[3], 2);
        msg.numDataBytes = Util::strtoi(&response[5], 2);

        const int numReg = msg.numDataBytes/2;
        for (int i=0; i<numReg; ++i) {
            msg.data[i] = Util::strtoi(&response[7 + (i * 4)], 4);
        }

        msg.errorCheck = Util::strtoi(&response[7 + (numReg*4)], 2);

        char ret[7 + (numReg*4) + 1];
        Error_t e = toAscii(msg, ret);
        int lrc = Util::strtoi(&ret[7 + (numReg*4)], 2);

#ifndef SIMULATE
        if (msg.errorCheck != lrc) {
            LOG_ERROR("Data corrupted. Received LRC: {}, Computed LRC: {}", msg.errorCheck, lrc);
            return kCorruptedDataError;
        }
#endif
        return kNoError;
    }

    static Error_t parseQueryAllResponse(const uint16_t* pData, RTU::MultiRegResponse_t& msg) {
        if (!pData) return kInvalidArgsError;

        msg.position = (int32_t)((pData[1] << 16) + pData[0]);
        msg.alarm = pData[2];
        msg.inputPort = pData[3];
        msg.outputPort = pData[4];
        msg.devStat1 = pData[5];
        msg.devStat2 = pData[6];
        msg.expDevStat = pData[7];
        msg.sysStat = (uint32_t)((pData[8] << 16) + pData[9]);

        return kNoError;
    }

private:
    static Error_t toAscii(const RTU::CtrlInfo_t& msg, char* buf = nullptr) {
        if (!buf) return kSegvError;

        sprintf(buf, ":%02X", msg.slaveAddr);
        sprintf(&buf[3], "%02X", msg.functionCode);
        sprintf(&buf[5], "%04X", msg.startAddr);
        sprintf(&buf[9], "%04X", msg.data);
        int lrc = computeLrc(buf, 13);
        sprintf(&buf[13], "%02X\r\n", lrc);
        buf[18] = 0;

        return kNoError;
    }

    static Error_t toAscii(const RTU::DataReadQuery_t& msg, char* buf = nullptr) {
        if (!buf) return kSegvError;

        sprintf(buf, ":%02X", msg.slaveAddr);
        sprintf(&buf[3], "%02X", msg.functionCode);
        sprintf(&buf[5], "%04X", msg.startAddr);
        sprintf(&buf[9], "%04X", msg.numRegisters);
        int lrc = computeLrc(buf, 13);
        sprintf(&buf[13], "%02X\r\n", lrc);
        buf[18] = 0;

        return kNoError;
    }

    static Error_t toAscii(const RTU::DataReadResponse_t& msg, char* buf = nullptr) {
        if (!buf) return kSegvError;

        sprintf(buf, ":%02X", msg.slaveAddr);
        sprintf(&buf[3], "%02X", msg.functionCode);
        sprintf(&buf[5], "%02X", msg.numDataBytes);
        int numReg = msg.numDataBytes / 2;

        if (numReg < 1) return kInvalidArgsError;

        for (int i = 0; i<numReg; ++i) sprintf(&buf[7 + (i*4)], "%04X", msg.data[i]);

        int lrc = computeLrc(buf, 7 + (numReg*4));
        sprintf(&buf[7 + (numReg * 4)], "%02X\r\n", lrc);
        buf[12 + (numReg * 4)] = 0;

        return kNoError;
    }

    static Error_t toAscii(const RTU::PositioningDataQuery_t& msg, char* buf = nullptr) {
        if (!buf) return kSegvError;

        sprintf(buf, ":%02X", msg.slaveAddr);
        sprintf(&buf[3], "%02X", msg.functionCode);
        sprintf(&buf[5], "%04X", msg.startAddr);
        sprintf(&buf[9], "%04X", msg.numRegisters);
        sprintf(&buf[13], "%02X", msg.numBytes);

        for (int i=0; i<msg.numRegisters; ++i) {
            sprintf(&buf[15 + (i*4)], "%04X", msg.pData[i]);
        }
        int lrc1 = computeLrc(buf, 51);
        sprintf(&buf[51], "%02X\r\n", lrc1);
        buf[56] = 0;

        return kNoError;
    }

    static uint8_t computeLrc(const char* msg, size_t len) {
        uint8_t sum = 0;
        char temp[3];
        temp[2] = '\0';
        for (int i=1; i < len; i += 2) {
            temp[0] = msg[i];
            temp[1] = msg[i+1];
            sum += strtol(temp, nullptr, 16);
        }
        return (sum ^ 0xFF) + 1;
    }
};



#endif //SHIMONCONTROLLER_MODBUS_H
