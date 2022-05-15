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

// Number of registers to query at a time
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

    enum class MsgType {
        RTU,
        ASCII
    };

    Modbus() = default;

    static Error_t computeRTU(const Message_t& msg, uint8_t* buf, size_t* pLength) {
        RTU::PositioningDataQuery_t query{};
        Error_t e;
        static const uint8_t kNumRegisters = 9;
        uint16_t data[kNumRegisters]{};
        query.numRegisters = kNumRegisters;
        query.pData = data;
        e = _translate(msg, query);
        ERROR_CHECK(e, e);
        return _packRTU(query, buf, pLength);
    }

    [[nodiscard]] static Error_t servoOn(int armId, bool bOn = true, uint8_t* ret = nullptr, size_t* pLength= nullptr) {
//        RTU::CtrlInfo_t msg = {
//                .slaveAddr = (uint8_t)(armId + 1),
//                .functionCode = (uint8_t)((bOn) ? 0x06 : 0x05),
//                .startAddr = (uint16_t)((bOn) ? 0x0D00 : 0x0403),
//                .data = (uint16_t)((bOn) ? 0x1000 : 0x0000)
//        };

        RTU::CtrlInfo_t msg = {
                .slaveAddr = (uint8_t)(armId + 1),
                .functionCode = 0x05,
                .startAddr = 0x0403,
                .data = (uint16_t)((bOn) ? 0xFF00 : 0x0000)
        };

        return _packRTU(msg, ret, pLength);
    }

    [[nodiscard]] static Error_t home(int armId, uint8_t* ret = nullptr, size_t* pLength= nullptr) {
//        static RTU::CtrlInfo_t msg = {
//                .functionCode = 0x06,
//                .startAddr = 0x0D00,
//                .data = 0x1010,
//        };
        static RTU::CtrlInfo_t msg = {
                .functionCode = 0x05,
                .startAddr = 0x040B,
                .data = 0xFF00,
        };
        msg.slaveAddr = armId + 1;
        return _packRTU(msg, ret, pLength);
    }

    [[nodiscard]] static Error_t alarmReset(uint8_t armId, uint8_t* ret = nullptr, size_t* pLength= nullptr) {
        RTU::CtrlInfo_t msg = {
                .slaveAddr = uint8_t(armId + 1),
                .functionCode = 0x05,
                .startAddr = 0x0407,
                .data = 0xFF00,
        };

        return _packRTU(msg, ret, pLength);
    }

    /*
     * Multiple FC03 register read
     */
    static Error_t queryAll(int armId, uint8_t* query= nullptr, size_t* length = nullptr, uint8_t fc = FC03) {
        if (!query) return kInvalidArgsError;
        RTU::DataReadQuery_t temp = {
                .slaveAddr = (uint8_t)(armId + 1),
                .functionCode = fc,
                .startAddr = 0x9000,
                .numRegisters = MODBUS_MULTI_NUM_REG
        };

        return _packRTU(temp, query, length);
    }

    static Error_t query(int armId, uint8_t* query= nullptr, size_t* length= nullptr, uint8_t fc = FC03, uint16_t reg=DSS1) {
        if (!query) return kInvalidArgsError;

        RTU::DataReadQuery_t temp = {
                .slaveAddr = (uint8_t)(armId + 1),
                .functionCode = fc,
                .startAddr = reg,
                .numRegisters = 1
        };

        return _packRTU(temp, query, length);
    }

    static Error_t parseDataReadResponse(const uint8_t* response, RTU::DataReadResponse_t& msg) {
        if (!response) return kInvalidArgsError;

        msg.slaveAddr = response[0];
        msg.functionCode = response[1];
        msg.numDataBytes = response[2];

        const int numReg = msg.numDataBytes/2;
        for (int i=0; i<numReg; ++i) {
            msg.data[i] = (response[3 + (2*i)] << 8) + response[3 + (2*i) + 1];
        }

        msg.errorCheck = (response[3 + msg.numDataBytes] << 8) + response[3 + msg.numDataBytes + 1];

        uint8_t ret[3 + msg.numDataBytes + 1];
        size_t length;
        Error_t e = _packRTU(msg, ret, &length);
        uint16_t crc = (ret[3 + msg.numDataBytes] << 8) + ret[3 + msg.numDataBytes + 1];

#ifndef SIMULATE
        if (msg.errorCheck != crc) {
            LOG_ERROR("Data corrupted. Received LRC: {}, Computed LRC: {}", msg.errorCheck, crc);
            return kCorruptedDataError;
        }
#endif
        return kNoError;
    }

    static Error_t computeAscii(const Message_t& msg, char* ret = nullptr) {
        RTU::PositioningDataQuery_t query{};
        Error_t e;
        static const uint8_t kNumRegisters = 9;
        uint16_t data[kNumRegisters]{};
        query.numRegisters = kNumRegisters;
        query.pData = data;
        e = _translate(msg, query);
        ERROR_CHECK(e, e);
        return _toAscii(query, ret);
    }

    [[nodiscard]] static Error_t servoOn(int armId, bool bOn = true, char* ret = nullptr) {
        RTU::CtrlInfo_t msg = {
            .slaveAddr = (uint8_t)(armId + 1),
            .functionCode = (uint8_t)((bOn) ? 0x06 : 0x05),
            .startAddr = (uint16_t)((bOn) ? 0x0D00 : 0x0403),
            .data = (uint16_t)((bOn) ? 0x1000 : 0x0000)
        };

        return _toAscii(msg, ret);
    }

    [[nodiscard]] static Error_t home(int armId, char* ret = nullptr) {
        static RTU::CtrlInfo_t msg = {
                .functionCode = 0x06,
                .startAddr = 0x0D00,
                .data = 0x1010,
        };
        msg.slaveAddr = armId + 1;
        return _toAscii(msg, ret);;
    }

    [[nodiscard]] static Error_t alarmReset(uint8_t armId, char* ret) {
        RTU::CtrlInfo_t msg = {
                .slaveAddr = uint8_t(armId + 1),
                .functionCode = 0x05,
                .startAddr = 0x0407,
                .data = 0xFF00,
        };

        return _toAscii(msg, ret);
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

        return _toAscii(temp, query);
    }

    static Error_t query(int armId, char* query= nullptr, uint8_t fc = FC03, uint16_t reg=DSS1) {
        if (!query) return kInvalidArgsError;

        RTU::DataReadQuery_t temp = {
                .slaveAddr = (uint8_t)(armId + 1),
                .functionCode = fc,
                .startAddr = reg,
                .numRegisters = 1
        };

        return _toAscii(temp, query);
    }

    static Error_t parseDataReadResponse(const char* response, RTU::DataReadResponse_t& msg) {
        if (!response) return kInvalidArgsError;

        msg.slaveAddr = Util::strtoi(&response[1], 2);
        msg.functionCode = Util::strtoi(&response[3], 2);
        msg.numDataBytes = Util::strtoi(&response[5], 2);

        const int numReg = msg.numDataBytes/2;
        for (int i=0; i<numReg; ++i) {
            msg.data[i] = Util::strtoi(&response[7 + (i * 4)], 4);
        }

        msg.errorCheck = Util::strtoi(&response[7 + (numReg*4)], 2);

        char ret[7 + (numReg*4) + 1];
        Error_t e = _toAscii(msg, ret);
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
    static Error_t _translate(const Message_t& msg, RTU::PositioningDataQuery_t& query) {
        uint32_t vel = msg.v_max * 100;
        uint32_t pos = msg.position * 100;

        auto* data = query.pData;
        data[0] = (pos >> 16) & 0xFF;           // Position MSB
        data[1] = pos & 0xFFFF;                 // Position LSB
        data[2] = 0x0;                          // position band MSB
        data[3] = int(msg.precision * 100);     // position band LSB
        data[4] = (vel >> 16) & 0xFF;           // speed MSB
        data[5] = vel & 0xFFFF;                 // speed LSB
        data[6] = int(msg.acceleration * 100);  // Acceleration
        data[7] = (uint16_t)msg.push;
        data[8] = 0x0;


        query.functionCode = 0x10;
        query.startAddr = 0x9900;
        query.numBytes = query.numRegisters * 2;
        query.slaveAddr = msg.armID + 1;

        return kNoError;
    }

    static Error_t _packRTU(const RTU::CtrlInfo_t& msg, uint8_t* buf = nullptr, size_t* length= nullptr) {
        if (!buf) return kInvalidArgsError;

        buf[0] = msg.slaveAddr;
        buf[1] = msg.functionCode;
        buf[2] = (msg.startAddr >> 8) & 0xFF;
        buf[3] = msg.startAddr & 0xFF;
        buf[4] = (msg.data >> 8) & 0xFF;
        buf[5] = msg.data & 0xFF;

        uint16_t crc = _computeCRC16(buf, 6);
        buf[6] = (crc >> 8) & 0xFF;
        buf[7] = crc & 0xFF;

        if (length) *length = 8;
        return kNoError;
    }

    static Error_t _packRTU(const RTU::DataReadQuery_t& msg, uint8_t* buf = nullptr, size_t* length= nullptr) {
        if (!buf) return kInvalidArgsError;

        buf[0] = msg.slaveAddr;
        buf[1] = msg.functionCode;
        buf[2] = (msg.startAddr >> 8) & 0xFF;
        buf[3] = msg.startAddr & 0xFF;
        buf[4] = (msg.numRegisters >> 8) & 0xFF;
        buf[5] = msg.numRegisters & 0xFF;

        uint16_t crc = _computeCRC16(buf, 6);
        buf[6] = (crc >> 8) & 0xFF;
        buf[7] = crc & 0xFF;

        if (length) *length = 8;
        return kNoError;
    }

    static Error_t _packRTU(const RTU::DataReadResponse_t& msg, uint8_t* buf = nullptr, size_t* length= nullptr) {
        if (!buf) return kInvalidArgsError;

        buf[0] = msg.slaveAddr;
        buf[1] = msg.functionCode;
        buf[2] = msg.numDataBytes;
        int numReg = msg.numDataBytes / 2;

        if (numReg < 1) return kInvalidArgsError;
        for (int i = 0; i<numReg; ++i) {
            buf[3 + (2*i)] = (msg.data[i] >> 8) & 0xFF;
            buf[3 + (2*i) + 1] = msg.data[i] & 0xFF;
        }

        uint16_t crc = _computeCRC16(buf, 3 + (2*numReg));
        buf[3 + (2*numReg)] = (crc >> 8) & 0xFF;
        buf[3 + (2*numReg) + 1] = crc & 0xFF;

        if (length) *length = 3 + (2*numReg) + 2;
        return kNoError;
    }

    static Error_t _packRTU(const RTU::PositioningDataQuery_t& msg, uint8_t* buf = nullptr, size_t* length= nullptr) {
        if (!buf) return kInvalidArgsError;

        buf[0] = msg.slaveAddr;
        buf[1] = msg.functionCode;
        buf[2] = (msg.startAddr >> 8) & 0xFF;
        buf[3] = msg.startAddr & 0xFF;
        buf[4] = (msg.numRegisters >> 8) & 0xFF;
        buf[5] = msg.numRegisters & 0xFF;
        buf[6] = msg.numBytes;

        for (int i=0; i<msg.numRegisters; ++i) {
            buf[7 + (2*i)] = (msg.pData[i] >> 8) & 0xFF;
            buf[7 + (2*i) + 1] = msg.pData[i] & 0xFF;
        }

        uint16_t crc = _computeCRC16(buf, 7 + (2*msg.numRegisters));
        buf[7 + (2*msg.numRegisters)] = (crc >> 8) & 0xFF;
        buf[7 + (2*msg.numRegisters) + 1] = crc & 0xFF;

        if (length) *length = 7 + (2*msg.numRegisters) + 2;
        return kNoError;
    }

    static Error_t _toAscii(const RTU::CtrlInfo_t& msg, char* buf = nullptr) {
        if (!buf) return kInvalidArgsError;

        sprintf(buf, ":%02X", msg.slaveAddr);
        sprintf(&buf[3], "%02X", msg.functionCode);
        sprintf(&buf[5], "%04X", msg.startAddr);
        sprintf(&buf[9], "%04X", msg.data);
        int lrc = _computeLrc8(buf, 13);
        sprintf(&buf[13], "%02X\r\n", lrc);
        buf[18] = 0;

        return kNoError;
    }

    static Error_t _toAscii(const RTU::DataReadQuery_t& msg, char* buf = nullptr) {
        if (!buf) return kInvalidArgsError;

        sprintf(buf, ":%02X", msg.slaveAddr);
        sprintf(&buf[3], "%02X", msg.functionCode);
        sprintf(&buf[5], "%04X", msg.startAddr);
        sprintf(&buf[9], "%04X", msg.numRegisters);
        int lrc = _computeLrc8(buf, 13);
        sprintf(&buf[13], "%02X\r\n", lrc);
        buf[18] = 0;

        return kNoError;
    }

    static Error_t _toAscii(const RTU::DataReadResponse_t& msg, char* buf = nullptr) {
        if (!buf) return kInvalidArgsError;

        sprintf(buf, ":%02X", msg.slaveAddr);
        sprintf(&buf[3], "%02X", msg.functionCode);
        sprintf(&buf[5], "%02X", msg.numDataBytes);
        int numReg = msg.numDataBytes / 2;

        if (numReg < 1) return kInvalidArgsError;

        for (int i = 0; i<numReg; ++i) sprintf(&buf[7 + (i*4)], "%04X", msg.data[i]);

        int lrc = _computeLrc8(buf, 7 + (numReg * 4));
        sprintf(&buf[7 + (numReg * 4)], "%02X\r\n", lrc);
        buf[12 + (numReg * 4)] = 0;

        return kNoError;
    }

    static Error_t _toAscii(const RTU::PositioningDataQuery_t& msg, char* buf = nullptr) {
        if (!buf) return kInvalidArgsError;

        sprintf(buf, ":%02X", msg.slaveAddr);
        sprintf(&buf[3], "%02X", msg.functionCode);
        sprintf(&buf[5], "%04X", msg.startAddr);
        sprintf(&buf[9], "%04X", msg.numRegisters);
        sprintf(&buf[13], "%02X", msg.numBytes);

        for (int i=0; i<msg.numRegisters; ++i) {
            sprintf(&buf[15 + (i*4)], "%04X", msg.pData[i]);
        }
        int lrc1 = _computeLrc8(buf, 51);
        sprintf(&buf[51], "%02X\r\n", lrc1);
        buf[56] = 0;

        return kNoError;
    }

    static uint8_t _computeLrc8(const char* msg, size_t len) {
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

    /* Table of CRC values for high-order byte */
    static constexpr uint8_t auiCRCHi[] = {
            0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
            0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
            0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
            0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
            0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
            0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
            0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
            0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
            0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
            0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
            0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
            0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
            0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
            0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
            0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
            0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
            0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
            0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
            0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
            0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
            0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
            0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
            0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
            0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
            0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
            0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
    };

/* Table of CRC values for low-order byte */
    static constexpr uint8_t auiCRCLo[] = {
            0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
            0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
            0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
            0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
            0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
            0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
            0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
            0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
            0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
            0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
            0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
            0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
            0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
            0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
            0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
            0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
            0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
            0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
            0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
            0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
            0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
            0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
            0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
            0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
            0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
            0x43, 0x83, 0x41, 0x81, 0x80, 0x40
    };

    static uint16_t _computeCRC16(const uint8_t* buf, size_t length) {
        uint8_t uiCrcHi = 0xFF; /* high CRC byte initialized */
        uint8_t uiCrcLo = 0xFF; /* low CRC byte initialized */
        unsigned int uiIdx; /* will index into CRC lookup table */

        /* pass through message buffer */
        while (length--) {
            uiIdx = uiCrcHi ^ *buf++; /* calculate the CRC  */
            uiCrcHi = uiCrcLo ^ auiCRCHi[uiIdx];
            uiCrcLo = auiCRCLo[uiIdx];
        }

        return (uiCrcHi << 8 | uiCrcLo);
    }
};



#endif //SHIMONCONTROLLER_MODBUS_H
