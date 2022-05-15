//
// Created by Raghavasimhan Sankaranarayanan on 5/11/22.
//

#ifndef SHIMONCONTROLLER_IAICONTROLLER_H
#define SHIMONCONTROLLER_IAICONTROLLER_H

#include "Modbus.h"
#include "Def.h"
#include "ErrorDef.h"
#include "Logger.h"
#include "SerialDevice.h"

#define IAI_BUFFER_SIZE 256
#define RTU_BUFFER_SIZE 256

#define IAI_TIMEOUT 100 // ms

#define ALL_ARMS -1

class IAIController: Modbus {
public:
    // Singleton Class
    IAIController(const IAIController&) = delete;
    IAIController(IAIController&&) = delete;
    IAIController& operator=(const IAIController&) = delete;
    IAIController& operator=(IAIController&&) = delete;

    static IAIController& getInstance() {
        static IAIController instance;
        return instance;
    }

    Error_t init(const std::string& port, int iBaudrate) {
        if (m_bInitialized) return kReInitializationError;
        Error_t e;
        e = m_serialDevice.openPort(port);
        ERROR_CHECK(e, e);

        e = m_serialDevice.init(iBaudrate);
        ERROR_CHECK(e, e);

        m_bInitialized = true;
        return kNoError;
    }

    Error_t reset(bool bSoftReset = false) {
        if (!m_bInitialized) return kNoError;
        Error_t e;

        if (!bSoftReset) {
            e = m_serialDevice.reset();
            ERROR_CHECK(e, e);
        }

        m_bHomed = false;
        m_bInitialized = false;

        return kNoError;
    }

    bool isHomed() {
        if (!m_bInitialized) return kNotInitializedError;
        bool bHomed = true;

        static char line[16];

        size_t length;
        for (int i=0; i<NUM_ARMS; ++i) {
            Error_t e;
            e = query(i, rtuMsg, &length, FC03, DSS1);
            ERROR_CHECK(e, false);

            e = m_serialDevice.write(rtuMsg, length);
            ERROR_CHECK(e, false);

            e = m_serialDevice.readBytes(rtuMsg, 6, 50);
            ERROR_CHECK(e, false);

            RTU::DataReadResponse_t resp{};
            e = parseDataReadResponse(line, resp);
            ERROR_CHECK(e, false);

            auto data = resp.data[0];

            if (!((data >> HEND) & 0x1)) bHomed = false;
        }

        return bHomed;
    }

    Error_t write(const Modbus::Message_t& msg, MsgType msgType= MsgType::RTU) {
        if (!m_bInitialized) return kNotInitializedError;
        if (!m_bHomed) return kNotHomedError;
        Error_t e;

        if (msgType == MsgType::RTU) {
            size_t length;
            e = computeRTU(msg, rtuMsg, &length);
            ERROR_CHECK(e, e);

            Util::sleep_us(200);
            e = m_serialDevice.write(rtuMsg, length);
            ERROR_CHECK(e, e);
            Util::sleep_us(200);

            return kNoError;
        }

        e = computeAscii(msg, asciiMsg);
        ERROR_CHECK(e, e);

        return m_serialDevice.write(asciiMsg);

    }

    Error_t setServo(int armId, bool bOn = true) {
        if (!m_bInitialized) return kNotInitializedError;
        Error_t e;
        size_t length;
        e = servoOn(armId, bOn, rtuMsg, &length);
        ERROR_CHECK(e, e);

        return m_serialDevice.write(rtuMsg, length);
    }

    Error_t home() {
        if (!m_bInitialized) return kNotInitializedError;
        LOG_INFO("Homing IAI Actuators");
        size_t length;

        Error_t e;
        e = Modbus::home(ALL_ARMS, rtuMsg, &length);
        ERROR_CHECK(e, e);

        e = m_serialDevice.write(rtuMsg, length);
        ERROR_CHECK(e, e);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

#ifndef SIMULATE
        int i = 0;
        while (!isHomed()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            if (i++ > 60) LOG_WARN("Homing Timeout...");
        }
#else
        std::this_thread::sleep_for(std::chrono::seconds(5));
#endif

        m_bHomed = true;
        LOG_INFO("IAI Actuators Homing Complete!");
        return kNoError;
    }

    Error_t updateStats(int armId) {
        if (!m_bInitialized) return kNotInitializedError;
        Error_t e;
        uint8_t buf[RTU_BUFFER_SIZE];
        size_t length;
        e = Modbus::queryAll(armId, buf, &length, FC03);
        ERROR_CHECK(e, e);

        e = m_serialDevice.write(buf, length);
        ERROR_CHECK(e, e);

        e = m_serialDevice.readBytes(buf, length, IAI_TIMEOUT);
        ERROR_CHECK(e, e);

        RTU::DataReadResponse_t msg{};
        e = Modbus::parseDataReadResponse(buf, msg);
        ERROR_CHECK(e, e);

        return Modbus::parseQueryAllResponse(msg.data, m_status[armId]);
    }

    int getPosition(int armId) const {
        return m_status[armId].position;
    }

private:
    bool m_bInitialized = false;
    SerialDevice m_serialDevice;

    bool m_bHomed = false;
    RTU::MultiRegResponse_t m_status[NUM_ARMS]{};

    static inline char asciiMsg[IAI_BUFFER_SIZE];
    static inline uint8_t rtuMsg[RTU_BUFFER_SIZE];

    IAIController() = default;
    ~IAIController() = default;
};

#endif //SHIMONCONTROLLER_IAICONTROLLER_H
