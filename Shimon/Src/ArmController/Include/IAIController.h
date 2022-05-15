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

#define IAI_BUFFER_SIZE 64
#define IAI_TIMEOUT 100 // ms

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

        for (int i=0; i<NUM_ARMS; ++i) {
            Error_t e;
            e = query(i, asciiMsg, FC03, DSS1);
            ERROR_CHECK(e, false);

            e = m_serialDevice.write(asciiMsg);
            ERROR_CHECK(e, false);

            e = m_serialDevice.readline(line, '\n', 16, 50);
            ERROR_CHECK(e, false);

            RTU::DataReadResponse_t resp;
            e = parseDataReadResponse(line, resp);
            ERROR_CHECK(e, false);

            auto data = resp.data[0];

            if (!((data >> HEND) & 0x1)) bHomed = false;
        }

        return bHomed;
    }

    Error_t write(const Modbus::Message_t& msg) {
        if (!m_bInitialized) return kNotInitializedError;
        if (!m_bHomed) return kNotHomedError;
        Error_t e;
        e = computeAscii(msg, asciiMsg);
        ERROR_CHECK(e, e);

        return m_serialDevice.write(asciiMsg);
    }

    Error_t setServo(int armId, bool bOn = true) {
        if (!m_bInitialized) return kNotInitializedError;
        Error_t e;
        e = Modbus::servoOn(armId, bOn, asciiMsg);
        ERROR_CHECK(e, e);

        return m_serialDevice.write(asciiMsg);
    }

    Error_t home() {
        if (!m_bInitialized) return kNotInitializedError;
        LOG_INFO("Homing IAI Actuators");
        for (int i=0; i<NUM_ARMS; ++i) {
            Error_t e;
            e = Modbus::home(i, asciiMsg);
            ERROR_CHECK(e, e);

            e = m_serialDevice.write(asciiMsg);
            ERROR_CHECK(e, e);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

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
        char buf[IAI_BUFFER_SIZE];
        e = Modbus::queryAll(armId, buf, FC03);
        ERROR_CHECK(e, e);

        e = m_serialDevice.write(buf);
        ERROR_CHECK(e, e);

        e = m_serialDevice.readline(buf, '\n', IAI_BUFFER_SIZE, IAI_TIMEOUT);
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
    RTU::MultiRegResponse_t m_status[NUM_ARMS];

    static inline char asciiMsg[IAI_BUFFER_SIZE];

    IAIController() = default;
    ~IAIController() = default;
};

#endif //SHIMONCONTROLLER_IAICONTROLLER_H
