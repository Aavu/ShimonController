//
// Created by Raghavasimhan Sankaranarayanan on 5/11/22.
//

#ifndef SHIMONCONTROLLER_IAICONTROLLER_H
#define SHIMONCONTROLLER_IAICONTROLLER_H

#include <mutex>
#include "Modbus.h"
#include "Def.h"
#include "ErrorDef.h"
#include "Logger.h"
#include "SerialDevice.h"

#define IAI_BUFFER_SIZE 256
#define RTU_BUFFER_SIZE 256

#define IAI_SERIAL_SLEEP (3 + 5 + (int)round(10. * (17 + 8) / (IAI_BAUDRATE/1000.0)))

#define IAI_TIMEOUT 1000 // ms
#define IAI_NTRY 3

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

        LOG_TRACE("Initializing IAI Actuator");
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

    bool isHomingComplete() {
        if (!m_bInitialized) return false;
        bool bHomed = true;

        size_t length;
        for (int i=0; i<NUM_ARMS; ++i) {
            Error_t e;

            int iTry = 0;

            while(iTry < IAI_NTRY) {
                e = query(i, asciiMsg, FC03, DSS1);
                ERROR_CHECK(e, false);

                e = m_serialDevice.flush();
                ERROR_CHECK(e, false);

                e = m_serialDevice.write(asciiMsg);
                ERROR_CHECK(e, false);

                // Tout = To + α + (10 x Bprt/Kbr) [msec]
                Util::sleep_ms(3 + 5 + (int) ceil(10. * (7 + 8) / (IAI_BAUDRATE / 1000.0)));
                int n = m_serialDevice.readline(asciiMsg, '\n', IAI_BUFFER_SIZE, IAI_TIMEOUT);
                if (n > 0) {
                    break;
                }
                iTry++;
            }

            RTU::DataReadResponse_t resp{};
            e = parseDataReadResponse(asciiMsg, resp);
            ERROR_CHECK(e, false);

            auto data = resp.data[0];

            if (!((data >> HEND) & 0x1)) bHomed = false;
        }

        return bHomed;
    }

    Error_t setPosition(const Modbus::Message_t& msg, MsgType msgType= MsgType::ASCII) {
        std::lock_guard<std::mutex> lk(m_mtx);
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

        RTU::PositioningDataQuery_t query{};
        e = computeAscii(msg, query, asciiMsg);
        ERROR_CHECK(e, e);

        e = m_serialDevice.write(asciiMsg);
        ERROR_CHECK(e, e);

        // Tout = To + α + (10 x Bprt/Kbr) [msec]
        Util::sleep_ms(IAI_SERIAL_SLEEP);
#ifndef SIMULATE
        bool success = true;
        char response[IAI_BUFFER_SIZE];
        int n = m_serialDevice.readline(response, '\n', IAI_BUFFER_SIZE, 10);
        if (n == 0) {
            LOG_ERROR("Timeout");
            return kTimeoutError;
        } else if ( n < 0) {
            LOG_ERROR("Read Error");
            return kReadError;
        }

        RTU::PositioningDataResponse_t resp{};

        e = parsePositioningDataResponse(response, resp);
        ERROR_CHECK(e, e);
//
//        success = false;
//        if (resp.startAddr == query.startAddr) {
//            if (resp.numRegisters == query.numRegisters) {
//                if (resp.functionCode == query.functionCode) {
//                    if (resp.slaveAddr == query.slaveAddr) {
//                        success = true;
//                    }
//                }
//            }
//        }
//
////        if (!resp.operator==(query)) return kSetValueError;
        if (!success) return kSetValueError;
#else
        Util::sleep_ms(10);
#endif
        return kNoError;
    }

    Error_t setServo(int armId, bool bOn = true) {
        std::lock_guard<std::mutex> lk(m_mtx);
        if (!m_bInitialized) return kNotInitializedError;
        Error_t e;
        size_t length;
        e = servoOn(armId, bOn, rtuMsg, &length);
        ERROR_CHECK(e, e);

        Util::sleep_us(200);
        e = m_serialDevice.write(rtuMsg, length);
        ERROR_CHECK(e, e);
        Util::sleep_us(200);

        return kNoError;
    }

    [[nodiscard]] bool isHomed() const {
        return m_bHomed;
    }

    Error_t home() {
        if (!m_bInitialized) return kNotInitializedError;
        LOG_INFO("Homing IAI Actuators");
        size_t length;

        Error_t e;

        for (int i=0; i<NUM_ARMS; ++i) {
            e = Modbus::normalStatus(i, asciiMsg);
            ERROR_CHECK(e, e);

//        Util::sleep_us(200);
            e = m_serialDevice.write(asciiMsg);
            ERROR_CHECK(e, e);
//        Util::sleep_us(200);

            e = m_serialDevice.flush();
            ERROR_CHECK(e, e);
            Util::sleep_ms(100);

            e = Modbus::home(i, asciiMsg);
            ERROR_CHECK(e, e);

//        Util::sleep_us(200);
            e = m_serialDevice.write(asciiMsg);
            ERROR_CHECK(e, e);
//        Util::sleep_us(200);
            Util::sleep_ms(100);
        }

#ifndef SIMULATE
        int i = 0;
        int timeOut_ms = 60000;
        int sleep_ms = 100;
        while (!isHomingComplete()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
            i += sleep_ms;
            if (i > timeOut_ms) {
                LOG_WARN("Homing Timeout...");
                return kTimeoutError;
            }
        }
#else
        std::this_thread::sleep_for(std::chrono::seconds(1));
#endif

        m_bHomed = true;
        LOG_INFO("IAI Actuators Homing Complete!");
        return kNoError;
    }

    Error_t clearFault() {
        Error_t e;
        for (int i=0; i<NUM_ARMS; ++i) {
            e = clearFault(i);
            ERROR_CHECK(e, e);
        }

        return kNoError;
    }

    Error_t clearFault(int armId) {
        std::lock_guard<std::mutex> lk(m_mtx);
        Error_t e;
        e = Modbus::alarmReset(armId, asciiMsg);
        ERROR_CHECK(e, e);
        e = m_serialDevice.write(asciiMsg);
        ERROR_CHECK(e, e);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        e = Modbus::alarmNormal(armId, asciiMsg);
        ERROR_CHECK(e, e);

        e = m_serialDevice.write(asciiMsg);
        ERROR_CHECK(e, e);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        return kNoError;
    }

    Error_t updateStats(int armId) {
        if (!m_bInitialized) return kNotInitializedError;
        Error_t e;
//        uint8_t buf[RTU_BUFFER_SIZE];
//        size_t length;
        e = Modbus::queryAll(armId, asciiMsg, FC03);
        ERROR_CHECK(e, e);

        e = m_serialDevice.flush();
        ERROR_CHECK(e, e);

        e = m_serialDevice.write(asciiMsg);
        ERROR_CHECK(e, e);

        // Tout = To + α + (10 x Bprt/Kbr) [msec]
        Util::sleep_ms(20 + 5 + (int)ceil(10. * (7 + 8) / (IAI_BAUDRATE/1000.0)));
        int n = m_serialDevice.readline(asciiMsg, '\n', IAI_BUFFER_SIZE, IAI_TIMEOUT);
        if (n == 0) {
            LOG_ERROR("Timeout");
            return kTimeoutError;
        } else if ( n < 0) {
            LOG_ERROR("Read Error");
            return kReadError;
        }

        RTU::DataReadResponse_t msg{};
        e = parseDataReadResponse(asciiMsg, msg);
        ERROR_CHECK(e, e);

        return parseQueryAllResponse(msg.data, m_status[armId]);
    }

    [[nodiscard]] int getPosition(int armId) const {
        return m_status[armId].position;
    }

private:
    bool m_bInitialized = false;
    SerialDevice m_serialDevice;

    bool m_bHomed = false;
    RTU::MultiRegResponse_t m_status[NUM_ARMS]{};

    static inline char asciiMsg[IAI_BUFFER_SIZE];
    static inline uint8_t rtuMsg[RTU_BUFFER_SIZE];

    std::mutex m_mtx;

    IAIController() = default;
    ~IAIController() = default;
};

#endif //SHIMONCONTROLLER_IAICONTROLLER_H
