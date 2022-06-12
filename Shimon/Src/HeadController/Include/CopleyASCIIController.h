//
// Created by Raghavasimhan Sankaranarayanan on 4/12/22.
// Based on Shimon Head Java Code
//

#ifndef SHIMONCONTROLLER_COPLEYASCIICONTROLLER_CUH
#define SHIMONCONTROLLER_COPLEYASCIICONTROLLER_CUH

#include <climits>
#include <iostream>
#include <unistd.h>
#include <sstream>
#include <unordered_map>
#include "SerialDevice.h"
#include "ErrorDef.h"

#include "Logger.h"

#define PATIENCE 100     // ms
#define COPLEY_BUFFER_SIZE 32

class CopleyASCIIController {
public:
    // Singleton Class
    CopleyASCIIController(const CopleyASCIIController&) = delete;
    CopleyASCIIController(CopleyASCIIController&&) = delete;
    CopleyASCIIController& operator=(const CopleyASCIIController&) = delete;
    CopleyASCIIController& operator=(CopleyASCIIController&&) = delete;

    static CopleyASCIIController& getInstance() {
        static CopleyASCIIController instance;
        return instance;
    }

    Error_t init(const std::string& port, int iBaudrate) {
        Error_t e;

#ifndef SIMULATE
        if (m_serialDevice.isPortOpen()) {
            LOG_DEBUG("{} already Open...", port);
            return kNoError;
        }
#endif

        e = m_serialDevice.openPort(port);
        ERROR_CHECK(e, e);

        // If fresh starting after power cycle, the baudrate is reset to 9600
        e = m_serialDevice.init(9600);
        ERROR_CHECK(e, e);

        e = m_serialDevice.sendBreak();
        ERROR_CHECK(e, e);

#ifndef SIMULATE
        LOG_INFO("Current Baudrate: {}", getBaudrate());
#endif
        e = changeBaudrate(iBaudrate);
        ERROR_CHECK(e, e);

        m_bInitialized = true;
        return kNoError;
    }

    Error_t reset() {
        if (!m_bInitialized) return kNoError;
        Error_t e;

        e = resetDrive();
        ERROR_CHECK(e, e);

        e = m_serialDevice.reset();
        ERROR_CHECK(e, e);

        m_bInitialized = false;
        return kNoError;
    }

    Error_t changeBaudrate(int iBaudrate) {
        Error_t e;
        char buffer[COPLEY_BUFFER_SIZE];
        char response[COPLEY_BUFFER_SIZE];
        sprintf(buffer, "s r0x90 %d\r", iBaudrate);

        int n;
        e = sendMessage(buffer, nullptr, true);
        ERROR_CHECK(e, e);

        // There should also be a delay of 100 mS minimum before characters at the new baud rate are sent to the drive
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        e = m_serialDevice.init(iBaudrate);
        ERROR_CHECK(e, e);

#ifndef SIMULATE
        LOG_INFO("New Baudrate: {}", getBaudrate());
#endif
        return kNoError;
    }

    int getBaudrate() {
        return get(-1, 0x90, true, true);
    }

    Error_t setTarget(int axis, int position, int v_max, int accel, int decel) {
        LOG_INFO("axis: {} \t pos: {} \t vmax: {} \t acc: {}", axis, position, v_max, accel);

        Error_t e;
        Values& v = m_lastValues[axis];

        bool bChanged = false;

        if (v.position != position) {
            e = set(axis, 0xca, position);
            ERROR_CHECK(e, e);
            v.position = position;
            bChanged = true;
        }

        if (v.vel != v_max) {
            // Set maximum velocity to vMax counts/second.
            e = set(axis, 0xcb, v_max);
            ERROR_CHECK(e, e);
            v.vel = v_max;
            bChanged = true;
        }

        if (v.accel != accel) {
            // Set maximum acceleration to accel counts/second2.
            e = set(axis, 0xcc, accel);
            ERROR_CHECK(e, e);
            v.accel = accel;
            bChanged = true;
        }

        if (v.decel != decel) {
            // Set maximum deceleration to accel counts/second2.
            e = set(axis, 0xcd, decel);
            ERROR_CHECK(e, e);
            v.decel = decel;
            bChanged = true;
        }

        if (v.enable != 21) {
            // Enable the amplifier in Programmed Position (Trajectory Generator) Mode.
            e = set(axis, 0x24, 21);
            ERROR_CHECK(e, e);
            v.enable = 21;
            bChanged = true;
        }

        if (v.mode != 0) {
            e = set(axis, 0xc8, 0);
            ERROR_CHECK(e, e);
            v.mode = 0;
            bChanged = true;
        }

        if (bChanged) trajectory(axis, 1);
        return kNoError;
    }

    Error_t servoOn(int axis) {
        Values& v = m_lastValues[axis];
        Error_t e;

        if (v.enable != 21) {
            // Set the drive in Programmed Position (Trajectory Generator) Mode.
            e = set(axis, 0x24, 21);
            ERROR_CHECK(e, e);
            v.enable = 21;
        }

        return kNoError;
    }

    Error_t servoOff(int axis) {
        Values& v = m_lastValues[axis];
        Error_t e;

        if (v.enable != 0) {
            // Enable the amplifier in Programmed Position (Trajectory Generator) Mode.
            e = set(axis, 0x24, 0);
            ERROR_CHECK(e, e);
            v.enable = 0;
        }

        return kNoError;
    }

    /**
     * @param axis
     * @return -1 : don't know  0: not moving  1: moving
     */
    int isMoving(int axis) {
        int s = queryStatus(axis);
        return s == -1 ? -1 : (s & (1 << 15)) >> 15;
    }

    /**
     * @param axis
     * @return -1 : don't know  0: not home yet  1: home end
     */
    int queryHomeEnd(int axis) {
        int s = queryStatus(axis);
        return s == -1 ? -1 : (s & (1 << 12)) >> 12;
    }

    /**
     * @param axis
     * @return -1 : don't know  0: not home yet  1: home end
     */
    int queryStatus(int axis) {
        return get(axis, 0xc9);
    }

    Error_t homeHardStop(int axis) {
        return home(axis, 532);
    }

    Error_t homeNegHardStop(int axis) {
        return home(axis, 516);
    }


    Error_t homeManual(int axis) {
        return home(axis, 512);
    }

    Error_t home(int axis, int method) {
        LOG_INFO("Homing Axis {} with method {}", axis, method);
        // Sets the homing method to use the negative hard stop as home
        Error_t e;
        e = set(axis, 0xc2, method);
        ERROR_CHECK(e, e);

        // Sets the slow velocity to 80000 counts/second.
        e = set(axis, 0xc4, 400000);
        ERROR_CHECK(e, e);

        // Sets the home offset to 1000 counts.
        e = set(axis, 0xc6, 4000);
        ERROR_CHECK(e, e);

        // Set the drive in programmed position mode.
        e = set(axis, 0x24, 21);
        ERROR_CHECK(e, e);

        // Starts the homing sequence.
        return trajectory(axis, 2);
    }

private:
    struct Values {
        int position = 0;
        int vel = INT_MAX;
        int accel = INT_MAX;
        int decel = INT_MAX;
        int mode = INT_MAX;
        int enable = INT_MAX;;
    };

    bool m_bInitialized = false;
    volatile bool m_bIsSending = false;
    Values m_lastValues[NUM_HEAD_AXES];
    SerialDevice m_serialDevice;
    char m_buffer[COPLEY_BUFFER_SIZE]{};

    CopleyASCIIController() {
//        std::fill(m_buffer, &m_buffer[COPLEY_BUFFER_SIZE], '\0');
    }

    ~CopleyASCIIController() = default;

//    static char* trim(char* str) {
//        const char* spaceTypes = "\t\n\r\f\v ";
//        char* ptr = str;
//        bool incPtr = false;
//        for (int i=0; i< strlen(str); ++i) {
//            for (int j=0; j< strlen(spaceTypes); ++j) {
//                if (str[i] == spaceTypes[j]) {
//                    ptr++;
//                    incPtr = true;
//                }
//            }
//
//            if (!incPtr) return ptr;
//            incPtr = false;
//        }
//        return ptr;
//    }

    // Trim to the first integer value in string
    static char* trim(char* str) {
        for (int i = 0; i<strlen(str); ++i) {
            if (isdigit(str[i])) return &str[i];
        }
        return nullptr;
    }

    int get(int axis, int var, bool ram = true, bool ignoreInit= false) {
        if (axis < 0) {
            if (ram) sprintf(m_buffer, "g r0x%02x\r", var);
            else sprintf(m_buffer, "g f0x%02x\r", var);
        } else {
            sprintf(m_buffer, "%d g r0x%02x\r", axis, var);
        }
        char response[COPLEY_BUFFER_SIZE];

        Error_t e;
        auto cmd = std::string(m_buffer);
        e = command(cmd, response, ignoreInit);
        if (e != kNoError) return -1 * e;

        char* trimmed = trim(response);
        return (int) strtol(trimmed, nullptr, 10);
    }

    Error_t set(int axis, int var, int value) {
        sprintf(m_buffer, "%d s r0x%02x %d\r", axis, var, value);
        char response[COPLEY_BUFFER_SIZE];

        Error_t e;
        e = command(std::string(m_buffer), response);
        ERROR_CHECK(e, e);

//        std::cout << "axis: " << axis << "\t response: " << response << std::endl;

        if (strcmp(response, "ok\r") != 0) return kSetValueError;
        return kNoError;
    }

    Error_t resetDrive() {
        return sendMessage("r\r", nullptr, true);
    }

    Error_t trajectory(int axis, int kind) {
        sprintf(m_buffer, "%d t %d\r", axis, kind);
        char response[COPLEY_BUFFER_SIZE];

        Error_t e;
        e = command(std::string(m_buffer), response);
        ERROR_CHECK(e, e);

        if (strcmp(response, "ok\r") != 0) return kSetValueError;
        return kNoError;
    }

    Error_t command(const std::string& cmd, char* pReturnMsg = nullptr, bool ignoreInit=false) {
        Error_t e;
        e = sendMessage(cmd, pReturnMsg, ignoreInit);
        ERROR_CHECK(e, e);

        if (pReturnMsg && pReturnMsg[0] == 'e') {
            int errCode = (int)strtol(&pReturnMsg[2], nullptr, 10);
            LOG_ERROR("Copley Error code: {}", errCode);
            return kUnknownError;
        }

        return kNoError;
    }

    Error_t sendMessage(const std::string& msg, char* returnMsg= nullptr, bool ignoreInit = false) {
        if (!ignoreInit && !m_bInitialized) return kNotInitializedError;
        if (m_bIsSending) {
            if (returnMsg) returnMsg = (char*)"block\r";
            return kNoError;
        }

        m_bIsSending = true;

        if (!m_serialDevice.isInitialized()) return kNotInitializedError;

        int n;
        Error_t e;
        e = m_serialDevice.write(msg);
        ERROR_CHECK(e, e);

        if (returnMsg) {
            e = readResponse(&m_buffer[0], n);
            ERROR_CHECK(e, e);

            m_buffer[n] = '\0';
            sprintf(returnMsg, "%s", m_buffer);
        }

        m_bIsSending = false;
        return kNoError;
    }

    Error_t readResponse(char* line, int& n) {
#ifdef SIMULATE
        sprintf(line, "ok\r");
        n = 32;
#else
        n = m_serialDevice.readline(line, '\r', 32, PATIENCE);
#endif
        if (n == 0) {
            LOG_WARN("timeout reached...");
            return kTimeoutError;
        } else if (n < 0) {
            return kReadError;
        }
        return kNoError;
    }
};

#endif //SHIMONCONTROLLER_COPLEYASCIICONTROLLER_CUH
