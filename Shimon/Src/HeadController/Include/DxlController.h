//
// Created by Raghavasimhan Sankaranarayanan on 4/12/22.
//

#ifndef SHIMONCONTROLLER_MOUTH_H
#define SHIMONCONTROLLER_MOUTH_H

#include <iostream>
#include "dynamixel_sdk.h"
#include "Def.h"
#include "ErrorDef.h"

#define PROTOCOL_VERSION    1.0

// Control table address
#define ADDR_MX_TORQUE_ENABLE       24
#define ADDR_MX_GOAL_POSITION       30
#define ADDR_MX_MOVING_SPEED        32
#define ADDR_MX_GOAL_ACCELERATION   73
#define ADDR_MX_RETURN_DELAY_TIME   5

#define MOUTH_ID        4
#define EYEBROW_ID      5
#define TORQUE_ENABLE   1               // Value for enabling the torque
#define TORQUE_DISABLE  0               // Value for disabling the torque
#define RETURN_TIME     0

#define COMM_SUCCESS    0                   // Communication Success result value
#define COMM_TX_FAIL    -1001               // Communication Tx Failed

class DxlController {
public:
    // Singleton Class
    DxlController(const DxlController&) = delete;
    DxlController(DxlController&&) = delete;
    DxlController& operator=(const DxlController&) = delete;
    DxlController& operator=(DxlController&&) = delete;

    static DxlController& getInstance() {
        static DxlController instance;
        return instance;
    }

    Error_t init(const std::string& port) {
#ifndef SIMULATE
        m_pPortHandler = dynamixel::PortHandler::getPortHandler(port.c_str());

        if (!m_pPortHandler->openPort()) return kFileOpenError;
        if (!m_pPortHandler->setBaudRate(DXL_BAUDRATE)) return kSetValueError;

        int dxl_comm_result;
        uint8_t dxl_error = 0;
        // Enable DYNAMIXEL Torque
        dxl_comm_result = m_pPacketHandler->write1ByteTxRx(m_pPortHandler, EYEBROW_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        checkError(dxl_comm_result, dxl_error);

        dxl_comm_result = m_pPacketHandler->write1ByteTxRx(m_pPortHandler, MOUTH_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        checkError(dxl_comm_result, dxl_error);

        dxl_comm_result = m_pPacketHandler->write1ByteTxRx(m_pPortHandler, EYEBROW_ID, ADDR_MX_RETURN_DELAY_TIME, RETURN_TIME, &dxl_error);
        checkError(dxl_comm_result, dxl_error);

        dxl_comm_result = m_pPacketHandler->write1ByteTxRx(m_pPortHandler, MOUTH_ID, ADDR_MX_RETURN_DELAY_TIME, RETURN_TIME, &dxl_error);
        checkError(dxl_comm_result, dxl_error);
#endif
        m_bInitialized = true;
        return kNoError;
    }

    Error_t reset() {
        close();
        return kNoError;
    }

    void close() {
        if (!m_bInitialized) return;
#ifndef SIMULATE
        int dxl_comm_result = COMM_TX_FAIL;
        uint8_t dxl_error = 0;
        // Disable DYNAMIXEL Torque
        dxl_comm_result = m_pPacketHandler->write1ByteTxRx(m_pPortHandler, EYEBROW_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        checkError(dxl_comm_result, dxl_error);

        dxl_comm_result = m_pPacketHandler->write1ByteTxRx(m_pPortHandler, MOUTH_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        checkError(dxl_comm_result, dxl_error);
#endif
        m_bInitialized = false;
    }

    void setPosition(int id, short pos) {
        if (!m_bInitialized) return;
#ifndef SIMULATE
        int dxl_comm_result = COMM_TX_FAIL;
        uint8_t dxl_error = 0;
        // Disable DYNAMIXEL Torque
        dxl_comm_result = m_pPacketHandler->write2ByteTxRx(m_pPortHandler, id, ADDR_MX_GOAL_POSITION, pos, &dxl_error);
        checkError(dxl_comm_result, dxl_error);
#endif
    }

    void setAccel(int id, short accel) {
        if (!m_bInitialized) return;
#ifndef SIMULATE
        int dxl_comm_result = COMM_TX_FAIL;
        uint8_t dxl_error = 0;
        // Disable DYNAMIXEL Torque
        dxl_comm_result = m_pPacketHandler->write2ByteTxRx(m_pPortHandler, id, ADDR_MX_GOAL_ACCELERATION, accel, &dxl_error);
        checkError(dxl_comm_result, dxl_error);
#endif
    }

    void setVelocity(int id, short vel) {
        if (!m_bInitialized) return;
#ifndef SIMULATE
        int dxl_comm_result = COMM_TX_FAIL;
        uint8_t dxl_error = 0;
        dxl_comm_result = m_pPacketHandler->write2ByteTxRx(m_pPortHandler, id, ADDR_MX_MOVING_SPEED, vel, &dxl_error);
        checkError(dxl_comm_result, dxl_error);
#endif
    }

    void checkError(int result, uint8_t err) {
#ifndef SIMULATE
        if (result != COMM_SUCCESS) {
            printf("%s\n", m_pPacketHandler->getTxRxResult(result));
        }
        else if (err != 0) {
            printf("%s\n", m_pPacketHandler->getRxPacketError(err));
        }
#endif
    }

    Error_t servoOn(int id) {
        int dxl_comm_result = COMM_TX_FAIL;
        uint8_t dxl_error = 0;

#ifndef SIMULATE
        dxl_comm_result = m_pPacketHandler->write1ByteTxRx(m_pPortHandler, id, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        checkError(dxl_comm_result, dxl_error);
#endif
        return kNoError;
    }

    Error_t servoOff(int id) {
        int dxl_comm_result = COMM_TX_FAIL;
        uint8_t dxl_error = 0;
#ifndef SIMULATE
        dxl_comm_result = m_pPacketHandler->write1ByteTxRx(m_pPortHandler, id, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        checkError(dxl_comm_result, dxl_error);
#endif
        return kNoError;
    }

    Error_t setTarget(int id, int position, int v_max, int accel, int decel) {
        setAccel(id, (short)accel);
        return setTarget(id, position, v_max);
    }

    Error_t setTarget(int id, int position) {
        setPosition(id, (short)position);
        return kNoError;
    }

    Error_t setTarget(int id, int position, int velocity) {
        setVelocity(id, (short)velocity);
        return setTarget(id, position);
    }

private:
    bool m_bInitialized = false;
    dynamixel::PortHandler* m_pPortHandler = nullptr;
    dynamixel::PacketHandler* m_pPacketHandler;

    DxlController() : m_pPacketHandler(dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION)) {}

};

#endif //SHIMONCONTROLLER_MOUTH_H
