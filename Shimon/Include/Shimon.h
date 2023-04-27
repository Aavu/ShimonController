//
// Created by Raghavasimhan Sankaranarayanan on 4/12/22.
//

#ifndef SHIMONCONTROLLER_SHIMON_H
#define SHIMONCONTROLLER_SHIMON_H

#include <iostream>
#include <chrono>
#include "ArmController.h"
#include "HeadController.h"
#include "OscListener.h"
#include "OscTransmitter.h"
#include "CircularQueue.h"

#include "Def.h"
#include "ErrorDef.h"
#include "StatusDef.h"
#include "Logger.h"

class Shimon {
public:
    Shimon(int port, const std::string& hmConfigFile, tp programStartTime = std::chrono::steady_clock::now());
    Error_t init(const std::string& masterHostName, int masterPort);
    void start();
    void stop();

private:
    bool m_bArmInitialized = false;
    bool m_bHeadInitialized = false;
    std::atomic<bool> m_bRunning = false;
    const tp m_kProgramStartTime;
    OscTransmitter m_oscTransmitter;
    OscListener m_oscListener;
    HeadController m_headController;
    ArmController m_armController;

    std::unique_ptr<std::thread> m_pMasterTransmitThread = nullptr;
    std::mutex m_mtx;
    std::condition_variable m_cv;
    CircularQueue<std::array<int, NUM_ARMS>> m_positionTransmitQueue;

    void sysMsgCallback(const char* msg);
    void armStatusCallback(Status_t status);
    void headStatusCallback(Status_t status);

    void armPositionCallback(std::array<int, NUM_ARMS> position);

    Error_t initArms();
    Error_t initHead();
    Error_t initMasterTransmitter(const std::string& host, int port);
    void masterTransmitHandler();
};


#endif //SHIMONCONTROLLER_SHIMON_H
