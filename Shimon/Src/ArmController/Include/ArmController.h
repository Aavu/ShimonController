//
// Created by Raghavasimhan Sankaranarayanan on 4/9/22.
//

#ifndef SHIMONCONTROLLER_ARMCONTROLLER_H
#define SHIMONCONTROLLER_ARMCONTROLLER_H

#include <iostream>
#include <fstream>
#include <climits>
#include <bitset>
#include <chrono>
#include <thread>
#include <future>
#include <thread>
#include <queue>
#include <list>

#include "Arm.h"
#include "StrikerController.h"
#include "OscListener.h"
#include "OscTransmitter.h"
#include "SliderModel.h"
#include "ArmCommands.h"
#include "CommandManager.h"
#include "IAIController.h"
#include "Modbus.h"
#include "SerialDevice.h"

#include "Def.h"
#include "ErrorDef.h"
#include "StatusDef.h"
#include "Logger.h"

// Only for Debugging
#include <csignal>
#include <utility>

using namespace std::chrono;

#define NUM_ARM_THREADS 1

class ArmController {
public:
    ArmController(OscListener& oscListener, tp programStartTime = steady_clock::now());
    ~ArmController();
    Error_t init();

    Error_t reset();
    Error_t resetArms();
    Error_t start();
    Error_t stop();

    Error_t servosOn(bool bTurnOn = true);
    Error_t home();

    Error_t clearFault();

    void setStatusCallback(std::function<void(Status_t)> callback_fn) { m_statusCallback = std::move(callback_fn); }
    void setPositionCallback(std::function<void(std::array<int, NUM_ARMS>)> callback_fn) {
        m_positionCallback = std::move(callback_fn);
    }

private:
    bool m_bInitialized = false;
    const int kInitialArmPosition[NUM_ARMS] = {0, 50, 1345, 1385};
    const int kW[NUM_ARMS] = {1, 1, -1, -1};
    const int kB[NUM_ARMS] = {0, -40, 1350, 1385};
    volatile std::atomic<bool> m_bRunning = false;
    std::array<Arm*, NUM_ARMS> m_pArms{};

    long long m_iNoteCounter = 0;

    OscListener& m_oscListener;
    CommandManager<Arm::Message_t, Port::Arm> m_cmdManager;
    StrikerController m_strikerController;

    std::function<void(Status_t)> m_statusCallback = nullptr;
    std::function<void(std::array<int, NUM_ARMS>)> m_positionCallback = nullptr;

    std::unique_ptr<std::thread> m_pStatusQueryThread = nullptr;

    std::mutex m_mtx, m_oscMtx;
    std::condition_variable m_cv;

    std::mutex m_newCmdMtx;
    std::condition_variable m_newCmdCv;

    IAIController& m_IAIController;

    // For debugging
    std::ofstream m_debugLog;
    tp kProgramStartTime;
    //

    void strikerChoreoCallback(uint8_t idCode, int target, int time_ms);
    void armMidiCallback(bool bNoteON, int note, int velocity);
    void armServoCallback(const char* cmd);
    void armCallback(int armId, int position, float acceleration, float v_max);

    Error_t updateArmPositionsToMaster(int interval_ms = -1);
    void statusQueryHandler();

    static int midiToPosition(int note);

    /*
     * Check for interference for a given arm.
     * Returns 0 if the position is possible to achieve, 1 if right side has interference, -1 if left side has interference
     * It also returns the position that the interfering arm needs to move in order to make the requested move possible
     */
    int checkInterference(int armId, int position, int direction, int& ret);

    Error_t planPath(Arm::Message_t& msg);
};


#endif //SHIMONCONTROLLER_ARMCONTROLLER_H
