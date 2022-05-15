//
// Created by Raghavasimhan Sankaranarayanan on 4/9/22.
//

#ifndef SHIMONCONTROLLER_ARMCONTROLLER_H
#define SHIMONCONTROLLER_ARMCONTROLLER_H

#include <iostream>
#include <climits>
#include <chrono>
#include <thread>
#include <future>
#include <thread>
#include <queue>
#include <list>

#include "Arm.h"
#include "StrikerController.h"
#include "OscListener.h"
#include "Trajectory.h"
#include "ArmCommands.h"
#include "CommandManager.h"
//#include "OscTransmitter.h"
#include "IAIController.h"
#include "Modbus.h"
#include "SerialDevice.h"

#include "Def.h"
#include "ErrorDef.h"
#include "Logger.h"

using namespace std::chrono;

class ArmController {
public:
    struct Message_t {
        int arm_id;
        int target;
        float acceleration;
        float v_max;
        int midiNote;
        int midiVelocity;
        uint8_t strikerId;
        float time_ms;

        void pprint() const {
            std::bitset<8> sId(strikerId);
            std::cout << "Id: " << arm_id
                    << "\t target: " << target
                    << "\t acc: " << acceleration
                    << "\t vmax: " << v_max
                    << "\t midi vel: " << midiVelocity
                    << "\t strikerId: " << sId
                    << "\t time (ms): " << time_ms
                    << std::endl;
        }
    };

    ArmController(OscListener& oscListener, size_t cmdBufferSize);
    ~ArmController();
    Error_t init(const std::string& strikerHost, int strikerPort, const std::string& devArm, int iArmBaudrate);
    Error_t initMasterTransmitter(const std::string& host, int port);

    Error_t reset();
    Error_t start();
    Error_t stop();

    Error_t servosOn(bool bTurnOn = true);
    Error_t home();

private:
    bool m_bInitialized = false;
    const int kInitialArmPosition[NUM_ARMS] = {0, 50, 1345, 1385};
    const int kW[NUM_ARMS] = {1, 1, -1, -1};
    const int kB[NUM_ARMS] = {0, -40, 1350, 1385};
    volatile std::atomic<bool> m_bRunning = false;
    std::array<Arm*, NUM_ARMS> m_pArms{};
    OscListener& m_oscListener;
//    OscTransmitter m_oscTransmitter;
    CommandManager<Message_t, Port::Arm> m_cmdManager;
    StrikerController m_strikerController;

    std::unique_ptr<std::thread> m_pMasterTransmitThread = nullptr;
    std::unique_ptr<std::thread> m_pThread = nullptr;
    std::unique_ptr<std::thread> m_pStrikerThread = nullptr;
    std::unique_ptr<std::thread> m_pStatusQueryThread = nullptr;

    std::mutex m_mtx;
    std::condition_variable m_cv;

    IAIController& m_IAIController;

    void armMidiCallback(int note, int velocity);
    void armServoCallback(const char* cmd);
    void armCallback(int armId, int position, float acceleration, float v_max);

    void masterTransmitHandler();

    void threadHandler();
//    void strikerThreadHandler();
    void statusQueryHandler();

    void strikerMidiCallback(char type, uint8_t strikerIds, uint8_t midiVelocity);
    void strikerCallBack(char type, uint8_t strikerIds, int dummy, int position, int acc);

    static int midiToPosition(int note);

    /*
     * Check for interference for a given arm.
     * Returns 0 if the position is possible to achieve, 1 if right side has interference, -1 if left side has interference
     * It also returns the position that the interfering arm needs to move in order to make the requested move possible
     */
    int checkInterference(int armId, int position, int direction, int& ret);

    Error_t prepareToPlay(int note, Message_t* msg=nullptr, bool bMoveInterferingArm=true);

    Error_t moveInterferingArms(int armId, int toPos);
    Error_t moveInterferingArm_rec(int armId, int toPos, int direction, int initialArmId, std::list<int>& positions);
};


#endif //SHIMONCONTROLLER_ARMCONTROLLER_H
