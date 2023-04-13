//
// Created by Raghavasimhan Sankaranarayanan on 4/12/22.
//

#ifndef SHIMONCONTROLLER_HEADCONTROLLER_H
#define SHIMONCONTROLLER_HEADCONTROLLER_H

#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <optional>

#include "yaml-cpp/yaml.h"
#include "HDMotor.h"
#include "Config.h"
#include "MotorController.h"
#include "Motor.h"
#include "CopleyASCIIController.h"
#include "DxlController.h"
#include "OscListener.h"
#include "Def.h"
#include "Util.h"
#include "HeadCommands.h"
#include "CommandManager.h"
#include "ErrorDef.h"
#include "Logger.h"

// Gestures
#include "Gestures/BreatheGesture.h"
#include "Gestures/HeadBangGesture.h"

class HeadController {
public:
    HeadController(OscListener& oscListener, const std::string& configFile, size_t cmdBufferSize, tp programStartTime);
    ~HeadController();
    Error_t init(bool shouldHome = true);
    Error_t reset();
    Error_t home();

    Error_t start();
    Error_t stop();

private:
    volatile std::atomic<bool> m_bRunning = false;
    bool m_bInitialized = false;
    std::unique_ptr<std::thread> m_pThread = nullptr;
    std::mutex m_mtx;
    std::condition_variable m_cv;
    const tp m_kProgramStartTime;

    OscListener& m_oscListener;
    MotorController m_motorController;
    CommandManager<HeadCmdPacket_t, Port::Head> m_cmdManager;

    BreatheGesture m_breatheGesture;
    HeadBangGesture m_headBangGesture;

    void headCallback(const char* cmd, std::optional<float> val1, std::optional<float> val2);
    void threadHandler();
    void handleHighLevelCommands(HeadCommand cmd, std::optional<float> val);
};


#endif //SHIMONCONTROLLER_HEADCONTROLLER_H
