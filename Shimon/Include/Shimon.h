//
// Created by Raghavasimhan Sankaranarayanan on 4/12/22.
//

#ifndef SHIMONCONTROLLER_SHIMON_H
#define SHIMONCONTROLLER_SHIMON_H

#include <iostream>
#include "ArmController.h"
#include "HeadController.h"
#include "OscListener.h"
#include "Def.h"
#include "ErrorDef.h"
#include "Logger.h"

class Shimon {
public:
    Shimon(int port, const std::string& hmConfigFile, size_t cmdBufferSize);
    Error_t initArms(const std::string& strikerHost, int strikerPort, const std::string& devArm, int iArmBaudrate);
    Error_t initHead();
    void start();
    void stop();

private:
    bool m_bArmInitialized = false;
    bool m_bHeadInitialized = false;
    OscListener m_oscListener;
    HeadController m_headController;
    ArmController m_armController;

    void sysMsgCallback(const char* msg);
};


#endif //SHIMONCONTROLLER_SHIMON_H
