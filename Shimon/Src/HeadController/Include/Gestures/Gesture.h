//
// Created by Raghavasimhan Sankaranarayanan on 5/2/22.
//

#ifndef SHIMONCONTROLLER_GESTURE_H
#define SHIMONCONTROLLER_GESTURE_H

#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>

#include "CommandManager.h"
#include "HeadCommands.h"

class Gesture {
public:
    explicit Gesture(CommandManager<HeadCmdPacket_t, Port::Head>& cmdManager) : m_cmdManager(cmdManager) {}
    virtual void start() {}
    virtual void stop() {};

protected:
    CommandManager<HeadCmdPacket_t, Port::Head>& m_cmdManager;
};

#endif //SHIMONCONTROLLER_GESTURE_H
