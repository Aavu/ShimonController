//
// Created by Raghavasimhan Sankaranarayanan on 5/2/22.
//

#ifndef SHIMONCONTROLLER_BREATHEGESTURE_H
#define SHIMONCONTROLLER_BREATHEGESTURE_H

#include <iostream>
#include <future>
#include <array>

#include "Gesture.h"
#include "Scheduler.h"

class BreatheGesture : public Gesture {
public:
    explicit BreatheGesture(CommandManager<HeadCmdPacket_t, Port::Head>& cmdManager) : Gesture(cmdManager) {}

    ~BreatheGesture() {
        stop();
    }

    void start() final {
        {
            HeadCmdPacket_t cmdPkt(HeadCommand::Neck, kNeckPosition, 6.f);
            m_cmdManager.push(Port::Head::Copley, std::move(cmdPkt));
        }
        {
            HeadCmdPacket_t cmdPkt(HeadCommand::HeadTilt, 0, 30.f);
            m_cmdManager.push(Port::Head::Copley, std::move(cmdPkt));
        }

        auto neckUp = false;
        auto headUp = false;

        m_futures[0] = m_scheduler.schedule([this, &neckUp] {
            std::cout << "neck up: " << neckUp << std::endl;
            HeadCmdPacket_t cmdPkt(HeadCommand::Neck, kNeckPosition + ((float)(!neckUp) * .3f), 1.f, 0.05f);
            m_cmdManager.push(Port::Head::Copley, std::move(cmdPkt));
            neckUp ^= 1;
        }, Scheduler::Delay_ms(int(0.8*delay_ms)), Scheduler::Period_ms(delay_ms));

        m_futures[1] = m_scheduler.schedule([this, &headUp] {
            HeadCmdPacket_t cmdPkt(HeadCommand::HeadTilt, kHeadTiltPosition - (headUp ? .5f : .3f), (headUp ? 13 : 1), 0.1f);
            m_cmdManager.push(Port::Head::Copley, std::move(cmdPkt));
            headUp ^= 1;
        }, Scheduler::Delay_ms(0), Scheduler::Period_ms(delay_ms));
    }

    void stop() final {
        m_scheduler.stop();
    }

private:
    int delay_ms = 2000;
    const float kNeckPosition = 0;
    const float kHeadTiltPosition = 0;
    Scheduler m_scheduler;
    std::array<std::future<void>, 2> m_futures;
};

#endif //SHIMONCONTROLLER_BREATHEGESTURE_H
