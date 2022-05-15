//
// Created by Raghavasimhan Sankaranarayanan on 5/3/22.
//

#ifndef SHIMONCONTROLLER_HEADBANGGESTURE_H
#define SHIMONCONTROLLER_HEADBANGGESTURE_H

#include <iostream>
#include <future>
#include <array>

#include "Gesture.h"
#include "Scheduler.h"

class HeadBangGesture : public Gesture {
public:
    explicit HeadBangGesture(CommandManager<HeadCmdPacket_t, Port::Head>& cmdManager) : Gesture(cmdManager) {}

    ~HeadBangGesture() { stop(); }

    void beat(int length) {
        HeadCmdPacket_t cmdPkt(HeadCommand::Neck, kNeckPosition + .2, 4, 0.5);
        m_cmdManager.push(Port::Head::Copley, std::move(cmdPkt));

        while (length < 500) length *= 2;

        // Down
        m_futures[0] = m_scheduler.schedule([this] {
            HeadCmdPacket_t cmdPkt(HeadCommand::Neck, kNeckPosition - .2, 13, 0.5);
            m_cmdManager.push(Port::Head::Copley, std::move(cmdPkt));
        }, Scheduler::Delay_ms(length / 2));

        // Up
        m_futures[1] = m_scheduler.schedule([this] {
            HeadCmdPacket_t cmdPkt(HeadCommand::HeadTilt, -.1, 30, 0.5);
            m_cmdManager.push(Port::Head::Copley, std::move(cmdPkt));
        }, Scheduler::Delay_ms(length / 5));

        // Down
        m_futures[2] = m_scheduler.schedule([this] {
            HeadCmdPacket_t cmdPkt(HeadCommand::HeadTilt, -.4, 50, 1);
            m_cmdManager.push(Port::Head::Copley, std::move(cmdPkt));
        }, Scheduler::Delay_ms((length / 2) + (length / 5)));
    }

    void stop() final {
        m_scheduler.stop();
        HeadCmdPacket_t cmdPkt(HeadCommand::Neck, kNeckPosition);
        m_cmdManager.push(Port::Head::Copley, std::move(cmdPkt));
    }

private:
    const float kNeckPosition = 0.1f;

    Scheduler m_scheduler;
    std::array<std::future<void>, 3> m_futures;
};

#endif //SHIMONCONTROLLER_HEADBANGGESTURE_H
