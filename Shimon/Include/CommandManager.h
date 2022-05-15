//
// Created by Raghavasimhan Sankaranarayanan on 5/2/22.
//

#ifndef SHIMONCONTROLLER_COMMANDMANAGER_H
#define SHIMONCONTROLLER_COMMANDMANAGER_H

#include <iostream>
#include <vector>
#include <mutex>
#include <unordered_map>
#include <type_traits>

#include "Def.h"
#include "CircularQueue.h"

template<class T, class P>
class CommandManager {
public:
    explicit CommandManager(size_t bufferSize, std::condition_variable& cv) : m_cv(cv) {
        for (int i=0; i<(int)Port::kNumPorts; ++i)
            m_portQueueMap[(P)i] = new CircularQueue<T>(bufferSize);
    }

    ~CommandManager() {
        for (auto& element : m_portQueueMap) {
            delete element.second;
            element.second = nullptr;
        }
    }

    bool push(P port, T cmdPacket, bool notifyAll = true) {
        if (!m_portQueueMap[port]->push(std::move(cmdPacket))) return false;
        if (notifyAll) m_cv.notify_all();
        return true;
    }

    bool pop(P port, T& pkt, bool notifyAll = true) {
        if (!m_portQueueMap[port]->pop(pkt)) return false;
        if (notifyAll) m_cv.notify_all();
        return true;
    }

    [[nodiscard]] bool isEmpty(P port = (P)Port::kNumPorts) {
        if (port == (P)Port::kNumPorts) {
            for (auto& element : m_portQueueMap)
                if (!element.second->isEmpty()) return false;
            return true;
        }

        return m_portQueueMap[port]->isEmpty();
    }

    int getNumCommandsInQueue(P port) {
        return m_portQueueMap[port]->numPacketsInQueue();
    }

private:
    std::unordered_map<P, CircularQueue<T>*> m_portQueueMap;
    std::condition_variable& m_cv;
};

#endif //SHIMONCONTROLLER_COMMANDMANAGER_H
