//
// Created by Raghavasimhan Sankaranarayanan on 5/2/22.
//

#ifndef SHIMONCONTROLLER_CIRCULARQUEUE_H
#define SHIMONCONTROLLER_CIRCULARQUEUE_H

#include <iostream>

template<class T>
class CircularQueue {
public:
    explicit CircularQueue(size_t bufferSize):
            m_iSize((int)bufferSize),
            m_pQueue(new T[m_iSize]),
            m_iHead(0), m_iTail(0), m_iNumValues(0)
    {

    }

    ~CircularQueue() {
        delete[] m_pQueue;
        m_pQueue = nullptr;
    }

    bool push(T payload) {
        if (isFull()) std::cout << "Warning: Queue full. Overwriting old values.\n";
        m_pQueue[m_iTail] = std::move(payload);
        m_iTail = (m_iTail + 1) % m_iSize;
        m_iNumValues = (m_iNumValues + 1) % (m_iSize + 1);

        return true;
    }

    bool pop(T& ret) {
        if (isEmpty()) return false;
        ret = std::move(m_pQueue[m_iHead]);
        m_iHead = (m_iHead + 1) % m_iSize;
        m_iNumValues--;
        return true;
    }

    bool peek(T& ret) {
        if (isEmpty()) return false;
        ret = std::copy(m_pQueue[m_iHead]);
        return true;
    }

    bool reset() {
        m_iNumValues = 0;
        m_iHead = 0;
        m_iTail = 0;

        return true;
    }

    [[nodiscard]] bool isEmpty() const { return m_iNumValues == 0; }
    [[nodiscard]] bool isFull() const { return m_iNumValues == m_iSize; }

    [[nodiscard]] size_t numPacketsInQueue() const { return m_iNumValues; }

private:
    int m_iSize;
    T* m_pQueue;

    int m_iHead, m_iTail, m_iNumValues;
};

#endif //SHIMONCONTROLLER_CIRCULARQUEUE_H
