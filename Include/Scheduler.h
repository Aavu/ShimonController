//
// Created by Raghavasimhan Sankaranarayanan on 5/2/22.
//

#ifndef SHIMONCONTROLLER_SCHEDULER_H
#define SHIMONCONTROLLER_SCHEDULER_H

#include <iostream>
#include <atomic>
#include <chrono>
#include <future>
#include <utility>
#include <mutex>
#include <condition_variable>

// Reference : https://codereview.stackexchange.com/questions/232791/timer-for-scheduling-tasks-in-c11
class Scheduler {
public:
    using Timestamp = std::chrono::time_point<std::chrono::system_clock,std::chrono::microseconds>;
    using Delay_ms = std::chrono::milliseconds;
    using Period_ms = std::chrono::milliseconds;

    template <typename T, typename ... Args>
    std::future<void> schedule(T task, Timestamp const& time, Args... args)  {
        using namespace std::chrono;
        return std::async(std::launch::async, [=]() {
            std::unique_lock<std::mutex> lk(m_mtx);
            m_cv.wait_until(lk, time, [this, &time] { return !(time_point_cast<Timestamp::duration>(system_clock::now()) < time) || !m_bRunning; });
            invoke(task, args...);
        });
    }

    template <typename T, typename Period, typename ... Args>
    std::future<void> schedule(T task, Timestamp const& time, Period const& period, Args... args)  {
        using namespace std::chrono;
        m_bRunning = true;
        return std::async(std::launch::async, [=]() {
            std::unique_lock<std::mutex> lk(m_mtx);
            m_cv.wait_until(lk, time, [this, &time] { return !(time_point_cast<Timestamp::duration>(system_clock::now()) < time) || !m_bRunning; });
            while (m_bRunning) {
                invoke(task, args...);
                m_cv.wait_for(lk, period, [=] { return !m_bRunning; });
            }
        });
    }

    template <typename T, typename ... Args>
    std::future<void> schedule(T task, Delay_ms const& delay, Args... args)  {
        using namespace std::chrono;
        return std::async(std::launch::async, [=]() {
            auto start = system_clock::now();
            std::unique_lock<std::mutex> lk(m_mtx);
            m_cv.wait_for(lk, delay, [this, &delay, &start] { return !(duration_cast<Delay_ms>(system_clock::now() - start) < delay) || !m_bRunning; });
            invoke(task, args...);
        });
    }

    template <typename T, typename ... Args>
    std::future<void> schedule(T task, Delay_ms const& delay, Period_ms const& period, Args... args)  {
        using namespace std::chrono;
        m_bRunning = true;
        return std::async(std::launch::async, [=]() {
            auto start = system_clock::now();
            std::unique_lock<std::mutex> lk(m_mtx);
            m_cv.wait_for(lk, delay, [=] { return !(duration_cast<Delay_ms>(system_clock::now() - start) < delay) || !m_bRunning; });
            while (m_bRunning) {
                invoke(task, args...);
                m_cv.wait_for(lk, period, [this] { return !m_bRunning; });
            }
        });
    }

    void stop() {
        m_bRunning = false;
        m_cv.notify_all();
    }

private:
    std::atomic<bool> m_bRunning;
    std::mutex m_mtx;
    std::condition_variable m_cv;

    template<typename F, typename... Args>
    inline auto invoke(F f, Args&&... args) -> decltype(std::ref(f)(std::forward<Args>(args)...)) {
        return std::ref(f)(std::forward<Args>(args)...);
    }
};

#endif //SHIMONCONTROLLER_SCHEDULER_H
