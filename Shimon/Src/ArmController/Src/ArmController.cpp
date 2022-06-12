//
// Created by Raghavasimhan Sankaranarayanan on 4/9/22.
//

#include "ArmController.h"
#include "Util.h"

ArmController::ArmController(OscListener& oscListener, size_t cmdBufferSize) : m_oscListener(oscListener),
                                                                                m_cmdManager(cmdBufferSize, m_cv),
                                                                                m_strikerController(cmdBufferSize),
                                                                                m_IAIController(IAIController::getInstance())
{
    for (int i = 0; i< NUM_ARMS; ++i) {
        m_pArms[i] = new Arm(i, kInitialArmPosition[i], kW[i], kB[i]);
    }

    ArmCommandUtil::populateStrCmdMap();
}

ArmController::~ArmController() {
    for (int i = 0; i< NUM_ARMS; ++i) {
        delete m_pArms[i];
        m_pArms[i] = nullptr;
    }

    reset();
}

Error_t ArmController::init() {
    Error_t e;

    e = m_IAIController.init(IAI_ACTUATOR, IAI_BAUDRATE);
    ERROR_CHECK(e, e);

    LOG_TRACE("Setting setArmServoCallback");
    m_oscListener.setArmServoCallback([this](auto &&PH1) {
        armServoCallback(std::forward<decltype(PH1)>(PH1));
    });

    LOG_TRACE("Setting setArmMidiCallback");
    m_oscListener.setArmMidiCallback([this](auto &&PH1, auto &&PH2) {
        armMidiCallback(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2));
    });

    LOG_TRACE("Setting setArmRawCallback");
    m_oscListener.setArmRawCallback([this](auto &&PH1, auto &&PH2, auto &&PH3, auto &&PH4) {
        armCallback(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2), std::forward<decltype(PH3)>(PH3), std::forward<decltype(PH4)>(PH4));
    });


    e = m_strikerController.init(STRIKER_HOST, STRIKER_PORT);
    ERROR_CHECK(e, e);

    m_bInitialized = true;

    return kNoError;
}

Error_t ArmController::initMasterTransmitter(const std::string& host, int port) {
    return kNoError; //m_oscTransmitter.init(host, port);
}

Error_t ArmController::reset() {
    return m_strikerController.reset();
}

void ArmController::armMidiCallback(int note, int velocity) {

    std::lock_guard<std::mutex> lk(m_oscMtx);
    Message_t msg{};
    msg.midiVelocity = velocity;
    msg.midiNote = note;

    Error_t e;
    e = prepareToPlay(note, &msg, false);

    if (e == kNoError) {
        // Arm
        m_cmdManager.push(Port::Arm::IAI, msg);
    }
}

Error_t ArmController::start() {
    if (!m_bInitialized) return kNotInitializedError;
    m_bRunning = true;
//    m_pThread = std::make_unique<std::thread>([this] {threadHandler();});
    for (auto & thread : m_pThreadPool) {
        thread = std::make_unique<std::thread>([this] { threadPoolHandler(); });
    }
    m_pStatusQueryThread = std::make_unique<std::thread>([this] {statusQueryHandler();});
//    m_pMasterTransmitThread = new std::thread([this]() { masterTransmitHandler(); })

    return kNoError;
}

Error_t ArmController::stop() {
    if (!m_bInitialized) return kNotInitializedError;
    m_bRunning = false;
    m_cv.notify_all();
    m_newCmdCv.notify_all();

    for (auto & thread : m_pThreadPool)
        if (thread)
            if (thread->joinable())
                thread->join();
    if (m_pStatusQueryThread->joinable()) m_pStatusQueryThread->join();

//    if (m_pMasterTransmitThread) m_pMasterTransmitThread->join();
    return kNoError;
}

Error_t ArmController::prepareToPlay(int note, ArmController::Message_t *msg, bool bMoveInterferingArm) {
    auto timeNow = steady_clock::now();
    auto originalNote = note;
    for (int o = 0; o < NUM_OCTAVES_TO_TRY; ++o) {
        note = Util::transpose(originalNote, kOctavesToTry[o]*12);
        int target = midiToPosition(note);
        std::list<Message_t> c_msg;

        for (int i=0; i<m_pArms.size(); ++i) {
            Arm* pArm = m_pArms[i];
            auto t = std::max(pArm->getArrivalTime(), timeNow);
            if (pArm->getPosition() == target) {
                LOG_INFO("Arm {} already at {}", i, target);
                msg->arm_id = i;
                msg->acceleration = 0;
                msg->v_max = 0;
                msg->target = pArm->getPosition();
                pArm->setMsgTime(timeNow);
                return kNoError;
            }

            float diffTime = (float)duration_cast<microseconds>(timeNow - t).count() / 1000.f;
            int time_ms = (int)std::max(1.f, diffTime + DLY);

//            LOG_INFO("arm {}, time {} {}", i, diffTime, time_ms);
            float temp0, temp1;
            Error_t e;
            e = pArm->computeMotionParam(target, (float)time_ms, &temp0, &temp1);
            int dir = (target > pArm->getPosition()) ? 1 : -1;

            int dummy;
            // returns 0 if there is no interference, 1 if interference is to the right and -1 if interference is to the left
            int interference = checkInterference(i, target, dir, dummy);
            if (interference != 0) continue;
//            LOG_INFO("time: {}", diffTime);
            if (e == kNoError) {    // Motion possible
                Message_t m = {
                        .arm_id = i,
                        .target = target,
                        .acceleration = temp1,
                        .v_max = temp0,
                        .time_ms = time_ms
                };
                c_msg.push_back(m);
            }
        }

        c_msg.sort([](const Message_t& m1, const Message_t& m2) {
            return m1.acceleration < m2.acceleration;
        });

//        if (bMoveInterferingArm) {
//            for (auto& m: c_msg) {
//                Error_t e = moveInterferingArms(m.arm_id, m.target);
//                if (e != kNoError) break;
//
//                *msg = m;
//                return kNoError;
//            }
//        } else {
        if (!c_msg.empty()) {
            auto& m = c_msg.front();
            msg->acceleration = m.acceleration;
            msg->v_max = m.v_max;
            msg->arm_id = m.arm_id;
            msg->midiVelocity = m.midiVelocity;
            msg->target = m.target;
            msg->time_ms = m.time_ms;
            m_pArms[m.arm_id]->updateTrajectory((int) m.time_ms, m.target, m.acceleration, m.v_max);
            m_pArms[m.arm_id]->setMsgTime(timeNow);
//            auto diff = duration_cast<milliseconds>(msg->arrivalTime - msg->msgTime).count();
//            LOG_INFO("Arm {}, Time {} {} {}", m.arm_id, diff, duration_cast<milliseconds>(msg->msgTime.time_since_epoch()).count(), duration_cast<milliseconds>(msg->arrivalTime.time_since_epoch()).count());
            return kNoError;
        }
//        }
        LOG_INFO("{} octave not possible.", kOctavesToTry[o]);
    }

    LOG_WARN("Impossible to moveArm to the note...");
    return kImpossibleError;
}

Error_t ArmController::moveInterferingArms(int armId, int toPos) {
    int dir = (toPos > m_pArms[armId]->getPosition()) ? 1 : -1;
    std::list<int> pos;
    return moveInterferingArm_rec(armId, toPos, dir, armId, pos);
}

Error_t ArmController::moveInterferingArm_rec(int armId, int toPos, int direction, int initialArmId, std::list<int>& positions) {
    Error_t e;
    int pos;
    int interferingArm = checkInterference(armId, toPos, direction, pos);
    if (interferingArm == 0) {
        if (armId < initialArmId)
            for (int i=armId; i<initialArmId; ++i) {
                e = m_pArms[i]->move(positions.back(), DLY *.5);
                if (e != kNoError) return kImpossibleError;
//                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                positions.pop_back();
            }
        else
            for(int i=armId; i > initialArmId; --i) {
                e = m_pArms[i]->move(positions.back(), DLY *.5);
                if (e != kNoError) return kImpossibleError;
//                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                positions.pop_back();
            }
        return kNoError;
    }
    if (pos == -10000) return kImpossibleError;

    positions.push_back(pos);

    return moveInterferingArm_rec(armId + direction, pos, direction, initialArmId, positions);
}

int ArmController::checkInterference(int armId, int target, int direction, int& ret) {
    if (armId < 0 || armId >= NUM_ARMS) {
        ret = -10000;
        return 0;
    }

    if (direction < 0) {
        int l_idx = armId - 1;

        if (l_idx < 0) {
            ret = -10000;
            return 0;
        }

        // after motion complete
        int leftBoundary = m_pArms[l_idx]->getRightBoundary();
        ret = target - kBoundaries[l_idx][1];
        if (ret < 0) ret = -10000;
        if (target <= leftBoundary) return -1;

        // TODO: Need to calculate the boundary using at the time when the current arm would reach this position and not from the current time
        // For arm in motion

    } else {
        int r_idx = armId + 1;

        if (r_idx > NUM_ARMS) {
            ret = -10000;
            return 0;
        }

        // after motion complete
        int rightBoundary = m_pArms[r_idx]->getLeftBoundary();
        ret = target + kBoundaries[r_idx][0];
        if (ret > SLIDER_LIMIT) ret = -10000;
        if (target >= rightBoundary) return 1;

        // For arm in motion
//        rightBoundary = m_pArms[r_idx]->getLeftBoundary();
    }

    return 0;
}

int ArmController::midiToPosition(int note) {
    int position = std::min(MAX_NOTE - MIN_NOTE, std::max(0, note - MIN_NOTE));
    position = std::min(position, SLIDER_LIMIT);
    return kNotePositionTable[position];
}

void ArmController::armCallback(int armId, int position, float acceleration, float v_max) {
    m_pArms[armId]->move(position, acceleration, v_max);
}

void ArmController::armServoCallback(const char *sCmd) {
    auto cmd = ArmCommandUtil::getCommand(sCmd);

    Error_t e;
    switch (cmd) {
        case ArmCommand::Home:
            LOG_INFO("Homing...");
            e = home();
            break;
        case ArmCommand::Off:
            LOG_INFO("Servos off...");
            e = servosOn(false);
            break;
        case ArmCommand::On:
            LOG_INFO("Servos on...");
            e = servosOn(true);
            break;
        case ArmCommand::AlarmClear:
            LOG_INFO("Alarm Clear...");
            e = clearFault();
            break;
        case ArmCommand::Reset:
            LOG_INFO("Restart Strikers...");
            e = m_strikerController.restart();
            break;
        default:
            LOG_WARN("Unknown Servo Command : {}", sCmd);
            e = kUnknownCaseError;
            break;
    }

    if (e != kNoError) {
        LOG_ERROR("Error Code: {}", e);
    }
}

void ArmController::strikerCallBack(char type, uint8_t strikerIds, int dummy, int position, int acc) {
//    m_future = std::async(std::launch::async, [this, type, strikerIds, position, acc] () {
//        std::this_thread::sleep_for( std::chrono::milliseconds(DLY));
//        m_strikerController.strike(strikerIds, (float)position, (float)acc, StrikerController::getMode(type));
//    });
}

void ArmController::strikerMidiCallback(char type, uint8_t strikerIds, uint8_t midiVelocity) {
//    m_future = std::async(std::launch::async, [this, strikerIds, type, midiVelocity] () {
//        std::this_thread::sleep_for( std::chrono::milliseconds(DLY));
//        m_strikerController.strike(strikerIds, midiVelocity, StrikerController::getMode(type));
//    });
}

void ArmController::masterTransmitHandler() {
//    while (m_bRunning) {
//        bool isEmpty = false;
//        {
//            std::lock_guard<std::mutex> lk(m_mtx);
//            isEmpty = m_transmissionQueue.empty();
//        }
//        if (isEmpty) {
//            sleep(100);
//            continue;
//        }
//
//        float fVal;
//        {
//            std::lock_guard<std::mutex> lk(m_mtx);
//            fVal = m_transmissionQueue.front();
//            m_transmissionQueue.pop();
//        }
//
//
//    }
}

void ArmController::threadPoolHandler() {
    while (true) {
        Message_t msg{};
        int id;
        _tp now, msgTime, arrivalTime;
        bool success = false;
        {
            std::unique_lock<std::mutex> lk(m_mtx);
            m_cv.wait(lk, [this] { return !m_cmdManager.isEmpty() || !m_bRunning; });
            if (!m_bRunning) break;
            success = m_cmdManager.pop(Port::Arm::IAI, msg);
            id = msg.arm_id;
            msgTime = m_pArms[id]->getMsgTime();
            arrivalTime = m_pArms[id]->getArrivalTime();
//            LOG_INFO("Elapsed {} ms", duration_cast<milliseconds>(std::chrono::steady_clock::now() - now).count());
        }

        if (success) {
            now = std::chrono::steady_clock::now();
            LOG_DEBUG("Sleeping for {} ms", duration_cast<milliseconds>(msgTime - now).count());
            std::this_thread::sleep_until(msgTime);
            m_newCmdCv.notify_all();
            Error_t e = m_pArms[id]->move(msg.target, msg.acceleration, msg.v_max);
            if (e != kNoError) LOG_ERROR("Move Error Code: {}", e);

            LOG_DEBUG("Arrival Time: {} ms", duration_cast<milliseconds>(arrivalTime - now).count() - 50);
            std::this_thread::sleep_until(arrivalTime - milliseconds(50));
            e = m_strikerController.strike(msg.midiNote, id, msg.midiVelocity);
            if (e != kNoError) LOG_ERROR("Error Code: {}", e);
        }
    }
}

void ArmController::statusQueryHandler() {
    const int interval_ms = 1;
    int i = 0;
    while (m_bRunning) {
        std::unique_lock<std::mutex> lk(m_newCmdMtx);
        m_newCmdCv.wait(lk, [this] { return !m_bRunning || m_IAIController.isHomed();});
        if (!m_bRunning) break;
        auto now = std::chrono::steady_clock::now();
        for (auto *pArm: m_pArms) {
            pArm->updatePositionFromTrajectory(interval_ms);
//            Error_t e = pArm->updateStatusFromDevice();
//            if (e != kNoError) {

//            }
//            if (e != kNoError) {
//                LOG_ERROR("Error updating position for Arm: {}", pArm->getID());
//            }
        }
        std::this_thread::sleep_until(now + std::chrono::milliseconds(interval_ms));
    }
}

Error_t ArmController::home() {
    Error_t e;

    e = servosOn(true);
    ERROR_CHECK(e, e);

    return m_IAIController.home();
}

Error_t ArmController::servosOn(bool bTurnOn) {
    Error_t e;

    e = clearFault();
    ERROR_CHECK(e, e);

    // Set id to -1 to set all axes at once
    e = m_IAIController.setServo(-1, bTurnOn);
    return e;
}

Error_t ArmController::clearFault() {
    return m_IAIController.clearFault();
}
