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

Error_t ArmController::init(const std::string& strikerHost, int strikerPort, const std::string& devArm, int iArmBaudrate) {
    Error_t e;

    e = m_IAIController.init(devArm, iArmBaudrate);
    ERROR_CHECK(e, e);

    m_oscListener.setArmServoCallback([this](auto &&PH1) {
        armServoCallback(std::forward<decltype(PH1)>(PH1));
    });

    m_oscListener.setArmMidiCallback([this](auto &&PH1, auto &&PH2) {
        armMidiCallback(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2));
    });

    m_oscListener.setArmRawCallback([this](auto &&PH1, auto &&PH2, auto &&PH3, auto &&PH4) {
        armCallback(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2), std::forward<decltype(PH3)>(PH3), std::forward<decltype(PH4)>(PH4));
    });

    e = m_strikerController.init(strikerHost, strikerPort);
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
//    auto startTime = steady_clock::now();
//    LOG_INFO("Note: {}\t midiVel: {}", note, velocity);
    Message_t msg{};
    msg.midiVelocity = velocity;
    msg.midiNote = note;

    Error_t e;
    e = prepareToPlay(note, &msg, false);
    int armId = msg.arm_id;

    if (e == kNoError) {
        // Arm
        LOG_INFO("ArmID: {} \t Position: {} \t acc: {} \t v_max: {} \t delta time: {}", armId, msg.target, msg.acceleration, (int)std::round(msg.v_max), msg.time_ms);
        m_pArms[armId]->move(msg.target, msg.time_ms, msg.acceleration, msg.v_max);
    }

    m_strikerController.strike(note, armId,velocity);
}

Error_t ArmController::start() {
    if (!m_bInitialized) return kNotInitializedError;
    m_bRunning = true;
    m_pThread = std::make_unique<std::thread>([this] {threadHandler();});
    m_pStatusQueryThread = std::make_unique<std::thread>([this] {statusQueryHandler();});
//    m_pStrikerThread = std::make_unique<std::thread>([this] {strikerThreadHandler();});
//    m_pMasterTransmitThread = new std::thread([this]() { masterTransmitHandler(); })

    return kNoError;
}

Error_t ArmController::stop() {
    if (!m_bInitialized) return kNotInitializedError;
    m_bRunning = false;
    m_cv.notify_all();
    if (m_pThread->joinable()) m_pThread->join();
    if (m_pStatusQueryThread->joinable()) m_pStatusQueryThread->join();
//    if (m_pStrikerThread->joinable()) m_pStrikerThread->join();
//    if (m_pMasterTransmitThread) m_pMasterTransmitThread->join();
    return kNoError;
}

Error_t ArmController::prepareToPlay(int note, ArmController::Message_t *msg, bool bMoveInterferingArm) {
    auto timeNow = steady_clock::now();

    auto originalNote = note;
    for (int o = 0; o < NUM_OCTAVES_TO_TRY; ++o) {
        note = Util::transpose(originalNote, kOctavesToTry[o]*12);
        int position = midiToPosition(note);
        std::list<Message_t> c_msg;

        for (int i=0; i<m_pArms.size(); ++i) {
            Arm* pArm = m_pArms[i];
            if (pArm->getPosition() == position) {
                LOG_INFO("Arm {} already at {}", i, position);
                pArm->setLastTime(timeNow);
                return kAlreadyThereError;
            }

            float diffTime = (float)duration_cast<microseconds>(timeNow - pArm->getLastTime()).count() / 1000.f;
            float time_ms = std::min(std::max(1.f, diffTime), (float)DLY);

            float temp0, temp1;
            Error_t e;
            e = pArm->computeMotionParam(position, time_ms, &temp0, &temp1);
            int dir = (position > pArm->getPosition()) ? 1 : -1;
            int dummy;
            int possible = checkInterference(i, position, dir, dummy);
            if (possible != 0) continue;

            if (e == kNoError) {    // Motion possible
                Message_t m = {
                        .arm_id = i,
                        .target = position,
                        .acceleration = temp1,
                        .v_max = temp0,
                        .time_ms = time_ms
                };
                c_msg.push_back(std::move(m));
            }
        }

        c_msg.sort([](const Message_t& m1, const Message_t& m2) {
            return m1.acceleration < m2.acceleration;
        });

        if (bMoveInterferingArm) {
            for (auto& m: c_msg) {
                Error_t e = moveInterferingArms(m.arm_id, m.target);
                if (e != kNoError) break;

                *msg = std::move(m);
                return kNoError;
            }
        } else {
            if (!c_msg.empty()) {
                auto& m = c_msg.front();
                msg->acceleration = m.acceleration;
                msg->v_max = m.v_max;
                msg->arm_id = m.arm_id;
                msg->midiVelocity = m.midiVelocity;
                msg->target = m.target;
                msg->time_ms = m.time_ms;
                return kNoError;
            }

        }


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

        int leftBoundary = (l_idx >= 0) ? (m_pArms[l_idx]->getRightBoundary()) : -1;

        ret = std::max(0, target - ((l_idx < 0) ? 0 : kBoundaries[l_idx][1]));
        if (ret < 0) ret = -10000;
        if (target <= leftBoundary) return -1;
    } else {
        int r_idx = armId + 1;

        int rightBoundary = (r_idx < NUM_ARMS) ? (m_pArms[r_idx]->getLeftBoundary()) : SLIDER_LIMIT + 1;

        ret = target + ((r_idx < NUM_ARMS) ? kBoundaries[r_idx][0] : 0);
        if (ret > SLIDER_LIMIT) ret = -10000;
        if (target >= rightBoundary) return 1;
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
            e = servosOn(true);
            break;
        default:
            LOG_WARN("Unknown Servo Command : {}", sCmd);
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

void ArmController::threadHandler() {
    while (true) {
        auto loopTime = std::chrono::steady_clock::now();
        std::unique_lock<std::mutex> lk(m_mtx);
        LOG_TRACE("Waiting for Arm Command...");
        m_cv.wait(lk, [this] { return !m_cmdManager.isEmpty() || !m_bRunning;});
        if (!m_bRunning) break;

        Message_t msg{};

        // Arm Commands
        if (m_cmdManager.pop(Port::Arm::IAI, msg)) {
            auto id = msg.arm_id;
            m_pArms[id]->move(msg.target, msg.acceleration, msg.v_max);
        }
    }
}

//void ArmController::strikerThreadHandler() {
//    while (true) {
//        std::unique_lock<std::mutex> lk(m_mtx);
//        LOG_TRACE("Waiting for Striker Command...");
//        m_cv.wait(lk, [this] { return !m_cmdManager.isEmpty() || !m_bRunning;});
//        if (!m_bRunning) break;
//
//        Message_t msg{};
//
//        // Striker Commands
//        if (m_cmdManager.pop(Port::Arm::Epos, msg)) {
//            m_strikerController.strike(msg.midiNote, msg.arm_id, msg.midiVelocity);
//        }
//
//    }
//}

void ArmController::statusQueryHandler() {
    while (m_bRunning) {
        for (auto* pArm: m_pArms) {
            Error_t e = pArm->updateStatusFromDevice();
            if (e != kNoError) {
                LOG_ERROR("Error updating position for Arm: {}", pArm->getID());
            }
        }
        std::this_thread::sleep_for( std::chrono::milliseconds(1));
    }
}

Error_t ArmController::home() {
    Error_t e;

    e = servosOn(true);
    ERROR_CHECK(e, e);

    return m_IAIController.home();
}

Error_t ArmController::servosOn(bool bTurnOn) {
    LOG_INFO("Servos on...");
    Error_t e;
//    for (auto* arm: m_pArms) {
//        e = arm->servoOn(bTurnOn);
//        ERROR_CHECK(e, e);
//    }

    // Set id to -1 to set all axes at once
    e = m_IAIController.setServo(-1, bTurnOn);
    ERROR_CHECK(e, e);

    return kNoError;
}
