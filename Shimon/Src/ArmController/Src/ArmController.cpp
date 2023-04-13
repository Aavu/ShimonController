//
// Created by Raghavasimhan Sankaranarayanan on 4/9/22.
//

#include "ArmController.h"
#include "Util.h"

ArmController::ArmController(OscListener& oscListener,
                             size_t cmdBufferSize,
                             tp programStartTime) : m_oscListener(oscListener)
                                                    , m_cmdManager(cmdBufferSize, m_cv)
                                                    , m_strikerController(cmdBufferSize)
                                                    , m_IAIController(IAIController::getInstance())
                                                    , kProgramStartTime(programStartTime)
                                                    , m_debugLog("debug.log", std::ios::trunc)
{
    for (int i = 0; i< NUM_ARMS; ++i) {
        m_pArms[i] = new Arm(i, kInitialArmPosition[i], kW[i], kB[i]);
    }

    ArmCommandUtil::populateStrCmdMap();
}

ArmController::~ArmController() {
    reset();

    for (int i = 0; i< NUM_ARMS; ++i) {
        delete m_pArms[i];
        m_pArms[i] = nullptr;
    }

    m_debugLog.close();
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

    e = m_strikerController.init(STRIKER_PORT, STRIKER_BAUDRATE);
//    ERROR_CHECK(e, e);

    m_iNoteCounter = 0;
    m_bInitialized = true;

    return kNoError;
}

Error_t ArmController::reset() {
    auto err = m_IAIController.reset(true);
    ERROR_CHECK(err, err);

    err = resetArms();
    ERROR_CHECK(err, err);
    m_iNoteCounter = 0;
    return m_strikerController.reset();
}

Error_t ArmController::resetArms() {
    Error_t e = kNoError;
    for (int i = 0; i< NUM_ARMS; ++i) {
        auto err = m_pArms[i]->reset();
        if (err != kNoError) e = err;
    }

    return e;
}

void ArmController::armMidiCallback(int note, int velocity) {

    std::lock_guard<std::mutex> lk(m_oscMtx);
    Arm::Message_t msg{};
    LOG_DEBUG("Note: {}, velocity: {}", note, velocity);
    msg.midiVelocity = velocity;
    msg.midiNote = note;

    m_iNoteCounter++;
    Error_t e = planPath(msg, false);
    if (e != kNoError) {
        LOG_ERROR("Path Planning Failed for note {} ...", msg.midiNote);
        return;
    }

    bool success = m_cmdManager.push(Port::Arm::IAI, msg);
    if (!success) {
        LOG_ERROR("Unable to push msg");
    }
}

Error_t ArmController::start() {
    if (!m_bInitialized) return kNotInitializedError;
    m_bRunning = true;
    for (auto & thread : m_pThreadPool) {
        thread = std::make_unique<std::thread>([this] { threadPoolHandler(); });
    }
    m_pStatusQueryThread = std::make_unique<std::thread>([this] {statusQueryHandler();});

    return m_strikerController.start();
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
    if (m_pStatusQueryThread && m_pStatusQueryThread->joinable()) m_pStatusQueryThread->join();

    m_strikerController.stop();
    return kNoError;
}

Error_t ArmController::planPath(Arm::Message_t& msg, bool bMoveInterferingArm) {
    auto midiVelocity = msg.midiVelocity;
    auto e = kImpossibleError;
    auto timeNow = steady_clock::now();

    auto originalNote = msg.midiNote;
    if (originalNote <= 0) return kInvalidArgsError;

    for (int o = 0; o < NUM_OCTAVES_TO_TRY; ++o) {
        msg.midiNote = Util::transpose(originalNote, kOctavesToTry[o]*12);
        int target = midiToPosition(msg.midiNote);

        if (target < 0 || target > SLIDER_LIMIT) {
            e = kImpossibleError;
            goto return_err;
        }

        std::list<Arm::Message_t> c_msg;

        // To keep track of movable arms if interference is cleared
        std::list<Arm::Message_t> candidateMsgs;

        for (int i=0; i<m_pArms.size(); ++i) {
            Arm* pArm = m_pArms[i];

            // New target is not yet set. Thus, current arm msg is the last sent message.
            auto lastTime = pArm->getMsgTime();
            auto lastTarget = pArm->getTarget();

            long diffTime = duration_cast<milliseconds>(timeNow + milliseconds(DLY) - pArm->getArrivalTime()).count();
            int time_ms = (int)std::min(DLY + 0l, std::max(1l, diffTime));

            Arm::Message_t m = {
                    .arm_id = i,
                    .target = target,
                    .midiVelocity = msg.midiVelocity,
                    .time_ms = time_ms,
                    .arrivalTime = std::max(timeNow, pArm->getArrivalTime()) + milliseconds(time_ms),
                    .msgTime = std::max(timeNow, pArm->getArrivalTime())
            };

            e = pArm->computeMotionParam(m);
            int dir = (target > lastTarget) ? 1 : -1;
            dir = (target == lastTarget) ? 0 : dir;

            if (e == kNoError) {    // Motion possible
                int dummy;
                // returns 0 if there is no interference, 1 if interference is to the right and -1 if interference is to the left
                int interference = checkInterference(i, target, dir, dummy);
                if (interference != 0) {
                    e = kInterferenceError;
                    candidateMsgs.push_back(m);
                    continue;
                }

                c_msg.push_back(m);
            }
        }

        if (bMoveInterferingArm) {
            if (!candidateMsgs.empty()) {
                candidateMsgs.sort([](const Arm::Message_t& m1, const Arm::Message_t& m2) {
                    return m1.acceleration < m2.acceleration;
                });

//                Arm::Message_t interferenceMsg;
//                auto& temp = candidateMsgs.front();
//                Error_t _err = getInterference(temp, interferenceMsg);
            }
        }

        if (!c_msg.empty()) {
            c_msg.sort([](const Arm::Message_t& m1, const Arm::Message_t& m2) {
                return m1.acceleration < m2.acceleration;
            });

            auto& m = c_msg.front();
            msg.acceleration = m.acceleration;
            msg.v_max = m.v_max;
            msg.arm_id = m.arm_id;
            msg.midiVelocity = m.midiVelocity;
            msg.target = m.target;
            msg.time_ms = m.time_ms;
            msg.msgTime = m.msgTime;
            msg.arrivalTime = m.arrivalTime;

//            auto lastMsgTime = m_pArms[m.arm_id]->getMsgTime();
            m_pArms[m.arm_id]->setMessage(msg);
            LOG_INFO("S.No: {} \t Note: {} \t Arm: {} \t time: {} \t position: {} \t target: {} \t acc: {} \t v_max: {} \t msgTime: {} \t arrival time: {}",
                     m_iNoteCounter,
                     msg.midiNote,
                     msg.arm_id,
                     msg.time_ms,
                     m_pArms[msg.arm_id]->getLastTarget(),
                     msg.target,
                     msg.acceleration,
                     (int)std::round(msg.v_max),
                     duration_cast<milliseconds>(msg.msgTime - kProgramStartTime).count(),
                     duration_cast<milliseconds>(msg.arrivalTime - kProgramStartTime).count());
            LOG_INFO("Arm 0: {}, Arm 1: {}, Arm 2: {}, Arm 3: {}", m_pArms[0]->getPosition(), m_pArms[1]->getPosition(), m_pArms[2]->getPosition(), m_pArms[3]->getPosition());
            e = kNoError;
            goto return_err;
        }

        LOG_INFO("{} octave not possible for note {} at position {}. Reason code: {}", kOctavesToTry[o], originalNote, target, e);
    }

    return_err:
    {
//        for (int i = 0; i < m_pArms.size(); ++i) {
//            Arm *pArm = m_pArms[i];
//            auto lb = pArm->getLeftBoundary();
//            auto lbt = pArm->getLeftBoundaryFor(target);
//            auto rb = pArm->getRightBoundary();
//            auto rbt = pArm->getRightBoundaryFor(target);
//            std::cout << lb << " " << rb << " ";
//        }
//
//        std::cout << std::endl;

        if (e == kImpossibleError)
            LOG_WARN("Impossible to moveArm to the note...");

        if (e != kNoError)
            msg.midiNote = originalNote;    // Restore original msg

        msg.midiVelocity = midiVelocity;
        return e;
    }
}

Error_t ArmController::getInterference(const Arm::Message_t& msg, Arm::Message_t& returnMsg) {
    Error_t e = kNoError;
    returnMsg = msg;
    return e;
}

int ArmController::checkInterference(int armId, int target, int direction, int& ret) {
    if (armId < 0 || armId >= NUM_ARMS) {
        ret = -10000;
        return 0;
    }

    int leftBoundary, rightBoundary;
    int interference = 0;

    if (direction < 0) {
        int l_idx = armId - 1;

        if (l_idx < 0) {
            ret = -10000;
            return 0;
        }

        leftBoundary = m_pArms[l_idx]->getRightBoundary();
        ret = target - kBoundaries[l_idx][1];
        if (ret < 0) ret = -10000;
        if (target <= leftBoundary) interference = -1;

    } else if (direction > 0) {
        int r_idx = armId + 1;

        if (r_idx >= NUM_ARMS) {
            ret = -10000;
            return 0;
        }

        rightBoundary = m_pArms[r_idx]->getLeftBoundary();
        ret = target + kBoundaries[r_idx][0];
        if (ret > SLIDER_LIMIT) ret = -10000;
        if (target >= rightBoundary) interference = 1;
    } else {    // This means the arm is probably already there
        ret = 0;
        interference = 0;
    }

    return interference;
}

int ArmController::midiToPosition(int note) {
    if (note < MIN_NOTE || note > MAX_NOTE) return -1;
    int position = note - MIN_NOTE;
    position = std::min(position, MAX_NOTE - MIN_NOTE + 1);
    return kNotePositionTable[position];
}

void ArmController::armCallback(int armId, int position, float acceleration, float v_max) {
    Arm::Message_t msg {.arm_id = armId, .target = position, .acceleration = acceleration, .v_max = v_max };
    m_pArms[armId]->setTargetAndMove(msg);
}

void ArmController::armServoCallback(const char *sCmd) {
    auto cmd = ArmCommandUtil::getCommand(sCmd);

    Error_t e;
    switch (cmd) {
        case ArmCommand::Home:
            LOG_INFO("Homing...");
            e = home();
            m_iNoteCounter = 0;
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
            m_iNoteCounter = 0;
            break;
        default:
            LOG_WARN("Unknown Servo Command : {}", sCmd);
            e = kUnknownCaseError;
            break;
    }

    if (e != kNoError) {
        LOG_ERROR("Error Code: {}", e);
    }

    m_debugLog.close();
    m_debugLog.open("debug.log", std::ios::trunc);
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

void ArmController::threadPoolHandler() {
    while (m_bRunning) {
        Arm::Message_t msg{};
        int id;
        tp now, msgTime, arrivalTime;
        bool success = false;
        {
            std::unique_lock<std::mutex> lk(m_mtx);
            m_cv.wait(lk, [this] { return !m_cmdManager.isEmpty() || !m_bRunning; });
            if (!m_bRunning) break;
            success = m_cmdManager.pop(Port::Arm::IAI, msg);
            id = msg.arm_id;

            if (m_pArms[id]->getTarget() != msg.target) {
                LOG_WARN("Order screwed up!!!. Arm target: {}, msg target: {}", m_pArms[id]->getTarget(), msg.target);
                m_pArms[id]->setMessage(msg);
            }

            msgTime = m_pArms[id]->getMsgTime();
            arrivalTime = m_pArms[id]->getArrivalTime();
//            LOG_INFO("Elapsed {} ms", duration_cast<milliseconds>(std::chrono::steady_clock::now() - now).count());
        }

        if (success) {
            now = std::chrono::steady_clock::now();
//            LOG_DEBUG("Sleeping for {} ms", duration_cast<milliseconds>(msgTime - now).count());
            std::this_thread::sleep_until(msgTime);
            m_newCmdCv.notify_all();
            Error_t e = m_pArms[id]->move();
            if (e != kNoError) {
                if (e != kAlreadyThereError)
                    LOG_ERROR("Move Error Code: {}", e);
            }
            e = m_strikerController.scheduleStrike(msg.midiNote, id, msg.midiVelocity, arrivalTime);
            if (e != kNoError) LOG_ERROR("Error Scheduling Strike: {}", e);
        } else {
            LOG_ERROR("Cannot get Arm msg. Error Code: {}", kBufferReadError);
        }
    }
}

void ArmController::statusQueryHandler() {
    const int interval_ms = 1;
    std::array<int, NUM_ARMS> position{};
    std::array<int, NUM_ARMS> lastPosition{-1 ,-1, -1, -1};
    while (m_bRunning) {
        std::unique_lock<std::mutex> lk(m_newCmdMtx);
        m_newCmdCv.wait(lk, [this] { return !m_bRunning || m_IAIController.isHomed();});
        if (!m_bRunning) break;
        auto now = std::chrono::steady_clock::now();
        for (auto *pArm: m_pArms) {
            Error_t e = kNoError;
//#ifndef SIMULATE
//            e = pArm->updateStatusFromDevice();
//#else
            pArm->updatePositionFromTrajectory(interval_ms);
//#endif
//            if (e != kNoError) {
//                LOG_ERROR("Error updating position for Arm: {}", pArm->getID());
//                pArm->updatePositionFromTrajectory(interval_ms);
//            }

            position[pArm->getID()] = pArm->getPosition();
        }

        if (m_positionCallback) {
            if (Util::arePositionsDifferent(position, lastPosition)) {
                m_positionCallback(position);
                LOG_INFO("Positions: {}, {}, {}, {}", position[0], position[1], position[2], position[3]);
            }
        }
        lastPosition = position;
        std::this_thread::sleep_until(now + std::chrono::milliseconds(interval_ms));
    }
}

Error_t ArmController::home() {
    Error_t e;

    e = servosOn(true);
    ERROR_CHECK(e, e);

    e = m_IAIController.home();
    if (m_statusCallback && e == kNoError) m_statusCallback(Status_t::HomingComplete);
    ERROR_CHECK(e, e);
    e = resetArms();

    return e;
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
