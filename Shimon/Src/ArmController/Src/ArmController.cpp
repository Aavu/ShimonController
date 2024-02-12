//
// Created by Raghavasimhan Sankaranarayanan on 4/9/22.
//

#include "ArmController.h"
#include "Util.h"

ArmController::ArmController(OscListener& oscListener,
                             tp programStartTime) : m_oscListener(oscListener)
                                                    , m_cmdManager(m_cv)
                                                    , m_strikerController(programStartTime)
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

    m_oscListener.setStrikerChoreoCallback([this](auto &&PH1, auto &&PH2, auto &&PH3) {
        strikerChoreoCallback(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2), std::forward<decltype(PH3)>(PH3));
    });

    LOG_TRACE("Setting setArmMidiCallback");
    m_oscListener.setArmMidiCallback([this](auto &&PH1, auto &&PH2, auto &&PH3) {
        armMidiCallback(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2), std::forward<decltype(PH3)>(PH3));
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

void ArmController::strikerChoreoCallback(uint8_t idCode, int target, int time_ms) {
    std::lock_guard<std::mutex> lk(m_oscMtx);
    Error_t e = m_strikerController.scheduleStrike(idCode, target, time_ms);
    if (e != kNoError) LOG_ERROR("Error Scheduling Strike: {}", e);
}

void ArmController::armMidiCallback(bool bNoteON, int note, int velocity) {
    std::lock_guard<std::mutex> lk(m_oscMtx);
    Arm::Message_t msg{};

    if (!bNoteON) {
        return;
    }

//    LOG_DEBUG("Note: {}, velocity: {}", note, velocity);
    msg.midiVelocity = velocity;
    msg.midiNote = note;

    m_iNoteCounter++;
    Error_t e = planPath(msg);
    if (e != kNoError) {
        LOG_ERROR("Path Planning Failed for note {} ...", msg.midiNote);
        return;
    }

    e = m_pArms[msg.arm_id]->addMessageToQueue(msg);
    if (e != kNoError) {
        LOG_ERROR("Failed adding message to queue for arm {}. Error {}", msg.arm_id, e);
        return;
    }

//    msg.pprint();
    e = m_strikerController.scheduleStrike(msg.midiNote, msg.arm_id, msg.midiVelocity, msg.arrivalTime, StrikerController::Mode::Strike);
    if (e != kNoError) LOG_ERROR("Error Scheduling Strike: {}", e);
}

Error_t ArmController::start() {
    if (!m_bInitialized) return kNotInitializedError;
    m_bRunning = true;
    m_pStatusQueryThread = std::make_unique<std::thread>([this] {statusQueryHandler();});

    for (auto* pArm: m_pArms)
        pArm->start();

    return m_strikerController.start();
}

Error_t ArmController::stop() {
    if (!m_bInitialized) return kNotInitializedError;
    m_bRunning = false;
    m_cv.notify_all();
    m_newCmdCv.notify_all();

    if (m_pStatusQueryThread && m_pStatusQueryThread->joinable()) m_pStatusQueryThread->join();

    m_strikerController.stop();

    for (auto* pArm: m_pArms)
        pArm->stop();

    return kNoError;
}

Error_t ArmController::planPath(Arm::Message_t& msg) {
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

        for (auto* pArm: m_pArms) {
            LOG_INFO("Boundaries Arm {} Left {} Right {}", pArm->getID(), pArm->getLeftBoundary(), pArm->getRightBoundary());
        }
        for (auto* pArm: m_pArms) {
            int id = pArm->getID();

            // current position is the position of the arm (maybe in the future) after it strikes the most recent message
            int mostRecentPosition = pArm->getMostRecentTarget();

            tp arrivalTime = timeNow + milliseconds(DLY);
            long diffTime = duration_cast<milliseconds>(arrivalTime - pArm->getMostRecentArrivalTime()).count();
//            if (diffTime < 1) {
//                LOG_DEBUG("Diff time is {} ms. Motion not possible for arm {}", diffTime, id);
//                continue;
//            }

            int time_ms = (int)std::max(0l, std::min(DLY + 0l, diffTime));

            Arm::Message_t m = {
                    .arm_id = id,
                    .target = target,
                    .midiVelocity = msg.midiVelocity,
                    .time_ms = time_ms,
                    .arrivalTime = arrivalTime,
                    .msgTime = std::max(timeNow, pArm->getMostRecentArrivalTime())
            };

            e = pArm->computeMotionParam(m);
            int dir = (target > mostRecentPosition) ? 1 : -1;
            dir = (target == mostRecentPosition) ? 0 : dir;

            if (e == kNoError) {    // Motion possible
                int dummy;
                // returns 0 if there is no interference, 1 if interference is to the right and -1 if interference is to the left
                int interference = checkInterference(id, target, dir, dummy);
                if (interference != 0) {
                    e = kInterferenceError;
                    continue;
                }

                c_msg.push_back(m);
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

            e = kNoError;
            goto return_err;
        }

        LOG_INFO("{} octave not possible for note {} at position {}. Reason code: {}", kOctavesToTry[o], originalNote, target, e);
        if (e == kInterferenceError) {
            LOG_INFO("Arm 0: {}, Arm 1: {}, Arm 2: {}, Arm 3: {}", m_pArms[0]->getPosition(), m_pArms[1]->getPosition(), m_pArms[2]->getPosition(), m_pArms[3]->getPosition());
        }
    }

    return_err:
    {
        if (e == kImpossibleError)
            LOG_WARN("Impossible to moveArm to the note...");

        if (e != kNoError)
            msg.midiNote = originalNote;    // Restore original msg
        return e;
    }
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
    m_pArms[armId]->move(msg);
}

void ArmController::armServoCallback(const char *sCmd) {
    auto cmd = ArmCommandUtil::getCommand(sCmd);

    Error_t e;
    switch (cmd) {
        case ArmCommand::Home:
            e = home();
            m_iNoteCounter = 0;
            break;
        case ArmCommand::Off:
            e = servosOn(false);
            break;
        case ArmCommand::On:
            e = servosOn(true);
            break;
        case ArmCommand::AlarmClear:
            e = clearFault();
            break;
        case ArmCommand::Reset:
            LOG_INFO("Restart Strikers...");
            e = m_strikerController.restart();
            if (e != kNoError) LOG_ERROR("Error Code: {}", e);
            for (auto* pArm: m_pArms) {
                pArm->resetBuffer();
            }

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

void ArmController::statusQueryHandler() {
    const int interval_ms = 1;
    tp timeNow = steady_clock::now();
    while (m_bRunning) {
        updateArmPositionsToMaster(interval_ms);
        timeNow += std::chrono::milliseconds(interval_ms);
        std::this_thread::sleep_until(timeNow);
    }
}

Error_t ArmController::updateArmPositionsToMaster(int interval_ms) {
    std::array<int, NUM_ARMS> position{};
    bool bDidChange = false;
    for (auto *pArm: m_pArms) {
        if (interval_ms > 0)
            pArm->updatePositionFromTrajectory(interval_ms);
        int temp = pArm->getPosition();
        bDidChange = bDidChange || (position[pArm->getID()] != temp);
        position[pArm->getID()] = temp;
    }

    if (m_positionCallback) {
        if (bDidChange) {
            m_positionCallback(position);
//                LOG_INFO("Positions: {}, {}, {}, {}", position[0], position[1], position[2], position[3]);
        }
    }

    return kNoError;
}

Error_t ArmController::home() {
    Error_t e;

    e = servosOn(true);
    ERROR_CHECK(e, e);

    LOG_INFO("Homing...");
    e = m_IAIController.home();
    if (m_statusCallback && e == kNoError) m_statusCallback(Status_t::HomingComplete);
    ERROR_CHECK(e, e);
    e = resetArms();
    updateArmPositionsToMaster();
    return e;
}

Error_t ArmController::servosOn(bool bTurnOn) {
    Error_t e;

    e = clearFault();
    ERROR_CHECK(e, e);

    LOG_INFO(bTurnOn ? "Servos on..." : "Servos off...");
    // Set id to -1 to set all axes at once
    e = m_IAIController.setServo(-1, bTurnOn);
    return e;
}

Error_t ArmController::clearFault() {
    LOG_INFO("Alarm Clear...");
    return m_IAIController.clearFault();
}
