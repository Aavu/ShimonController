//
// Created by Raghavasimhan Sankaranarayanan on 03/30/22.
//

#ifndef STRIKERCONTROLLER_H
#define STRIKERCONTROLLER_H

#include "def.h"
#include "striker.h"
#include <HardwareTimer.h>

class StrikerController {
public:
    static StrikerController* createInstance() {
        pInstance = new StrikerController;
        return pInstance;
    }

    static void destroyInstance() {
        delete pInstance;
        pInstance = nullptr;
    }

    Error_t init(MotorSpec spec, bool bInitCAN = true) {
        int err = 0;

        if (bInitCAN) {
            if (!CanBus.begin(CAN_BAUD_1000K, CAN_STD_FORMAT)) {
                LOG_ERROR("CAN open failed");
                return kFileOpenError;
            }

            CanBus.attachRxInterrupt(canRxHandle);
        }

        RPDOTimer.setPeriod(PDO_RATE * 1000);
        RPDOTimer.attachInterrupt(RPDOTimerIRQHandler);

        return initStrikers(spec);
    }

    Error_t initStrikers(MotorSpec spec) {
        m_motorSpec = spec;
        Error_t err = kNoError;
        for (int i = 1; i < NUM_STRIKERS + 1; ++i) {
            err = m_striker[i].init(i, spec);
            if (err != kNoError) {
                LOG_ERROR("Cannot initialize striker with id %i. Error: %i", i, err);
            }
        }

        return kNoError;
    }

    Error_t resetStrikers() {
        enablePDO(false);
        for (int i = 1; i < NUM_STRIKERS + 1; ++i) {
            m_striker[i].reset();
        }
    }

    void reset(bool bTerminateCAN = true) {
        m_bPlaying = false;
        RPDOTimer.stop();
        auto err = resetStrikers();
        if (err != kNoError) {
            LOG_ERROR("Cannot reset strikers");
        }
        if (bTerminateCAN)
            CanBus.end();
    }

    static Striker::Command getStrikerCmd(char cmd) {
        switch (cmd) {
        case 's':
            return Striker::Command::Normal;
        case 't':
            return Striker::Command::Tremolo;
        case 'r':
            return Striker::Command::Restart;
        case 'q':
            return Striker::Command::Quit;
        case 'c':
            return Striker::Command::Choreo;
        default:
            LOG_ERROR("unknown command : %i", cmd);
            return Striker::Command::Normal;
        }
    }

    Error_t restart() {
        auto spec = m_motorSpec;
        reset(false);
        Error_t err = init(spec, false);
        if (err != kNoError) {
            LOG_ERROR("Cannot init controller");
            return err;
        }

        start();

        return kNoError;
    }

    uint8_t prepare(uint8_t idCode, Striker::Command cmd, uint16_t param1, uint16_t param2) {
        uint8_t uiStrike = 0;

        if (cmd == Striker::Command::Restart) {
            restart();
            return 0;
        }

        // If commnd is tremolo with param1 0, this means stop tremolo. Change the command accordingly
        if (cmd == Striker::Command::Tremolo && param1 == 0) cmd = Striker::Command::StopTremolo;

        for (int i = 1; i < NUM_STRIKERS + 1; ++i) {
            if (idCode & (1 << (i - 1))) {
                bool bStrike = m_striker[i].prepare(cmd, param1, param2);
                uiStrike += bStrike << (i - 1);
            }
        }

        return uiStrike;
    }

    void executeCommand(uint8_t idCode, Striker::Command cmd, uint16_t param1, uint16_t param2) {
        uint8_t uiStrike = prepare(idCode, cmd, param1, param2);
        strike(uiStrike);
    }

    void start() {
        Error_t err = enablePDO();
        if (err != kNoError) {
            LOG_ERROR("cannot enable PDO");
            return;
        }

        err = enableStrikers();
        if (err != kNoError) {
            LOG_ERROR("cannot enable Strikers");
            return;
        }

        m_bPlaying = true;
        RPDOTimer.start();
    }

    void stop() {
        Error_t err;
        err = enablePDO(false);
        if (err != kNoError) {
            LOG_ERROR("cannot disable PDO");
            return;
        }

        err = enableStrikers(false);
        if (err != kNoError) {
            LOG_ERROR("cannot disable Strikers");
            return;
        }

        m_bPlaying = false;
        RPDOTimer.stop();
    }

    Error_t enablePDO(bool bEnable = true) {
        Error_t err;
        for (int i = 1; i < NUM_STRIKERS + 1; ++i) {
            err = m_striker[i].enablePDO(bEnable);
            if (err != kNoError) {
                LOG_ERROR("EnablePDO failed. Error Code %i", err);
                return err;
            }
        }

        return kNoError;
    }

    Error_t enableStrikers(bool bEnable = true) {
        int err;
        for (int i = 1; i < NUM_STRIKERS + 1; ++i) {
            err = m_striker[i].enable(bEnable);
            if (err != 0) {
                LOG_ERROR("Enable failed. Error Code %h", err);
                return kSetValueError;
            }
        }

        return kNoError;
    }

    int strike(uint8_t idCode) {
        for (int i = 1; i < NUM_STRIKERS + 1; ++i) {
            if (idCode & (1 << (i - 1))) {
                // LOG_LOG("(idcode: %h) Striking: %i", idCode, i);
                m_striker[i].strike();
            }
        }
    }

private:
    Striker m_striker[NUM_STRIKERS + 1]; // 0 is dummy
    static StrikerController* pInstance;
    volatile bool m_bPlaying = false;
    MotorSpec m_motorSpec = MotorSpec::EC60;

    HardwareTimer RPDOTimer;

    StrikerController(): RPDOTimer(TIMER_CH1) {}

    ~StrikerController() {
        reset();
        destroyInstance();
    }

    static void canRxHandle(can_message_t* arg) {
        auto id = arg->id - COB_ID_SDO_SC;
        if (id > 0 && id < NUM_STRIKERS + 1) {
            pInstance->m_striker[id].setRxMsg(*arg);
        }

        id = arg->id - COB_ID_TPDO3;
        if (id > 0 && id < NUM_STRIKERS + 1) {
            pInstance->m_striker[id].PDO_processMsg(*arg);
        }

        id = arg->id - COB_ID_EMCY;
        if (id > 0 && id < NUM_STRIKERS + 1) {
            pInstance->m_striker[id].handleEMCYMsg(*arg);
        }
    }

    static void RPDOTimerIRQHandler() {
        for (int i = 1; i < NUM_STRIKERS + 1; ++i) {
            pInstance->m_striker[i].update();
        }
    }
};

StrikerController* StrikerController::pInstance = nullptr;
#endif // STRIKERCONTROLLER_H
