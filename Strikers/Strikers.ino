//
// Created by Raghavasimhan Sankaranarayanan on 03/30/22.
//

#include "src/strikerController.h"
#include "src/logger.h"

StrikerController* pController = nullptr;

String inputString = "";        // To hold incoming serial message
bool stringComplete = false;    // Setting this flag will start processing of received message

void setup() {
    LOG_LOG("Initializing Shimon's Strikers...");
    inputString.reserve(10);
    pController = StrikerController::createInstance();
    int err = pController->init(MotorSpec::EC60);
    if (err != 0) {
        LOG_ERROR("Controller Init failed");
        return;
    }
    // Home
    delay(100);
    err = pController->home();
    if (err != 0) {
        LOG_ERROR("Controller Homing failed");
        return;
    }
    LOG_LOG("Successfully Initialized! Controller Starting...");
    delay(50);
    pController->start();
    delay(75);
    LOG_LOG("Listening for commands...");   // "in format (ascii characters) <mode><id code><midi velocity><Channel Pressure>"
}

void loop() {
    if (stringComplete) {
        uint8_t idCode;
        uint16_t param1;
        uint16_t param2;
        Striker::Command cmd;
        Error_t err = parseCommand(inputString, cmd, idCode, param1, param2);
        inputString = "";
        stringComplete = false;

        if (err == kNoError) {
            LOG_LOG("cmd %c, idCode: %i, param1: %i, param2: %i", cmd, idCode, param1, param2);
            pController->executeCommand(idCode, cmd, param1, param2);
        }
    }
}

void serialEvent() {
    while (Serial.available()) {
        char inChar = (char) Serial.read();
        inputString += inChar;
        if (inChar == '\n') {
            stringComplete = true;
        }
    }
}

// To stop tremolo, send mode t with velocity 0
Error_t parseCommand(const String& rawData, Striker::Command& cmd, uint8_t& idCode, uint16_t& param1, uint16_t& param2) {
    if (rawData.length() < 5) return kCorruptedDataError;

    cmd = StrikerController::getStrikerCmd(rawData[0]);
    idCode = rawData[1];
    param1 = (uint8_t(rawData[2]) << 8) + (uint8_t) rawData[3];
    param2 = (uint8_t(rawData[4]) << 8) + (uint8_t) rawData[5];

    return kNoError;
}