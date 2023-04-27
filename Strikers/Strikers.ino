//
// Created by Raghavasimhan Sankaranarayanan on 03/30/22.
//

#include "src/strikerController.h"
#include "src/logger.h"

StrikerController* pController = nullptr;

String inputString = "";        // To hold incoming serial message
bool stringComplete = false;    // Setting this flag will start processing of received message

void setup() {
    // put your setup code here, to run once:
    LOG_LOG("Initializing Shimon's Strikers...");
    inputString.reserve(10);
    pController = StrikerController::createInstance();
    int err = pController->init(MotorSpec::EC60);
    if (err != 0) {
        LOG_ERROR("Controller Init failed");
        return;
    }

    LOG_LOG("Successfully Initialized! Controller Starting...");
    pController->start();
    delay(75);
    LOG_LOG("Listening for commands...");   // "in format (ascii characters) <mode><id code><midi velocity>"
}

void loop() {
    if (stringComplete) {
        // LOG_LOG("%s", inputString);
        uint8_t idCode;
        uint8_t midiVelocity;
        uint8_t chPressure;
        char cMode;
        Error_t err = parseCommand(inputString, cMode, idCode, midiVelocity, chPressure);
        inputString = "";
        stringComplete = false;

        if (err == kNoError) {
            LOG_LOG("mode %c, idCode: %i, velocity: %i, pressure: %i", cMode, idCode, midiVelocity, chPressure);
            pController->executeCommand(idCode, cMode, midiVelocity, chPressure);
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

// Format example to strike using motor 1 with velocity 80: s<SCH>P ... explanation s -> normal strike, <SCH> -> ascii of 0b00000001, P -> ascii of 80
// Pressure is another parameter to map when using choreo
// To stop tremolo, send mode t with velocity 0
Error_t parseCommand(const String& cmd, char& mode, uint8_t& idCode, uint8_t& midiVelocity, uint8_t& channelPressure) {
    if (cmd.length() < 4) return kCorruptedDataError;

    mode = cmd[0];
    idCode = cmd[1];
    midiVelocity = cmd[2];
    if (cmd.length() == 5) {
        channelPressure = cmd[3];
    }

    return kNoError;
}