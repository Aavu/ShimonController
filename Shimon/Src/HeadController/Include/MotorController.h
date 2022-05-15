//
// Created by Raghavasimhan Sankaranarayanan on 4/27/22.
//

#ifndef SHIMONCONTROLLER_MOTORCONTROLLER_H
#define SHIMONCONTROLLER_MOTORCONTROLLER_H

#include <utility>

#include <iostream>
#include "Def.h"
#include "HeadCommands.h"
#include "CopleyASCIIController.h"
#include "DxlController.h"
#include "HDMotor.h"
#include "DxlMotor.h"
#include "Motor.h"
#include "yaml-cpp/yaml.h"

#include "ErrorDef.h"

class MotorController {
public:
    explicit MotorController(std::string  configFile) :m_configFile(std::move(configFile)) {}

    Error_t init(bool shouldHome = true) {
        Error_t e;
        e = parseConfig(m_configFile);
        ERROR_CHECK(e, e);
        if (shouldHome) return home();
        return kNoError;
    }

    Error_t reset() {
        Error_t e;

        for (auto& element: m_motors) {
            if (element.second) {
                e = element.second->reset();
                ERROR_CHECK(e, e);
            }
        }

        return kNoError;
    }

    Error_t home() {
        LOG_INFO("Homing Head");
        Error_t e;
        e = servosOn(true);
        ERROR_CHECK(e, e);

        for (auto& element: m_motors) {
            e = element.second->home();
            ERROR_CHECK(e, e);
        }

        bool bHomed = false;

        while (!bHomed) {
            bHomed = true;
            for (auto& element: m_motors) {
                auto response = element.second->isHomed();
                if (!response) bHomed = false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        return kNoError;
    }

    Error_t zero() {
        LOG_INFO("Zeroing head");
        Error_t e;
        for (auto& element: m_motors) {
            e = element.second->zero();
            ERROR_CHECK(e, e);
        }
        return kNoError;
    }

    Error_t servosOn(bool turnOn) {
        LOG_INFO("Switching Servos On");
        Error_t e;
        for (auto& element: m_motors) {
            e = element.second->servoOn(turnOn);
            ERROR_CHECK(e, e);
        }

        return kNoError;
    }

    Error_t send(const HeadCmdPacket_t& cmdPacket) {
        auto cmd = cmdPacket.cmd;
        switch (cmdPacket.cmdType) {
            case HeadCommandType::Dummy:
                return kNoError;
            case HeadCommandType::Control:
                return m_motors[cmd]->set(cmdPacket.fVel, cmdPacket.fAcc);
            case HeadCommandType::LowLevel:
                return m_motors[cmd]->goTo(*cmdPacket.fPos, cmdPacket.fVel, cmdPacket.fAcc);
            case HeadCommandType::Behavioral:
                std::cout << "Behavioral Commands not implemented yet\n";
                return kNotImplementedError;
        }
        return kNoError;
    }

private:
    std::unordered_map<HeadCommand, std::unique_ptr<Motor>> m_motors;
    std::string m_configFile;

    Error_t parseConfig(const std::string& configFile) {
        try {
            auto temp = YAML::LoadFile(configFile);

            for (auto&& config: temp) {
                auto axis = config["axis"].as<int>();
                auto cmd = HeadCommandUtil::getCommand(Util::toLowerCase(config["name"].as<std::string>()));
                MotorConfig_t motorConfig {
                        .name = cmd,
                        .axis = axis,
                        .zeroEncoder = config["zeroEnc"].as<int>(),
                        .minPosition = (config["min"]) ? config["min"].as<float>() : (float)INT32_MIN,
                        .maxPosition = (config["max"]) ? config["max"].as<float>() : (float)INT32_MAX,
                        .scale = config["scale"].as<float>(),
                        .maxVelocity = config["maxVel"].as<float>(),
                        .maxAcceleration = config["maxAcc"].as<float>(),
                        .defaultPosition = config["default"].as<float>(),
                        .defaultVelocity = config["defaultVel"].as<float>(),
                        .homing = (config["homing"].as<std::string>() == HOMING_MANUAL) ? MotorConfig_t::HomingType::Manual : MotorConfig_t::HomingType::HardStop,
                        .motorType = (config["type"].as<std::string>() == MOTOR_HD) ? MotorConfig_t::MotorType::HD : MotorConfig_t::MotorType::Dxl
                };

                if (m_motors[cmd]) {
                    std::cerr << "Warning: Overlapping motor names in HeadMotorConfig.yaml ...\n";
                    return kNamingError;
                }

                if (cmd == HeadCommand::Unknown) {
                    std::cerr << "Warning: Please check the names in HeadMotorConfig.yaml to align with the names in Commands.h file ...\n";
                    return kUnknownCaseError;
                }

                Error_t e;
                if (motorConfig.motorType == MotorConfig_t::MotorType::HD) {
                    auto m = std::make_unique<HDMotor>();
                    e = m->init(std::move(motorConfig));
                    ERROR_CHECK(e, e);
                    m_motors[cmd] = std::move(m);
                } else {
                    auto m = std::make_unique<DxlMotor>();
                    e = m->init(std::move(motorConfig));
                    ERROR_CHECK(e, e);
                    m_motors[cmd] = std::move(m);
                }
            }
        } catch (YAML::BadFile& e) {
            std::cerr << "HeadMotorConfig.yaml not found. Please provide correct file path and re-run. \n";
            return kFileParseError;
        } catch (YAML::Exception& e) {
            std::cerr << e.msg << std::endl;
            return kFileParseError;
        }

        return kNoError;
    }
};

#endif //SHIMONCONTROLLER_MOTORCONTROLLER_H
