//
// Created by Raghavasimhan Sankaranarayanan on 4/9/22.
//

#ifndef SHIMONCONTROLLER_DEF_H
#define SHIMONCONTROLLER_DEF_H

#define SIMULATE

#ifndef SPDLOG_ACTIVE_LEVEL
#define SPDLOG_ACTIVE_LEVEL 1   // Debug
#endif

#define HOST "192.168.2.31"
#define PORT 9000
#define DLY 465 // ms

#define ARM_CMD_OSC_ROUTE "/arm"
#define STRIKER_CMD_OSC_ROUTE "/striker"

#define IAI_ACTUATOR "/dev/IAIactuator"
#define IAI_ACTUATOR_BAUDRATE 230400

#define STRIKER_HOST "192.168.2.31" //"169.254.60.1"
#define STRIKER_PORT 1002

#define NUM_ARMS 4
#define NUM_STRIKERS 8
#define NUM_OCTAVES_TO_TRY 4

const int kNotePositionTable[49] = {0, 10, 44, 73, 102, 157, 184, 212, 240, 267, 294, 324, 377, 406, 434, 463, 490, 546, 574, 599, 624, 651, 673, 698, 749, 771, 798, 820, 846, 894, 919, 945, 969, 993, 1018, 1044, 1092, 1118, 1142, 1167, 1193, 1240, 1266, 1291, 1315, 1339, 1364, 1385, 1385};
const int kBoundaries[NUM_ARMS][2] = {{-1, 40}, {40, 150}, {150, 40}, {40, -1}};
const int kBlackKeys[5] = {1, 3, 6, 8, 10};
const int kOctavesToTry[NUM_OCTAVES_TO_TRY] = {0, 1, -1, 2};

#define MIN_NOTE 48    // Midi Note Number
#define MAX_NOTE 95    // Midi Note Number
#define SLIDER_LIMIT 1385   // mm
#define ACC_THRESHOLD 3 // g

// Head stuffs
#define HM_CONFIG_FILE "../Config/HeadMotorConfig.yaml"
#define SHIMON_OSC_ROUTE "/shimon"

#define HEAD_CMD_OSC_ROUTE "/head-commands"
#define HEAD_CMD_OSC_PORT 30310

#define COPLEY_PORT "/dev/copleyASCII"
#define COPLEY_BAUDRATE 115200

#define NUM_HEAD_AXES 4

#define DXL_PORT "/dev/dynamixel"
#define DXL_BAUDRATE 57600

#define HOMING_MANUAL "manual"
#define HOMING_HARDSTOP "hardstop"

#define MOTOR_HD "HD"
#define MOTOR_DXL "dxl"

namespace Port {
    enum class Head {
        Copley = 0, // All head actuators
        Dxl,        // Eyebrow & mouth

        kNumPorts
    };

    enum class Arm {
        IAI = 0,    // Linear actuator
        Epos,       // Strikers

        kNumPorts
    };

    const int kNumPorts = 2;
}

#define CMD_BUFFER_SIZE 16
#define THREAD_TIME_PERIOD 20   // ms
#endif //SHIMONCONTROLLER_DEF_H
