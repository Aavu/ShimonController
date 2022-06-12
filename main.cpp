#include <iostream>
#include "Shimon.h"
#include <thread>
#include "Def.h"
#include "csignal"
#include "ErrorDef.h"
#include "Logger.h"

std::unique_ptr<Shimon> pShimon;

void sigHandle(int sig) {
    std::cout << "Stopping...\n";
    if (pShimon)
        pShimon->stop();
}

int main() {
    signal(SIGINT, sigHandle);
    signal(SIGTERM, sigHandle);
//    std::cout << SPDLOG_ACTIVE_LEVEL << std::endl;
    Logger::init((Logger::Level)SPDLOG_ACTIVE_LEVEL);
    LOG_INFO("Shimon Booting up");

    Error_t e;

    pShimon = std::make_unique<Shimon>(PORT, HM_CONFIG_FILE, CMD_BUFFER_SIZE);

    e = pShimon->initArms();
    if (e != kNoError) {
        LOG_ERROR("Arm Init failed. Error Code {}", e);
        return e;
    }

//    e = pShimon->initHead();
//    if (e != kNoError) {
//        LOG_ERROR("Head Init failed. Error Code {}", e);
//        return e;
//    }

    pShimon->start();

    return 0;
}
