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
    Logger::init(Logger::trace);
    LOG_INFO("Shimon Booting up");

    Error_t e;

    pShimon = std::make_unique<Shimon>(PORT, HM_CONFIG_FILE, CMD_BUFFER_SIZE);

    pShimon->init(MASTER_HOST, MASTER_PORT);
    pShimon->start();

    return 0;
}
