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
    LOG_INFO("Shimon Controller (Build 0.0.2)");
#ifdef SIMULATE
    LOG_INFO("*******SIMULATION MODE*******");
#endif

    Error_t e;

    pShimon = std::make_unique<Shimon>(PORT, HM_CONFIG_FILE);

    pShimon->init(MASTER_HOST, MASTER_PORT);
    pShimon->start();

    return 0;
}
