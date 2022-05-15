#include <iostream>
#include "Shimon.h"
#include "Def.h"
#include "csignal"
#include "ErrorDef.h"
#include "Logger.h"
#include "Util.h"

std::unique_ptr<Shimon> pShimon;

void sigHandle(int sig) {
    std::cout << "\nStopping...\n";
    if (pShimon)
        pShimon->stop();
}

//void* operator new(size_t size) {
////    static uint32_t uiAllocCount = 0;
////    ++uiAllocCount;
//    std::cout << "Allocating " << size << "bytes\n";
//    return malloc(size);
//}

int main() {
    signal(SIGINT, sigHandle);
    signal(SIGTERM, sigHandle);

    LOG_INFO("Shimon Booting up");

    Error_t e;

    pShimon = std::make_unique<Shimon>(PORT, HM_CONFIG_FILE, CMD_BUFFER_SIZE);

    e = pShimon->initArms(STRIKER_HOST, STRIKER_PORT, IAI_ACTUATOR, IAI_ACTUATOR_BAUDRATE);
    if (e != kNoError) return e;

//    e = pShimon->initHead();
//    if (e != kNoError) return e;

    pShimon->start();
    return 0;
}
