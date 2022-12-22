#include "logger.h"

int main()
{
    // using namespace bzllog;
    if (!BZLLogger::Get().Init("logs/test.log"))
    {
        std::cout<<"no file."<<std::endl;
        return 1;
    }

    STM_DEBUG() << "STM_DEBUG" << 1;
    // PRINT_WARN("PRINT_WARN, %d", 1);
    // LOG_INFO("LOG_INFO {}", 1);

    BZLLogger::Get().SetLevel(spdlog::level::info);
    STM_INFO() << "STM_DEBUG " << 2;
    // PRINT_WARN("PRINT_WARN, %d", 2);
    // LOG_INFO("LOG_INFO {}", 2);

    // call before spdlog static variables destroy
    BZLLogger::Get().ShutDown();
    return 0;
}