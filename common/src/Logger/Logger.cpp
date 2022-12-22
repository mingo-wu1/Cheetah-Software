/*!
 * @file logger.cpp
 * @brief Log system interface
 * @author hanyuanqiang
 */

#include "Logger/Logger.h"

static BZL_QUADRUPED::Logger systemLoger("system");

static void sigxxx_handler(int sig)
{
    QUADRUPED_INFO(systemLoger, "SIG num: %d, Interrupt!", sig);
    BZL_QUADRUPED::LoggerShutDown();
    exit(0);
}

namespace BZL_QUADRUPED
{
    void LoggerInit(int argc, char const * const argv[])
    {
#if (USE_LOGGER == 1)
        rclcpp::init(argc, argv);
        rclcpp::uninstall_signal_handlers();
#else
        (void)argc;
        (void)argv;
#endif
        for (int i = 1; i <= 31; i++)
        {
            if (17 == i)
            {
                continue;
            }
            signal(i, sigxxx_handler);
        }
    }

    void LoggerShutDown()
    {
#if (USE_LOGGER == 1)
        rclcpp::shutdown();
#endif
    }

    Logger::Logger(std::string name)
#if (USE_LOGGER == 1)
        : _logger(rclcpp::get_logger(name))
#endif
    {
        _name = name;
    }

#if (USE_LOGGER == 1)
    const rclcpp::Logger& Logger::GetLogger()
    {
        return _logger;
    }
#else
    const std::string& Logger::GetLogger()
    {
        return _name;
    }
#endif 

}  // namespace BZL_QUADRUPED