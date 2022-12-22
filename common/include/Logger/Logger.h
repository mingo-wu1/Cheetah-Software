/*!
 * @file Logger.h
 * @brief Log system interface
 * @author hanyuanqiang
 */

#ifndef LOGGER_H
#define LOGGER_H

#if (USE_LOGGER == 1)
#include <rclcpp/rclcpp.hpp>
#else
#include <string>
#include <csignal>
#endif

namespace BZL_QUADRUPED
{
    void LoggerInit(int argc, char const * const argv[]);
    void LoggerShutDown();

    class Logger
    {
        public:
        Logger(std::string name);
#if (USE_LOGGER == 1)
        const rclcpp::Logger& GetLogger();
#else
        const std::string& GetLogger();
#endif        

        private:
        std::string _name;
#if (USE_LOGGER == 1)
        rclcpp::Logger _logger;
#endif

    };
}



#if (USE_LOGGER == 1)
    #define QUADRUPED_DEBUG(logger, ...)        RCLCPP_DEBUG(logger.GetLogger(), __VA_ARGS__)
    #define QUADRUPED_INFO(logger, ...)         RCLCPP_INFO(logger.GetLogger(), __VA_ARGS__)
    #define QUADRUPED_WARN(logger, ...)         RCLCPP_WARN(logger.GetLogger(), __VA_ARGS__)
    #define QUADRUPED_ERROR(logger, ...)        RCLCPP_ERROR(logger.GetLogger(), __VA_ARGS__)
    #define QUADRUPED_FATAL(logger, ...)        RCLCPP_FATAL(logger.GetLogger(), __VA_ARGS__)
#else
    #define QUADRUPED_DEBUG(logger, ...) \
        do { \
            printf("[DEBUG] "); \
            printf("[%s]: ", logger.GetLogger().c_str()); \
            printf(__VA_ARGS__); \
            printf("\n"); \
        } while (0)
    #define QUADRUPED_INFO(logger, ...) \
        do { \
            printf("[INFO] "); \
            printf("[%s]: ", logger.GetLogger().c_str()); \
            printf(__VA_ARGS__); \
            printf("\n"); \
        } while (0)
    #define QUADRUPED_WARN(logger, ...) \
        do { \
            printf("[WARN] "); \
            printf("[%s]: ", logger.GetLogger().c_str()); \
            printf(__VA_ARGS__); \
            printf("\n"); \
        } while (0)
    #define QUADRUPED_ERROR(logger, ...) \
        do { \
            printf("[ERROR] "); \
            printf("[%s]: ", logger.GetLogger().c_str()); \
            printf(__VA_ARGS__); \
            printf("\n"); \
        } while (0)
    #define QUADRUPED_FATAL(logger, ...) \
        do { \
            printf("[FATAL] "); \
            printf("[%s]: ", logger.GetLogger().c_str()); \
            printf(__VA_ARGS__); \
            printf("\n"); \
        } while (0)
#endif


#endif