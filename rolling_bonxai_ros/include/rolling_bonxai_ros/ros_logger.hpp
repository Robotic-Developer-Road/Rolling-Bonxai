#pragma once

#include "rolling_bonxai/logger.hpp"
#include <rclcpp/rclcpp.hpp>

namespace RollingBonxai
{
    class RosLogger : public Logger
    {
    public:
        using SharedPtr = std::shared_ptr<RosLogger>;

        RosLogger(rclcpp::Logger logger) : logger_{logger} {};
        ~RosLogger() override {};

        void log_debug(const std::string& str) override
        {
            RCLCPP_DEBUG(logger_, str.c_str());
        }

        void log_info(const std::string& str) override
        {
            RCLCPP_INFO(logger_, str.c_str());
        }

        void log_warn(const std::string& str)
        {
            RCLCPP_WARN(logger_, str.c_str());
        }

        void log_error(const std::string& str)
        {
            RCLCPP_ERROR(logger_, str.c_str());
        }

        void log_fatal(const std::string& str)
        {
            RCLCPP_FATAL(logger_, str.c_str());
        }
    
    private:
        rclcpp::Logger logger_;
    };
}