#ifndef LOGGER_H_
#define LOGGER_H_

#include <iostream>
#include <string>
#include <fmt/core.h>

template <typename LoggerImpl>
class Logger
{
protected:
    std::string logger_name;

public:
    Logger(const std::string &name) : logger_name(name) {}
    template <typename... Args>
    void info(const std::string &format, Args... args)
    {
        static_cast<LoggerImpl *>(this)->log_info(fmt::format(format, args...));
    }

    template <typename... Args>
    void error(const std::string &format, Args... args)
    {
        static_cast<LoggerImpl *>(this)->log_error(fmt::format(format, args...));
    }

    template <typename... Args>
    void warn(const std::string &format, Args... args)
    {
        static_cast<LoggerImpl *>(this)->log_warn(fmt::format(format, args...));
    }
};

#ifndef ROS2
#include <spdlog/spdlog.h>
class SpdlogImpl : public Logger<SpdlogImpl>
{
public:
    SpdlogImpl(const std::string &name) : Logger<SpdlogImpl>(name) {}
    void log_info(const std::string &message)
    {
        spdlog::info("{} {}", logger_name, message);
    }

    void log_error(const std::string &message)
    {
        spdlog::error("{} {}", logger_name, message);
    }

    void log_warn(const std::string &message)
    {
        spdlog::warn("{} {}", logger_name, message);
    }
};
#endif

#ifdef ROS2
#include <rclcpp/rclcpp.hpp>
class RosImpl : public Logger<RosImpl>
{
public:
    RosImpl(const std::string &name) : Logger<RosImpl>(name) {}
    void log_info(const std::string &message)
    {
        RCLCPP_INFO(rclcpp::get_logger(logger_name), message.c_str());
    }

    void log_error(const std::string &message)
    {
        RCLCPP_ERROR(rclcpp::get_logger(logger_name), message.c_str());
    }

    void log_warn(const std::string &message)
    {
        RCLCPP_WARN(rclcpp::get_logger(logger_name), message.c_str());
    }
};
#endif

#ifdef ROS2
using LoggerImpl = RosImpl;
#else
using LoggerImpl = SpdlogImpl;
#endif

#endif /* LOGGER_H_ */