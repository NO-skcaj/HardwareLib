#pragma once

#include <string>
#include <functional>

#include <frc/smartdashboard/SmartDashboard.h>


namespace LoggerFactory
{
    inline std::function<void()> CreateLoggedValue(std::string_view name, double* value)
    {
        return [name, value]() {frc::SmartDashboard::PutNumber(name, *value);};
    }

    inline std::function<void()> CreateLoggedValue(std::string_view name, std::string_view* value)
    {
        return [name, value]() {frc::SmartDashboard::PutString(name, *value);};
    }

    inline std::function<void()> CreateLoggedValue(std::string_view name, bool* value)
    {
        return [name, value]() {frc::SmartDashboard::PutBoolean(name, *value);};
    }
};