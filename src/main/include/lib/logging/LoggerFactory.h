#pragma once

#include <string>
#include <functional>

#include <frc/smartdashboard/SmartDashboard.h>


/* A message to anyone looking at this code:
 *
 * I seriously can't tell if this is a good or a bad way to do this.
 * I wish WPI just had a Log(key, value) function, and I could forget about this.
 * But they hate me personally. So in the wise words of a profound programmer:
 * 
 *    "Why don't you go study an ant colony? You ******* *******. The human species is a goddamn ant colony..."
 *          - Terry A. Davis
 *            (RIP 1969-2018)
*/

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