#pragma once

#include <frc/smartdashboard/SmartDashboard.h>
#include <typeinfo>


/* A message to anyone looking at this code:
 *
 * I seriously can't tell if this is a good or a bad way to do this.
 * I wish WPI just had a Log(key, value) function, and I could forget about this.
 * But they hate me personally. So in the wise words of a profound programmer:
 * 
 *    "Why don't you go study an ant colony? You ******* *******. The human species is a goddamn ant colony, okay? You've got to separate the... because they're different species of ants, okay?"
 *          - Terry A. Davis
 *            (RIP 1969-2018)
 * 
 *  You're not gonna believe what they added...
 *  Nevermind, I'm just gonna email Peter Johnson
*/


void log(std::string_view name, double value)
{
    frc::SmartDashboard::PutNumber(name, value);
}

void log(std::string_view name, bool value)
{
    frc::SmartDashboard::PutBoolean(name, value);
}

void log(std::string_view name, std::string_view value)
{
    frc::SmartDashboard::PutString(name, value);
}