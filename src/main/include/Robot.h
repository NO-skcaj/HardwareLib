// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <hal/FRCUsageReporting.h>

#include "lib/Logging.h"
#include "Controller.h"

class Robot : public frc::TimedRobot 
{

  public:

    Robot();
    
    void RobotPeriodic() override;
    
};
