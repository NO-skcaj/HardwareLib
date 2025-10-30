#include "Robot.h"

Robot::Robot()
{
  // Enable LiveWindow in test mode
  EnableLiveWindowInTest(true);

  // Report the robot framework usage
  HAL_Report(HALUsageReporting::kResourceType_Framework, HALUsageReporting::kFramework_RobotBuilder);

  Controller::GetInstance();
}

void Robot::RobotPeriodic()
{
  frc2::CommandScheduler::GetInstance().Run();
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
