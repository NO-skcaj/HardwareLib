#include "Robot.h"


Robot::Robot() 
  : m_testMotor{1, 
                hardware::motor::MotorConfiguration{30_A, true, 
                1.0, 0.0, 0.0, 
                0.0, 0.0, 0.0}, 
                frc::DCMotor::NEO()
  },
  m_driverController{0}
{
}

void Robot::RobotPeriodic()
{
  m_testMotor.SetReferenceState(units::turns_per_second_t{m_driverController.GetRawAxis(1)});
  m_testMotor.Periodic();
  frc::SmartDashboard::PutNumber("Test Motor Position", m_testMotor.GetPosition().value());
  frc::SmartDashboard::PutNumber("Test Motor Velocity", m_testMotor.GetVelocity().value());
  frc::SmartDashboard::PutNumber("Input", m_driverController.GetRawAxis(1));
  frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {}

void Robot::TestInit() {
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
