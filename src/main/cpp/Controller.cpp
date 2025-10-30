#include "Controller.h"


Controller* Controller::GetInstance()
{
    static Controller controller;
    static Controller* controllerPtr;
    return controllerPtr;
}

/// @brief Constructor for the DriveController class.
/// @param gripper reference to the gripper subsystem.
Controller::Controller() 
    : m_driveController   {constants::controller::DrivePort},

      m_flywheelLimiter   {1.0 / 0.5_s}, // Full throttle change in 0.5 seconds

      m_swerve {Swerve::GetInstance()},
      m_volcano{Volcano::GetInstance()}
{
    m_swerve ->SetDefaultCommand(ChassisDrive(m_swerve, GetChassisSpeeds() ));
    m_volcano->SetDefaultCommand(VolcanoIndexerControl(m_volcano, [this] { return m_flywheelLimiter.Calculate(frc::ApplyDeadband(m_driveController.GetRightTriggerAxis(), constants::controller::FlywheelDeadZone)).value(); } ));

    // Configure the operator controller
    std::pair<Button, frc2::CommandPtr> runOnceControls[] = {
        {constants::controller::A,           ChassisZeroHeading(Gyro::GetInstance())},
        {constants::controller::B,           FlipFieldCentricity(m_swerve)},
        {constants::controller::RightBumper, VolcanoFlywheelOn(m_volcano)},
        {constants::controller::LeftBumper,  VolcanoFlywheelOff(m_volcano)}
    };

    for (auto& [button, command] : runOnceControls)
    {
        frc2::JoystickButton (&m_driveController, int(button))
            .OnTrue(std::move(command));
    }
}

std::function<frc::ChassisSpeeds()> Controller::GetChassisSpeeds()
{
    return [this] () -> frc::ChassisSpeeds
    {
        // Return the x speed
        return frc::ChassisSpeeds{
            -constants::swerve::RobotSwerveConfig.maxSpeed           * frc::ApplyDeadband(m_driveController.GetRawAxis(1), constants::controller::TranslationDeadZone),
            -constants::swerve::RobotSwerveConfig.maxSpeed           * frc::ApplyDeadband(m_driveController.GetRawAxis(0),  constants::controller::TranslationDeadZone),
             constants::swerve::RobotSwerveConfig.maxAngularVelocity * frc::ApplyDeadband(m_driveController.GetRawAxis(4), constants::controller::RotateDeadZone)
        };
    };
}

/// @brief Method to convert a joystick value from -1.0 to 1.0 to exponential mode.
/// @param joystickValue The raw joystick value.
/// @param exponent The exponential value.
/// @return The resulting exponential value.
double Controller::GetExponentialValue(double joystickValue, double exponent)
{
    int    direction = (joystickValue < 0.0) ? -1 : 1;
    double absValue  = std::abs(joystickValue);
    double output    = std::pow(absValue, exponent) * direction;

    // Ensure the range of the output
    if (output < -1.0) output = -1.0;
    if (output > 1.0)  output = 1.0;

    // Return the output value
    return output;
}