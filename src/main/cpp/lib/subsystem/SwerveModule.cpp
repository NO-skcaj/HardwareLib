#include "lib/subsystem/SwerveModule.h"

using namespace subsystem;

/// @brief Class constructor for the SwerveModule class.
/// @param driveMotorCanId The CAN ID for the swerve module drive motor.
/// @param angleMotorCanId The CAN ID for the swerve module angle motor.
/// @param angleEncoderCanId The CAN ID for the swerve module angle encoder.
/// @param turnConfig The motor configuration for the angle motor.
/// @param driveConfig The motor configuration for the drive motor.
/// @param driveMotorConversion The conversion factor for the drive motor (wheel circumference / (gear ratio * motor revolutions)).
/// @param angleMotorConversion The conversion factor for the angle motor (motor revolutions / (2 * pi)).
SwerveModule::SwerveModule(int driveMotorCanId, int angleMotorCanId, int angleEncoderCanId, 
                           hardware::motor::MotorConfiguration turnConfig, hardware::motor::MotorConfiguration driveConfig,
                           units::meter_t  driveMotorConversion,
                           units::radian_t angleMotorConversion)
    :   m_driveMotor          {driveMotorCanId, driveConfig, frc::DCMotor::KrakenX60()},
        m_angleMotor          {angleMotorCanId, turnConfig,  frc::DCMotor::KrakenX60()},
        m_angleAbsoluteEncoder{angleEncoderCanId},
        m_driveConversion{driveMotorConversion},
        m_angleConversion{angleMotorConversion}

{
    // Ensure the drive motor encoder is reset to zero
    m_driveMotor.SetReferenceState(0_tr);
}

/// @brief Method to set the swerve module state to the desired state.
/// @param desiredState The desired swerve module velocity and angle.
/// @param description String to show the module state on the SmartDashboard.
void SwerveModule::SetDesiredState(frc::SwerveModuleState& desiredState)
{
    // Optimize the reference state to avoid spinning further than 90 degrees.
    desiredState.Optimize(GetPosition().angle);
    // Some WPI magic math cosine to prevent jittering
    desiredState.speed = units::meters_per_second_t{desiredState.speed.value() * std::cos(desiredState.angle.Radians().value() - GetPosition().angle.Radians().value())};

    // Set the motor speed and angle
    if (frc::RobotBase::IsSimulation())
    {
        m_driveMotor.SetReferenceState(units::turns_per_second_t(desiredState.speed.value()));

        m_angleMotor.SetReferenceState(units::turn_t(desiredState.angle.Radians().value()));
    } else
    {
        m_driveMotor.SetReferenceState(units::turns_per_second_t(desiredState.speed.value() / m_driveConversion.value()));

        m_angleMotor.SetReferenceState(units::turn_t(desiredState.angle.Radians().value() / m_angleConversion.value()));
    }
}

/// @brief  Method to retrieve the swerve module state.
/// @return The swerve module speed and angle state.
frc::SwerveModuleState SwerveModule::GetState()
{
    units::meters_per_second_t driveVelocity{0.0};
    units::radian_t            anglePosition{0.0};

    // Determine the module wheel velocity
    if (frc::RobotBase::IsSimulation())
    {
        driveVelocity = units::meters_per_second_t{m_driveMotor.GetVelocity().value()};

        anglePosition = units::radian_t{m_angleMotor.GetPosition().value()};
    } else
    {
        driveVelocity = units::meters_per_second_t{m_driveMotor.GetVelocity().value() * m_driveConversion.value()};

        anglePosition = units::radian_t{m_angleMotor.GetPosition().value() * m_angleConversion.value()};
    }
    // Return the swerve module state
    return {driveVelocity, anglePosition};
}

/// @brief Method to retrieve the swerve module position.
frc::SwerveModulePosition SwerveModule::GetPosition()
{
    units::meter_t  drivePosition{0};
    units::radian_t anglePosition{0};
    
    // Determine the module wheel position
    if (frc::RobotBase::IsSimulation())
    {
        drivePosition = m_driveMotor.GetPosition().value() * 1.0_m; // Conversion factor is 1 in sim

        anglePosition = units::radian_t{m_angleMotor.GetPosition().value()};

        // Return the swerve module position
        return {drivePosition, anglePosition};
    } else
    {
        drivePosition = m_driveMotor.GetPosition().value() * m_driveConversion;

        anglePosition = units::radian_t{m_angleMotor.GetPosition().value() * m_angleConversion};
    }

    // Return the swerve module position
    return {drivePosition, anglePosition};
}

// Reset the drive encoder position.
void SwerveModule::ResetDriveEncoder()
{
    m_driveMotor.SetReferenceState(0_tr);
}

/// @brief Method to set the swerve wheel encoder to the forward angle.
/// @param forwardAngle The absolute angle for the forward direction.
void SwerveModule::SetWheelAngleToForward(units::angle::radian_t forwardAngle)
{
    if (frc::RobotBase::IsSimulation()) return;

    // Ensure the drive motor encoder is reset to zero
    m_driveMotor.SetReferenceState(0_tr);

    // Set the motor angle encoder position to the forward direction
    m_angleMotor.OffsetEncoder(units::turn_t((GetAbsoluteEncoderAngle().value() - forwardAngle.value()) / m_angleConversion.value()));

    // Set the motor angle to the forward direction
    m_angleMotor.SetReferenceState(0.0_tr);
}

/// @brief Method to read the absolute encode in radians.
/// @return The absolute angle value in radians.
units::angle::radian_t SwerveModule::GetAbsoluteEncoderAngle()
{
    // The GetAbsolutePosition() method returns a value from -1 to 1
    double encoderValue = m_angleAbsoluteEncoder.GetTurns().value();

    // To convert to radians
    return encoderValue * (2.0_rad * std::numbers::pi);
}