#pragma once

#include <frc2/command/SubsystemBase.h>

#include <wpi/array.h>
#include <units/angle.h>

#include <networktables/NetworkTableInstance.h>
#include <networktables/StructArrayTopic.h>

#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>

#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>

#include <frc/SwerveDriveKinematics.h>

#include "lib/hardware/gyro/Gyro.h"
#include "lib/subsystem/SwerveModule.h"


namespace subsystem
{
    struct SwerveConfiguration
    {
        // In order to access the motors and encoders, we need their CAN IDs
        int frontLeftDriveCANid;
        int frontLeftTurnCANid;
        int frontLeftEncoderCANid;

        int frontRightDriveCANid;
        int frontRightTurnCANid;
        int frontRightEncoderCANid; 

        int backLeftDriveCANid;
        int backLeftTurnCANid; 
        int backLeftEncoderCANid;  

        int backRightDriveCANid;
        int backRightTurnCANid;
        int backRightEncoderCANid;

        // PID, feedforward, and other configurations for the motors
        hardware::motor::MotorConfiguration driveMotorConfig; 
        hardware::motor::MotorConfiguration turnMotorConfig;

        // All encoders are going to be slightly off, this corrects that
        units::radian_t frontLeftForwardAngle;
        units::radian_t frontRightForwardAngle;
        units::radian_t rearLeftForwardAngle;
        units::radian_t rearRightForwardAngle;

        // These make sure to limit how fast the robot can go
        units::meters_per_second_t  maxSpeed; 
        units::radians_per_second_t maxAngularVelocity;

        // Conversion factors for the motors from encoders to actual movement
        units::meter_t  driveConversion;
        units::radian_t angleConversion;
        
        // The physical dimensions of the robot
        units::meter_t wheelBase;
        units::meter_t trackWidth;
    };

    class Swerve : public frc2::SubsystemBase
    {
        public:

            static Swerve* GetInstance()
            {
                static Swerve instance;
                return &instance;
            }

            void Drive(frc::ChassisSpeeds speeds, bool fieldRelative = true)
            {
                // Set the module states
                m_desiredStates = m_kinematics.ToSwerveModuleStates(
                    fieldRelative ? 
                        frc::ChassisSpeeds::FromFieldRelativeSpeeds(speeds, GetHeading()) 
                        : speeds);
            }

            void Periodic() override
            {
                // This method will be called once per scheduler run
                // Set the desired state for each swerve module
                frc::SwerveDriveKinematics<4>::DesaturateWheelSpeeds(m_desiredStates, 3_mps);
                m_swerveModules[0].SetDesiredState(m_desiredStates[0]);
                m_swerveModules[1].SetDesiredState(m_desiredStates[1]);
                m_swerveModules[2].SetDesiredState(m_desiredStates[2]);
                m_swerveModules[3].SetDesiredState(m_desiredStates[3]);

                if (frc::RobotBase::IsSimulation())
                {
                    for (auto& module : m_swerveModules)
                    {
                        module.SimPeriodic();
                    }
                }
            }

            void SetWheelAnglesToZero()
            {
                // We don't want to do this in sim; there isnt actual motors
                if (frc::RobotBase::IsSimulation()) return;

                auto config = GetSwerveConfig();

                // Set the swerve wheel angles to zero
                m_swerveModules[0].SetWheelAngleToForward(config.frontLeftForwardAngle);
                m_swerveModules[1].SetWheelAngleToForward(config.frontRightForwardAngle);
                m_swerveModules[2].SetWheelAngleToForward(config.rearLeftForwardAngle);
                m_swerveModules[3].SetWheelAngleToForward(config.rearRightForwardAngle);
            }

            void ResetDriveEncoders()
            {
                // Reset the swerve motor encoders
                m_swerveModules[0].ResetDriveEncoder();
                m_swerveModules[1].ResetDriveEncoder();
                m_swerveModules[2].ResetDriveEncoder();
                m_swerveModules[3].ResetDriveEncoder();
            }

            wpi::array<frc::SwerveModuleState, 4>    GetModuleStates()
            {
                // Return the swerve module states
                return {m_swerveModules[0].GetState(),
                        m_swerveModules[1].GetState(),
                        m_swerveModules[2].GetState(),
                        m_swerveModules[3].GetState()};
            }

            wpi::array<frc::SwerveModulePosition, 4> GetModulePositions()
            {
                return {m_swerveModules[0].GetPosition(),
                        m_swerveModules[1].GetPosition(),
                        m_swerveModules[2].GetPosition(),
                        m_swerveModules[3].GetPosition()};
            }

            virtual frc::Rotation2d GetHeading() = 0;

            virtual frc::Pose2d     GetPose2d() = 0;

        private:

            virtual SwerveConfiguration* GetSwerveConfig() = 0;

            Swerve()
                : m_config{GetSwerveConfig()},
                  m_swerveModules{
                    SwerveModule{config.frontLeftDriveCANid,  config.frontLeftTurnCANid,  config.frontLeftEncoderCANid,  config.driveConversion, config.angleConversion, config.driveMotorConfig, config.turnMotorConfig},
                    SwerveModule{config.frontRightDriveCANid, config.frontRightTurnCANid, config.frontRightEncoderCANid, config.driveConversion, config.angleConversion, config.driveMotorConfig, config.turnMotorConfig},
                    SwerveModule{config.backLeftDriveCANid,   config.backLeftTurnCANid,   config.backLeftEncoderCANid,   config.driveConversion, config.angleConversion, config.driveMotorConfig, config.turnMotorConfig},
                    SwerveModule{config.backRightDriveCANid,  config.backRightTurnCANid,  config.backRightEncoderCANid,  config.driveConversion, config.angleConversion, config.driveMotorConfig, config.turnMotorConfig}
                  },
                  m_kinematics{
                    frc::Translation2d{+config.wheelBase / 2, +config.trackWidth / 2}, // Front Left
                    frc::Translation2d{+config.wheelBase / 2, -config.trackWidth / 2}, // Front Right
                    frc::Translation2d{-config.wheelBase / 2, +config.trackWidth / 2}, // Back Left
                    frc::Translation2d{-config.wheelBase / 2, -config.trackWidth / 2}  // Back Right
                },
                m_loggedModuleStatePublisher{nt::NetworkTableInstance::GetDefault()
                .GetStructArrayTopic<frc::SwerveModuleState>("/Data/SwerveStates").Publish()}
            {}

            SwerveConfiguration*                  m_config;

            wpi::array<frc::SwerveModuleState, 4> m_desiredStates;
            wpi::array<subsystem::SwerveModule, 4>           m_swerveModules;
            frc::SwerveDriveKinematics<4>         m_kinematics;

            nt::StructArrayPublisher<frc::SwerveModuleState> m_loggedModuleStatePublisher;

    };

}