#pragma once

#include <frc2/command/SubsystemBase.h>

#include <wpi/array.h>
#include <units/angle.h>

#include <networktables/NetworkTableInstance.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/StructTopic.h>

#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>

#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveDriveKinematics.h>

#include <frc/estimator/SwerveDrivePoseEstimator.h>

#include "lib/hardware/gyro/Navx.h"
#include "lib/subsystem/SwerveModule.h"

// #include "lib/hardware/vision/PhotonVision.h"

#include "Constants.h"


class Swerve : public frc2::SubsystemBase
{
    public:
    
        static Swerve* GetInstance()
        {
            static Swerve instance;
            static Swerve* instancePtr = &instance;
            return instancePtr;
        }

        inline void Drive(frc::ChassisSpeeds speeds)
        {
            m_loggedDesiredSpeedsPublisher.Set(speeds);

            // Set the module states
            auto m_desiredStates = m_kinematics.ToSwerveModuleStates(
                m_isFieldRelative ? 
                    frc::ChassisSpeeds::FromFieldRelativeSpeeds(speeds, GetHeading()) 
                    : speeds);

            // Set the desired state for each swerve module
            m_swerveModules[0].SetDesiredState(m_desiredStates[0]);
            m_swerveModules[1].SetDesiredState(m_desiredStates[1]);
            m_swerveModules[2].SetDesiredState(m_desiredStates[2]);
            m_swerveModules[3].SetDesiredState(m_desiredStates[3]);
        }

        inline void Periodic() override
        {
            // This method will be called once per scheduler run

            m_loggedModuleStatePublisher.Set(
                wpi::array<frc::SwerveModuleState, 4>{
                    m_swerveModules[0].GetState(),
                    m_swerveModules[1].GetState(),
                    m_swerveModules[2].GetState(),
                    m_swerveModules[3].GetState()
                }
            );

            OdometryPeriodic();
        }

        inline void ResetWheelAnglesToZero()
        {
            // Set the swerve wheel angles to zero
            m_swerveModules[0].SetWheelAngleToForward(constants::swerve::RobotSwerveConfig.frontLeftForwardAngle);
            m_swerveModules[1].SetWheelAngleToForward(constants::swerve::RobotSwerveConfig.frontRightForwardAngle);
            m_swerveModules[2].SetWheelAngleToForward(constants::swerve::RobotSwerveConfig.rearLeftForwardAngle);
            m_swerveModules[3].SetWheelAngleToForward(constants::swerve::RobotSwerveConfig.rearRightForwardAngle);
        }

        inline void ResetDriveEncoders()
        {
            // Reset the swerve motor encoders
            m_swerveModules[0].ResetDriveEncoder();
            m_swerveModules[1].ResetDriveEncoder();
            m_swerveModules[2].ResetDriveEncoder();
            m_swerveModules[3].ResetDriveEncoder();
        }

        inline wpi::array<frc::SwerveModuleState, 4> GetModuleStates()
        {
            // Return the swerve module states
            return {m_swerveModules[0].GetState(),
                    m_swerveModules[1].GetState(),
                    m_swerveModules[2].GetState(),
                    m_swerveModules[3].GetState()};
        }

        std::array<frc::SwerveModulePosition, 4> GetModulePositions()
        {
            // Return the swerve module states
            return {m_swerveModules[0].GetPosition(),
                    m_swerveModules[1].GetPosition(),
                    m_swerveModules[2].GetPosition(),
                    m_swerveModules[3].GetPosition()};
        }

        inline void FlipFieldCentric()
        {
            m_isFieldRelative = !m_isFieldRelative;
        }

        frc::Rotation2d GetHeading()
        {
            return hardware::gyro::Navx::GetInstance()->GetRotation().ToRotation2d();
        }

        frc::Pose2d GetPose2d()
        {
            return m_poseEstimator.GetEstimatedPosition();
        }

    protected:

        explicit Swerve() :
                m_kinematics{
                frc::Translation2d{+constants::swerve::RobotSwerveConfig.wheelBase / 2, +constants::swerve::RobotSwerveConfig.trackWidth / 2}, // Front Left
                frc::Translation2d{+constants::swerve::RobotSwerveConfig.wheelBase / 2, -constants::swerve::RobotSwerveConfig.trackWidth / 2}, // Front Right
                frc::Translation2d{-constants::swerve::RobotSwerveConfig.wheelBase / 2, +constants::swerve::RobotSwerveConfig.trackWidth / 2}, // Back Left
                frc::Translation2d{-constants::swerve::RobotSwerveConfig.wheelBase / 2, -constants::swerve::RobotSwerveConfig.trackWidth / 2}  // Back Right
                },
                m_swerveModules{
                subsystem::SwerveModule{constants::swerve::RobotSwerveConfig.frontLeftDriveCANid,  constants::swerve::RobotSwerveConfig.frontLeftTurnCANid,  constants::swerve::RobotSwerveConfig.frontLeftEncoderCANid,  constants::swerve::RobotSwerveConfig.driveMotorConfig, constants::swerve::RobotSwerveConfig.turnMotorConfig, constants::swerve::RobotSwerveConfig.driveConversion, constants::swerve::RobotSwerveConfig.angleConversion},
                subsystem::SwerveModule{constants::swerve::RobotSwerveConfig.frontRightDriveCANid, constants::swerve::RobotSwerveConfig.frontRightTurnCANid, constants::swerve::RobotSwerveConfig.frontRightEncoderCANid, constants::swerve::RobotSwerveConfig.driveMotorConfig, constants::swerve::RobotSwerveConfig.turnMotorConfig, constants::swerve::RobotSwerveConfig.driveConversion, constants::swerve::RobotSwerveConfig.angleConversion},
                subsystem::SwerveModule{constants::swerve::RobotSwerveConfig.backLeftDriveCANid,   constants::swerve::RobotSwerveConfig.backLeftTurnCANid,   constants::swerve::RobotSwerveConfig.backLeftEncoderCANid,   constants::swerve::RobotSwerveConfig.driveMotorConfig, constants::swerve::RobotSwerveConfig.turnMotorConfig, constants::swerve::RobotSwerveConfig.driveConversion, constants::swerve::RobotSwerveConfig.angleConversion},
                subsystem::SwerveModule{constants::swerve::RobotSwerveConfig.backRightDriveCANid,  constants::swerve::RobotSwerveConfig.backRightTurnCANid,  constants::swerve::RobotSwerveConfig.backRightEncoderCANid,  constants::swerve::RobotSwerveConfig.driveMotorConfig, constants::swerve::RobotSwerveConfig.turnMotorConfig, constants::swerve::RobotSwerveConfig.driveConversion, constants::swerve::RobotSwerveConfig.angleConversion}
                },
                m_poseEstimator{m_kinematics, frc::Rotation2d(), std::array<frc::SwerveModulePosition, 4>{}, frc::Pose2d()},
                m_isFieldRelative{true},
                
                m_loggedModuleStatePublisher{nt::NetworkTableInstance::GetDefault().GetStructArrayTopic<frc::SwerveModuleState>("/Data/SwerveStates").Publish()},
                m_loggedPosePublisher{nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose2d>("/Data/CurrentPose").Publish()},
                m_loggedDesiredSpeedsPublisher{nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::ChassisSpeeds>("/Data/DesiredSpeeds").Publish()}
        {
            // Set the swerve modules to their forward angles
            ResetWheelAnglesToZero();
        }

        void OdometryPeriodic()
        {
            // Update the pose estimator
            m_poseEstimator.Update(
                GetHeading(),
                GetModulePositions()
            );

            // // Update with vision measurements if available
            // std::optional<std::pair<frc::Pose2d, units::second_t>> visionPose = PhotonVision::GetInstance()->GetResult();
            // if (visionPose)
            //     m_poseEstimator.AddVisionMeasurement(visionPose.value().first, visionPose.value().second, PhotonVision::GetInstance()->GetEstimationStdDevs(visionPose.value().first));
        }

        frc::SwerveDriveKinematics<4>           m_kinematics;

        std::array<subsystem::SwerveModule, 4>  m_swerveModules;
        
        frc::SwerveDrivePoseEstimator<4>        m_poseEstimator;   

        bool                                    m_isFieldRelative;

        nt::StructArrayPublisher<frc::SwerveModuleState> m_loggedModuleStatePublisher;
        nt::StructPublisher<frc::Pose2d>                 m_loggedPosePublisher;
        nt::StructPublisher<frc::ChassisSpeeds>          m_loggedDesiredSpeedsPublisher;

};