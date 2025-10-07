#pragma once

#include <frc2/command/SubsystemBase.h>

#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>

#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>

#include <frc/SwerveDriveKinematics.h>

#include "lib/subsystem/SwerveModule.h"


class Swerve : public frc2::SubsystemBase
{
    public:


        void Drive(frc::ChassisSpeeds speeds, bool fieldRelative = true);
        {
            // Set the module states
            m_desiredStates = m_kinematics.ToSwerveModuleStates(
                centricity ? 
                    frc::ChassisSpeeds::FromFieldRelativeSpeeds(speeds, m_gyro->GetRotation().ToRotation2d()) 
                    : speeds);
        }

        void Periodic() override
        {
            // This method will be called once per scheduler run
            // Set the desired state for each swerve module
            frc::SwerveDriveKinematics<4>::DesaturateWheelSpeeds(m_desiredStates, 3_mps);
            for (const auth& swerveModule : m_swerveModules)
            {
                swerveModule.SetDesiredState(m_desiredStates[i]);
            }
        }

        virtual frc::Rotation2d GetHeading() {};
        
    private:

        Swerve(int frontLeftDriveCANid,  int frontLeftTurnCANid,  int frontLeftEncoderCANid,  
               int frontRightDriveCANid, int frontRightTurnCANid, int frontRightEncoderCANid, 
               int backLeftDriveCANid,   int backLeftTurnCANid,   int backLeftEncoderCANid,   
               int backRightDriveCANid,  int backRightTurnCANid,  int backRightEncoderCANid
               units::meter_t driveConversion, units::radian_t angleConversion,
               units::meter_t wheelBase,       units::meter_t  trackWidth)
        {

        }

        wpi::array<frc::SwerveModuleState, 4> m_desiredStates;
        wpi::array<SwerveModule, 4>           m_swerveModules;
        frc::SwerveDriveKinematics<4>         m_kinematics;

};