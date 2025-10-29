#pragma once

#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>

#include <frc/Joystick.h>
#include <frc/XboxController.h>

#include <frc/filter/SlewRateLimiter.h>

#include "commands/BasicCommands.h"

#include "Constants.h"


class Controller
{
        public:

            static Controller* GetInstance();

        private:
        
            Controller();
        
            std::function<frc::ChassisSpeeds()> GetChassisSpeeds();

            void              ConfigureOperator();

            void              ConfigureDriverControls();
            void              ConfigureDriverJogControls();
        
            double            GetThrottleRange();
            double            GetExponentialValue(double joystickValue, double exponent);
            
            frc::XboxController m_driveController;

            // // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
            // frc::SlewRateLimiter<units::scalar>  m_xspeedLimiter;
            // frc::SlewRateLimiter<units::scalar>  m_yspeedLimiter;
            // frc::SlewRateLimiter<units::scalar> m_rotLimiter;
            
            frc::SlewRateLimiter<units::scalar>             m_flywheelLimiter;

            Swerve*  m_swerve;
            Volcano* m_volcano;
};