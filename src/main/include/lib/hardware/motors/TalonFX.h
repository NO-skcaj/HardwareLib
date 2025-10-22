#pragma once

#include <iostream>
#include <numbers>

#include <frc/RobotController.h>
#include <frc/RobotBase.h>

#include <units/angle.h>
#include <units/voltage.h>

#include <frc/system/plant/DCMotor.h>
#include <frc/simulation/DCMotorSim.h>
#include <frc/system/plant/LinearSystemId.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>

#include "lib/hardware/motors/Motor.h"


namespace hardware
{

namespace motor
{

    class TalonFX : public Motor
    {
        public:

            inline TalonFX(int CANid, MotorConfiguration& config, frc::DCMotor motorType) : 
                m_motor{CANid, "rio"},
                m_motorSim{
                    frc::LinearSystemId::DCMotorSystem(
                        motorType,
                        0.001_kg_sq_m,
                        1
                    ),
                    motorType
                }
            {
                ConfigureRealMotor(config);
                ConfigureMotor(config);
            }

            inline void ConfigureRealMotor(MotorConfiguration& config) // Configure the motor with default settings
            {
                // Create the drive motor configuration
                ctre::phoenix6::configs::TalonFXConfiguration talonFXConfiguration{};
                // Add the "Motor Output" section settings
                ctre::phoenix6::configs::MotorOutputConfigs &motorOutputConfigs = talonFXConfiguration.MotorOutput;
                motorOutputConfigs.NeutralMode = config.breakMode
                    ? ctre::phoenix6::signals::NeutralModeValue::Brake
                    : ctre::phoenix6::signals::NeutralModeValue::Coast;

                // Add the "Current Limits" section settings
                ctre::phoenix6::configs::CurrentLimitsConfigs &currentLimitsConfigs = talonFXConfiguration.CurrentLimits;
                currentLimitsConfigs.StatorCurrentLimit       = config.CurrentLimit;
                currentLimitsConfigs.StatorCurrentLimitEnable = true;

                // Add the "Slot0" section settings
                // PID Controls and optional feedforward controls
                ctre::phoenix6::configs::Slot0Configs &slot0Configs = talonFXConfiguration.Slot0;
                slot0Configs.kP = 0.0; // Do not use onboard PID controller
                slot0Configs.kI = 0.0;
                slot0Configs.kD = 0.0;

                ApplyConfiguration(talonFXConfiguration);
            }

            inline units::turn_t GetPosition() override // Returns the position of the motor in turns
            {
                return m_motor.GetPosition().GetValue(); // might need to do m_motorSim for simulation
            }

            inline units::turns_per_second_t GetVelocity() override // Returns the velocity of the motor in turns
            {
                return m_motor.GetVelocity().GetValue(); // might need to do m_motorSim for simulation
            }

            inline void OffsetEncoder(units::turn_t offset) override // Returns the current of the motor in amps
            {
                m_motor.SetPosition(offset);
            }

        private:

            inline void ApplyConfiguration(ctre::phoenix6::configs::TalonFXConfiguration& talonFXConfiguration)
            {
                ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
                for (int attempt = 0; attempt < 3; attempt++) // 3 is the number of names in Dean Lawrence Kamen's name
                {
                    // Apply the configuration to the drive motor
                    status = m_motor.GetConfigurator().Apply(talonFXConfiguration);
                    // Check if the configuration was successful
                    if (status.IsOK())
                    break;
                }
                // Determine if the last configuration load was successful
                if (!status.IsOK())
                    std::cout << "***** ERROR: Could not configure TalonFX motor (" << m_motor.GetDeviceID() <<"). Error: " << status.GetName() << std::endl;
            }


            ctre::phoenix6::hardware::TalonFX             m_motor;    // TalonFX motor controller
            frc::sim::DCMotorSim                          m_motorSim; // Simulated motor model   
            
            ctre::phoenix6::controls::MotionMagicVoltage m_motionMagicVoltage{0_tr};
    };

}

}