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

            inline TalonFX(int CANid, MotorConfiguration config, frc::DCMotor motorModel) : 
                m_motor{CANid, "rio"},
                m_motorSim{
                    frc::LinearSystemId::DCMotorSystem(
                        motorModel,
                        0.001_kg_sq_m,
                        1
                    ),
                    motorModel
                }
            {
                ConfigureRealMotor(config);
                ConfigureMotor(config);
            }

            inline void ConfigureRealMotor(MotorConfiguration config) // Configure the motor with default settings
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
                slot0Configs.kP = 1.0; // Do not use onboard PID controller
                slot0Configs.kI = 0.0;
                slot0Configs.kD = 0.0;

                ApplyConfiguration(talonFXConfiguration);
            }

            inline units::turn_t GetPosition() override // Returns the position of the motor in turns
            {
                if (frc::RobotBase::IsSimulation())
                {
                    return units::turn_t{m_motorSim.GetAngularPosition().value()};
                }
                return m_motor.GetPosition().GetValue();
            }

            inline units::turns_per_second_t GetVelocity() override // Returns the velocity of the motor in turns
            {
                if (frc::RobotBase::IsSimulation())
                {
                    return units::turns_per_second_t{m_motorSim.GetAngularVelocity().value()};
                }
                return m_motor.GetVelocity().GetValue();
            }

            inline void OffsetEncoder(units::turn_t offset) override // Returns the current of the motor in amps
            {
                m_motor.SetPosition(offset);
            }

        private:

            void SetVoltage(units::volt_t voltage) override // sets the voltage to the motor
            {
                m_motor.SetVoltage(voltage);
                m_motorSim.SetInputVoltage(voltage);
            }

            void SimPeriodic() override
            {
                auto& talonFXSim = m_motor.GetSimState();

                // Set the supply voltage of the TalonFX
                talonFXSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

                // Get the motor voltage of the TalonFX
                auto motorVoltage = talonFXSim.GetMotorVoltage();

                // Use the motor voltage to calculate new position and velocity
                m_motorSim.SetInputVoltage(motorVoltage);
                m_motorSim.Update(20_ms); // Assume 20 ms loop time

                // Apply the new rotor position and velocity to the TalonFX
                double gearRatio = 1.0; // Replace with your actual gear ratio
                talonFXSim.SetRawRotorPosition(m_motorSim.GetAngularPosition() * gearRatio);
                talonFXSim.SetRotorVelocity(m_motorSim.GetAngularVelocity() * gearRatio);
            }

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