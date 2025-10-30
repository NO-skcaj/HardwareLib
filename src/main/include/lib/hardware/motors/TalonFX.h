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

            inline TalonFX(CANid_t CANid, MotorConfiguration config, frc::DCMotor motorModel, units::kilogram_square_meter_t simMomentOfIntertia = 0.001_kg_sq_m) 
                : Motor{frc::sim::DCMotorSim{
                    frc::LinearSystemId::DCMotorSystem(
                            motorModel,
                            simMomentOfIntertia,
                            1
                        ),
                        motorModel
                    }
                  },
                  m_motor{CANid, "rio"}
            {
                ConfigureMotor(config);
            }

            inline void ConfigureMotor(MotorConfiguration config) override // Configure the motor with default settings
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
                slot0Configs.kP = config.P;
                slot0Configs.kI = config.I;
                slot0Configs.kD = config.D;
                slot0Configs.kS = config.S;
                slot0Configs.kV = config.V;
                slot0Configs.kA = config.A;

                // Try to apply the configuration multiple times in case of failure
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

            inline void SetReferenceState(double motorInput) override // output to motor within (-1,1)
            {
                // Set the motor speed and angle
                m_motor.Set(motorInput);

                m_motorSim.SetInputVoltage(motorInput * frc::RobotController::GetBatteryVoltage());
            }

            inline void SetReferenceState(units::turns_per_second_t motorInput) override // output to motor within (-1,1)
            {
                // Set the motor speed and angle
                m_motor.SetControl(ctre::phoenix6::controls::VelocityVoltage(motorInput));

                m_motorSim.SetAngularVelocity(units::radians_per_second_t{motorInput.value()});
            }

            inline void SetReferenceState(units::volt_t motorInput) override // output to motor within (-1,1)
            {
                // Set the motor speed and angle
                m_motor.SetVoltage(motorInput);

                m_motorSim.SetInputVoltage(motorInput);
            }

            inline void SetReferenceState(units::turn_t motorInput) override // output to motor in turns
            {
                // Set the arm set position
                m_motor.SetControl(m_motionMagicVoltage.WithPosition(motorInput).WithSlot(0));

                m_motorSim.SetAngle(units::radian_t{motorInput.value()});
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

            inline void SimPeriodic() override
            {
                auto& talonFXSim = m_motor.GetSimState();

                // Set the supply voltage of the TalonFX
                talonFXSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

                // Get the motor voltage of the TalonFX
                auto motorVoltage = talonFXSim.GetMotorVoltage();

                // Use the motor voltage to calculate new position and velocity
                m_motorSim.SetInputVoltage(motorVoltage);
                m_motorSim.Update(20_ms);

                // Apply the new rotor position and velocity to the TalonFX
                talonFXSim.SetRawRotorPosition(units::turn_t{m_motorSim.GetAngularPosition().value()});
                talonFXSim.SetRotorVelocity(units::turns_per_second_t{m_motorSim.GetAngularVelocity().value()});
            }

        private:

            ctre::phoenix6::hardware::TalonFX             m_motor;    // TalonFX motor controller
            
            ctre::phoenix6::controls::MotionMagicVoltage m_motionMagicVoltage{0_tr};
    };

}

}