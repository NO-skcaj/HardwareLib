#pragma once

#include "Motor.h"

#include <iostream>
#include <numbers>

#include <frc/system/plant/DCMotor.h>
#include <frc/simulation/DCMotorSim.h>
#include <frc/system/plant/LinearSystemId.h>

#include <frc/controller/SimpleMotorFeedforward.h>

#include <rev/SparkMax.h>
#include <rev/sim/SparkMaxSim.h>
#include <rev/SparkLowLevel.h>
#include <rev/config/SparkMaxConfig.h>


namespace hardware
{

namespace motor
{
    class SparkMax : public Motor
    {
        public:

            inline SparkMax(CANid_t CANid, MotorConfiguration config, frc::DCMotor motorModel, units::kilogram_square_meter_t simMomentOfIntertia = 0.001_kg_sq_m) 
            : Motor{frc::sim::DCMotorSim{
                frc::LinearSystemId::DCMotorSystem(
                        motorModel,
                        simMomentOfIntertia,
                        1
                    ),
                    motorModel
                }
              },
              m_motor{CANid, config.breakMode
                              ? rev::spark::SparkLowLevel::MotorType::kBrushless 
                              : rev::spark::SparkLowLevel::MotorType::kBrushed},
              m_angleEncoder{m_motor.GetEncoder()}, 
              m_turnClosedLoopController{m_motor.GetClosedLoopController()},

              m_feedforward{config.S * 1_V, config.V * 1_V * 1_s / 1_tr, config.A * 1_V * 1_s * 1_s / 1_tr},

              m_motorModel{motorModel},
              m_sparkSim{&m_motor, &m_motorModel}
            {
                ConfigureMotor(config);
            }

            inline void ConfigureMotor(MotorConfiguration config) override // Configure the motor with default settings
            {
                // Configure the angle motor
                rev::spark::SparkMaxConfig sparkMaxConfig{};

                // Configure the motor controller
                sparkMaxConfig
                    .SetIdleMode(config.breakMode 
                                        ? rev::spark::SparkBaseConfig::IdleMode::kBrake 
                                        : rev::spark::SparkBaseConfig::IdleMode::kCoast)
                    .SmartCurrentLimit(config.CurrentLimit.value());
                    
                // All you have to do for encoders is make sure that they are in turns. If a particular motor only does counts... submit a pull request.
                sparkMaxConfig.encoder // Do not set counts per second, only counts in brushed motors (ie cims)
                    .PositionConversionFactor(1) // in turns (This will have decimals precision)
                    .VelocityConversionFactor(1); // in turns per second

                // Configure the closed loop controller
                sparkMaxConfig.closedLoop
                    .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
                    .Pid(config.P, config.I, config.D);

                // Write the configuration to the motor controller
                m_motor.Configure(sparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
            }

            // MAJOR TODO: whenever the REV library updates, change SetReference (deprecated) to SetSetpoint

            inline void SetReferenceState(double motorInput) override // output to motor within (-1,1)
            {
                m_turnClosedLoopController.SetReference(motorInput, 
                                                        rev::spark::SparkMax::ControlType::kDutyCycle,
                                                        rev::spark::ClosedLoopSlot::kSlot0, 
                                                        m_feedforward.Calculate(0_tps).value(),
                                                        rev::spark::SparkClosedLoopController::ArbFFUnits::kVoltage);
            }

            inline void SetReferenceState(units::turns_per_second_t motorInput) override // output to motor within (-1,1)
            {
                m_turnClosedLoopController.SetReference(motorInput.value(), 
                                                        rev::spark::SparkMax::ControlType::kVelocity,
                                                        rev::spark::ClosedLoopSlot::kSlot0, 
                                                        m_feedforward.Calculate(motorInput).value(),
                                                        rev::spark::SparkClosedLoopController::ArbFFUnits::kVoltage);
            }

            inline void SetReferenceState(units::volt_t motorInput) override // output to motor within (-1,1)
            {
                m_turnClosedLoopController.SetReference(motorInput.value() + m_feedforward.Calculate(0_tps).value(), // just add the static value
                                                        rev::spark::SparkMax::ControlType::kVoltage);
            }

            inline void SetReferenceState(units::turn_t motorInput) override // output to motor in turns
            {
                m_turnClosedLoopController.SetReference(motorInput.value(), 
                                                        rev::spark::SparkMax::ControlType::kPosition,
                                                        rev::spark::ClosedLoopSlot::kSlot0, 
                                                        m_feedforward.Calculate(units::turns_per_second_t{GetPosition().value() - motorInput.value()}).value(),
                                                        rev::spark::SparkClosedLoopController::ArbFFUnits::kVoltage);
            }

            inline units::turn_t GetPosition() override // Returns the position of the motor in turns
            {
                return units::turn_t{m_angleEncoder.GetPosition()};
            }

            inline units::turns_per_second_t GetVelocity() override // Returns the velocity of the motor in turn velocity
            {
                return units::turns_per_second_t{m_angleEncoder.GetVelocity() / 60}; // return in rotations per MINUTE???
            }

            inline void OffsetEncoder(units::turn_t offset) override
            {
                m_angleEncoder.SetPosition(offset.value());
            }

            inline void SimPeriodic() override
            {
                m_motorSim.SetInputVoltage(m_sparkSim.GetAppliedOutput() * frc::RobotController::GetBatteryVoltage());
                m_motorSim.Update(0.02_s);
                m_sparkSim.iterate(m_motorSim.GetAngularVelocity().value(), frc::RobotController::GetBatteryVoltage().value(), 0.02);
            }

        private:

            rev::spark::SparkMax                  m_motor;                    // SparkMax motor controller
            rev::spark::SparkRelativeEncoder      m_angleEncoder;             // Relative encoder onboard the sparkmax
            rev::spark::SparkClosedLoopController m_turnClosedLoopController; // PID Controller for SparkMax

            frc::SimpleMotorFeedforward<units::turns> m_feedforward; // Feedforward controller for the motor

            frc::DCMotor                          m_motorModel; // Type of motor attached to the SparkMax
            rev::spark::SparkMaxSim               m_sparkSim;   // Simulated SparkMax model

    };
}

}