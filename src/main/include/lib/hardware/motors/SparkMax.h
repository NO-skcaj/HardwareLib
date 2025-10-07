#pragma once

#include "Motor.h"

#include <iostream>

#include <frc/system/plant/DCMotor.h>

#include <rev/SparkSim.h>

#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>


namespace hardware
{

namespace motor
{

    // needs feedforward and motion magic support
    class SparkMax : public Motor
    {
        public:
            inline SparkMax(int CANid, bool isBrushless) 
                :   m_motor{CANid, isBrushless ? rev::spark::SparkMax::MotorType::kBrushless : rev::spark::SparkMax::MotorType::kBrushed},
                    m_angleEncoder{m_motor.GetEncoder()}, 
                    m_turnClosedLoopController{m_motor.GetClosedLoopController()}
                    // m_motorSim{m_motor, frc::DCMotor::NEO(1)}
            {}

            inline SparkMax(int CANid, bool isBrushless, MotorConfiguration config) : 
                m_motor{CANid, isBrushless ? rev::spark::SparkMax::MotorType::kBrushless : rev::spark::SparkMax::MotorType::kBrushed},
                m_angleEncoder{m_motor.GetEncoder()}, 
                m_turnClosedLoopController{m_motor.GetClosedLoopController()}
                // m_motorSim{m_motor, frc::DCMotor::NEO(1)}
            {
                ConfigureMotor(config);
            }

            inline void ConfigureMotor(MotorConfiguration config) override // Configure the motor with default settings
            {
                // Configure the angle motor
                rev::spark::SparkMaxConfig sparkMaxConfig{};

                // Configure the motor controller
                sparkMaxConfig
                    .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
                    .SmartCurrentLimit(config.CurrentLimit.value());

                // sparkMaxConfig.encoder
                //     .PositionConversionFactor(config.PositionConversionFactor)
                //     .VelocityConversionFactor(config.VelocityConversionFactor); // not using these as to stay consistent with the TalonFX vice versa

                // Configure the closed loop controller
                sparkMaxConfig.closedLoop
                    .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
                    .Pid(config.PID.P, config.PID.I, config.PID.D);

                // Write the configuration to the motor controller
                m_motor.Configure(sparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
            }

            inline void SetSpeed(double motorInput) override // output to motor within (-1,1)
            {
                m_turnClosedLoopController.SetReference(motorInput, rev::spark::SparkMax::ControlType::kDutyCycle);
            }

            inline void SetSpeed(units::turns_per_second_t motorInput) override // output to motor within (-1,1)
            {
                m_turnClosedLoopController.SetReference(motorInput.value(), rev::spark::SparkMax::ControlType::kVelocity);
            }

            inline void SetSpeed(units::volt_t motorInput) override // output to motor within (-1,1)
            {
                m_turnClosedLoopController.SetReference(motorInput.value(), rev::spark::SparkMax::ControlType::kVoltage);
            }

            inline void SetPosition(units::turn_t motorInput) override // output to motor in turns
            {
                m_turnClosedLoopController.SetReference(motorInput.value(), rev::spark::SparkMax::ControlType::kPosition);
            }

            inline units::turn_t GetPosition() override // Returns the position of the motor in turns
            {
                return units::turn_t{m_angleEncoder.GetPosition()};
            }

            inline units::turns_per_second_t GetVelocity() override // Returns the velocity of the motor in turns
            {
                return units::turns_per_second_t{m_angleEncoder.GetVelocity()};
            }

            inline void OffsetEncoder(units::turn_t offset) override // Returns the current of the motor in amps
            {
                m_angleEncoder.SetPosition(offset.value());
            }

            inline void SimPeriodic() override // Simulate the motor
            {
                
            }

        private:
            rev::spark::SparkMax                  m_motor;                    // SparkMax motor controller
            rev::spark::SparkRelativeEncoder      m_angleEncoder;             // Relative encoder onboard the sparkmax
            rev::spark::SparkClosedLoopController m_turnClosedLoopController; // PID Controller for SparkMax

            // rev::spark::SparkSim                  m_motorSim;                 // Simulation model for SparkMax
    };
}

}