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
    
    // needs feedforward and motion magic support
    class SparkMax : public Motor
    {
        public:

            inline SparkMax(int CANid, MotorConfiguration config, frc::DCMotor motorModel) 
            : m_motor{CANid, config.breakMode
                                ? rev::spark::SparkLowLevel::MotorType::kBrushless 
                                : rev::spark::SparkLowLevel::MotorType::kBrushed},
                m_angleEncoder{m_motor.GetEncoder()}, 
                m_turnClosedLoopController{m_motor.GetClosedLoopController()},
                m_motorModel{motorModel},
                m_motorSim{
                    frc::LinearSystemId::DCMotorSystem(
                        m_motorModel,
                        0.001_kg_sq_m,
                        1
                    ),
                    m_motorModel
                },
                m_sparkSim{&m_motor, &m_motorModel}
            {
                ConfigureRealMotor(config);
                ConfigureMotor(config);
            }

            inline void ConfigureRealMotor(MotorConfiguration config) // Configure the motor with default settings
            {
                // Configure the angle motor
                rev::spark::SparkMaxConfig sparkMaxConfig{};

                // Configure the motor controller
                sparkMaxConfig
                    .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
                    .SmartCurrentLimit(config.CurrentLimit.value());
                    
                // All you have to do for encoders is make sure that they are in turns. If a particular motor only does counts... submit a pull request.

                // Configure the closed loop controller
                sparkMaxConfig.closedLoop
                    .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
                    .Pid(1.0,0.0,0.0); // Do not use the onboard PID controller

                // Write the configuration to the motor controller
                m_motor.Configure(sparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
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

        private:

            void SetVoltage(units::volt_t voltage) override
            {
                m_motor.SetVoltage(voltage);
            }

            void SimPeriodic() override
            {
                m_motorSim.SetInputVoltage(m_sparkSim.GetAppliedOutput() * frc::RobotController::GetBatteryVoltage());
                m_motorSim.Update(0.02_s);
                m_sparkSim.iterate(m_motorSim.GetAngularVelocity().value(), frc::RobotController::GetBatteryVoltage().value(), 0.02);
            }

            rev::spark::SparkMax                  m_motor;                    // SparkMax motor controller
            rev::spark::SparkRelativeEncoder      m_angleEncoder;             // Relative encoder onboard the sparkmax
            rev::spark::SparkClosedLoopController m_turnClosedLoopController; // PID Controller for SparkMax

            frc::DCMotor                          m_motorModel; // Type of motor attached to the SparkMax
            frc::sim::DCMotorSim                  m_motorSim;   // Simulated motor model
            rev::spark::SparkMaxSim               m_sparkSim;   // Simulated SparkMax model

    };
}

}