#pragma once

#include "Motor.h"

#include <iostream>

#include <frc/system/plant/DCMotor.h>
#include <frc/simulation/DCMotorSim.h>
#include <frc/system/plant/LinearSystemId.h>

#include <frc/controller/SimpleMotorFeedforward.h>

#include <rev/SparkMax.h>
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

            inline SparkMax(int CANid, MotorConfiguration config, frc::DCMotor motorType) 
            :   m_motor{CANid, config.brakeMode 
                                ? rev::spark::SparkLowLevel::MotorType::kBrushless 
                                : rev::spark::SparkLowLevel::MotorType::kBrushed},
                m_angleEncoder{m_motor.GetEncoder()}, 
                m_turnClosedLoopController{m_motor.GetClosedLoopController()}
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
                    .Pid(0.0,0.0,0.0); // Do not use the onboard PID controller

                // Write the configuration to the motor controller
                m_motor.Configure(sparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
            }

            inline units::turn_t GetPosition() override // Returns the position of the motor in turns
            {
                return frc::RobotBase::IsSimulation()
                    ? units::turn_t{m_motorSim.GetAngularPosition().value() / (2 * std::numbers::pi)} 
                    : units::turn_t{m_angleEncoder.GetPosition()};
            }

            inline units::turns_per_second_t GetVelocity() override // Returns the velocity of the motor in turn velocity
            {
                return frc::RobotBase::IsSimulation() 
                    ? units::turns_per_second_t{m_motorSim.GetAngularVelocity().value() / (2 * std::numbers::pi)} 
                    : units::turns_per_second_t{m_angleEncoder.GetVelocity()};
            }

            inline void OffsetEncoder(units::turn_t offset) override
            {
                m_angleEncoder.SetPosition(offset.value());
            }

        private:

            int help = 0;

            rev::spark::SparkMax                       m_motor;                    // SparkMax motor controller
            rev::spark::SparkRelativeEncoder           m_angleEncoder;             // Relative encoder onboard the sparkmax
            rev::spark::SparkClosedLoopController      m_turnClosedLoopController; // PID Controller for SparkMax

            frc::sim::DCMotorSim                       m_motorSim; // Simulated motor model
            frc::SimpleMotorFeedforward<units::meters> m_feedForwards;

    };
}

}