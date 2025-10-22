#pragma once

#include <memory>
#include <optional>

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>
#include <units/current.h>

#include <frc/system/plant/DCMotor.h>
#include <frc/simulation/DCMotorSim.h>
#include <frc/system/plant/LinearSystemId.h>

#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/controller/PIDController.h>

#include <frc/RobotBase.h>

#include "lib/hardware/hardware.h"


namespace hardware
{

namespace motor
{

    struct MotorConfiguration
    {
        units::ampere_t CurrentLimit;
        bool   breakMode;
        double P;
        double I;
        double D;
        double S; // add your G term to this
        double V;
        double A;
    };

    // This class is used to abstract the motor controller interface
    // EVERYTHING is in turns. Update encoders to match. Conversions happens in implementation.
    class Motor : public Hardware
    {
        public:

            Motor() 
                : m_feedForwards{0_V, 0_V * 1_s / 1_m, 0_V * 1_s * 1_s / 1_m},
                  m_pidController{0.0, 0.0, 0.0},
                  m_motorSim{
                      frc::LinearSystemId::DCMotorSystem(
                          frc::DCMotor::KrakenX60(),
                          0.001_kg_sq_m,
                          1
                      ),
                      frc::DCMotor::KrakenX60()
                  }
            {}
            
            void ConfigureMotor(MotorConfiguration config) 
            {
                // Configure PID
                m_pidController.SetPID(config.P, config.I, config.D);
                // Configure Feedforward
                m_feedForwards.SetKs(config.S * 1_V);
                m_feedForwards.SetKv(config.V * 1_V * 1_s / 1_m);
                m_feedForwards.SetKa(config.A * 1_V * 1_s * 1_s / 1_m);
            }

            // sets the voltage goal of the motor.
            void SetReferenceState(double motorInput) // output to motor within (-1,1)
            {
                motorInput = std::clamp(motorInput, -1.0, 1.0); // ensure within bounds, if its not you might be wanting to draw like 30 volts

                m_currentInput        = units::volt_t{motorInput * frc::RobotController::GetBatteryVoltage().value()};
                m_currentState        = MotorState::ARBITARY;
                m_lastInput.arbitrary = motorInput;
            }

            void SetReferenceState(units::volt_t motorInput) 
            {
                m_currentInput      = motorInput;
                m_currentState      = MotorState::VOLTAGE;
                m_lastInput.voltage = motorInput;
            }

            void SetReferenceState(units::turns_per_second_t motorInput) 
            {
                m_currentInput = m_feedForwards.Calculate(units::meters_per_second_t{motorInput.value()}) + 
                                  units::volt_t{m_pidController.Calculate(GetVelocity().value(), motorInput.value())};
                m_currentState       = MotorState::VELOCITY;
                m_lastInput.velocity = motorInput;
            }

            void SetReferenceState(units::turn_t motorInput) 
            {
                // you normally won't use feedforward for position control BUTTT sometime you need static/gravity compensation
                units::turns_per_second_t desiredVelocity{(motorInput - GetPosition()).value()}; // Not sure if this is the right way to do this
                m_currentInput = m_feedForwards.Calculate(units::meters_per_second_t{desiredVelocity.value()}) + 
                                  units::volt_t{m_pidController.Calculate(GetPosition().value(), desiredVelocity.value())};

                m_currentState       = MotorState::POSITION;
                m_lastInput.position = motorInput;
            }
            
            // runs every robot cycle, 
            void Periodic () override 
            {
                if (frc::RobotBase::IsSimulation()) SimPeriodic();

                switch (m_currentState)
                {
                    case MotorState::ARBITARY:
                    case MotorState::VOLTAGE:
                        break; // Both of these are already set and will not change until the user says so.
                    case MotorState::VELOCITY:
                        SetReferenceState(m_lastInput.velocity); // use this to update the motor output using the controllers.
                        break;
                    case MotorState::POSITION:
                        SetReferenceState(m_lastInput.position); // use this to update the motor output using the controllers.
                        break;
                    case MotorState::IDLE:
                    default:
                        SetVoltage(0_V);
                        break;
                }
            }

            virtual units::turn_t GetPosition() // need to override
            {
                return units::turn_t{0};
            }

            virtual units::turns_per_second_t GetVelocity() // need to override
            {
                return units::turns_per_second_t{0};
            }

            virtual void OffsetEncoder(units::turn_t offset)
            {
                // do nothing, need to override
            }


        protected:

            virtual void SetVoltage(units::volt_t voltage) // need to override
            {

            }

        private:

            enum class MotorState
            {
                IDLE,
                ARBITARY,
                VELOCITY,
                POSITION,
                VOLTAGE
            };

            union LastInput 
            {
                double                    arbitrary;
                units::turns_per_second_t velocity;
                units::turn_t             position;
                units::volt_t             voltage;
            };

            MotorState m_currentState = MotorState::IDLE;
            LastInput  m_lastInput;
            
            // runs every periodic cycle
            void SimPeriodic() 
            {
                m_motorSim.SetInputVoltage(m_currentInput);
                m_motorSim.Update(20_ms); // assume 20 ms loop time
            }

            units::volt_t                              m_currentInput;

            frc::SimpleMotorFeedforward<units::meters> m_feedForwards;
            frc::PIDController                         m_pidController;

            frc::sim::DCMotorSim                       m_motorSim; // Simulated motor model

    };

};

};