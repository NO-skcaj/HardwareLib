#pragma once

#include <frc2/command/SubsystemBase.h>

#include "lib/hardware/motors/TalonFX.h"
#include "lib/hardware/motors/SparkMax.h"

#include "Constants.h"


class Volcano : public frc2::SubsystemBase
{
    public:

        static Volcano* GetInstance()
        {
            static Volcano instance;
            static Volcano* instancePtr = &instance;
            return instancePtr;
        }

        // Double tapping either buttons will toggle between RUNNING or IDLE and off, single tap will run/stop based on which button you choose
        // ex: given its on idle, you press the idle button again and it will turn off
        // ex: given its off, you press the run (or idle) button and it will turn on to run (or idle) speed
        // the boolean is an option between running or idle
        inline void SetFlywheel(bool isRunning)
        {
            switch (m_flywheelState)
            {
                case FlywheelState::RUNNING:
                    if (!isRunning)
                    { 
                        m_flywheelMotor.SetReferenceState(constants::volcano::volcanoConfig.flywheelIdleVoltage);
                        m_flywheelState = FlywheelState::IDLE;
                    } else
                    {
                        m_flywheelMotor.SetReferenceState(0_V);
                        m_flywheelState = FlywheelState::OFF;
                    }
                    break;
                case FlywheelState::IDLE:
                    if (isRunning)
                    { 
                        m_flywheelMotor.SetReferenceState(constants::volcano::volcanoConfig.flywheelOnVoltage);
                        m_flywheelState = FlywheelState::RUNNING;
                    } else
                    {
                        m_flywheelMotor.SetReferenceState(0_V);
                        m_flywheelState = FlywheelState::OFF;
                    }
                    break;
                case FlywheelState::OFF:
                    if (isRunning)
                    {
                        m_flywheelMotor.SetReferenceState(constants::volcano::volcanoConfig.flywheelOnVoltage);
                        m_flywheelState = FlywheelState::RUNNING;
                    } else
                    {
                        m_flywheelMotor.SetReferenceState(constants::volcano::volcanoConfig.flywheelIdleVoltage);
                        m_flywheelState = FlywheelState::IDLE;
                    }
                    break;
            }
            
        }

        // Not sure what this should be, so it will be between 0 and 12 volts, limit as needed DO NOT LIMIT IN IMPLEMENTATION, LIMIT HERE
        inline void SetIndexer(double speed)
        {
            for (auto& motor : m_indexerMotors)
            {
                motor.SetReferenceState(speed);
            }
        }

    private:

        explicit Volcano()
            : m_flywheelMotor{constants::volcano::volcanoConfig.flywheelMotorCANid, constants::volcano::volcanoConfig.flywheelMotorConfig, frc::DCMotor::Falcon500()}
            , m_indexerMotors{
                hardware::motor::SparkMax{constants::volcano::volcanoConfig.indexerMotors[0], constants::volcano::volcanoConfig.indexerMotorsConfig, frc::DCMotor::NEO()},
                hardware::motor::SparkMax{constants::volcano::volcanoConfig.indexerMotors[1], constants::volcano::volcanoConfig.indexerMotorsConfig, frc::DCMotor::NEO()},
                hardware::motor::SparkMax{constants::volcano::volcanoConfig.indexerMotors[2], constants::volcano::volcanoConfig.indexerMotorsConfig, frc::DCMotor::NEO()}
             }
            , m_flywheelState{FlywheelState::OFF}

        {
        }

        hardware::motor::TalonFX m_flywheelMotor;
        hardware::motor::SparkMax m_indexerMotors[3];

        enum class FlywheelState
        {
            RUNNING,
            IDLE,
            OFF
        };

        FlywheelState m_flywheelState;
};