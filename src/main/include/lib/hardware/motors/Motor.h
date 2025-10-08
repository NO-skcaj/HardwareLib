#pragma once

#include <memory>
#include <optional>

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>
#include <units/current.h>

#include "lib/hardware/hardware.h"


namespace hardware
{

namespace motor
{

    struct PIDConfiguration
    {
        double P;
        double I;
        double D;
    };

    struct FeedForwardConfiguration
    {
        double S;
        double V;
        double A;
    };

    struct MagicMotionConfiguration
    {
        units::turns_per_second_t         TargetVelocity;
        units::turns_per_second_squared_t TargetAcceleration; 
        units::turns_per_second_cubed_t   TargetJerk;
    };

    struct MotorConfiguration
    {
        units::ampere_t                         CurrentLimit;
        PIDConfiguration                        PID;
        std::optional<FeedForwardConfiguration> FeedForward;
        std::optional<MagicMotionConfiguration> MagicMotion;
    };

    // This class is used to abstract the motor controller interface
    class Motor : public Hardware
    {
        public:
            
            virtual void ConfigureMotor(MotorConfiguration config) {};

            // output to motor
            virtual void SetSpeed(double                    motorInput) {};
            virtual void SetSpeed(units::turns_per_second_t motorInput) {};
            virtual void SetSpeed(units::volt_t             motorInput) {};

            // positional control
            virtual void SetPosition(units::turn_t motorInput) {};

            // set the encoder position to an offset, useful for zeroing the encoder
            virtual void OffsetEncoder(units::turn_t offset) {};

            // get the current position according to the motor
            virtual units::turn_t GetPosition() { return 0_tr; };
            
            // get the velocity of the motor in turns per second
            virtual units::turns_per_second_t GetVelocity() { return 0_tps; };

            // runs every periodic cycle
            virtual void SimPeriodic() {};
    };

}

}