#pragma once

#include <memory>
#include <optional>

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>
#include <units/current.h>

#include <frc/RobotBase.h>

#include "lib/hardware/hardware.h"
#include "lib/MotorController.h"


namespace hardware
{

namespace motor
{

    struct MotorConfiguration : public ControlConfiguration
    {
        units::ampere_t          CurrentLimit;
    };

    // This class is used to abstract the motor controller interface
    class Motor : public Hardware
    {
        public:
            
            virtual void ConfigureMotor(hardware::motor::MotorConfiguration config) {  }

            // output to motor
            virtual void Set(double                    motorInput) {}
            virtual void Set(units::volt_t             motorInput) {}
            virtual void Set(units::turns_per_second_t motorInput) {}
            virtual void Set(units::turn_t             motorInput) {}

            // set the encoder position to an offset, useful for zeroing the encoder
            virtual void OffsetEncoder(units::turn_t offset) {}

            // get the current position according to the motor
            virtual units::turn_t GetPosition() { return 0_tr; }
            
            // get the velocity of the motor in turns per second
            virtual units::turns_per_second_t GetVelocity() { return 0_tps; }

            virtual void Periodic () override { if (frc::RobotBase::IsSimulation()) SimPeriodic(); } // runs every robot cycle

            // runs every periodic cycle
            virtual void SimPeriodic() {}
    };

};

};