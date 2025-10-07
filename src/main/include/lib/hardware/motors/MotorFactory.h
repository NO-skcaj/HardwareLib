#pragma once

#include "lib/hardware/motors/Motor.h"
#include "lib/hardware/motors/TalonFX.h"
#include "lib/hardware/motors/SparkMax.h"


namespace hardware
{

namespace motor
{

    // Factory method to create a motor controller based on CAN ID
    inline Motor Get(int CANid, MotorConfiguration& config, bool preferTalonFX)
    {
        if (preferTalonFX)
        {
            try 
            {
                TalonFX motor{CANid, config};

                return (Motor) motor;
            }
            catch (...)
            {
            }
        }

        try
        {
            SparkMax motor{CANid, true, config};

            return (Motor) motor;
        }
        catch (...)
        {
        }

        if (!preferTalonFX)
        {
            try 
            {
                TalonFX motor{CANid, config};

                return (Motor) motor;
            }
            catch (...)
            {
            }
        }

        std::cerr << "Error: Could not create motor with CAN ID " << CANid << std::endl;
        return preferTalonFX ? (Motor) TalonFX{CANid, config} : (Motor) SparkMax{CANid, true, config};
    }

}

}