#pragma once

#include <ctre/phoenix6/CANcoder.hpp>

#include "lib/hardware/encoders/Encoder.h"


namespace hardware
{

namespace encoder
{

    // CANCoder class to support the CANCoder
    class CANCoder : public Encoder
    {
        public:
            // Constructor for the CANCoder class
            // The CANCoder is a CAN device, so the CAN ID is passed in
            CANCoder(CANid_t CANid) 
                : m_encoder{CANid, "rio"}
            {}

            // Configure the CANCoder
            units::turn_t GetTurns() override
            {
                return m_encoder.GetAbsolutePosition().GetValue();
            }

        private:
            ctre::phoenix6::hardware::CANcoder m_encoder;
    };

}

}