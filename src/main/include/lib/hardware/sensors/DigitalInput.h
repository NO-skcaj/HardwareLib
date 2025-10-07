#pragma once

#include "lib/hardware/sensor/Sensor.h"

#include <frc/DigitalInput.h>


namespace hardware
{

namespace sensor
{

    class DigitalInput : public Sensor<bool>
    {
        public:
            DigitalInput(int CanId) : m_sensor{CanId} {};

            bool operator==(bool operand) override
            {
                return m_sensor.Get() == operand;
            }

            operator bool() override
            {
                return m_sensor.Get();
            }

        private:
            frc::DigitalInput m_sensor;

    };

}

}