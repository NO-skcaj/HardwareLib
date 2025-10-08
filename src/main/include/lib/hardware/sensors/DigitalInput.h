#pragma once

#include "lib/hardware/sensor/Sensor.h"

#include <frc/DigitalInput.h>

#include <frc/filter/Debouncer.h>


namespace hardware
{

namespace sensor
{

    class DigitalInput : public Sensor<bool>
    {
        public:
            DigitalInput(int CanId, units::second_t toleranceZone = 0_s)
                : m_sensor{CanId},
                  m_filter{toleranceZone, frc::Debouncer::DebounceType::kBoth}
            {}

            bool operator==(bool operand) override
            {
                return m_sensor.Get() == operand;
            }

            operator bool() override
            {
                return m_sensor.Get();
            }

            bool Get() override
            {
                return m_sensor.Get();
            }

            void Periodic() override
            {
                m_value = m_filter.Calculate(m_sensor.Get());
            }

        private:

            frc::DigitalInput m_sensor;

            frc::Debouncer m_filter;

            bool m_value;

    };

}

}