#pragma once

#include <frc/DutyCycleEncoder.h>
#include <units/angle.h>

// Uses the DutyCycleEncoder class to use an SRX mag absolute encoder
class AbsoluteEncoder{
    public:
        // Offset must in the range from 0 to 1
        AbsoluteEncoder( const int absoluteEncoderChannel, const double absoluteEncoderOffset = 0.0) : 
                        m_absoluteEncoder{absoluteEncoderChannel}, 
                        m_absoluteEncoderOffset{absoluteEncoderOffset} { }

        // Returns the angle in degrees
        units::degree_t GetPosition( void ) {
            return ( m_absoluteEncoder.GetAbsolutePosition() - m_absoluteEncoderOffset ) * 360_deg - 180_deg;
        }
    private:
        frc::DutyCycleEncoder m_absoluteEncoder;

        const double m_absoluteEncoderOffset;
};