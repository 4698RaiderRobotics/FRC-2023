#pragma once

#include <frc/DutyCycleEncoder.h>
#include <units/angle.h>

// Uses the DutyCycleEncoder class to use an SRX mag absolute encoder
class AbsoluteEncoder{
    public:
        /**
         * @param absoluteEncoderOffset Must be a value from 0 to 1
         * @param inverted -1 for inverted encoder, defaults to 1
        */
        AbsoluteEncoder( const int absoluteEncoderChannel, const double absoluteEncoderOffset = 0.0, int inverted = 1 ) : 
                        m_absoluteEncoder{absoluteEncoderChannel}, 
                        m_absoluteEncoderOffset{absoluteEncoderOffset},
                        m_inverted{ inverted } { }

        // Returns the angle in degrees
        units::degree_t GetPosition( void ) {
            return ( m_absoluteEncoder.GetAbsolutePosition() - m_absoluteEncoderOffset ) * 360_deg * m_inverted - 180_deg ;
        }

        double GetRawPosition() {
            return m_absoluteEncoder.GetAbsolutePosition();
        }
    private:
        frc::DutyCycleEncoder m_absoluteEncoder;

        const double m_absoluteEncoderOffset;

        int m_inverted;
};