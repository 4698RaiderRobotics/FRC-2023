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
        AbsoluteEncoder( const int absoluteEncoderChannel, const double absoluteEncoderOffset = 0.0, bool inverted = false ) : 
                        m_absoluteEncoder{absoluteEncoderChannel}, 
                        m_absoluteEncoderOffset{absoluteEncoderOffset},
                        m_invertFactor{ 1 }
         { 
            if ( inverted ) {
                m_invertFactor = -1;
            }
        }

        // Returns the angle in degrees
        units::degree_t GetPosition( void ) {
            return ( m_absoluteEncoder.GetAbsolutePosition() - m_absoluteEncoderOffset ) * 360_deg * m_invertFactor;
        }
        double GetRawPosition() {
            return m_absoluteEncoder.GetAbsolutePosition();
        }
    private:
        frc::DutyCycleEncoder m_absoluteEncoder;

        const double m_absoluteEncoderOffset;

        int m_invertFactor;
};