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

            m_absoluteEncoder.SetPositionOffset(absoluteEncoderOffset);
            m_absoluteEncoder.SetDistancePerRotation(360.0);
        }

        // Returns the angle in degrees wrapping 0-360
        units::degree_t GetRollOverPosition(void) {
            return (m_absoluteEncoder.GetAbsolutePosition() - m_absoluteEncoderOffset) * 360_deg * m_invertFactor;
        }

        // Returns the angle in degrees without wrapping at zero
        units::degree_t GetCountingPosition(void) {
            return (m_absoluteEncoder.GetDistance() * 1_deg * m_invertFactor) - position_offset;
        }
        
        double GetRawPosition() {
            return m_absoluteEncoder.GetAbsolutePosition();
        }

        void SetPositionOffset(void) {
            while (GetCountingPosition() > 180_deg) {
                position_offset += 360_deg;
            }
            while ( GetCountingPosition() < -180_deg ) {
                position_offset -= 360_deg;
            }
            
        }

        bool IsConnected( void ) {
            return m_absoluteEncoder.IsConnected();
        }
private:
        frc::DutyCycleEncoder m_absoluteEncoder;

        const double m_absoluteEncoderOffset;

        int m_invertFactor;
        units::degree_t position_offset{ 0_deg };
};