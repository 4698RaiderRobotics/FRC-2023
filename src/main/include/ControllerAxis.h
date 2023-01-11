#pragma once

#include <cmath>
#include <frc/GenericHID.h>

// Applies deadband and Expo to an axis of a Joystick controller.
class ControllerAxis {
public:
    ControllerAxis( const frc::GenericHID &js, int axis, bool invert=false ) : m_joystick{js}, m_axis{axis}, m_invert{invert} {}
    void SetDeadband( double deadband ) { m_deadband = deadband; }
    void SetMixer( double mixer ) { m_mixer = mixer; }
    void SetInvert( bool invert ) { m_invert = invert; }
    double GetAxis( void ) {
        double x = m_joystick.GetRawAxis( m_axis );
        if( m_invert ) x *= -1.0;

        double slope = 1.0 / (1.0 - m_deadband);

        if( x >= -m_deadband && x <= +m_deadband ) {
            x = 0.0;
        } else if( x < -m_deadband ) {
            x += m_deadband;
            x *= slope;
        } else if( x > +m_deadband ) {
            x -= m_deadband;
            x *= slope;
        }

        return m_mixer * std::pow(x, 3.0) + (1.0 - m_mixer) * x;
    }

private:
    const frc::GenericHID &m_joystick;

    int m_axis;
    bool m_invert;
    double m_deadband{0.08};
    double m_mixer{0.75};
};