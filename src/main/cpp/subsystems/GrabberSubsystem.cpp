// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/GrabberSubsystem.h"

#include <frc/Timer.h>

GrabberSubsystem::GrabberSubsystem( frc::PowerDistribution &pdp ) : frc2::SubsystemBase(), m_pdp{pdp} {
    fmt::print( "GrabberSubsystem::GrabberSubsystem\n" );
}

// This method will be called once per scheduler run
void GrabberSubsystem::Periodic() {
    #if defined(Claw)
    frc::SmartDashboard::PutBoolean("Rollers", (m_roller.GetSensorCollection().GetIntegratedSensorVelocity() * physical::tics_per_100ms_to_deg_per_s > 1 ));
    #endif

    //double current = m_intake.GetOutputCurrent();
 //   fmt::print( "GrabberSubsystem::Periodic current is {}\n", current );

    if( (m_hasCone || m_hasCube) && m_isEjecting ) {
        if( (frc::Timer::GetFPGATimestamp() - m_startTime) > 1.0_s ) {
            m_hasCone = false;
            m_hasCube = false;
            m_intake.Set( 0.0 );
        }
    } else if( m_loadingCone || m_loadingCube ) { // Loading a game piece
        double current = m_pdp.GetCurrent( 9 );
        if ( ( (frc::Timer::GetFPGATimestamp() - m_startTime) > 0.5_s) && ( current > m_target_amps ) ) {
            fmt::print( "GrabberSubsystem::Periodic stoppped with current of {}\n", current );
            if( m_loadingCone ) {
                m_hasCone = true;
                m_loadingCone = false;
            } else {
                m_hasCube = true;
                m_loadingCube = false;
            }
            m_intake.Set( 0.0 );
        } else if( (frc::Timer::GetFPGATimestamp() - m_startTime) > 5_s) {
            // Timed out without getting a game piece
            m_loadingCone = false;
            m_loadingCube = false;
            m_intake.Set( 0.0 );
        }
    }
}

// Opens grabber
#if defined(Claw)
void GrabberSubsystem::Open( ) {
    m_grab.Set( frc::DoubleSolenoid::Value::kForward );
}

// Closes grabber
void GrabberSubsystem::Close( ) {
    m_grab.Set( frc::DoubleSolenoid::Value::kReverse );
}
// Toggles the rollers on or off
void GrabberSubsystem::Toggle( void ) {
    if( std::fabs( m_spin_speed ) > 0.01 ) {
        m_roller.Set( ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0 );
        m_spin_speed = 0.0;
    } else {
        m_roller.Set( ctre::phoenix::motorcontrol::ControlMode::PercentOutput, kRollerGripPercent );
        m_spin_speed = kRollerGripPercent;
    }
}
#endif
// Speed of rollers is value from -1 to 1
void GrabberSubsystem::Spin( double speed ) {
    m_spin_speed = speed;
    #if defined(Claw)
    m_roller.Set( ctre::phoenix::motorcontrol::ControlMode::PercentOutput, m_spin_speed );
    #else 
    m_intake.Set(m_spin_speed);
    #endif
}
#if !defined(Claw)
units::ampere_t GrabberSubsystem::GetCurrent() {
    return units::ampere_t{m_intake.GetOutputCurrent()};
}

void GrabberSubsystem::Cone( bool cone_inward ) {
    m_startTime = frc::Timer::GetFPGATimestamp();
    
    m_isEjecting = !cone_inward;
    
    if( cone_inward ) {
        m_intake.Set ( m_spin_speed );
    } else {
        m_intake.Set( -m_spin_speed );
    }
    
    fmt::print( "Cone inward {}\n", cone_inward );
}

void GrabberSubsystem::Cube( bool cube_inward ) {
    m_startTime = frc::Timer::GetFPGATimestamp();

    m_isEjecting = !cube_inward;
    
    if( cube_inward ) {
        m_intake.Set ( -m_spin_speed );
    } else {
        m_intake.Set( m_spin_speed );
    }
    
    fmt::print( "Cube inward {}\n", cube_inward );
}

void GrabberSubsystem::HandleCone( void ) {
    if( !m_hasCone && m_hasCube ) {
        this->HandleCube();
        return;
    }
    
    m_startTime = frc::Timer::GetFPGATimestamp();
    
    if( m_hasCone ) {
        m_isEjecting = true;
        m_intake.Set ( -m_spin_speed );
    } else {
        m_loadingCone = true;
        m_loadingCube = false;
        m_hasCone = false;
        m_hasCube = false;
        m_isEjecting = false;
        m_target_amps = m_cone_max_amps;
        m_intake.Set( m_spin_speed );
    }
}

void GrabberSubsystem::HandleCube( void ) {
    if( !m_hasCube && m_hasCone ) {
        this->HandleCone();
        return;
    }

    m_startTime = frc::Timer::GetFPGATimestamp();
    
    if( m_hasCube ) {
        m_isEjecting = true;
        m_intake.Set ( m_spin_speed );
    } else {
        m_loadingCone = false;
        m_loadingCube = true;
        m_hasCone = false;
        m_hasCube = false;
        m_isEjecting = false;
        m_target_amps = m_cube_max_amps;
        m_intake.Set( -m_spin_speed );
    }
}

void GrabberSubsystem::GrabberTest() {
    //frc::SmartDashboard::PutData( &m_grab );
    //frc::SmartDashboard::PutNumber( "Roller Speed", m_roller.GetMotorOutputPercent() );
    //fmt::print( "GrabberTest" );    
    frc::SmartDashboard::PutNumber("Intake Speed", m_intake.Get());
    frc::SmartDashboard::PutNumber("Intake Current", m_intake.GetOutputCurrent());
}
#endif
