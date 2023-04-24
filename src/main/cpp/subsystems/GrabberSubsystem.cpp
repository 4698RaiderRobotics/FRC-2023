// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/GrabberSubsystem.h"
#include "subsystems/ArmSubsystem.h"
#include <frc/Timer.h>

GrabberSubsystem::GrabberSubsystem( frc::PowerDistribution &pdp, ArmSubsystem *arm) : frc2::SubsystemBase(), m_pdp{pdp}, m_arm{arm}{
    fmt::print( "GrabberSubsystem::GrabberSubsystem\n" );
    m_intake.RestoreFactoryDefaults();
    m_intake.SetSmartCurrentLimit(40);
}

// This method will be called once per scheduler run
void GrabberSubsystem::Periodic() {

    //double current = m_intake.GetOutputCurrent();
 //   fmt::print( "GrabberSubsystem::Periodic current is {}\n", current );
    double current = m_pdp.GetCurrent( 9 );
    double stallSpeed = m_enc.GetVelocity();

    if( (m_hasCone || m_hasCube) && m_isEjecting ) {
        if( (frc::Timer::GetFPGATimestamp() - m_startTime) > 1.0_s ) {
            m_hasCone = false;
            m_hasCube = false;
            m_intake.Set( 0.0 );
        }
    } else if( m_loadingCone || m_loadingCube ) { // Loading a game piece
        
        if ( ( (frc::Timer::GetFPGATimestamp() - m_startTime) > 0.5_s) && ( abs( stallSpeed ) < m_target_stall_speed ) ) {
            fmt::print( "GrabberSubsystem::Periodic stopped with current of {} and speed of {}\n", current, stallSpeed );
            if( m_loadingCone ) {
                m_hasCone = true;
                m_loadingCone = false;
                m_intake.Set( 0.05 );
            } else {
                m_hasCube = true;
                m_loadingCube = false;
                m_intake.Set( -0.05 );
            }
 //           m_intake.Set( 0.0 );
        } else if( (frc::Timer::GetFPGATimestamp() - m_startTime) > 5_s) {
            // Timed out without getting a game piece
            m_loadingCone = false;
            m_loadingCube = false;
            m_intake.Set( 0.0 );
        }
    }
    //fmt::print( "Intake current: {}\n", current );
    frc::SmartDashboard::PutNumber( "Intake Current", current );
}

// Speed of rollers is value from -1 to 1
void GrabberSubsystem::Spin( double speed ) {
    m_spin_speed = speed;
    m_intake.Set(m_spin_speed);
}
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
        m_intake.Set ( -m_cone_shoot_speed);
    } else {
        m_loadingCone = true;
        m_loadingCube = false;
        m_hasCone = false;
        m_hasCube = false;
        m_isEjecting = false;
        m_target_amps = m_cone_max_amps;
        m_intake.Set( m_cone_intake_speed );
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
        if(m_arm->GetAngle() < 10_deg){
            m_intake.Set(m_cube_shoot_slow_speed);
        }
        else {
            m_intake.Set ( m_cube_shoot_fast_speed );
        }
    } else {
        m_loadingCone = false;
        m_loadingCube = true;
        m_hasCone = false;
        m_hasCube = false;
        m_isEjecting = false;
        m_target_amps = m_cube_max_amps;
        m_intake.Set( -m_cube_intake_speed );
    }
}

void GrabberSubsystem::GrabberTest() {
    //frc::SmartDashboard::PutData( &m_grab );
    //frc::SmartDashboard::PutNumber( "Roller Speed", m_roller.GetMotorOutputPercent() );
    //fmt::print( "GrabberTest" );    
    frc::SmartDashboard::PutNumber("Intake Speed", m_intake.Get());
    frc::SmartDashboard::PutNumber("Intake Current", m_intake.GetOutputCurrent());
}