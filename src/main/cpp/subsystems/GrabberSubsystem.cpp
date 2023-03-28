// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/GrabberSubsystem.h"

#include <frc/Timer.h>

GrabberSubsystem::GrabberSubsystem() : frc2::SubsystemBase() {
    fmt::print( "GrabberSubsystem::GrabberSubsystem\n" );
}

// This method will be called once per scheduler run
void GrabberSubsystem::Periodic() {
    #if defined(Claw)
    frc::SmartDashboard::PutBoolean("Rollers", (m_roller.GetSensorCollection().GetIntegratedSensorVelocity() * physical::tics_per_100ms_to_deg_per_s > 1 ));
    #endif

    //double current = m_intake.GetOutputCurrent();
     double current = 0.0;
    
    if ( ( (frc::Timer::GetFPGATimestamp() - m_startTime) > 1.0_s) && ( current > 80 ) ) {
        fmt::print( "GrabberSubsystem::Periodic stoppped with current of {}\n", current );
        m_intake.Set( 0.0 );
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

void GrabberSubsystem::Cone( bool direction ) {
    m_startTime = frc::Timer::GetFPGATimestamp();
    
    direction ? m_intake.Set( m_spin_speed ) : m_intake.Set( -m_spin_speed );
    
    fmt::print( "Cone {}\n", direction );
}

void GrabberSubsystem::Cube( bool cube_inward ) {
    m_startTime = frc::Timer::GetFPGATimestamp();
    
    if( cube_inward ) {
        m_intake.Set ( -m_spin_speed );
    } else {
        m_intake.Set( m_spin_speed );
    }
    
    fmt::print( "Cube {}\n", cube_inward );
}

void GrabberSubsystem::GrabberTest() {
    //frc::SmartDashboard::PutData( &m_grab );
    //frc::SmartDashboard::PutNumber( "Roller Speed", m_roller.GetMotorOutputPercent() );
    //fmt::print( "GrabberTest" );    
    frc::SmartDashboard::PutNumber("Intake Speed", m_intake.Get());
    frc::SmartDashboard::PutNumber("Intake Current", m_intake.GetOutputCurrent());
}
#endif
