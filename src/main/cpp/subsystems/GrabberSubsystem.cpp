// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/GrabberSubsystem.h"

GrabberSubsystem::GrabberSubsystem() = default;

// This method will be called once per scheduler run
void GrabberSubsystem::Periodic() {}

void GrabberSubsystem::Open( ) {
    m_grab.Set( frc::DoubleSolenoid::Value::kForward );
}

void GrabberSubsystem::Close( ) {
    m_grab.Set( frc::DoubleSolenoid::Value::kReverse );
}

// Speed is value from -1 to 1
void GrabberSubsystem::Spin( double speed ) {
    m_spin_speed = speed;
    m_roller.Set( ctre::phoenix::motorcontrol::ControlMode::PercentOutput, m_spin_speed );
}

void GrabberSubsystem::Toggle( void ) {
    if( std::fabs(m_spin_speed) > 0.01 ) {
        m_roller.Set( ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0 );
        m_spin_speed = 0.0;
    } else {
        m_roller.Set( ctre::phoenix::motorcontrol::ControlMode::PercentOutput, kRollerGripPercent );
        m_spin_speed = kRollerGripPercent;
    }
}

void GrabberSubsystem::GrabberTest() {
    frc::SmartDashboard::PutData( &m_grab );
    frc::SmartDashboard::PutNumber( "Roller Speed", m_roller.GetMotorOutputPercent() );
}