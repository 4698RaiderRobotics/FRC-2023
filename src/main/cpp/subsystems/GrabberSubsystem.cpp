// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/GrabberSubsystem.h"

GrabberSubsystem::GrabberSubsystem() = default;

// This method will be called once per scheduler run
void GrabberSubsystem::Periodic() {}

void GrabberSubsystem::Open( ) {
    m_grab.Set( frc::DoubleSolenoid::Value::kReverse );
}

void GrabberSubsystem::Close( ) {
    m_grab.Set( frc::DoubleSolenoid::Value::kForward );
}

// Speed is value from -1 to 1
void GrabberSubsystem::Spin( double speed ) {
    m_roller.Set( ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speed );
}

void GrabberSubsystem::GrabberTest() {
    frc::SmartDashboard::PutData( &m_grab );
    frc::SmartDashboard::PutNumber( "Roller Speed", m_roller.GetMotorOutputPercent() );
}