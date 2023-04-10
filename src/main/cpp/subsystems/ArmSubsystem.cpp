// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ArmSubsystem.h"
#include <frc/DriverStation.h>
#include <frc/Timer.h>

ArmSubsystem::ArmSubsystem() {
    m_right.SetInverted( true );
    m_right.Follow( m_left );
    m_left.SetNeutralMode( ctre::phoenix::motorcontrol::Brake );
    m_right.SetNeutralMode( ctre::phoenix::motorcontrol::Brake );

    m_setpoint.position = m_enc.GetPosition();
};

void ArmSubsystem::Periodic() {
    // Holds arm in position after enabled
    if ( disabled && frc::DriverStation::IsEnabled() ) {
        m_setpoint.position = m_enc.GetPosition();
        m_angle = m_enc.GetPosition();
        disabled = false;
    } else if ( !disabled && frc::DriverStation::IsDisabled() ) {
        disabled = true;
    }
    
    units::degree_t measurement = m_enc.GetPosition();
    // Checks for unplugged absolute encoder on the arm
    if ( measurement > 120_deg || measurement < -140_deg ) {
        FRC_ReportError	( -111 /*generic error*/, "Arm Absolute Encoder out of bounds." );
        return;
    }

    m_goal = { m_angle, 0_deg_per_s };

    // Creates a new trapezoid profile every loop
    frc::TrapezoidProfile<units::degrees> m_profile{ m_constraints, m_goal, m_setpoint };
    m_setpoint = m_profile.Calculate( dt );

    frc::SmartDashboard::PutNumber( "Arm Raw Output", m_enc.GetRawPosition() );
    frc::SmartDashboard::PutNumber( "Arm Setpoint Position", m_setpoint.position.value() );
    frc::SmartDashboard::PutNumber( "Arm Setpoint Velocity", m_setpoint.velocity.value() );

    // Calculates setpoint using PID and feedforward control
    double output = pid.Calculate( measurement.value(), m_setpoint.position.value());
    double feedforwardOut = feedforward.Calculate( m_setpoint.position, m_setpoint.velocity ).value();
    frc::SmartDashboard::PutNumber( "Arm Voltage", output * 12 + feedforwardOut );

    m_left.Set( ctre::phoenix::motorcontrol::ControlMode::PercentOutput, output + feedforwardOut / 12 );
}

// Changed the setpoint of the arm
void ArmSubsystem::GotoAngle( units::degree_t angle ) {
    // fmt::print( "GoToAngle = {}\n", angle);
    m_angle = angle;

        // Range check angle
    if( m_angle > max_angle ) m_angle = max_angle;
    if( m_angle < min_angle ) m_angle = min_angle;
}

// Allows setpoint of arm to be nudged
void ArmSubsystem::AdjustAngle( units::degree_t delta_angle ) {
    m_angle += delta_angle;
        // fmt::print( "AdjustAngle = {}\n", delta_angle );

        // Range check angle
    if( m_angle > max_angle ) m_angle = max_angle;
    if( m_angle < min_angle ) m_angle = min_angle;
}

bool ArmSubsystem::Finished( ) {
    return units::math::abs( m_enc.GetPosition() - m_angle ) < physical::kArmAngleError;
}

void ArmSubsystem::ArmDataSetup( ) {

    frc::SmartDashboard::PutNumber("kG", feedforward.kG.value() );
    frc::SmartDashboard::PutNumber("kP", kP);
    frc::SmartDashboard::PutNumber("kD", kD);
    frc::SmartDashboard::PutNumber("kV", kV);
    frc::SmartDashboard::PutNumber("kS", kS);

    frc::SmartDashboard::PutNumber( "Arm Position", m_enc.GetPosition().value() );
    frc::SmartDashboard::PutNumber( "Arm Velocity", m_left.GetSensorCollection().GetIntegratedSensorVelocity() 
                                                    * physical::tics_per_100ms_to_deg_per_s * physical::kArmGearRatio );

}

void ArmSubsystem::ArmDataUpdate( ) {

    double g = frc::SmartDashboard::GetNumber( "kG", 0.0 );
    double p = frc::SmartDashboard::GetNumber( "kP", 0.0 );
    double d = frc::SmartDashboard::GetNumber( "kD", 0.0 );
    double v = frc::SmartDashboard::GetNumber( "kV", 0.0 );
    double s = frc::SmartDashboard::GetNumber( "kS", 0.0 );

    if( g != feedforward.kG.value() ) { feedforward.kG = units::volt_t{ g }; }
    if( p != kP ) { kP = p; pid.SetP( kP ); }
    if( d != kD ) { kD = d; pid.SetD( kD ); }
    if( v != kV ) { kV = v; feedforward.kV = units::unit_t<frc::ArmFeedforward::kv_unit>{ kV }; }
    if( s != kS ) { kS = s; feedforward.kS = units::volt_t{ kS }; }

    frc::SmartDashboard::PutNumber( "Arm Position", m_enc.GetPosition().value() );
    frc::SmartDashboard::PutNumber( "Arm Velocity", m_left.GetSensorCollection().GetIntegratedSensorVelocity() 
                                                    * physical::tics_per_100ms_to_deg_per_s * physical::kArmGearRatio );
    

    frc::SmartDashboard::PutNumber( "Arm Raw Pos", m_enc.GetRawPosition() );
    
}

void ArmSubsystem::Brake( bool state ) {
    if ( state ) {
        m_brake.Set( frc::DoubleSolenoid::Value::kReverse );
    } else {
        m_brake.Set( frc::DoubleSolenoid::Value::kForward );
    }
}
units::degree_t ArmSubsystem::GetAngle() {
    return m_angle;
}