// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ArmSubsystem.h"

ArmSubsystem::ArmSubsystem() {
    m_right.SetInverted( true );
    m_right.Follow( m_left );
};

// This method will be called once per scheduler run
void ArmSubsystem::Periodic() {

}
void ArmSubsystem::Arm( units::degree_t angle ) {
    m_goal = { angle, 0_deg_per_s };

    frc::TrapezoidProfile<units::degrees> m_profile{ m_constraints, m_goal, m_setpoint };
    m_setpoint = m_profile.Calculate( dt );

    units::degree_t measurement = m_enc.GetPosition();

    frc::ArmFeedforward feedforward{ units::volt_t{ kS }, units::volt_t{ kG },  units::unit_t<frc::ArmFeedforward::kv_unit> {kV}  };

    frc2::PIDController pid{ kP, kI, kD };

    double output = pid.Calculate( measurement.value(), m_setpoint.position.value());
    double feedforwardOut = feedforward.Calculate( units::radian_t{ measurement }, units::radians_per_second_t{ m_setpoint.velocity() } ).value();

    m_left.Set( ctre::phoenix::motorcontrol::ControlMode::PercentOutput, output  + feedforwardOut / 12 );
}

void ArmSubsystem::ArmTestSetup() {
    frc::SmartDashboard::PutNumber("kG", kG);
    frc::SmartDashboard::PutNumber("kP", kP);
    frc::SmartDashboard::PutNumber("kD", kD);
    frc::SmartDashboard::PutNumber("kV", kV);
    frc::SmartDashboard::PutNumber("kS", kS);
}

void ArmSubsystem::ArmTest() {
    double g = frc::SmartDashboard::GetNumber( "kG", 0.0 );
    double p = frc::SmartDashboard::GetNumber( "kP", 0.0 );
    double d = frc::SmartDashboard::GetNumber( "kD", 0.0 );
    double v = frc::SmartDashboard::GetNumber( "kV", 0.0 );
    double s = frc::SmartDashboard::GetNumber( "kS", 0.0 );

    if((g != kG)) { kG = g; }
    if((p != kP)) { kP = p; }
    if((d != kD)) { kD = d; }
    if((v != kV)) { kV = v; }
    if((s != kS)) { kS = s; }

    frc::SmartDashboard::PutNumber( "Arm Position", m_enc.GetPosition().value() );
    frc::SmartDashboard::PutNumber( "Arm Velocity", m_left.GetMotorOutputPercent() );
    frc::SmartDashboard::PutNumber( "Arm Setpoint Position", m_setpoint.position.value() );
    frc::SmartDashboard::PutNumber( "Arm Setpoint Velocity", m_setpoint.velocity.value() );

    frc::SmartDashboard::PutNumber( "Arm Raw Pos", m_enc.GetRawPosition() );
}