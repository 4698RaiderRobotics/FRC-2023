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