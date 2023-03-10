// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TurnToAngle.h"

TurnToAngle::TurnToAngle( Drivetrain *drive, units::degree_t angle ) 
          : m_drive { drive }, m_angle{ angle } {
  AddRequirements( { drive } );
}

void TurnToAngle::Initialize() {
  m_goal = { m_angle };
  m_setpoint = { m_drive->GetPose().Rotation().Degrees() };
  m_finished = false;
}

void TurnToAngle::Execute() {
  if ( units::math::abs( m_goal.position - m_setpoint.position ) > 180_deg ) {
    if ( m_goal.position < 0_deg ) {
      m_goal.position += 360_deg;
    } else if ( m_setpoint.position < 0_deg ) {
      m_setpoint.position += 360_deg;
    }
  }
  frc::TrapezoidProfile<units::degrees> m_profile{ m_constraints, m_goal, m_setpoint };
  m_setpoint = m_profile.Calculate( dt );
  if ( m_setpoint.position > 180_deg ) {
      m_setpoint.position -= 360_deg;
  }

  frc::ChassisSpeeds speeds{ 0_mps, 0_mps, m_setpoint.velocity };
  m_drive->Drive( speeds );

  m_finished = m_profile.IsFinished( dt );
}

// Returns true when the command should end.
bool TurnToAngle::IsFinished() {
  return m_finished;
}
