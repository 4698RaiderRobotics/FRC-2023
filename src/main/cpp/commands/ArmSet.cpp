// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/Timer.h>

#include "commands/ArmSet.h"


ArmSet::ArmSet( ArmSubsystem *arm, units::degree_t angle )
        : m_arm{ arm }, m_angle{ angle } {
  SetName( "ArmSet" );
  AddRequirements( { arm } );
}

// Called when the command is initially scheduled.
void ArmSet::Initialize() {
  m_startTime = frc::Timer::GetFPGATimestamp();
}

// Called repeatedly when this Command is scheduled to run
void ArmSet::Execute() {
  fmt::print( "ArmSet::Execute going to angle {}", m_angle );
  m_arm->GotoAngle( m_angle );
}

// Called once the command ends or is interrupted.
void ArmSet::End(bool interrupted) {
  
}

// Returns true when the command should end.
bool ArmSet::IsFinished() {
  return m_arm->Finished( ) || frc::Timer::GetFPGATimestamp() - m_startTime > 2_s;
}
