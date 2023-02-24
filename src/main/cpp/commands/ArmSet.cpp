// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ArmSet.h"

ArmSet::ArmSet( units::degree_t angle, ArmSubsystem *arm )
        : m_angle{ angle }, m_arm{ arm } {
  AddRequirements( { arm } );
}

// Called when the command is initially scheduled.
void ArmSet::Initialize() {
  
}

// Called repeatedly when this Command is scheduled to run
void ArmSet::Execute() {
  m_arm->Arm( m_angle );
}

// Called once the command ends or is interrupted.
void ArmSet::End(bool interrupted) {
  
}

// Returns true when the command should end.
bool ArmSet::IsFinished() {
  return false;
}
