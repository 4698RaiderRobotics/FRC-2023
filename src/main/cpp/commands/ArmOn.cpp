// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ArmOn.h"

ArmOn::ArmOn( ArmSubsystem *arm ) 
  : m_arm{arm} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements( { arm } );
}

// Called when the command is initially scheduled.
void ArmOn::Initialize() {
  m_startTime = frc::Timer::GetFPGATimestamp();
}

// Called repeatedly when this Command is scheduled to run
void ArmOn::Execute() {

}

// Called once the command ends or is interrupted.
void ArmOn::End(bool interrupted) {}

// Returns true when the command should end.
bool ArmOn::IsFinished() {
  return false;
}
