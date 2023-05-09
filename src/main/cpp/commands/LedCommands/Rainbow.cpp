// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include <frc/Timer.h>

#include "commands/LedCommands/Rainbow.h"

Rainbow::Rainbow(LEDs* led): m_led{ led } {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({ led });
}

// Called when the command is initially scheduled.
void Rainbow::Initialize() {
  m_led->SetAll(frc::Color(0, 0, 0));
}

// Called repeatedly when this Command is scheduled to run
void Rainbow::Execute() {
  m_led->Rainbow();
}

// Called once the command ends or is interrupted.
void Rainbow::End(bool interrupted) {
  m_led->SetAll(frc::Color(0, 0, 0));

}

// Returns true when the command should end.
bool Rainbow::IsFinished() {
  return false;
}
