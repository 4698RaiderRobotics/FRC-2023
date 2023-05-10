// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include <frc/Timer.h>

#include "commands/LedCommands/Breath_Pulse.h"

Breath_Pulse::Breath_Pulse(LEDs* led, frc::Color color): m_led{ led }, m_color{ color } {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({ led });
}

// Called when the command is initially scheduled.
void Breath_Pulse::Initialize() {
  m_led->SetAll(frc::Color(0, 0, 0));
}

// Called repeatedly when this Command is scheduled to run
void Breath_Pulse::Execute() {
  m_led->Sinusoidal_Pulse(m_color, 5_s);
}

// Called once the command ends or is interrupted.
void Breath_Pulse::End(bool interrupted) {
  m_led->SetAll(frc::Color(0, 0, 0));
}

// Returns true when the command should end.
bool Breath_Pulse::IsFinished() {
  return false;
}
