// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include <frc/Timer.h>
#include <frc2/command/PrintCommand.h>

#include "commands/LedCommands/LedIdle.h"

#include "commands/LedCommands/Breath_Pulse.h"
#include "commands/LedCommands/Chase.h"
#include "commands/LedCommands/Linear_Pulse.h"
#include "commands/LedCommands/Rainbow.h"
#include "commands/LedCommands/Sinusoidal_Pulse.h"

LedIdle::LedIdle() {
  // Use addRequirements() here to declare subsystem dependencies.
  SelectCommand(
    [this] {
      return Select();
    },

    std::pair{ ONE, frc2::PrintCommand("one") }, std::pair{ TWO, frc2::PrintCommand("two") }
    );
}
CommandSelector Select() {
  return ONE;
}
// Called when the command is initially scheduled.
void LedIdle::Initialize() {
  m_startTime = frc::Timer::GetFPGATimestamp();
  m_led->SetAll(frc::Color(0, 0, 0));
}

// Called repeatedly when this Command is scheduled to run
void LedIdle::Execute() {
  units::minute_t time = frc::Timer::GetFPGATimestamp();

}

// Called once the command ends or is interrupted.
void LedIdle::End(bool interrupted) {}

// Returns true when the command should end.
bool LedIdle::IsFinished() {
  return false;
}
