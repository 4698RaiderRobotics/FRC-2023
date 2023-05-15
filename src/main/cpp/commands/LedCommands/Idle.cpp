// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include <frc/Timer.h>
#include <iostream>
#include "commands/LedCommands/Idle.h"

Idle::Idle(LEDs* led): m_led{ led } {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({ led });
}

// Called when the command is initially scheduled.
void Idle::Initialize() {
  m_led->SetAll(frc::Color(0, 0, 0));
  std::cout << "Intiallized \n";

}

// Called repeatedly when this Command is scheduled to run
void Idle::Execute() {
  units::second_t time = frc::Timer::GetFPGATimestamp();
  units::second_t period = 10_s;
  auto floored = (time / period).value();
  // 5 options
  int selector = static_cast<int>(floored) % 5;
  switch (selector) {
  case 0:
    m_led->Linear_Pulse(frc::Color{ 255,0,0 }, 3_s);
    break;
  case 1:
    m_led->Chase(frc::Color{ 255,0,0 }, 10);
    break;
  case 2:
    m_led->Linear_Pulse(frc::Color{ 255,0,0 }, 5_s);
    break;
  case 3:
    m_led->Sinusoidal_Pulse(frc::Color{ 255,0,0 }, 5_s);
    break;
  case 4:
    m_led->Rainbow();
  }
}

// Called once the command ends or is interrupted.
void Idle::End(bool interrupted) {
  m_led->SetAll(frc::Color(0, 0, 0));
  std::cout << "Ended \n";

}

// Returns true when the command should end.
bool Idle::IsFinished() {
  return false;
}
