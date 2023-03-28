// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Config.h"
#if !defined(Claw)

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include <units/time.h>

#include "subsystems/GrabberSubsystem.h"

class Intake
    : public frc2::CommandHelper<frc2::CommandBase, Intake> {
 public:
  Intake( GrabberSubsystem *grabber, bool reversed );
  
  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
 private:
  GrabberSubsystem *m_grabber;
  bool m_reversed;
  units::second_t m_startTime;
};
#endif