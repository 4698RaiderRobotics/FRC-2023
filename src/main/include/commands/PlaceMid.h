// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>
#include <frc2/command/CommandScheduler.h>

#include "commands/ArmSet.h"

#include "subsystems/ArmSubsystem.h"
#include "subsystems/GrabberSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class PlaceMid
    : public frc2::CommandHelper<frc2::CommandBase, PlaceMid> {
 public:
  PlaceMid( ArmSubsystem* arm, GrabberSubsystem* grabber );

  void Initialize() override;

  bool IsFinished() override { return true; }

 private:
  ArmSubsystem *m_arm;
  GrabberSubsystem *m_grabber;

  bool m_cubePlace = false;
  bool m_conePlace = false;

  units::second_t m_startTime;

  Command *m_armsetcmd { nullptr };

};
