// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/GrabberSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class CloseGrabber
    : public frc2::CommandHelper<frc2::CommandBase, CloseGrabber> {
 public:
  CloseGrabber( GrabberSubsystem *grabber, bool spin_on_close = true );

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  bool m_spin_on_close;
  GrabberSubsystem *m_grabber;
};
