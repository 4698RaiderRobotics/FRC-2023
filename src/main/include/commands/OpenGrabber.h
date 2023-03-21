// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Config.h"

#if defined(Claw)
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/GrabberSubsystem.h"

class OpenGrabber
    : public frc2::CommandHelper<frc2::CommandBase, OpenGrabber> {
 public:
  OpenGrabber( GrabberSubsystem *grabber, double speed = 0 );

  void Initialize() override;

  void Execute() override;

  bool IsFinished() override;
  
 private:
  GrabberSubsystem *m_grabber;
  double m_speed;
};
#endif
