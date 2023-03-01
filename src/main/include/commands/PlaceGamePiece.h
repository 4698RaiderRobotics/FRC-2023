// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc/Timer.h>
#include <units/length.h>
#include <units/acceleration.h>

#include "subsystems/Drivetrain.h"
#include "subsystems/ArmSubsystem.h"
#include "subsystems/GrabberSubsystem.h"

class PlaceGamePiece
    : public frc2::CommandHelper<frc2::SequentialCommandGroup, PlaceGamePiece> {
 public:
  PlaceGamePiece( Drivetrain *drive, ArmSubsystem *arm, GrabberSubsystem *grabber, 
                  frc::Pose2d targetPose, units::degree_t angle );

  Command::InterruptionBehavior GetInterruptionBehavior() const override;

 private:
  frc::Timer m_timer;
};
