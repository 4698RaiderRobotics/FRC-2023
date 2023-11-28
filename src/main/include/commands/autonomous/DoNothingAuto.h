// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryConfig.h>

#include "subsystems/Drivetrain.h"
#include "subsystems/ArmSubsystem.h"
#include "subsystems/GrabberSubsystem.h"
#include "subsystems/LEDs.h"

class DoNothingAuto
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 DoNothingAuto> {
 public:
  DoNothingAuto( Drivetrain *drive, ArmSubsystem *arm, GrabberSubsystem *grabber, LEDs *leds );

 private:
  frc::Trajectory m_trajectory;
  frc::TrajectoryConfig m_config{2_mps, 2_mps_sq};
};