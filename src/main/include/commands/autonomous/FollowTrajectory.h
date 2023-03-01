// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/trajectory/Trajectory.h>
#include <units/time.h>

#include "subsystems/Drivetrain.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class FollowTrajectory
    : public frc2::CommandHelper<frc2::CommandBase, FollowTrajectory> {
 public:
  FollowTrajectory( Drivetrain *drive, frc::Trajectory trajectory, frc::Rotation2d heading );

  void Initialize() override;

  void Execute() override;

  bool IsFinished() override;

  private:
    Drivetrain *m_drive;
    frc::Trajectory m_trajectory;
    frc::Rotation2d m_heading;
    

    units::second_t m_autoElapsed;
};
