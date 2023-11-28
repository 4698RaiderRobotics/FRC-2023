// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>

#include <units/angular_acceleration.h>
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
  FollowTrajectory(Drivetrain *drive, frc::Pose2d endPose);

  FollowTrajectory(Drivetrain *drive, frc::Trajectory trajectory, frc::Rotation2d endAngle, bool invert = false);

  void Initialize() override;

  void Execute() override;

  bool IsFinished() override;
  
  private:
    Drivetrain *m_drive;
    frc::Pose2d m_startPose;
    frc::Pose2d m_endPose;
    frc::Pose2d m_relativePose;
    frc::Rotation2d m_startHeading, m_finishHeading;
    frc::Rotation2d m_endAngle;

    frc::Trajectory m_trajectory;

    frc::TrajectoryConfig m_config{ 2_mps, 2_mps_sq };

    units::second_t m_autoElapsed;

    bool hasTraj;
    bool m_invert = false;
    bool atTarget;
};
