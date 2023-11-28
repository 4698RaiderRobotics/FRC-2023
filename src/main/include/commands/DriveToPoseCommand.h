// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include <frc/geometry/Pose2d.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>

#include "subsystems/Drivetrain.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class DriveToPoseCommand
    : public frc2::CommandHelper<frc2::CommandBase, DriveToPoseCommand> {
 public:
  DriveToPoseCommand(  Drivetrain* );
  DriveToPoseCommand(  Drivetrain*, frc::Pose2d targetPose );

  void Initialize() override;

  void Execute() override;

  bool IsFinished() override;

private:
  units::second_t m_elapsed = 0_s;

  Drivetrain *m_drive;
  frc::Pose2d m_targetpose;

  frc::TrapezoidProfile<units::meters>::Constraints m_linearConstraints{ 3_mps, 1.5_mps_sq };
  frc::TrapezoidProfile<units::degrees>::Constraints m_omegaConstraints{ 360_deg_per_s, 90_deg_per_s_sq };
  
  frc::TrapezoidProfile<units::meters> m_xProfile{ m_linearConstraints, {0.0_m}, {0.0_m} };
  frc::TrapezoidProfile<units::meters> m_yProfile{ m_linearConstraints, {0.0_m}, {0.0_m} };
  frc::TrapezoidProfile<units::degrees> m_omegaProfile{ m_omegaConstraints, {0.0_deg}, {0.0_deg} };
  frc::TrapezoidProfile<units::meters>::State m_xSetpoint;
  frc::TrapezoidProfile<units::meters>::State m_ySetpoint;
  frc::TrapezoidProfile<units::degrees>::State m_omegaSetpoint;
};
